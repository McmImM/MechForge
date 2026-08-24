"""TCP client + discovery for the shared MechForge daemon (mf_server).

Clients never own the state. They find the daemon via its published location
(~/.mechforge/server_info.json), verify it with a ping, and send text command
lines to get CommandResult replies -- so GUI / CLI / MCP all operate on the
single mech held by mf_server.
"""

import json
import queue
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Callable, Self

from mf_core import mf_TransportError, mf_EngineError, mf_CancelledError
import mf_commands as mc
from mf_server import HOST, SERVER_INFO_PATH

PING_TIMEOUT = 1.0  # seconds before a probe gives up
START_TIMEOUT = 3.0  # seconds to wait for a freshly started daemon
PING_INTERVAL = 0.1  # how often to re-probe while waiting


def read_server_info() -> dict:
    """Read the daemon's published location.

    Raises OSError (missing file) / KeyError (missing field) / ValueError
    (bad JSON or port) if the file is absent or stale.
    """
    return json.loads(SERVER_INFO_PATH.read_text())


def ping(port: int, host: str = HOST, timeout: float = PING_TIMEOUT) -> bool:
    """True if a daemon on host:port answers a ping with ready."""
    try:
        s = socket.create_connection((host, port), timeout=timeout)
        with s.makefile("rwb") as f:
            f.write(b'{"cmd":"ping"}\n')
            f.flush()
            line = f.readline()
        if not line:
            return False
        return json.loads(line.decode()).get("status") == "ready"
    except OSError, ValueError:
        return False


def start_server() -> subprocess.Popen:
    """Launch the daemon, detached from this process.

    Cross-platform: uses the running interpreter (sys.executable) and resolves
    mf_server.py relative to this file (works in src/ and the flat build/lib/).
    """
    script = Path(__file__).resolve().parent / "mf_server.py"
    return subprocess.Popen(
        [sys.executable, str(script)],
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def _find_live_port() -> int | None:
    """Port of a running, ping-verified daemon, or None (stale/missing file)."""
    try:
        info = read_server_info()
        port = int(info["port"])
    except OSError, KeyError, ValueError:
        return None
    return port if ping(port) else None


class RemoteCore:
    """Minimal TCP client for the shared daemon.

    send_command(line) -> CommandResult is the primitive everything else builds
    on (GUI buttons, the command log, MCP tools). The full MechForgeCore-style
    interface -- live mech mirror and streaming solve -- is wired up next once
    the server streams solve.
    """

    def __init__(self, port: int, host: str = HOST) -> None:
        self._host = host
        self._port = port
        # Blocking socket: streaming solve must not hit an arbitrary timeout.
        self._sock = socket.create_connection((host, port))
        self._closed = False
        self._server_gone = False
        self._on_gone: Callable[[], None] | None = None
        self._lines: queue.Queue[str | None] = queue.Queue()
        threading.Thread(target=self._read_loop, daemon=True).start()

    def send_command(self, line: str) -> mc.CommandResult:
        self._sock.sendall((line + "\n").encode())
        raw = self._get()
        resp = json.loads(raw)
        return mc.CommandResult(
            ok=bool(resp.get("ok")),
            text=resp.get("text") or "",
            data=resp.get("data"),
            error=resp.get("error"),
        )

    def solve(self, line: str):
        """Send a 'solve ...' command; yield each step until the terminal line.

        The server streams each step as a JSON line and finishes with a
        terminal status (done / cancelled / error). To stop early, call
        cancel(); the server then replies with the terminal "cancelled" line
        and this generator raises mf_CancelledError.
        """
        self._sock.sendall((line + "\n").encode())
        while True:
            raw = self._get()
            resp = json.loads(raw)
            status = resp.get("status")
            if status == "done":
                return
            if status == "cancelled":
                raise mf_CancelledError("solve cancelled")
            if status == "error":
                raise mf_EngineError(resp.get("message", "solve failed"))
            yield resp

    def cancel(self) -> None:
        """Ask the server to cancel the currently running solve.

        The server sets its cancel event (it sends no ack, so nothing pollutes
        the step stream); the solve worker then replies with the terminal
        {"status":"cancelled"} line, which solve() turns into
        mf_CancelledError.
        """
        self._sock.sendall(b'{"cmd":"cancel"}\n')

    def shutdown(self) -> None:
        """Ask the shared daemon to shut down gracefully, then close.

        The daemon drains in-flight work before exiting (see mf_server.serve);
        this client's connection is closed by the daemon during drain, and we
        also close it here. Any further use of this client will fail.
        """
        try:
            self._sock.sendall(b'{"cmd":"shutdown"}\n')
        finally:
            self.close()

    def _read_loop(self) -> None:
        """Sole socket reader: accumulate bytes, split on newlines, queue lines.

        Uses the raw socket (no makefile): a makefile's SocketIO shares one
        internal lock between readline and close, so a reader blocked in
        readline holds it forever and close() from another thread would hang.
        Because this thread is the ONLY reader there is no MSG_PEEK-vs-makefile
        race: it reliably sees the real EOF when the server closes the socket.
        """
        buf = b""
        while not self._closed:
            try:
                chunk = self._sock.recv(4096)
            except OSError:
                break
            if not chunk:  # EOF: server closed the connection
                break
            buf += chunk
            while b"\n" in buf:
                line, _, rest = buf.partition(b"\n")
                buf = rest
                self._lines.put(line.decode())
        self._lines.put(None)  # sentinel so blocked getters wake up
        if not self._closed:
            self._server_gone = True
            if self._on_gone is not None:
                self._on_gone()

    def _get(self) -> str:
        """Block for the next line from the reader thread."""
        # Block until a line is available from the reader thread
        line = self._lines.get()

        if line is None:
            raise mf_TransportError("server closed the connection")
        return line

    def monitor(self, on_gone) -> None:
        """Register a callback fired (background thread) when the server
        closes this connection (e.g. the daemon is shutting down gracefully).

        The reader thread (started in __init__) detects EOF and calls this.
        """
        self._on_gone = on_gone

    def close(self) -> None:
        self._closed = True
        self._sock.close()

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *_exc) -> None:
        self.close()


def ensure_server(timeout: float = START_TIMEOUT) -> RemoteCore:
    """Return a client connected to the shared daemon, starting one if needed."""
    port = _find_live_port()
    if port is not None:
        return RemoteCore(port, HOST)  # reuse the running daemon

    # none alive -> spawn one
    start_server()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        port = _find_live_port()
        if port is not None:
            return RemoteCore(port, HOST)  # fresh daemon published itself
        time.sleep(PING_INTERVAL)
    raise RuntimeError("MechForge server did not become ready")
