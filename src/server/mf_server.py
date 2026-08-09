"""Shared MechForge daemon: owns the single MechForgeCore and serves clients over TCP.

Multi-client discovery: the daemon binds an OS-assigned port and publishes its
location to a well-known file (~/.mechforge/server_info.json). Clients read that
file, verify the daemon with a ping, and reuse it (see mf_remote.ensure_server).
"""

import json
import os
import socket
import sys
import threading
from pathlib import Path

from mf_core import MechForgeCore, MechForgeError, mf_CancelledError
import mf_commands as mc

HOST = "127.0.0.1"
SERVER_INFO_PATH = Path.home() / ".mechforge" / "server_info.json"

# Set when a client sends {"cmd":"shutdown"}; serve() polls it to stop accepting.
_shutdown = threading.Event()

# Set when any client sends {"cmd":"cancel"}; the active solve worker checks it.
_solve_cancel = threading.Event()


# --- publishing my location (so clients can find me) -------------------------


def write_server_info(port: int) -> None:
    """Atomically publish host/port/pid so clients can discover the daemon."""
    SERVER_INFO_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = SERVER_INFO_PATH.with_name(SERVER_INFO_PATH.name + ".tmp")
    tmp.write_text(json.dumps({"host": HOST, "port": port, "pid": os.getpid()}))
    os.replace(tmp, SERVER_INFO_PATH)  # atomic: readers never see a partial file


def _solve_worker(
    core: MechForgeCore,
    text: str,
    f,
    lock: threading.Lock,
    write_lock: threading.Lock,
    cancel_event: threading.Event,
) -> None:
    """Stream a solve to the client; cancel_event stops it (and the engine)."""
    with lock:  # hold the core for the whole solve (blocks other clients)
        try:
            for step in mc.run_solve(core, text):
                if cancel_event.is_set():
                    cancel_event.clear()
                    core.cancel()  # SIGINT -> engine returns "cancelled"
                    # Don't emit this step; the generator's next _recv() reads
                    # the engine's "cancelled" and raises mf_CancelledError,
                    # which drains the pipe correctly (no stray line left).
                    continue
                with write_lock:
                    f.write((json.dumps(step) + "\n").encode())
                    f.flush()
            with write_lock:
                f.write(b'{"status":"done"}\n')
                f.flush()
        except mf_CancelledError:
            with write_lock:
                f.write(b'{"status":"cancelled"}\n')
                f.flush()
        except MechForgeError as e:
            with write_lock:
                f.write(
                    (json.dumps({"status": "error", "message": str(e)}) + "\n").encode()
                )
                f.flush()
        except OSError:
            # client disconnected mid-solve: stop the engine too, so its
            # "cancelled" line doesn't pollute the next solve on this core.
            core.cancel()


def handle(
    core: MechForgeCore,
    lock: threading.Lock,
    write_lock: threading.Lock,
    conn: socket.socket,
) -> None:
    """Serve one client connection: read command lines, reply one JSON line."""
    f = conn.makefile("rwb")
    while True:
        line = f.readline()
        if not line:  # EOF: client disconnected
            break
        text = line.decode().strip()
        if not text:
            continue

        # Control commands (JSON); regular commands are plain text.
        if text.startswith("{") and text.endswith("}"):
            try:
                ctrl = json.loads(text)
            except ValueError:
                ctrl = None
            if ctrl is not None and ctrl.get("cmd") == "ping":
                with write_lock:
                    f.write(b'{"status":"ready"}\n')
                    f.flush()
                continue
            if ctrl is not None and ctrl.get("cmd") == "shutdown":
                _shutdown.set()
                break
            if ctrl is not None and ctrl.get("cmd") == "cancel":
                # No ack: an extra line would pollute the client's step stream.
                # The solve worker answers with the terminal "cancelled" instead.
                _solve_cancel.set()
                continue

        # solve is streaming: run it on a worker thread so THIS thread stays
        # free to read further commands (notably {"cmd":"cancel"}).
        if text.split(maxsplit=1)[0] == "solve":
            _solve_cancel.clear()  # drop any stale cancel from a finished solve
            threading.Thread(
                target=_solve_worker,
                args=(core, text, f, lock, write_lock, _solve_cancel),
                daemon=True,
            ).start()
            continue

        # Regular commands run on this thread, serialized by the lock. This
        # blocks while a solve holds the lock, so the protocol is: a connection
        # must not send regular commands mid-solve (cancel is always allowed;
        # a GUI should use a separate connection for commands).
        with lock:  # serialize access to the single, non-thread-safe core
            result = mc.run_command(core, text)
        reply = {
            "ok": result.ok,
            "text": result.text,
            "data": result.data,
            "error": result.error,
        }
        with write_lock:
            f.write((json.dumps(reply) + "\n").encode())
            f.flush()
    conn.close()


def serve() -> None:
    # Created here (not at import time) so importing this module has no side effects.
    core = MechForgeCore()
    lock = threading.Lock()
    write_lock = threading.Lock()  # serializes socket writes (solve steps vs replies)

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((HOST, 0))  # port 0 -> OS assigns a free one
    port = srv.getsockname()[1]  # ask the OS which port we got
    srv.listen(5)

    write_server_info(port)
    print(f"listening on {HOST}:{port}", file=sys.stderr)

    try:
        srv.settimeout(0.5)  # wake periodically so shutdown is noticed promptly
        while not _shutdown.is_set():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            threading.Thread(
                target=handle, args=(core, lock, write_lock, conn), daemon=True
            ).start()
    finally:
        # remove the published info so clients don't try to connect to a dead daemon
        SERVER_INFO_PATH.unlink(missing_ok=True)
        srv.close()


if __name__ == "__main__":
    serve()
