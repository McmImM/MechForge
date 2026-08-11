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

# Graceful-shutdown timings (seconds).
SHUTDOWN_GRACE = 2.0  # safety net if a solve worker is slow to stop on shutdown
FINAL_JOIN = 1.0  # how long to wait after cancel/close before exiting


# --- publishing my location (so clients can find me) -------------------------


def write_server_info(port: int) -> None:
    """Atomically publish host/port/pid so clients can discover the daemon."""
    SERVER_INFO_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = SERVER_INFO_PATH.with_name(SERVER_INFO_PATH.name + ".tmp")
    tmp.write_text(json.dumps({"host": HOST, "port": port, "pid": os.getpid()}))
    os.replace(tmp, SERVER_INFO_PATH)  # atomic: readers never see a partial file


def _write_json(f, write_lock: threading.Lock, data: dict) -> None:
    """Write one JSON object as a line to the client.

    The per-connection write_lock serializes writes to this buffered socket:
    the solve worker (step lines) and the handle thread (control replies)
    would otherwise interleave bytes and corrupt the client's JSON stream.
    """
    with write_lock:
        f.write((json.dumps(data) + "\n").encode())
        f.flush()


def _solve_worker(
    core: MechForgeCore,
    text: str,
    f,
    lock: threading.Lock,
    write_lock: threading.Lock,
) -> None:
    """Stream a solve to the client; _shutdown/_solve_cancel stop it (and the engine)."""
    with lock:  # hold the core for the whole solve (blocks other clients)
        try:
            for step in mc.run_solve(core, text):
                if _shutdown.is_set() or _solve_cancel.is_set():
                    # Server shutting down or solve cancelled: stop the engine
                    # immediately (shutdown gives no grace period). Don't emit
                    # this step; the generator's next _recv() reads the
                    # engine's "cancelled" and raises mf_CancelledError, which
                    # drains the pipe correctly (no stray line left).
                    _solve_cancel.clear()
                    core.cancel()  # SIGINT -> engine returns "cancelled"
                    continue
                _write_json(f, write_lock, step)
            _write_json(f, write_lock, {"status": "done"})
        except mf_CancelledError:
            _write_json(f, write_lock, {"status": "cancelled"})
        except MechForgeError as e:
            _write_json(f, write_lock, {"status": "error", "message": str(e)})
        except OSError:
            # client disconnected mid-solve: stop the engine too, so its
            # "cancelled" line doesn't pollute the next solve on this core.
            core.cancel()


def handle(
    core: MechForgeCore,
    lock: threading.Lock,
    conn: socket.socket,
    active: set[socket.socket],
    active_lock: threading.Lock,
    threads: list[threading.Thread],
) -> None:
    """Serve one client connection: read command lines, reply one JSON line.

    Each connection has its own write_lock: only THIS connection's threads
    (the handle loop and its solve worker) write this socket, so a
    per-connection lock is enough -- cross-connection writes target different
    sockets and need no mutual exclusion.
    """
    f = conn.makefile("rwb")
    write_lock = threading.Lock()  # guards writes to this connection's socket
    try:
        while True:
            if _shutdown.is_set():
                break

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
                    _write_json(f, write_lock, {"status": "ready"})
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
                w = threading.Thread(
                    target=_solve_worker,
                    args=(core, text, f, lock, write_lock, _solve_cancel),
                    daemon=True,
                )
                threads.append(w)  # joinable at shutdown for a graceful drain
                w.start()
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
            _write_json(f, write_lock, reply)
    except OSError:
        # Connection closed underneath us (e.g. during shutdown) -- exit quietly.
        pass
    finally:
        with active_lock:
            active.discard(conn)
        conn.close()


def serve() -> None:
    # Created here (not at import time) so importing this module has no side effects.
    core = MechForgeCore()
    lock = threading.Lock()

    active: set[socket.socket] = set()  # live client sockets
    active_lock = threading.Lock()  # guards `active`
    threads: list[threading.Thread] = []  # handle + solve-worker threads

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
            with active_lock:
                active.add(conn)
            t = threading.Thread(
                target=handle,
                args=(core, lock, conn, active, active_lock, threads),
                daemon=True,
            )
            threads.append(t)
            t.start()
    finally:
        # 1) Wait for in-flight work to release the lock. Solve workers stop
        #    immediately on _shutdown (no grace period), so this usually
        #    completes at once; the timeout is a safety net for a worker that
        #    is slow to respond. If it times out, force-cancel the solve.
        got = lock.acquire(timeout=SHUTDOWN_GRACE)
        if not got:
            _solve_cancel.set()
            got = lock.acquire(timeout=FINAL_JOIN)
        if got:
            lock.release()

        # 2) No work is in flight now: close connections so any handle thread
        #    still blocked on readline notices EOF and exits.
        with active_lock:
            conns = list(active)
        for conn in conns:
            conn.close()

        # 3) Give threads a moment to finish (freed readlines, cancelled solves).
        for t in threads:
            t.join(timeout=FINAL_JOIN)

        # 4) Remove the published info so clients don't connect to a dead daemon.
        SERVER_INFO_PATH.unlink(missing_ok=True)
        srv.close()


if __name__ == "__main__":
    serve()
