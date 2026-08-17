"""Integration tests for the shared server chain: REPL -> mf_remote -> mf_server.

Each case runs as a separate process (python3 run_cases.py <case>). Every case
starts from a clean slate: it shuts down any pre-existing daemon, connects via
ensure_server() (which starts a fresh one), exercises the chain, then shuts the
daemon down again so no daemon is left behind.

Prints deterministic text to stdout, compared against expected_*.txt by
run_tests.py. Any assertion failure raises -> traceback to stderr -> RUN FAIL.
"""

import socket
import sys
import threading
import time
from pathlib import Path
import json

BUILD_LIB = Path(__file__).resolve().parent.parent.parent / "build" / "debug" / "lib"
sys.path.insert(0, str(BUILD_LIB))

from mf_remote import ensure_server, RemoteCore, ping  # noqa: E402
from mf_core import mf_CancelledError  # noqa: E402
from mf_server import SERVER_INFO_PATH  # noqa: E402
from mf_repl import MechForgeREPL  # noqa: E402

CLEAN_SLA = 0.2  # settle time after shutdown / before start


def _shutdown_daemon() -> None:
    """Test fixture: shut the daemon down via the current control protocol.

    Waits until the daemon has unlinked server_info.json -- its last shutdown
    step, right before the process exits. We watch the FILE, not the pid: a
    zombie keeps /proc/<pid> so os.kill(pid, 0) still succeeds and can't tell
    "exited" from "alive", which would make every teardown wait out the full
    deadline.
    """
    try:
        info = json.loads(SERVER_INFO_PATH.read_text())
        port = int(info["port"])
        s = socket.create_connection(("127.0.0.1", port), timeout=1)
        with s.makefile("rwb") as f:
            f.write(b'{"cmd":"shutdown"}\n')
            f.flush()
    except OSError, KeyError, ValueError:
        pass  # nothing live to stop

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        try:
            int(json.loads(SERVER_INFO_PATH.read_text())["port"])
        except OSError, KeyError, ValueError:
            return  # server_info gone: daemon finished cleaning up
        time.sleep(0.1)
    time.sleep(CLEAN_SLA)  # give up waiting; proceed anyway


def fresh_server() -> RemoteCore:
    """Kill any existing daemon, then connect to a freshly started one."""
    _shutdown_daemon()
    return ensure_server()


def finish() -> None:
    """Shut the daemon down (idempotent; leaves nothing behind)."""
    _shutdown_daemon()


def build_mech(rc: RemoteCore) -> None:
    """Build the standard crank-slider-ish mechanism via text commands."""
    for line in [
        "jointAdd revolute 0 0 --name j0",
        "jointAdd revolute 1 0 --name j1",
        "linkAdd 0 1 1 --name l0",
        "drivingAdd angle 0 1 --theta '10*t'",
    ]:
        r = rc.send_command(line)
        if not r.ok:
            raise AssertionError(f"{line!r} failed: {r.error}")


def count_components(rc: RemoteCore) -> tuple[int, int, int]:
    """Return (joints, links, drivings) as seen through show -a -f json."""
    r = rc.send_command("show -a -f json")
    if not r.ok:
        raise AssertionError(f"show failed: {r.error}")
    data = r.data or {}
    return (
        len(data.get("joints", [])),
        len(data.get("links", [])),
        len(data.get("drivings", [])),
    )


# --- cases ----------------------------------------------------------------


def case_server_basics() -> None:
    """Discovery/startup, command CRUD round-trip, unknown-command error."""
    rc = fresh_server()
    try:
        print(f"server_info exists: {SERVER_INFO_PATH.exists()}")
        build_mech(rc)
        print("build_mech: ok")
        j, l, d = count_components(rc)
        print(f"show: joints={j} links={l} drivings={d}")
        # error path: unknown command -> ok=False, error set, server survives
        r = rc.send_command("bogus")
        print(f"unknown cmd: ok={r.ok} err={r.error}")
        r = rc.send_command("jointAdd revolute 0")  # missing y positional
        print(f"bad args: ok={r.ok} err={r.error}")
    finally:
        rc.close()
        finish()


def case_shared_state() -> None:
    """Two clients see and mutate the SAME mech held by the daemon."""
    rc1 = fresh_server()
    try:
        r = rc1.send_command("jointAdd revolute 0 0 --name j0")
        print(f"client1 jointAdd: ok={r.ok}")
        rc2 = ensure_server()  # second connection, same daemon
        try:
            j, _, _ = count_components(rc2)
            print(f"client2 sees joints: {j}")
            r = rc2.send_command("jointAdd revolute 1 0 --name j1")
            print(f"client2 jointAdd: ok={r.ok}")
            j, _, _ = count_components(rc1)
            print(f"client1 sees joints after client2 add: {j}")
        finally:
            rc2.close()
    finally:
        rc1.close()
        finish()


def case_solve_stream() -> None:
    """Remote solve streams one step line per timestep."""
    rc = fresh_server()
    try:
        build_mech(rc)
        steps = list(rc.solve("solve -e 1 -s 0.25"))
        ts = [float(s["time"]) for s in steps]
        print(f"solve: steps={len(steps)} first_t={ts[0]:.2f} last_t={ts[-1]:.2f}")
        q0 = steps[0]["q"]
        print(f"solve: first step q len={len(q0)}")
    finally:
        rc.close()
        finish()


def case_solve_cancel() -> None:
    """A remote solve can be cancelled; the pipe stays clean afterwards."""
    rc = fresh_server()
    try:
        build_mech(rc)

        def canceller() -> None:
            time.sleep(0.3)
            rc.cancel()

        threading.Thread(target=canceller, daemon=True).start()
        try:
            for _ in rc.solve("solve -e 100000 -s 0.01"):
                pass
            print("solve: NOT cancelled (bug)")
        except mf_CancelledError:
            print("solve: cancelled")
        # the connection must still work (no stray step pollutes the pipe)
        j, _, _ = count_components(rc)
        print(f"post-cancel show: ok joints={j}")
    finally:
        rc.close()
        finish()


def case_shutdown() -> None:
    """Graceful shutdown removes server_info.json and leaves no daemon."""
    rc = fresh_server()
    print(f"server_info exists: {SERVER_INFO_PATH.exists()}")
    rc.close()
    _shutdown_daemon()
    print(f"server_info gone: {not SERVER_INFO_PATH.exists()}")
    # a fresh ensure_server must restart a clean daemon afterwards
    rc2 = ensure_server()
    try:
        j, _, _ = count_components(rc2)
        print(f"restart clean mech: joints={j}")
    finally:
        rc2.close()
        finish()


def case_repl_bridge() -> None:
    """MechForgeREPL (thin shell) drives RemoteCore -> server end to end."""
    rc = fresh_server()
    try:
        app = MechForgeREPL(rc)
        r = app._run("jointAdd revolute 0 0 --name j0")
        print(f"repl._run jointAdd: ok={r.ok} text={r.text or r.error}")
        r = app._run("jointAdd revolute 1 0 --name j1")
        print(f"repl._run jointAdd2: ok={r.ok}")
        r = app._run("linkAdd 0 1 1 --name l0")
        print(f"repl._run linkAdd: ok={r.ok}")
        r = app._run("drivingAdd angle 0 1 --theta '10*t'")
        print(f"repl._run drivingAdd: ok={r.ok}")
        r = app._run("show -a -f json")
        data = r.data or {}
        print(f"repl._run show: ok={r.ok} joints={len(data.get('joints', []))}")
        # streaming solve through the REPL's _solve (RemoteCore path)
        n = sum(1 for _ in app._solve("solve -e 1 -s 0.5"))
        print(f"repl._solve: steps={n}")
    finally:
        rc.close()
        finish()


def case_solve_shutdown() -> None:
    """Graceful shutdown while a solve is running.

    An infinite solve holds the core lock; the daemon must drain it (cancel it
    after the grace period), close connections, unlink server_info.json and
    exit. A fresh daemon then starts with an empty mech.
    """
    rc1 = fresh_server()
    try:
        build_mech(rc1)
        result: dict[str, str] = {}
        done = threading.Event()

        def run_solve() -> None:
            try:
                for _ in rc1.solve("solve -e 100000 -s 0.01"):
                    pass
                result["status"] = "finished"
            except mf_CancelledError:
                result["status"] = "cancelled"
            except Exception as e:  # noqa: BLE001 - also report transport errors
                result["status"] = type(e).__name__
            finally:
                done.set()

        threading.Thread(target=run_solve, daemon=True).start()
        time.sleep(0.3)  # let the solve start and grab the core lock
        _shutdown_daemon()  # request a graceful shutdown from "another client"
        done.wait(timeout=8)  # the infinite solve must be cancelled during drain
        print(f"solve ended: {result.get('status', '?')}")

        # the daemon's finally unlinks server_info after draining; wait for it
        deadline = time.monotonic() + 4
        while SERVER_INFO_PATH.exists() and time.monotonic() < deadline:
            time.sleep(0.1)
        print(f"server_info gone: {not SERVER_INFO_PATH.exists()}")

        rc3 = ensure_server()  # a fresh daemon must start with an empty mech
        try:
            j, _, _ = count_components(rc3)
            print(f"restart clean mech: joints={j}")
        finally:
            rc3.close()
    finally:
        rc1.close()
        finish()


CASES = {
    "server_basics": case_server_basics,
    "shared_state": case_shared_state,
    "solve_stream": case_solve_stream,
    "solve_cancel": case_solve_cancel,
    "shutdown": case_shutdown,
    "solve_shutdown": case_solve_shutdown,
    "repl_bridge": case_repl_bridge,
}

if __name__ == "__main__":
    name = sys.argv[1] if len(sys.argv) > 1 else ""
    fn = CASES.get(name)
    if fn is None:
        print(f"unknown case: {name}", file=sys.stderr)
        sys.exit(2)
    fn()
