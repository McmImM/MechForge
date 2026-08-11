"""cmd2 interactive shell over the shared mf_commands command layer.

The REPL is a thin presentation layer: cmd2 provides input editing/history and
per-command completion/help (using the parser objects owned by mf_commands),
while every command is executed through the injected client -- by default a
RemoteCore connected to the shared server (ensure_server), or a local
MechForgeCore for offline/dev use. Either way all clients operate on the same
mech held by the server.
"""

import json
import signal
import sys

import cmd2

from mf_core import (
    MechForgeCore,
    MechForgeError,
    mf_CancelledError,
    mf_TransportError,
)
import mf_commands as mc
from mf_remote import ensure_server, RemoteCore

# Rename the help category used for cmd2's built-in commands
cmd2.Cmd.DEFAULT_CATEGORY = "MechForge Commands"


class MechForgeREPL(cmd2.Cmd):
    """Interactive shell for building and solving mechanisms."""

    def __init__(self, client: MechForgeCore | RemoteCore) -> None:
        super().__init__()

        # disable builtin cmds
        DISABLED_BUILTINS = [
            "edit",
            "run_script",
            "shell",
            "run_pyscript",
            "set",
            "shortcuts",
        ]
        for cmd in DISABLED_BUILTINS:
            self.disable_command(cmd, "Not Enabled.")

        # define prompt name
        self.prompt = "mf> "

        # preset command aliases
        self.aliases["h"] = "help"
        self.aliases["q"] = "quit"
        self.aliases["c"] = "clear"

        self.client = client
        if isinstance(client, RemoteCore) and hasattr(signal, "SIGUSR1"):
            signal.signal(signal.SIGUSR1, self._sigusr1)  # main thread
            client.monitor(self._on_server_gone)

    def _sigusr1(self, signum, frame):
        """Signal handler (runs in the main thread): interrupt whatever the
        REPL is blocked on (prompt_toolkit / readline) and exit.

        cmd2 does not catch SystemExit, so it propagates out of cmdloop().
        """
        self.poutput("\nserver has shut down; exiting")
        raise SystemExit(0)

    def _on_server_gone(self) -> None:
        """Called from the RemoteCore monitor thread when the server closed us."""
        signal.raise_signal(signal.SIGUSR1)

    # --- helpers ---------------------------------------------------------
    @property
    def raw_line(self) -> str:
        """The raw command line cmd2 is currently executing.

        cmd2 exposes the current Statement as ``self.current_command`` while a
        command runs (including inside subcommand handlers). ``run_command`` /
        ``run_solve`` re-parse the full text through the shared command layer,
        so every do_* method can just forward ``self.raw_line``.
        """
        stmt = self.current_command
        return stmt.raw if stmt is not None else ""

    def _run(self, line: str) -> mc.CommandResult:
        """Execute a command line against the client (remote daemon or local core)."""
        if isinstance(self.client, RemoteCore):
            try:
                return self.client.send_command(line)
            except mf_TransportError, OSError:
                # The shared daemon is gone (e.g. another client shut it down);
                # without it there is nothing left to do -- end the session.
                self.poutput("server has shut down; exiting")
                raise SystemExit(0)
        return mc.run_command(self.client, line)

    def _solve(self, line: str):
        """Stream solve steps from the client (remote daemon or local core)."""
        if isinstance(self.client, RemoteCore):
            try:
                yield from self.client.solve(line)
            except mf_TransportError, OSError:
                self.poutput("server has shut down; exiting")
                raise SystemExit(0)
        else:
            yield from mc.run_solve(self.client, line)

    def _emit(self, result: mc.CommandResult) -> None:
        """Print a CommandResult the way a human shell should."""
        if result.ok:
            self.poutput(result.text)
        else:
            self.perror(result.error or "command failed")

    def sigint_handler(self, signum, frame):
        """Ctrl-C during a remote solve: cancel on the server, don't raise.

        cmd2 registers this as the SIGINT handler and its default turns the
        signal into KeyboardInterrupt, which would just abort this client's
        read loop while the server keeps solving. For a RemoteCore we instead
        tell the server to cancel and skip the default (so no KeyboardInterrupt
        is raised); RemoteCore.solve() then sees the server's "cancelled" line
        and raises mf_CancelledError, which do_solve prints.
        """
        if isinstance(self.client, RemoteCore):
            cur = self.current_command
            if cur is not None and cur.command == "solve":
                self.client.cancel()
                return  # skip the default raise; the "cancelled" line follows
        super().sigint_handler(signum, frame)

    # --- clear screen (presentation-only; not part of mf_commands) -------
    def do_clear(self, args):
        """Clear the terminal screen"""
        # ANSI escape: clear screen + move cursor to home
        self.poutput("\033[2J\033[H")

    # --- shutdown (presentation; not part of mf_commands) ----------------
    def do_shutdown(self, args):
        """Shut down the shared server gracefully (drains in-flight work).

        The daemon stops accepting, lets in-flight commands/solves finish (or
        cancels an infinite solve), then exits. This REPL session also ends.
        """
        if isinstance(self.client, RemoteCore):
            self.client.shutdown()
            self.poutput("server shutting down")
        else:
            self.perror("no shared server to shut down (local core in use)")
        return True  # end the REPL session

    # --- load -------------------------------------------------------------
    @cmd2.with_argparser(mc.load_parser)
    def do_load(self, args):
        """Load a mechanism from a JSON file (validated against the schema)."""
        self._emit(self._run(self.raw_line))

    # --- show -------------------------------------------------------------
    @cmd2.with_argparser(mc.show_parser)
    def do_show(self, args):
        """Show the current mechanism."""
        self._emit(self._run(self.raw_line))

    # --- add / edit / remove joint ---------------------------------------
    jointAdd_parser = mc.jointAdd_parser

    @cmd2.with_argparser(jointAdd_parser)
    def do_jointAdd(self, args):
        """Add a joint to the mechanism (type-specific subcommand)."""
        args.cmd2_subcommand_func(args)

    @cmd2.as_subcommand_to(
        "jointAdd", "ground", mc.JOINT_PARSERS["ground"], help="grounded joint"
    )
    def _jointAdd_ground(self, args):
        self._emit(self._run(self.raw_line))

    @cmd2.as_subcommand_to(
        "jointAdd", "fixed", mc.JOINT_PARSERS["fixed"], help="fixed joint"
    )
    def _jointAdd_fixed(self, args):
        self._emit(self._run(self.raw_line))

    @cmd2.as_subcommand_to(
        "jointAdd",
        "revolute",
        mc.JOINT_PARSERS["revolute"],
        help="revolute joint",
    )
    def _jointAdd_revolute(self, args):
        self._emit(self._run(self.raw_line))

    @cmd2.as_subcommand_to(
        "jointAdd",
        "prismatic",
        mc.JOINT_PARSERS["prismatic"],
        help="prismatic joint",
    )
    def _jointAdd_prismatic(self, args):
        self._emit(self._run(self.raw_line))

    @cmd2.as_subcommand_to(
        "jointAdd", "free", mc.JOINT_PARSERS["free"], help="free joint"
    )
    def _jointAdd_free(self, args):
        self._emit(self._run(self.raw_line))

    @cmd2.with_argparser(mc.jointEdit_parser)
    def do_jointEdit(self, args):
        """Edit a joint; only the fields you specify are changed."""
        self._emit(self._run(self.raw_line))

    @cmd2.with_argparser(mc.jointRemove_parser)
    def do_jointRemove(self, args):
        """Remove a joint (and links/drivings that reference it)."""
        self._emit(self._run(self.raw_line))

    # --- add / edit / remove link ----------------------------------------
    @cmd2.with_argparser(mc.linkAdd_parser)
    def do_linkAdd(self, args):
        """Add a link between two joints."""
        self._emit(self._run(self.raw_line))

    @cmd2.with_argparser(mc.linkEdit_parser)
    def do_linkEdit(self, args):
        """Edit a link; only the fields you specify are changed."""
        self._emit(self._run(self.raw_line))

    @cmd2.with_argparser(mc.linkRemove_parser)
    def do_linkRemove(self, args):
        """Remove a link from the mechanism."""
        self._emit(self._run(self.raw_line))

    # --- add / edit / remove driving constraint ---------------------------
    drivingAdd_parser = mc.drivingAdd_parser

    @cmd2.with_argparser(drivingAdd_parser)
    def do_drivingAdd(self, args):
        """Add a driving constraint (type-specific subcommand)."""
        args.cmd2_subcommand_func(args)

    @cmd2.as_subcommand_to(
        "drivingAdd",
        "position",
        mc.DRIVING_PARSERS["position"],
        help="position driving",
    )
    def _drivingAdd_position(self, args):
        self._emit(self._run(self.raw_line))

    @cmd2.as_subcommand_to(
        "drivingAdd",
        "angle",
        mc.DRIVING_PARSERS["angle"],
        help="angle driving",
    )
    def _drivingAdd_angle(self, args):
        self._emit(self._run(self.raw_line))

    @cmd2.as_subcommand_to(
        "drivingAdd",
        "distance",
        mc.DRIVING_PARSERS["distance"],
        help="distance driving",
    )
    def _drivingAdd_distance(self, args):
        self._emit(self._run(self.raw_line))

    @cmd2.with_argparser(mc.drivingEdit_parser)
    def do_drivingEdit(self, args):
        """Edit a driving constraint; only the fields you specify are changed."""
        self._emit(self._run(self.raw_line))

    @cmd2.with_argparser(mc.drivingRemove_parser)
    def do_drivingRemove(self, args):
        """Remove a driving constraint from the mechanism."""
        self._emit(self._run(self.raw_line))

    # --- solve ------------------------------------------------------------
    @cmd2.with_argparser(mc.solve_parser)
    def do_solve(self, args):
        """Solve the mechanism, streaming each step as it is computed."""
        try:
            for step in self._solve(self.raw_line):
                self.poutput(json.dumps(step))
        except mf_CancelledError as e:
            self.poutput(f"solve cancelled: {e}")
        except MechForgeError as e:
            self.perror(f"solve failed: {e}")


def main() -> int:
    """Entry point used by the thin 'mf-repl' launcher in build/bin."""
    client = ensure_server()  # connect to (or start) the shared daemon
    app = MechForgeREPL(client)
    return app.cmdloop()


if __name__ == "__main__":
    sys.exit(main())
