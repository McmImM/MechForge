"""Test cases for the mf_commands command layer (test3).

Each case prints a small, deterministic transcript to stdout; run_tests.py
compares it against the matching expected_*.txt file. Run a single case with:

    python3 run_cases.py <case>
"""

import json
import sys
from pathlib import Path

# The mf_*.py modules are flattened into build/lib/ by CMake.
BUILD_LIB = Path(__file__).resolve().parent.parent.parent / "build" / "lib"
sys.path.insert(0, str(BUILD_LIB))

from mf_core import MechForgeCore  # noqa: E402  # type: ignore[import-not-found]
import mf_commands as mc  # noqa: E402  # type: ignore[import-not-found]

# --- parsing (no engine needed) ---------------------------------------------


def case_parse_joint():
    """Build joints of every type from their subcommand parser args."""
    for name in ("ground", "fixed", "revolute", "prismatic", "free"):
        parser = mc.JOINT_PARSERS[name]()
        if name == "ground":
            args = parser.parse_args(["1", "2"])
        elif name == "prismatic":
            args = parser.parse_args(["10", "0", "--slide-axis", "1", "0"])
        else:
            args = parser.parse_args(["10", "0"])
        j = mc.build_joint(mc.JOINT_TYPES[name], args)
        print(f"{name}: type={j.type.value} pos={j.pos} name={j.name}")
        if j.groundPos is not None:
            print(f"  ground={j.groundPos}")
        if j.prismaticAxis is not None:
            print(f"  slide_axis={j.prismaticAxis} slide_pos={j.prismaticPos}")
        if j.vel is not None:
            print(f"  vel={j.vel}")


def case_parse_driving():
    """Build drivings of every type from their subcommand parser args."""
    lines = {
        "position": ["1", "0", "--posX", "100*cos(10*t)", "--posY", "0"],
        "angle": ["1", "0", "--theta", "10*t", "--omega", "10"],
        "distance": ["1", "0", "--distance", "5", "--vel", "1"],
    }
    for name, argv in lines.items():
        parser = mc.DRIVING_PARSERS[name]()
        args = parser.parse_args(argv)
        d = mc.build_driving(mc.DRIVING_TYPES[name], args)
        print(
            f"{name}: type={d.type.value} jointA={d.jointA} "
            f"jointB={d.jointB} name={d.name}"
        )
        print(f"  to_dict={json.dumps(d.to_dict(), sort_keys=True)}")


# --- run_command against a real engine --------------------------------------


def case_crud():
    """Add / edit / remove joints, links and drivings through run_command."""
    c = MechForgeCore()
    for line in (
        "jointAdd ground 0 0",
        "jointAdd revolute 0 0",
        "jointAdd prismatic 10 0 --slide-axis 1 0",
        "linkAdd 1 2 10",
        "drivingAdd angle 1 0 --theta 100*cos(10*t)",
        "jointEdit 1 --name crank",
        "drivingEdit 0 --omega 10",
        "jointRemove 2",  # cascades: drops the link referencing joint 2
        "jointAdd free 5 5",
    ):
        r = mc.run_command(c, line)
        print(("OK " if r.ok else "ERR") + r.text)


def case_show():
    """show command: summary, component dump, list/json formats."""
    c = MechForgeCore()
    for line in (
        "jointAdd ground 0 0",
        "jointAdd revolute 0 0",
        "jointAdd prismatic 10 0 --slide-axis 1 0",
        "linkAdd 1 2 10",
    ):
        mc.run_command(c, line)
    for line in ("show", "show -a", "show -j -l", "show -a -f json"):
        r = mc.run_command(c, line)
        print(f"=== {line} ===")
        print(r.text)


def case_errors():
    """Error paths: unknown commands/types, missing args, bad references."""
    c = MechForgeCore()
    for line in (
        "jointRemove 5",  # nonexistent joint
        "jointAdd blah 1 2",  # unknown joint type
        "linkAdd 0 9 5",  # nonexistent joint in a link
        "jointAdd revolute",  # missing required args (argparse exit)
        "bogus cmd",  # unknown command
        "solve -e 1 -s 0.1",  # solve is streamed via run_solve
        "",  # empty line
    ):
        r = mc.run_command(c, line)
        print(("OK " if r.ok else "ERR") + " | " + (r.text or r.error or ""))


def case_solve():
    """run_solve streams one step per line."""
    c = MechForgeCore()
    for line in (
        "jointAdd ground 0 0",
        "jointAdd revolute 0 0",
        "jointAdd prismatic 10 0 --slide-axis 1 0",
        "linkAdd 1 2 10",
        "drivingAdd angle 1 0 --theta 100*cos(10*t)",
    ):
        mc.run_command(c, line)
    steps = list(mc.run_solve(c, "solve -e 0.2 -s 0.05"))
    print("n_steps:", len(steps))
    print("times:", json.dumps([s["time"] for s in steps]))
    print("keys:", sorted(steps[0].keys()))
    print("status:", steps[0]["status"])


CASES = {
    "parse_joint": case_parse_joint,
    "parse_driving": case_parse_driving,
    "crud": case_crud,
    "show": case_show,
    "errors": case_errors,
    "solve": case_solve,
}


def main():
    if len(sys.argv) != 2 or sys.argv[1] not in CASES:
        print("usage: run_cases.py <" + "|".join(CASES) + ">")
        return 2
    CASES[sys.argv[1]]()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
