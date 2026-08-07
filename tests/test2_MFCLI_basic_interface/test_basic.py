"""Test 2: MechForge client basic interface.

Exercises the mechforge Python client against the real C++ solver:
- data model (Joint/Link/DrivingConstraint/Mech) + to_dict/from_dict roundtrip
- file loading with schema validation (getMechFromFile)
- mechanism management (add/remove/replace joints, links, drivings)
- end-to-end build + solve through the C++ subprocess

Runs from tests/test2_MFCLI_basic_interface/. On success it prints
"ALL TESTS PASSED" to stdout (compared against expected.txt); any failure
prints the traceback to stderr and exits non-zero (reported as RUN FAIL).
"""

import json
import sys
import tempfile
from pathlib import Path

# mechforge.py is copied into the CMake build output by configure_file,
# so SCHEMA_DIR and MECHFORGE_SOLVE resolve correctly from there.
BUILD_BIN = Path(__file__).resolve().parent.parent.parent / "build" / "bin"
sys.path.insert(0, str(BUILD_BIN))

# The module is found at runtime via sys.path above; the IDE can't see that,
# so silence its false "import could not be resolved" diagnostic.
import mechforge as mf  # noqa: E402  # type: ignore[import-not-found]

MECH_JSON = Path(__file__).resolve().parent / "mech.json"


def _check(name: str, fn) -> None:
    """Run a check; print progress to stderr so failures are diagnosable."""
    try:
        fn()
    except Exception as e:
        print(f"FAIL: {name}: {type(e).__name__}: {e}", file=sys.stderr)
        raise
    print(f"PASS: {name}", file=sys.stderr)


def test_model_roundtrip():
    data = json.loads(MECH_JSON.read_text())
    mech = mf.Mech.from_dict(data)

    assert len(mech.joints) == 3
    assert len(mech.links) == 1
    assert len(mech.drivingConstraints) == 1

    assert mech.joints[0].type == mf.JointType.Grounded
    assert mech.joints[2].type == mf.JointType.Prismatic
    assert mech.joints[2].prismaticAxis == (1.0, 0.0)
    assert mech.links[0].jointA == 1 and mech.links[0].jointB == 2
    assert mech.drivingConstraints[0].type == mf.DrivingType.Position
    assert mech.drivingConstraints[0].posX == "100*cos(10*t)"

    # ids are contiguous here -> to_dict must round-trip byte-identically
    assert mech.to_dict() == data
    assert mf.Mech.from_json(json.dumps(data)).to_dict() == data

    assert mech.ready is True


def test_joint_validation():
    # constructing a Grounded joint without groundPos must fail loudly
    try:
        mf.Joint(type=mf.JointType.Grounded, pos=(0.0, 0.0))
        raise AssertionError("should have raised")
    except mf.mf_InvalidActionError:
        pass


def test_get_mech_from_file_and_schema():
    cli = mf.MechForgeClient()
    cli.getMechFromFile(MECH_JSON)
    assert len(cli.mech.joints) == 3
    assert cli.mech.joints[0].id == 0

    # a file that violates the schema (Grounded without groundPos) must be rejected
    bad = {"joints": [{"id": 0, "type": "Grounded", "pos": [0, 0]}]}
    with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
        json.dump(bad, f)
        bad_path = f.name
    try:
        try:
            cli.getMechFromFile(bad_path)
            raise AssertionError("should have raised")
        except mf.mf_InvalidActionError:
            pass
    finally:
        Path(bad_path).unlink(missing_ok=True)


def test_mechanism_management():
    cli = mf.MechForgeClient()
    assert not cli.mech.ready

    j0 = mf.Joint(type=mf.JointType.Grounded, pos=(0.0, 0.0), groundPos=(0.0, 0.0))
    j1 = mf.Joint(type=mf.JointType.Free, pos=(10.0, 0.0))
    cli.add_joint(j0)
    cli.add_joint(j1)
    assert j0.id == 0 and j1.id == 1

    cli.add_link(mf.Link(jointA=0, jointB=1, length=10.0))
    assert cli.mech.links[0].id == 0

    cli.add_driving(
        mf.DrivingConstraint(
            type=mf.DrivingType.Position,
            jointA=1,
            jointB=0,
            posX="10",
            posY="0",
            velX="0",
            velY="0",
            accX="0",
            accY="0",
        )
    )
    assert cli.mech.drivingConstraints[0].id == 0

    cli.remove_driving(0)
    assert len(cli.mech.drivingConstraints) == 0
    cli.remove_link(0)
    assert len(cli.mech.links) == 0
    cli.remove_joint(0)
    assert len(cli.mech.joints) == 1
    assert cli.mech.joints[0].id == 1

    # removing a joint must cascade to links/drivings that reference it
    cli.clear_mech()
    j0 = mf.Joint(type=mf.JointType.Grounded, pos=(0.0, 0.0), groundPos=(0.0, 0.0))
    j1 = mf.Joint(type=mf.JointType.Free, pos=(10.0, 0.0))
    cli.add_joint(j0)
    cli.add_joint(j1)
    cli.add_link(mf.Link(jointA=0, jointB=1, length=10.0))
    cli.remove_joint(1)
    assert len(cli.mech.links) == 0  # link referencing removed joint is dropped


def test_build_solve():
    cli = mf.MechForgeClient()
    cli.getMechFromFile(MECH_JSON)

    steps = list(cli.solve(endTime=0.2, timeStep=0.05))
    assert len(steps) == 4
    assert all(len(s["q"]) == 6 for s in steps)  # 3 joints x 2 coordinates
    # times are accumulated in floating point, so check spacing, not exact values
    times = [s["time"] for s in steps]
    assert all(0.049 < b - a < 0.051 for a, b in zip(times, times[1:]))
    # joint 0 is grounded at the origin -> its position stays ~(0, 0)
    assert abs(steps[0]["q"][0]) < 1e-6 and abs(steps[0]["q"][1]) < 1e-6


def main():
    checks = [
        ("model roundtrip", test_model_roundtrip),
        ("joint validation", test_joint_validation),
        ("getMechFromFile + schema", test_get_mech_from_file_and_schema),
        ("mechanism management", test_mechanism_management),
        ("build + solve", test_build_solve),
    ]
    for name, fn in checks:
        _check(name, fn)
    print("ALL TESTS PASSED")


if __name__ == "__main__":
    main()
