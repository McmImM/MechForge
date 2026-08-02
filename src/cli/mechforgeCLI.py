import json
import subprocess
from pathlib import Path
from dataclasses import dataclass, field
from enum import Enum
from typing import cast
import signal
import jsonschema
from referencing import Registry, Resource
from referencing.jsonschema import DRAFT202012

SCHEMA_DIR = Path(__file__).resolve().parent.parent / "schemas"
MECHFORGE_SOLVE = Path(__file__).resolve().parent / "mechforge_solve"


# --- Type definitions ---
class JointType(Enum):
    Grounded = "Grounded"
    Fixed = "Fixed"
    Revolute = "Revolute"
    Prismatic = "Prismatic"
    Free = "Free"


class DrivingType(Enum):
    Position = "Position"
    Angle = "Angle"
    Distance = "Distance"


class SolveLevel(Enum):
    Position = "pos"
    Velocity = "vel"
    Acceleration = "acc"


Vec2 = tuple[float, float]


# --- Exceptions ---
class MechForgeError(Exception):
    """Base class for all MechForge errors."""


class mf_TransportError(MechForgeError):
    """Raised when there is a transport error between the CLI and mechforge_solve."""


class mf_EngineError(MechForgeError):
    """Raised when mechforge_solve returns an error."""


class mf_UserError(MechForgeError):
    """Error raised due to user input or action."""


class mf_InvalidActionError(mf_UserError):
    """Raised when the user attempts an invalid action."""


class mf_CancelledError(mf_UserError):
    """Raised when a time-consuming operation is cancelled by the user."""


# --- Data classes ---
@dataclass
class Joint:
    id: int = field(init=False, default=-1)
    type: JointType
    pos: Vec2
    vel: Vec2 | None = None
    acc: Vec2 | None = None

    name: str | None = None

    # Grounded
    groundPos: Vec2 | None = None
    # Prismatic
    prismaticAxis: Vec2 | None = None
    prismaticPos: Vec2 | None = None

    _REQUIRED_FIELDS = {
        JointType.Grounded: ("groundPos",),
        JointType.Prismatic: ("prismaticAxis", "prismaticPos"),
    }

    def _validate(self):
        required_fields = self._REQUIRED_FIELDS.get(self.type, ())
        for field in required_fields:
            if getattr(self, field) is None:
                raise mf_InvalidActionError(
                    f"joint {self.id}: {self.type.value} requires {field}"
                )

    def __post_init__(self):
        self._validate()

    def to_json(self, index: int | None = None) -> dict:
        self._validate()
        data = {
            "id": self.id if index is None else index,
            "type": self.type.value,
            "pos": list(self.pos),
        }
        if self.vel is not None:
            data["vel"] = list(self.vel)
        if self.acc is not None:
            data["acc"] = list(self.acc)
        if self.type == JointType.Grounded:
            data["groundPos"] = list(cast(Vec2, self.groundPos))
            # has been validated to be not None; same below
        elif self.type == JointType.Prismatic:
            data["slide"] = {
                "axis": list(cast(Vec2, self.prismaticAxis)),
                "pos": list(cast(Vec2, self.prismaticPos)),
            }
        return data


@dataclass
class Link:
    id: int = field(init=False, default=-1)
    jointA: int
    jointB: int
    length: float
    name: str | None = None

    def to_json(self, index: int | None = None) -> dict:
        return {
            "id": self.id if index is None else index,
            "jointA": self.jointA,
            "jointB": self.jointB,
            "length": self.length,
        }


@dataclass
class DrivingConstraint:
    id: int = field(init=False, default=-1)
    type: DrivingType
    jointA: int
    jointB: int
    name: str | None = None

    # Position
    posX: str | None = None
    posY: str | None = None
    velX: str | None = None
    velY: str | None = None
    accX: str | None = None
    accY: str | None = None

    # Angle
    theta: str | None = None
    omega: str | None = None
    alpha: str | None = None

    # Distance
    distance: str | None = None
    vel: str | None = None
    acc: str | None = None

    def to_json(self) -> dict:
        data = {
            "type": self.type.value,
            "jointA": self.jointA,
            "jointB": self.jointB,
        }
        if self.type == DrivingType.Position:
            data.update(
                {
                    "posX": self.posX,
                    "posY": self.posY,
                    "velX": self.velX,
                    "velY": self.velY,
                    "accX": self.accX,
                    "accY": self.accY,
                }
            )
        elif self.type == DrivingType.Angle:
            data.update({"theta": self.theta, "omega": self.omega, "alpha": self.alpha})
        elif self.type == DrivingType.Distance:
            data.update({"distance": self.distance, "vel": self.vel, "acc": self.acc})
        return data


@dataclass
class Mech:
    joints: list[Joint]
    links: list[Link]
    drivingConstraints: list[DrivingConstraint]

    @property
    def ready(self) -> bool:
        return len(self.joints) > 0

    def to_json(self) -> dict:
        return {
            "joints": [j.to_json(i) for i, j in enumerate(self.joints)],
            "links": [l.to_json(i) for i, l in enumerate(self.links)],
            "drivings": [d.to_json() for d in self.drivingConstraints],
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Mech":
        """Build a Mech from a JSON-like dict (same shape as :meth:`to_json`)."""
        joints: list[Joint] = []
        for j in data.get("joints", []):
            jt = JointType(j["type"])
            joint = Joint(
                type=jt,
                pos=tuple(j.get("pos", (0.0, 0.0))),
                vel=tuple(j["vel"]) if "vel" in j else None,
                acc=tuple(j["acc"]) if "acc" in j else None,
                name=j.get("name"),
                groundPos=(
                    tuple(j["groundPos"])
                    if jt == JointType.Grounded and "groundPos" in j
                    else None
                ),
                prismaticAxis=(
                    tuple(j["slide"]["axis"])
                    if jt == JointType.Prismatic and "slide" in j
                    else None
                ),
                prismaticPos=(
                    tuple(j["slide"]["pos"])
                    if jt == JointType.Prismatic and "slide" in j
                    else None
                ),
            )
            joint.id = j["id"]
            joints.append(joint)

        links: list[Link] = []
        for l in data.get("links", []):
            link = Link(
                jointA=l["jointA"],
                jointB=l["jointB"],
                length=l["length"],
                name=l.get("name"),
            )
            link.id = l["id"]
            links.append(link)

        drivings: list[DrivingConstraint] = []
        for d in data.get("drivings", []):
            dt = DrivingType(d["type"])
            dc = DrivingConstraint(
                type=dt,
                jointA=d["jointA"],
                jointB=d["jointB"],
                name=d.get("name"),
            )
            if dt == DrivingType.Position:
                dc.posX = d.get("posX", "")
                dc.posY = d.get("posY", "")
                dc.velX = d.get("velX", "")
                dc.velY = d.get("velY", "")
                dc.accX = d.get("accX", "")
                dc.accY = d.get("accY", "")
            elif dt == DrivingType.Angle:
                dc.theta = d.get("theta", "")
                dc.omega = d.get("omega", "")
                dc.alpha = d.get("alpha", "")
            elif dt == DrivingType.Distance:
                dc.distance = d.get("distance", "")
                dc.vel = d.get("vel", "")
                dc.acc = d.get("acc", "")
            drivings.append(dc)

        return cls(joints=joints, links=links, drivingConstraints=drivings)

    @classmethod
    def from_json(cls, json_str: str) -> "Mech":
        """Build a Mech from a JSON string."""
        return cls.from_dict(json.loads(json_str))


# --- Schema validation ---

_schema_validator: jsonschema.Validator | None = None


def _load_schema_validator() -> jsonschema.Validator:
    """Build (and cache) a validator for the mechanism schema.

    Each schema's ``$id`` is rewritten to its local file URI so that relative
    ``$ref`` (e.g. ``defs.json#/$defs/Vec2``) resolve against local files
    instead of the ``raw.githubusercontent.com`` URLs embedded in the schema
    files (which would otherwise trigger a network fetch).
    """
    global _schema_validator
    if _schema_validator is not None:
        return _schema_validator

    def _load(path: Path) -> Resource:
        contents = json.loads(path.read_text())
        contents["$id"] = path.as_uri()
        return Resource.from_contents(contents, default_specification=DRAFT202012)

    resources = [(p.as_uri(), _load(p)) for p in SCHEMA_DIR.glob("*.json")]
    registry = Registry().with_resources(resources)

    schema_path = SCHEMA_DIR / "mech.json"
    schema = json.loads(schema_path.read_text())
    schema["$id"] = schema_path.as_uri()

    _schema_validator = jsonschema.Draft202012Validator(schema, registry=registry)
    return _schema_validator


def _validate_mech_dict(data: dict) -> None:
    """Validate a mechanism dict against ``mech.json``.

    Raises ``mf_InvalidActionError`` on the first schema violation.
    """
    validator = _load_schema_validator()
    error = next(validator.iter_errors(data), None)
    if error is not None:
        path = "/".join(str(p) for p in error.path) or "<root>"
        raise mf_InvalidActionError(
            f"Invalid mechanism at {path}: {error.message}"
        ) from error


# --- CLI ---
class MFCLI:
    def __init__(self, mech: Mech | None = None):
        self.mech: Mech
        if mech is None:
            self.mech = Mech(joints=[], links=[], drivingConstraints=[])
        else:
            self.mech = mech

        self._proc = subprocess.Popen(
            [str(MECHFORGE_SOLVE)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

    # communication with mechforge_solve

    def _send(self, data: dict):
        if self._proc.stdin is None:
            raise mf_TransportError("Subprocess broken: stdin is closed")
        json.dump(data, self._proc.stdin)
        self._proc.stdin.write("\n")
        self._proc.stdin.flush()

    def _recv(self) -> dict:
        if self._proc.stdout is None:
            raise mf_TransportError("Subprocess broken: stdout is closed")
        line = self._proc.stdout.readline()
        if not line:
            raise mf_TransportError("Subprocess broken: stdout is closed")
        return json.loads(line)

    def _send_mech(self):
        if not self.mech.ready:
            raise mf_InvalidActionError(
                "No mechanism existed. Please create a mechanism first."
            )
        self._send({"cmd": "build", "mech": self.mech.to_json()})
        resp = self._recv()
        if resp["status"] == "error":
            raise mf_EngineError("Build failed: " + resp["message"])
        return resp["message"]

    def cancel(self):
        """Cancel the current time-consuming operation."""
        if self._proc is not None and self._proc.poll() is None:
            self._proc.send_signal(signal.SIGINT)

    # Mechanism management

    def create_mech(self, mech: Mech | None = None) -> None:
        """You can simply use cli.mech = mech to set the mechanism as well."""
        self.mech = (
            mech
            if mech is not None
            else Mech(joints=[], links=[], drivingConstraints=[])
        )

    def clear_mech(self) -> None:
        self.mech = Mech(joints=[], links=[], drivingConstraints=[])

    def getMechFromFile(self, file_path: str | Path) -> None:
        """Load a mechanism definition from a JSON file, replacing the current one.

        The file is validated against ``mech.json`` before being loaded.
        """
        with open(file_path) as f:
            data = json.load(f)
        _validate_mech_dict(data)
        self.mech = Mech.from_dict(data)

    def add_joint(self, joint: Joint) -> None:
        """Add a joint to the mechanism."""
        joint.id = self.mech.joints[-1].id + 1 if self.mech.joints else 0
        self.mech.joints.append(joint)

    def remove_joint(self, joint_id: int):
        """Remove a joint from the mechanism."""
        idx = next(
            (i for i, j in enumerate(self.mech.joints) if j.id == joint_id), None
        )
        if idx is None:
            raise mf_InvalidActionError(f"Joint with id {joint_id} not found.")
        else:
            del self.mech.joints[idx]

        for i, link in enumerate(self.mech.links):
            if link.jointA == joint_id or link.jointB == joint_id:
                del self.mech.links[i]

        for i, constraint in enumerate(self.mech.drivingConstraints):
            if constraint.jointA == joint_id or constraint.jointB == joint_id:
                del self.mech.drivingConstraints[i]

    def replace_joint(self, joint_id: int, new_joint: Joint):
        """Replace a joint in the mechanism."""
        idx = next(
            (i for i, j in enumerate(self.mech.joints) if j.id == joint_id), None
        )
        if idx is None:
            raise mf_InvalidActionError(f"Joint with id {joint_id} not found.")
        else:
            new_joint.id = joint_id
            self.mech.joints[idx] = new_joint

    def add_link(self, link: Link):
        """Add a link to the mechanism."""
        link.id = self.mech.links[-1].id + 1 if self.mech.links else 0
        self.mech.links.append(link)

    def remove_link(self, link_id: int):
        """Remove a link from the mechanism."""
        idx = next((i for i, l in enumerate(self.mech.links) if l.id == link_id), None)
        if idx is None:
            raise mf_InvalidActionError(f"Link with id {link_id} not found.")
        else:
            del self.mech.links[idx]

    def add_driving(self, constraint: DrivingConstraint) -> None:
        """Add a driving constraint to the mechanism."""
        constraint.id = (
            self.mech.drivingConstraints[-1].id + 1
            if self.mech.drivingConstraints
            else 0
        )
        self.mech.drivingConstraints.append(constraint)

    def remove_driving(self, driving_id: int):
        """Remove a driving constraint from the mechanism."""
        idx = next(
            (
                i
                for i, d in enumerate(self.mech.drivingConstraints)
                if d.id == driving_id
            ),
            None,
        )
        if idx is None:
            raise mf_InvalidActionError(
                f"Driving constraint with id {driving_id} not found."
            )
        else:
            del self.mech.drivingConstraints[idx]

    # Solving

    def solve(
        self,
        endTime: float,
        timeStep: float,
        maxIterations: int = 100,
        tolerance: float = 1e-9,
        solveLevel: SolveLevel = SolveLevel.Position,
    ):
        """
        Solve the mechanism.
        ---
        @param endTime: The end time of the simulation. If <= 0, the simulation will run until the user cancels it.
        @param timeStep: The time step for the simulation.
        @param maxIterations: The maximum number of iterations for the solver.
        @param tolerance: The tolerance for the solver.
        @param solveLevel: The level of the solve (position, velocity, or acceleration).
        """
        if not self.mech.ready:
            raise mf_InvalidActionError(
                "No mechanism existed. Please create a mechanism first."
            )

        self._send_mech()
        self._send(
            {
                "cmd": "solve",
                "solverConfig": {
                    "endTime": endTime,
                    "timeStep": timeStep,
                    "maxIterations": maxIterations,
                    "tolerance": tolerance,
                    "solveLevel": solveLevel.value,
                },
            }
        )

        drained = False
        try:
            while True:
                resp = self._recv()
                if resp["status"] == "done":
                    drained = True
                    break
                elif resp["status"] == "cancelled":
                    drained = True
                    raise mf_CancelledError("Solve cancelled by user.")
                elif resp["status"] == "error":
                    drained = True
                    raise mf_EngineError("Solve failed: " + resp["message"])
                yield resp
        finally:
            if not drained:
                try:
                    while True:
                        r = self._recv()
                        if r.get("status") in ("done", "error", "cancelled"):
                            break
                except mf_TransportError:
                    # Engine process died (EOF) mid-solve. Do not raise from this
                    # cleanup path: it would mask an exception already in flight
                    # (or turn a plain early `break` into a spurious error). The
                    # dead process is guaranteed to surface as mf_TransportError
                    # on the next command anyway.
                    pass


if __name__ == "__main__":
    pass
