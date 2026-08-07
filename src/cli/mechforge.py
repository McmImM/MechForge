import json
import subprocess
from pathlib import Path
from dataclasses import dataclass, field
from enum import Enum
from typing import ClassVar, cast
import signal
import jsonschema
from referencing import Registry, Resource
from referencing.jsonschema import DRAFT202012

SCHEMA_DIR = Path(__file__).resolve().parent.parent / "schemas"
MECHFORGE_SOLVE = Path(__file__).resolve().parent / "mechforge_solve"

# Public API of the mechforge client module.
__all__ = [
    "MechForgeClient",
    "Joint",
    "Link",
    "DrivingConstraint",
    "Mech",
    "JointType",
    "DrivingType",
    "SolveLevel",
    "Vec2",
    "MechForgeError",
    "mf_TransportError",
    "mf_EngineError",
    "mf_UserError",
    "mf_InvalidActionError",
    "mf_CancelledError",
]


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
    _DEFAULT_NAME = "joint"

    id: int = field(init=False, default=-1)
    type: JointType
    pos: Vec2
    vel: Vec2 | None = None
    acc: Vec2 | None = None

    name: str = _DEFAULT_NAME

    # Grounded
    groundPos: Vec2 | None = None
    # Prismatic
    prismaticAxis: Vec2 | None = None
    prismaticPos: Vec2 | None = None

    # The exact set of type-specific fields a given joint type carries: each
    # listed field is required, and any other type-specific field is forbidden.
    # The common fields (pos/vel/acc/name) are always allowed. Mirrors the
    # mech.json schema (Revolute/Fixed/Free carry no extra fields).
    _TYPE_FIELDS = {
        JointType.Grounded: ("groundPos",),
        JointType.Prismatic: ("prismaticAxis", "prismaticPos"),
        JointType.Fixed: (),
        JointType.Revolute: (),
        JointType.Free: (),
    }

    # Every type-specific field across all joint types. Derived from
    # _TYPE_FIELDS so adding a new type-specific field automatically extends
    # both the "requires" and "does not allow" checks below.
    _ALL_TYPE_FIELDS: ClassVar[tuple[str, ...]] = tuple(
        sorted({f for fields in _TYPE_FIELDS.values() for f in fields})
    )

    def _validate(self):
        type_fields = self._TYPE_FIELDS.get(self.type, ())
        # Every type-specific field for this type is required...
        for field in type_fields:
            if getattr(self, field) is None:
                raise mf_InvalidActionError(f"{self.type.value} joint requires {field}")
        # ...and any other type-specific field is forbidden.
        for field in self._ALL_TYPE_FIELDS:
            if field not in type_fields and getattr(self, field) is not None:
                raise mf_InvalidActionError(
                    f"{self.type.value} joint does not allow {field}"
                )

    def __post_init__(self):
        if self.name is None:
            self.name = self._DEFAULT_NAME
        self._validate()

    def to_dict(self, index: int | None = None) -> dict:
        self._validate()
        data = {
            "id": self.id if index is None else index,
            "type": self.type.value,
            "name": self.name,
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
    _DEFAULT_NAME = "link"

    id: int = field(init=False, default=-1)
    jointA: int
    jointB: int
    length: float
    name: str = _DEFAULT_NAME

    def _validate(self):
        if self.length <= 0:
            raise mf_InvalidActionError("Link length must be positive")

    def __post_init__(self):
        if self.name is None:
            self.name = self._DEFAULT_NAME
        self._validate()

    def to_dict(self, index: int | None = None) -> dict:
        self._validate()
        data = {
            "id": self.id if index is None else index,
            "jointA": self.jointA,
            "jointB": self.jointB,
            "length": self.length,
            "name": self.name,
        }
        return data


@dataclass
class DrivingConstraint:
    _DEFAULT_NAME = "driving"

    id: int = field(init=False, default=-1)
    type: DrivingType
    jointA: int
    jointB: int
    name: str = _DEFAULT_NAME

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

    # The exact set of type-specific fields a given driving type carries: each
    # listed field is required, and any other type-specific field is forbidden.
    # Mirrors the mech.json schema.
    _TYPE_FIELDS = {
        DrivingType.Position: ("posX", "posY", "velX", "velY", "accX", "accY"),
        DrivingType.Angle: ("theta", "omega", "alpha"),
        DrivingType.Distance: ("distance", "vel", "acc"),
    }

    # Every type-specific field across all driving types (derived).
    _ALL_TYPE_FIELDS: ClassVar[tuple[str, ...]] = tuple(
        sorted({f for fields in _TYPE_FIELDS.values() for f in fields})
    )

    def _validate(self):
        type_fields = self._TYPE_FIELDS.get(self.type, ())
        for field in type_fields:
            if getattr(self, field) is None:
                raise mf_InvalidActionError(
                    f"{self.type.value} driving requires {field}"
                )
        for field in self._ALL_TYPE_FIELDS:
            if field not in type_fields and getattr(self, field) is not None:
                raise mf_InvalidActionError(
                    f"{self.type.value} driving does not allow {field}"
                )

    def __post_init__(self):
        if self.name is None:
            self.name = self._DEFAULT_NAME
        self._validate()

    def to_dict(self) -> dict:
        self._validate()
        data = {
            "type": self.type.value,
            "jointA": self.jointA,
            "jointB": self.jointB,
            "name": self.name,
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
    name: str = "Unnamed Mechanism"
    description: str = "No description provided."

    @property
    def ready(self) -> bool:
        return len(self.joints) > 0

    def to_dict(self) -> dict:
        """Serialize to the wire/project format.

        Joint ids are renumbered to contiguous C++ indices and links/drivings
        references are remapped to match, so the output stays self-consistent
        even after joints have been removed.
        """
        id_map = {j.id: i for i, j in enumerate(self.joints)}

        joints = [j.to_dict(id_map[j.id]) for j in self.joints]

        links = []
        for i, l in enumerate(self.links):
            d = l.to_dict(i)
            d["jointA"] = id_map[l.jointA]
            d["jointB"] = id_map[l.jointB]
            links.append(d)

        drivings = []
        for d in self.drivingConstraints:
            dd = d.to_dict()
            dd["jointA"] = id_map[d.jointA]
            dd["jointB"] = id_map[d.jointB]
            drivings.append(dd)

        return {
            "name": self.name,
            "description": self.description,
            "joints": joints,
            "links": links,
            "drivings": drivings,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Mech":
        """Build a Mech from a JSON-like dict (same shape as :meth:`to_dict`)."""
        name = "Unnamed Mechanism"
        description = "No description provided."

        if "name" in data:
            name = data["name"]

        if "description" in data:
            description = data["description"]

        joints: list[Joint] = []
        for j in data.get("joints", []):
            jt = JointType(j["type"])
            joint = Joint(
                type=jt,
                pos=tuple(j.get("pos", (0.0, 0.0))),
                vel=tuple(j["vel"]) if "vel" in j else None,
                acc=tuple(j["acc"]) if "acc" in j else None,
                name=j.get("name", Joint._DEFAULT_NAME),
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
                name=l.get("name", Link._DEFAULT_NAME),
            )
            link.id = l["id"]
            links.append(link)

        drivings: list[DrivingConstraint] = []
        for i, d in enumerate(data.get("drivings", [])):
            dt = DrivingType(d["type"])
            fields: dict[str, str] = {}
            if dt == DrivingType.Position:
                fields.update(
                    {
                        "posX": d.get("posX", ""),
                        "posY": d.get("posY", ""),
                        "velX": d.get("velX", ""),
                        "velY": d.get("velY", ""),
                        "accX": d.get("accX", ""),
                        "accY": d.get("accY", ""),
                    }
                )
            elif dt == DrivingType.Angle:
                fields.update(
                    {
                        "theta": d.get("theta", ""),
                        "omega": d.get("omega", ""),
                        "alpha": d.get("alpha", ""),
                    }
                )
            elif dt == DrivingType.Distance:
                fields.update(
                    {
                        "distance": d.get("distance", ""),
                        "vel": d.get("vel", ""),
                        "acc": d.get("acc", ""),
                    }
                )
            # Build fully-specified so __post_init__ validation can run.
            dc = DrivingConstraint(
                type=dt,
                jointA=d["jointA"],
                jointB=d["jointB"],
                name=d.get("name", DrivingConstraint._DEFAULT_NAME),
                **fields,
            )
            dc.id = i  # the file format carries no driving id; assign one
            drivings.append(dc)

        return cls(
            joints=joints,
            links=links,
            drivingConstraints=drivings,
            name=name,
            description=description,
        )

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


# --- Client ---
class MechForgeClient:
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
        self._send({"cmd": "build", "mech": self.mech.to_dict()})
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

    def edit_joint(
        self,
        joint_id: int,
        *,
        type: JointType | None = None,
        pos: Vec2 | None = None,
        vel: Vec2 | None = None,
        acc: Vec2 | None = None,
        name: str | None = None,
        groundPos: Vec2 | None = None,
        prismaticAxis: Vec2 | None = None,
        prismaticPos: Vec2 | None = None,
    ) -> None:
        """Partially edit an existing joint; only the given fields change.

        Switching type handles type-specific fields:
        - to Grounded: groundPos is required unless the joint is already Grounded
        - to Prismatic: prismaticAxis is required unless already Prismatic;
          prismaticPos defaults to the joint position when newly Prismatic
        - to Fixed/Revolute/Free: any stale type-specific fields are cleared
        """
        idx = next(
            (i for i, j in enumerate(self.mech.joints) if j.id == joint_id), None
        )
        if idx is None:
            raise mf_InvalidActionError(f"Joint with id {joint_id} not found.")

        old = self.mech.joints[idx]
        new_type = type if type is not None else old.type
        new_pos = pos if pos is not None else old.pos
        new_vel = vel if vel is not None else old.vel
        new_acc = acc if acc is not None else old.acc
        new_name = name if name is not None else old.name

        # Merge type-specific fields against what the new type allows. The
        # constructed Joint re-validates everything via Joint._validate(), the
        # single source of truth for "requires" / "does not allow" -- no rule
        # is duplicated here.
        allowed = Joint._TYPE_FIELDS.get(new_type, ())

        def carry(field, explicit, old_value):
            """Explicit value if given, else keep old if the new type allows
            that field; stale fields are cleared otherwise."""
            if explicit is not None:
                return explicit
            return old_value if field in allowed else None

        new_ground = carry("groundPos", groundPos, old.groundPos)
        new_axis = carry("prismaticAxis", prismaticAxis, old.prismaticAxis)
        new_ppos = carry("prismaticPos", prismaticPos, old.prismaticPos)
        # A joint newly switched to Prismatic starts its slide origin at the
        # joint position, so --slide-origin isn't required there.
        if (
            prismaticPos is None
            and old.type != JointType.Prismatic
            and "prismaticPos" in allowed
        ):
            new_ppos = new_pos

        new_joint = Joint(
            type=new_type,
            pos=new_pos,
            vel=new_vel,
            acc=new_acc,
            name=new_name,
            groundPos=new_ground,
            prismaticAxis=new_axis,
            prismaticPos=new_ppos,
        )
        new_joint.id = joint_id
        self.mech.joints[idx] = new_joint

    def _validate_link(self, jointA: int, jointB: int) -> None:
        """Validate that a link's joints exist and are distinct."""
        joint_ids = {j.id for j in self.mech.joints}
        if jointA not in joint_ids or jointB not in joint_ids:
            raise mf_InvalidActionError(
                f"Link references non-existent joint(s): {jointA}, {jointB}"
            )
        if jointA == jointB:
            raise mf_InvalidActionError("Link joints must be different")

    def add_link(self, link: Link):
        """Add a link to the mechanism."""
        self._validate_link(link.jointA, link.jointB)
        link.id = self.mech.links[-1].id + 1 if self.mech.links else 0
        self.mech.links.append(link)

    def edit_link(
        self,
        link_id: int,
        *,
        jointA: int | None = None,
        jointB: int | None = None,
        length: float | None = None,
        name: str | None = None,
    ) -> None:
        """Partially edit an existing link; only the given fields change."""
        idx = next((i for i, l in enumerate(self.mech.links) if l.id == link_id), None)
        if idx is None:
            raise mf_InvalidActionError(f"Link with id {link_id} not found.")

        old = self.mech.links[idx]
        new_jointA = jointA if jointA is not None else old.jointA
        new_jointB = jointB if jointB is not None else old.jointB
        new_length = length if length is not None else old.length
        new_name = name if name is not None else old.name

        self._validate_link(new_jointA, new_jointB)

        new_link = Link(
            jointA=new_jointA,
            jointB=new_jointB,
            length=new_length,
            name=new_name,
        )
        new_link.id = link_id
        self.mech.links[idx] = new_link

    def remove_link(self, link_id: int):
        """Remove a link from the mechanism."""
        idx = next((i for i, l in enumerate(self.mech.links) if l.id == link_id), None)
        if idx is None:
            raise mf_InvalidActionError(f"Link with id {link_id} not found.")
        else:
            del self.mech.links[idx]

    def _validate_driving(self, jointA: int, jointB: int) -> None:
        """Validate that a driving's joints exist and are distinct."""
        joint_ids = {j.id for j in self.mech.joints}
        if jointA not in joint_ids or jointB not in joint_ids:
            raise mf_InvalidActionError(
                f"Driving references non-existent joint(s): {jointA}, {jointB}"
            )
        if jointA == jointB:
            raise mf_InvalidActionError("Driving joints must be different")

    def add_driving(self, constraint: DrivingConstraint) -> None:
        """Add a driving constraint to the mechanism."""
        self._validate_driving(constraint.jointA, constraint.jointB)
        constraint.id = (
            self.mech.drivingConstraints[-1].id + 1
            if self.mech.drivingConstraints
            else 0
        )
        self.mech.drivingConstraints.append(constraint)

    def edit_driving(
        self,
        driving_id: int,
        *,
        jointA: int | None = None,
        jointB: int | None = None,
        name: str | None = None,
        posX: str | None = None,
        posY: str | None = None,
        velX: str | None = None,
        velY: str | None = None,
        accX: str | None = None,
        accY: str | None = None,
        theta: str | None = None,
        omega: str | None = None,
        alpha: str | None = None,
        distance: str | None = None,
        vel: str | None = None,
        acc: str | None = None,
    ) -> None:
        """Partially edit an existing driving; only the given fields change.

        Type-specific expression fields are merged and validated by
        ``DrivingConstraint._validate`` (the single source of truth); fields the
        driving's type does not allow are rejected there.
        """
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

        old = self.mech.drivingConstraints[idx]
        new_jointA = jointA if jointA is not None else old.jointA
        new_jointB = jointB if jointB is not None else old.jointB
        new_name = name if name is not None else old.name

        self._validate_driving(new_jointA, new_jointB)

        overrides = {
            "posX": posX,
            "posY": posY,
            "velX": velX,
            "velY": velY,
            "accX": accX,
            "accY": accY,
            "theta": theta,
            "omega": omega,
            "alpha": alpha,
            "distance": distance,
            "vel": vel,
            "acc": acc,
        }
        merged = {
            f: (v if v is not None else getattr(old, f)) for f, v in overrides.items()
        }
        new_constraint = DrivingConstraint(
            type=old.type,
            jointA=new_jointA,
            jointB=new_jointB,
            name=new_name,
            **merged,
        )
        new_constraint.id = driving_id
        self.mech.drivingConstraints[idx] = new_constraint

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

    def show(self) -> dict:
        """Return the full mechanism as a JSON-serializable dict.

        This is the wire/project format: joints are renumbered to contiguous
        indices, references are remapped, and component names are included.
        """
        return self.mech.to_dict()

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
