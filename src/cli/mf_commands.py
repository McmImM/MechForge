"""Headless command executor shared by REPL / server / MCP.

Single source of truth for command-line parsing: every argparse parser and the
model-building helpers live here, so the cmd2 REPL, the TCP server and the MCP
bridge all agree on the same command syntax.
"""

import json
from dataclasses import dataclass
import shlex
import argparse
from typing import Any, cast
from collections.abc import Callable, Iterator

import cmd2

from mf_core import *


@dataclass
class CommandResult:
    ok: bool
    text: str = ""
    data: dict | None = None
    error: str | None = None


# --- formatters --------------------------------------------------------------
# 'list' is the default format (see show_parser); FORMATTERS dict order also
# sets the -f choices, so the formatters must live here next to show_parser.


def fmt_num(v: float) -> str:
    """Render a coordinate/measurement consistently as a float."""
    f = float(v)
    return f"{f:.1f}" if f.is_integer() else f"{f:g}"


def joint_view(j: Joint) -> dict:
    return j.to_dict()  # index=None -> keeps the real user-facing id


def link_view(l: Link) -> dict:
    return l.to_dict()


def driving_view(drv: DrivingConstraint) -> dict:
    return {"id": drv.id, **drv.to_dict()}


def format_json(sections: dict[str, Any]) -> str:
    return json.dumps(sections, indent=2)


def format_list(sections: dict[str, Any]) -> str:
    """Human-friendly list view of the mechanism sections."""
    joints = cast(list[dict[str, Any]], sections.get("joints", []))
    links = cast(list[dict[str, Any]], sections.get("links", []))
    drivings = cast(list[dict[str, Any]], sections.get("drivings", []))

    has_joints = "joints" in sections
    has_links = "links" in sections
    has_drivings = "drivings" in sections

    lines = []
    if sections.get("name"):
        lines.append(f"name: {sections['name']}")
    if sections.get("description"):
        lines.append(f"description: {sections['description']}")

    # Each component group is rendered as its own block, separated by "---".
    groups = []

    if has_joints:
        g = []
        for j in joints:
            line = (
                f"Joint {j['id']} {j.get('name', '?')} {j['type']:<10} "
                f"pos=({fmt_num(j['pos'][0])}, {fmt_num(j['pos'][1])})"
            )
            if "groundPos" in j:
                gp = j["groundPos"]
                line += f"  ground=({fmt_num(gp[0])}, {fmt_num(gp[1])})"
            if "slide" in j:
                axis, pos = j["slide"]["axis"], j["slide"]["pos"]
                line += (
                    f"  slide axis=({fmt_num(axis[0])}, {fmt_num(axis[1])}) "
                    f"pos=({fmt_num(pos[0])}, {fmt_num(pos[1])})"
                )
            g.append(line)
        if g:
            groups.append("\n".join(g))

    if has_links:
        g = []
        for l in links:
            line = (
                f"Link {l['id']} {l.get('name', '?')} "
                f"jointA={l['jointA']} jointB={l['jointB']} "
                f"len={fmt_num(l['length'])}"
            )
            g.append(line)
        if g:
            groups.append("\n".join(g))

    if has_drivings:
        g = []
        for d in drivings:
            line = (
                f"Driving {d['id']} {d.get('name', '?')} {d['type']:<10} "
                f"jointA={d['jointA']} jointB={d['jointB']}"
            )
            g.append(line)
        if g:
            groups.append("\n".join(g))

    if groups:
        lines.append("---")
        lines.append("\n---\n".join(groups))

        # Footer summary counts only the sections that were actually shown.
        counts = []
        if has_joints:
            counts.append(f"{len(joints)} joint(s)")
        if has_links:
            counts.append(f"{len(links)} link(s)")
        if has_drivings:
            counts.append(f"{len(drivings)} driving(s)")
        lines.append("---")
        lines.append(", ".join(counts))

    return "\n".join(lines)


FORMATTERS = {
    "list": format_list,
    "json": format_json,
}


# --- joint type subcommands (jointAdd) --------------------------------------


def grounded_extra(parser: cmd2.Cmd2ArgumentParser) -> None:
    """Type-specific arguments for a Grounded joint (ground point)."""
    parser.add_argument("gx", type=float, help="ground x position")
    parser.add_argument("gy", type=float, help="ground y position")


def prismatic_extra(parser: cmd2.Cmd2ArgumentParser) -> None:
    """Type-specific options for a Prismatic joint."""
    parser.add_argument(
        "--slide-axis",
        nargs=2,
        type=float,
        metavar=("AX", "AY"),
        required=True,
        help="slide axis",
    )
    parser.add_argument(
        "--slide-origin",
        nargs=2,
        type=float,
        metavar=("SX", "SY"),
        default=None,
        help="slide origin (defaults to the joint position)",
    )


def joint_parser_factory(
    with_pos: bool = True,
    extra: Callable[[cmd2.Cmd2ArgumentParser], None] | None = None,
) -> Callable[[], cmd2.Cmd2ArgumentParser]:
    """Return a zero-arg factory for a joint TYPE subcommand parser.

    with_pos includes the x/y positionals (skipped for Grounded, whose
    position is the ground point); extra adds type-specific options.
    """

    def build() -> cmd2.Cmd2ArgumentParser:
        parser = cmd2.Cmd2ArgumentParser()
        if with_pos:
            parser.add_argument("x", type=float, help="initial x position")
            parser.add_argument("y", type=float, help="initial y position")
        parser.add_argument("--name", default=None, help="optional name")
        parser.add_argument(
            "--vel",
            nargs=2,
            type=float,
            metavar=("VX", "VY"),
            help="initial velocity",
        )
        parser.add_argument(
            "--acc",
            nargs=2,
            type=float,
            metavar=("AX", "AY"),
            help="initial acceleration",
        )
        if extra is not None:
            extra(parser)
        return parser

    return build


def build_joint(jtype: JointType, args: argparse.Namespace) -> Joint:
    """Build a Joint from parsed subcommand args; may raise MechForgeError."""
    if jtype == JointType.Grounded:
        # A grounded joint is bolted to the ground, so its position IS the
        # ground point (GX GY are positionals, not x/y).
        pos = ground = (args.gx, args.gy)
        prismatic_axis = prismatic_pos = None
    elif jtype == JointType.Prismatic:
        pos = (args.x, args.y)
        ground = None
        prismatic_axis = (args.slide_axis[0], args.slide_axis[1])
        prismatic_pos = (
            (args.slide_origin[0], args.slide_origin[1])
            if args.slide_origin is not None
            else pos
        )
    else:
        pos = (args.x, args.y)
        ground = prismatic_axis = prismatic_pos = None

    return Joint(
        type=jtype,
        pos=pos,
        vel=(args.vel[0], args.vel[1]) if args.vel else None,
        acc=(args.acc[0], args.acc[1]) if args.acc else None,
        name=args.name or Joint._DEFAULT_NAME,
        groundPos=ground,
        prismaticAxis=prismatic_axis,
        prismaticPos=prismatic_pos,
    )


# --- driving type subcommands (drivingAdd) -----------------------------------


def driving_joint_args(parser: cmd2.Cmd2ArgumentParser) -> None:
    """jointA/jointB positionals plus --name shared by every driving type."""
    parser.add_argument("jointA", type=int, help="driven joint id")
    parser.add_argument("jointB", type=int, help="reference joint id")
    parser.add_argument("--name", default=None, help="optional name")


def position_extra(parser: cmd2.Cmd2ArgumentParser) -> None:
    parser.add_argument("--posX", required=True, help="x position expression")
    parser.add_argument("--posY", required=True, help="y position expression")
    parser.add_argument("--velX", default="0", help="x velocity expression")
    parser.add_argument("--velY", default="0", help="y velocity expression")
    parser.add_argument("--accX", default="0", help="x acceleration expression")
    parser.add_argument("--accY", default="0", help="y acceleration expression")


def angle_extra(parser: cmd2.Cmd2ArgumentParser) -> None:
    parser.add_argument("--theta", required=True, help="angle expression")
    parser.add_argument("--omega", default="0", help="angular velocity expression")
    parser.add_argument("--alpha", default="0", help="angular acceleration expression")


def distance_extra(parser: cmd2.Cmd2ArgumentParser) -> None:
    parser.add_argument("--distance", required=True, help="distance expression")
    parser.add_argument("--vel", default="0", help="velocity expression")
    parser.add_argument("--acc", default="0", help="acceleration expression")


def driving_parser_factory(
    extra: Callable[[cmd2.Cmd2ArgumentParser], None],
) -> Callable[[], cmd2.Cmd2ArgumentParser]:
    """Return a zero-arg factory for a driving TYPE subcommand parser."""

    def build() -> cmd2.Cmd2ArgumentParser:
        parser = cmd2.Cmd2ArgumentParser()
        driving_joint_args(parser)
        extra(parser)
        return parser

    return build


def build_driving(dtype: DrivingType, args: argparse.Namespace) -> DrivingConstraint:
    """Build a DrivingConstraint from parsed subcommand args."""
    name = args.name or DrivingConstraint._DEFAULT_NAME
    if dtype == DrivingType.Position:
        return DrivingConstraint(
            type=dtype,
            jointA=args.jointA,
            jointB=args.jointB,
            name=name,
            posX=args.posX,
            posY=args.posY,
            velX=args.velX,
            velY=args.velY,
            accX=args.accX,
            accY=args.accY,
        )
    elif dtype == DrivingType.Angle:
        return DrivingConstraint(
            type=dtype,
            jointA=args.jointA,
            jointB=args.jointB,
            name=name,
            theta=args.theta,
            omega=args.omega,
            alpha=args.alpha,
        )
    else:  # DrivingType.Distance
        return DrivingConstraint(
            type=dtype,
            jointA=args.jointA,
            jointB=args.jointB,
            name=name,
            distance=args.distance,
            vel=args.vel,
            acc=args.acc,
        )


# --- top-level command parsers ----------------------------------------------
# jointAdd / drivingAdd keep their subparsers registry here so the cmd2 REPL
# can attach subcommands with cmd2.as_subcommand_to; run_command dispatches via
# the *_PARSERS tables below instead of argparse subparser dest semantics.


# load
load_parser = cmd2.Cmd2ArgumentParser()
load_parser.add_argument("file", help="path to a mechanism JSON file")

# show
show_parser = cmd2.Cmd2ArgumentParser()
show_parser.add_argument("-j", "--joints", action="store_true", help="show joints")
show_parser.add_argument("-l", "--links", action="store_true", help="show links")
show_parser.add_argument(
    "-d", "--driving", action="store_true", help="show driving constraints"
)
show_parser.add_argument(
    "-a", "--all", action="store_true", help="show all mechanism components"
)
show_parser.add_argument(
    "-f",
    "--format",
    choices=list(FORMATTERS),
    default=None,  # None = not specified; lets us detect an explicit -f
    help="output format: list (default) or json (only applies when dumping components)",
)

# jointAdd
jointAdd_parser = cmd2.Cmd2ArgumentParser(
    description="Add a joint to the mechanism (type-specific subcommand)."
)
jointAdd_parser.add_subparsers(title="joint type", metavar="TYPE", required=True)

# jointEdit
jointEdit_parser = cmd2.Cmd2ArgumentParser(
    description="Edit a joint; only the fields you specify are changed."
)
jointEdit_parser.add_argument("id", type=int, help="joint id to edit")
jointEdit_parser.add_argument(
    "--type",
    choices=[t.value for t in JointType],
    default=None,
    help="new joint type",
)
jointEdit_parser.add_argument(
    "--pos",
    nargs=2,
    type=float,
    metavar=("X", "Y"),
    default=None,
    help="new joint position",
)
jointEdit_parser.add_argument("--name", default=None, help="new name")
jointEdit_parser.add_argument(
    "--vel",
    nargs=2,
    type=float,
    metavar=("VX", "VY"),
    default=None,
    help="new velocity",
)
jointEdit_parser.add_argument(
    "--acc",
    nargs=2,
    type=float,
    metavar=("AX", "AY"),
    default=None,
    help="new acceleration",
)
jointEdit_parser.add_argument(
    "--ground",
    nargs=2,
    type=float,
    metavar=("GX", "GY"),
    default=None,
    help="new ground position (Grounded)",
)
jointEdit_parser.add_argument(
    "--slide-axis",
    nargs=2,
    type=float,
    metavar=("AX", "AY"),
    default=None,
    help="new slide axis (Prismatic)",
)
jointEdit_parser.add_argument(
    "--slide-origin",
    nargs=2,
    type=float,
    metavar=("SX", "SY"),
    default=None,
    help="new slide origin (Prismatic)",
)

# jointRemove
jointRemove_parser = cmd2.Cmd2ArgumentParser()
jointRemove_parser.add_argument("id", type=int, help="joint id to remove")

# linkAdd
linkAdd_parser = cmd2.Cmd2ArgumentParser(description="Add a link between two joints.")
linkAdd_parser.add_argument("jointA", type=int, help="id of the first joint")
linkAdd_parser.add_argument("jointB", type=int, help="id of the second joint")
linkAdd_parser.add_argument("length", type=float, help="link length")
linkAdd_parser.add_argument("--name", default=None, help="optional name")

# linkEdit
linkEdit_parser = cmd2.Cmd2ArgumentParser(
    description="Edit a link; only the fields you specify are changed."
)
linkEdit_parser.add_argument("id", type=int, help="link id to edit")
linkEdit_parser.add_argument(
    "--jointA", type=int, default=None, help="new first joint id"
)
linkEdit_parser.add_argument(
    "--jointB", type=int, default=None, help="new second joint id"
)
linkEdit_parser.add_argument("--length", type=float, default=None, help="new length")
linkEdit_parser.add_argument("--name", default=None, help="new name")

# linkRemove
linkRemove_parser = cmd2.Cmd2ArgumentParser()
linkRemove_parser.add_argument("id", type=int, help="link id to remove")

# drivingAdd
drivingAdd_parser = cmd2.Cmd2ArgumentParser(
    description="Add a driving constraint (type-specific subcommand)."
)
drivingAdd_parser.add_subparsers(title="driving type", metavar="TYPE", required=True)

# drivingEdit
drivingEdit_parser = cmd2.Cmd2ArgumentParser(
    description="Edit a driving constraint; only the fields you specify are changed."
)
drivingEdit_parser.add_argument("id", type=int, help="driving id to edit")
drivingEdit_parser.add_argument(
    "--jointA", type=int, default=None, help="new driven joint id"
)
drivingEdit_parser.add_argument(
    "--jointB", type=int, default=None, help="new reference joint id"
)
drivingEdit_parser.add_argument("--name", default=None, help="new name")
drivingEdit_parser.add_argument(
    "--posX", default=None, help="new x position expression"
)
drivingEdit_parser.add_argument(
    "--posY", default=None, help="new y position expression"
)
drivingEdit_parser.add_argument(
    "--velX", default=None, help="new x velocity expression"
)
drivingEdit_parser.add_argument(
    "--velY", default=None, help="new y velocity expression"
)
drivingEdit_parser.add_argument(
    "--accX", default=None, help="new x acceleration expression"
)
drivingEdit_parser.add_argument(
    "--accY", default=None, help="new y acceleration expression"
)
drivingEdit_parser.add_argument("--theta", default=None, help="new angle expression")
drivingEdit_parser.add_argument(
    "--omega", default=None, help="new angular velocity expression"
)
drivingEdit_parser.add_argument(
    "--alpha", default=None, help="new angular acceleration expression"
)
drivingEdit_parser.add_argument(
    "--distance", default=None, help="new distance expression"
)
drivingEdit_parser.add_argument(
    "--vel", default=None, help="new velocity expression (Distance)"
)
drivingEdit_parser.add_argument(
    "--acc", default=None, help="new acceleration expression (Distance)"
)

# drivingRemove
drivingRemove_parser = cmd2.Cmd2ArgumentParser()
drivingRemove_parser.add_argument("id", type=int, help="driving id to remove")

# solve
solve_parser = cmd2.Cmd2ArgumentParser()
solve_parser.add_argument(
    "-e",
    "--endTime",
    type=float,
    default=1.0,
    help="simulation end time (<= 0 runs until cancelled)",
)
solve_parser.add_argument(
    "-s", "--timeStep", type=float, default=0.01, help="time step"
)
solve_parser.add_argument(
    "-l",
    "--solveLevel",
    choices=[lv.value for lv in SolveLevel],
    default="pos",
    help="solve level",
)
solve_parser.add_argument("-n", "--maxIterations", type=int, default=100)
solve_parser.add_argument("-t", "--tolerance", type=float, default=1e-9)


# --- subcommand dispatch tables ---------------------------------------------
# Single source for "subcommand name -> enum -> parser factory"; used by both
# run_command (below) and the cmd2 REPL (via cmd2.as_subcommand_to).

JOINT_TYPES = {
    "ground": JointType.Grounded,
    "fixed": JointType.Fixed,
    "revolute": JointType.Revolute,
    "prismatic": JointType.Prismatic,
    "free": JointType.Free,
}

JOINT_PARSERS = {
    "ground": lambda: joint_parser_factory(with_pos=False, extra=grounded_extra)(),
    "fixed": lambda: joint_parser_factory()(),
    "revolute": lambda: joint_parser_factory()(),
    "prismatic": lambda: joint_parser_factory(extra=prismatic_extra)(),
    "free": lambda: joint_parser_factory()(),
}

DRIVING_TYPES = {
    "position": DrivingType.Position,
    "angle": DrivingType.Angle,
    "distance": DrivingType.Distance,
}

DRIVING_PARSERS = {
    "position": lambda: driving_parser_factory(position_extra)(),
    "angle": lambda: driving_parser_factory(angle_extra)(),
    "distance": lambda: driving_parser_factory(distance_extra)(),
}


# --- run_command -------------------------------------------------------------


def _parse(parser: cmd2.Cmd2ArgumentParser, argv: list[str]) -> argparse.Namespace:
    """Parse argv, turning argparse's SystemExit on bad input into an error.

    argparse calls ``sys.exit(2)`` on invalid input; in a headless/server
    context that would kill the process, so we surface it as a command error
    instead (the usage text is still printed to stderr by argparse).
    """
    try:
        return parser.parse_args(argv)
    except SystemExit:
        raise mf_InvalidActionError("invalid arguments") from None


def _cmd_load(client: MechForgeCore, argv: list[str]) -> CommandResult:
    args = _parse(load_parser, argv)
    client.getMechFromFile(args.file)
    m = client.mech
    return CommandResult(
        ok=True,
        text=(
            f"loaded {len(m.joints)} joint(s), {len(m.links)} link(s), "
            f"{len(m.drivingConstraints)} driving(s)"
        ),
    )


def _cmd_show(client: MechForgeCore, argv: list[str]) -> CommandResult:
    args = _parse(show_parser, argv)
    fmt = args.format if args.format is not None else "list"
    m = client.mech

    # No component flag (and no -a) -> plain-text summary.
    if not (args.all or args.joints or args.links or args.driving):
        summary = (
            f"name: {m.name}\n"
            f"description: {m.description}\n"
            f"---\n"
            f"{len(m.joints)} joint(s), {len(m.links)} link(s), "
            f"{len(m.drivingConstraints)} driving(s)"
        )
        if args.format is not None:
            summary = (
                f"note: --format has no effect on the summary; "
                f"use -a or -j/-l/-d to dump components as '{fmt}'\n\n"
            ) + summary
        return CommandResult(ok=True, text=summary)

    # Mechanism metadata always shown; components selected below.
    sections: dict[str, object] = {
        "name": m.name,
        "description": m.description,
    }
    if args.all:
        sections["joints"] = [joint_view(j) for j in m.joints]
        sections["links"] = [link_view(l) for l in m.links]
        sections["drivings"] = [driving_view(d) for d in m.drivingConstraints]
    else:
        if args.joints:
            sections["joints"] = [joint_view(j) for j in m.joints]
        if args.links:
            sections["links"] = [link_view(l) for l in m.links]
        if args.driving:
            sections["drivings"] = [driving_view(d) for d in m.drivingConstraints]

    formatter = FORMATTERS.get(fmt)
    if formatter is None:  # defensive; argparse choices already restrict this
        return CommandResult(ok=False, error=f"unsupported format: {fmt}")
    return CommandResult(ok=True, text=formatter(sections), data=sections)


def _cmd_joint_add(client: MechForgeCore, argv: list[str]) -> CommandResult:
    if not argv:
        return CommandResult(ok=False, error="jointAdd requires a TYPE")
    name, rest = argv[0], argv[1:]
    if name not in JOINT_TYPES:
        return CommandResult(ok=False, error=f"unknown joint type: {name}")
    args = _parse(JOINT_PARSERS[name](), rest)
    joint = build_joint(JOINT_TYPES[name], args)
    client.add_joint(joint)
    return CommandResult(
        ok=True,
        text=f"added Joint {joint.id} ({joint.type.value})",
        data={"id": joint.id, "type": joint.type.value},
    )


def _cmd_joint_edit(client: MechForgeCore, argv: list[str]) -> CommandResult:
    args = _parse(jointEdit_parser, argv)
    changes: dict[str, Any] = {}
    if args.type is not None:
        changes["type"] = JointType(args.type)
    if args.pos is not None:
        changes["pos"] = (args.pos[0], args.pos[1])
    if args.name is not None:
        changes["name"] = args.name
    if args.vel is not None:
        changes["vel"] = (args.vel[0], args.vel[1])
    if args.acc is not None:
        changes["acc"] = (args.acc[0], args.acc[1])
    if args.ground is not None:
        changes["groundPos"] = (args.ground[0], args.ground[1])
    if args.slide_axis is not None:
        changes["prismaticAxis"] = (args.slide_axis[0], args.slide_axis[1])
    if args.slide_origin is not None:
        changes["prismaticPos"] = (args.slide_origin[0], args.slide_origin[1])
    client.edit_joint(args.id, **changes)
    return CommandResult(ok=True, text=f"edited Joint {args.id}", data={"id": args.id})


def _cmd_joint_remove(client: MechForgeCore, argv: list[str]) -> CommandResult:
    args = _parse(jointRemove_parser, argv)
    client.remove_joint(args.id)
    return CommandResult(ok=True, text=f"removed Joint {args.id}", data={"id": args.id})


def _cmd_link_add(client: MechForgeCore, argv: list[str]) -> CommandResult:
    args = _parse(linkAdd_parser, argv)
    link = Link(
        jointA=args.jointA,
        jointB=args.jointB,
        length=args.length,
        name=args.name or Link._DEFAULT_NAME,
    )
    client.add_link(link)
    return CommandResult(
        ok=True,
        text=f"added Link {link.id} (jointA={link.jointA}, jointB={link.jointB})",
        data={"id": link.id, "jointA": link.jointA, "jointB": link.jointB},
    )


def _cmd_link_edit(client: MechForgeCore, argv: list[str]) -> CommandResult:
    args = _parse(linkEdit_parser, argv)
    changes: dict[str, Any] = {}
    if args.jointA is not None:
        changes["jointA"] = args.jointA
    if args.jointB is not None:
        changes["jointB"] = args.jointB
    if args.length is not None:
        changes["length"] = args.length
    if args.name is not None:
        changes["name"] = args.name
    client.edit_link(args.id, **changes)
    return CommandResult(ok=True, text=f"edited Link {args.id}", data={"id": args.id})


def _cmd_link_remove(client: MechForgeCore, argv: list[str]) -> CommandResult:
    args = _parse(linkRemove_parser, argv)
    client.remove_link(args.id)
    return CommandResult(ok=True, text=f"removed Link {args.id}", data={"id": args.id})


def _cmd_driving_add(client: MechForgeCore, argv: list[str]) -> CommandResult:
    if not argv:
        return CommandResult(ok=False, error="drivingAdd requires a TYPE")
    name, rest = argv[0], argv[1:]
    if name not in DRIVING_TYPES:
        return CommandResult(ok=False, error=f"unknown driving type: {name}")
    args = _parse(DRIVING_PARSERS[name](), rest)
    dc = build_driving(DRIVING_TYPES[name], args)
    client.add_driving(dc)
    return CommandResult(
        ok=True,
        text=f"added Driving {dc.id} ({dc.type.value})",
        data={"id": dc.id, "type": dc.type.value},
    )


def _cmd_driving_edit(client: MechForgeCore, argv: list[str]) -> CommandResult:
    args = _parse(drivingEdit_parser, argv)
    changes: dict[str, Any] = {}
    for field in (
        "jointA",
        "jointB",
        "name",
        "posX",
        "posY",
        "velX",
        "velY",
        "accX",
        "accY",
        "theta",
        "omega",
        "alpha",
        "distance",
        "vel",
        "acc",
    ):
        value = getattr(args, field)
        if value is not None:
            changes[field] = value
    client.edit_driving(args.id, **changes)
    return CommandResult(
        ok=True, text=f"edited Driving {args.id}", data={"id": args.id}
    )


def _cmd_driving_remove(client: MechForgeCore, argv: list[str]) -> CommandResult:
    args = _parse(drivingRemove_parser, argv)
    client.remove_driving(args.id)
    return CommandResult(
        ok=True, text=f"removed Driving {args.id}", data={"id": args.id}
    )


COMMANDS = {
    "load": _cmd_load,
    "show": _cmd_show,
    "jointAdd": _cmd_joint_add,
    "jointEdit": _cmd_joint_edit,
    "jointRemove": _cmd_joint_remove,
    "linkAdd": _cmd_link_add,
    "linkEdit": _cmd_link_edit,
    "linkRemove": _cmd_link_remove,
    "drivingAdd": _cmd_driving_add,
    "drivingEdit": _cmd_driving_edit,
    "drivingRemove": _cmd_driving_remove,
}


def run_command(client: MechForgeCore, line: str) -> CommandResult:
    """Parse a text command line and execute it against a client.

    ``client`` is any object implementing the MechForgeCore interface (a local
    ``MechForgeCore`` subprocess wrapper or a remote ``RemoteCore``), so the
    same command lines work for the REPL, the TCP server and the MCP bridge.
    """
    tokens = shlex.split(line)
    if not tokens:
        return CommandResult(ok=False, error="empty command")
    if tokens[0] == "solve":
        # solve streams one line per step; handled by run_solve(), not here.
        return CommandResult(
            ok=False, error="solve is streaming; use run_solve() instead"
        )
    handler = COMMANDS.get(tokens[0])
    if handler is None:
        return CommandResult(ok=False, error=f"unknown command: {tokens[0]}")
    try:
        return handler(client, tokens[1:])
    except MechForgeError as e:
        return CommandResult(ok=False, error=str(e))
    except Exception as e:
        return CommandResult(ok=False, error=f"{type(e).__name__}: {e}")


def run_solve(client: MechForgeCore, line: str) -> Iterator[dict]:
    """Execute a 'solve ...' command line, yielding each step dict as produced.

    Solve is streaming, so unlike run_command this is a generator: callers
    (REPL, server) consume the yielded steps and handle mf_CancelledError /
    MechForgeError raised during iteration themselves.
    """
    tokens = shlex.split(line)
    if not tokens or tokens[0] != "solve":
        raise mf_InvalidActionError("run_solve expects a 'solve ...' command line")
    args = _parse(solve_parser, tokens[1:])
    yield from client.solve(
        endTime=args.endTime,
        timeStep=args.timeStep,
        maxIterations=args.maxIterations,
        tolerance=args.tolerance,
        solveLevel=SolveLevel(args.solveLevel),
    )
