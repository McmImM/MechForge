"""Headless command executor shared by REPL / server / MCP.

Single source of truth for command-line parsing: every argparse parser and the
model-building helpers live here, so the cmd2 REPL, the TCP server and the MCP
bridge all agree on the same command syntax.
"""

import json
from dataclasses import dataclass
import shlex
import argparse
from enum import Enum
from typing import Any, Sequence, cast
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

# Human-readable descriptions for the joint TYPE picker. Single source of
# truth for what the GUI shows next to each subcommand name.
JOINT_TYPE_HELP = {
    "ground": "grounded joint (bolted to the ground)",
    "fixed": "fixed joint",
    "revolute": "revolute joint",
    "prismatic": "prismatic joint (slides along an axis)",
    "free": "free joint",
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

# Human-readable descriptions for the driving TYPE picker.
DRIVING_TYPE_HELP = {
    "position": "drive a joint's position with expressions",
    "angle": "drive a joint's angle with expressions",
    "distance": "drive the distance between two joints",
}

DRIVING_PARSERS = {
    "position": lambda: driving_parser_factory(position_extra)(),
    "angle": lambda: driving_parser_factory(angle_extra)(),
    "distance": lambda: driving_parser_factory(distance_extra)(),
}


# Subcommand registries, keyed by the top-level parser object. A command with
# subcommands only needs: a registry dict + a help map + one call here. This
# module-level table avoids attaching custom attributes to argparse's private
# _SubParsersAction (which Pyright rejects and is fragile across versions).
SUBPARSER_REGISTRIES: dict[
    cmd2.Cmd2ArgumentParser,
    tuple[dict[str, Callable[[], cmd2.Cmd2ArgumentParser]], dict[str, str]],
] = {}


def _register_subcommands(
    parser: cmd2.Cmd2ArgumentParser,
    registry: dict[str, Callable[[], cmd2.Cmd2ArgumentParser]],
    help_map: dict[str, str],
) -> None:
    """Register a subcommand registry + help map for a top-level parser.

    Stored in the module-level SUBPARSER_REGISTRIES table (keyed by parser),
    so _resolve_parser can discover subcommands generically without touching
    argparse's private _SubParsersAction internals.
    """
    SUBPARSER_REGISTRIES[parser] = (registry, help_map)


# Attach the subcommand registries. A command with subcommands only needs: a
# registry dict + a help map + one call here.
_register_subcommands(jointAdd_parser, JOINT_PARSERS, JOINT_TYPE_HELP)
_register_subcommands(drivingAdd_parser, DRIVING_PARSERS, DRIVING_TYPE_HELP)


# Command name -> top-level parser. Replaces globals()[f"{cmd}_parser"] so
# adding a command only means adding one entry here (no name-mangling magic).
PARSERS = {
    "load": load_parser,
    "show": show_parser,
    "jointAdd": jointAdd_parser,
    "jointEdit": jointEdit_parser,
    "jointRemove": jointRemove_parser,
    "linkAdd": linkAdd_parser,
    "linkEdit": linkEdit_parser,
    "linkRemove": linkRemove_parser,
    "drivingAdd": drivingAdd_parser,
    "drivingEdit": drivingEdit_parser,
    "drivingRemove": drivingRemove_parser,
    "solve": solve_parser,
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


NORMAL_COMMANDS = {
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

STREAMING_COMMANDS = {
    "solve": None,  # handled by run_solve() instead of run_command()
}

COMMANDS = {**NORMAL_COMMANDS, **STREAMING_COMMANDS}


def run_command(client: MechForgeCore, line: str) -> CommandResult:
    """Parse a text command line and execute it against a client.

    ``client`` is any object implementing the MechForgeCore interface (a local
    ``MechForgeCore`` subprocess wrapper or a remote ``RemoteCore``), so the
    same command lines work for the REPL, the TCP server and the MCP bridge.
    """
    tokens = shlex.split(line)
    if not tokens:
        return CommandResult(ok=False, error="empty command")
    if tokens[0] in STREAMING_COMMANDS.keys():
        # solve streams one line per step; handled by run_solve(), not here.
        return CommandResult(
            ok=False, error="solve is streaming; use run_solve() instead"
        )
    handler = NORMAL_COMMANDS.get(tokens[0])
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


# --- prompt sequences (AutoCAD-style command line) --------------------------
# build_prompts(tokens) returns the REMAINING prompts for a command given the
# tokens confirmed so far. The GUI command line calls it after every
# confirmation; when it returns [] the command is complete and can be executed.
# Subcommand resolution (jointAdd/drivingAdd TYPE) happens here, so the GUI
# never needs to know about JOINT_PARSERS / DRIVING_PARSERS.


class PromptKind(Enum):
    """What a prompt expects from the user.

    - REQUIRED: the user must type a value (normal color)
    - OPTIONAL: the user may type a value or skip (GUI renders dimmer)
    - CHOICE:   the user picks one of the listed options (cannot skip, but
                it is not free text either)
    """

    REQUIRED = "required"
    OPTIONAL = "optional"
    CHOICE = "choice"


def _resolve_parser(
    tokens: list[str],
) -> tuple[cmd2.Cmd2ArgumentParser, list[tuple[str, str]], bool]:
    """(parser, type_choices, subcommand_confirmed) for a command.

    Generic: looks up the top-level parser in PARSERS, then scans its actions
    for a _SubParsersAction. If one exists and the TYPE token is a registered
    subcommand, returns the sub-parser (subcommand_confirmed=True); otherwise
    returns the top-level parser with the (name, desc) choice list from the
    registry's help map. Adding a new subcommand command needs no change here
    -- only a registry + help map + _register_subcommands call.
    """
    cmd = tokens[0]
    parser = PARSERS[cmd]
    # Generic subcommand discovery: look up the parser in the module-level
    # registry table (no argparse private-attribute poking).
    entry = SUBPARSER_REGISTRIES.get(parser, None)
    if entry is not None:
        registry, help_map = entry
        if len(tokens) >= 2 and tokens[1] in registry:
            return registry[tokens[1]](), [], True
        return parser, list(help_map.items()), False
    return parser, [], False


def _display_option(option_strings: Sequence[str]) -> str:
    """Prefer the long flag (--joints) over the short one (-j)."""
    for opt in option_strings:
        if opt.startswith("--"):
            return opt
    return option_strings[0]


def _prompts_from_parser(
    parser: cmd2.Cmd2ArgumentParser,
    type_choices: list[tuple[str, str]],
    consumed: int = 0,
    supplied_flags: set[str] | None = None,
) -> list[tuple[str, str, PromptKind]]:
    """Introspect an argparse parser into (label, help, kind) prompt triples.

    kind is a PromptKind:
      - REQUIRED -> the user must type a value
      - OPTIONAL -> the user may type a value or skip (GUI renders dimmer)
      - CHOICE   -> the user picks one of the listed options (cannot skip)

    consumed is how many positional values the confirmed tokens already
    supply; those positionals are skipped (their prompts are not returned).
    supplied_flags is the set of option strings already present in the
    confirmed tokens; those optionals are skipped too.

    - subparsers (jointAdd/drivingAdd TYPE) -> one ("name", desc, CHOICE)
      triple per option, so the GUI can render a picker directly. The option
      list comes from the caller (JOINT_TYPE_HELP / DRIVING_TYPE_HELP), NOT
      from action.choices: cmd2 registers subcommands via as_subcommand_to,
      which never fills the _SubParsersAction.choices dict.
    - optionals -> their first option string (e.g. "--posX", "--name"),
      kind = REQUIRED if action.required else OPTIONAL.
    - positionals -> metavar or dest uppercased (e.g. "X", "JOINTA"), REQUIRED.
    -h/--help is filtered out: it is argparse's built-in, not a real prompt.
    """
    prompts: list[tuple[str, str, PromptKind]] = []
    positional_seen = 0
    for action in parser._actions:
        if isinstance(action, argparse._HelpAction):
            continue  # argparse's built-in -h/--help, not a real prompt
        if type_choices and isinstance(action, argparse._SubParsersAction):
            for name, desc in type_choices:
                prompts.append((name, desc, PromptKind.CHOICE))
        elif action.option_strings:
            if supplied_flags and any(
                opt in supplied_flags for opt in action.option_strings
            ):
                continue  # already supplied by a confirmed token
            kind = PromptKind.REQUIRED if action.required else PromptKind.OPTIONAL
            prompts.append(
                (_display_option(action.option_strings), action.help or "", kind)
            )
        else:
            # metavar may be a tuple (e.g. ("X", "Y") for nargs=2); flatten it.
            if positional_seen < consumed:
                positional_seen += 1
                continue  # already supplied by a confirmed token
            metavar = action.metavar
            if isinstance(metavar, tuple):
                label = " ".join(metavar)
            else:
                label = metavar or action.dest.upper()
            prompts.append((label, action.help or "", PromptKind.REQUIRED))
    return prompts


def _count_consumed_positionals(tokens: list[str], subcommand_confirmed: bool) -> int:
    """How many positional values the confirmed tokens already supply.

    tokens[0] is the command name; tokens[1] is the subcommand name when
    subcommand_confirmed. The remaining tokens are walked in order: a plain
    value consumes one positional, a --flag consumes itself plus its value
    (so neither counts as a positional).
    """
    rest = tokens[2:] if subcommand_confirmed else tokens[1:]
    parser, _, _ = _resolve_parser(tokens)
    flag_values: dict[str, int] = {}
    for action in parser._actions:
        if action.option_strings:
            n = action.nargs if isinstance(action.nargs, int) else 1
            for opt in action.option_strings:
                flag_values[opt] = n
    consumed = 0
    i = 0
    while i < len(rest):
        if rest[i].startswith("-"):
            i += 1 + flag_values.get(rest[i], 1)
        else:
            consumed += 1
            i += 1
    return consumed


def _collect_supplied_flags(tokens: list[str]) -> set[str]:
    """Option strings already present in the confirmed tokens."""
    return {tok for tok in tokens if tok.startswith("-")}


def build_prompts(tokens: list[str]) -> list[tuple[str, str, PromptKind]]:
    """Remaining (label, help, kind) prompts for the tokens confirmed so far.

    Empty list means the command is complete: the caller can join the tokens
    and execute the line. Unknown command -> empty list (the caller decides
    how to surface it).

    kind is a PromptKind:
      - REQUIRED: the user must type a value (normal color)
      - OPTIONAL: the user may type a value or skip (GUI renders dimmer)
      - CHOICE:   the user picks one of the listed options (GUI renders a
                  picker; the label is the option name, help is its desc)
    """
    if not tokens:
        # No tokens yet: offer every command name as a CHOICE so the GUI
        # can render command-name completion.
        return [(name, "", PromptKind.CHOICE) for name in COMMANDS]
    cmd = tokens[0]
    if cmd not in COMMANDS:
        return []
    parser, type_choices, subcommand_confirmed = _resolve_parser(tokens)
    consumed = _count_consumed_positionals(tokens, subcommand_confirmed)
    supplied = _collect_supplied_flags(tokens)
    return _prompts_from_parser(parser, type_choices, consumed, supplied)


def is_flag(token: str, cmd: list[str]) -> bool:
    if not cmd or cmd[0] not in PARSERS.keys():
        return False
    parser, _, _ = _resolve_parser(cmd)
    known_flags = {
        opt
        for action in parser._actions
        if action.option_strings
        for opt in action.option_strings
    }
    return token in known_flags


def flag_value_pending(tokens: list[str]) -> tuple[str, int] | None:
    """If the last token is a flag that expects a value, return (flag, remaining args count).
    Otherwise return None. This is used to determine if the user has just typed a flag and is expected to provide a value next.
    """
    if not tokens or tokens[0] not in PARSERS.keys():
        return None

    parser, _, _ = _resolve_parser(tokens)
    known_flags = {
        opt
        for action in parser._actions
        if action.option_strings
        for opt in action.option_strings
    }

    # find last flag in tokens
    last_idx = None
    last_flag = None
    for i, token in enumerate(tokens[1:], start=1):
        if token in known_flags:
            last_idx = i
            last_flag = token

    # if not found, return None
    if last_flag is None:
        return None

    # calculate how many values are expected for this flag
    for action in parser._actions:
        if last_flag in action.option_strings:
            nargs = action.nargs
            if nargs == 0:
                # store_true / store_false flags don't expect a value
                return None
            n = nargs if isinstance(nargs, int) else 1
            # last_idx can't be None here because we found a last_flag.
            # This assertion is for the annoying type checking.
            assert last_idx is not None
            supplied = len(tokens) - 1 - last_idx
            remaining = n - supplied
            if remaining > 0:
                return (last_flag, remaining)
            else:
                return None
