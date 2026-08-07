import sys
import json
from typing import Any, cast

import cmd2

from mechforge import *

# Rename the help category used for cmd2's built-in commands
cmd2.Cmd.DEFAULT_CATEGORY = "MechForge Commands"


# --- model-view serialization ------------------------------------------------
# to_dict() renumbers ids only when an index is passed; called without one it
# keeps the user-facing id and now also includes names. So the human-facing
# view is just to_dict() -- except drivings, which have no wire id.


def _joint_view(j):
    return j.to_dict()  # index=None -> keeps the real user-facing id


def _link_view(l):
    return l.to_dict()


def _driving_view(drv):
    return {"id": drv.id, **drv.to_dict()}


# --- output formatters ------------------------------------------------------
# To add a format: write a formatter(sections: dict) -> str and register it
# here. The `-f` choices are derived from this dict, so they stay in sync.


def _format_json(sections):
    return json.dumps(sections, indent=2)


def _fmt_num(v: float) -> str:
    """Render a coordinate/measurement consistently as a float."""
    f = float(v)
    return f"{f:.1f}" if f.is_integer() else f"{f:g}"


def _format_list(sections):
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
                f"pos=({_fmt_num(j['pos'][0])}, {_fmt_num(j['pos'][1])})"
            )
            if "groundPos" in j:
                gp = j["groundPos"]
                line += f"  ground=({_fmt_num(gp[0])}, {_fmt_num(gp[1])})"
            if "slide" in j:
                axis, pos = j["slide"]["axis"], j["slide"]["pos"]
                line += (
                    f"  slide axis=({_fmt_num(axis[0])}, {_fmt_num(axis[1])}) "
                    f"pos=({_fmt_num(pos[0])}, {_fmt_num(pos[1])})"
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
                f"len={_fmt_num(l['length'])}"
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


# 'list' is the default format (see do_show); dict order also sets the -f choices.
FORMATTERS = {
    "list": _format_list,
    "json": _format_json,
}


# --- joint type subcommands (jointAdd / jointEdit) ------------------------


def _grounded_extra(parser):
    """Type-specific arguments for a Grounded joint (ground point)."""
    parser.add_argument("gx", type=float, help="ground x position")
    parser.add_argument("gy", type=float, help="ground y position")


def _prismatic_extra(parser):
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


def _joint_parser_factory(with_pos: bool = True, extra=None):
    """Return a zero-arg factory for a joint TYPE subcommand parser.

    with_pos includes the x/y positionals (skipped for Grounded, whose
    position is the ground point); extra adds type-specific options.
    """

    def build():
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


def _build_joint(jtype: JointType, args) -> Joint:
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


# --- driving type subcommands (drivingAdd) --------------------------------


def _driving_joint_args(parser):
    """jointA/jointB positionals plus --name shared by every driving type."""
    parser.add_argument("jointA", type=int, help="driven joint id")
    parser.add_argument("jointB", type=int, help="reference joint id")
    parser.add_argument("--name", default=None, help="optional name")


def _position_extra(parser):
    parser.add_argument("--posX", required=True, help="x position expression")
    parser.add_argument("--posY", required=True, help="y position expression")
    parser.add_argument("--velX", default="0", help="x velocity expression")
    parser.add_argument("--velY", default="0", help="y velocity expression")
    parser.add_argument("--accX", default="0", help="x acceleration expression")
    parser.add_argument("--accY", default="0", help="y acceleration expression")


def _angle_extra(parser):
    parser.add_argument("--theta", required=True, help="angle expression")
    parser.add_argument("--omega", default="0", help="angular velocity expression")
    parser.add_argument("--alpha", default="0", help="angular acceleration expression")


def _distance_extra(parser):
    parser.add_argument("--distance", required=True, help="distance expression")
    parser.add_argument("--vel", default="0", help="velocity expression")
    parser.add_argument("--acc", default="0", help="acceleration expression")


def _driving_parser_factory(extra):
    """Return a zero-arg factory for a driving TYPE subcommand parser."""

    def build():
        parser = cmd2.Cmd2ArgumentParser()
        _driving_joint_args(parser)
        extra(parser)
        return parser

    return build


def _build_driving(dtype: DrivingType, args) -> DrivingConstraint:
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


class MechForgeREPL(cmd2.Cmd):
    """Interactive shell for building and solving mechanisms."""

    def __init__(self) -> None:
        super().__init__()

        # disable builtin cmds
        DISABLED_BUILTINS = [
            # "alias",
            # "macro",
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

        self.cli = MechForgeClient()

    # --- clear screen ---
    def do_clear(self, args):
        """Clear the terminal screen"""
        # ANSI escape: clear screen + move cursor to home
        # I think it works on any terminal that supports ANSI escape codes.
        self.poutput("\033[2J\033[H")

    # --- load ---
    load_parser = cmd2.Cmd2ArgumentParser()
    load_parser.add_argument("file", help="path to a mechanism JSON file")

    @cmd2.with_argparser(load_parser)
    def do_load(self, args):
        """Load a mechanism from a JSON file (validated against the schema)."""
        try:
            self.cli.getMechFromFile(args.file)
        except Exception as e:
            self.perror(f"load failed: {e}")
            return
        m = self.cli.mech
        self.poutput(
            f"loaded {len(m.joints)} joint(s), {len(m.links)} link(s), "
            f"{len(m.drivingConstraints)} driving(s)"
        )

    # --- show mechanism ---
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

    @cmd2.with_argparser(show_parser)
    def do_show(self, args):
        """Show the current mechanism.

        No flag prints a summary; -a dumps everything; -j/-l/-d (combinable)
        dump the selected components.
        """
        m = self.cli.mech
        fmt = args.format if args.format is not None else "list"

        # No component flag (and no -a) -> plain-text summary.
        if not (args.all or args.joints or args.links or args.driving):
            if args.format is not None:
                self.poutput(
                    f"note: --format has no effect on the summary; "
                    f"use -a or -j/-l/-d to dump components as '{fmt}'\n"
                )
            self.poutput(f"name: {m.name}")
            self.poutput(f"description: {m.description}")
            self.poutput("---")
            self.poutput(
                f"{len(m.joints)} joint(s), {len(m.links)} link(s), "
                f"{len(m.drivingConstraints)} driving(s)"
            )
            return

        # Mechanism metadata always shown; components selected below.
        sections: dict[str, object] = {
            "name": m.name,
            "description": m.description,
        }

        if args.all:
            sections["joints"] = [_joint_view(j) for j in m.joints]
            sections["links"] = [_link_view(l) for l in m.links]
            sections["drivings"] = [_driving_view(d) for d in m.drivingConstraints]
        else:
            if args.joints:
                sections["joints"] = [_joint_view(j) for j in m.joints]
            if args.links:
                sections["links"] = [_link_view(l) for l in m.links]
            if args.driving:
                sections["drivings"] = [_driving_view(d) for d in m.drivingConstraints]

        formatter = FORMATTERS.get(fmt)
        if formatter is None:  # defensive; argparse choices already restrict this
            self.perror(f"unsupported format: {fmt}")
            return
        self.poutput(formatter(sections))

    # --- add / replace / remove joint ---
    jointAdd_parser = cmd2.Cmd2ArgumentParser(
        description="Add a joint to the mechanism (type-specific subcommand)."
    )
    jointAdd_parser.add_subparsers(title="joint type", metavar="TYPE", required=True)

    @cmd2.with_argparser(jointAdd_parser)
    def do_jointAdd(self, args):
        """Add a joint to the mechanism."""
        args.cmd2_subcommand_func(args)

    @cmd2.as_subcommand_to(
        "jointAdd",
        "ground",
        _joint_parser_factory(with_pos=False, extra=_grounded_extra),
        help="grounded joint",
    )
    def _jointAdd_ground(self, args):
        self._add_joint(JointType.Grounded, args)

    @cmd2.as_subcommand_to(
        "jointAdd", "fixed", _joint_parser_factory(), help="fixed joint"
    )
    def _jointAdd_fixed(self, args):
        self._add_joint(JointType.Fixed, args)

    @cmd2.as_subcommand_to(
        "jointAdd", "revolute", _joint_parser_factory(), help="revolute joint"
    )
    def _jointAdd_revolute(self, args):
        self._add_joint(JointType.Revolute, args)

    @cmd2.as_subcommand_to(
        "jointAdd",
        "prismatic",
        _joint_parser_factory(extra=_prismatic_extra),
        help="prismatic joint",
    )
    def _jointAdd_prismatic(self, args):
        self._add_joint(JointType.Prismatic, args)

    @cmd2.as_subcommand_to(
        "jointAdd", "free", _joint_parser_factory(), help="free joint"
    )
    def _jointAdd_free(self, args):
        self._add_joint(JointType.Free, args)

    def _add_joint(self, jtype: JointType, args) -> None:
        try:
            joint = _build_joint(jtype, args)
            self.cli.add_joint(joint)
        except MechForgeError as e:
            self.perror(f"jointAdd failed: {e}")
            return
        self.poutput(f"added Joint {joint.id} ({joint.type.value})")

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

    @cmd2.with_argparser(jointEdit_parser)
    def do_jointEdit(self, args):
        """Edit a joint; only the fields you specify are changed."""
        changes = {}
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
        try:
            self.cli.edit_joint(args.id, **changes)
        except MechForgeError as e:
            self.perror(f"jointEdit failed: {e}")
            return
        self.poutput(f"edited Joint {args.id}")

    jointRemove_parser = cmd2.Cmd2ArgumentParser()
    jointRemove_parser.add_argument("id", type=int, help="joint id to remove")

    @cmd2.with_argparser(jointRemove_parser)
    def do_jointRemove(self, args):
        """Remove a joint (and links/drivings that reference it)."""
        try:
            self.cli.remove_joint(args.id)
        except MechForgeError as e:
            self.perror(f"jointRemove failed: {e}")
            return
        self.poutput(f"removed Joint {args.id}")

    # --- add / edit / remove link ---
    linkAdd_parser = cmd2.Cmd2ArgumentParser(
        description="Add a link between two joints."
    )
    linkAdd_parser.add_argument("jointA", type=int, help="id of the first joint")
    linkAdd_parser.add_argument("jointB", type=int, help="id of the second joint")
    linkAdd_parser.add_argument("length", type=float, help="link length")
    linkAdd_parser.add_argument("--name", default=None, help="optional name")

    @cmd2.with_argparser(linkAdd_parser)
    def do_linkAdd(self, args):
        """Add a link between two joints."""
        try:
            link = Link(
                jointA=args.jointA,
                jointB=args.jointB,
                length=args.length,
                name=args.name or Link._DEFAULT_NAME,
            )
            self.cli.add_link(link)
        except MechForgeError as e:
            self.perror(f"linkAdd failed: {e}")
            return
        self.poutput(
            f"added Link {link.id} (jointA={link.jointA}, jointB={link.jointB})"
        )

    linkRemove_parser = cmd2.Cmd2ArgumentParser()
    linkRemove_parser.add_argument("id", type=int, help="link id to remove")

    @cmd2.with_argparser(linkRemove_parser)
    def do_linkRemove(self, args):
        """Remove a link from the mechanism."""
        try:
            self.cli.remove_link(args.id)
        except MechForgeError as e:
            self.perror(f"linkRemove failed: {e}")
            return
        self.poutput(f"removed Link {args.id}")

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

    @cmd2.with_argparser(linkEdit_parser)
    def do_linkEdit(self, args):
        """Edit a link; only the fields you specify are changed."""
        changes = {}
        if args.jointA is not None:
            changes["jointA"] = args.jointA
        if args.jointB is not None:
            changes["jointB"] = args.jointB
        if args.length is not None:
            changes["length"] = args.length
        if args.name is not None:
            changes["name"] = args.name
        try:
            self.cli.edit_link(args.id, **changes)
        except MechForgeError as e:
            self.perror(f"linkEdit failed: {e}")
            return
        self.poutput(f"edited Link {args.id}")

    # --- add / edit / remove driving constraint ---
    drivingAdd_parser = cmd2.Cmd2ArgumentParser(
        description="Add a driving constraint (type-specific subcommand)."
    )
    drivingAdd_parser.add_subparsers(
        title="driving type", metavar="TYPE", required=True
    )

    @cmd2.with_argparser(drivingAdd_parser)
    def do_drivingAdd(self, args):
        """Add a driving constraint to the mechanism."""
        args.cmd2_subcommand_func(args)

    @cmd2.as_subcommand_to(
        "drivingAdd",
        "position",
        _driving_parser_factory(_position_extra),
        help="position driving",
    )
    def _drivingAdd_position(self, args):
        self._add_driving(DrivingType.Position, args)

    @cmd2.as_subcommand_to(
        "drivingAdd",
        "angle",
        _driving_parser_factory(_angle_extra),
        help="angle driving",
    )
    def _drivingAdd_angle(self, args):
        self._add_driving(DrivingType.Angle, args)

    @cmd2.as_subcommand_to(
        "drivingAdd",
        "distance",
        _driving_parser_factory(_distance_extra),
        help="distance driving",
    )
    def _drivingAdd_distance(self, args):
        self._add_driving(DrivingType.Distance, args)

    def _add_driving(self, dtype: DrivingType, args) -> None:
        try:
            dc = _build_driving(dtype, args)
            self.cli.add_driving(dc)
        except MechForgeError as e:
            self.perror(f"drivingAdd failed: {e}")
            return
        self.poutput(f"added Driving {dc.id} ({dc.type.value})")

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
    drivingEdit_parser.add_argument("--posX", default=None, help="new x position expression")
    drivingEdit_parser.add_argument("--posY", default=None, help="new y position expression")
    drivingEdit_parser.add_argument("--velX", default=None, help="new x velocity expression")
    drivingEdit_parser.add_argument("--velY", default=None, help="new y velocity expression")
    drivingEdit_parser.add_argument("--accX", default=None, help="new x acceleration expression")
    drivingEdit_parser.add_argument("--accY", default=None, help="new y acceleration expression")
    drivingEdit_parser.add_argument("--theta", default=None, help="new angle expression")
    drivingEdit_parser.add_argument("--omega", default=None, help="new angular velocity expression")
    drivingEdit_parser.add_argument("--alpha", default=None, help="new angular acceleration expression")
    drivingEdit_parser.add_argument("--distance", default=None, help="new distance expression")
    drivingEdit_parser.add_argument("--vel", default=None, help="new velocity expression (Distance)")
    drivingEdit_parser.add_argument("--acc", default=None, help="new acceleration expression (Distance)")

    @cmd2.with_argparser(drivingEdit_parser)
    def do_drivingEdit(self, args):
        """Edit a driving constraint; only the fields you specify are changed."""
        changes = {}
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
        try:
            self.cli.edit_driving(args.id, **changes)
        except MechForgeError as e:
            self.perror(f"drivingEdit failed: {e}")
            return
        self.poutput(f"edited Driving {args.id}")

    drivingRemove_parser = cmd2.Cmd2ArgumentParser()
    drivingRemove_parser.add_argument("id", type=int, help="driving id to remove")

    @cmd2.with_argparser(drivingRemove_parser)
    def do_drivingRemove(self, args):
        """Remove a driving constraint from the mechanism."""
        try:
            self.cli.remove_driving(args.id)
        except MechForgeError as e:
            self.perror(f"drivingRemove failed: {e}")
            return
        self.poutput(f"removed Driving {args.id}")

    # --- solve ---
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

    @cmd2.with_argparser(solve_parser)
    def do_solve(self, args):
        """Solve the mechanism, streaming each step as it is computed."""
        try:
            for step in self.cli.solve(
                endTime=args.endTime,
                timeStep=args.timeStep,
                maxIterations=args.maxIterations,
                tolerance=args.tolerance,
                solveLevel=SolveLevel(args.solveLevel),
            ):
                self.poutput(json.dumps(step))
        except mf_CancelledError as e:
            self.poutput(f"solve cancelled: {e}")
        except MechForgeError as e:
            self.perror(f"solve failed: {e}")


if __name__ == "__main__":
    app = MechForgeREPL()
    sys.exit(app.cmdloop())
