"""On the bench in front of the arms, a spruce-pine-fir 2x4 (38 mm thick, 89 mm deep, 30 cm long) stands on edge
across the robot's view, clamped between two pairs of jaws and raised on two 2x4 offcuts laid flat, with a 40 mm gap
between them under its middle. A pencil X is marked on the middle of its top edge, and a matching X on the middle of its
bottom edge, over the gap.

To the right, a blue 12 V drill/driver lies on its side on the bench with a 1/4 in hex-shank brad-point wood bit (6 in
long) in its quick-change holder, the bit pointing toward the robot. It is set to forward and high speed. Its
variable-speed trigger is on the front of its rubber-gripped handle, just under the motor housing: squeezing the handle
front to back pushes the trigger in, and the harder it is squeezed, the faster the bit turns.

The bit cuts only while it turns and is pushed into the wood. A brad point's centre spur grabs the wood when the bit is
pushed square to it; tilted well over, it skates across the surface. Once the bit is a few millimetres in, the hole
keeps the line it was started on, and levering the drill off that line binds the bit.
"""

# No `from __future__ import annotations` here: the task loader imports this file without registering it in
# sys.modules, and dataclasses then cannot resolve string annotations. Every dataclass below is a plain one with
# concrete field types; no Isaac Lab config class is declared in this file.
import contextlib
import importlib
import importlib.abc
import importlib.machinery
import importlib.util
import math
import sys
import types
import warnings
from collections.abc import Callable, Iterator, Mapping, Sequence
from dataclasses import dataclass
from types import FrameType
from typing import Any, Protocol

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Sphere

SCENE = "station"
SCORE_OVER = "end"
TOP: float = TABLE_TOP[SCENE]
G = 9.81
INCH = 0.0254
Vec3 = tuple[float, float, float]
Pt = Sequence[float] | np.ndarray  # a point or a direction, as a sequence or an array

# --- the board (world frame: x away from the arms, y to their left, z up; metres) --------------------------------
LUM_T = 1.5 * INCH  # a dry dressed 2x4: 38.1 x 88.9 mm (1-1/2 x 3-1/2 in)
LUM_D = 3.5 * INCH
BOARD = "board"
BOARD_LEN = 0.300
BOARD_X = 0.330  # the board's centre line (it runs along y)
MARK_Y = -0.053  # both Xs: on the board's centre line, at this y
RISER_H = LUM_T  # it stands on two 2x4 offcuts laid flat: 38.1 mm tall (the chuck stops the bit 10 mm over the bench)
SLOT = 0.040  # the open gap between them, under the bottom X
RISER_LEN = 0.090
RISER_OFF = SLOT / 2 + RISER_LEN / 2
RISERS = ("riser_right", "riser_left")
JAWS = ("jaw_right_near", "jaw_right_far", "jaw_left_near", "jaw_left_far")
JAW_SIZE = (0.022, 0.040, 0.050)  # each clamp jaw: 22 thick, 40 wide, 50 tall, standing on a riser
TOP_FACE = TOP + RISER_H + LUM_D  # the board's top edge (the entry face)
BOTTOM_FACE = TOP + RISER_H  # its bottom edge (the exit face)
X_TOP = (BOARD_X, MARK_Y, TOP_FACE)
X_BOTTOM = (BOARD_X, MARK_Y, BOTTOM_FACE)
SPF_RGB = (0.86, 0.74, 0.55)
JAW_RGB = (0.20, 0.22, 0.25)
PENCIL_RGB = (0.12, 0.12, 0.13)
MARK_H = 0.0002  # pencil marks stand this proud of the face (looks only)
LINE_W = 0.0012
X_STROKE = 0.022
WOOD_FRICTION = (0.40, 0.35)  # dry wood: 0.25-0.5 (Engineering ToolBox); estimate within it

# --- the drill/driver (a 12 V pocket drill/driver, built from primitives) ------------------------------------------
# Body frame: x along the bit (toward its point), y from the motor housing down the handle toward the battery, z
# through its side. The origin is the handle's centre. Lying on its side on the bench, z points up and the bit's axis
# lies 30 mm above the bench. Envelope, mass and centre of mass follow our own model of a Bosch PS21-class tool
# (150 x 64 x 178 mm, 0.816 kg with its battery), plus the bit.
DRILL, TRIGGER, SPUR = "drill", "trigger", "spur"
BLUE_RGB, GRIP_RGB, STEEL_RGB = (0.09, 0.25, 0.42), (0.06, 0.06, 0.07), (0.70, 0.71, 0.72)
TRIGGER_RGB = (0.55, 0.08, 0.08)
HANDLE_SIZE = (0.042, 0.080, 0.036)  # the handle: 42 front to back, 80 tall, 36 wide
BIT_Y = -0.067  # the bit's axis, across the handle's top
HEAD = ((0.0065, BIT_Y, 0.0), 0.030, 0.125)  # (centre, radius, length): the motor and gear housing
COLLAR = ((0.08425, BIT_Y, 0.0), 0.011, 0.0305)  # the 1/4 in hex quick-change holder
CHUCK_X = 0.0995  # the holder's front face
BIT_R = 0.003175  # 1/4 in
BIT_EXPOSED = 0.127  # a 6 in (152 mm) bit with about 25 mm in the holder
TIP_X = CHUCK_X + BIT_EXPOSED
POINT_LEN = 0.003  # the brad point's centre spur
BIT = ((CHUCK_X + (BIT_EXPOSED - POINT_LEN) / 2, BIT_Y, 0.0), BIT_R, BIT_EXPOSED - POINT_LEN)
TIP_LOCAL = (TIP_X, BIT_Y, 0.0)
FOOT = ((0.004, 0.0605, 0.0), (0.070, 0.041, 0.060))  # the battery foot: as wide as the housing
TRIGGER_SIZE = (0.008, 0.014, 0.016)
TRIGGER_REST = (
    HANDLE_SIZE[0] / 2 + TRIGGER_SIZE[0] / 2,
    -0.014,
    0.0,
)  # 8 mm proud of the handle's front, 16 mm under the housing
TRIGGER_MASS = 0.005
TRIGGER_TRAVEL = 0.006
TRIGGER_K = 2000.0  # N/m: 15 N to start it moving, 18 N at the switch, 27 N fully in (estimate; no figure found)
TRIGGER_PRELOAD = 0.0075
TRIGGER_DAMPING = 3.0
TRIGGER_ON = 0.0015  # the switch closes this far in
GRIP = (0.004, -0.012, 0.0)  # the pinch: across the handle front to back, the pads over the trigger
SPUR_R = 0.0032  # the spur body: the bit's point, as a ball of the bit's radius
SPUR_REST = (TIP_X - SPUR_R, BIT_Y, 0.0)
SPUR_MASS = 0.001
TOOL_MASS = (
    0.816 + 0.030
)  # the tool with its battery (our model, Bosch's kit figure) and the bit (estimate)
DRILL_MASS = TOOL_MASS - TRIGGER_MASS - SPUR_MASS
DRILL_COM = (0.0183, -0.0223, 0.0)  # our model's centre of mass, with the bit's
DRILL_INERTIA = (
    2.96e-3,
    0.98e-3,
    3.63e-3,
)  # kg m^2 about x, y, z through the COM (our model; ixz neglected)
HOUSING_FRICTION = (0.35, 0.30)  # glass-filled nylon on wood (estimate)
GRIP_FRICTION = (1.0, 0.9)  # the rubber-overmoulded handle (estimate)
SPUR_FRICTION = math.tan(
    math.radians(8.0)
)  # the point skates at more than about 8 degrees (estimate)
DRILL_YAW = math.pi  # lying on its side, its bit toward the arms

# --- the cut: how the bit makes a hole (estimates; README.md, "Sources") -----------------------------------------
LOADED_RPM = 1000.0  # the tool's high range is 0-1,300 rpm no-load; about 1,000 cutting (estimate)
THRUST_PER_FEED = 1.19e5  # N per (m per turn) above the bite: McMillin and Woodson 1972, scaled (README, "Sources")
FEED_PER_N = LOADED_RPM / 60.0 / THRUST_PER_FEED  # m/s per N of push above BITE_PUSH, at full speed
# The push the brad point and spurs need before the bit cuts at all (estimate). McMillin and Woodson (1972, Forest
# Products Journal 22(11), p. 59) found 95 percent of a spur bit's thrust goes to advancing its spurs and brad, and
# Szwajka and Trzepiecinski (2018, BioResources 13(4), Fig. 6) measured drilling thrust linear in feed with 40-54 N
# at zero feed (9.3 mm drill, MDF of 742 kg/m3); scaled to 1/4 in in SPF (x0.68 diameter, x0.57-0.63 density) that
# is about 15-25 N. A bench measurement would replace it.
BITE_PUSH = 20.0  # N
TORQUE_IDLE = 0.05  # N m: flute and chip friction in the hole (estimate)
TORQUE_PER_FEED = 180.0  # N m per (m per turn): scaled as above
TORQUE_BIND = 0.6  # N m: a bit levered off its line binds (estimate)
SPUR_K = 2.0e4  # N/m: the spur's spring along the bit (it reads the push)
SPUR_DAMPING = 40.0
LOCK_DEPTH = 0.003  # the hole keeps its line once the bit is this far in
LOCK_SPAN = 0.060  # the line is held at the point and this far up the bit
LOCK_K = (
    2.0e4,
    8.0e3,
)  # N/m at the point and up the bit: about 29 N m/rad against levering (estimate)
LOCK_DAMPING = 100.0
# Every drive gets a finite force limit, well over what it carries in use (a drive left at the infinite default is
# rejected on some stacks): the trigger about 27 N, the spur's spring and the line lock tens of newtons.
TRIGGER_FORCE_LIMIT = 200.0  # N
SPUR_FORCE_LIMIT = 2000.0  # N
LOCK_FORCE_LIMIT = 2000.0  # N
TWIST_DAMPING = 0.5  # N m s/rad: the reaction torque's drive
TWIST_SPIN = 3600.0  # deg/s: its target speed, never reached; its force limit sets the torque
WALK_ANGLE = math.radians(8.0)
BIND_ANGLE = math.radians(2.0)
LEVER_ANGLE = math.radians(6.0)
EXIT_PAST = (
    0.004  # through once the point is this far past the bottom face (the outer spurs have cut out)
)
SPUR_FREE = (
    BIT_EXPOSED - 0.010
)  # after breakthrough the spur rides up the bit: the holder stops 10 mm short
ON_WOOD = 0.002  # the spur's bottom within this of the top face: on the wood
AIR_GAP = 0.010  # running with the point more than this off the wood, outside the hole ...
AIR_STEPS = 4  # ... for this many control steps in a row: run while carried (0)
DROP_SPEED = 0.5  # m/s downward with no gripper holding it: the drill is falling
DEPTH_PART = 0.020  # 0.25 needs a hole at least this deep


def rz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _v(values: Pt) -> np.ndarray:
    return np.asarray(values, dtype=float)


def _t(values: Any) -> Vec3:
    return (float(values[0]), float(values[1]), float(values[2]))


PICK_TCP = (
    0.360,
    -0.300,
    TOP + 0.030,
)  # where the TCP pinches the handle, the drill lying at its start
DRILL_START = _t(_v(PICK_TCP) - rz(DRILL_YAW) @ _v(GRIP))
TRIGGER_START = _t(_v(DRILL_START) + rz(DRILL_YAW) @ _v(TRIGGER_REST))
SPUR_START = _t(_v(DRILL_START) + rz(DRILL_YAW) @ _v(SPUR_REST))


def _box(
    name: str,
    center: Pt,
    size: Pt,
    rgb: Pt,
    mass: float | None = None,
    yaw: float = 0.0,
) -> Box:
    return Box(name, pos=_t(center), size=_t(size), color=_t(rgb), mass=mass, yaw=yaw)


def _jaw_center(name: str) -> Vec3:
    side = -1.0 if "right" in name else 1.0
    face = -1.0 if "near" in name else 1.0
    return (
        BOARD_X + face * (LUM_T / 2 + JAW_SIZE[0] / 2),
        MARK_Y + side * RISER_OFF,
        TOP + RISER_H + JAW_SIZE[2] / 2,
    )


OBJECTS = [
    _box(BOARD, (BOARD_X, MARK_Y, TOP + RISER_H + LUM_D / 2), (LUM_T, BOARD_LEN, LUM_D), SPF_RGB),
    _box(
        RISERS[0],
        (BOARD_X, MARK_Y - RISER_OFF, TOP + RISER_H / 2),
        (LUM_D, RISER_LEN, RISER_H),
        SPF_RGB,
    ),
    _box(
        RISERS[1],
        (BOARD_X, MARK_Y + RISER_OFF, TOP + RISER_H / 2),
        (LUM_D, RISER_LEN, RISER_H),
        SPF_RGB,
    ),
    *(_box(j, _jaw_center(j), JAW_SIZE, JAW_RGB) for j in JAWS),
    _box(DRILL, DRILL_START, HANDLE_SIZE, GRIP_RGB, DRILL_MASS, DRILL_YAW),
    _box(TRIGGER, TRIGGER_START, TRIGGER_SIZE, TRIGGER_RGB, TRIGGER_MASS, DRILL_YAW),
    Sphere(SPUR, pos=SPUR_START, radius=SPUR_R, color=STEEL_RGB, mass=SPUR_MASS, yaw=DRILL_YAW),
]
SPECS = {o.name: o for o in OBJECTS}
STATIC = (BOARD, *RISERS, *JAWS)
FREE = (DRILL, TRIGGER, SPUR)

# --- every body's shapes, in its own frame (centre, size): what its outline takes in -----------------------------
Shape = tuple[Vec3, Vec3]


def _cyl_x(center: Pt, radius: float, length: float) -> Shape:
    return _t(center), (length, 2 * radius, 2 * radius)


def _x_marks() -> tuple[Shape, Shape]:
    """The two pencil Xs (board frame), each as the box its two crossed strokes fill."""
    half = (X_STROKE + LINE_W) / (2 * math.sqrt(2))
    size = (2 * half, 2 * half, MARK_H)
    top: Shape = ((0.0, 0.0, LUM_D / 2 + MARK_H / 2), size)
    bottom: Shape = ((0.0, 0.0, -LUM_D / 2 - MARK_H / 2), size)
    return top, bottom


def shapes(name: str) -> list[Shape]:
    """Every shape of a body as a box in its own frame: the primitive the task-file API makes and what the hook adds
    (looks-only shapes too: the framework's outline takes in all of a body's geometry)."""
    spec = SPECS[name]
    if isinstance(spec, Sphere):
        return [((0.0, 0.0, 0.0), (2 * spec.radius, 2 * spec.radius, 2 * spec.radius))]
    own: Shape = ((0.0, 0.0, 0.0), _t(spec.size))
    if name == BOARD:
        return [own, *_x_marks()]
    if name == DRILL:
        point: Shape = ((TIP_X - POINT_LEN / 2, BIT_Y, 0.0), (POINT_LEN, 2 * BIT_R, 2 * BIT_R))
        return [
            own,
            _cyl_x(*HEAD),
            _cyl_x(*COLLAR),
            _cyl_x(*BIT),
            point,
            (_t(FOOT[0]), _t(FOOT[1])),
        ]
    return [own]


def local_outline(name: str) -> tuple[np.ndarray, np.ndarray]:
    """A body's outline in its own frame (low and high corners), as built."""
    lows = [np.subtract(c, np.divide(s, 2)) for c, s in shapes(name)]
    highs = [np.add(c, np.divide(s, 2)) for c, s in shapes(name)]
    return np.min(lows, axis=0), np.max(highs, axis=0)


# --- the scene hook: what the task-file API can't express -------------------------------------------------------
# yam_arena.scene.task_object(spec) turns each task object into an Arena object. Right after that module is imported,
# this wraps task_object, matching this file's own object specs by value, so that after Arena spawns one its spawner
# also builds what that object needs:
#  - the board: wood's friction (the framework's grippy material is replaced) and the two pencil Xs (looks);
#  - the risers: wood's friction;
#  - the drill: its housing, holder and battery foot (more colliders of its body), the bit and its point (looks: the
#    bit touches the wood only through the spur), the rubber grip's friction, its mass properties;
#  - the trigger: its spring joint to the handle;
#  - the spur: the point's friction, its spring joint along the bit (the cut's depth is its set point, written each
#    step), and the hole's line lock (two joints from the world to the drill whose drives are off until the bit is
#    in; their set points follow the line it started on) with the reaction torque's drive.
# On a machine without the simulator nothing imports yam_arena.scene, and this does nothing. What was built is
# recorded on the scene module; scoring gives 0 unless every part was built (it never fails open).
_SCENE_MODULE = "yam_arena.scene"
PARTS_FOLDER = "parts"
JOINTS_FOLDER = "drill_hole_joints"
HOOK_TAG = "drill_hole_mark_to_mark_parts"
BUILD_RECORD = (
    "drill_hole_mark_to_mark_build"  # an attribute of the scene module: what the hook built
)
JOINT_NAMES = ("trigger", "spur", "lock_point", "lock_upper")


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


class _Pxr:
    """The pxr modules, imported when the simulator is running."""

    def __init__(self) -> None:
        self.gf = importlib.import_module("pxr.Gf")
        self.sdf = importlib.import_module("pxr.Sdf")
        self.geom = importlib.import_module("pxr.UsdGeom")
        self.physics = importlib.import_module("pxr.UsdPhysics")
        self.shade = importlib.import_module("pxr.UsdShade")
        self.usd = importlib.import_module("pxr.Usd")


def _material(
    px: _Pxr,
    stage: Any,
    path: str,
    rgb: Pt,
    friction: tuple[float, float] | None,
    combine_min: bool = False,
) -> Any:
    material = px.shade.Material.Define(stage, path)
    shader = px.shade.Shader.Define(stage, f"{path}/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", px.sdf.ValueTypeNames.Color3f).Set(
        px.gf.Vec3f(*(_linear(c) for c in rgb))
    )
    shader.CreateInput("roughness", px.sdf.ValueTypeNames.Float).Set(0.6)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    if friction is not None:
        api = px.physics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(friction[0])
        api.CreateDynamicFrictionAttr(friction[1])
        api.CreateRestitutionAttr(0.0)
        if (
            combine_min
        ):  # this material's friction holds against any other (PhysX: min outranks average)
            prim = material.GetPrim()
            prim.AddAppliedSchema("PhysxMaterialAPI")
            prim.CreateAttribute(
                "physxMaterial:frictionCombineMode", px.sdf.ValueTypeNames.Token
            ).Set("min")
    return material


def _bind(px: _Pxr, prim: Any, material: Any, collides: bool) -> None:
    api = px.shade.MaterialBindingAPI.Apply(prim)
    api.Bind(material)
    if collides:
        api.Bind(material, px.shade.Tokens.weakerThanDescendants, "physics")


def spec_colliders(px: _Pxr, stage: Any, body: str) -> list[Any]:
    """The colliders the framework made for a spec (its own primitive, wherever under the body it put it): every prim
    under the body with a collision API, except the parts this file adds."""
    folder = f"{body}/{PARTS_FOLDER}"
    return [
        prim
        for prim in px.usd.PrimRange(stage.GetPrimAtPath(body))
        if prim.HasAPI(px.physics.CollisionAPI)
        and not (str(prim.GetPath()) == folder or str(prim.GetPath()).startswith(folder + "/"))
    ]


def _rebind_physics(px: _Pxr, stage: Any, body: str, material: Any) -> None:
    """Replace the physics material the framework bound on a spec's own collider (grippy rubber) with ``material``.
    No collider found means the body isn't as stated: the build fails (and the score is 0)."""
    found = spec_colliders(px, stage, body)
    if not found:
        raise RuntimeError(f"no collider under {body}")
    for prim in found:
        px.shade.MaterialBindingAPI.Apply(prim).Bind(
            material, px.shade.Tokens.strongerThanDescendants, "physics"
        )


def _place(px: _Pxr, prim: Any, center: Pt, scale: Pt | None = None, yaw_deg: float = 0.0) -> None:
    xform = px.geom.Xformable(prim)
    xform.AddTranslateOp().Set(px.gf.Vec3d(*(float(c) for c in center)))
    if yaw_deg:
        xform.AddRotateZOp().Set(float(yaw_deg))
    if scale is not None:
        xform.AddScaleOp().Set(px.gf.Vec3f(*(float(s) for s in scale)))


def _cube(
    px: _Pxr,
    stage: Any,
    path: str,
    center: Pt,
    size: Pt,
    material: Any,
    collides: bool,
    yaw_deg: float = 0.0,
) -> str:
    cube = px.geom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([px.gf.Vec3f(-0.5, -0.5, -0.5), px.gf.Vec3f(0.5, 0.5, 0.5)])
    _place(px, cube.GetPrim(), center, size, yaw_deg)
    if collides:
        px.physics.CollisionAPI.Apply(cube.GetPrim())
    _bind(px, cube.GetPrim(), material, collides)
    return path


def _cylinder_x(
    px: _Pxr,
    stage: Any,
    path: str,
    center: Pt,
    radius: float,
    length: float,
    material: Any,
    collides: bool,
) -> str:
    cylinder = px.geom.Cylinder.Define(stage, path)
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(length)
    cylinder.CreateAxisAttr("X")
    cylinder.CreateExtentAttr(
        [px.gf.Vec3f(-length / 2, -radius, -radius), px.gf.Vec3f(length / 2, radius, radius)]
    )
    _place(px, cylinder.GetPrim(), center)
    if collides:
        px.physics.CollisionAPI.Apply(cylinder.GetPrim())
    _bind(px, cylinder.GetPrim(), material, collides)
    return path


def _cone_x(
    px: _Pxr, stage: Any, path: str, center: Pt, radius: float, length: float, material: Any
) -> str:
    """A looks-only cone along +x (its point toward +x)."""
    cone = px.geom.Cone.Define(stage, path)
    cone.CreateRadiusAttr(radius)
    cone.CreateHeightAttr(length)
    cone.CreateAxisAttr("X")
    cone.CreateExtentAttr(
        [px.gf.Vec3f(-length / 2, -radius, -radius), px.gf.Vec3f(length / 2, radius, radius)]
    )
    _place(px, cone.GetPrim(), center)
    _bind(px, cone.GetPrim(), material, False)
    return path


def _world_of(px: _Pxr, stage: Any, path: str) -> np.ndarray:
    xf = px.geom.Xformable(stage.GetPrimAtPath(path)).ComputeLocalToWorldTransform(
        px.usd.TimeCode.Default()
    )
    t = xf.ExtractTranslation()
    return _v((t[0], t[1], t[2]))


def _quat(px: _Pxr, wxyz: Sequence[float]) -> Any:
    return px.gf.Quatf(*(float(v) for v in wxyz))


def _joint(
    px: _Pxr,
    stage: Any,
    path: str,
    kind: str,
    body0: str,
    body1: str,
    pos0: Pt,
    rot0: Sequence[float],
    pos1: Pt = (0.0, 0.0, 0.0),
    rot1: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
    axis: str = "X",
) -> Any:
    """A joint from body0 (or the world, "") to body1: its frame at ``pos0``/``rot0`` (w, x, y, z) in body0 (or the
    stage) and at ``pos1``/``rot1`` in body1."""
    cls = {"prismatic": px.physics.PrismaticJoint, "d6": px.physics.Joint}[kind]
    joint = cls.Define(stage, path)
    if body0:
        joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr(px.gf.Vec3f(*(float(v) for v in pos0)))
    joint.CreateLocalRot0Attr(_quat(px, rot0))
    joint.CreateLocalPos1Attr(px.gf.Vec3f(*(float(v) for v in pos1)))
    joint.CreateLocalRot1Attr(_quat(px, rot1))
    if kind == "prismatic":
        joint.CreateAxisAttr(axis)
    return joint


def _drive(
    px: _Pxr,
    prim: Any,
    kind: str,
    stiffness: float,
    damping: float,
    target: float = 0.0,
    velocity: float = 0.0,
    max_force: float | None = None,
) -> None:
    drive = px.physics.DriveAPI.Apply(prim, kind)
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(stiffness))
    drive.CreateDampingAttr(float(damping))
    drive.CreateTargetPositionAttr(float(target))
    drive.CreateTargetVelocityAttr(float(velocity))
    if max_force is not None:
        drive.CreateMaxForceAttr(float(max_force))


def _lock(px: _Pxr, prim: Any, axes: Sequence[str]) -> None:
    for axis in axes:  # locked: low above high
        limit = px.physics.LimitAPI.Apply(prim, axis)
        limit.CreateLowAttr(1.0)
        limit.CreateHighAttr(-1.0)


def _tune_body(px: _Pxr, prim: Any, iterations: tuple[int, int]) -> None:
    prim.AddAppliedSchema("PhysxRigidBodyAPI")
    ints = px.sdf.ValueTypeNames.Int
    prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", ints).Set(iterations[0])
    prim.CreateAttribute("physxRigidBody:solverVelocityIterationCount", ints).Set(iterations[1])


DOWN_WXYZ = (
    math.cos(math.pi / 4),
    0.0,
    math.sin(math.pi / 4),
    0.0,
)  # x turned to point straight down
FLIP_X_WXYZ = (0.0, 1.0, 0.0, 0.0)  # half a turn about x
FLIP_Z_WXYZ = (0.0, 0.0, 0.0, 1.0)  # half a turn about z
SOLVER = (32, 4)


def author_board(stage: Any, env_root: str, name: str = BOARD) -> list[str]:
    """The board: wood's friction on its own collider and the two pencil Xs (looks only)."""
    px = _Pxr()
    body = f"{env_root}/{name}"
    folder = f"{body}/{PARTS_FOLDER}"
    wood = _material(px, stage, f"{folder}/wood", SPF_RGB, WOOD_FRICTION)
    _rebind_physics(px, stage, body, wood)
    pencil = _material(px, stage, f"{folder}/pencil", PENCIL_RGB, None)
    made = [f"{folder}/wood", f"{folder}/pencil"]
    for face, (center, _) in zip(("top", "bottom"), _x_marks(), strict=True):
        for k, angle in enumerate((45.0, -45.0)):
            made.append(
                _cube(
                    px,
                    stage,
                    f"{folder}/x_{face}_{k}",
                    center,
                    (X_STROKE, LINE_W, MARK_H),
                    pencil,
                    False,
                    angle,
                )
            )
    return made


def author_wood(stage: Any, env_root: str, name: str) -> list[str]:
    """A riser: wood's friction on its own collider."""
    px = _Pxr()
    wood = _material(px, stage, f"{env_root}/{name}/{PARTS_FOLDER}/wood", SPF_RGB, WOOD_FRICTION)
    _rebind_physics(px, stage, f"{env_root}/{name}", wood)
    return [f"{env_root}/{name}/{PARTS_FOLDER}/wood"]


def author_drill(stage: Any, env_root: str, name: str = DRILL) -> list[str]:
    """The drill's housing, holder and battery foot (more colliders of its body), the bit and its point (looks only),
    the rubber grip's friction on the handle, and its mass properties."""
    px = _Pxr()
    body = f"{env_root}/{name}"
    folder = f"{body}/{PARTS_FOLDER}"
    housing = _material(px, stage, f"{folder}/housing", BLUE_RGB, HOUSING_FRICTION)
    black = _material(px, stage, f"{folder}/black", GRIP_RGB, HOUSING_FRICTION)
    grip = _material(px, stage, f"{folder}/grip", GRIP_RGB, GRIP_FRICTION)
    steel = _material(px, stage, f"{folder}/steel", STEEL_RGB, None)
    _rebind_physics(px, stage, body, grip)
    made = [f"{folder}/housing", f"{folder}/black", f"{folder}/grip", f"{folder}/steel"]
    made.append(_cylinder_x(px, stage, f"{folder}/head", HEAD[0], HEAD[1], HEAD[2], housing, True))
    made.append(
        _cylinder_x(px, stage, f"{folder}/holder", COLLAR[0], COLLAR[1], COLLAR[2], black, True)
    )
    made.append(_cylinder_x(px, stage, f"{folder}/bit", BIT[0], BIT[1], BIT[2], steel, False))
    made.append(
        _cone_x(
            px,
            stage,
            f"{folder}/brad_point",
            (TIP_X - POINT_LEN / 2, BIT_Y, 0.0),
            BIT_R,
            POINT_LEN,
            steel,
        )
    )
    made.append(_cube(px, stage, f"{folder}/battery_foot", FOOT[0], FOOT[1], black, True))
    prim = stage.GetPrimAtPath(body)
    mass = px.physics.MassAPI.Apply(prim)
    mass.CreateCenterOfMassAttr(px.gf.Vec3f(*(float(v) for v in DRILL_COM)))
    mass.CreateDiagonalInertiaAttr(px.gf.Vec3f(*(float(v) for v in DRILL_INERTIA)))
    mass.CreatePrincipalAxesAttr(px.gf.Quatf(1.0, 0.0, 0.0, 0.0))
    _tune_body(px, prim, SOLVER)
    return made


def author_trigger(stage: Any, env_root: str, name: str = TRIGGER) -> list[str]:
    """The trigger's spring joint to the handle: it slides in along the bit's axis, sprung out against its stop."""
    px = _Pxr()
    path = f"{env_root}/{JOINTS_FOLDER}/trigger"
    joint = _joint(
        px,
        stage,
        path,
        "prismatic",
        f"{env_root}/{DRILL}",
        f"{env_root}/{name}",
        TRIGGER_REST,
        (1.0, 0, 0, 0),
    )
    joint.CreateLowerLimitAttr(-TRIGGER_TRAVEL)
    joint.CreateUpperLimitAttr(0.0)
    _drive(
        px,
        joint.GetPrim(),
        "linear",
        TRIGGER_K,
        TRIGGER_DAMPING,
        TRIGGER_PRELOAD,
        max_force=TRIGGER_FORCE_LIMIT,
    )
    return [path]


def author_spur(stage: Any, env_root: str, name: str = SPUR) -> list[str]:
    """The spur: the point's friction (it holds against any other material: min), its spring joint along the bit (its
    frame's x points up the bit, so its set point is the cut's depth), and the hole's line lock: two joints from the
    world (their frames at the top X, x pointing down) to the drill (at the point, and LOCK_SPAN up the bit), every
    drive off until the bit has bitten. The point's joint also carries the reaction torque's drive about the bit."""
    px = _Pxr()
    body = f"{env_root}/{name}"
    point = _material(
        px, stage, f"{body}/{PARTS_FOLDER}/point", STEEL_RGB, (SPUR_FRICTION, SPUR_FRICTION), True
    )
    _rebind_physics(px, stage, body, point)
    made = [f"{body}/{PARTS_FOLDER}/point"]
    joints = f"{env_root}/{JOINTS_FOLDER}"
    spur = _joint(
        px,
        stage,
        f"{joints}/spur",
        "d6",
        f"{env_root}/{DRILL}",
        body,
        SPUR_REST,
        FLIP_Z_WXYZ,
        (0.0, 0.0, 0.0),
        FLIP_Z_WXYZ,
    )
    _lock(px, spur.GetPrim(), ("transY", "transZ", "rotX", "rotY", "rotZ"))
    slide = px.physics.LimitAPI.Apply(spur.GetPrim(), "transX")
    slide.CreateLowAttr(-0.001)
    slide.CreateHighAttr(float(BIT_EXPOSED - 0.005))
    _drive(px, spur.GetPrim(), "transX", SPUR_K, SPUR_DAMPING, max_force=SPUR_FORCE_LIMIT)
    made.append(f"{joints}/spur")
    origin = _world_of(px, stage, env_root)
    anchor = _v(X_TOP) + origin  # body0 is the world: its frame is in stage coordinates
    for label, along in (("lock_point", 0.0), ("lock_upper", LOCK_SPAN)):
        lock = _joint(
            px,
            stage,
            f"{joints}/{label}",
            "d6",
            "",
            f"{env_root}/{DRILL}",
            anchor,
            DOWN_WXYZ,
            (TIP_X - along, BIT_Y, 0.0),
            FLIP_X_WXYZ,
        )
        for axis in ("transY", "transZ"):
            _drive(px, lock.GetPrim(), axis, 0.0, 0.0, max_force=LOCK_FORCE_LIMIT)
        if label == "lock_point":
            _drive(
                px,
                lock.GetPrim(),
                "rotX",
                0.0,
                TWIST_DAMPING * math.pi / 180.0,  # USD angular drives are per degree
                velocity=-TWIST_SPIN,
                max_force=0.0,
            )
        made.append(f"{joints}/{label}")
    _tune_body(px, stage.GetPrimAtPath(body), SOLVER)
    return made


AUTHORS: dict[str, Callable[[Any, str], list[str]]] = {
    BOARD: lambda stage, root: author_board(stage, root),
    RISERS[0]: lambda stage, root: author_wood(stage, root, RISERS[0]),
    RISERS[1]: lambda stage, root: author_wood(stage, root, RISERS[1]),
    DRILL: lambda stage, root: author_drill(stage, root),
    TRIGGER: lambda stage, root: author_trigger(stage, root),
    SPUR: lambda stage, root: author_spur(stage, root),
}


def _env_roots(prim: Any, name: str) -> list[str]:
    """Every environment folder holding the object ``name``, given its prim just spawned (cloned copies included)."""
    stage = prim.GetStage()
    envs = prim.GetPath().GetParentPath().GetParentPath()
    return [
        str(env.GetPath())
        for env in stage.GetPrimAtPath(envs).GetChildren()
        if env.GetChild(name).IsValid()
    ]


def _build_record(scene: Any) -> dict[str, Any]:
    """What the hook built, kept on the scene module so every copy of this file loaded in the process reads the same
    record: object name -> "built" or "failed", and "env_root" -> the scored environment's folder."""
    record = getattr(scene, BUILD_RECORD, None)
    if not isinstance(record, dict):
        record = {}
        setattr(scene, BUILD_RECORD, record)
    return record


def _with_parts(spawn: Callable[..., Any], name: str, record: dict[str, Any]) -> Callable[..., Any]:
    def spawn_and_build(prim_path: str, cfg: Any, *args: Any, **kwargs: Any) -> Any:
        prim = spawn(prim_path, cfg, *args, **kwargs)
        try:
            roots = _env_roots(prim, name)
            if not roots:
                raise RuntimeError("no environment holds it")
            for root in roots:
                AUTHORS[name](prim.GetStage(), root)
            first = next((r for r in roots if r.endswith("/env_0")), roots[0])
            record.setdefault("env_root", first)
        except Exception as error:  # the scene is incomplete: scoring sees the record and gives 0
            record[name] = "failed"
            _warn_once(
                f"build-{name}", f"could not build {name}'s parts ({type(error).__name__}: {error})"
            )
        else:
            if record.get(name) != "failed":  # a failure in any spawn sticks
                record[name] = "built"
        return prim

    setattr(spawn_and_build, HOOK_TAG, name)
    return spawn_and_build


def _ours(spec: Any) -> bool:
    """Whether ``spec`` is one of this file's objects, compared field by field (a second load of this file is served
    by the first wrapper; another task's object that merely shares a name is left alone)."""
    try:
        return getattr(spec, "name", None) in AUTHORS and any(spec == own for own in OBJECTS)
    except Exception:
        return False


def _spawner_cfgs(item: object) -> Iterator[Any]:
    """The spawner configs an Arena object holds (one level deep): objects with a callable ``func`` and a ``size`` or
    a ``radius`` (a cuboid's or a sphere's)."""
    seen: set[int] = set()
    stack = [item, *vars(item).values()] if hasattr(item, "__dict__") else [item]
    for candidate in stack:
        for value in (
            candidate,
            getattr(candidate, "spawn", None),
            getattr(candidate, "spawner_cfg", None),
        ):
            if value is None or id(value) in seen:
                continue
            seen.add(id(value))
            if callable(getattr(value, "func", None)) and (
                hasattr(value, "size") or hasattr(value, "radius")
            ):
                yield value


def build_scene(scene: Any) -> bool:
    """Wrap ``scene.task_object`` so this file's objects come out with their parts; True if it wrapped it now."""
    original = getattr(scene, "task_object", None)
    if original is None or getattr(original, HOOK_TAG, None) is not None:
        return False
    record = _build_record(scene)

    def task_object(spec: Any) -> Any:
        if (
            _ours(spec) and spec.name == OBJECTS[0].name
        ):  # the first of this task's objects: a new build starts
            record.clear()
        made = original(spec)
        if _ours(spec):
            found = 0
            for cfg in _spawner_cfgs(made):
                found += 1
                if getattr(cfg.func, HOOK_TAG, None) is None:
                    cfg.func = _with_parts(cfg.func, spec.name, record)
            if not found:
                record[spec.name] = "failed"
                _warn_once(
                    f"spawner-{spec.name}",
                    f"found no spawner for {spec.name}; its parts will be missing",
                )
        return made

    setattr(task_object, HOOK_TAG, "task_object")
    scene.task_object = task_object
    return True


class _SceneWatcher(importlib.abc.MetaPathFinder):
    """Waits for yam_arena.scene to be imported, lets it load normally, then applies build_scene. It answers for that
    one module name only, and removes itself once the module has loaded."""

    marker = "drill_hole_mark_to_mark.scene_watcher"

    def __init__(self) -> None:
        self._looking = False

    def find_spec(
        self, fullname: str, path: Sequence[str] | None, target: types.ModuleType | None = None
    ) -> importlib.machinery.ModuleSpec | None:
        if fullname != _SCENE_MODULE or self._looking:
            return None
        self._looking = True  # find_spec below asks every finder again, this one included
        try:
            spec = importlib.util.find_spec(fullname)
        except (
            Exception
        ) as error:  # never break the framework's import: let the other finders answer
            self.disarm()
            _warn_once(
                "finder", f"could not watch the scene module ({type(error).__name__}: {error})"
            )
            return None
        finally:
            self._looking = False
        if spec is None:
            self.disarm()
            return None
        if spec.loader is not None:
            spec.loader = _BuildAfterLoad(spec.loader, self)
        return spec

    def disarm(self) -> None:
        if self in sys.meta_path:
            sys.meta_path.remove(self)


class _BuildAfterLoad(importlib.abc.Loader):
    """The module's own loader, plus build_scene once the module has run."""

    def __init__(self, inner: importlib.abc.Loader, watcher: _SceneWatcher) -> None:
        self._inner = inner
        self._watcher = watcher

    def create_module(self, spec: importlib.machinery.ModuleSpec) -> types.ModuleType | None:
        return self._inner.create_module(spec)

    def exec_module(self, module: types.ModuleType) -> None:
        try:
            self._inner.exec_module(module)
        finally:
            self._watcher.disarm()
        try:
            build_scene(module)
        except Exception as error:  # the import must still succeed; the task then scores 0
            with contextlib.suppress(Exception):
                _build_record(module).update(dict.fromkeys(AUTHORS, "failed"))
            _warn_once(
                "wrap", f"could not wrap the scene builder ({type(error).__name__}: {error})"
            )

    def __getattr__(self, name: str) -> Any:  # get_source and friends, for tracebacks
        if name.startswith("__") or name in ("_inner", "_watcher"):
            raise AttributeError(name)
        return getattr(self._inner, name)


def install_hook() -> None:
    """Wrap the scene builder now if it is already imported; otherwise wait for its import (once)."""
    scene = sys.modules.get(_SCENE_MODULE)
    if scene is not None:
        build_scene(scene)
        return
    if not any(getattr(finder, "marker", None) == _SceneWatcher.marker for finder in sys.meta_path):
        sys.meta_path.insert(0, _SceneWatcher())


def parts_built() -> bool:
    """Whether the hook built every object's parts (read from the scene module's record; no record, no scene)."""
    scene = sys.modules.get(_SCENE_MODULE)
    record = getattr(scene, BUILD_RECORD, None) if scene is not None else None
    return isinstance(record, dict) and all(record.get(name) == "built" for name in AUTHORS)


def _env_root() -> str:
    scene = sys.modules.get(_SCENE_MODULE)
    record = getattr(scene, BUILD_RECORD, None) if scene is not None else None
    root = record.get("env_root") if isinstance(record, dict) else None
    return root if isinstance(root, str) else "/World/envs/env_0"


# --- reading the parts ------------------------------------------------------------------------------------------
class _State(Protocol):
    pos: Any
    quat_xyzw: Any
    lin_vel: Any
    ang_vel: Any
    low: Any
    high: Any


def rotation(quat_xyzw: Any) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


MAX_SPEED, MAX_SPIN = 0.01, 0.5  # still: m/s, rad/s
TABLE_X = (-0.19, 0.91)
TABLE_Y = (-0.65, 0.65)
LOST_BELOW = 0.05


@dataclass(frozen=True)
class PartView:
    """One body as the simulator reports it: its outline's centre and corners, its rotation, its body origin (from
    its outline and the shapes it was built with), how fast it moves, and its vertical speed."""

    name: str
    center: np.ndarray
    low: np.ndarray
    high: np.ndarray
    rot: np.ndarray
    speed: float
    spin: float
    vz: float

    @staticmethod
    def of(name: str, state: _State) -> "PartView":
        vel = np.asarray(state.lin_vel, dtype=float)
        return PartView(
            name,
            np.asarray(state.pos, dtype=float),
            np.asarray(state.low, dtype=float),
            np.asarray(state.high, dtype=float),
            rotation(state.quat_xyzw),
            float(np.linalg.norm(vel)),
            float(np.linalg.norm(np.asarray(state.ang_vel, dtype=float))),
            float(vel[2]),
        )

    @property
    def origin(self) -> np.ndarray:
        low, high = local_outline(self.name)
        middle = (self.low + self.high) / 2
        return np.asarray(middle - self.rot @ ((low + high) / 2), dtype=float)

    def world(self, local: Pt) -> np.ndarray:
        return np.asarray(self.origin + self.rot @ _v(local), dtype=float)

    def local(self, point: Pt) -> np.ndarray:
        return np.asarray(self.rot.T @ (_v(point) - self.origin), dtype=float)

    @property
    def still(self) -> bool:
        return self.speed < MAX_SPEED and self.spin < MAX_SPIN

    @property
    def lost(self) -> bool:
        x, y = float(self.center[0]), float(self.center[1])
        off = not (TABLE_X[0] <= x <= TABLE_X[1] and TABLE_Y[0] <= y <= TABLE_Y[1])
        return off or float(self.low[2]) < TOP - LOST_BELOW or not np.all(np.isfinite(self.center))


def views(objects: Mapping[str, _State]) -> dict[str, PartView]:
    return {name: PartView.of(name, objects[name]) for name in FREE}


def angle_between(a: Pt, b: Pt) -> float:
    u, v = _v(a), _v(b)
    c = float(u @ v) / ((float(np.linalg.norm(u)) * float(np.linalg.norm(v))) or 1.0)
    return math.acos(max(-1.0, min(1.0, c)))


# --- the drill and the hole: what cuts, how fast, and where -------------------------------------------------------
DOWN = (0.0, 0.0, -1.0)


def trigger_travel(drill: PartView, trigger: PartView) -> float:
    """How far the trigger is squeezed in (m), from its body's place on the handle."""
    rel = drill.local(trigger.origin)
    return float(min(max(TRIGGER_REST[0] - rel[0], 0.0), TRIGGER_TRAVEL))


def trigger_speed(travel: float) -> float:
    """The variable-speed trigger: 0 below its switch point, then in proportion up to full squeeze (0..1)."""
    if travel < TRIGGER_ON:
        return 0.0
    return float(min(1.0, (travel - TRIGGER_ON) / (TRIGGER_TRAVEL - TRIGGER_ON)))


def bit_tip(drill: PartView) -> np.ndarray:
    return drill.world(TIP_LOCAL)


def bit_axis(drill: PartView) -> np.ndarray:
    return np.asarray(drill.rot[:, 0], dtype=float)


def spur_offset(drill: PartView, spur: PartView) -> float:
    """How far up the bit the spur sits from its rest at the point (m)."""
    return float(SPUR_REST[0] - drill.local(spur.origin)[0])


def on_plane(point: Pt, axis: Pt, z: float) -> np.ndarray | None:
    """Where the line through ``point`` along ``axis`` crosses the horizontal plane at height ``z``."""
    p, a = _v(point), _v(axis)
    if abs(float(a[2])) < 1e-6:
        return None
    return np.asarray(p + a * ((z - float(p[2])) / float(a[2])), dtype=float)


def over_board(point: Pt, margin: float = 0.0) -> bool:
    """Whether a point is over the board's top edge (x, y within its footprint, less ``margin``)."""
    x, y = float(point[0]), float(point[1])
    return abs(x - BOARD_X) <= LUM_T / 2 - margin and abs(y - MARK_Y) <= BOARD_LEN / 2 - margin


def _xy(point: Pt) -> np.ndarray:
    return np.asarray(_v(point)[:2], dtype=float)


@dataclass
class Hole:
    """The hole over the trial: how deep the bit has cut (the spur's set point, m), where and along what line it went
    in, whether and where it came out, and this step's push, feed, torque and angles."""

    depth: float = 0.0
    deepest: float = 0.0
    locked: bool = False
    pin: np.ndarray | None = (
        None  # where the spur has pinned the point on the top face (world), before the line locks
    )
    entry: np.ndarray | None = None  # where the line crosses the top face (world)
    line: np.ndarray | None = None  # its direction (unit, pointing down into the board)
    through: bool = False
    exit: np.ndarray | None = None  # where it crosses the bottom face (world)
    withdrawn: bool = False  # the bit came back out after biting: this trial's hole is done
    levered: bool = False
    ran_in_air: bool = False
    air_steps: int = 0
    push: float = 0.0  # N, read from the spur's give
    feed: float = 0.0  # m/s this step
    torque: float = 0.0  # N m this step
    tilt: float = 0.0  # the bit from straight down (rad)
    off_line: float = 0.0  # the bit from its locked line (rad)
    speed: float = 0.0  # the trigger's speed setting (0..1)
    travel: float = 0.0  # the trigger's travel (m)
    last_target: float = 0.0  # the spur's set point written last step

    def entry_error(self) -> float:
        return (
            math.inf if self.entry is None else float(np.linalg.norm(_xy(self.entry) - _xy(X_TOP)))
        )

    def exit_error(self) -> float:
        return (
            math.inf if self.exit is None else float(np.linalg.norm(_xy(self.exit) - _xy(X_BOTTOM)))
        )

    def step(self, parts: Mapping[str, PartView], dt: float) -> list[tuple[str, str, float]]:
        """Advance one control step; the writes ((joint, attribute, value), joints by their JOINT_NAMES)."""
        drill, spur = parts[DRILL], parts[SPUR]
        self.travel = trigger_travel(drill, parts[TRIGGER])
        self.speed = trigger_speed(self.travel)
        axis, tip = bit_axis(drill), bit_tip(drill)
        self.push = max(0.0, (spur_offset(drill, spur) - self.last_target) * SPUR_K)
        self.tilt = angle_between(axis, DOWN)
        spur_bottom = spur.world((0.0, 0.0, 0.0))[2] - SPUR_R
        on_wood = (
            over_board(spur.world((0.0, 0.0, 0.0)))
            and abs(float(spur_bottom) - TOP_FACE) <= ON_WOOD
        )
        self.off_line = (
            0.0 if self.line is None or not self.locked else angle_between(axis, self.line)
        )
        if (
            not self.locked and not self.withdrawn
        ):  # the brad point's spur grips the wood when pushed in square
            if self.pin is None and on_wood and self.push >= BITE_PUSH and self.tilt <= WALK_ANGLE:
                self.pin = on_plane(tip, axis, TOP_FACE)
            elif (
                self.pin is not None and not on_wood
            ):  # lifted off before the line locked: it lets go
                self.pin = None
        bound = self.locked and self.off_line > BIND_ANGLE
        if self.locked and self.off_line > LEVER_ANGLE:
            self.levered = True
        cutting = (
            self.speed > 0.0
            and self.push >= BITE_PUSH
            and not (self.withdrawn or self.through or bound)
        )
        if not self.locked:
            cutting = cutting and on_wood and self.tilt <= WALK_ANGLE
        self.feed = FEED_PER_N * (self.push - BITE_PUSH) * self.speed if cutting else 0.0
        self.depth += self.feed * dt
        per_turn = self.feed / (LOADED_RPM / 60.0 * self.speed) if self.feed > 0.0 else 0.0
        if self.feed > 0.0:
            self.torque = TORQUE_IDLE + TORQUE_PER_FEED * per_turn
        elif bound and self.speed > 0.0:
            self.torque = TORQUE_BIND
        else:
            self.torque = 0.0
        if not self.locked and not self.withdrawn and self.depth >= LOCK_DEPTH:
            self.locked = True
            self.entry = on_plane(tip, axis, TOP_FACE)
            self.line = axis.copy()
            self.pin = None  # the line lock holds the point from here
        if self.locked and not self.through and self.depth >= LUM_D + EXIT_PAST:
            self.through = True
            self.exit = on_plane(tip, axis, BOTTOM_FACE)
            self.depth = SPUR_FREE
        self.deepest = max(self.deepest, min(self.depth, LUM_D))
        if (
            self.locked and float(tip[2]) > TOP_FACE + ON_WOOD
        ):  # the point has come back out of the hole: nothing holds it any more
            self.locked = False
            self.withdrawn = True
            self.pin = None
        near_wood = self.locked or (
            over_board(tip, -AIR_GAP) and float(tip[2]) <= TOP_FACE + AIR_GAP
        )
        self.air_steps = self.air_steps + 1 if self.speed > 0.0 and not near_wood else 0
        if self.air_steps >= AIR_STEPS:
            self.ran_in_air = True
        target = 0.0 if self.withdrawn else self.depth
        writes: list[tuple[str, str, float]] = [
            ("spur", "drive:transX:physics:targetPosition", target)
        ]
        writes += self.lock_writes(drill)
        writes.append(("lock_point", "drive:rotX:physics:maxForce", self.torque))
        self.last_target = target
        return writes

    def lock_writes(self, drill: PartView) -> list[tuple[str, str, float]]:
        """The point's and the line's drives: the point held where the spur pinned it, then both held on the locked
        line at each held point's depth (the joints' y is world y, their z is world x, from the top X); off otherwise."""
        out: list[tuple[str, str, float]] = []
        for joint, along, k in (
            ("lock_point", 0.0, LOCK_K[0]),
            ("lock_upper", LOCK_SPAN, LOCK_K[1]),
        ):
            on = self.locked and self.entry is not None and self.line is not None
            pinned = (
                joint == "lock_point" and not on and not self.withdrawn and self.pin is not None
            )
            if pinned and self.pin is not None:
                out += [
                    (joint, "drive:transY:physics:stiffness", k),
                    (joint, "drive:transZ:physics:stiffness", k),
                    (joint, "drive:transY:physics:damping", LOCK_DAMPING),
                    (joint, "drive:transZ:physics:damping", LOCK_DAMPING),
                    (joint, "drive:transY:physics:targetPosition", float(self.pin[1]) - X_TOP[1]),
                    (joint, "drive:transZ:physics:targetPosition", float(self.pin[0]) - X_TOP[0]),
                ]
                continue
            out += [
                (joint, "drive:transY:physics:stiffness", k if on else 0.0),
                (joint, "drive:transZ:physics:stiffness", k if on else 0.0),
                (joint, "drive:transY:physics:damping", LOCK_DAMPING if on else 0.0),
                (joint, "drive:transZ:physics:damping", LOCK_DAMPING if on else 0.0),
            ]
            if on and self.entry is not None and self.line is not None:
                held = drill.world((TIP_X - along, BIT_Y, 0.0))
                on_line = on_plane(self.entry, self.line, float(held[2]))
                if on_line is not None:
                    out += [
                        (
                            joint,
                            "drive:transY:physics:targetPosition",
                            float(on_line[1]) - X_TOP[1],
                        ),
                        (
                            joint,
                            "drive:transZ:physics:targetPosition",
                            float(on_line[0]) - X_TOP[0],
                        ),
                    ]
        return out


def reset_writes() -> list[tuple[str, str, float]]:
    """A new trial: the spur back at the point, the line lock off, no torque."""
    out: list[tuple[str, str, float]] = [("spur", "drive:transX:physics:targetPosition", 0.0)]
    for joint in ("lock_point", "lock_upper"):
        for axis in ("transY", "transZ"):
            out += [
                (joint, f"drive:{axis}:physics:stiffness", 0.0),
                (joint, f"drive:{axis}:physics:damping", 0.0),
                (joint, f"drive:{axis}:physics:targetPosition", 0.0),
            ]
    out.append(("lock_point", "drive:rotX:physics:maxForce", 0.0))
    return out


def _stage() -> Any:
    return importlib.import_module("omni.usd").get_context().get_stage()


def apply_writes(writes: Sequence[tuple[str, str, float]]) -> None:
    """Make the step's writes on the live stage (only where the simulator runs): drive set points, stiffness, damping
    and force limits."""
    if not writes or not _simulator_present():
        return
    try:
        stage = _stage()
        root = _env_root()
        for joint, attr, value in writes:
            stage.GetPrimAtPath(f"{root}/{JOINTS_FOLDER}/{joint}").GetAttribute(attr).Set(
                float(value)
            )
    except Exception as error:
        _warn_once("writes", f"could not write the hole's joints ({type(error).__name__}: {error})")


# --- the robot's grippers ------------------------------------------------------------------------------------------
PAD_TIP_PAST_TCP = 0.013
PAD_LENGTH = 0.030
PAD_WIDTH = 0.027


@dataclass(frozen=True)
class Gripper:
    """One arm's tool point (TCP), finger opening, approach and closing directions (world)."""

    tcp: np.ndarray
    opening: float
    approach: tuple[float, float, float] = (0.0, 0.0, -1.0)
    closing: tuple[float, float, float] = (0.0, 1.0, 0.0)
    joints: tuple[float, ...] = ()

    def finger_points(self) -> list[np.ndarray]:
        tcp = np.asarray(self.tcp, dtype=float)
        approach, closing = _v(self.approach), _v(self.closing)
        across = np.cross(approach, closing)
        points = [tcp]
        for finger in (-0.5, 0.5):
            for along in (
                PAD_TIP_PAST_TCP,
                PAD_TIP_PAST_TCP - PAD_LENGTH / 2,
                PAD_TIP_PAST_TCP - PAD_LENGTH,
            ):
                for edge in (-0.5, 0.0, 0.5):
                    points.append(
                        tcp
                        + finger * self.opening * closing
                        + along * approach
                        + edge * PAD_WIDTH * across
                    )
        return points


def point_to_box(point: Pt, origin: Pt, rot: np.ndarray, center: Pt, size: Pt) -> float:
    """The distance from a point to a turned box (its centre and size in a body frame at ``origin``, ``rot``)."""
    local = rot.T @ (_v(point) - _v(origin)) - _v(center)
    gap = np.maximum(np.abs(local) - _v(size) / 2, 0.0)
    return float(np.linalg.norm(gap))


def drill_parts(parts: Mapping[str, PartView]) -> list[tuple[np.ndarray, np.ndarray, Vec3, Vec3]]:
    """The drill's shapes and the trigger's and spur's, each as (origin, rotation, centre, size)."""
    out = []
    for name in FREE:
        view = parts[name]
        for center, size in shapes(name):
            out.append((view.origin, view.rot, center, size))
    return out


HAND_CLEAR = 0.050  # every finger pad point at least this far from the drill's parts (its shapes, not its outline)
HOLD_REACH = 0.035  # a gripper holds the drill: its TCP this near the handle's pinch point ...
HOLD_OPEN = (
    HANDLE_SIZE[0] + TRIGGER_SIZE[0] + 0.005
)  # ... its fingers closed to the handle's depth (+5 mm)


def hands_clear(parts: Mapping[str, PartView], grippers: Sequence[Gripper]) -> bool:
    boxes = drill_parts(parts)
    return all(
        point_to_box(p, o, r, c, s) >= HAND_CLEAR
        for g in grippers
        for p in g.finger_points()
        for o, r, c, s in boxes
    )


def held(parts: Mapping[str, PartView], grippers: Sequence[Gripper] | None) -> bool:
    """Whether a gripper holds the drill (unknown arms count as holding: a fall is then not called)."""
    if grippers is None:
        return True
    pinch = parts[DRILL].world(GRIP)
    return any(
        g.opening <= HOLD_OPEN
        and float(np.linalg.norm(np.asarray(g.tcp, dtype=float) - pinch)) <= HOLD_REACH
        for g in grippers
    )


# --- the trial's preconditions --------------------------------------------------------------------------------------
GEOMETRY_TOL = 0.0005
SETTLE = 0.001
_STATE_FIELDS = ("pos", "quat_xyzw", "lin_vel", "ang_vel", "low", "high")
QUAT_NORM = (0.99, 1.01)


def all_finite(objects: Mapping[str, Any], grippers: Sequence[Gripper] | None) -> bool:
    try:
        values = [
            np.asarray(getattr(objects[n], f), dtype=float) for n in SPECS for f in _STATE_FIELDS
        ]
        for g in grippers or ():
            values += [
                np.asarray(v, dtype=float)
                for v in (g.joints, g.tcp, g.opening, g.approach, g.closing)
            ]
        if not all(bool(np.all(np.isfinite(v))) for v in values):
            return False
        norms = [
            float(np.linalg.norm(np.asarray(objects[n].quat_xyzw, dtype=float))) for n in SPECS
        ]
        return all(QUAT_NORM[0] <= n <= QUAT_NORM[1] for n in norms)
    except (LookupError, AttributeError, TypeError, ValueError, ArithmeticError):
        return False


def _mm(v: Any) -> str:
    return "(" + ", ".join(f"{1000 * float(x):.2f}" for x in np.ravel(v)) + ")"


def geometry_problems(objects: Mapping[str, Any]) -> list[str]:
    """A trial's first step: every object's reported outline against the shapes built, each body where it was put (or
    settled at most SETTLE onto its support) and turned as it was put, the trigger and the spur at rest."""
    problems = []
    try:
        for name, spec in SPECS.items():
            st = objects[name]
            rot = rotation(st.quat_xyzw)
            low, high = local_outline(name)
            built = np.abs(rot) @ (high - low)
            seen = np.asarray(st.high, dtype=float) - np.asarray(st.low, dtype=float)
            if float(np.max(np.abs(seen - built))) > GEOMETRY_TOL:
                problems.append(f"{name}'s outline is {_mm(seen)} mm, built {_mm(built)} mm")
            middle = (np.asarray(st.low, dtype=float) + np.asarray(st.high, dtype=float)) / 2
            d = middle - rot @ ((low + high) / 2) - _v(spec.pos)
            if not (
                math.hypot(d[0], d[1]) <= GEOMETRY_TOL
                and -SETTLE - GEOMETRY_TOL <= d[2] <= GEOMETRY_TOL
            ):
                problems.append(f"{name} starts {_mm(d)} mm from where it was put")
            if angle_between(
                rot @ _v((1.0, 0.0, 0.0)), rz(spec.yaw) @ _v((1.0, 0.0, 0.0))
            ) > math.radians(1.0):
                problems.append(f"{name} starts turned from how it was put")
        parts = views(objects)
        if trigger_travel(parts[DRILL], parts[TRIGGER]) > GEOMETRY_TOL:
            problems.append("the trigger starts pressed")
        if abs(spur_offset(parts[DRILL], parts[SPUR])) > GEOMETRY_TOL:
            problems.append("the spur starts off the bit's point")
    except (LookupError, AttributeError, TypeError, ValueError, ArithmeticError) as error:
        problems.append(f"can't read the parts ({type(error).__name__}: {error})")
    return problems


# --- the rules -------------------------------------------------------------------------------------------------------
ENTRY_TOL, ENTRY_LOOSE = 0.003, 0.006  # the hole's entry from the top X
EXIT_TOL, EXIT_MID, EXIT_LOOSE = 0.004, 0.008, 0.012  # its exit from the bottom X
DOWN_TOL = 0.005  # the drill set down: its lowest point within this of the bench
HOLD_STEPS = 20


def lowest_point(view: PartView) -> float:
    """The lowest point of a body's shapes (each a turned box), world z."""
    lows = []
    for center, size in shapes(view.name):
        corners = np.array(
            [[sx, sy, sz] for sx in (-0.5, 0.5) for sy in (-0.5, 0.5) for sz in (-0.5, 0.5)]
        )
        pts = (corners * _v(size) + _v(center)) @ view.rot.T + view.origin
        lows.append(float(np.min(pts[:, 2])))
    return min(lows)


def on_bench(parts: Mapping[str, PartView]) -> bool:
    """The drill set down on the bench: its lowest point at the bench, and its outline clear of the board, the
    risers and the jaws (it rests on the bench itself), its trigger and spur with it."""
    drill = parts[DRILL]
    if abs(lowest_point(drill) - TOP) > DOWN_TOL:
        return False
    block_low = _v((BOARD_X - LUM_D / 2, MARK_Y - BOARD_LEN / 2))
    block_high = _v((BOARD_X + LUM_D / 2, MARK_Y + BOARD_LEN / 2))
    over = bool(np.all(drill.high[:2] > block_low) and np.all(drill.low[:2] < block_high))
    return not over


@dataclass(frozen=True)
class State:
    """One step's facts. ``clear`` is None when the arms' state is unknown."""

    lost: tuple[str, ...]
    through: bool
    entry: float  # m from the top X (inf: no hole)
    exit: float  # m from the bottom X (inf: not through)
    deepest: float  # m
    on_bench: bool
    trigger_off: bool
    still: bool
    clear: bool | None
    falling: bool

    @property
    def finished(self) -> bool:
        return self.on_bench and self.trigger_off and self.clear is True

    @property
    def full(self) -> bool:
        return (
            not self.lost
            and self.through
            and self.entry <= ENTRY_TOL
            and self.exit <= EXIT_TOL
            and self.finished
            and self.still
        )


def state(parts: Mapping[str, PartView], grippers: Sequence[Gripper] | None, hole: Hole) -> State:
    return State(
        lost=tuple(n for n in FREE if parts[n].lost),
        through=hole.through,
        entry=hole.entry_error(),
        exit=hole.exit_error(),
        deepest=hole.deepest,
        on_bench=on_bench(parts),
        trigger_off=trigger_speed(trigger_travel(parts[DRILL], parts[TRIGGER])) == 0.0,
        still=all(parts[n].still for n in FREE),
        clear=None if grippers is None else hands_clear(parts, grippers),
        falling=parts[DRILL].vz < -DROP_SPEED and not held(parts, grippers),
    )


UNREADABLE = State((), False, math.inf, math.inf, 0.0, False, False, False, None, False)
LADDER = (0.25, 0.5, 0.75, 1.0)


@dataclass
class Ledger:
    steps: int = 0
    full_steps: int = 0
    lost: bool = False
    dropped: bool = False
    void: str | None = None

    def observe(self, now: State) -> None:
        self.steps += 1
        self.lost = self.lost or bool(now.lost)
        self.dropped = self.dropped or now.falling
        self.full_steps = self.full_steps + 1 if now.full and not self.lost else 0

    def spoil(self, reason: str) -> None:
        if self.void is None:
            self.void = reason


@dataclass(frozen=True)
class Grade:
    state: State
    full_steps: int
    score: float
    void: str | None = None
    built: bool = False
    complete: bool = False

    @property
    def counts(self) -> bool:
        return self.built and self.void is None


def grade(
    ledger: Ledger, hole: Hole, now: State, built: bool, hold_steps: int = HOLD_STEPS
) -> Grade:
    """The ladder (README.md). 0 unless the scene's parts were built, or if the trial is void, a part has left the
    table, the drill fell, the bit was levered more than 6 degrees off its line in the hole, the drill ran with its
    point away from the wood, or the drill isn't set down on the bench with its trigger off and every finger 5 cm from
    it. Otherwise:
    1.0 through, entry within 3 mm of the top X, exit within 4 mm of the bottom X, everything still, held for 1 s;
    0.75 through, entry within 3 mm, exit within 8 mm;
    0.5 through, entry within 6 mm, exit within 12 mm;
    0.25 a hole at least 20 mm deep, entry within 6 mm."""
    spoiled = ledger.lost or ledger.dropped or hole.levered or hole.ran_in_air
    complete = now.full and not spoiled
    if not built or ledger.void is not None or spoiled or now.lost or not now.finished:
        score = 0.0
    elif now.through and now.entry <= ENTRY_TOL and now.exit <= EXIT_MID:
        full = now.exit <= EXIT_TOL and complete and ledger.full_steps >= hold_steps
        score = LADDER[3] if full else LADDER[2]
    elif now.through and now.entry <= ENTRY_LOOSE and now.exit <= EXIT_LOOSE:
        score = LADDER[1]
    elif now.deepest >= DEPTH_PART and now.entry <= ENTRY_LOOSE:
        score = LADDER[0]
    else:
        score = 0.0
    return Grade(now, ledger.full_steps, score, ledger.void, built, complete)


# --- episodes and the robot's state ------------------------------------------------------------------------------------
_LEDGER = Ledger()
_HOLE = Hole()
_GRADE: Grade | None = None
_last_objects: object = None
_embodiment: Any = None
_last_clock: float | None = None
_hooked = False
_warned: set[str] = set()
_SERVO: "Servo | None" = None  # the demo's watching, once a demo plan is built (it only reads)
_ARMS = (("left_robot", "left"), ("right_robot", "right"))
RESET_TAG = "drill_hole_mark_to_mark_resets"


def _warn_once(key: str, message: str) -> None:
    """Warn once per key. Never raises."""
    if key not in _warned:
        _warned.add(key)
        with contextlib.suppress(Exception):
            warnings.warn(f"drill_hole_mark_to_mark: {message}", RuntimeWarning, stacklevel=3)


def new_episode() -> None:
    global _LEDGER, _HOLE, _GRADE, _last_objects, _last_clock
    _LEDGER = Ledger()
    _HOLE = Hole()
    _GRADE = None
    _last_objects = None
    _last_clock = None
    _warned.difference_update(("robot", "void"))
    apply_writes(reset_writes())
    if _SERVO is not None:
        with contextlib.suppress(Exception):  # the demo's watching never stops a reset
            _SERVO.restart()


def _calling_embodiment() -> Any:
    """The embodiment whose step is calling us, found by what it has (an episode clock, a task, a reset)."""
    frame: FrameType | None = sys._getframe(2)
    while frame is not None:
        candidate = frame.f_locals.get("self")
        if (
            candidate is not None
            and hasattr(candidate, "_t")
            and hasattr(candidate, "task")
            and callable(getattr(candidate, "reset", None))
        ):
            return candidate
        frame = frame.f_back
    return None


def _reset_callbacks(embodiment: Any) -> list[Callable[[], None]] | None:
    current = embodiment.reset
    callbacks = getattr(current, RESET_TAG, None)
    if isinstance(callbacks, list):
        return callbacks
    hooked: list[Callable[[], None]] = []
    original = current

    def reset(*args: Any, **kwargs: Any) -> Any:
        for callback in tuple(hooked):
            callback()
        return original(*args, **kwargs)

    setattr(reset, RESET_TAG, hooked)
    try:
        embodiment.reset = reset
    except (AttributeError, TypeError):
        return None
    return hooked


def _hook_resets() -> None:
    """Task files get no reset hook, so hook the embodiment's reset once: every new trial starts a fresh ledger and
    hole, and puts the hole's joints back."""
    global _hooked, _embodiment
    if _hooked:
        return
    embodiment = _calling_embodiment()
    if embodiment is not None:
        callbacks = _reset_callbacks(embodiment)
        if callbacks is not None:
            if new_episode not in callbacks:
                callbacks.append(new_episode)
            _embodiment = embodiment
            _hooked = True
            return
    _warn_once(
        "reset",
        "could not hook the embodiment's reset, so the arms' state is unknown: every trial scores 0",
    )


def _restart_if_clock_went_back() -> None:
    global _last_clock
    if _embodiment is None:
        return
    try:
        clock = float(_embodiment._t)
    except (AttributeError, TypeError, ValueError):
        return
    if _last_clock is not None and clock < _last_clock:
        new_episode()
    _last_clock = clock


def _control_hz() -> float:
    try:
        hz = float(_embodiment.info.control_hz)
        return hz if math.isfinite(hz) and hz > 0 else 20.0
    except (LookupError, AttributeError, TypeError, ValueError, ArithmeticError):
        return 20.0


def _numpy(values: Any) -> np.ndarray:
    for step in ("detach", "cpu", "numpy"):
        if hasattr(values, step):
            values = getattr(values, step)()
    return np.asarray(values, dtype=float)


def grippers_of(embodiment: Any) -> list[Gripper]:
    ik = importlib.import_module("yam_arena.ik")
    station = importlib.import_module("yam_arena.station")
    out = []
    for name, side in _ARMS:
        arm_ids, finger_ids = embodiment._ids[name]
        joints = embodiment._base.scene[name].data.joint_pos
        q = _numpy(getattr(joints, "torch", joints)[0])
        tcp, rot = ik.tcp_pose(q[list(arm_ids)], station.BASES[side])
        rot = np.asarray(rot, dtype=float)
        out.append(
            Gripper(
                np.asarray(tcp, dtype=float),
                float(-2.0 * q[list(finger_ids)].mean()),
                approach=_t(rot[:, 2]),
                closing=_t(rot[:, 0]),
                joints=tuple(float(v) for v in q),
            )
        )
    return out


def _robot() -> list[Gripper] | None:
    if _embodiment is None:
        return None
    try:
        return grippers_of(_embodiment)
    except Exception as error:
        _warn_once(
            "robot",
            f"can't read the arms' state ({type(error).__name__}: {error}); each such step scores 0",
        )
        return None


def _update(objects: Mapping[str, Any]) -> Grade:
    """Advance the trial once per control step (success() and score() share the step's state dict)."""
    global _last_objects, _GRADE
    if objects is _last_objects and _GRADE is not None:
        return _GRADE
    try:
        _hook_resets()
    except Exception as error:
        _warn_once("hook", f"could not look for the embodiment ({type(error).__name__}: {error})")
    _restart_if_clock_went_back()
    grippers = _robot()
    if not all_finite(objects, grippers):
        _LEDGER.spoil(
            "a non-finite (or degenerate) position, orientation, velocity, outline or joint reading"
        )
    elif _LEDGER.steps == 0:
        problems = geometry_problems(objects)
        if problems:
            _LEDGER.spoil("the parts are not as built: " + "; ".join(problems))
    if _LEDGER.void is not None:
        _warn_once("void", f"this trial scores 0: {_LEDGER.void}")
    built = parts_built()
    if not built:
        _warn_once(
            "built", "the scene's parts were not built (see earlier warnings): every trial scores 0"
        )
    try:
        parts = views(objects)
        apply_writes(_HOLE.step(parts, 1.0 / _control_hz()))
        now = state(parts, grippers, _HOLE)
    except (LookupError, AttributeError, TypeError, ValueError, ArithmeticError) as error:
        _LEDGER.spoil(f"can't read the parts ({type(error).__name__}: {error})")
        _warn_once("void", f"this trial scores 0: {_LEDGER.void}")
        now = UNREADABLE
    _LEDGER.observe(now)
    _GRADE = grade(_LEDGER, _HOLE, now, built, max(1, round(_control_hz())))
    _last_objects = objects
    return _GRADE


def score(objects: Mapping[str, Any]) -> float:
    """1.0: a hole straight through the board, in at the top X and out at the bottom X, the drill set down off; see
    README.md."""
    return _update(objects).score


def success(objects: Mapping[str, Any]) -> bool:
    grade_now = _update(objects)
    return grade_now.complete and grade_now.counts


# --- the scripted demo -------------------------------------------------------------------------------------------------
# The right arm takes the drill where it lies, pinching its handle front to back from above (one pad on the trigger,
# lightly, so it stays off), lifts it, carries it on its side to the board and turns it bit-down there (the grip pitches
# 90 degrees to point at the board). Then, watching the drill (below), it lines the point up over the top X with the bit
# square and sets it down. The left arm comes in beside the drill and pinches the side of its motor housing; both
# push, the left hand along the bit and the right hand a smaller share at the handle; the right hand squeezes the
# trigger, and both follow the feed down, easing the push for the last few millimetres. Once the bit is through, the
# right hand lets the trigger go, the left hand eases off, lets go and moves away, and the right hand draws the bit back
# out along its hole and lays the drill back down where it was.
class Grip(float):
    """A Waypoint yaw that carries a full tool rotation (3x3, columns: closing axis, across, approach)."""

    rot: np.ndarray

    def __new__(cls, rot: np.ndarray) -> "Grip":
        obj = super().__new__(cls, 0.0)
        obj.rot = np.asarray(rot, dtype=float)
        return obj


GRIP_TAG = "drill_hole_mark_to_mark_grip"


def install_grip_rotation() -> None:
    """Let ArmPlan read a Grip's rotation (the official grip_rotation only turns a straight-down grip about z)."""
    demo_mod: Any = importlib.import_module("yam_arena.demo")
    original = demo_mod.grip_rotation
    if getattr(original, GRIP_TAG, None) is not None:
        return

    def grip_rotation(yaw: float = 0.0) -> np.ndarray:
        rot = getattr(yaw, "rot", None)
        return np.array(rot, dtype=float) if rot is not None else original(yaw)

    setattr(grip_rotation, GRIP_TAG, True)
    grip_rotation.__wrapped__ = original  # type: ignore[attr-defined]
    demo_mod.grip_rotation = grip_rotation


def rx(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


R_PICK = np.diag(
    [1.0, -1.0, -1.0]
)  # straight down, the fingers closing along x (across the handle, front to back)
R_DRILL = np.column_stack(
    [(0.0, 0.0, 1.0), (0.0, -1.0, 0.0), (1.0, 0.0, 0.0)]
)  # R_PICK pitched to point along +x
R_REL = R_PICK.T @ rz(DRILL_YAW)  # the drill's frame in the TCP's, as picked
FINGER_MAX = -FINGER_OPEN * 2  # fingers fully open: about 94 mm apart
FINGER_K = 5000.0  # N/m: each finger's position stiffness (the official gripper gains)
WRIST_K = 10.0  # N m/rad: the wrist joints' stiffness (the official gains)
HANDLE_DEPTH = HANDLE_SIZE[0] + TRIGGER_SIZE[0]  # across the handle and the trigger at rest: 50 mm
OPEN_WIDE = 0.070
CARRY_OPEN = (
    HANDLE_DEPTH - 0.0055
)  # on its side: pinched about 14 N a side, 2.5x the weight on friction, under the trigger's 15 N preload
BITDOWN_OPEN = (
    HANDLE_DEPTH - 0.0035
)  # bit-down: about 9 N (the weight rests on the trigger's pad; it stays off)
# Squeezed for drilling: with the trigger bottomed the handle is 44 mm across, so RUN_OPEN puts FINGER_K x (44 - 30.8)
# mm = 66 N on the two pads together. The right hand pushes the whole push. Its grip sits a few degrees off the bit,
# so much of the push goes through its pads' friction rather than their normal loads; the README ("The push") gives
# the pad loads measured in the simulation.
RUN_OPEN = 0.0308
PUSH = 30.0  # N: the push while feeding: 0.084 mm a turn, through in about 72 s at full trigger
EASE_PUSH = 24.0  # N: eased for the last EASE_DEPTH, so the breakthrough lunge stays shorter
EASE_DEPTH = 0.008
WEIGHT = TOOL_MASS * G
LEVER = TIP_X - GRIP[0]  # the point below the pinch, along the bit
APPROACH_GAP = 0.010  # the point this far over the X once turned bit-down (before the sag)
CARRY_HEIGHT = (
    0.195  # m above the bench: the TCP carrying the drill on its side, 18 mm over the board's top
)
TURN_FROM = 0.10  # m: it is turned bit-down while it moves this far forward onto the drilling spot
BACK_OFF = TURN_FROM  # m: drawn out, it is moved this far back toward the robot, bit-down, before it is turned
HALF_DROP = 0.10  # m: it is lowered this far while turned half onto its side (the arm can't reach it higher)
# Put down close in front of the board: lying on its side where it was turned, it is moved back at that height to
# over SET_DOWN_AT (the TCP's x, y; for the demo 55 mm further back toward the robot) and lowered straight down onto
# the bench there, so its housing ends at least 25 mm in front of the board's supports (the risers' and the clamp
# jaws' near faces) and its bit points back toward the robot, clear of the board and the hole's exit. Carrying it back to its start failed in an earlier version: over a
# 4 s carry the grip turned about 10 degrees, the drill turned in the pads, its trigger was pressed on and it slipped
# out.
SET_DOWN_AT = (0.198, -0.103)
SET_BACK_S, SET_DOWN_S = 2.0, 4.0  # s: back over the spot, then down to the bench, slowly

# The left hand. It braces the drill: it pinches the side of the motor housing, nearly level, closing across it along x
# from the left arm's side, and follows the drill down without pushing, as a person's second hand steadies a drill. Its
# target follows the housing and the bit's tilt, so it holds the drill against twisting and leaning without loading it.
LEFT_GRASP = (0.052, BIT_Y, 0.0)  # on the housing's axis, 17 mm behind its front (drill frame)
LEFT_REACH = -0.004  # m: the TCP this far past the housing's axis (short: clear of the right hand)
LEFT_PITCH = math.radians(12.0)  # the left grip's approach points this far below level ...
LEFT_HEADING = -math.pi / 2  # ... along -y, and it closes along x
LEFT_PINCH = 30.0  # N a pad on the housing (the left hand only steadies the drill)
LEFT_PINCH_OPEN = 2 * HEAD[1] - 2 * LEFT_PINCH / FINGER_K  # commanded on the 60 mm housing: 48 mm
LEFT_OPEN = 0.085  # open round the housing: 12.5 mm clear a side
LEFT_BACK = 0.080  # m: its stand-off, level, before it closes in and after it lets go
PIN_PUSH = 21.0  # N: the push is held here, just over the bite, until the spur pins the point (it can't slide)
L_FOLLOW, L_BRACE, L_RELEASE = "left follow", "left brace", "left release"
BRACE_IN_S, PINCH_S, SETTLE_S, LET_GO_S, AWAY_S = 2.0, 1.0, 1.0, 0.6, 1.5
_L_APPROACH = np.array(
    [
        math.cos(LEFT_PITCH) * math.cos(LEFT_HEADING),
        math.cos(LEFT_PITCH) * math.sin(LEFT_HEADING),
        -math.sin(LEFT_PITCH),
    ]
)
_L_CLOSING = np.array([math.sin(LEFT_HEADING), -math.cos(LEFT_HEADING), 0.0])
R_LEFT = np.column_stack([_L_CLOSING, np.cross(_L_APPROACH, _L_CLOSING), _L_APPROACH])
R_LEFT_PARK = np.diag([-1.0, 1.0, -1.0])  # parked straight down, closing along -x as on the housing

# The arm's give with the drill held bit-down and the grip level at the drilling spot, measured on the official stack in
# traced runs of the earlier, open-loop demo (the official arm's gains under the drill's 0.85 kg and the push). The
# watching below corrects whatever is left, so these only set where the arm is first sent: at the landing (hanging),
# the grip's closing axis leans GIVE_HANG past its command and the TCP sits SAG_XYZ off it; pushing GIVE_PUSH_N on the
# wood, the lean eases to GIVE_PUSHED.
SAG = 0.068  # m: hanging bit-down, the TCP sat this far below its command ...
SAG_X, SAG_Y = (-0.0062, 0.0075)  # ... and this far along x and y at the landing
ARM_K_V = WEIGHT / SAG  # N/m: so the arm's vertical springiness there is about 122 N/m
SAG_LYING = 0.020  # m: carried on its side (the straight-down pinch), the TCP hung this far low
GIVE_HANG = (
    math.radians(11.3),
    math.radians(12.5),
)  # the closing axis's lean toward +x and +y, hanging
GIVE_PUSHED = (math.radians(9.1), math.radians(10.6))  # ... and pushing GIVE_PUSH_N on the wood
GIVE_PUSH_N = 17.0


def ry(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def give(push: float) -> tuple[float, float]:
    """The grip's lean (toward +x, toward +y; rad) at a push on the wood, in proportion between the measured two."""
    s = push / GIVE_PUSH_N
    return (
        GIVE_HANG[0] + (GIVE_PUSHED[0] - GIVE_HANG[0]) * s,
        GIVE_HANG[1] + (GIVE_PUSHED[1] - GIVE_HANG[1]) * s,
    )


def lean_rot(lean: tuple[float, float]) -> np.ndarray:
    """The turn that leans a vertical closing axis toward +x by lean[0] and toward +y by lean[1] (rad)."""
    return np.asarray(ry(lean[0]) @ rx(-lean[1]), dtype=float)


_AT_PUSH = give(PUSH)  # the right hand pushes the whole push
LEAN_TRIM = (
    -_AT_PUSH[0],
    -_AT_PUSH[1],
)  # the grip leans back by the give at that load: square once pushed
PLACE_TRIM = (0.0, 0.0, 0.0)  # a further correction of where the point is first sent


def _at(p: Pt) -> Vec3:
    return (float(p[0]), float(p[1]), float(p[2]))


def _finger(opening: float) -> float:
    return float(-min(max(opening, 0.0), FINGER_MAX) / 2)


def drilling_rot(lean: tuple[float, float] = LEAN_TRIM) -> np.ndarray:
    return np.asarray(lean_rot(lean) @ R_DRILL, dtype=float)


def tcp_for_point(point: Pt, rot: np.ndarray) -> np.ndarray:
    """Where the TCP is first sent for the bit's point to land on ``point`` with the grip commanded ``rot``: hanging,
    the grip turns by the measured give (GIVE_HANG) and the TCP sits SAG_X, SAG_Y off (its height is the kinematic one:
    the approach adds the vertical sag)."""
    hanging = lean_rot(GIVE_HANG) @ rot
    offset = hanging @ R_REL @ (_v(TIP_LOCAL) - _v(GRIP))
    return np.asarray(_v(point) - offset - _v((SAG_X, SAG_Y, 0.0)) + _v(PLACE_TRIM), dtype=float)


def feed_speed(push: float) -> float:
    """The feed at full trigger for a push (m/s)."""
    return FEED_PER_N * max(0.0, push - BITE_PUSH)


def feed_time(push: float = PUSH, ease_push: float = EASE_PUSH) -> float:
    """Seconds to go through at full trigger: ``push`` to the last EASE_DEPTH, ``ease_push`` from there to the exit."""
    return (LUM_D - EASE_DEPTH) / max(feed_speed(push), 1e-6) + (EASE_DEPTH + EXIT_PAST) / max(
        feed_speed(ease_push), 1e-6
    )


# --- watching the drill (a scripted demo may read the simulator's state: object poses, joint states, latch flags) -----
# Open loop, the arm's give under the 0.85 kg drill held bit-down (60-80 mm of sag and an 8-13 degree lean, different in
# every pose) put the point 9-36 mm off the board's 38 mm edge in four traced runs. So while the drill is bit-down the
# demo watches it, as a person watches the bit: every control step it reads the drill's pose (where the bit's point is,
# which way the bit points), the push on the point (its spur's give) and the hole's state (pinned, locked and its line,
# depth, through), and corrects where the arm is sent by a share of the remaining error. The correction is a turn of
# the arm's target about the aim point (squaring the bit) and a shift (the point over the X; the push held; the height).
# It changes only where the arm is sent: it never moves, sets or reads back into a part, and the task's rules and
# mechanics are the same for any policy.
SERVO_GAIN = 0.15  # the share of the remaining error corrected each control step
PUSH_GAIN = 0.15  # ... of the push's error, through PUSH_STIFFNESS
PUSH_STIFFNESS = 400.0  # N/m: the push's growth as the arm is sent lower (traced: about 420)
TOUCH_PUSH = 3.0  # N: the point rests on the X under the bite while the bit is squared
TOUCHED = 1.0  # N: a push this large says the point is on the wood
HOVER = 0.015  # m: the point held this far over the X while it is lined up
CLEAR = 0.040  # m: drawn out until the point is this far over the top edge, and moved back at that height
DESCENT = 0.015  # m/s: lowered this fast until the point touches
STEP_TURN = math.radians(0.5)  # at most this much turn, shift and rise per control step
STEP_SHIFT = 0.002
STEP_RISE = 0.002
# Until the spur pins the point, the push grows slowly: at most this much lower a step (about 0.3 N a step at the
# arm's 650 N/m). Pushing through the handle, 55 mm off the bit, turns the drill about the arm's soft wrist, and its
# point, 222 mm below the pinch, swings with it: in an earlier version a full-rate ramp leaned the bit 2.8 degrees and
# slid the point 11 mm at 9-19 N, under the 20 N bite. Slowly, with the point slid back toward the X until the spur
# pins it, it ends within about 3 mm of the X (README).
STEP_RISE_PIN = 0.0005
MAX_TURN = math.radians(25.0)  # the correction never grows past these
MAX_SHIFT = 0.060
MAX_RISE = 0.300
WATCH_TILT = math.radians(45.0)  # it watches only while the bit points within this of straight down
LINE_UP, TOUCH, DRILLING, DRAW_OUT = "line up", "touch", "drill", "draw out"
LINE_UP_S, TOUCH_S, PUSH_S, DRAW_S = 5.0, 2.5, 4.0, 5.5
BACK_S, HALF_TURN_S = 2.5, 2.5
FEED_MARGIN = (
    1.35  # the feed's window over its time at full trigger (the sim's trigger has read 5.2 of 6 mm)
)
BRACE_LABEL = "hold the point while the left hand takes the housing"
FREE_LABEL = "hold it while the left hand lets go"


def _turn(vec: np.ndarray) -> np.ndarray:
    """The rotation by rotation vector ``vec`` (rad)."""
    angle = float(np.linalg.norm(vec))
    if angle < 1e-12:
        return np.eye(3)
    k = np.asarray(vec, dtype=float) / angle
    cross = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.asarray(
        np.eye(3) + math.sin(angle) * cross + (1.0 - math.cos(angle)) * (cross @ cross), dtype=float
    )


def turned_by(rot: np.ndarray) -> float:
    """How far a rotation turns (rad)."""
    return math.acos(max(-1.0, min(1.0, (float(np.trace(rot)) - 1.0) / 2.0)))


def _rotvec(rot: np.ndarray) -> np.ndarray:
    """The rotation vector of ``rot`` (rad; turns under 180 degrees)."""
    angle = turned_by(rot)
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
    return np.asarray(axis / (2.0 * math.sin(angle)) * angle, dtype=float)


def halfway(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    """The rotation half way from ``start`` to ``end`` (the shorter way)."""
    return np.asarray(_turn(_rotvec(end @ start.T) / 2.0) @ start, dtype=float)


def _clip(vec: np.ndarray, most: float) -> np.ndarray:
    n = float(np.linalg.norm(vec))
    return vec if n <= most else np.asarray(vec * (most / n), dtype=float)


def tilt_of(axis: Pt) -> np.ndarray:
    """The smallest turn that takes straight down onto ``axis``."""
    a = _v(axis) / float(np.linalg.norm(_v(axis)))
    cross = np.cross(_v(DOWN), a)
    n = float(np.linalg.norm(cross))
    if n < 1e-12:
        return np.eye(3)
    return _turn(cross / n * math.atan2(n, float(_v(DOWN) @ a)))


def right_share(gripper: Gripper, fingers: tuple[float, float]) -> float:
    """The difference of the right hand's two pads' normal loads (N, upper minus lower), from its fingers' positions.
    Each finger is a FINGER_K position drive with the same target, so the loads differ by FINGER_K times the difference
    of the positions. The first finger's pad is on the gripper's -closing side, the second's on its +closing side
    (official URDF). It is the hand's push only while the closing axis is along the bit (a tilted grip also pushes by
    its pads' friction)."""
    q1, q2 = float(fingers[0]), float(fingers[1])
    lower, upper = (q1, q2) if float(_v(gripper.closing)[2]) > 0.0 else (q2, q1)
    return FINGER_K * (lower - upper)


def _arm_reading(embodiment: Any, name: str) -> np.ndarray:
    arm_ids, finger_ids = embodiment._ids[name]
    joints = embodiment._base.scene[name].data.joint_pos
    q = _numpy(getattr(joints, "torch", joints)[0])
    return np.asarray([*q[list(arm_ids)], *q[list(finger_ids)]], dtype=float)


def _nominal_grasp() -> np.ndarray:
    """Where the left hand takes the housing with the drill square over the X, its point HOVER up."""
    rot = R_DRILL @ R_REL
    origin = _v(X_TOP) + _v((0.0, 0.0, HOVER)) - rot @ _v(TIP_LOCAL)
    return np.asarray(origin + rot @ _v(LEFT_GRASP), dtype=float)


_READ_ERRORS = (LookupError, AttributeError, TypeError, ValueError, ArithmeticError, ImportError)


class Servo:
    """The demo's watching while the drill is bit-down. Every live waypoint's target goes through ``place`` and
    ``orient``: turned by ``turn`` about the aim point, then shifted by ``shift``. ``update`` corrects them once per
    control step, in the waypoint's mode: LINE_UP (point over the aim, bit square, point HOVER over the top edge),
    TOUCH (lowered until the point touches, then TOUCH_PUSH), DRILLING (``push``, ``ease_push`` for the last EASE_DEPTH;
    held once through), DRAW_OUT (the point CLEAR over the top edge). It squares the bit to ``square`` until the hole
    has its line, then to the line. ``squares=False`` leaves the bit's lean alone.

    The left hand's targets go through ``left_place`` and ``left_orient``: its grasp on the housing as the drill is
    seen now (following the bit's tilt). It braces the drill and never pushes: the right hand holds the whole push
    (read from the spur) by its height. L_FOLLOW, L_BRACE and L_RELEASE only name its phases; while it lets go
    (L_RELEASE) the right hand holds its height."""

    def __init__(
        self,
        aim: Pt = X_TOP,
        square: Pt = DOWN,
        push: float = PUSH,
        ease_push: float = EASE_PUSH,
        squares: bool = True,
    ) -> None:
        self.aim = _v((float(aim[0]), float(aim[1]), TOP_FACE))
        self.square = _v(square) / float(np.linalg.norm(_v(square)))
        self.push, self.ease_push, self.squares = float(push), float(ease_push), bool(squares)
        self.phases: list[tuple[float, float, str | None]] = []
        self.left_phases: list[tuple[float, float, str | None]] = []
        self.turn = np.eye(3)
        self.shift = np.zeros(3)
        self.tick = -1
        self.mode: str | None = None
        self.left_mode: str | None = None
        self.aim_error = math.nan  # m: the bit's line on the top edge, from the aim (last read)
        self.square_error = math.nan  # rad: the bit from its wanted line (last read)
        self.right_push = (
            math.nan
        )  # N: the right hand's pads' load difference (back minus trigger), traced
        self.left_seen: tuple[np.ndarray, np.ndarray] | None = None  # grasp point, bit's tilt
        self.want: np.ndarray | None = None  # the line the bit is squared to (last read)
        self.restart()

    def restart(self) -> None:
        """A new trial: no correction yet."""
        self.turn = np.eye(3)
        self.shift = np.zeros(3)
        self.tick = -1
        self.mode = None
        self.left_mode = None
        self.aim_error = math.nan
        self.square_error = math.nan
        self.right_push = math.nan
        self.left_seen = None
        self.want = None

    def follow(self, plan: Sequence[Waypoint]) -> None:
        """Take the right arm's plan's timing: each waypoint's span and its mode (None: not watching)."""
        self.phases = self._spans(plan, LivePos)

    def follow_left(self, plan: Sequence[Waypoint]) -> None:
        """Take the left arm's plan's timing the same way."""
        self.left_phases = self._spans(plan, LeftPos)

    def _spans(self, plan: Sequence[Waypoint], kind: type) -> list[tuple[float, float, str | None]]:
        out: list[tuple[float, float, str | None]] = []
        start = 0.0
        for w in plan:
            pos = w.pos
            ours = isinstance(pos, kind) and getattr(pos, "servo", None) is self
            mode: str | None = getattr(pos, "mode", None) if ours else None
            out.append((start, start + float(w.duration), mode))
            start += float(w.duration)
        return out

    def mode_at(self, t: float) -> str | None:
        for start, end, mode in self.phases:
            if start <= t < end:
                return mode
        return None

    def left_mode_at(self, t: float) -> str | None:
        for start, end, mode in self.left_phases:
            if start <= t < end:
                return mode
        return None

    def place(self, pos: Pt) -> np.ndarray:
        self.update()
        return np.asarray(self.aim + self.turn @ (_v(pos) - self.aim) + self.shift, dtype=float)

    def orient(self, rot: np.ndarray) -> np.ndarray:
        self.update()
        return np.asarray(self.turn @ rot, dtype=float)

    def _left_frame(self) -> tuple[np.ndarray, np.ndarray]:
        if self.left_seen is None:
            return _nominal_grasp(), np.eye(3)
        return self.left_seen

    def left_place(self, back: float) -> np.ndarray:
        self.update()
        grasp, tilt = self._left_frame()
        approach = tilt @ R_LEFT[:, 2]
        away = -approach
        away[2] = 0.0  # its stand-off is level with the grasp, toward the left arm
        away = away / max(float(np.linalg.norm(away)), 1e-9)
        return np.asarray(grasp + LEFT_REACH * approach + float(back) * away, dtype=float)

    def left_orient(self) -> np.ndarray:
        """Its grip's turn: while it braces (L_BRACE) and the bit is being squared, held to the line the bit should be
        on (the right hand's square, or the hole's line once locked), so its wrist resists the drill turning off it;
        otherwise the bit's tilt as seen."""
        self.update()
        _, tilt = self._left_frame()
        if self.left_mode == L_BRACE and self.squares and self.want is not None:
            tilt = tilt_of(self.want)
        return np.asarray(tilt @ R_LEFT, dtype=float)

    def update(self) -> None:
        """Once per control step (the trial's step count moves on): read the last step's state and correct."""
        steps = _LEDGER.steps
        if steps == self.tick:
            return
        self.tick = steps
        objects = _last_objects
        if not isinstance(objects, Mapping):
            return
        hz = _control_hz()
        now = (steps + 1) / hz  # the time the arms' plans are reading now
        self.mode, self.left_mode = self.mode_at(now), self.left_mode_at(now)
        if self.mode is None and self.left_mode is None:
            return
        saved = (self.turn.copy(), self.shift.copy(), self.left_seen, self.want)
        try:
            drill = views(objects)[DRILL]
            self.read_arms()
            if self.mode is not None:
                self.correct(self.mode, drill, _HOLE, 1.0 / hz)
            if self.left_mode is not None:
                self.correct_left(drill)
        except _READ_ERRORS:
            self.turn, self.shift, self.left_seen, self.want = saved  # unreadable: no change
            return
        seen_ok = self.left_seen is None or all(np.all(np.isfinite(v)) for v in self.left_seen)
        want_ok = self.want is None or bool(np.all(np.isfinite(self.want)))
        if not (
            np.all(np.isfinite(self.turn))
            and np.all(np.isfinite(self.shift))
            and seen_ok
            and want_ok
        ):
            self.turn, self.shift, self.left_seen, self.want = saved  # nor a non-finite one

    def read_arms(self) -> None:
        """The right hand's pads' load difference, from its fingers' joints (traced only: nothing steers by it)."""
        self.right_push = math.nan
        embodiment = _embodiment
        if embodiment is None:
            return
        with contextlib.suppress(Exception):  # the demo never stops the run over a reading
            right = grippers_of(embodiment)[1]
            q = _arm_reading(embodiment, "right_robot")
            self.right_push = right_share(right, (float(q[6]), float(q[7])))

    def target_push(self, hole: Hole) -> float:
        return self.ease_push if hole.depth >= LUM_D - EASE_DEPTH else self.push

    def correct_left(self, drill: PartView) -> None:
        """The left hand, in every mode: follow the housing as it is seen now (its grasp point and the bit's tilt)."""
        axis = bit_axis(drill)
        grasp = np.asarray(drill.world(LEFT_GRASP), dtype=float)
        if not (np.all(np.isfinite(axis)) and np.all(np.isfinite(grasp))):
            return
        if angle_between(axis, DOWN) > WATCH_TILT:
            return
        self.left_seen = (grasp, tilt_of(axis))

    def correct(self, mode: str, drill: PartView, hole: Hole, dt: float) -> None:
        tip, axis = bit_tip(drill), bit_axis(drill)
        if angle_between(axis, DOWN) > WATCH_TILT:
            return
        in_hole = hole.locked and hole.line is not None
        want = (
            np.array(hole.line, dtype=float)
            if in_hole and hole.line is not None
            else self.square.copy()
        )  # copies: the servo never holds the hole's own arrays
        self.square_error = angle_between(axis, want)
        self.want = want.copy()
        if self.squares:
            pivot = np.cross(axis, want)
            pivot[2] = 0.0  # square it; never turn it about the vertical
            n = float(np.linalg.norm(pivot))
            if n > 1e-9:
                step = min(SERVO_GAIN * self.square_error, STEP_TURN)
                turned = _turn(pivot / n * step) @ self.turn
                if turned_by(turned) <= MAX_TURN:
                    self.turn = turned
        meet = on_plane(tip, axis, TOP_FACE)
        if meet is not None:
            miss = self.aim[:2] - meet[:2]
            self.aim_error = float(np.linalg.norm(miss))
            free = hole.pin is None and not hole.locked and not hole.withdrawn
            above = float(tip[2]) >= TOP_FACE - ON_WOOD  # never slid sideways into the board's side
            light = (
                hole.push < (TOUCHED if mode == LINE_UP else BITE_PUSH)
            )  # touching lightly, it is slid on (and, pushing, kept on the X until the spur pins it)
            # On the wood, only while the bit is square enough for the spur to bite: a point that skates (a bit
            # leaned past WALK_ANGLE) isn't chased across the board.
            pinnable = mode == LINE_UP or angle_between(axis, DOWN) <= WALK_ANGLE
            if mode in (LINE_UP, TOUCH, DRILLING) and free and light and above and pinnable:
                shifted = self.shift[:2] + _clip(SERVO_GAIN * miss, STEP_SHIFT)
                if float(np.linalg.norm(shifted)) <= MAX_SHIFT:
                    self.shift[:2] = shifted
        most = (
            STEP_RISE_PIN
            if mode == DRILLING and hole.pin is None and not (hole.locked or hole.withdrawn)
            else STEP_RISE
        )
        rise = min(max(self.rise(mode, tip, hole, in_hole, dt), -most), most)
        self.shift[2] = min(max(float(self.shift[2]) + rise, -MAX_RISE), MAX_RISE)

    def rise(self, mode: str, tip: np.ndarray, hole: Hole, in_hole: bool, dt: float) -> float:
        """This step's change of the target's height (m, up positive)."""
        if mode in (LINE_UP, DRAW_OUT):
            gap = HOVER if mode == LINE_UP else CLEAR
            return SERVO_GAIN * (TOP_FACE + gap - float(tip[2]))
        if hole.through or hole.withdrawn:
            return 0.0  # through: held while the trigger is let go
        if mode == DRILLING and self.left_mode == L_RELEASE:
            return 0.0  # held while the left hand lets go
        target = TOUCH_PUSH if mode == TOUCH else self.target_push(hole)
        if mode == DRILLING and hole.pin is None and not hole.locked:
            target = min(target, PIN_PUSH)  # just over the bite until the spur pins the point
        if in_hole or hole.push >= TOUCHED:
            return PUSH_GAIN * (hole.push - target) / PUSH_STIFFNESS
        if over_board(tip):
            return -DESCENT * dt  # lowered until the point touches
        return 0.0

    def report(self) -> dict[str, Any]:
        """The correction now, for traces."""
        return {
            "mode": self.mode,
            "turn_deg": round(math.degrees(turned_by(self.turn)), 3),
            "shift_mm": [round(1000.0 * float(v), 2) for v in self.shift],
            "aim_error_mm": None
            if math.isnan(self.aim_error)
            else round(1000.0 * self.aim_error, 2),
            "square_error_deg": None
            if math.isnan(self.square_error)
            else round(math.degrees(self.square_error), 3),
            "left_mode": self.left_mode,
            "right_pads_n": None if math.isnan(self.right_push) else round(self.right_push, 3),
        }


class LivePos:
    """A waypoint position the servo corrects whenever the arm's plan reads it (the official ArmPlan reads every
    waypoint's position, up to the current one, each control step). ``mode`` is what the servo does while this
    waypoint is the current one (None: it holds its correction)."""

    def __init__(self, servo: Servo, pos: Pt, mode: str | None) -> None:
        self.servo = servo
        self.nominal = _v(pos)
        self.mode = mode

    def value(self) -> np.ndarray:
        return self.servo.place(self.nominal)

    def __array__(self, dtype: Any = None, copy: Any = None) -> np.ndarray:
        return np.asarray(self.value(), dtype=dtype)

    def __len__(self) -> int:
        return 3

    def __getitem__(self, index: Any) -> Any:
        return tuple(float(v) for v in self.value())[index]

    def __iter__(self) -> Iterator[float]:
        return iter(tuple(float(v) for v in self.value()))


class LiveGrip(float):
    """A Waypoint yaw whose tool rotation the servo corrects whenever the arm's plan reads it."""

    servo: Servo
    nominal: np.ndarray

    def __new__(cls, servo: Servo, rot: np.ndarray) -> "LiveGrip":
        obj = super().__new__(cls, 0.0)
        obj.servo = servo
        obj.nominal = np.asarray(rot, dtype=float)
        return obj

    @property
    def rot(self) -> np.ndarray:
        return self.servo.orient(self.nominal)


class LeftPos:
    """A left-hand waypoint position: the servo's grasp on the housing, ``back`` along the grip's approach (its
    stand-off); ``mode`` names the left hand's phase while it is the current one."""

    def __init__(self, servo: Servo, back: float, mode: str | None) -> None:
        self.servo = servo
        self.back = float(back)
        self.mode = mode

    def value(self) -> np.ndarray:
        return self.servo.left_place(self.back)

    def __array__(self, dtype: Any = None, copy: Any = None) -> np.ndarray:
        return np.asarray(self.value(), dtype=dtype)

    def __len__(self) -> int:
        return 3

    def __getitem__(self, index: Any) -> Any:
        return tuple(float(v) for v in self.value())[index]

    def __iter__(self) -> Iterator[float]:
        return iter(tuple(float(v) for v in self.value()))


class LeftGrip(float):
    """A left-hand Waypoint yaw: the servo's grip on the housing (following the bit's tilt)."""

    servo: Servo

    def __new__(cls, servo: Servo) -> "LeftGrip":
        obj = super().__new__(cls, 0.0)
        obj.servo = servo
        return obj

    @property
    def rot(self) -> np.ndarray:
        return self.servo.left_orient()


def drill_plan(
    lean: tuple[float, float] = LEAN_TRIM,
    push: float = PUSH,
    ease_push: float = EASE_PUSH,
    aim: Pt = X_TOP,
    squeeze: float = RUN_OPEN,
    lever: float = 0.0,
    square: Pt = DOWN,
    squares: bool = True,
    brace: bool = True,
) -> list[Waypoint]:
    """The right arm's whole job. ``lean`` is the drilling grip's first lean (from the measured give); the servo then
    aims at ``aim``, squares the bit to ``square`` (if ``squares``) and pushes ``push`` (``ease_push`` at the end).
    ``lever`` turns the grip about its approach axis once the bit is in. With ``brace``, it holds the point while the
    left hand takes the housing, and the drill while the left hand lets go (``brace_plan``)."""
    rot = drilling_rot(lean)
    servo = Servo(aim=aim, square=square, push=push, ease_push=ease_push, squares=squares)
    pick, down = Grip(R_PICK), LiveGrip(servo, rot)
    hover = _v(PICK_TCP) + _v((0.0, 0.0, 0.07))
    lifted = _v((PICK_TCP[0], PICK_TCP[1], TOP + CARRY_HEIGHT))
    contact = tcp_for_point(aim, rot)  # the TCP with the point just on the aim (no sag)
    over_x = contact + _v((0.0, 0.0, APPROACH_GAP + SAG))

    def at(mode: str | None) -> LivePos:
        return LivePos(servo, over_x, mode)

    # On its side (the drill sits steadily in the pinch this way), carried to just in front of the drilling spot; it is
    # turned bit-down while it moves the last TURN_FROM forward, its point clear of the board's near face and top. From
    # there every target is the servo's.
    lying = _v((over_x[0] - TURN_FROM, over_x[1], TOP + CARRY_HEIGHT))
    run, light = _finger(squeeze), _finger(BITDOWN_OPEN)
    plan = [
        Waypoint(2.0, _at(hover), _finger(OPEN_WIDE), "over the drill's handle", pick),
        Waypoint(1.5, _at(PICK_TCP), _finger(OPEN_WIDE), "down round the handle", pick),
        Waypoint(0.8, _at(PICK_TCP), _finger(CARRY_OPEN), "pinch the handle (trigger off)", pick),
        Waypoint(1.5, _at(lifted), _finger(CARRY_OPEN), "lift the drill", pick),
        Waypoint(4.0, _at(lying), _finger(CARRY_OPEN), "carry it on its side to the board", pick),
        Waypoint(4.0, at(None), light, "turn it bit-down over the X", down),
        Waypoint(LINE_UP_S, at(LINE_UP), light, "line the point up over the X", down),
        Waypoint(TOUCH_S, at(TOUCH), light, "set the point on the X", down),
    ]
    if brace:
        plan.append(Waypoint(BRACE_IN_S + PINCH_S, at(TOUCH), light, BRACE_LABEL, down))
    plan += [
        # Squeezed while the point still rests lightly (it turns on the mark without cutting, under the bite), so the
        # firm grip holds the drill while the push builds: in the light grip the push turned it in the pads and its
        # point swung 2.7 mm off the X before it bit (in an earlier version).
        Waypoint(0.6, at(TOUCH), run, "squeeze the trigger", down),
        Waypoint(PUSH_S, at(DRILLING), run, "push", down),
        Waypoint(feed_time(push, ease_push) * FEED_MARGIN, at(DRILLING), run, "feed", down),
    ]
    held = down
    if lever:  # levered, held so until the trigger is let go, then straightened to draw it out
        held = LiveGrip(servo, rx(lever) @ rot)
        plan.append(Waypoint(2.0, at(None), run, "lever the drill", held))
    # Drawn out with its point CLEAR over the top edge, it is moved straight back toward the robot at that height
    # (the servo still holds the point's height), so its point is in front of the board, its supports and the
    # clamp jaws; then it is turned onto its side there, the bit swinging back toward the robot, away from the board
    # (lowered as it turns: its point stays level, well in front of everything, and the arm can't reach higher). It
    # keeps the light bit-down pinch through the turn (a firmer one, with the weight on the trigger's pad, would switch
    # the trigger on) and is pinched firmly only once it lies on its side, the weight off that pad.
    backed = LivePos(servo, over_x - _v((BACK_OFF, 0.0, 0.0)), DRAW_OUT)
    half = LiveGrip(servo, halfway(rot, R_PICK))
    plan.append(Waypoint(0.6, at(DRILLING), light, "let the trigger go", held))
    if brace:
        plan.append(Waypoint(SETTLE_S + LET_GO_S + AWAY_S, at(DRILLING), light, FREE_LABEL, down))
    plan += [
        Waypoint(DRAW_S, at(DRAW_OUT), light, "draw the bit out", down),
        Waypoint(BACK_S, backed, light, "move it back off the board", down),
        Waypoint(
            HALF_TURN_S,
            LivePos(servo, backed.nominal - _v((0.0, 0.0, HALF_DROP)), None),
            light,
            "turn it half onto its side",
            half,
        ),
    ]
    return plan + put_back(lying)


def put_back(lying: Pt) -> list[Waypoint]:
    """From bit-down (or half turned) in front of the board: turn the drill onto its side at ``lying`` in the light
    pinch, pinch it firmly there, move it back at that height over SET_DOWN_AT, lay it on the bench there, close in
    front of the board (10 mm into the bench: it finishes against it), let go and rise straight up."""
    pick = Grip(R_PICK)
    over = _v((SET_DOWN_AT[0], SET_DOWN_AT[1], float(lying[2])))
    laid = _v((SET_DOWN_AT[0], SET_DOWN_AT[1], PICK_TCP[2] - 0.010))
    risen = _v((float(laid[0]), float(laid[1]), TOP + CARRY_HEIGHT))
    return [
        Waypoint(
            3.5,
            _at(lying),
            _finger(BITDOWN_OPEN),
            "turn it onto its side in front of the board",
            pick,
        ),
        Waypoint(0.6, _at(lying), _finger(CARRY_OPEN), "pinch it firmly on its side", pick),
        Waypoint(SET_BACK_S, _at(over), _finger(CARRY_OPEN), "move it back over the bench", pick),
        Waypoint(SET_DOWN_S, _at(laid), _finger(CARRY_OPEN), "lay it on the bench", pick),
        Waypoint(0.6, _at(laid), _finger(OPEN_WIDE), "let go of the drill", pick),
        Waypoint(1.0, _at(risen), _finger(OPEN_WIDE), "rise", pick),
    ]


PARK = {"left": (0.20, 0.30, TOP + 0.15), "right": (0.20, -0.30, TOP + 0.15)}


def _duration(plan: Sequence[Waypoint]) -> float:
    return float(sum(w.duration for w in plan))


def _pad(plan: list[Waypoint], until: float, label: str) -> list[Waypoint]:
    total = _duration(plan)
    if until > total + 1e-9:
        last = plan[-1]
        plan = [*plan, Waypoint(until - total, last.pos, last.finger, label, last.yaw)]
    return plan


def engage(right: list[Waypoint]) -> list[Waypoint]:
    """Make the plan's servo (the one its live waypoints share) the one that watches, on this plan's timing."""
    global _SERVO
    for w in right:
        if isinstance(w.pos, LivePos):
            w.pos.servo.follow(right)
            _SERVO = w.pos.servo
            break
    return right


def _starts(plan: Sequence[Waypoint]) -> dict[str, float]:
    out: dict[str, float] = {}
    t = 0.0
    for w in plan:
        out.setdefault(str(w.label), t)
        t += float(w.duration)
    return out


def brace_plan(right: Sequence[Waypoint]) -> list[Waypoint] | None:
    """The left arm's job, timed to the right arm's plan (None if that plan leaves it no place): wait parked; while
    the right hand lines the point up and sets it, come in beside the housing, open, LEFT_BACK off along the grip's
    approach; move round it while the right hand holds the point, and pinch it; brace the housing while the right hand
    pushes and feeds; once the trigger is let go, hold it a moment while it settles, let go, move back, and park."""
    at = _starts(right)
    if BRACE_LABEL not in at or FREE_LABEL not in at:
        return None
    holder = next(w for w in right if w.label == BRACE_LABEL).pos
    if not isinstance(holder, LivePos):
        return None
    servo = holder.servo
    grip, parked = LeftGrip(servo), Grip(R_LEFT_PARK)
    wide, pinch = _finger(LEFT_OPEN), _finger(LEFT_PINCH_OPEN)
    t_line, t_brace = at["line the point up over the X"], at[BRACE_LABEL]
    t_push, t_free = at["push"], at[FREE_LABEL]
    plan = _pad(
        [Waypoint(2.0, PARK["left"], _finger(OPEN_WIDE), "park (left)", parked)],
        t_line,
        "wait (left)",
    )
    return [
        *plan,
        Waypoint(
            t_brace - t_line,
            LeftPos(servo, LEFT_BACK, L_FOLLOW),
            wide,
            "left hand beside the housing",
            grip,
        ),
        Waypoint(
            BRACE_IN_S, LeftPos(servo, 0.0, L_FOLLOW), wide, "left hand round the housing", grip
        ),
        Waypoint(
            t_push - t_brace - BRACE_IN_S,
            LeftPos(servo, 0.0, L_FOLLOW),
            pinch,
            "pinch the housing",
            grip,
        ),
        Waypoint(t_free - t_push, LeftPos(servo, 0.0, L_BRACE), pinch, "brace the housing", grip),
        Waypoint(
            SETTLE_S,
            LeftPos(servo, 0.0, L_RELEASE),
            pinch,
            "hold the housing while it settles",
            grip,
        ),
        Waypoint(LET_GO_S, LeftPos(servo, 0.0, L_RELEASE), wide, "let go of the housing", grip),
        Waypoint(
            AWAY_S, LeftPos(servo, LEFT_BACK, L_RELEASE), wide, "move the left hand back", grip
        ),
        Waypoint(3.0, PARK["left"], _finger(OPEN_WIDE), "park (left)", parked),
    ]


def engage_left(left: list[Waypoint]) -> list[Waypoint]:
    """Give the left arm's plan's timing to the servo its waypoints share."""
    for w in left:
        if isinstance(w.pos, LeftPos):
            w.pos.servo.follow_left(left)
            break
    return left


def assemble(right: list[Waypoint]) -> dict[str, list[Waypoint]]:
    """The right arm's plan, parked at the end; the left arm's (``brace_plan``, or parked throughout); both held still
    2 s more."""
    right = [*right, Waypoint(1.5, PARK["right"], _finger(OPEN_WIDE), "park (right)", Grip(R_PICK))]
    end = _duration(right) + 2.0
    left = brace_plan(right) or [
        Waypoint(2.0, PARK["left"], _finger(OPEN_WIDE), "park (left)", Grip(R_PICK))
    ]
    return {
        "left": engage_left(_pad(left, end, "hold still")),
        "right": engage(_pad(right, end, "hold still")),
    }


def demo() -> dict[str, list[Waypoint]]:
    install_grip_rotation()
    return assemble(drill_plan())


def _simulator_present() -> bool:
    """Whether Isaac Lab can be imported here: the scene hook and the stage writes need it."""
    try:
        return importlib.util.find_spec("isaaclab") is not None
    except (ImportError, ValueError):
        return False


if _simulator_present():
    try:
        install_hook()
    except (
        Exception
    ) as error:  # never break loading the task over the hook; the scene then scores 0
        _warn_once("hook", f"could not install the scene hook ({type(error).__name__}: {error})")
