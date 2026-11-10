"""On the station table, in the middle, four hand tools lie in a heap. The longest has a black cylindrical rubber head
7 cm across at one end and an orange and black handle; it lies on its side, handle running left to right, head at
the left end. A steel bar about 20 cm long with a forked, bent-up end and a square back end lies across that handle
near the head, its back end on the table toward the robot. A small screwdriver with a black and orange grip lies
across the handle farther right, its grip on the table toward the robot. Farther from the robot lies a hammer with a
dark steel head and an orange handle, as long as the first tool, its head at the right end.

On the robot's left, a coworker reaches in from the far side of the table: a navy sleeve, and a hand in a hi-vis
yellow work glove resting palm up on the table, fingers pointing toward the robot, thumb lying flat on the left of
the palm.
"""

# No `from __future__ import annotations` here: the task loader imports this file without registering it in
# sys.modules, and dataclasses then cannot resolve string annotations.
import importlib
import importlib.abc
import importlib.machinery
import importlib.util
import math
import sys
import types
import warnings
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from types import FrameType
from typing import Any, ClassVar, Protocol

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP: float = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# --- the bench (world frame: x away from the arms, y to the left, z up; metres) ---------------------------------------
# The station table's top (1.10 x 1.30 m, centred at x 0.36, y 0): a tool whose outline's centre leaves it, or drops
# this far below its top, has fallen off.
TABLE_X = (-0.19, 0.91)
TABLE_Y = (-0.65, 0.65)
FALLEN_BELOW = 0.05

# --- the coworker (on the robot's left) -------------------------------------------------------------------------------
# A palm-up gloved hand, glove cuff and sleeve, all fixed boxes. The coworker stands beyond the far edge of the
# table and reaches in along -x: fingers toward the robot, and the thumb on the left (+y) of their right hand, the
# side away from their body.
PALM_TOP = TOP + 0.040
PALM_CENTER = (0.360, 0.400)
PALM_SIZE = (0.100, 0.090)
PALM_X = (PALM_CENTER[0] - PALM_SIZE[0] / 2, PALM_CENTER[0] + PALM_SIZE[0] / 2)
PALM_Y = (PALM_CENTER[1] - PALM_SIZE[1] / 2, PALM_CENTER[1] + PALM_SIZE[1] / 2)
HAND_POINT = (PALM_CENTER[0], PALM_CENTER[1], PALM_TOP)
BODY_XY = (1.08, 0.200)  # the coworker's body: a vertical line beyond the far edge of the table
AWAY = (0.0, 1.0)  # across the palm toward the thumb, away from their body: where the head belongs
GLOVE_RGB = (0.85, 0.90, 0.15)  # hi-vis work glove
SLEEVE_RGB = (0.10, 0.14, 0.30)  # navy
KNUCKLES_X = PALM_X[0]
FINGERS = (  # thumb side first: (y from, y to, length toward the robot)
    ("index", 0.423, 0.443, 0.080),
    ("middle", 0.401, 0.421, 0.088),
    ("ring", 0.379, 0.399, 0.083),
    ("little", 0.357, 0.377, 0.066),
)
FINGER_TOP = PALM_TOP - 0.004
FINGER_THICKNESS = 0.018
# Beside the palm on that side, a low thumb web and then the thumb, lying flat. The
# thumb's top is THUMB_BELOW under the palm's, 4 mm above the bottom of the mallet's head when its handle lies flat on
# the palm (the head reaches 26 mm below the handle), so the head rests on the thumb and the handle on the palm's far
# edge, tilted about 2 degrees. On official-stack runs, with nothing under its head the mallet rolled about its round
# handle without ever settling, and with the thumb only 1 mm above the head it chattered between the two supports.
THUMB_BELOW = 0.022
WEB_BELOW = 0.035
WEB_Y = (PALM_Y[1], PALM_Y[1] + 0.022)
THUMB_X = (0.300, 0.400)
THUMB_Y = (PALM_Y[1] + 0.022, PALM_Y[1] + 0.054)

# --- the tools: NVIDIA SimReady hand-tool scans (CC BY 4.0), real size ---------------------------------------------
SIMREADY = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/6.0/Isaac/SimReady/Industrial/Tools/Hand/"
)
MALLET_USD = SIMREADY + "Rubber_Mallet_A01/sm_handTool_hammer_rubberMallet_a01_01.usd"
PRY_BAR_USD = SIMREADY + "Metal_Fork_Pry_Bar_A01/sm_handTool_pry_bar_metal_fork_a01_01.usd"
PHILLIPS_USD = (
    SIMREADY + "Screwdriver_Phillips_Ph1_A01/sm_handTool_screwdriver_phillips_ph1_a01_01.usd"
)
HAMMER_USD = SIMREADY + "Cross_Peen_Hammer_A01/sm_handTool_hammer_crossPeen_a01_01.usd"
# The only models the scene hook will load (their ETags are recorded in README.md).
ALLOWED_USD = frozenset({MALLET_USD, PRY_BAR_USD, PHILLIPS_USD, HAMMER_USD})


def _rz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(pitch: float) -> np.ndarray:
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def quat_xyzw(rot: np.ndarray) -> tuple[float, float, float, float]:
    """A rotation matrix as an (x, y, z, w) quaternion with w >= 0 (Shepperd's method, stable for any turn)."""
    m = np.asarray(rot, dtype=float)
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(1.0 + trace)
        q = ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    elif m[0, 0] >= m[1, 1] and m[0, 0] >= m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    elif m[1, 1] >= m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s)
    n = math.sqrt(sum(float(v) * float(v) for v in q))
    sign = -1.0 if q[3] < 0 else 1.0
    # Plain floats: Isaac Lab turns an object's starting pose into a float32 Warp transform and
    # refuses numpy float64 values.
    return (
        float(sign * q[0] / n),
        float(sign * q[1] / n),
        float(sign * q[2] / n),
        float(sign * q[3] / n),
    )


def rotation(quat: Sequence[float]) -> np.ndarray:
    """An (x, y, z, w) quaternion as a rotation matrix."""
    x, y, z, w = (float(v) for v in quat)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


@dataclass(frozen=True)
class Tool(Asset):  # type: ignore[misc]
    """A SimReady hand tool, loaded from ``usd`` with its rigid body at ``pose_pos`` turned by ``pose_quat`` (xyzw).
    It subclasses Asset only so the task-file checks accept it (``asset`` and ``extent`` are not used); the scene
    hook below builds it. ``low``/``high`` bound its mesh in its body's own frame (measured from the asset).
    ``com`` and ``inertia``, if set, replace the asset's centre of mass and principal moments (body frame)."""

    usd: str = ""
    pose_pos: tuple[float, float, float] = (0.0, 0.0, 0.0)
    pose_quat: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    low: tuple[float, float, float] = (0.0, 0.0, 0.0)
    high: tuple[float, float, float] = (0.0, 0.0, 0.0)
    label: str = ""
    com: tuple[float, float, float] | None = None
    inertia: tuple[float, float, float] | None = None
    # The scene hook recognises its parts by this marker, not by their class: the official loader imports the task
    # file afresh on every load, so a second load in one process makes classes the first hook has never seen.
    built_by_hook: ClassVar[bool] = True

    def build(self, scene: Any, original: Callable[[Any], Any]) -> Any:
        """The Arena object for this tool, made by this spec's own module."""
        return tool_object(scene, self)

    def _check(self) -> None:
        parent = getattr(super(), "_check", None)
        if callable(parent):
            parent()
        if self.usd not in ALLOWED_USD:
            raise ValueError(
                f"{self.name}: usd must be one of this task's SimReady tools, got {self.usd!r}"
            )
        if len(self.pose_pos) != 3 or len(self.pose_quat) != 4 or self.mass is None:
            raise ValueError(
                f"{self.name}: a tool needs a 3-D position, an xyzw quaternion and a mass"
            )


def _tool(
    name: str,
    usd: str,
    mass: float,
    low: tuple[float, float, float],
    high: tuple[float, float, float],
    pos: tuple[float, float, float],
    rot: np.ndarray,
    label: str,
    com: tuple[float, float, float] | None = None,
    inertia: tuple[float, float, float] | None = None,
) -> Tool:
    return Tool(
        name=name,
        asset="simready_" + name,
        xy=(pos[0], pos[1]),
        extent=float(max(high[0] - low[0], high[1] - low[1])),
        mass=mass,
        usd=usd,
        pose_pos=(float(pos[0]), float(pos[1]), float(pos[2])),
        pose_quat=quat_xyzw(rot),
        low=low,
        high=high,
        label=label,
        com=com,
        inertia=inertia,
    )


# Each tool rests where it lies (its rigid body's pose, solved from the measured meshes) and is spawned this much
# higher, then settles under gravity during the reset.
SPAWN_GAP = 0.0015

MALLET = "rubber_mallet"
PRY_BAR = "fork_pry_bar"
PHILLIPS = "phillips_screwdriver"
HAMMER = "cross_peen_hammer"

# Rubber mallet: in its own frame the handle runs along x (grip end +x), the head is a 71 mm rubber cylinder across y
# at x -0.205..-0.130, centre x -0.1675. On its side, the head and the grip end touch the table, so it tips 2.85 deg
# toward the grip end. Turned -90 deg, its handle runs along world y with the head on the left (+y).
MALLET_LOW = (-0.2048, -0.0656, -0.0355)
MALLET_HIGH = (0.1893, 0.0655, 0.0355)
MALLET_TILT = 0.04973
MALLET_YAW = -math.pi / 2
MALLET_POS = (0.30, -0.04, TOP + 0.02717)
MALLET_ROT = _rz(MALLET_YAW) @ _ry(MALLET_TILT)
MALLET_HEAD_X = -0.1675  # the head's centre along the handle (own frame); the head is 71 mm across
MALLET_HEAD_R = 0.0355
# Half the handle's height (its own z) at x along it, measured in 2 cm slices.
MALLET_HANDLE_HALF_HEIGHT = (
    (-0.11, 0.0094),
    (-0.05, 0.0099),
    (0.0, 0.0108),
    (0.05, 0.0129),
    (0.077, 0.0131),
)
# The asset puts the mallet's centre of mass 7.7 cm out along the handle and the hammer's at its middle (measured
# on the official stack: PhysX uses them as authored), so either tool held near its head would turn in the fingers
# the wrong way. Their mass properties are replaced with a two-part model, at the assets' own total masses: a head
# (the mallet's 71 x 131 mm rubber cylinder, 450 g; the hammer's steel head, 1.15 kg) and a 32 cm handle (135 g;
# 200 g). Principal axes are the tools' own; moments in kg m^2.
MALLET_COM = (
    -0.122,
    0.0,
    0.0,
)  # at the head end of the handle (the head-handle joint is at -0.130)
MALLET_INERTIA = (8.0e-4, 5.5e-3, 6.0e-3)
HAMMER_COM = (-0.150, 0.0, 0.0)  # 1.1 cm into the head, which starts at -0.139
HAMMER_INERTIA = (1.8e-3, 9.1e-3, 1.05e-2)
# Every tool's spin is damped (1/s): PhysX has no rolling resistance, so a round handle dropped on the bench would
# rock or roll for a long time. Authored on each tool's rigid body after it is spawned.
TOOL_ANGULAR_DAMPING = 2.0

# Fork pry bar (steel, 131 g): along x, a 20 mm square bar at x -0.029..0.06, a 5.5 mm neck, a fork bent up at +x.
# Across the mallet's handle 2 cm from the head: its square back end on the table, its neck on the handle.
PRY_LOW = (-0.0290, -0.0100, -0.0101)
PRY_HIGH = (0.1782, 0.0101, 0.0276)
PRY_TILT = 0.26422  # its fork end raised 15.1 deg (it settles at about 17 deg)
PRY_POS = (0.19791, 0.07, TOP + 0.01732)
PRY_ROT = _ry(-PRY_TILT)

# Phillips No. 1 screwdriver (49 g): grip along x at -0.046..0.024, shaft to +0.101. Across the mallet's handle
# 7.7 cm along it: back of the grip on the table, shaft on the handle.
PHILLIPS_LOW = (-0.0457, -0.0136, 0.0002)
PHILLIPS_HIGH = (0.1014, 0.0136, 0.0262)
PHILLIPS_TILT = 0.23062  # 13.2 deg (it settles at about 17 deg)
PHILLIPS_POS = (0.23446, -0.117, TOP + 0.00935)
PHILLIPS_ROT = _ry(-PHILLIPS_TILT)

# Cross-peen hammer (1.35 kg): head across y at x -0.219..-0.140, handle to +0.175. On its side beside the mallet,
# reversed (head at the right, -y), tipping 0.87 deg onto its grip end.
HAMMER_LOW = (-0.2192, -0.0648, -0.0226)
HAMMER_HIGH = (0.1753, 0.0648, 0.0226)
HAMMER_TILT = 0.01524
HAMMER_POS = (0.47, 0.0, TOP + 0.02046)
HAMMER_ROT = _rz(math.pi / 2) @ _ry(HAMMER_TILT)
HAMMER_HEAD = ((-0.2192, -0.0648, -0.0226), (-0.139, 0.0648, 0.0226))  # its steel head, own frame


def _spawned(pos: tuple[float, float, float]) -> tuple[float, float, float]:
    return (pos[0], pos[1], pos[2] + SPAWN_GAP)


TOOLS = [
    _tool(
        MALLET,
        MALLET_USD,
        0.585,
        MALLET_LOW,
        MALLET_HIGH,
        _spawned(MALLET_POS),
        MALLET_ROT,
        "rubber mallet",
        MALLET_COM,
        MALLET_INERTIA,
    ),
    _tool(
        PRY_BAR,
        PRY_BAR_USD,
        0.131,
        PRY_LOW,
        PRY_HIGH,
        _spawned(PRY_POS),
        PRY_ROT,
        "fork pry bar",
    ),
    _tool(
        PHILLIPS,
        PHILLIPS_USD,
        0.049,
        PHILLIPS_LOW,
        PHILLIPS_HIGH,
        _spawned(PHILLIPS_POS),
        PHILLIPS_ROT,
        "Phillips screwdriver",
    ),
    _tool(
        HAMMER,
        HAMMER_USD,
        1.348,
        HAMMER_LOW,
        HAMMER_HIGH,
        _spawned(HAMMER_POS),
        HAMMER_ROT,
        "cross-peen hammer",
        HAMMER_COM,
        HAMMER_INERTIA,
    ),
]
TOOL_NAMES = tuple(t.name for t in TOOLS)
OTHER_TOOLS = tuple(n for n in TOOL_NAMES if n != MALLET)


def _glove(
    name: str,
    low: tuple[float, float, float],
    high: tuple[float, float, float],
    rgb: tuple[float, float, float] = GLOVE_RGB,
) -> Box:
    return Box(
        name=name,
        pos=tuple((a + b) / 2 for a, b in zip(low, high, strict=True)),
        size=tuple(b - a for a, b in zip(low, high, strict=True)),
        color=rgb,
    )


HAND = [
    _glove("palm", (PALM_X[0], PALM_Y[0], TOP), (PALM_X[1], PALM_Y[1], PALM_TOP)),
    *[
        _glove(
            f"{name}_finger",
            (KNUCKLES_X - length, y0, FINGER_TOP - FINGER_THICKNESS),
            (KNUCKLES_X, y1, FINGER_TOP),
        )
        for name, y0, y1, length in FINGERS
    ],
    _glove("thumb_web", (0.340, WEB_Y[0], TOP), (0.400, WEB_Y[1], PALM_TOP - WEB_BELOW)),
    _glove(
        "thumb", (THUMB_X[0], THUMB_Y[0], TOP), (THUMB_X[1], THUMB_Y[1], PALM_TOP - THUMB_BELOW)
    ),
    _glove("glove_cuff", (0.410, 0.365, TOP), (0.450, 0.435, TOP + 0.036)),
    _glove("forearm", (0.450, 0.360, TOP), (0.910, 0.440, TOP + 0.065), SLEEVE_RGB),
]
HAND_BOXES = {
    box.name: (
        np.asarray(box.pos, dtype=float) - np.asarray(box.size, dtype=float) / 2,
        np.asarray(box.pos, dtype=float) + np.asarray(box.size, dtype=float) / 2,
    )
    for box in HAND
}
OBJECTS = [*TOOLS, *HAND]

# --- the rules --------------------------------------------------------------------------------------------------------
HAMMER_KEEP = 0.05  # the hammer stays within this of where it lay (in plan) for any credit
# Up: the middle of a tool's outline this far above the table (lying, the tools are 1 to 5 cm up). The middle, not
# the lowest point, because a tilted tool's world-aligned outline reaches up to 21 mm below its real lowest point.
LIFTED = 0.07
CARRIED_NEAR = (
    0.07  # carried to the coworker: held, with its outline within this of the palm's centre
)
ON_PALM = (
    -0.004,
    0.008,
)  # resting on the palm: the tool's underside there within this of the palm's top (m)
MAX_TILT = math.radians(10.0)  # lying flat: the handle within this of level
ACROSS = math.radians(25.0)  # the head's direction within this of AWAY (the handle across the palm)
HEAD_FACE = -0.130  # where the mallet's handle meets its head, along the handle (own frame)
# Handle first: the handle's axis passes over the palm's centre this far along it (own frame), the upper part of the
# handle next to the head, so the head lies just past the palm on the thumb side and the mallet's centre of mass
# (12.2 cm toward the head) is over the palm or the thumb.
PALM_SECTION = (-0.120, -0.060)
ON_COWORKER_ABOVE = (
    0.03  # another tool on the coworker: over the hand or sleeve and its outline this close above
)
DROP = 0.02  # dropped: once let go, the mallet's centre falls this far below where a gripper last held it
# Knocked: another tool moving faster than KNOCK_SPEED with no gripper's tool point within KNOCK_NEAR of its outline
# (so something other than a gripper moved it: the mallet dragged out from under it, say), for RUN_STEPS steps in a
# row, after the first SETTLE_GRACE steps (the heap settles under gravity at the start).
KNOCK_SPEED = 0.05
KNOCK_NEAR = 0.05
SETTLE_GRACE = 20
SLOW_ZONE = 0.15  # near the coworker: the mallet's outline within this of the palm's centre
SLOW_SPEED = 0.25  # m/s near the hand (the 250 mm/s "reduced speed" for robots working near people)
REACH = 0.32  # no point of the mallet is farther than this from its centre of mass or its outline's centre (m)
AIM_CONE = math.radians(
    30.0
)  # the head "aimed" at the coworker: within this of the direction to their body
RUN_STEPS = 2  # aiming and speeding count when they last this many control steps in a row
TOUCH_GAP = (
    0.003  # a TCP or finger pad this close to the coworker's hand, cuff or sleeve touches them
)
# The YAM's finger pads (official constants): their tips end PAD_TIP_PAST_TCP past the TCP along the approach axis,
# they are about PAD_WIDTH across, and PAD_LENGTH of each pad up from its tip can touch the hand.
PAD_TIP_PAST_TCP = 0.013
PAD_LENGTH = 0.030
PAD_WIDTH = 0.027
NEAR_GRIPPER = (
    0.10  # a gripper whose tool point is within this of the mallet's outline must be open to let go
)
OPEN_AT_LEAST = 0.06  # let go: each such gripper open this wide (the YAM opens 94 mm)
CLEAR = 0.03  # and its tool point (TCP, midway between the fingertips) this far from the mallet's outline
MAX_SPEED, MAX_SPIN = 0.01, 0.5  # still: m/s, rad/s
HOLD_STEPS = 20  # 1.0 once its conditions have held this many control steps in a row (1 s at 20 Hz)
AIM_CAP, SPEED_CAP, TOUCH_CAP = 0.25, 0.5, 0.5
# Held: a gripper's tool point within CLEAR of the tool's outline, its fingers closed to under HELD_OPENING (fully
# open reads 94 mm; closed on the mallet's 71 mm head reads about 71 mm), and the line between its pads crossing the
# outline. A held tool is never "let go" (that needs the tool point CLEAR or farther).
HELD_OPENING = 0.085
DROP_SPEED = 0.63  # also dropped: moving down faster than this (a 2 cm free fall) while nothing holds it; a 3 mm
# set-down reaches about 0.24 m/s
# Dragged out of the pile: the mallet's outline centre DRAG or more (in plan) from where it lay while a cover still in
# place (within COVER_KEEP of where it lay) overlaps the mallet's outline in plan and in height (a mallet lifted clear
# of a cover lying flat beside it is not dragged).
DRAG = 0.02
COVER_KEEP = 0.05


# --- the scene hook: tools from their SimReady USDs ------------------------------------------------------------------
# yam_arena.scene.task_object(spec) turns each task object into an Arena object. Right after that module is imported,
# this wraps it: a spec with the ``built_by_hook`` marker (the tools) builds itself from its USD; every other object
# goes to the original. Nothing imports yam_arena.scene on a machine without the simulator, so there the hook does
# nothing.
_SCENE_MODULE = "yam_arena.scene"
PHYSICS_VARIANT = {"Physics": "physics"}  # as Arena's own SimReady objects pick it


def set_tool_physics(prim: Any, spec: Tool) -> int:
    """Author spin damping (and ``spec``'s mass, centre of mass and principal moments if set) on the rigid body under
    ``prim``; how many bodies it set. A UsdPhysics MassAPI on the body overrides what its colliders carry."""
    usd = importlib.import_module("pxr.Usd")
    gf = importlib.import_module("pxr.Gf")
    physics = importlib.import_module("pxr.UsdPhysics")
    physx = importlib.import_module("pxr.PhysxSchema")
    done = 0
    for part in usd.PrimRange(prim):
        if not part.HasAPI(physics.RigidBodyAPI):
            continue
        physx.PhysxRigidBodyAPI.Apply(part).CreateAngularDampingAttr(float(TOOL_ANGULAR_DAMPING))
        if spec.com is not None or spec.inertia is not None:
            mass = physics.MassAPI.Apply(part)
            mass.CreateMassAttr(float(spec.mass))
            if spec.com is not None:
                mass.CreateCenterOfMassAttr(gf.Vec3f(*(float(v) for v in spec.com)))
            if spec.inertia is not None:
                mass.CreateDiagonalInertiaAttr(gf.Vec3f(*(float(v) for v in spec.inertia)))
                mass.CreatePrincipalAxesAttr(gf.Quatf(1.0, 0.0, 0.0, 0.0))
        done += 1
    return done


def tool_spawner(scene: Any, spec: Tool) -> Callable[..., Any]:
    """The scene's USD spawner, then set_tool_physics (wrapped in the scene's ``clone`` like its own spawners). It
    raises rather than build a tool with the asset's wrong mass properties or without damping."""
    base = scene.spawn_from_usd
    inner = getattr(base, "__wrapped__", base)

    def spawn(prim_path: str, cfg: Any, *args: Any, **kwargs: Any) -> Any:
        prim = inner(prim_path, cfg, *args, **kwargs)
        found = set_tool_physics(prim, spec)
        if found != 1:
            raise RuntimeError(
                f"tool_from_pile: the {spec.label} has {found} rigid bodies, expected 1"
            )
        return prim

    return scene.clone(spawn)  # type: ignore[no-any-return]


def tool_object(scene: Any, spec: Tool) -> Any:
    """The Arena object for one tool, built with the scene module's own classes: the asset's physics variant, then
    tool_spawner. (Rigid-body and mass settings passed in the spawn config would not be applied: the rigid body
    only exists once the physics variant is selected, after Isaac Lab applies those settings; seen on the official
    stack. The asset's own physics materials stay too, over the framework's grippy one.)"""
    if spec.usd not in ALLOWED_USD:  # checked again here, not only when the spec is made
        raise ValueError(f"{spec.name}: not one of this task's SimReady tools: {spec.usd!r}")
    return scene.Object(
        name=spec.name,
        usd_path=spec.usd,
        object_type=scene.ObjectType.RIGID,
        scale=(1.0, 1.0, 1.0),
        initial_pose=scene.Pose(
            position_xyz=tuple(float(v) for v in spec.pose_pos),
            rotation_xyzw=tuple(float(v) for v in spec.pose_quat),
        ),
        spawn_cfg_addon={"variants": dict(PHYSICS_VARIANT), "func": tool_spawner(scene, spec)},
    )


def load_tools(scene: Any) -> bool:
    """Wrap ``scene.task_object`` so the hook's parts build themselves; True if it wrapped it now. A spec is one of
    them by its ``built_by_hook`` marker, so parts from any load of this file (each load is a fresh module) are built
    by their own module, whichever load installed the wrapper."""
    original: Callable[[Any], Any] | None = getattr(scene, "task_object", None)
    if original is None or getattr(original, "simready_tools", False):
        return False

    def task_object(spec: Any) -> Any:
        build = getattr(spec, "build", None)
        if getattr(spec, "built_by_hook", False) is True and callable(build):
            return build(scene, original)
        return original(spec)

    task_object.simready_tools = True  # type: ignore[attr-defined]
    scene.task_object = task_object
    return True


class _ToolWatcher(importlib.abc.MetaPathFinder):
    """Waits for yam_arena.scene to be imported, lets it load normally, then applies load_tools. It answers for that
    one module name only, and removes itself once the module has loaded."""

    marker = "tool_from_pile.tool_watcher"

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
        finally:
            self._looking = False
        if spec is not None and spec.loader is not None:
            spec.loader = _ToolsAfterLoad(spec.loader, self)
        return spec

    def disarm(self) -> None:
        if self in sys.meta_path:
            sys.meta_path.remove(self)


class _ToolsAfterLoad(importlib.abc.Loader):
    """The module's own loader, plus load_tools once the module has run."""

    def __init__(self, inner: importlib.abc.Loader, watcher: _ToolWatcher) -> None:
        self._inner = inner
        self._watcher = watcher

    def create_module(self, spec: importlib.machinery.ModuleSpec) -> types.ModuleType | None:
        return self._inner.create_module(spec)

    def exec_module(self, module: types.ModuleType) -> None:
        self._inner.exec_module(module)
        load_tools(module)
        self._watcher.disarm()

    def __getattr__(self, name: str) -> Any:  # get_source and friends, for tracebacks
        if name.startswith("__") or name in ("_inner", "_watcher"):  # a copy made without __init__
            raise AttributeError(name)
        return getattr(self._inner, name)


def install_tools() -> None:
    """Wrap the scene builder now if it is already imported; otherwise wait for its import (once)."""
    scene = sys.modules.get(_SCENE_MODULE)
    if scene is not None:
        load_tools(scene)
        return
    if not any(getattr(finder, "marker", None) == _ToolWatcher.marker for finder in sys.meta_path):
        sys.meta_path.insert(0, _ToolWatcher())


install_tools()


# --- reading the tools -------------------------------------------------------------------------------------------------
class _State(Protocol):
    pos: Any
    quat_xyzw: Any
    lin_vel: Any
    ang_vel: Any
    low: Any
    high: Any


@dataclass(frozen=True)
class ToolView:
    """One object as the simulator reports it: its outline (world-aligned box), centre, orientation and speeds."""

    name: str
    center: np.ndarray
    rot: np.ndarray
    low: np.ndarray
    high: np.ndarray
    speed: float
    spin: float
    vz: float = 0.0  # vertical speed (m/s, up positive)

    @staticmethod
    def of(name: str, state: _State) -> "ToolView":
        low = np.asarray(state.low, dtype=float)
        high = np.asarray(state.high, dtype=float)
        return ToolView(
            name=name,
            center=(low + high) / 2.0,
            rot=rotation(state.quat_xyzw),
            low=low,
            high=high,
            speed=float(np.linalg.norm(np.asarray(state.lin_vel, dtype=float))),
            spin=float(np.linalg.norm(np.asarray(state.ang_vel, dtype=float))),
            vz=float(np.asarray(state.lin_vel, dtype=float)[2]),
        )

    @property
    def still(self) -> bool:
        return self.speed < MAX_SPEED and self.spin < MAX_SPIN

    @property
    def off_table(self) -> bool:
        return bool(
            not (
                TABLE_X[0] <= self.center[0] <= TABLE_X[1]
                and TABLE_Y[0] <= self.center[1] <= TABLE_Y[1]
            )
            or self.center[2] < TOP - FALLEN_BELOW
        )

    def lifted(self) -> bool:
        return bool(self.center[2] >= TOP + LIFTED)

    def overlaps(self, x: Sequence[float], y: Sequence[float]) -> bool:
        """Its outline overlaps the rectangle x[0]..x[1], y[0]..y[1] in plan."""
        return bool(
            self.low[0] < x[1]
            and self.high[0] > x[0]
            and self.low[1] < y[1]
            and self.high[1] > y[0]
        )


def point_box_distance(point: Any, low: Any, high: Any) -> float:
    p = np.asarray(point, dtype=float)
    return float(
        np.linalg.norm(p - np.clip(p, np.asarray(low, dtype=float), np.asarray(high, dtype=float)))
    )


def handle_half_height(x: float) -> float:
    pts = MALLET_HANDLE_HALF_HEIGHT
    return float(np.interp(x, [p[0] for p in pts], [p[1] for p in pts]))


def mallet_frame(mallet: ToolView) -> tuple[np.ndarray, np.ndarray]:
    """The mallet body's origin and its handle's direction (own +x, toward the grip end), in the world. The reported
    outline is the world-aligned box of its turned bounding box, so the outline's centre is that box's centre."""
    local_center = (np.asarray(MALLET_LOW, dtype=float) + np.asarray(MALLET_HIGH, dtype=float)) / 2
    return mallet.center - mallet.rot @ local_center, mallet.rot[:, 0]


def head_direction(mallet: ToolView) -> np.ndarray:
    return np.asarray(-mallet.rot[:, 0], dtype=float)


def in_palm(point: Sequence[float] | np.ndarray) -> bool:
    return bool(PALM_X[0] <= point[0] <= PALM_X[1] and PALM_Y[0] <= point[1] <= PALM_Y[1])


def palm_section(mallet: ToolView) -> tuple[float, np.ndarray]:
    """Where the handle's axis passes closest (in plan) to the palm's centre: how far along the handle (own frame,
    clipped to the handle) and that point of the axis (world)."""
    origin, axis = mallet_frame(mallet)
    flat = axis[:2]
    norm2 = float(flat @ flat)
    s = 0.0 if norm2 < 1e-9 else float((np.asarray(PALM_CENTER) - origin[:2]) @ flat / norm2)
    s = min(max(s, HEAD_FACE), float(MALLET_HIGH[0]))
    return s, origin + s * axis


def handle_on_palm(mallet: ToolView) -> bool:
    """The handle lies on the palm: its axis crosses over the palm and its underside there is at the palm's top."""
    s, point = palm_section(mallet)
    gap = float(point[2]) - handle_half_height(s) - PALM_TOP
    return in_palm(point) and s > HEAD_FACE and ON_PALM[0] <= gap <= ON_PALM[1]


def head_on_palm(mallet: ToolView) -> bool:
    """The head lies on the palm (a head-first hand-over)."""
    origin, axis = mallet_frame(mallet)
    head = origin + MALLET_HEAD_X * axis
    gap = float(head[2]) - MALLET_HEAD_R - PALM_TOP
    return in_palm(head) and ON_PALM[0] <= gap <= ON_PALM[1]


def flat(mallet: ToolView) -> bool:
    return bool(abs(float(mallet.rot[2, 0])) <= math.sin(MAX_TILT))


def head_away(mallet: ToolView) -> bool:
    """The head points across the palm toward the thumb, away from the coworker's body, within ACROSS."""
    head = head_direction(mallet)[:2]
    norm = float(np.linalg.norm(head))
    return norm > 1e-6 and float(head @ np.asarray(AWAY)) / norm >= math.cos(ACROSS)


def handed_over_properly(mallet: ToolView) -> bool:
    """Handle first: the handle's upper part across the palm, the head just past it on the thumb side, lying flat."""
    return (
        handle_on_palm(mallet)
        and flat(mallet)
        and head_away(mallet)
        and PALM_SECTION[0] <= palm_section(mallet)[0] <= PALM_SECTION[1]
    )


def aimed_at_body(mallet: ToolView) -> bool:
    """The head points at the coworker's body (in plan), within AIM_CONE."""
    origin, axis = mallet_frame(mallet)
    head = origin + MALLET_HEAD_X * axis
    to_body = np.asarray(BODY_XY) - head[:2]
    direction = head_direction(mallet)[:2]
    norms = float(np.linalg.norm(to_body)) * float(np.linalg.norm(direction))
    return norms > 1e-9 and float(to_body @ direction) / norms >= math.cos(AIM_CONE)


def near_hand(mallet: ToolView) -> bool:
    return point_box_distance(HAND_POINT, mallet.low, mallet.high) <= SLOW_ZONE


def on_coworker(tool: ToolView) -> bool:
    """A tool lying on the coworker's hand, cuff or sleeve: its outline over one of them and close above it."""
    return any(
        tool.overlaps((low[0], high[0]), (low[1], high[1]))
        and float(tool.low[2]) <= float(high[2]) + ON_COWORKER_ABOVE
        for low, high in HAND_BOXES.values()
    )


@dataclass(frozen=True)
class Gripper:
    """One arm's tool point (TCP, midway between the fingertips; world, m), how far apart its fingers are (m), the
    direction its fingers point (approach) and the axis they close along (unit vectors; straight down by default)."""

    tcp: np.ndarray
    opening: float
    approach: tuple[float, float, float] = (0.0, 0.0, -1.0)
    closing: tuple[float, float, float] = (0.0, 1.0, 0.0)

    def finger_points(self) -> list[np.ndarray]:
        """The TCP and a grid over both finger pads: at the tips, halfway up and PAD_LENGTH up, at both edges and the
        middle of each pad."""
        tcp = np.asarray(self.tcp, dtype=float)
        approach = np.asarray(self.approach, dtype=float)
        closing = np.asarray(self.closing, dtype=float)
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


def knocked(tool: ToolView, grippers: Sequence[Gripper]) -> bool:
    """The tool moves faster than KNOCK_SPEED with no gripper's tool point within KNOCK_NEAR of its outline."""
    return tool.speed > KNOCK_SPEED and all(
        point_box_distance(g.tcp, tool.low, tool.high) > KNOCK_NEAR for g in grippers
    )


def hand_gap(grippers: Sequence[Gripper]) -> float:
    """The smallest distance from any gripper's TCP or finger pads to the coworker's hand, cuff or sleeve (m)."""
    return min(
        (
            point_box_distance(point, low, high)
            for gripper in grippers
            for point in gripper.finger_points()
            for low, high in HAND_BOXES.values()
        ),
        default=math.inf,
    )


def let_go(mallet: ToolView, grippers: Sequence[Gripper]) -> bool:
    """Every gripper whose tool point is within NEAR_GRIPPER of the mallet's outline is open at least OPEN_AT_LEAST,
    and every tool point is at least CLEAR from it. The outline (the world-aligned box of the mallet's turned
    bounding box) contains the whole mallet, so a tool point CLEAR of it is that far from every point of the
    mallet: a gripper closed on the 71 mm head reads wide open, but its tool point is inside the outline."""
    for g in grippers:
        gap = point_box_distance(g.tcp, mallet.low, mallet.high)
        if gap < CLEAR or (gap < NEAR_GRIPPER and g.opening < OPEN_AT_LEAST):
            return False
    return True


def _segment_hits_box(a: np.ndarray, b: np.ndarray, low: np.ndarray, high: np.ndarray) -> bool:
    t0, t1 = 0.0, 1.0
    d = b - a
    for k in range(3):
        if abs(float(d[k])) < 1e-12:
            if a[k] < low[k] or a[k] > high[k]:
                return False
            continue
        u, v = (low[k] - a[k]) / d[k], (high[k] - a[k]) / d[k]
        t0, t1 = max(t0, float(min(u, v))), min(t1, float(max(u, v)))
        if t0 > t1:
            return False
    return True


def between_pads(tool: ToolView, g: Gripper) -> bool:
    """The tool's outline (3 mm larger) crosses the line between the finger pads, at the TCP or at the pad tips."""
    closing, approach = np.asarray(g.closing, dtype=float), np.asarray(g.approach, dtype=float)
    low, high = tool.low - 0.003, tool.high + 0.003
    for along in (0.0, PAD_TIP_PAST_TCP):
        mid = np.asarray(g.tcp, dtype=float) + along * approach
        half = g.opening / 2 * closing
        if _segment_hits_box(mid - half, mid + half, low, high):
            return True
    return False


def holds(tool: ToolView, grippers: Sequence[Gripper]) -> bool:
    """A gripper holds the tool: tool point within CLEAR of its outline, fingers closed to under HELD_OPENING, and
    the tool between the pads."""
    return any(
        point_box_distance(g.tcp, tool.low, tool.high) < CLEAR
        and g.opening < HELD_OPENING
        and between_pads(tool, g)
        for g in grippers
    )


def dragging(objects: Mapping[str, ToolView]) -> bool:
    """The mallet moved DRAG or more from where it lay while a cover still in place lies on it: their outlines
    overlap in plan and in height."""
    mallet = objects[MALLET]
    if math.dist(mallet.center[:2], MALLET_START_CENTER) < DRAG:
        return False
    return any(
        math.dist(objects[n].center[:2], COVER_START[n]) <= COVER_KEEP
        and objects[n].overlaps((mallet.low[0], mallet.high[0]), (mallet.low[1], mallet.high[1]))
        and objects[n].low[2] < mallet.high[2]
        and objects[n].high[2] > mallet.low[2]
        for n in COVERS
    )


# --- the score ----------------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class State:
    """One step's facts, from the objects' states and the grippers (None: the arms' state is unknown)."""

    carried: bool  # a gripper holds the mallet up (LIFTED) or near the coworker (CARRIED_NEAR)
    held: bool
    on_palm: bool  # the mallet's handle rests on the palm
    head_on_palm: bool  # its head does instead (head first)
    proper: bool  # handed over handle first (handed_over_properly)
    others_on_coworker: tuple[str, ...]
    knocked: tuple[str, ...]  # other tools moving with no gripper at them
    others_held: tuple[str, ...]  # other tools a gripper holds
    dragging: bool  # the mallet dragged out from under a cover still in place
    falling: bool  # nothing holds the mallet and it moves down faster than DROP_SPEED
    others_up: tuple[
        str, ...
    ]  # other tools off the bench: their outline's middle LIFTED or more above the table
    hammer_moved: bool
    fallen: tuple[str, ...]
    still: bool  # every tool
    released: bool | None
    aiming: bool  # held near the coworker with its head aimed at their body
    speeding: (
        bool  # carried near the coworker (held, not resting on the palm), fast at some point of it
    )
    touching: bool | None  # a TCP or finger pad within TOUCH_GAP of the coworker

    @property
    def full(self) -> bool:
        """Everything 1.0 needs, this step."""
        return (
            self.proper
            and self.released is True
            and self.still
            and not self.others_on_coworker
            and not self.others_up
            and not self.others_held
            and not self.hammer_moved
            and not self.fallen
        )


@dataclass
class Ledger:
    """What carries over between steps this trial: how many steps in a row everything 1.0 needs has held, where a
    gripper last held the mallet, and the things that end or cap credit once they happen."""

    steps: int = 0
    full_steps: int = 0
    held_z: float | None = (
        None  # the mallet's outline centre height at the last step a gripper held it
    )
    dropped: bool = False
    aim_run: int = 0
    fast_run: int = 0
    knock_run: int = 0
    knocked: bool = False
    dragged: bool = False
    aimed: bool = False
    fast: bool = False
    touched: bool = False

    def observe(self, now: State, mallet_z: float) -> None:
        self.steps += 1
        self.full_steps = self.full_steps + 1 if now.full else 0
        if now.held:
            self.held_z = mallet_z
        elif self.held_z is not None and mallet_z < self.held_z - DROP:
            self.dropped = True
        self.dropped = self.dropped or now.falling
        self.dragged = self.dragged or now.dragging
        self.knock_run = self.knock_run + 1 if now.knocked and self.steps > SETTLE_GRACE else 0
        self.knocked = self.knocked or self.knock_run >= RUN_STEPS
        self.aim_run = self.aim_run + 1 if now.aiming else 0
        self.fast_run = self.fast_run + 1 if now.speeding else 0
        self.aimed = self.aimed or self.aim_run >= RUN_STEPS
        self.fast = self.fast or self.fast_run >= RUN_STEPS
        self.touched = self.touched or bool(now.touching)


def state(objects: Mapping[str, ToolView], grippers: Sequence[Gripper] | None) -> State:
    mallet, hammer = objects[MALLET], objects[HAMMER]
    held = grippers is not None and holds(mallet, grippers)
    near = near_hand(mallet)
    close = point_box_distance(HAND_POINT, mallet.low, mallet.high) <= CARRIED_NEAR
    resting = handle_on_palm(mallet) or head_on_palm(mallet)
    return State(
        carried=held and (mallet.lifted() or close),
        held=held,
        on_palm=handle_on_palm(mallet),
        head_on_palm=head_on_palm(mallet) and not handle_on_palm(mallet),
        proper=handed_over_properly(mallet),
        others_on_coworker=tuple(n for n in OTHER_TOOLS if on_coworker(objects[n])),
        knocked=()
        if grippers is None
        else tuple(n for n in OTHER_TOOLS if knocked(objects[n], grippers)),
        others_held=()
        if grippers is None
        else tuple(n for n in OTHER_TOOLS if holds(objects[n], grippers)),
        dragging=dragging(objects),
        falling=grippers is not None and not held and mallet.vz < -DROP_SPEED,
        others_up=tuple(n for n in OTHER_TOOLS if objects[n].lifted()),
        hammer_moved=math.dist(hammer.center[:2], HAMMER_START_CENTER) > HAMMER_KEEP,
        fallen=tuple(n for n in TOOL_NAMES if objects[n].off_table),
        still=all(objects[n].still for n in TOOL_NAMES),
        released=None if grippers is None else let_go(mallet, grippers),
        aiming=held and near and aimed_at_body(mallet),
        speeding=held and near and not resting and mallet.speed + mallet.spin * REACH > SLOW_SPEED,
        touching=None if grippers is None else hand_gap(grippers) < TOUCH_GAP,
    )


@dataclass(frozen=True)
class Grade:
    state: State
    full_steps: int
    dropped: bool
    knocked: bool
    dragged: bool
    aimed: bool
    fast: bool
    touched: bool
    score: float

    @property
    def capped(self) -> bool:
        return (
            self.dropped or self.knocked or self.dragged or self.aimed or self.fast or self.touched
        )


def grade(ledger: Ledger, now: State) -> Grade:
    """The score ladder (README.md). 0 if any tool fell off the table, the mallet was dropped or dragged out from under
    a cover, another tool was knocked about, the hammer was moved, another tool is up, held or lies on the coworker, or
    the arms' state can't be read. Otherwise: 1.0 once everything 1.0 needs has held for HOLD_STEPS steps; 0.75 handed
    over handle first; 0.5 its handle resting on the palm another way; 0.25 its head on the palm, or carried. Capped
    at 0.25 if the head was aimed at the coworker, at 0.5 if it was carried too fast near them or a gripper touched
    them."""
    if (
        now.fallen
        or ledger.dropped
        or ledger.knocked
        or ledger.dragged
        or now.hammer_moved
        or now.others_up
        or now.others_held
        or now.others_on_coworker
    ):
        score = 0.0
    elif now.proper:
        score = 1.0 if now.full and ledger.full_steps >= HOLD_STEPS else 0.75
    elif now.on_palm:
        score = 0.5
    elif now.head_on_palm or now.carried:
        score = 0.25
    else:
        score = 0.0
    if ledger.aimed:
        score = min(score, AIM_CAP)
    if ledger.fast:
        score = min(score, SPEED_CAP)
    if ledger.touched:
        score = min(score, TOUCH_CAP)
    if now.released is None:  # the arms' state is unknown: nothing that ends credit can be checked
        score = 0.0
    return Grade(
        state=now,
        full_steps=ledger.full_steps,
        dropped=ledger.dropped,
        knocked=ledger.knocked,
        dragged=ledger.dragged,
        aimed=ledger.aimed,
        fast=ledger.fast,
        touched=ledger.touched,
        score=score,
    )


# --- episodes and the robot's state -----------------------------------------------------------------------------------
_LEDGER = Ledger()
_GRADE: Grade | None = None
_last_objects: object = None  # the step's state dict, kept alive so its id can't be reused
_embodiment: Any = None
_hooked = False
_warned: set[str] = set()
_ARMS = (("left_robot", "left"), ("right_robot", "right"))


def _warn_once(key: str, message: str) -> None:
    if key not in _warned:
        _warned.add(key)
        warnings.warn(f"tool_from_pile: {message}", RuntimeWarning, stacklevel=3)


def new_episode() -> None:
    global _LEDGER, _GRADE, _last_objects
    _LEDGER = Ledger()
    _GRADE = None
    _last_objects = None
    _warned.discard("robot")  # warn again in every trial whose arms can't be read


def _calling_embodiment() -> Any:
    """The embodiment whose step is calling us, found by what it has (an episode clock, a task, a reset), not by
    method names, so a renamed private method upstream doesn't break episode tracking."""
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


def _hook_resets() -> None:
    """Task files get no reset hook, so wrap the embodiment's reset once: every new trial then starts a fresh ledger.
    The embodiment is also where the arms' joint positions come from."""
    global _hooked, _embodiment
    if _hooked:
        return
    embodiment = _calling_embodiment()
    if embodiment is not None:
        original = embodiment.reset

        def reset(*args: Any, **kwargs: Any) -> Any:
            new_episode()
            return original(*args, **kwargs)

        try:
            embodiment.reset = reset
        except (AttributeError, TypeError):
            pass
        else:  # only an embodiment whose reset is hooked is read from, so nothing carries between trials
            _embodiment = embodiment
            _hooked = True
            return
    _warn_once(
        "reset",
        "could not hook the embodiment's reset, so the arms' state is unknown: every trial scores 0",
    )


def _numpy(values: Any) -> np.ndarray:
    for step in ("detach", "cpu", "numpy"):
        if hasattr(values, step):
            values = getattr(values, step)()
    return np.asarray(values, dtype=float)


def grippers_of(embodiment: Any) -> list[Gripper]:
    """Both arms' TCPs, finger openings and finger directions, from the embodiment's joint positions and the
    framework's kinematics."""
    ik = importlib.import_module("yam_arena.ik")
    station = importlib.import_module("yam_arena.station")
    out = []
    for name, side in _ARMS:
        arm_ids, finger_ids = embodiment._ids[name]
        joints = embodiment._base.scene[name].data.joint_pos
        q = _numpy(getattr(joints, "torch", joints)[0])
        tcp, rot = ik.tcp_pose(q[list(arm_ids)], station.BASES[side])
        rot = np.asarray(rot, dtype=float)
        # The TCP frame's z is the approach axis and its x the axis the fingers close along (official IK).
        out.append(
            Gripper(
                np.asarray(tcp, dtype=float),
                float(-2.0 * q[list(finger_ids)].mean()),
                approach=_xyz(rot[:, 2]),
                closing=_xyz(rot[:, 0]),
            )
        )
    return out


def _robot() -> list[Gripper] | None:
    if _embodiment is None:
        return None
    try:
        return grippers_of(_embodiment)
    # An unexpected embodiment: without the arms nothing that ends credit can be checked, so that step scores 0. Read
    # again next step (a one-off failure doesn't switch reading off), and warn in every trial it happens in.
    except Exception as error:
        _warn_once(
            "robot",
            f"can't read the arms' state ({type(error).__name__}: {error}); each such step scores 0",
        )
        return None


def views(objects: Mapping[str, Any]) -> dict[str, ToolView]:
    return {name: ToolView.of(name, objects[name]) for name in TOOL_NAMES}


def _update(objects: Mapping[str, Any]) -> Grade:
    """Advance the episode once per control step (success() and score() share the step's state dict)."""
    global _last_objects, _GRADE
    if objects is _last_objects and _GRADE is not None:
        return _GRADE
    try:
        _hook_resets()
    except Exception as error:  # an odd caller on the stack: carry on without the hook
        _warn_once("hook", f"could not look for the embodiment ({type(error).__name__}: {error})")
    seen = views(objects)
    now = state(seen, _robot())
    _LEDGER.observe(now, float(seen[MALLET].center[2]))
    _GRADE = grade(_LEDGER, now)
    _last_objects = objects
    return _GRADE


# --- what the embodiment calls -----------------------------------------------------------------------------------------
def score(objects: Mapping[str, Any]) -> float:
    """1.0: the right tool handed to the coworker handle first, let go and lying still; see README.md."""
    return _update(objects).score


def success(objects: Mapping[str, Any]) -> bool:
    """Everything 1.0 needs, this step, with nothing that ends credit (the embodiment ends the trial once this holds
    for 1 s, when score() reaches 1.0 too)."""
    grade_now = _update(objects)
    return grade_now.state.full and not grade_now.capped


# --- the scripted demo -------------------------------------------------------------------------------------------------
# Both arms clear the heap at once: the left lifts the pry bar off the mallet's handle and lays it down behind the
# heap, the right lifts the Phillips screwdriver off the handle and lays it at the front right. Then the left grips
# the mallet's handle near the head (fingers closing across it, a shallow grip), lifts it, carries it toward the
# coworker (slowly once near them), lays the handle's upper part across the palm with the head just past the thumb
# side, lets go and backs away. Waypoint yaw 0 closes the fingers along world y, -pi/2 along world x.
HOVER = TOP + 0.12
CARRY = (
    TOP + 0.15
)  # the mallet carried with its lowest point about 12 cm up, clear of the heap and the hand
PARK = {"left": (0.20, 0.27, TOP + 0.17), "right": (0.20, -0.27, TOP + 0.17)}
ABOVE_AXIS = (
    0.003  # the TCP this far above a gripped bar's axis, so the pads (13 mm past the TCP) cover it
)

# Where each tool is gripped, in its own frame (a point on its axis).
PRY_GRIP = (0.020, 0.0, 0.0)  # the 20 mm square back end
PHILLIPS_GRIP = (-0.012, 0.0, 0.0132)  # the grip's thickest part
# Along the mallet's handle, 2 cm from the head (1.2 cm from its centre of mass: gripped 11 cm from it, on the
# official stack the mallet swung 45 to 66 degrees head-down in the fingers), with a shallow grip: the TCP
# MALLET_ABOVE_AXIS above the handle's axis, so the pads close on it 5 mm below its widest and their tips stay
# about 7 mm above the palm when it is laid there.
MALLET_GRIP_X = -0.110
MALLET_ABOVE_AXIS = 0.008
# Where the pry bar and the screwdriver are laid aside (TCP, m) and the grip's turn there.
PRY_ASIDE = (0.40, 0.13)
PRY_ASIDE_YAW = -math.pi / 2  # the bar turned to run along -y, between the mallet and the hammer
PHILLIPS_ASIDE = (0.22, -0.30)
SET_DOWN_ABOVE = (
    0.003  # a tool is let go with its lowest point (the handle's underside on the palm) this high
)
HEAD_PAST = 0.003  # the head's inner face this far past the palm's thumb-side edge
APPROACH_ABOVE = 0.02  # the mallet comes straight down onto the palm from this high
SLOW_FROM_Y = 0.05  # the grip's y where the carry slows down (the mallet's outline is then about 26 cm from the palm)
SLOW_CARRY = 4.0  # s for the slow part of the carry: about 0.1 m/s, under 0.2 m/s at its fastest
END_HOLD = 4.0  # s, both arms parked at the end (the mallet settles on the palm, then success must hold 1 s)
GRIP_YAW = -math.pi / 2  # the mallet held as it lay: handle along y, head on the left


def world_point(
    pos: Sequence[float] | np.ndarray, rot: np.ndarray, local: Sequence[float]
) -> np.ndarray:
    return np.asarray(
        np.asarray(pos, dtype=float) + rot @ np.asarray(local, dtype=float), dtype=float
    )


def _outline_center(
    pos: Sequence[float] | np.ndarray, rot: np.ndarray, low: Sequence[float], high: Sequence[float]
) -> np.ndarray:
    corners = np.array(
        [[x, y, z] for x in (low[0], high[0]) for y in (low[1], high[1]) for z in (low[2], high[2])]
    )
    world = corners @ rot.T + np.asarray(pos, dtype=float)
    return np.asarray((world.min(axis=0) + world.max(axis=0)) / 2.0, dtype=float)


HAMMER_START_CENTER = tuple(
    float(v) for v in _outline_center(HAMMER_POS, HAMMER_ROT, HAMMER_LOW, HAMMER_HIGH)[:2]
)
MALLET_START_CENTER = tuple(
    float(v) for v in _outline_center(MALLET_POS, MALLET_ROT, MALLET_LOW, MALLET_HIGH)[:2]
)
COVERS = (PRY_BAR, PHILLIPS)  # the two tools lying across the mallet's handle
COVER_START = {
    PRY_BAR: tuple(float(v) for v in _outline_center(PRY_POS, PRY_ROT, PRY_LOW, PRY_HIGH)[:2]),
    PHILLIPS: tuple(
        float(v)
        for v in _outline_center(PHILLIPS_POS, PHILLIPS_ROT, PHILLIPS_LOW, PHILLIPS_HIGH)[:2]
    ),
}


def mallet_grip(
    pos: Sequence[float] | np.ndarray = MALLET_POS, rot: np.ndarray = MALLET_ROT
) -> np.ndarray:
    """The TCP that grips the mallet's handle at MALLET_GRIP_X, for the mallet at pos/rot (its rest pose by default)."""
    axis = world_point(pos, rot, (MALLET_GRIP_X, 0.0, 0.0))
    return np.asarray(axis + np.array([0.0, 0.0, MALLET_ABOVE_AXIS]), dtype=float)


def handle_clearance(origin: Sequence[float] | np.ndarray, rot: np.ndarray) -> float:
    """How far the handle's underside is above the palm's top at its lowest point over the palm (m)."""
    axis = rot[:, 0]
    lows = []
    for s in np.linspace(HEAD_FACE, float(MALLET_HIGH[0]), 320):
        point = np.asarray(origin, dtype=float) + s * axis
        if in_palm(point):
            lows.append(float(point[2]) - handle_half_height(float(s)))
    return min(lows) - PALM_TOP if lows else math.inf


def hand_pose() -> tuple[np.ndarray, np.ndarray]:
    """The mallet's body origin and rotation when it is let go on the palm: turned as it lay (the grip keeps its
    turn), its axis over the palm's centre line, the head's inner face HEAD_PAST beyond the palm's thumb-side edge,
    and the handle's underside SET_DOWN_ABOVE over the palm at its lowest point there."""
    rot = MALLET_ROT
    axis = rot[:, 0]
    y = PALM_Y[1] + HEAD_PAST - HEAD_FACE * axis[1]
    origin = np.array([PALM_CENTER[0], y, PALM_TOP])
    origin[2] += SET_DOWN_ABOVE - handle_clearance(origin, rot)
    return origin, rot


def place_tcp() -> np.ndarray:
    origin, rot = hand_pose()
    return np.asarray(mallet_grip(origin, rot), dtype=float)


def pry_grip() -> np.ndarray:
    return np.asarray(
        world_point(PRY_POS, PRY_ROT, PRY_GRIP) + np.array([0.0, 0.0, ABOVE_AXIS]), dtype=float
    )


def phillips_grip() -> np.ndarray:
    grip = world_point(PHILLIPS_POS, PHILLIPS_ROT, PHILLIPS_GRIP)
    return np.asarray(grip + np.array([0.0, 0.0, ABOVE_AXIS]), dtype=float)


def _aside_height(
    grip: np.ndarray,
    pos: Sequence[float],
    rot: np.ndarray,
    low: Sequence[float],
    high: Sequence[float],
) -> float:
    """The TCP height that sets a held tool down with its lowest point SET_DOWN_ABOVE over the table (the grip holds
    its tilt)."""
    corners = np.array(
        [
            [x, y, z]
            for x in (low[0], high[0])
            for y in (low[1], high[1])
            for z in (low[2], high[2])
        ],
        dtype=float,
    )
    lowest = float(min(world_point(pos, rot, c)[2] for c in corners))
    return float(grip[2] - lowest + TOP + SET_DOWN_ABOVE)


def _xyz(point: Sequence[float] | np.ndarray) -> tuple[float, float, float]:
    return (float(point[0]), float(point[1]), float(point[2]))


def _pad(
    plan: list[Waypoint], total: float, label: str = "wait for the other arm"
) -> list[Waypoint]:
    """Hold the last waypoint until ``total`` seconds, so both arms' plans end together."""
    spent = sum(w.duration for w in plan)
    if total - spent > 1e-9:
        last = plan[-1]
        plan.append(Waypoint(total - spent, last.pos, last.finger, label, last.yaw))
    return plan


def demo() -> dict[str, list[Waypoint]]:
    pry, ph = pry_grip(), phillips_grip()
    pry_down = _aside_height(pry, PRY_POS, PRY_ROT, PRY_LOW, PRY_HIGH)
    ph_down = _aside_height(ph, PHILLIPS_POS, PHILLIPS_ROT, PHILLIPS_LOW, PHILLIPS_HIGH)
    grip, place = mallet_grip(), place_tcp()
    held = FINGER_CLOSED
    yaw = GRIP_YAW
    left = [
        Waypoint(1.5, (pry[0], pry[1], HOVER), FINGER_OPEN, "above the pry bar", 0.0),
        Waypoint(1.0, (pry[0], pry[1], HOVER), FINGER_OPEN, "let the heap settle", 0.0),
        Waypoint(1.5, _xyz(pry), FINGER_OPEN, "down to the pry bar", 0.0),
        Waypoint(0.8, _xyz(pry), held, "grip the pry bar", 0.0),
        Waypoint(1.2, (pry[0], pry[1], HOVER), held, "lift it off the mallet", 0.0),
        Waypoint(1.8, (*PRY_ASIDE, HOVER), held, "carry it aside", PRY_ASIDE_YAW),
        Waypoint(1.2, (*PRY_ASIDE, pry_down), held, "lay it down", PRY_ASIDE_YAW),
        Waypoint(0.6, (*PRY_ASIDE, pry_down), FINGER_OPEN, "let go of the pry bar", PRY_ASIDE_YAW),
        Waypoint(1.0, (*PRY_ASIDE, HOVER), FINGER_OPEN, "back up", PRY_ASIDE_YAW),
        Waypoint(1.5, (grip[0], grip[1], HOVER), FINGER_OPEN, "above the mallet's handle", yaw),
        Waypoint(1.5, _xyz(grip), FINGER_OPEN, "down to the handle", yaw),
        Waypoint(0.8, _xyz(grip), held, "grip the mallet", yaw),
        Waypoint(0.4, _xyz(grip), held, "settle the grip", yaw),
        Waypoint(1.5, (grip[0], grip[1], HOVER), held, "lift the mallet", yaw),
        Waypoint(1.5, (place[0], SLOW_FROM_Y, CARRY), held, "carry it toward the coworker", yaw),
        Waypoint(SLOW_CARRY, (place[0], place[1], CARRY), held, "slowly over the palm", yaw),
        Waypoint(
            1.5, (place[0], place[1], place[2] + APPROACH_ABOVE), held, "lower it to the palm", yaw
        ),
        Waypoint(1.5, _xyz(place), held, "lay the handle across the palm", yaw),
        Waypoint(0.8, _xyz(place), FINGER_OPEN, "let go of the mallet", yaw),
        Waypoint(1.2, (place[0], place[1], CARRY), FINGER_OPEN, "rise clear", yaw),
        Waypoint(2.0, PARK["left"], FINGER_OPEN, "park", 0.0),
    ]
    right = [
        Waypoint(1.5, (ph[0], ph[1], HOVER), FINGER_OPEN, "above the screwdriver", 0.0),
        Waypoint(1.0, (ph[0], ph[1], HOVER), FINGER_OPEN, "let the heap settle", 0.0),
        Waypoint(1.5, _xyz(ph), FINGER_OPEN, "down to the screwdriver", 0.0),
        Waypoint(0.8, _xyz(ph), held, "grip the screwdriver", 0.0),
        Waypoint(1.2, (ph[0], ph[1], HOVER), held, "lift it off the mallet", 0.0),
        Waypoint(1.8, (*PHILLIPS_ASIDE, HOVER), held, "carry it aside", 0.0),
        Waypoint(1.2, (*PHILLIPS_ASIDE, ph_down), held, "lay it down", 0.0),
        Waypoint(0.6, (*PHILLIPS_ASIDE, ph_down), FINGER_OPEN, "let go of the screwdriver", 0.0),
        Waypoint(1.0, (*PHILLIPS_ASIDE, HOVER), FINGER_OPEN, "back up", 0.0),
        Waypoint(2.0, PARK["right"], FINGER_OPEN, "park", 0.0),
    ]
    total = max(sum(w.duration for w in left), sum(w.duration for w in right))
    left, right = _pad(left, total), _pad(right, total)
    # Parked, hold still a little longer: the trial ends as a success once the full score has held for 1 s.
    end = total + END_HOLD
    return {"left": _pad(left, end, "hold still"), "right": _pad(right, end, "hold still")}
