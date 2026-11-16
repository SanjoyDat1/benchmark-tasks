"""On the station table, in front of the arms: the top of a 10 inch cabinet table saw, set into a cutout in the bench
so that its cast-iron top stands 9.5 cm above the bench. The blade runs left to right (its plane faces the arms) and
the cut feeds toward the right arm, so the blade's back faces the right arm. A coworker has just finished cutting
dadoes, taken the dado set off and put the saw's blade back on: it is raised to full height, the saw is unplugged, and
the table insert is out, so the opening around the blade is open.

Under the opening, behind the blade, is the riving knife's mount: a steel base plate with two locating pins, and a
clamp whose pressure plate faces the pins from the robot's side. The clamp is open: its red handle, in the opening in
front of the blade on the robot's side, is up.

The riving knife, a curved steel plate 2.3 mm thick with two open slots at its foot, stands in a wooden storage holder
on the bench to the right of the saw, oriented as it goes on the saw.
"""

# No `from __future__ import annotations` here: the task loader imports this file without registering it in
# sys.modules, and dataclasses then cannot resolve string annotations.
import functools
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
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP: float = TABLE_TOP[SCENE]
SCORE_OVER = "end"
TABLE_X = (-0.19, 0.91)
TABLE_Y = (-0.65, 0.65)

# --- the saw (world frame: x away from the arms, y to the left, z up; metres) ----------------------------------
# The mount and its use follow SawStop's 10 in Professional Cabinet Saw owner's manual (pp. 32-33): a clamping handle
# lifted fully up opens the clamp, the knife goes in flat against the base plate, located by the positioning pins, and
# the handle lowered completely locks it. The blade and knife sizes are that saw's; the rest is modelled (README).
SAW_TOP = TOP + 0.095  # the top surface, 84 cm from the floor (the real saw's top is about 86 cm up)
TOP_PLATE = 0.020  # the cast-iron top, 20 mm thick
FOOTPRINT_X = (0.105, 0.590)  # the modelled part of the top: 485 x 485 mm
FOOTPRINT_Y = (-0.2625, 0.2225)
THROAT_X = (0.155, 0.295)  # the opening, with the table insert out: 140 x 340 mm
THROAT_Y = (-0.225, 0.115)
WALL = 0.010  # the cabinet's walls, from the bench up to the top's underside
BLADE_X = 0.250  # the blade's mid-plane (x)
ARBOR_Y = -0.020
BLADE_R = 0.127  # 10 in (254 mm) blade
BLADE_HEIGHT = 0.0794  # raised fully: 3-1/8 in of blade above the top
ARBOR_Z = SAW_TOP + BLADE_HEIGHT - BLADE_R
BLADE_KERF = 0.003  # the saw's own blade: 3 mm kerf on a 2 mm plate (its 2.3 mm knife goes between)
BLADE_PLATE = 0.002  # ... the plate (the carbide tips, BLADE_KERF wide, cut the kerf)
TOOTH_BAND = 0.008  # the carbide tips and their gullets: the blade's outer 8 mm
TEETH = 40

# --- the riving knife mount (under the opening, behind the blade) ------------------------------------------------
KNIFE_T = 0.0023  # SawStop's riving knife: 2.3 mm (0.090 in)
BASE_X = (
    BLADE_X + KNIFE_T / 2,
    BLADE_X + KNIFE_T / 2 + 0.010,
)  # the base plate: its face is the knife's +x face
BASE_Y = (-0.222, -0.152)
BASE_Z = (TOP + 0.028, SAW_TOP - TOP_PLATE - 0.002)
PIN_R = 0.003
PIN_LEN = 0.002  # short studs from the base plate: they end inside the knife's thickness
PIN_Z = ARBOR_Z - 0.005
PIN_Y = (-0.164, -0.187)  # the knife's centre of mass sits 6 mm behind the front pin: it stands on them
SLOT_W = 0.007  # the knife's slots: 7 mm for 6 mm pins
SLOT_DEPTH = 0.014
TANG_Y = (-0.215, -0.154)
SLOT_TOP = PIN_Z + PIN_R  # the slots' closed ends sit on the pins when the knife is home
TANG_BOTTOM = SLOT_TOP - SLOT_DEPTH
KNIFE_GAP = 0.005  # knife to blade, radially (Bosch: 3 to 8 mm)
KNIFE_ARC_R = BLADE_R + KNIFE_GAP
KNIFE_TOP = SAW_TOP + BLADE_HEIGHT - 0.003  # 3 mm below the top of the teeth
BACK_TOP = SAW_TOP + 0.035  # the knife's back: level, 35 mm above the top
STRIP = 0.005

# The knife's body frame: origin at the centre of its tang plate (the slotted foot's upper part), axes along the
# world's when it is home. Its parts, in that frame (centre, size). The front edge follows the blade 5 mm out.
KNIFE_HOME = (BLADE_X, (TANG_Y[0] + TANG_Y[1]) / 2, (SLOT_TOP + SAW_TOP) / 2)
_TANG_SIZE = (KNIFE_T, TANG_Y[1] - TANG_Y[0], SAW_TOP - SLOT_TOP)
_FINGERS_Y = (
    (PIN_Y[0] + SLOT_W / 2, TANG_Y[1]),
    (PIN_Y[1] + SLOT_W / 2, PIN_Y[0] - SLOT_W / 2),
    (TANG_Y[0], PIN_Y[1] - SLOT_W / 2),
)


def _arc_z(y: float) -> float:
    """Height of the knife's front edge (5 mm out from the blade) at y; -inf where the arc doesn't reach."""
    d = KNIFE_ARC_R**2 - (y - ARBOR_Y) ** 2
    return ARBOR_Z + math.sqrt(d) if d > 0 else -math.inf


def _top_edge(y: float) -> float:
    """The knife's top edge: level behind its front tip, down to its level back, then down to the top at the rear."""
    if y >= -0.110:
        return KNIFE_TOP
    if y >= -0.155:
        return KNIFE_TOP + (BACK_TOP - KNIFE_TOP) * (-0.110 - y) / 0.045
    if y >= -0.205:
        return BACK_TOP
    return BACK_TOP + (SAW_TOP + 0.012 - BACK_TOP) * (-0.205 - y) / 0.010


KNIFE_FRONT = ARBOR_Y - math.sqrt(KNIFE_ARC_R**2 - (KNIFE_TOP - ARBOR_Z) ** 2)  # where the front edge meets the top
_CROSSING = ARBOR_Y - math.sqrt(KNIFE_ARC_R**2 - (SAW_TOP - ARBOR_Z) ** 2)  # where it meets the saw's top surface


def _knife_world_parts() -> list[tuple[tuple[float, float, float], tuple[float, float, float]]]:
    """(centre, size) of every box of the knife when it is home, world frame."""
    parts = []
    for y0, y1 in _FINGERS_Y:  # the foot, between and around the two slots
        parts.append(
            (
                (BLADE_X, (y0 + y1) / 2, (TANG_BOTTOM + SLOT_TOP) / 2),
                (KNIFE_T, y1 - y0, SLOT_TOP - TANG_BOTTOM),
            )
        )
    y = round(KNIFE_FRONT, 3)
    while y > TANG_Y[0] + 1e-9:  # above the top: 5 mm strips from the front tip back
        y0, y1 = max(y - STRIP, TANG_Y[0]), y
        low = max(SAW_TOP, _arc_z(y1) if y1 > _CROSSING else SAW_TOP)
        high = min(_top_edge(y0), _top_edge(y1))
        if high - low > 0.002:
            parts.append(((BLADE_X, (y0 + y1) / 2, (low + high) / 2), (KNIFE_T, y1 - y0, high - low)))
        y -= STRIP
    return parts


KNIFE_PARTS = tuple(
    (tuple(float(c - h) for c, h in zip(centre, KNIFE_HOME, strict=True)), size)
    for centre, size in _knife_world_parts()
)  # in the knife's frame; the tang plate itself is the object's own box
_ALL_KNIFE = ((0.0, 0.0, 0.0), _TANG_SIZE), *KNIFE_PARTS
KNIFE_LOW = np.min([np.subtract(c, np.divide(s, 2)) for c, s in _ALL_KNIFE], axis=0)
KNIFE_HIGH = np.max([np.add(c, np.divide(s, 2)) for c, s in _ALL_KNIFE], axis=0)
KNIFE_BOX_CENTER = (KNIFE_LOW + KNIFE_HIGH) / 2  # the knife's outline centre, its own frame
_KNIFE_CENTRES = np.array([c for c, _ in _ALL_KNIFE], dtype=float)  # every box of the knife, its own frame
_KNIFE_HALVES = np.array([s for _, s in _ALL_KNIFE], dtype=float) / 2
KNIFE_MASS = round(sum(s[0] * s[1] * s[2] for _, s in _ALL_KNIFE) * 7850.0, 3)  # steel


def turn_y(angle: float) -> np.ndarray:
    """The rotation by ``angle`` (rad) about +y."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def turn_x(angle: float) -> np.ndarray:
    """The rotation by ``angle`` (rad) about +x."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# --- the clamp lever: one rigid body (a cross-shaft, the pressure plate, the handle and the spring arm) ---------
# It turns about a hinge along y. Lowering the handle swings the pressure plate (the jaw) onto the knife and presses it
# flat against the base plate; lifting the handle swings it back. The plate's face is a 2 mm strip just below the hinge
# axis, so near the knife it acts like a cam close to its high point: it advances only a few tenths of a millimetre for
# the lever's last tenths of a radian, which turns a modest torque into a firm clamp. An over-centre spring between a
# seat on the casting and the lever's spring arm, acting along a fixed line, makes the lever bistable: from the handle's
# up stop to its dead centre, 70% of the way down, the spring resists and pushes it back up; past the centre it snaps
# the handle down and holds the plate on the knife. The real handle turns a cam that pushes a separate plate across.
CLAMP_FORCE = 100.0  # N on a home knife at the least (README: 2 faces x 0.2 steel friction x 100 N holds a 40 N pull)
JAW_DX = 0.015  # the plate's face ahead (+x) of the hinge axis, on a home knife
JAW_FACE_Z = (-0.0035, -0.0015)  # the face: a 2 mm strip just below the axis (all of it swings back as it opens)
JAW_Y = (-0.209, -0.164)
JAW_HEIGHT = 0.008  # the plate meets the knife's foot plate 8 mm above the slots' ends
SHAFT_Y = (-0.212, 0.072)
SHAFT = (
    BLADE_X - KNIFE_T / 2 - JAW_DX,
    (SHAFT_Y[0] + SHAFT_Y[1]) / 2,
    SLOT_TOP + JAW_HEIGHT - (JAW_FACE_Z[0] + JAW_FACE_Z[1]) / 2,
)
OPEN_TURN = 0.60  # rad: the handle up: the lever this far round (+y) from where the plate rests on a home knife
CENTRE_TURN = 0.18  # rad, the same way: the spring's dead centre, 70% of the way from up to locked
CLOSED_TURN = -0.25  # rad: with no knife the lever goes on past the cam's high point to its stop
HANDLE_ARM = 0.050  # the handle's pad centre, behind (-x) the hinge axis
HANDLE_Y = (0.048, 0.072)
SPRING_Y = 0.060  # the spring works in the plane y = 0.06, under the handle
SPRING_ARM = 0.020  # the spring's end on the lever: 20 mm from the hinge axis
SPRING_SEAT = 0.032  # its seat on the casting: 32 mm from the hinge axis, on the spring's line
SPRING_TILT = -0.30  # rad: the spring's line leans this far from straight down toward the robot (-x), off the blade
HOLD_TORQUE = 0.10  # N m: with the handle up, the spring holds it there this hard
SPRING_DAMPING = 20.0  # N s/m along the spring's line
HINGE_DAMPING = 0.05  # N m s/rad: the lever's bearing, so it snaps over in a fraction of a second, not instantly
LEVER_MASS = 0.25  # steel: the shaft, the plate and the pressed-steel handle and spring arm
SPRING_LINE = (math.sin(SPRING_TILT), 0.0, -math.cos(SPRING_TILT))  # the spring acts along this (unit, world)


def _layout(x: float, y: float, z: float) -> tuple[float, float, float]:
    """A point given in the locked layout (world axes, origin on the hinge axis; y in world coordinates)."""
    return (x, y - SHAFT[1], z)


_SPRING_END = _layout(
    SPRING_ARM * math.sin(SPRING_TILT + CENTRE_TURN), SPRING_Y, -SPRING_ARM * math.cos(SPRING_TILT + CENTRE_TURN)
)  # at the dead centre the arm points along the spring's line
# The lever's parts besides the shaft (its object box), in the locked layout: (name, centre, size, turn about y).
_LOCKED_PARTS = (
    (
        "jaw",
        _layout(JAW_DX - 0.005, (JAW_Y[0] + JAW_Y[1]) / 2, (JAW_FACE_Z[0] + JAW_FACE_Z[1]) / 2),
        (0.010, JAW_Y[1] - JAW_Y[0], JAW_FACE_Z[1] - JAW_FACE_Z[0]),
        0.0,
    ),
    (
        "arm",
        _layout(-0.022, (HANDLE_Y[0] + HANDLE_Y[1]) / 2, 0.0),
        (0.036, HANDLE_Y[1] - HANDLE_Y[0], 0.006),
        0.0,
    ),
    (  # the handle's red pad: level when the handle is up
        "handle",
        _layout(-HANDLE_ARM, (HANDLE_Y[0] + HANDLE_Y[1]) / 2, 0.001),
        (0.020, HANDLE_Y[1] - HANDLE_Y[0], 0.008),
        -OPEN_TURN,
    ),
    (
        "spring_arm",
        (_SPRING_END[0] / 2, _SPRING_END[1], _SPRING_END[2] / 2),
        (0.006, 0.008, SPRING_ARM + 0.004),
        -CENTRE_TURN - SPRING_TILT,
    ),
)
_OPEN = turn_y(OPEN_TURN)
# The same parts in the lever's own frame: world axes when the handle is up (as it is spawned), origin on the axis.
LEVER_PARTS = tuple(
    (name, tuple(float(v) for v in _OPEN @ np.asarray(centre)), size, turn + OPEN_TURN)
    for name, centre, size, turn in _LOCKED_PARTS
)
SPRING_END = tuple(float(v) for v in _OPEN @ np.asarray(_SPRING_END))  # on the lever, its own frame
SPRING_SEAT_AT = tuple(
    float(c + SPRING_SEAT * u) for c, u in zip((SHAFT[0], SPRING_Y, SHAFT[2]), SPRING_LINE, strict=True)
)  # on the casting, world frame


def _box_corners(centre: Sequence[float], size: Sequence[float], turn: float = 0.0, axis: str = "y") -> np.ndarray:
    """A box's eight corners: ``size``, turned ``turn`` about ``axis`` ("x" or "y"), then placed at ``centre``."""
    half = np.asarray(size, dtype=float) / 2
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float)
    rot = turn_x(turn) if axis == "x" else turn_y(turn)
    out: np.ndarray = (signs * half) @ rot.T + np.asarray(centre, dtype=float)
    return out


_SHAFT_SIZE = (0.010, SHAFT_Y[1] - SHAFT_Y[0], 0.010)
_LEVER_CORNERS = np.vstack(
    [
        _box_corners((0.0, 0.0, 0.0), _SHAFT_SIZE),
        *(_box_corners(c, s, t) for _, c, s, t in LEVER_PARTS),
    ]
)
LEVER_LOW, LEVER_HIGH = _LEVER_CORNERS.min(axis=0), _LEVER_CORNERS.max(axis=0)
LEVER_BOX_CENTER = (LEVER_LOW + LEVER_HIGH) / 2


def _jaw_reach(theta: float) -> float:
    """How far ahead (+x) of the hinge axis the plate's face reaches with the lever ``theta`` round from locked."""
    corners = _box_corners(_LOCKED_PARTS[0][1], _LOCKED_PARTS[0][2]) @ turn_y(theta).T
    return float(corners[:, 0].max())


# Lever angles (rad, about +y) as read from the lever's orientation: 0 with the handle up, where it is spawned.
LOCKED_ANGLE = -OPEN_TURN  # the plate's face flat on a home knife
CENTRE_ANGLE = CENTRE_TURN - OPEN_TURN  # the dead centre
CLOSED_STOP = CLOSED_TURN - OPEN_TURN  # no knife: the lever's lower limit
OPEN_STOP = 0.0  # the handle up: its upper limit


def _line_position(angle: float) -> tuple[float, float]:
    """Where the spring's lever end is along the spring's line, from its seat, and how fast that changes with the
    lever's angle (m, m/rad)."""
    line = np.asarray(SPRING_LINE)  # no y part: where along y the ends are doesn't matter
    end = turn_y(angle) @ np.asarray(SPRING_END) + np.asarray(SHAFT)
    rate = turn_y(angle) @ np.cross(np.array([0.0, 1.0, 0.0]), np.asarray(SPRING_END))  # d(end)/d(angle)
    return float(line @ (end - np.asarray(SPRING_SEAT_AT))), float(line @ rate)


def _spring_setting() -> tuple[float, float]:
    """The spring's stiffness (N/m, rounded up to 100) and its target (m along its line) that hold the handle up with
    HOLD_TORQUE and press the plate on a home knife with CLAMP_FORCE at the face's lower edge."""
    g_up, rate_up = _line_position(OPEN_STOP)
    g_lock, rate_lock = _line_position(LOCKED_ANGLE)
    lock = CLAMP_FORCE * -JAW_FACE_Z[0]
    # torque = -k (g - target) rate: +HOLD_TORQUE up, -lock locked
    ratio = (HOLD_TORQUE / -rate_up) / (lock / rate_lock)
    target = (ratio * g_lock - g_up) / (ratio - 1.0)
    stiffness = HOLD_TORQUE / ((g_up - target) * -rate_up)
    return 100.0 * math.ceil(stiffness / 100.0), target


SPRING_K, SPRING_TARGET = _spring_setting()


def spring_torque(angle: float) -> float:
    """The spring's torque about +y on the lever at ``angle`` (N m; positive lifts the handle)."""
    g, rate = _line_position(angle)
    return -SPRING_K * (g - SPRING_TARGET) * rate


# --- the knife's storage holder, on the bench to the right of the saw -----------------------------------------
# The knife stands in it exactly as it goes on the saw, shifted: 1.7 mm toward the robot (where it is carried, clear
# of the base plate), 20.5 cm to the right and down to the bench.
CARRY_DX = -0.0017  # carried down the middle of the 3.5 mm gap between the base plate and the open jaw
HOLD_SHIFT = (CARRY_DX, -0.205, TOP + 0.0005 - TANG_BOTTOM)
KNIFE_START = tuple(h + s for h, s in zip(KNIFE_HOME, HOLD_SHIFT, strict=True))
_HX = KNIFE_START[0]
HOLDER_Y = (TANG_Y[0] + HOLD_SHIFT[1] - 0.004, TANG_Y[1] + HOLD_SHIFT[1] + 0.003)
REST_Y = (
    -0.150 + HOLD_SHIFT[1],
    -0.142 + HOLD_SHIFT[1],
)  # a rest under the knife's lower front edge

# --- colours (sRGB) -----------------------------------------------------------------------------------------
IRON = (0.55, 0.56, 0.58)
CABINET = (0.17, 0.17, 0.19)
STEEL = (0.80, 0.81, 0.83)
KNIFE_RGB = (0.62, 0.64, 0.68)
DARK_STEEL = (0.30, 0.30, 0.32)
LEVER_RGB = (0.80, 0.10, 0.08)
WOOD = (0.72, 0.55, 0.35)


def _slab(name: str, x: Sequence[float], y: Sequence[float], z: Sequence[float], rgb: Sequence[float]) -> Box:
    return Box(
        name,
        pos=((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2),
        size=(x[1] - x[0], y[1] - y[0], z[1] - z[0]),
        color=tuple(rgb),
    )


_TZ = (SAW_TOP - TOP_PLATE, SAW_TOP)
_WZ = (TOP, SAW_TOP - TOP_PLATE)
SAW = [
    _slab("saw_top_left", FOOTPRINT_X, (THROAT_Y[1], FOOTPRINT_Y[1]), _TZ, IRON),
    _slab("saw_top_right", FOOTPRINT_X, (FOOTPRINT_Y[0], THROAT_Y[0]), _TZ, IRON),
    _slab("saw_top_near", (FOOTPRINT_X[0], THROAT_X[0]), THROAT_Y, _TZ, IRON),
    _slab("saw_top_far", (THROAT_X[1], FOOTPRINT_X[1]), THROAT_Y, _TZ, IRON),
    _slab("saw_body_near", (FOOTPRINT_X[0], FOOTPRINT_X[0] + WALL), FOOTPRINT_Y, _WZ, CABINET),
    _slab("saw_body_far", (FOOTPRINT_X[1] - WALL, FOOTPRINT_X[1]), FOOTPRINT_Y, _WZ, CABINET),
    _slab("saw_body_left", FOOTPRINT_X, (FOOTPRINT_Y[1] - WALL, FOOTPRINT_Y[1]), _WZ, CABINET),
    _slab("saw_body_right", FOOTPRINT_X, (FOOTPRINT_Y[0], FOOTPRINT_Y[0] + WALL), _WZ, CABINET),
]
BLADE = "saw_blade"
PINS = ("locating_pin_front", "locating_pin_back")
BLADE_SPEC = Cylinder(BLADE, pos=(BLADE_X, ARBOR_Y, ARBOR_Z), radius=BLADE_R, height=BLADE_PLATE, color=STEEL)
PIN_SPECS = tuple(
    Cylinder(name, pos=(BASE_X[0] - PIN_LEN / 2, y, PIN_Z), radius=PIN_R, height=PIN_LEN, color=STEEL)
    for name, y in zip(PINS, PIN_Y, strict=True)
)
MOUNT = [
    BLADE_SPEC,
    _slab("knife_base_plate", BASE_X, BASE_Y, BASE_Z, DARK_STEEL),
    *PIN_SPECS,
    _slab(
        "spring_seat",
        (SPRING_SEAT_AT[0] - 0.004, SPRING_SEAT_AT[0] + 0.004),
        (SPRING_Y - 0.004, SPRING_Y + 0.004),
        (SPRING_SEAT_AT[2] - 0.006, SPRING_SEAT_AT[2]),
        DARK_STEEL,
    ),
]
HOLDER = [
    _slab(
        "holder_near",
        (_HX - KNIFE_T / 2 - 0.00035 - 0.017, _HX - KNIFE_T / 2 - 0.00035),
        HOLDER_Y,
        (TOP, TOP + 0.025),
        WOOD,
    ),
    _slab(
        "holder_far",
        (_HX + KNIFE_T / 2 + 0.00035, _HX + KNIFE_T / 2 + 0.00035 + 0.017),
        HOLDER_Y,
        (TOP, TOP + 0.025),
        WOOD,
    ),
    _slab(  # a post under the knife's lower front edge, so it can't tip forward in the slot
        "holder_rest",
        (_HX - 0.006, _HX + 0.006),
        REST_Y,
        (TOP, SAW_TOP + HOLD_SHIFT[2] - 0.0005),
        WOOD,
    ),
]
KNIFE = "riving_knife"
LEVER = "clamp_lever"
RIVING_KNIFE = Box(
    KNIFE, pos=KNIFE_START, size=_TANG_SIZE, mass=KNIFE_MASS, color=KNIFE_RGB
)  # the tang plate; the scene hook adds the foot's fingers and the blade-hugging upper part
CLAMP_LEVER = Box(
    LEVER, pos=SHAFT, size=_SHAFT_SIZE, mass=LEVER_MASS, color=LEVER_RGB
)  # the cross-shaft; the hook adds the jaw, the handle and the spring arm, the hinge and the spring
OBJECTS = [*SAW, *MOUNT, *HOLDER, RIVING_KNIFE, CLAMP_LEVER]


# --- the scene hook: the knife's and lever's shapes, the hinge and the spring, the blade's teeth -----------------
# yam_arena.scene.task_object(spec) turns each task object into an Arena object. Right after that module is imported,
# this wraps task_object; it recognises this file's objects by value (they are frozen dataclasses): the blade and pins
# are spawned with their axes along x (Isaac Lab's CylinderCfg.axis); the knife's spawner, after spawning its tang
# plate, adds the rest of the knife as more colliders of the same rigid body; the lever's adds, first, a
# UsdPhysics.RevoluteJoint along y to the world with limits and a damper (an angular drive with no stiffness) and the
# over-centre spring (a generic UsdPhysics.Joint, every motion free, with one linear drive along the spring's line),
# then its jaw, handle and spring arm. Every added cube has an authored extent, and every added collider gets the task
# objects' rubber-like physics material. Where Isaac Lab isn't installed nothing is installed,
# and the file warns.
_SCENE_MODULE = "yam_arena.scene"
PARTS_FOLDER = "parts"
HINGE = "clamp_hinge"
SPRING = "clamp_spring"
GRIPPY = (1.2, 1.0)  # the framework's friction for task objects (static, dynamic)
# The knife's foot is steel on steel pins and base plate: lightly oiled machine parts, about 0.3. With "min" the pair
# takes the lower of the two, so the round pins can steer the slots onto themselves (rubber-like 1.0 would stick).
STEEL_FRICTION = (0.35, 0.30)


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _materials(
    stage: Any, folder: str, rgb: Sequence[float], friction: Sequence[float] = (1.2, 1.0), combine: str = ""
) -> tuple[Any, Any]:
    """A look and a physics material (static and dynamic ``friction``; PhysX's friction combine mode ``combine``, if
    given, for the pair) under ``folder``."""
    gf = importlib.import_module("pxr.Gf")
    sdf = importlib.import_module("pxr.Sdf")
    physics = importlib.import_module("pxr.UsdPhysics")
    shade = importlib.import_module("pxr.UsdShade")
    look = shade.Material.Define(stage, f"{folder}/look")
    shader = shade.Shader.Define(stage, f"{folder}/look/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", sdf.ValueTypeNames.Color3f).Set(gf.Vec3f(*(_linear(c) for c in rgb)))
    shader.CreateInput("metallic", sdf.ValueTypeNames.Float).Set(0.8)
    shader.CreateInput("roughness", sdf.ValueTypeNames.Float).Set(0.4)
    look.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    grip = shade.Material.Define(stage, f"{folder}/grip")
    api = physics.MaterialAPI.Apply(grip.GetPrim())
    api.CreateStaticFrictionAttr(friction[0])
    api.CreateDynamicFrictionAttr(friction[1])
    api.CreateRestitutionAttr(0.0)
    if combine:
        prim = grip.GetPrim()
        prim.AddAppliedSchema("PhysxMaterialAPI")
        prim.CreateAttribute("physxMaterial:frictionCombineMode", sdf.ValueTypeNames.Token).Set(combine)
    return look, grip


def _add_boxes(
    stage: Any,
    body_path: str,
    parts: Sequence[tuple[str, Sequence[float], Sequence[float], float]],
    rgb: Sequence[float],
    collide: bool = True,
    friction: Sequence[float] = (1.2, 1.0),
    combine: str = "",
    materials: str = "materials",
    axis: str = "y",
) -> list[str]:
    """Cubes under ``body_path``/parts, placed (and turned about ``axis``, "x" or "y") in the body's frame; colliders
    with a material of the given friction (by default the framework's grippy one), kept under ``materials``."""
    gf = importlib.import_module("pxr.Gf")
    geom = importlib.import_module("pxr.UsdGeom")
    physics = importlib.import_module("pxr.UsdPhysics")
    shade = importlib.import_module("pxr.UsdShade")
    folder = f"{body_path}/{PARTS_FOLDER}"
    look, grip = _materials(stage, f"{folder}/{materials}", rgb, friction, combine)
    made = []
    for name, centre, size, turn in parts:
        path = f"{folder}/{name}"
        cube = geom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        # Authored extent: Arena's outlines come from USD bounds, so they don't depend on a computed fallback.
        cube.CreateExtentAttr([gf.Vec3f(-0.5, -0.5, -0.5), gf.Vec3f(0.5, 0.5, 0.5)])
        xform = geom.Xformable(cube.GetPrim())
        xform.AddTranslateOp().Set(gf.Vec3d(*(float(c) for c in centre)))
        half_sin = math.sin(turn / 2)
        imaginary = (half_sin, 0.0, 0.0) if axis == "x" else (0.0, half_sin, 0.0)
        xform.AddOrientOp().Set(gf.Quatf(math.cos(turn / 2), *imaginary))
        xform.AddScaleOp().Set(gf.Vec3f(*(float(s) for s in size)))
        binding = shade.MaterialBindingAPI.Apply(cube.GetPrim())
        binding.Bind(look)
        if collide:
            physics.CollisionAPI.Apply(cube.GetPrim())
            binding.Bind(grip, shade.Tokens.weakerThanDescendants, "physics")
        made.append(path)
    return made


def author_knife(stage: Any, knife_path: str) -> list[str]:
    """The knife's foot fingers (steel on the steel pins: they slide on them) and blade-hugging upper part (the
    framework's grippy material, where it is held), as colliders of the tang plate's body."""
    parts = [(f"part_{i:02d}", centre, size, 0.0) for i, (centre, size) in enumerate(KNIFE_PARTS)]
    foot, rest = parts[: len(_FINGERS_Y)], parts[len(_FINGERS_Y) :]
    made = _add_boxes(stage, knife_path, foot, KNIFE_RGB, friction=STEEL_FRICTION, combine="min", materials="steel")
    return made + _add_boxes(stage, knife_path, rest, KNIFE_RGB)


def author_lever(
    stage: Any, lever_path: str, hinge_path: str, spring_path: str, hinge_world: Sequence[float]
) -> list[str]:
    """The lever's hinge to the world (limits, a damper), the over-centre spring from its seat on the casting to the
    spring arm, then the lever's plate, handle arm, handle and spring arm. The joints come first: if anything fails,
    the lever is left a bare shaft, which the outline check catches."""
    gf = importlib.import_module("pxr.Gf")
    physics = importlib.import_module("pxr.UsdPhysics")
    joint = physics.RevoluteJoint.Define(stage, hinge_path)
    joint.CreateAxisAttr("Y")
    joint.CreateBody1Rel().SetTargets([lever_path])  # no body0: the world
    joint.CreateLocalPos0Attr(gf.Vec3f(*(float(c) for c in hinge_world)))
    joint.CreateLocalRot0Attr(gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLowerLimitAttr(math.degrees(CLOSED_STOP))
    joint.CreateUpperLimitAttr(math.degrees(OPEN_STOP))
    damper = physics.DriveAPI.Apply(joint.GetPrim(), "angular")
    damper.CreateTypeAttr("force")
    damper.CreateStiffnessAttr(0.0)
    damper.CreateDampingAttr(HINGE_DAMPING * math.pi / 180.0)  # USD angular drives take torque per degree/s
    damper.CreateTargetVelocityAttr(0.0)
    # The spring: a generic (D6) joint, every motion free, with one linear drive along the spring's line. Its frames'
    # x axes lie along the line, so the drive pushes or pulls the arm's end along it toward SPRING_TARGET.
    seat = tuple(h + a - c for h, a, c in zip(hinge_world, SPRING_SEAT_AT, SHAFT, strict=True))
    along = math.pi / 2 - SPRING_TILT  # the turn about y that takes x onto the spring's line
    line = gf.Quatf(math.cos(along / 2), 0.0, math.sin(along / 2), 0.0)
    spring = physics.Joint.Define(stage, spring_path)
    spring.CreateBody1Rel().SetTargets([lever_path])  # no body0: the seat is on the (fixed) casting
    spring.CreateLocalPos0Attr(gf.Vec3f(*(float(c) for c in seat)))
    spring.CreateLocalRot0Attr(line)
    spring.CreateLocalPos1Attr(gf.Vec3f(*(float(c) for c in SPRING_END)))
    spring.CreateLocalRot1Attr(line)
    drive = physics.DriveAPI.Apply(spring.GetPrim(), "transX")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(SPRING_K))  # N/m: a linear drive has no degrees in it
    drive.CreateDampingAttr(float(SPRING_DAMPING))
    drive.CreateTargetPositionAttr(float(SPRING_TARGET))
    made = [hinge_path, spring_path]
    made += _add_boxes(stage, lever_path, LEVER_PARTS, LEVER_RGB)
    return made


TOOTH_SIZE = (BLADE_KERF, 0.004, 0.004)  # across, radially (once turned to face out), round the rim
# Each tip is turned to face out, its outer corners on the rim: the blade is BLADE_R round at its tips, as a real
# 10 in blade is, and its tooth band (tooth_distance) holds every tip whole.
TOOTH_CENTRE_R = math.sqrt(BLADE_R**2 - (TOOTH_SIZE[2] / 2) ** 2) - TOOTH_SIZE[1] / 2
TEETH_PARTS = tuple(
    (
        f"tooth_{k:02d}",
        (0.0, TOOTH_CENTRE_R * math.cos(2 * math.pi * k / TEETH), TOOTH_CENTRE_R * math.sin(2 * math.pi * k / TEETH)),
        TOOTH_SIZE,
        2 * math.pi * k / TEETH,
    )
    for k in range(TEETH)
)  # carbide tips round the rim, in the blade prim's frame (world axes; the disc in y-z; each turned about x)


def author_teeth(stage: Any, blade_path: str) -> list[str]:
    """The blade's carbide tips (looks only: no colliders), 0.5 mm proud of the plate's faces. Arena counts them in
    the blade's outline (fixture_size)."""
    return _add_boxes(stage, blade_path, TEETH_PARTS, (0.35, 0.35, 0.38), collide=False, axis="x")


def _env_roots(prim: Any, name: str) -> list[str]:
    """Every environment folder holding ``name``, given that object's prim just spawned (clones included)."""
    stage = prim.GetStage()
    envs = prim.GetPath().GetParentPath().GetParentPath()
    return [str(env.GetPath()) for env in stage.GetPrimAtPath(envs).GetChildren() if env.GetChild(name).IsValid()]


def _world_position(stage: Any, path: str) -> tuple[float, float, float]:
    geom = importlib.import_module("pxr.UsdGeom")
    t = geom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path)).ExtractTranslation()
    return float(t[0]), float(t[1]), float(t[2])


def _wrap(spawn: Callable[..., Any], name: str) -> Callable[..., Any]:
    def spawn_and_build(prim_path: str, cfg: Any, *args: Any, **kwargs: Any) -> Any:
        prim = spawn(prim_path, cfg, *args, **kwargs)
        try:
            stage = prim.GetStage()
            for root in _env_roots(prim, name):
                path = f"{root}/{name}"
                if name == KNIFE:
                    author_knife(stage, path)
                elif name == LEVER:
                    author_lever(
                        stage,
                        path,
                        f"{root}/{HINGE}",
                        f"{root}/{SPRING}",
                        _world_position(stage, path),
                    )
                elif name == BLADE:
                    author_teeth(stage, path)
        except Exception as error:  # a knife or lever without its parts fails the outline check: every step scores 0
            _warn_once(f"build-{name}", f"could not build {name} ({type(error).__name__}: {error})")
        return prim

    spawn_and_build.riving_knife_task = name  # type: ignore[attr-defined]
    return spawn_and_build


def _spawner_cfgs(item: object) -> Iterator[Any]:
    """The spawner configs an Arena object holds (one level deep): objects with a callable ``func`` and a ``size``
    (a box) or a ``radius`` (a cylinder)."""
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
            if callable(getattr(value, "func", None)) and (hasattr(value, "size") or hasattr(value, "radius")):
                yield value


_AXIS_X = (BLADE_SPEC, *PIN_SPECS)
_BUILT = (RIVING_KNIFE, CLAMP_LEVER, BLADE_SPEC)


def _ours(spec: Any, specs: Sequence[Any]) -> bool:
    """Whether ``spec`` is one of ``specs``, by value (frozen dataclasses): a stranger's object of the same name isn't
    touched."""
    try:
        return any(spec == mine for mine in specs)
    except Exception:  # an object that can't be compared isn't ours
        return False


def build_saw(scene: Any) -> bool:
    """Wrap ``scene.task_object`` so the knife, lever, blade and pins come out as built here; True if it wrapped it
    now. It recognises this file's objects by value, so a second load of this file is served by the first wrapper."""
    original = getattr(scene, "task_object", None)
    if original is None or getattr(original, "riving_knife_task", False):
        return False

    def task_object(spec: Any) -> Any:
        made = original(spec)
        axis_x, built = _ours(spec, _AXIS_X), _ours(spec, _BUILT)
        if axis_x or built:
            name = spec.name
            found = 0
            for cfg in _spawner_cfgs(made):
                found += 1
                if axis_x:
                    if hasattr(cfg, "axis"):
                        cfg.axis = "X"
                    else:
                        _warn_once(
                            f"axis-{name}",
                            f"{name}'s spawner has no axis setting; it will stand upright",
                        )
                if built and getattr(cfg.func, "riving_knife_task", None) != name:
                    cfg.func = _wrap(cfg.func, name)
            if not found:
                _warn_once(f"spawner-{name}", f"found no spawner for {name}; the saw will be wrong")
        return made

    task_object.riving_knife_task = True  # type: ignore[attr-defined]
    scene.task_object = task_object
    return True


class _SceneWatcher(importlib.abc.MetaPathFinder):
    """Waits for yam_arena.scene to be imported, lets it load normally, then applies build_saw. It answers for that
    one module name only, and removes itself once the module has loaded."""

    marker = "fit_riving_knife.scene_watcher"

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
            spec.loader = _BuildAfterLoad(spec.loader, self)
        return spec

    def disarm(self) -> None:
        if self in sys.meta_path:
            sys.meta_path.remove(self)


class _BuildAfterLoad(importlib.abc.Loader):
    """The module's own loader, plus build_saw once the module has run."""

    def __init__(self, inner: importlib.abc.Loader, watcher: _SceneWatcher) -> None:
        self._inner = inner
        self._watcher = watcher

    def create_module(self, spec: importlib.machinery.ModuleSpec) -> types.ModuleType | None:
        return self._inner.create_module(spec)

    def exec_module(self, module: types.ModuleType) -> None:
        try:  # whatever happens, the watcher goes: it answers for this one import only
            self._inner.exec_module(module)
            try:
                build_saw(module)
            except Exception as error:  # never break the scene module's import; the outline checks then score 0
                self._watcher.disarm()  # before warning: with warnings as errors the warning raises
                _warn_once("saw", f"could not hook the scene builder ({type(error).__name__}: {error})")
        finally:
            self._watcher.disarm()

    def __getattr__(self, name: str) -> Any:  # get_source and friends, for tracebacks
        if name.startswith("__") or name in ("_inner", "_watcher"):  # a copy made without __init__
            raise AttributeError(name)
        return getattr(self._inner, name)


def install_saw() -> None:
    """Wrap the scene builder now if it is already imported; otherwise wait for its import (once)."""
    scene = sys.modules.get(_SCENE_MODULE)
    if scene is not None:
        build_saw(scene)
        return
    if not any(getattr(finder, "marker", None) == _SceneWatcher.marker for finder in sys.meta_path):
        sys.meta_path.insert(0, _SceneWatcher())


# --- geometry for scoring ---------------------------------------------------------------------------------------


def rotation(quat_xyzw: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def turn_angle(rot: np.ndarray) -> float:
    """How far (rad) a rotation turns from the identity."""
    return math.acos(max(-1.0, min(1.0, (float(np.trace(rot)) - 1.0) / 2.0)))


def lever_angle(rot: np.ndarray) -> float:
    """The lever's turn about the hinge (+y, rad): 0 with the handle up, negative as it is lowered."""
    return math.atan2(float(rot[0, 2]), float(rot[2, 2]))


def tooth_distances(points: np.ndarray) -> np.ndarray:
    """tooth_distance for an (n, 3) array of points."""
    p = np.asarray(points, dtype=float)
    across = np.maximum(0.0, np.abs(p[:, 0] - BLADE_X) - BLADE_KERF / 2)
    r = np.hypot(p[:, 1] - ARBOR_Y, p[:, 2] - ARBOR_Z)
    radial = np.where(r > BLADE_R, r - BLADE_R, np.maximum(0.0, BLADE_R - TOOTH_BAND - r))
    out: np.ndarray = np.hypot(across, radial)
    return out


def tooth_distance(point: Sequence[float] | np.ndarray) -> float:
    """Distance (m) from a point to the blade's tooth band: its outer TOOTH_BAND, out to the tips' corners on the
    rim, BLADE_KERF thick (the tips' width)."""
    p = np.asarray(point, dtype=float)
    across = max(0.0, abs(float(p[0]) - BLADE_X) - BLADE_KERF / 2)
    r = math.hypot(float(p[1]) - ARBOR_Y, float(p[2]) - ARBOR_Z)
    radial = r - BLADE_R if r > BLADE_R else max(0.0, BLADE_R - TOOTH_BAND - r)
    return math.hypot(across, radial)


TEETH_RESOLUTION = 0.0001  # boxes are measured to the band to within 0.1 mm, and never as further than they are
_HALF_SIGNS = np.array([[1.0, 1.0, 1.0], [1.0, 1.0, -1.0], [1.0, -1.0, 1.0], [1.0, -1.0, -1.0]])  # and their negatives
_ALL_SIGNS = np.vstack([_HALF_SIGNS, -_HALF_SIGNS])  # a box's eight corners
_MAX_ROUNDS, _MAX_BOXES = 80, 200_000  # band_gap's bounds on its work (never reached in practice; if they were, it
# would stop and answer with its lower bound: nearer, never further, than the truth)


def _band_bounds(centres: np.ndarray, edges: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """For boxes {centre + edges @ s : s in [-1, 1]^3} (``edges``: (n, 3, 3), half an edge per column, so flat faces
    and points are boxes too): their centres' distance to the tooth band, and a lower bound on every point's, the
    larger of two: the centre's less the box's reach, and the band's distance from the box's span across the blade
    (x) and out from the arbor."""
    d = tooth_distances(centres)
    corners = np.einsum("sj,nij->nsi", _HALF_SIGNS, edges)
    reach = np.linalg.norm(corners, axis=2).max(axis=1)
    span_x = np.abs(edges[:, 0, :]).sum(axis=1)
    span_r = np.linalg.norm(corners[:, :, 1:], axis=2).max(axis=1)
    across = np.maximum(0.0, np.abs(centres[:, 0] - BLADE_X) - span_x - BLADE_KERF / 2)
    r = np.hypot(centres[:, 1] - ARBOR_Y, centres[:, 2] - ARBOR_Z)
    radial = np.maximum(0.0, np.maximum(r - span_r - BLADE_R, BLADE_R - TOOTH_BAND - (r + span_r)))
    lower: np.ndarray = np.maximum(d - reach, np.hypot(across, radial))
    return d, lower


def band_gap(centres: np.ndarray, edges: np.ndarray, limit: float) -> float:
    """The distance (m) from the nearest point of some boxes (see _band_bounds) to the blade's tooth band (its outer
    TOOTH_BAND out to the tips' corners on the rim, BLADE_KERF thick): never more than the true distance, and at most
    TEETH_RESOLUTION less; about ``limit`` when the truth is more. Branch and bound: a box that may hold a point nearer
    than any found yet is halved, across the side along which the band's distance changes most, until it can't."""
    c = np.asarray(centres, dtype=float).reshape(-1, 3)
    e = np.asarray(edges, dtype=float).reshape(-1, 3, 3)
    best, floor = float(limit), math.inf
    for _ in range(_MAX_ROUNDS):
        if not len(c):
            break
        d, lower = _band_bounds(c, e)
        best = min(best, float(d.min()))
        keep = lower < best - TEETH_RESOLUTION
        if not keep.all():
            floor = min(floor, float(lower[~keep].min()))
        c, e = c[keep], e[keep]
        if len(c) > _MAX_BOXES // 2:
            break
        # Within the blade's thickness, how far across doesn't change the distance: split only out from the arbor.
        inside = np.abs(c[:, 0] - BLADE_X) + np.abs(e[:, 0, :]).sum(axis=1) <= BLADE_KERF / 2
        weight = np.linalg.norm(e[:, 1:, :], axis=1) + np.where(inside[:, None], 0.0, np.abs(e[:, 0, :]))
        k = np.argmax(weight, axis=1)
        rows = np.arange(len(c))
        half = e[rows, :, k] / 2
        e = e.copy()
        e[rows, :, k] = half
        c, e = np.concatenate([c - half, c + half]), np.concatenate([e, e])
    if len(c):  # stopped early: what is left counts as near as its bound allows
        floor = min(floor, float(_band_bounds(c, e)[1].min()))
    return max(0.0, min(best, floor))


def knife_boxes(origin: np.ndarray, rot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The knife's boxes, placed (for band_gap): centres and half-edges."""
    centres: np.ndarray = _KNIFE_CENTRES @ rot.T + origin
    edges: np.ndarray = rot[None, :, :] * _KNIFE_HALVES[:, None, :]
    return centres, edges


def _corners(boxes: tuple[np.ndarray, np.ndarray]) -> np.ndarray:
    centres, edges = boxes
    out: np.ndarray = centres[:, None, :] + np.einsum("sj,nij->nsi", _ALL_SIGNS, edges)
    return out


def travel(before: tuple[np.ndarray, np.ndarray] | None, now: tuple[np.ndarray, np.ndarray]) -> float:
    """How far (m) any point of the boxes went in a straight line since ``before`` (the same boxes then; 0 if none)."""
    if before is None or before[0].shape != now[0].shape:
        return 0.0
    return float(np.linalg.norm(_corners(now) - _corners(before), axis=2).max(initial=0.0))


def swept(
    before: tuple[np.ndarray, np.ndarray] | None, now: tuple[np.ndarray, np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    """The boxes ``now`` and on their way from ``before``, every point of each moving in a straight line, placed every
    SWEEP_STEP or closer (at most SWEEP_MAX times: a longer jump voids the trial before this is asked)."""
    if before is None or before[0].shape != now[0].shape:
        return now
    n = max(1, min(SWEEP_MAX, math.ceil(travel(before, now) / SWEEP_STEP)))
    t = np.linspace(0.0, 1.0, n + 1)
    centres = before[0][None] + t[:, None, None] * (now[0] - before[0])[None]
    edges = before[1][None] + t[:, None, None, None] * (now[1] - before[1])[None]
    return centres.reshape(-1, 3), edges.reshape(-1, 3, 3)


def point_box_distance(point: Any, low: Any, high: Any) -> float:
    p = np.asarray(point, dtype=float)
    return float(np.linalg.norm(np.maximum(np.maximum(np.asarray(low) - p, p - np.asarray(high)), 0.0)))


# --- scoring rules ------------------------------------------------------------------------------------------------
PINS_ALONG = 0.001  # both pins in their slots: the knife's frame within 1 mm of home along y (0.5 mm slot play)
PINS_DEPTH = SLOT_DEPTH - 2 * PIN_R  # ... and at most 8 mm above home: the pins still inside the 14 mm slots
PINS_ACROSS = (
    -PIN_LEN + 0.0001,
    0.0005,
)  # ... and over the pins' 2 mm length (off the base plate by at most that)
PINS_TURN = math.radians(5.0)
SEAT_DEPTH = 0.001  # seated: both slots' ends on the pins (within 1 mm of home, up and down)
SEAT_TURN = math.radians(3.0)
PRESSED = 0.0004  # locked: flat on the base plate (a 0.4 mm feeler gauge won't go between them)
LOCK_TURN = math.radians(1.0)  # ... turned less than 1 degree
JAW_ON_KNIFE = PRESSED  # ... the handle past its dead centre and the plate on the knife (the same 0.4 mm gauge)
# (no more than that short of the knife; "into" it is fine: under the clamp the simulated contacts give, and the plate
# rested 0.36 mm into the knife in a GPU run. With the knife pressed home the cam can't put it more than 0.8 mm in.)
TOOTH_CLEAR = 0.005  # a gripper's tool point or finger pads this close to the tooth band touches the teeth
KNIFE_TOOTH_CLEAR = 0.002  # the knife itself this close to the tooth band touches them (home: 5 mm)
SWEEP_STEP = 0.002  # the straight path since the last step is checked this often: a crossing of the band shows
SWEEP_MAX = 200  # ... within 1 mm (so under either clearance). Moving further than this many steps' worth (40 cm in
JUMP = SWEEP_MAX * SWEEP_STEP  # one control step, 8 m/s) is no motion of these arms: a reset the file didn't see or a
# broken reading, and it voids the trial
TEETH_REPORT = 0.05  # knife_teeth_gap and teeth_gap measure to this far by default (the scorer only to its limits)
CLEAR = 0.03  # let go: every tool point at least this far from the knife's and the lever's outlines
FINGER_CLEAR = 0.01  # ... and every point of its finger pads at least this far
MAX_SPEED, MAX_SPIN = 0.01, 0.5
STEP_S = 0.05  # one control step at 20 Hz: still means moving less than MAX_SPEED and MAX_SPIN from step to step,
# judged from the poses, not the reported velocities: clamped and let go, the knife and lever didn't change (0.01 mm,
# 0.0001 rad) for 6 s in a GPU run while the simulator reported 1.0 cm/s and 0.89 rad/s
HOLD_STEPS = 20  # 1 s at 20 Hz
FALLEN_BELOW = 0.0  # the knife's centre below the bench top: off the saw and holder, on the floor
TEETH_CAP = 0.5
BUILT_TOLERANCE = 0.0005  # the reported outlines must match the shapes built here this closely
QUAT_TOLERANCE = 1e-3  # a reported orientation must be a unit quaternion to this
PAD_TIP_PAST_TCP = 0.013
PAD_LENGTH = 0.030
PAD_WIDTH = 0.027


class _State(Protocol):
    pos: Any
    quat_xyzw: Any
    lin_vel: Any
    ang_vel: Any
    low: Any
    high: Any


_STATE_FIELDS = ("pos", "quat_xyzw", "lin_vel", "ang_vel", "low", "high")
_STATE_SIZES = (3, 4, 3, 3, 3, 3)  # how many numbers each holds
_BAD_STATE = (
    LookupError,
    AttributeError,
    TypeError,
    ValueError,
    ArithmeticError,
    RuntimeError,
)  # a state that can't be read


def finite(state: _State) -> bool:
    """Every number the simulator reported for an object is finite, and its orientation is a unit quaternion."""
    try:
        values = [np.asarray(getattr(state, f), dtype=float) for f in _STATE_FIELDS]
        if any(v.size != n for v, n in zip(values, _STATE_SIZES, strict=True)):
            return False
        if not all(bool(np.all(np.isfinite(v))) for v in values):
            return False
        return abs(float(np.linalg.norm(values[1])) - 1.0) < QUAT_TOLERANCE
    except _BAD_STATE:
        return False


def fixture_size(spec: Any) -> np.ndarray:
    """A fixed cylinder's outline as Arena reports it: lying along x, and for the blade with its teeth, which sit
    under its prim (Arena's outline takes in every default-purpose shape there): the tips' 3 mm across the 2 mm
    plate, and within its rim."""
    size = np.array([spec.height, 2 * spec.radius, 2 * spec.radius])
    if spec == BLADE_SPEC:
        corners = np.vstack([_box_corners(c, s, t, axis="x") for _, c, s, t in TEETH_PARTS])
        size = np.maximum(size, corners.max(axis=0) - corners.min(axis=0))  # symmetric: the centre doesn't move
    out: np.ndarray = size
    return out


def _fixture_ok(spec: Any, state: _State | None) -> bool:
    """A fixed cylinder (the blade, a pin) is where it should be, unturned, lying along x as the hook spawns it."""
    if state is None or not finite(state):
        return False
    low, high = np.asarray(state.low, dtype=float), np.asarray(state.high, dtype=float)
    size = fixture_size(spec)
    return bool(
        np.all(np.abs((high - low) - size) <= BUILT_TOLERANCE)
        and np.all(np.abs((low + high) / 2 - np.asarray(spec.pos)) <= BUILT_TOLERANCE)
        and turn_angle(rotation(state.quat_xyzw)) <= math.radians(0.5)
    )


def fixtures_ok(objects: Mapping[str, Any]) -> bool:
    """The blade and both pins came out as built here (an upright blade, say, would make the task easier)."""
    return all(_fixture_ok(spec, objects.get(spec.name)) for spec in (BLADE_SPEC, *PIN_SPECS))


@dataclass(frozen=True)
class KnifeView:
    """The knife as the simulator reports it: its frame's position and turn, and its outline."""

    origin: np.ndarray
    rot: np.ndarray
    low: np.ndarray
    high: np.ndarray

    @staticmethod
    def of(state: _State) -> "KnifeView":
        low, high = np.asarray(state.low, dtype=float), np.asarray(state.high, dtype=float)
        rot = rotation(state.quat_xyzw)
        # The outline is the world-aligned box of the knife's own-frame box turned into the world, so its centre is
        # the own-frame box centre turned and placed: the frame's origin follows.
        origin = (low + high) / 2 - rot @ KNIFE_BOX_CENTER
        return KnifeView(origin, rot, low, high)

    @property
    def offset(self) -> np.ndarray:
        offset: np.ndarray = self.origin - np.asarray(KNIFE_HOME)
        return offset

    @property
    def turn(self) -> float:
        return turn_angle(self.rot)

    @property
    def built(self) -> bool:
        """The reported outline is this file's knife, turned: the scene hook built it."""
        expected = np.abs(self.rot) @ (KNIFE_HIGH - KNIFE_LOW)
        return bool(np.all(np.abs((self.high - self.low) - expected) <= BUILT_TOLERANCE))

    def _over_pins(self) -> bool:
        d = self.offset
        return bool(PINS_ACROSS[0] <= d[0] <= PINS_ACROSS[1] and abs(d[1]) <= PINS_ALONG)

    @property
    def on_pins(self) -> bool:
        """Both pins inside their slots, anywhere up the slots."""
        d = self.offset
        return bool(self._over_pins() and -SEAT_DEPTH <= d[2] <= PINS_DEPTH and self.turn <= PINS_TURN)

    @property
    def seated(self) -> bool:
        """Both slots' ends on the pins."""
        return bool(self._over_pins() and abs(float(self.offset[2])) <= SEAT_DEPTH and self.turn <= SEAT_TURN)

    @property
    def pressed(self) -> bool:
        return bool(abs(float(self.offset[0])) <= PRESSED and self.turn <= LOCK_TURN)

    @property
    def fallen(self) -> bool:
        c = (self.low + self.high) / 2
        return bool(
            not (TABLE_X[0] <= c[0] <= TABLE_X[1] and TABLE_Y[0] <= c[1] <= TABLE_Y[1]) or c[2] < TOP + FALLEN_BELOW
        )


@dataclass(frozen=True)
class LeverView:
    angle: float
    rot: np.ndarray
    low: np.ndarray
    high: np.ndarray

    @staticmethod
    def of(state: _State) -> "LeverView":
        rot = rotation(state.quat_xyzw)
        return LeverView(lever_angle(rot), rot, np.asarray(state.low, dtype=float), np.asarray(state.high, dtype=float))

    @property
    def built(self) -> bool:
        """The reported outline is this file's lever, turned, and centred where its hinge holds it."""
        expected = np.abs(self.rot) @ (LEVER_HIGH - LEVER_LOW)
        centre = self.rot @ LEVER_BOX_CENTER + np.asarray(SHAFT)
        return bool(
            np.all(np.abs((self.high - self.low) - expected) <= BUILT_TOLERANCE)
            and np.all(np.abs((self.low + self.high) / 2 - centre) <= BUILT_TOLERANCE)
        )

    @property
    def past_centre(self) -> bool:
        return self.angle < CENTRE_ANGLE

    def jaw_gap(self, across: float) -> float:
        """The gap (m) between the plate's face and the knife's face, the knife ``across`` m off home toward the robot
        (negative: the plate would be into the knife)."""
        knife_face = BLADE_X - KNIFE_T / 2 + across
        return knife_face - (SHAFT[0] + _jaw_reach(self.angle + OPEN_TURN))


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
                    points.append(tcp + finger * self.opening * closing + along * approach + edge * PAD_WIDTH * across)
        return points

    def boxes(self) -> tuple[np.ndarray, np.ndarray]:
        """Its tool point and its two finger pads' faces (PAD_LENGTH along the approach, PAD_WIDTH across, the ones
        finger_points covers), as boxes for band_gap."""
        tcp = np.asarray(self.tcp, dtype=float)
        approach = np.asarray(self.approach, dtype=float)
        closing = np.asarray(self.closing, dtype=float)
        across = np.cross(approach, closing)
        middle = tcp + (PAD_TIP_PAST_TCP - PAD_LENGTH / 2) * approach
        pad = np.column_stack([approach * PAD_LENGTH / 2, across * PAD_WIDTH / 2, np.zeros(3)])
        centres = np.array([tcp, middle - self.opening / 2 * closing, middle + self.opening / 2 * closing])
        return centres, np.array([np.zeros((3, 3)), pad, pad])

    @property
    def finite(self) -> bool:
        values = [self.tcp, [self.opening], self.approach, self.closing]
        return all(bool(np.all(np.isfinite(np.asarray(v, dtype=float)))) for v in values)


def hand_boxes(grippers: Sequence[Gripper]) -> tuple[np.ndarray, np.ndarray]:
    parts = [g.boxes() for g in grippers]
    if not parts:
        return np.zeros((0, 3)), np.zeros((0, 3, 3))
    return np.concatenate([c for c, _ in parts]), np.concatenate([e for _, e in parts])


def teeth_gap(
    grippers: Sequence[Gripper], before: Sequence[Gripper] | None = None, limit: float = TEETH_REPORT
) -> float:
    """The distance (m) from the grippers' tool points and finger pads, and their straight paths since ``before`` (the
    same grippers a step earlier), to the blade's tooth band (band_gap: never more than it is)."""
    was = None if before is None or len(before) != len(grippers) else hand_boxes(before)
    return band_gap(*swept(was, hand_boxes(grippers)), limit)


def let_go(knife: KnifeView, lever: LeverView, grippers: Sequence[Gripper]) -> bool:
    """Every tool point at least CLEAR, and every point of every finger pad at least FINGER_CLEAR, from the knife's
    outline and the lever's outline."""
    return all(
        point_box_distance(g.tcp, low, high) >= CLEAR
        and min(point_box_distance(p, low, high) for p in g.finger_points()) >= FINGER_CLEAR
        for g in grippers
        for low, high in ((knife.low, knife.high), (lever.low, lever.high))
    )


@dataclass(frozen=True)
class State:
    """One step's facts (None: the arms' state is unknown)."""

    on_pins: bool
    seated: bool
    locked: bool
    fallen: bool
    still: bool
    released: bool | None
    near_teeth: bool | None
    built: bool = True
    sane: bool = True  # every reported number finite

    @property
    def full(self) -> bool:
        return bool(self.locked and self.released is True and self.still and not self.fallen)


@dataclass(frozen=True)
class Pose:
    """Where the knife and the lever were at a step: to tell whether they have moved since."""

    origin: np.ndarray
    rot: np.ndarray
    angle: float

    @staticmethod
    def of(knife: "KnifeView", lever: "LeverView") -> "Pose":
        return Pose(knife.origin, knife.rot, lever.angle)

    def knife(self) -> tuple[np.ndarray, np.ndarray]:
        return knife_boxes(self.origin, self.rot)

    def still_since(self, before: "Pose | None") -> bool:
        if before is None:
            return False
        moved = float(np.linalg.norm(self.origin - before.origin))
        turned = turn_angle(before.rot.T @ self.rot)
        return (
            moved < MAX_SPEED * STEP_S
            and turned < MAX_SPIN * STEP_S
            and abs(self.angle - before.angle) < (MAX_SPIN * STEP_S)
        )


def knife_teeth_gap(now: Pose, before: Pose | None = None, limit: float = TEETH_REPORT) -> float:
    """The distance (m) from every one of the knife's boxes, whole, and their straight paths since ``before``, to the
    blade's tooth band (band_gap: never more than it is)."""
    return band_gap(*swept(None if before is None else before.knife(), now.knife()), limit)


def state(
    knife: KnifeView,
    lever: LeverView,
    grippers: Sequence[Gripper] | None,
    sane: bool = True,
    fixtures: bool = True,
    before: Pose | None = None,
    hands_before: Sequence[Gripper] | None = None,
) -> State:
    """One step's facts. ``before``: the knife's and lever's pose at the last step (None on a trial's first step:
    nothing is still yet); ``hands_before``: the grippers then, so the paths since count too. A knife or gripper that
    went further than JUMP since then voids the trial (sane False)."""
    seated = knife.seated
    pose = Pose.of(knife, lever)
    hands_was = None if grippers is None or hands_before is None or len(hands_before) != len(grippers) else hands_before
    jumped = travel(None if before is None else before.knife(), pose.knife()) > JUMP or (
        grippers is not None and hands_was is not None and travel(hand_boxes(hands_was), hand_boxes(grippers)) > JUMP
    )
    if jumped:
        _warn_once("jump", f"the knife or a gripper went more than {JUMP * 100:.0f} cm in one step: the trial is void")
        before, hands_was = None, None
    knife_near = knife_teeth_gap(pose, before, KNIFE_TOOTH_CLEAR + TEETH_RESOLUTION) < KNIFE_TOOTH_CLEAR
    hands_near = (
        None if grippers is None else teeth_gap(grippers, hands_was, TOOTH_CLEAR + TEETH_RESOLUTION) < TOOTH_CLEAR
    )
    near_teeth = True if knife_near else hands_near  # the knife counts while the arms can't be read too
    jaw_on_knife = lever.jaw_gap(float(knife.offset[0])) <= JAW_ON_KNIFE
    return State(
        on_pins=knife.on_pins,
        seated=seated,
        locked=seated and knife.pressed and lever.past_centre and jaw_on_knife,
        fallen=knife.fallen,
        still=pose.still_since(before),
        released=None if grippers is None else let_go(knife, lever, grippers),
        near_teeth=near_teeth,
        built=knife.built and lever.built and fixtures,
        sane=sane and not jumped,
    )


@dataclass
class Ledger:
    steps: int = 0
    full_steps: int = 0
    touched_teeth: bool = False
    misbuilt: bool = False  # latched: the knife or lever didn't come out as built here
    corrupt: bool = False  # latched: the simulator reported a NaN or infinite number
    last_pose: Pose | None = None  # where the knife and lever were at the last step
    last_grippers: list[Gripper] | None = None  # ... and the grippers

    def observe(self, now: State) -> None:
        self.steps += 1
        self.misbuilt = self.misbuilt or not now.built
        self.corrupt = self.corrupt or not now.sane
        self.full_steps = self.full_steps + 1 if now.full else 0
        self.touched_teeth = self.touched_teeth or now.near_teeth is True  # one step (or the path to it) is enough


@dataclass(frozen=True)
class Grade:
    state: State
    full_steps: int
    touched_teeth: bool
    score: float
    void: bool = False  # misbuilt or corrupt: nothing this trial can be trusted


def grade(ledger: Ledger, now: State) -> Grade:
    """The ladder (README.md): 0 if the knife fell off the bench or the arms' state can't be read; 0 for the whole
    trial if the knife, lever, blade or a pin isn't the one built here, the simulator reported a non-finite number or
    a non-unit orientation, a knife or lever state is missing or unreadable, or something jumped (JUMP); else 1.0 for a knife seated, locked and let go, all still, held 1 s; 0.75 seated and
    locked; 0.5 seated on both pins; 0.25 both pins in its slots. Capped at 0.5 if a gripper's tool point or finger
    pads, or the knife itself, touched the blade's teeth."""
    void = ledger.misbuilt or ledger.corrupt
    if void or now.fallen or now.released is None:
        score = 0.0
    elif now.locked:
        score = 1.0 if now.full and ledger.full_steps >= HOLD_STEPS else 0.75
    elif now.seated:
        score = 0.5
    elif now.on_pins:
        score = 0.25
    else:
        score = 0.0
    if ledger.touched_teeth:
        score = min(score, TEETH_CAP)
    return Grade(now, ledger.full_steps, ledger.touched_teeth, score, void)


# --- episodes and the robot's state -------------------------------------------------------------------------------
_LEDGER = Ledger()
_GRADE: Grade | None = None
_last_objects: object = None  # the step's state dict, kept alive so its id can't be reused
_last_clock: float | None = None
_embodiment: Any = None
_hooked = False
_warned: set[str] = set()
_ARMS = (("left_robot", "left"), ("right_robot", "right"))
RESET_TAG = "fit_riving_knife_new_episode"


def _warn_once(key: str, message: str) -> None:
    if key not in _warned:
        _warned.add(key)
        warnings.warn(f"fit_riving_knife: {message}", RuntimeWarning, stacklevel=3)


def new_episode() -> None:
    global _LEDGER, _GRADE, _last_objects, _last_clock
    _LEDGER = Ledger()
    _GRADE = None
    _last_objects = None
    _last_clock = None
    _warned.discard("robot")  # warn again in every trial whose arms can't be read
    _warned.discard("jump")


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
    The wrapper keeps a list of callbacks, so another load of this file (with its own ledger) joins it. The embodiment
    is also where the arms' joint positions come from; only one whose reset is hooked is read."""
    global _hooked, _embodiment
    if _hooked:
        return
    embodiment = _calling_embodiment()
    if embodiment is not None:
        original = embodiment.reset
        callbacks = getattr(original, RESET_TAG, None)
        if isinstance(callbacks, list):  # wrapped by another load of this file
            if new_episode not in callbacks:
                callbacks.append(new_episode)
            _embodiment, _hooked = embodiment, True
            return
        callbacks = [new_episode]

        @functools.wraps(original)
        def reset(*args: Any, **kwargs: Any) -> Any:
            for callback in callbacks:
                callback()
            return original(*args, **kwargs)

        setattr(reset, RESET_TAG, callbacks)
        try:
            embodiment.reset = reset
        except (AttributeError, TypeError):
            pass
        else:
            _embodiment, _hooked = embodiment, True
            return
    _warn_once(
        "reset",
        "could not hook the embodiment's reset, so the arms' state is unknown: every step scores 0",
    )


def _numpy(values: Any) -> np.ndarray:
    for step in ("detach", "cpu", "numpy"):
        if hasattr(values, step):
            values = getattr(values, step)()
    return np.asarray(values, dtype=float)


def _xyz(v: Any) -> tuple[float, float, float]:
    a = np.asarray(v, dtype=float)
    return float(a[0]), float(a[1]), float(a[2])


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
    # An unexpected embodiment: without the arms nothing that ends or caps credit can be checked, so each such step
    # scores 0. Read again next step (a one-off failure doesn't switch reading off), and warn in every trial.
    except Exception as error:
        _warn_once(
            "robot",
            f"can't read the arms' state ({type(error).__name__}: {error}); each such step scores 0",
        )
        return None


def _clock() -> float | None:
    t = getattr(_embodiment, "_t", None)
    try:
        return None if t is None else float(t)
    except (TypeError, ValueError):
        return None


def _update(objects: Mapping[str, Any]) -> Grade:
    """Advance the episode once per control step (success() and score() share the step's state dict)."""
    global _last_objects, _GRADE, _last_clock
    if objects is _last_objects and _GRADE is not None:
        return _GRADE
    try:
        _hook_resets()
    except Exception as error:  # an odd caller on the stack: carry on without the hook
        _warn_once("hook", f"could not look for the embodiment ({type(error).__name__}: {error})")
    clock = _clock()
    if clock is not None and _last_clock is not None and clock < _last_clock:
        new_episode()  # the episode clock went back: a reset this file didn't see
    knife_state, lever_state = objects.get(KNIFE), objects.get(LEVER)
    grippers = _robot()
    now: State | None = None
    if (
        knife_state is not None
        and lever_state is not None
        and finite(knife_state)
        and finite(lever_state)
        and (grippers is None or all(g.finite for g in grippers))
    ):
        try:
            knife, lever = KnifeView.of(knife_state), LeverView.of(lever_state)
            now = state(
                knife,
                lever,
                grippers,
                fixtures=fixtures_ok(objects),
                before=_LEDGER.last_pose,
                hands_before=_LEDGER.last_grippers,
            )
            _LEDGER.last_pose, _LEDGER.last_grippers = Pose.of(knife, lever), grippers
        except _BAD_STATE:  # an odd shape, say: as unreadable as a NaN
            now = None
    if now is None:  # missing, unreadable or a NaN: read nothing, everything off, and the trial voided below
        now = State(False, False, False, False, False, None, None, True, False)
    _LEDGER.observe(now)
    _last_objects, _last_clock = objects, clock
    _GRADE = grade(_LEDGER, now)
    return _GRADE


def score(objects: Mapping[str, Any]) -> float:
    """See README.md for the ladder."""
    return _update(objects).score


def success(objects: Mapping[str, Any]) -> bool:
    """Everything 1.0 needs, this step, with nothing that ends or caps credit: the knife seated and locked, both
    grippers clear, all still (the embodiment ends the trial once this has held for 1 s, when score() reaches 1.0)."""
    grade_now = _update(objects)
    return grade_now.state.full and not grade_now.touched_teeth and not grade_now.void


# --- the scripted demo ---------------------------------------------------------------------------------------------
GRIP_YAW = math.pi / 2  # the right arm's fingers close along world x, across the knife's 2.3 mm
HANDLE_YAW = 0.0  # the left arm's close along world y: the gripper stays narrow across x, clear of the opening's edge
KNIFE_GRIP_Y = -0.180  # the knife is held (and pressed) at its level back, its pads 25 mm and more behind the blade
KNIFE_GRIP_HEIGHT = 0.010  # the TCP 10 mm above the saw top when the knife is home
LIFT = SAW_TOP - TANG_BOTTOM + 0.0115  # above home: the knife's foot 11.5 mm clear of the top while carried
PRESS = 0.0015  # the knife is lowered this far past home, so it bottoms on the pins (the grip slips)
BESIDE = 0.020  # it goes against the base plate 20 mm above home: its foot below the plate's top, above the pins
SEAT_OVERTRAVEL = 0.0005  # aimed 0.5 mm into the base plate, so it rests against it (the arm gives) as it slides down
# The right arm's measured lag carrying the knife (the TCP reached, less the TCP commanded; x, y) under the official
# controller, from a GPU trace: at the holder (+y of -0.3 m) and over the mount at the carry height and low down. The
# demo aims that much the other way. These are fixed numbers in the plan: nothing in the scene is read.
LAG_HOLDER = (-0.0005, 0.0012)
LAG_CARRY_Z, LAG_CARRY = 0.9255, (-0.0006, -0.0007)
LAG_LOW_Z, LAG_LOW = 0.8487, (-0.0017, -0.0021)
HANDLE_OVERSHOOT = 0.006  # the push aims the fingertips 6 mm below the pad's locked height: through the whole travel
PUSH_AT = -0.45  # rad: the push is aimed where the pad is with the lever here (the pad slides under the tips)


def _finger(opening: float) -> float:
    """The finger joint target for fingers ``opening`` m apart (each finger moves half; 94 mm at most)."""
    return max(float(FINGER_OPEN), -opening / 2)


def knife_tcp(shift: Sequence[float] = (0.0, 0.0, 0.0)) -> tuple[float, float, float]:
    """The TCP holding the knife at its grip point, the knife shifted ``shift`` from home."""
    return (BLADE_X + shift[0], KNIFE_GRIP_Y + shift[1], SAW_TOP + KNIFE_GRIP_HEIGHT + shift[2])


def right_lag(tcp: Sequence[float]) -> tuple[float, float]:
    """The right arm's expected TCP lag (x, y; m) at a TCP position while it carries the knife (measured; see above):
    the holder's at the holder, and over the mount interpolated by height between the carry's and the low one's."""
    if tcp[1] < -0.3:
        return LAG_HOLDER
    f = min(1.0, max(0.0, (LAG_CARRY_Z - tcp[2]) / (LAG_CARRY_Z - LAG_LOW_Z)))
    return (LAG_CARRY[0] + f * (LAG_LOW[0] - LAG_CARRY[0]), LAG_CARRY[1] + f * (LAG_LOW[1] - LAG_CARRY[1]))


def _aim(tcp: Sequence[float]) -> tuple[float, float, float]:
    """The right arm's command that should bring its TCP to ``tcp``: the target less the arm's expected lag."""
    dx, dy = right_lag(tcp)
    return (tcp[0] - dx, tcp[1] - dy, tcp[2])


def handle_top(angle: float = OPEN_STOP) -> tuple[float, float, float]:
    """The top centre of the handle's pad (world) with the lever at ``angle``."""
    centre, size, turn = next((c, sz, t) for name, c, sz, t in LEVER_PARTS if name == "handle")
    top = turn_y(turn) @ np.array([0.0, 0.0, size[2] / 2]) + np.asarray(centre)
    point = turn_y(angle) @ top + np.asarray(SHAFT)
    return float(point[0]), float(point[1]), float(point[2])


def demo() -> dict[str, list[Waypoint]]:
    """Right arm: lift the knife out of its holder by its back, carry it over the mount, lower it down the gap between
    the base plate and the open jaw to just above the pins, put it against the base plate and slide it down it onto
    both pins (the round pins steer its slots on); keep hold of it, seated, while the other arm locks it; then let go
    and lift clear. Left arm: wait over the clamp's handle, push it down through its travel once the knife is seated
    (it clicks over and locks), and lift clear. The right arm's targets allow for its measured lag (right_lag)."""
    yaw = GRIP_YAW
    start = knife_tcp(HOLD_SHIFT)
    carry = knife_tcp((CARRY_DX, 0.0, LIFT))
    beside = knife_tcp((CARRY_DX, 0.0, BESIDE))
    against = knife_tcp((SEAT_OVERTRAVEL, 0.0, BESIDE))
    low = knife_tcp((SEAT_OVERTRAVEL, 0.0, -PRESS))
    clear = (low[0], low[1], BACK_TOP + 0.020 + PAD_TIP_PAST_TCP)  # the open fingers above the knife's back
    side_right = (0.33, -0.25, SAW_TOP + 0.08)
    right = [
        Waypoint(2.0, (start[0], start[1], start[2] + 0.06), _finger(0.025), "above the knife in its holder", yaw),
        Waypoint(1.5, _aim(start), _finger(0.025), "fingers round the knife's back", yaw),
        Waypoint(0.8, _aim(start), FINGER_CLOSED, "grip the knife", yaw),
        Waypoint(2.0, _aim((start[0], start[1], carry[2])), FINGER_CLOSED, "lift it out of the holder", yaw),
        Waypoint(2.5, _aim(carry), FINGER_CLOSED, "carry it over the mount", yaw),
        Waypoint(0.8, _aim(carry), FINGER_CLOSED, "let it settle", yaw),
        Waypoint(3.0, _aim(beside), FINGER_CLOSED, "lower it beside the base plate", yaw),
        Waypoint(0.5, _aim(beside), FINGER_CLOSED, "steady", yaw),
        Waypoint(1.0, _aim(against), FINGER_CLOSED, "against the base plate", yaw),
        Waypoint(3.0, _aim(low), FINGER_CLOSED, "slide it down onto the pins", yaw),
        Waypoint(0.7, _aim(low), FINGER_CLOSED, "settle on the pins", yaw),  # t = 17.8
        Waypoint(4.5, _aim(low), FINGER_CLOSED, "keep it seated while it is locked", yaw),  # t = 22.3
        Waypoint(0.8, _aim(low), _finger(0.030), "let go of the knife", yaw),
        Waypoint(1.2, clear, _finger(0.030), "lift clear", yaw),
        Waypoint(1.5, side_right, _finger(0.030), "out to the side", yaw),  # t = 25.8
        Waypoint(2.5, side_right, _finger(0.030), "wait for the other arm", yaw),
    ]
    # The left arm's clock: over the handle early, pushing it down once the knife is seated (t = 17.8 to 21.3).
    up, aim = handle_top(), handle_top(PUSH_AT)
    over = (aim[0], aim[1], up[2] + 0.015 + PAD_TIP_PAST_TCP)
    touch = (aim[0], aim[1], up[2] + 0.001 + PAD_TIP_PAST_TCP)
    down = (aim[0], aim[1], handle_top(LOCKED_ANGLE)[2] - HANDLE_OVERSHOOT + PAD_TIP_PAST_TCP)
    side_left = (0.20, 0.22, SAW_TOP + 0.08)
    hyaw = HANDLE_YAW
    left = [
        Waypoint(3.0, over, FINGER_CLOSED, "over the clamp's handle", hyaw),
        Waypoint(14.8, over, FINGER_CLOSED, "wait for the knife to be seated", hyaw),  # t = 17.8
        Waypoint(1.0, touch, FINGER_CLOSED, "onto the handle", hyaw),
        Waypoint(2.0, down, FINGER_CLOSED, "push the handle down: it snaps over and locks", hyaw),  # t = 20.8
        Waypoint(0.5, down, FINGER_CLOSED, "hold", hyaw),
        Waypoint(1.2, over, FINGER_CLOSED, "lift clear", hyaw),
        Waypoint(1.5, side_left, FINGER_CLOSED, "out to the side", hyaw),
        Waypoint(4.3, side_left, FINGER_CLOSED, "wait", hyaw),
    ]
    return {"left": left, "right": right}


def _simulator_present() -> bool:
    """Whether Isaac Lab can be imported here: the hook is only installed where the scene can be built."""
    try:
        return importlib.util.find_spec("isaaclab") is not None
    except (ImportError, ValueError):
        return False


if _simulator_present():
    try:
        install_saw()
    except Exception as error:  # never break loading the task over the hook; the outline check then scores 0
        _warn_once("saw", f"could not install the saw hook ({type(error).__name__}: {error})")
else:
    _warn_once(
        "simulator",
        "Isaac Lab isn't importable here, so this file won't build its knife and lever: a scene without them fails "
        "the outline check and scores 0",
    )
