"""On the bench in front of the arms stands a builder's automatic level (a surveying telescope that reads heights
off a staff), screwed to the flat head of a short fixed stand. It is not level yet.

The instrument stands on a three-screw levelling base. Three black knurled foot-screw knobs, 30 mm across, sit on
top of the base's three short arms: one at the back, toward the robot, and two at the front, to its left and
right. Each knob turns a fine screw whose steel tip stands on the round grey base plate below, so turning a knob
raises or lowers that corner of the instrument. The white telescope, 26 cm long with a black eyepiece and a grey
sunshade over the objective lens, sits on top of the base and turns freely about the vertical axis. It starts
pointing away from the robot.

A circular bubble level (a bull's-eye vial) sits on a small bracket on the telescope housing's left side: a round
yellow-green window 28 mm across with a black ring 10 mm across printed at its centre and a white bubble 8 mm
across that floats to whichever side of the instrument is highest. At the start the instrument leans toward the
back-left, and the bubble's centre is about 5 mm from the ring's centre, on the ring's line: half out of the ring.
"""

# No `from __future__ import annotations` here: the task loader imports this file without registering it in
# sys.modules, and dataclasses then cannot resolve string annotations.
import contextlib
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
from dataclasses import dataclass, field
from types import FrameType
from typing import Any

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Cylinder, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]  # the table's top, z (m)

# --- how the foot screws' threads are simulated ------------------------------------------------------------------
# Each knob turns on a hinge on its arm, and its screw slides on a prismatic joint held by a very stiff spring whose
# set point this file moves to the knob's turn times the thread's pitch at every control step (score() and success()
# run every step). A PhysX rack-and-pinion joint was tried first and locked the knobs, so it is not used.

# --- where things are (world frame: x away from the robot, y to its left, z up; metres) --------------------------
X0, Y0 = 0.26, 0.0  # the instrument's vertical axis
COLUMN_RADIUS, COLUMN_HEIGHT = 0.025, 0.040  # the fixed stand's steel column, bolted to the bench
HEAD_RADIUS, HEAD_HEIGHT = 0.090, 0.020  # the stand's flat aluminium head (a tripod head)
# The levelling base's plate, screwed to the head; the three screw tips stand on it, 8 mm in from its edge.
PLATE_RADIUS, PLATE_HEIGHT = 0.112, 0.006
FP = TOP + COLUMN_HEIGHT + HEAD_HEIGHT + PLATE_HEIGHT  # the plate's top, where the screw tips stand

# The foot screws stand on a circle about the vertical axis: one toward the robot, two at the front.
SCREW_CIRCLE = 0.100
SCREWS = ("back", "left", "right")
SCREW_ANGLE = {"back": math.pi, "left": math.pi / 3.0, "right": -math.pi / 3.0}
TIP_RADIUS = 0.004  # the screw's rounded steel tip
PITCH = 0.001  # m per turn: a fine metric thread (M8 x 1)
TRAVEL = 0.004  # each screw may move 4 mm up or down from its start before its thread runs out

# The base: a round body (FP + 8 mm to FP + 50 mm) with three arms out to the screws, each ending in a boss.
BASE_RADIUS = 0.050
BASE_LOW, BASE_HIGH = FP + 0.008, FP + 0.050
ARM_LOW, ARM_HIGH, ARM_WIDTH = FP + 0.008, FP + 0.018, 0.018
ARM_FROM = 0.045
BOSS_RADIUS = 0.011
# Each knob sits on top of its boss, 2 mm clear of it; a spring washer between them gives the thread's friction.
KNOB_RADIUS, KNOB_HEIGHT = 0.015, 0.012
KNOB_LOW = FP + 0.020
KNOB_Z = KNOB_LOW + KNOB_HEIGHT / 2
WASHER_RADIUS, WASHER_HEIGHT = 0.012, 0.0016
# Spawned 0.2 mm under the knob; its spring presses it up.
WASHER_Z = KNOB_LOW - 0.0002 - WASHER_HEIGHT / 2

# The telescope, which turns on the base: housing, tube along its own x, sunshade at +x, eyepiece at -x.
HOUSING_RADIUS = 0.052
HOUSING_LOW, HOUSING_HIGH = FP + 0.052, FP + 0.072
TUBE_RADIUS, TUBE_HALF = 0.023, 0.100
TUBE_Z = FP + 0.120  # the telescope's axis, on a standard above the housing
SUNSHADE_RADIUS, SUNSHADE_FROM, SUNSHADE_TO = 0.026, 0.095, 0.125
# The eyepiece with its rubber eyecup, 40 mm long.
EYEPIECE_RADIUS, EYEPIECE_FROM, EYEPIECE_TO = 0.017, -0.140, -0.100
# The circular level: on a bracket on the housing's left side, toward the eyepiece (the telescope's own frame).
VIAL_XY = (-0.0333, 0.0475)
VIAL_BRACKET_RADIUS = 0.017
VIAL_RADIUS = 0.014  # the window (inside radius of the vial)
VIAL_TOP = FP + 0.088  # the liquid's surface, where the bubble floats
# The black ring: its centre line's radius and its line width.
RING_RADIUS, RING_LINE = 0.005, 0.0008
BUBBLE_RADIUS = 0.004
SENSITIVITY = math.radians(8.0 / 60.0) / 0.002  # rad of tilt per m the bubble moves: 8' per 2 mm
# The vial's radius of curvature (0.86 m): the bubble moves this x tilt.
VIAL_CURVE = 1.0 / SENSITIVITY
# The bubble's centre can move 10 mm before it meets the edge.
VIAL_REACH = VIAL_RADIUS - BUBBLE_RADIUS
# The bubble's counterweight hangs 5 cm below its pivot (the centre of curvature).
BUBBLE_DROP = 0.05

# Masses: 1.6 kg in all, as builder's automatic levels weigh 1.5 to 1.7 kg.
BASE_MASS, SCOPE_MASS = 0.70, 0.85
KNOB_MASS, SCREW_MASS, BUBBLE_MASS, WASHER_MASS = 0.015, 0.008, 0.005, 0.002
KNOB_DAMPING = 0.2  # N m s / rad: the thread's grease, felt at the knob
# N: each washer's push on its knob; its friction (about 0.02 N m) holds the knob.
WASHER_PRESS = 2.7
WASHER_STIFFNESS = 1500.0  # N / m
WASHER_PATCH = 0.010  # m: the washer's contact resists turning as a 10 mm friction patch
# N m s / rad: the drag of the telescope's vertical axis, its weight balanced on it. On the GPU, 5 made the swing so
# stiff that the arm's pull tipped the instrument and the telescope stopped at -26 degrees; 0.5 swung it fully.
SCOPE_DAMPING = 0.5
# The vertical axis also holds its heading, as a real one's friction does: a spring (N m / rad) toward a set point that
# this file moves along only when the telescope is turned more than SCOPE_SLIP past it. A hand swings it against at
# most 0.17 N m, but the small steady torque that made it creep on the GPU (about half a degree a second, even with
# 10 times the drag) cannot: with the spring it held to 0.06 degrees.
SCOPE_HOLD, SCOPE_SLIP = 20.0, math.radians(0.5)
HOLD_DOWN = 20.0  # N: the central spring that holds the base down on its three screws
HOLD_STIFFNESS = 2000.0  # N / m
# The threads' springs (N / m, N s / m). On the GPU, 1e6 N/m with PhysX's default 4 solver iterations let each screw
# settle about 0.8 mm under its 12 N load, and 1e8 N/m set the base vibrating; so 1e6, with SOLVER_ITERATIONS.
SCREW_STIFFNESS, SCREW_SPRING_DAMPING = 1.0e6, 2.0e3
# Solver position and velocity iterations for every body of the instrument: light knobs and screws jointed to a heavy
# base need more than PhysX's default for their joints and springs to hold.
SOLVER_ITERATIONS = (32, 4)
# 1 / s: the vial's liquid slowing the bubble (with less, its long pendulum's pivot stretched on the GPU).
BUBBLE_DAMPING = 20.0

# --- the starting state ------------------------------------------------------------------------------------------
# The instrument starts tilted 24' toward the back-left (150 degrees from x), halfway between two screws' lines, so no
# single screw can level it: turned alone, the best one leaves 12.0', and still 9.9' with every corner held up or
# pressed anywhere within the seat band. START_EXTENSION is how far each screw then stands out from where it would
# leave the instrument level (m; + lowers the tip, lifting that corner). The scripted demo does not use it: its turns
# are planned from the bubble's offset.
START_TILT_TOWARD = math.radians(150.0)
START_TILT_SET = math.radians(24.0 / 60.0)
# What it settles to on the GPU (2026-11-12, the same in every trial): the threads' springs give a little under the
# instrument's weight, the front-right most, so the start is 24.4' toward 148 degrees. The wrist camera shows this.
START_SETTLED, START_SETTLED_TOWARD = math.radians(24.4 / 60.0), math.radians(148.0)
START_EXTENSION = {
    s: SCREW_CIRCLE * START_TILT_SET * math.cos(SCREW_ANGLE[s] - START_TILT_TOWARD) for s in SCREWS
}
SPAWN_GAP = 0.0002  # every tip is spawned at least this far above the plate; the instrument settles onto them
# How far above its seated height the instrument is spawned.
LIFT = max(START_EXTENSION.values()) + SPAWN_GAP

# --- scoring ----------------------------------------------------------------------------------------------------
# 4': the bubble's centre within 1 mm of the ring's (the bubble in the ring).
TILT_RING = 1.0e-3 * SENSITIVITY
TILT_TOUCH = 2.0e-3 * SENSITIVITY  # 8': the bubble's edge on the ring
TILT_COMPENSATOR = math.radians(15.0 / 60.0)  # 15': the automatic compensator's working range
MAX_TILT = math.radians(2.0)  # anything tilted more than this has come off its screws
SEAT_GAP = 0.00005  # a tip more than 0.05 mm above the plate is not standing on it (rigid contacts rest at 0)
SEAT_PRESS = 0.00005  # a tip more than 0.05 mm into the plate is being pushed down (tune from resting contacts)
STILL_SPEED, STILL_SPIN = 0.005, 0.02  # m/s and rad/s
KNOB_STILL = math.radians(
    0.1
)  # a knob that turned less than this over a control step is not being turned
BANDS = ((TILT_RING, 0.75), (TILT_TOUCH, 0.5), (TILT_COMPENSATOR, 0.25))
SCORE_OVER = "end"

# --- colours (sRGB) ---------------------------------------------------------------------------------------------
WHITE = (0.93, 0.93, 0.90)
BLACK = (0.06, 0.06, 0.06)
DARK = (0.22, 0.23, 0.24)
STEEL = (0.62, 0.64, 0.66)
ALUMINIUM = (0.76, 0.77, 0.79)
LIQUID = (0.62, 0.78, 0.30)
BUBBLE_RGB = (0.98, 0.98, 0.96)


def screw_xy(screw: str) -> tuple[float, float]:
    """Where a foot screw stands, world (x, y)."""
    a = SCREW_ANGLE[screw]
    return (X0 + SCREW_CIRCLE * math.cos(a), Y0 + SCREW_CIRCLE * math.sin(a))


# --- the objects: one primitive per rigid body; the scene hook adds each body's other parts and the joints ------
BASE, SCOPE, BUBBLE = "level_base", "level_scope", "level_bubble"
KNOB = {s: f"level_knob_{s}" for s in SCREWS}
SCREW = {s: f"level_screw_{s}" for s in SCREWS}
WASHER = {s: f"level_washer_{s}" for s in SCREWS}
COLUMN, HEAD, PLATE = "level_stand_column", "level_stand_head", "level_base_plate"

BASE_Z = (BASE_LOW + BASE_HIGH) / 2
SCOPE_Z = (HOUSING_LOW + HOUSING_HIGH) / 2
SPAWN = {  # where each body is spawned: level, LIFT above its seated height
    BASE: (X0, Y0, BASE_Z + LIFT),
    SCOPE: (X0, Y0, SCOPE_Z + LIFT),
    BUBBLE: (X0 + VIAL_XY[0], Y0 + VIAL_XY[1], VIAL_TOP + LIFT),
    **{KNOB[s]: (*screw_xy(s), KNOB_Z + LIFT) for s in SCREWS},
    **{WASHER[s]: (*screw_xy(s), WASHER_Z + LIFT) for s in SCREWS},
    **{SCREW[s]: (*screw_xy(s), FP + TIP_RADIUS + LIFT - START_EXTENSION[s]) for s in SCREWS},
}
# Each jointed body's origin as spawned, and where each joint holds its two bodies together then (world, m).
JOINT_ROOTS = {**SPAWN, PLATE: (X0, Y0, FP - PLATE_HEIGHT / 2)}
HOLD_ANCHOR = (X0, Y0, FP + TIP_RADIUS + LIFT)  # the base on the plate, by its central spring
AXIS_ANCHOR = (X0, Y0, HOUSING_LOW - 0.001 + LIFT)  # the telescope on the base's vertical axis
PIVOT_ANCHOR = (
    X0 + VIAL_XY[0],
    Y0 + VIAL_XY[1],
    VIAL_TOP + LIFT - VIAL_CURVE,
)  # the bubble's pendulum
HOLD_TRAVEL = (-0.002, 0.012)  # how far the base may rise or sink on its spring (m)
WASHER_TRAVEL = (-0.0004, 0.0006)  # how far a spring washer may slide

STAND = [
    Cylinder(
        COLUMN,
        pos=(X0, Y0, TOP + COLUMN_HEIGHT / 2),
        radius=COLUMN_RADIUS,
        height=COLUMN_HEIGHT,
        color=STEEL,
    ),
    Cylinder(
        HEAD,
        pos=(X0, Y0, TOP + COLUMN_HEIGHT + HEAD_HEIGHT / 2),
        radius=HEAD_RADIUS,
        height=HEAD_HEIGHT,
        color=ALUMINIUM,
    ),
    Cylinder(
        PLATE,
        pos=(X0, Y0, FP - PLATE_HEIGHT / 2),
        radius=PLATE_RADIUS,
        height=PLATE_HEIGHT,
        color=DARK,
    ),
]
INSTRUMENT = [
    Cylinder(
        BASE,
        pos=SPAWN[BASE],
        radius=BASE_RADIUS,
        height=BASE_HIGH - BASE_LOW,
        mass=BASE_MASS,
        color=WHITE,
    ),
    Cylinder(
        SCOPE,
        pos=SPAWN[SCOPE],
        radius=HOUSING_RADIUS,
        height=HOUSING_HIGH - HOUSING_LOW,
        mass=SCOPE_MASS,
        color=WHITE,
    ),
    *(
        Cylinder(
            KNOB[s],
            pos=SPAWN[KNOB[s]],
            radius=KNOB_RADIUS,
            height=KNOB_HEIGHT,
            mass=KNOB_MASS,
            color=BLACK,
        )
        for s in SCREWS
    ),
    *(
        Cylinder(
            WASHER[s],
            pos=SPAWN[WASHER[s]],
            radius=WASHER_RADIUS,
            height=WASHER_HEIGHT,
            mass=WASHER_MASS,
            color=STEEL,
        )
        for s in SCREWS
    ),
    *(
        Sphere(SCREW[s], pos=SPAWN[SCREW[s]], radius=TIP_RADIUS, mass=SCREW_MASS, color=STEEL)
        for s in SCREWS
    ),
    Sphere(BUBBLE, pos=SPAWN[BUBBLE], radius=BUBBLE_RADIUS, mass=BUBBLE_MASS, color=BUBBLE_RGB),
]
OBJECTS = [*STAND, *INSTRUMENT]  # the bubble last: its spawner also adds the joints
BODIES = tuple(o.name for o in INSTRUMENT)


# --- the bodies' other parts (in the body's own frame, which is level and unturned when spawned) ----------------
@dataclass(frozen=True)
class Part:
    body: str
    name: str
    shape: str  # "box", "cylinder"
    centre: tuple[float, float, float]  # in the body's frame (m)
    size: tuple[float, float, float] = (0.0, 0.0, 0.0)  # a box's x, y, z edges
    radius: float = 0.0
    length: float = 0.0  # a cylinder's length along its axis
    axis: str = "Z"
    yaw: float = 0.0  # a box turned about z
    color: tuple[float, float, float] = WHITE
    collide: bool = True


def _parts() -> list[Part]:
    out: list[Part] = []
    arm_z = (ARM_LOW + ARM_HIGH) / 2 - BASE_Z
    arm_length = SCREW_CIRCLE - ARM_FROM
    for s in SCREWS:
        a = SCREW_ANGLE[s]
        mid = ARM_FROM + arm_length / 2
        out.append(
            Part(
                BASE,
                f"arm_{s}",
                "box",
                (mid * math.cos(a), mid * math.sin(a), arm_z),
                size=(arm_length, ARM_WIDTH, ARM_HIGH - ARM_LOW),
                yaw=a,
            )
        )
        out.append(
            Part(
                BASE,
                f"boss_{s}",
                "cylinder",
                (SCREW_CIRCLE * math.cos(a), SCREW_CIRCLE * math.sin(a), arm_z),
                radius=BOSS_RADIUS,
                length=ARM_HIGH - ARM_LOW,
            )
        )
    out.append(
        Part(
            BASE,
            "circle",
            "cylinder",
            (0.0, 0.0, BASE_HIGH - BASE_Z - 0.002),
            radius=BASE_RADIUS + 0.0015,
            length=0.004,
            color=DARK,
            collide=False,
        )
    )
    tube_z = TUBE_Z - SCOPE_Z
    out += [
        Part(
            SCOPE,
            "tube",
            "cylinder",
            (0.0, 0.0, tube_z),
            radius=TUBE_RADIUS,
            length=2 * TUBE_HALF,
            axis="X",
        ),
        # Look-only fittings: the focusing knob on the telescope's right side and the two endless horizontal-drive
        # (tangent) knobs low on the housing's sides. They have no collider, so nothing can turn or push them.
        Part(
            SCOPE,
            "focus_knob",
            "cylinder",
            (0.0, -(TUBE_RADIUS + 0.006), tube_z),
            radius=0.009,
            length=0.012,
            axis="Y",
            color=BLACK,
            collide=False,
        ),
        *(
            Part(
                SCOPE,
                f"tangent_knob_{side}",
                "cylinder",
                (0.0, sign * (HOUSING_RADIUS + 0.004), (HOUSING_LOW + HOUSING_HIGH) / 2 - SCOPE_Z),
                radius=0.008,
                length=0.008,
                axis="Y",
                color=BLACK,
                collide=False,
            )
            for side, sign in (("left", 1.0), ("right", -1.0))
        ),
        Part(
            SCOPE,
            "standard",
            "cylinder",
            (0.0, 0.0, (HOUSING_HIGH + TUBE_Z) / 2 - SCOPE_Z),
            radius=0.022,
            length=TUBE_Z - HOUSING_HIGH,
        ),
        Part(
            SCOPE,
            "sunshade",
            "cylinder",
            ((SUNSHADE_FROM + SUNSHADE_TO) / 2, 0.0, tube_z),
            radius=SUNSHADE_RADIUS,
            length=SUNSHADE_TO - SUNSHADE_FROM,
            axis="X",
            color=DARK,
        ),
        Part(
            SCOPE,
            "objective",
            "cylinder",
            (SUNSHADE_TO + 0.0005, 0.0, tube_z),
            radius=0.019,
            length=0.001,
            axis="X",
            color=(0.05, 0.08, 0.14),
            collide=False,
        ),
        Part(
            SCOPE,
            "eyepiece",
            "cylinder",
            ((EYEPIECE_FROM + EYEPIECE_TO) / 2, 0.0, tube_z),
            radius=EYEPIECE_RADIUS,
            length=EYEPIECE_TO - EYEPIECE_FROM,
            axis="X",
            color=BLACK,
        ),
        Part(
            SCOPE,
            "gunsight",
            "box",
            (0.0, 0.0, tube_z + TUBE_RADIUS + 0.004),
            size=(0.040, 0.006, 0.008),
            color=BLACK,
            collide=False,
        ),
        Part(
            SCOPE,
            "vial_bracket",
            "cylinder",
            (*VIAL_XY, VIAL_TOP - 0.005 - SCOPE_Z),
            radius=VIAL_BRACKET_RADIUS,
            length=0.006,
        ),
        Part(
            SCOPE,
            "vial_liquid",
            "cylinder",
            (*VIAL_XY, VIAL_TOP - 0.001 - SCOPE_Z),
            radius=VIAL_RADIUS,
            length=0.002,
            color=LIQUID,
            collide=False,
        ),
        Part(
            SCOPE,
            "vial_ring",
            "cylinder",
            (*VIAL_XY, VIAL_TOP + 0.0001 - SCOPE_Z),
            radius=RING_RADIUS + RING_LINE / 2,
            length=0.0002,
            color=BLACK,
            collide=False,
        ),
        Part(
            SCOPE,
            "vial_ring_inside",
            "cylinder",
            (*VIAL_XY, VIAL_TOP + 0.0003 - SCOPE_Z),
            radius=RING_RADIUS - RING_LINE / 2,
            length=0.0002,
            color=LIQUID,
            collide=False,
        ),
    ]
    for s in SCREWS:  # knurling on each knob, and the screw's shank from its tip up into the knob
        for i in range(16):
            a = 2 * math.pi * i / 16
            out.append(
                Part(
                    KNOB[s],
                    f"knurl_{i}",
                    "box",
                    (
                        (KNOB_RADIUS + 0.0003) * math.cos(a),
                        (KNOB_RADIUS + 0.0003) * math.sin(a),
                        0.0,
                    ),
                    size=(0.0012, 0.0024, KNOB_HEIGHT * 0.9),
                    yaw=a,
                    color=DARK,
                    collide=False,
                )
            )
        out.append(
            Part(
                SCREW[s],
                "shank",
                "cylinder",
                (0.0, 0.0, (KNOB_LOW - FP - TIP_RADIUS) / 2),
                radius=0.003,
                length=KNOB_LOW - FP - TIP_RADIUS,
                color=STEEL,
                collide=False,
            )
        )
    return out


PARTS = _parts()


# --- geometry -----------------------------------------------------------------------------------------------------
def rotation(quat_xyzw: Any) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if not (math.isfinite(n) and n > 1e-6):
        raise ValueError("not a rotation: the quaternion is zero or not finite")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def tilt_of(rot: np.ndarray) -> float:
    """The angle (rad) between a body's vertical axis and the world's."""
    return math.acos(max(-1.0, min(1.0, float(rot[2, 2]))))


def screw_local(screw: str) -> np.ndarray:
    a = SCREW_ANGLE[screw]
    return np.array([SCREW_CIRCLE * math.cos(a), SCREW_CIRCLE * math.sin(a)])


def seated_gradient(extension: Mapping[str, float]) -> np.ndarray:
    """With every tip on the flat plate, the instrument's slope (rad, along its own x and y) for these screw
    extensions (m): the three tips lie in one plane, so the extensions follow a plane, raised toward the high side."""
    rows = np.array([[*screw_local(s), 1.0] for s in SCREWS])
    slope = np.linalg.solve(rows, np.array([extension[s] for s in SCREWS]))
    return slope[:2]


def bubble_offset(gradient: np.ndarray) -> np.ndarray:
    """Where the bubble's centre sits from the vial's centre (m), toward the high side, until it meets the edge."""
    offset = VIAL_CURVE * np.asarray(gradient, dtype=float)
    reach = float(np.linalg.norm(offset))
    return offset if reach <= VIAL_REACH else offset * (VIAL_REACH / reach)


START_GRADIENT = seated_gradient(START_EXTENSION)
START_TILT = float(np.linalg.norm(START_GRADIENT))  # 25.0'


def turn_to_level(screw: str, extension: Mapping[str, float] = START_EXTENSION) -> float:
    """The knob turn (rad, counter-clockwise seen from above) that brings a screw to the level setting. A right-hand
    thread: clockwise drives the screw down and lifts the corner, at PITCH per turn."""
    return 2.0 * math.pi * extension[screw] / PITCH


def arcmin(rad: float) -> float:
    return math.degrees(rad) * 60.0


# --- the score --------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Status:
    tilt: float  # rad
    tip_gaps: tuple[float, ...]  # each screw tip's lowest point above the plate (m)
    speed: float
    spin: float
    knob_step: float = 0.0  # the most any knob turned on the base over the last control step (rad)

    @property
    def seated(self) -> bool:
        """Every screw tip stands on the plate: nothing is holding a corner up, or pushing one down into it."""
        return all(-SEAT_PRESS <= gap <= SEAT_GAP for gap in self.tip_gaps)

    @property
    def upright(self) -> bool:
        return self.tilt <= MAX_TILT

    @property
    def still(self) -> bool:
        """The base at rest and no knob being turned (a gripper turning one, or still twisting it)."""
        return self.speed < STILL_SPEED and self.spin < STILL_SPIN and self.knob_step < KNOB_STILL

    @property
    def level(self) -> bool:
        """The bubble in the ring with the instrument standing on its own three screws."""
        return self.seated and self.upright and self.tilt <= TILT_RING

    @property
    def credit(self) -> float:
        """Partial credit by how far the bubble is from the ring (score() never reports 1.0: that is success held
        for 1 s)."""
        if not (self.seated and self.upright):
            return 0.0
        for limit, value in BANDS:
            if self.tilt <= limit:
                return value
        return 0.0


def tip_gap(objects: Mapping[str, Any], screw: str) -> float:
    """How far a screw's rounded tip stands above the plate (m): its lowest point, from the tip's centre (the screw
    body's origin), not the screw's bounding box, whose shank would read low when the instrument tilts."""
    _, origin = _pose(objects[SCREW[screw]], SCREW[screw])
    return float(origin[2] - TIP_RADIUS - FP)


def assess(objects: Mapping[str, Any], knob_steps: Mapping[str, float] | None = None) -> Status:
    """The instrument's state; ``knob_steps`` is how far each knob turned over the last control step (the knobs'
    reported spin is solver jitter, so stillness is read from their angles)."""
    base = objects[BASE]
    base_rot = rotation(base.quat_xyzw)
    return Status(
        tilt=tilt_of(base_rot),
        tip_gaps=tuple(tip_gap(objects, s) for s in SCREWS),
        speed=float(np.linalg.norm(np.asarray(base.lin_vel, dtype=float))),
        spin=float(np.linalg.norm(np.asarray(base.ang_vel, dtype=float))),
        knob_step=max((abs(v) for v in (knob_steps or {}).values()), default=0.0),
    )


def score(objects: Mapping[str, Any]) -> float:
    """0.75 with the bubble in the ring (tilt 4' or less), 0.5 touching it (8'), 0.25 within the compensator's
    range (15'), else 0; 0 if a screw tip is off the plate (the instrument held up, not standing on its screws), and
    0 unless the instrument is as built (see fault())."""
    if fault(objects):
        return 0.0
    return assess(objects).credit


def success(objects: Mapping[str, Any]) -> bool:
    """Level (bubble in the ring), standing on its own three screws, the base still and no knob being turned; held
    for 1 s it ends the trial."""
    if fault(objects):
        return False
    status = assess(objects, _TURNS.step)
    return status.level and status.still


# --- the instrument as built: checked every step before anything is scored -------------------------------------
# The score trusts no constant it can check. The scene hook notes, in a ledger shared by every load of this file in
# the process (kept on yam_arena.scene, the module it wraps), that every body got its parts and that every joint was
# authored; until it says so nothing scores. Then every step: every number the simulator reports must be finite (a
# NaN or an infinity zeroes the rest of the trial); each body's reported outline must be the shape built, parts
# included (it cannot change, so a mismatch also zeroes the rest of the trial); and each joint must hold its two
# bodies where it should. Anything more than 0.5 mm off scores 0. This file reads only the objects' states: no joint
# readings, no grippers.
GEOMETRY_TOL = 0.0005
# A screw must stand within 0.15 mm of where its knob's turn has driven it (54 degrees of turn). On the GPU an idle
# screw stays within 0.05 mm of that (its spring gives a little as the load shifts) and a gripper pressing on a knob
# moves it up to 0.4 mm (those steps score 0); a hand pushing the base level would need two screws to move 0.6 mm.
THREAD_TOL = 0.00015
# A screw that settles further than this from where it was set fails the trial's start: a thread that does not hold
# (a PhysX rack-and-pinion tried first left 1.7 mm), not the springs' give under the weight.
REST_LIMIT = 0.0012
QUAT_TOL = 1.0e-3
FIELDS = ("pos", "quat_xyzw", "lin_vel", "ang_vel", "low", "high")
PARTED = frozenset(part.body for part in PARTS)  # the bodies the hook adds parts to
_LEDGER = "_level_auto_level_build"


def built_outline(name: str) -> tuple[np.ndarray, np.ndarray]:
    """The low and high corners of an object's own shape and its parts, in its frame as spawned: the box the
    simulator reports the object's bounds from."""
    spec = next(o for o in OBJECTS if o.name == name)
    half = np.full(3, float(spec.radius))
    if hasattr(spec, "height"):
        half[2] = spec.height / 2
    low, high = -half, half.copy()
    for part in PARTS:
        if part.body != name:
            continue
        if part.shape == "box":
            c, s = abs(math.cos(part.yaw)), abs(math.sin(part.yaw))
            x, y, z = (v / 2 for v in part.size)
            extent = np.array([c * x + s * y, s * x + c * y, z])
        else:
            extent = np.full(3, part.radius)
            extent["XYZ".index(part.axis)] = part.length / 2
        centre = np.array(part.centre, dtype=float)
        low, high = np.minimum(low, centre - extent), np.maximum(high, centre + extent)
    return low, high


OUTLINES = {name: built_outline(name) for name in (*BODIES, PLATE)}
# (body, the body it is jointed to, the joint's anchor as spawned, its travel along that body's z: None for a hinge
# or a pivot, which holds the two together at the anchor)
JOINT_CHECKS = (
    (BASE, PLATE, HOLD_ANCHOR, HOLD_TRAVEL),
    (SCOPE, BASE, AXIS_ANCHOR, None),
    *((KNOB[s], BASE, SPAWN[KNOB[s]], None) for s in SCREWS),
    *((WASHER[s], BASE, SPAWN[WASHER[s]], WASHER_TRAVEL) for s in SCREWS),
    *((SCREW[s], BASE, SPAWN[SCREW[s]], (-TRAVEL, TRAVEL)) for s in SCREWS),
    (BUBBLE, SCOPE, PIVOT_ANCHOR, None),
)


def finite(objects: Mapping[str, Any]) -> bool:
    """Every object of this task is reported; every field of every state has its shape (3 numbers, 4 for the
    orientation); every number is finite; and every orientation is a unit quaternion (to 1e-3)."""
    try:
        if any(o.name not in objects for o in OBJECTS):
            return False
        values = []
        for state in objects.values():
            for f in FIELDS:
                value = np.asarray(getattr(state, f), dtype=float)
                if value.shape != ((4,) if f == "quat_xyzw" else (3,)):
                    return False
                values.append(value)
        if not np.isfinite(np.concatenate(values)).all():
            return False
        norms = (
            float(np.linalg.norm(np.asarray(state.quat_xyzw, dtype=float)))
            for state in objects.values()
        )
        return all(abs(n - 1.0) <= QUAT_TOL for n in norms)
    except (AttributeError, TypeError, ValueError):
        return False


def _pose(state: Any, name: str) -> tuple[np.ndarray, np.ndarray]:
    """An object's rotation and origin (the point that was at JOINT_ROOTS), from its reported bounding-box centre."""
    rot = rotation(state.quat_xyzw)
    low, high = OUTLINES[name]
    return rot, np.asarray(state.pos, dtype=float) - rot @ ((low + high) / 2)


def _joint_gap(poses: Mapping[str, Any], child: str, parent: str, anchor: Any) -> np.ndarray:
    """How far the child's copy of a joint's anchor is from the parent's, in the parent's frame (m)."""
    (r0, o0), (r1, o1) = poses[parent], poses[child]
    a = np.asarray(anchor, dtype=float)
    held = o1 + r1 @ (a - np.asarray(JOINT_ROOTS[child]))
    holder = o0 + r0 @ (a - np.asarray(JOINT_ROOTS[parent]))
    return np.asarray(r0.T @ (held - holder))


def as_built(
    objects: Mapping[str, Any], turns: Mapping[str, float], rest: Mapping[str, float] | None = None
) -> tuple[str, bool, dict[str, float]]:
    """("", False, rises) if every body's outline is the shape built, every joint holds its bodies where it should,
    every screw has settled within REST_LIMIT of where it was set and stands where its knob's turn (``turns``, rad)
    has driven it from its ``rest``; otherwise what is off, and True if it cannot come right in this trial (an
    outline). ``rises`` is each screw's slide on its thread from where it was set (m, + up into the base)."""
    poses = {}
    for name, (low, high) in OUTLINES.items():
        state = objects[name]
        rot, origin = _pose(state, name)
        reported = np.asarray(state.high, dtype=float) - np.asarray(state.low, dtype=float)
        off = float(np.max(np.abs(reported - np.abs(rot) @ (high - low))))
        if off > GEOMETRY_TOL:
            return f"{name}'s outline is {1000 * off:.2f} mm off the shape built", True, {}
        poses[name] = (rot, origin)
    for child, parent, anchor, travel in JOINT_CHECKS:
        gap = _joint_gap(poses, child, parent, anchor)
        if travel is not None:
            gap[2] -= float(np.clip(gap[2], travel[0], travel[1]))
        off = float(np.linalg.norm(gap))
        if off > GEOMETRY_TOL:
            return f"{child} is {1000 * off:.2f} mm off its joint", False, {}
    # The thread: a screw that slides without its knob turning (a coupling that does nothing, or a hand pushing the
    # base level) is not a working foot screw. Each is judged from where it settled at the trial's first step.
    rises = {s: float(_joint_gap(poses, SCREW[s], BASE, SPAWN[SCREW[s]])[2]) for s in SCREWS}
    for s in SCREWS:
        settled = rises[s] - screw_target(turns[s])
        if abs(settled) > REST_LIMIT:
            return (
                f"the {s} screw stands {1000 * settled:.2f} mm off where it was set",
                False,
                rises,
            )
        if rest is not None and abs(settled - rest[s]) > THREAD_TOL:
            off = abs(settled - rest[s])
            return (
                f"the {s} screw is {1000 * off:.2f} mm off where its knob's turn puts it",
                False,
                rises,
            )
    return "", False, rises


def _ledger(create: bool = False) -> dict[str, Any] | None:
    """The build ledger every load of this file shares, kept on the scene module the hook wraps."""
    scene = sys.modules.get(_SCENE_MODULE)
    if scene is None:
        return None
    ledger = getattr(scene, _LEDGER, None)
    if not isinstance(ledger, dict):
        if not create:
            return None
        ledger = {}
        setattr(scene, _LEDGER, ledger)
    return ledger


def _stage_key(stage: Any) -> str:
    """A stage's root layer: the same for every Python handle on one stage, different for another stage."""
    try:
        return str(stage.GetRootLayer().identifier)
    except Exception:
        return ""


def _record(stage: Any, body: str, built: bool, joints: Sequence[str] = (), root: str = "") -> None:
    """Note what the hook made of ``body``; the first body built on a new stage starts a new record (a failure to
    install the hook is kept)."""
    ledger = _ledger(create=True)
    if ledger is None:
        return
    key = _stage_key(stage)
    if ledger.get("stage") != key:
        kept = ledger.get("hook_failed")
        ledger.clear()
        ledger.update(
            stage=key, parts=set(), joints=False, failed=False, joint_paths=[], threads={}
        )
        if kept:
            ledger["hook_failed"] = kept
    if not built:
        ledger["failed"] = True
        return
    ledger["parts"].add(body)
    if body == BUBBLE:  # the last body: its spawner authored every joint
        ledger["joints"] = True
        ledger["joint_paths"] = list(joints)
        ledger["threads"] = {s: f"{root}/{JOINTS_FOLDER}/thread_{s}" for s in SCREWS}
        ledger["axis"] = f"{root}/{JOINTS_FOLDER}/vertical_axis"


def _hook_failed(reason: str) -> None:
    """The hook could not be installed or run: record it (nothing then scores) and warn."""
    ledger = _ledger(create=True)
    if ledger is not None:
        ledger["hook_failed"] = reason
    _warn_once("hook-install", reason)


def mechanism_built() -> bool:
    """The hook authored every body's parts and every joint, nothing failed, and the stage being simulated now
    holds every joint the ledger lists (a record from another stage does not count)."""
    ledger = _ledger()
    if not ledger or ledger.get("hook_failed") or ledger.get("failed") is not False:
        return False
    if ledger.get("joints") is not True or not PARTED.issubset(ledger.get("parts", ())):
        return False
    paths = ledger.get("joint_paths") or []
    if not paths:
        return False
    try:
        stage = importlib.import_module("omni.usd").get_context().get_stage()
        return all(bool(stage.GetPrimAtPath(path).IsValid()) for path in paths)
    except Exception:
        return False


@dataclass
class Trial:
    """What this trial has shown that can never come right (a reading that was not finite, an outline unlike the
    shape built): empty while the trial may still score. ``rest`` is where each screw settled on its thread at the
    trial's first step, less any turn by then (m): the threads are judged from there."""

    spoiled: str = ""
    rest: dict[str, float] | None = None
    scope_hold: float = 0.0  # the telescope's set point on its axis (rad)


_TRIAL = Trial()
_verdict: tuple[object, str] = (None, "")


def fault(objects: Mapping[str, Any]) -> str:
    """Why this step scores nothing ("" if it may score), worked out once per control step (success() and score()
    share the step's state dict). It also follows each knob's turn, and with the "drive" coupling moves each screw's
    set point to it. It never raises: anything it cannot read scores nothing."""
    global _verdict
    if _verdict[0] is objects:
        return _verdict[1]
    try:
        _follow_episodes()
    except Exception as error:  # an odd caller on the stack: carry on without it
        _warn_once("episodes", f"could not follow the episodes ({type(error).__name__}: {error})")
    try:
        why = _judge(objects)
    except Exception as error:
        why = f"could not read the instrument ({type(error).__name__}: {error})"
    _verdict = (objects, why)
    return why


def _judge(objects: Mapping[str, Any]) -> str:
    readable = finite(objects)
    why = _TRIAL.spoiled
    if not why and not readable:
        why = _TRIAL.spoiled = "a reported state is not a finite, well-formed reading"
    if readable:
        base_rot = rotation(objects[BASE].quat_xyzw)
        _TURNS.advance(
            {s: knob_angle(base_rot, rotation(objects[KNOB[s]].quat_xyzw)) for s in SCREWS}
        )
        _write_targets({s: screw_target(_TURNS.total[s]) for s in SCREWS})
        _hold_telescope(knob_angle(base_rot, rotation(objects[SCOPE].quat_xyzw)))
    if not why and not mechanism_built():
        why = "the instrument's parts and joints were not all built"
    if not why:
        why, for_good, rises = as_built(objects, _TURNS.total, _TRIAL.rest)
        if for_good:
            _TRIAL.spoiled = why
        elif (
            not why and _TRIAL.rest is None
        ):  # the trial's first sound step: where each screw settled
            _TRIAL.rest = {s: rises[s] - screw_target(_TURNS.total[s]) for s in SCREWS}
    return why


# --- the "drive" coupling's set points, and episodes ------------------------------------------------------------
@dataclass
class Turns:
    """Each knob's turn from its start (rad), unwrapped across control steps."""

    total: dict[str, float]
    last: dict[str, float]
    step: dict[str, float] = field(default_factory=lambda: dict.fromkeys(SCREWS, 0.0))

    @staticmethod
    def fresh() -> "Turns":
        return Turns(total=dict.fromkeys(SCREWS, 0.0), last=dict.fromkeys(SCREWS, 0.0))

    def advance(self, angles: Mapping[str, float]) -> None:
        for s in SCREWS:
            step = (angles[s] - self.last[s] + math.pi) % (2.0 * math.pi) - math.pi
            self.total[s] += step
            self.last[s] = angles[s]
            self.step[s] = step


def knob_angle(base_rot: np.ndarray, knob_rot: np.ndarray) -> float:
    """A knob's turn on the base about the base's vertical axis (rad, counter-clockwise seen from above)."""
    rel = base_rot.T @ knob_rot
    return math.atan2(float(rel[1, 0]), float(rel[0, 0]))


def screw_target(turn: float) -> float:
    """The screw's rise on its prismatic joint (m, + up into the base) for a knob turned ``turn`` rad."""
    return PITCH * turn / (2.0 * math.pi)


_TURNS = Turns.fresh()
_hooked_to: Any = None
_episode: tuple[Any, float] = (
    None,
    math.inf,
)  # the embodiment last seen scoring, and its episode clock then
_warned: set[str] = set()
RESET_TAG = "level_auto_level_new_episode"


def _warn_once(key: str, message: str) -> None:
    """Warn once per kind. A warning never raises out of this file (under -W error it would)."""
    if key not in _warned:
        _warned.add(key)
        with contextlib.suppress(Exception):
            warnings.warn(f"level_auto_level: {message}", RuntimeWarning, stacklevel=3)


def new_episode() -> None:
    """A new trial: a clean record, every knob back at its start, and with the "drive" coupling every screw's set
    point too."""
    global _TURNS, _TRIAL, _verdict
    _TURNS = Turns.fresh()
    _TRIAL = Trial()
    _verdict = (None, "")
    _write_targets(dict.fromkeys(SCREWS, 0.0))
    _write_scope_hold(0.0)


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


def _follow_episodes() -> None:
    """Start a new trial when another embodiment calls, or when the calling embodiment's episode clock has not moved
    on (every episode's first scored step has the clock at one control step, so a clock that goes back, or stays,
    is a new episode); and wrap that embodiment's reset once, so a reset starts one too. There is no hook for this
    in the task-file API: the embodiment is found by walking the call stack (see _calling_embodiment)."""
    global _episode
    embodiment = _calling_embodiment()
    if embodiment is None:
        return
    clock = float(embodiment._t)
    last, last_clock = _episode
    if embodiment is not last or not clock > last_clock:
        new_episode()
    _episode = (embodiment, clock)
    _hook_resets(embodiment)


def _hook_resets(embodiment: Any) -> None:
    """Wrap the embodiment's reset once, so each new trial calls new_episode(). The wrapper keeps a list of callbacks:
    every load of this file in the process adds its own, and no reset is wrapped twice."""
    global _hooked_to
    if embodiment is _hooked_to:
        return
    callbacks = getattr(embodiment.reset, RESET_TAG, None)
    if not isinstance(callbacks, list):
        original = embodiment.reset
        callbacks = []

        @functools.wraps(original)
        def reset(*args: Any, **kwargs: Any) -> Any:
            for callback in list(callbacks):
                callback()
            return original(*args, **kwargs)

        setattr(reset, RESET_TAG, callbacks)
        try:
            embodiment.reset = reset
        except (AttributeError, TypeError):
            _warn_once(
                "reset",
                "could not hook the embodiment's reset; its episode clock still starts trials",
            )
            return
    if new_episode not in callbacks:
        callbacks.append(new_episode)
    _hooked_to = embodiment


def _write_targets(rises: Mapping[str, float]) -> None:
    """Move each screw's spring set point (the "drive" coupling) on the stage. The joints' paths are in the shared
    ledger, so whichever load of this file is scoring finds the joints another load's hook authored."""
    threads = (_ledger() or {}).get("threads") or {}
    if not threads:
        return
    try:
        stage = importlib.import_module("omni.usd").get_context().get_stage()
        for s, rise in rises.items():
            prim = stage.GetPrimAtPath(threads[s])
            prim.GetAttribute("drive:linear:physics:targetPosition").Set(float(rise))
    except Exception as error:
        _warn_once(
            "drive", f"could not set the screws' set points ({type(error).__name__}: {error})"
        )


def _hold_telescope(turn: float) -> None:
    """Move the telescope's set point along only where it has been turned more than SCOPE_SLIP past it."""
    off = (turn - _TRIAL.scope_hold + math.pi) % (2.0 * math.pi) - math.pi
    if abs(off) > SCOPE_SLIP:
        _TRIAL.scope_hold = turn - math.copysign(SCOPE_SLIP, off)
        _write_scope_hold(_TRIAL.scope_hold)


def _write_scope_hold(turn: float) -> None:
    """Set the vertical axis's spring set point (degrees on the stage), from the shared ledger's joint path."""
    axis = (_ledger() or {}).get("axis")
    if not axis:
        return
    try:
        stage = importlib.import_module("omni.usd").get_context().get_stage()
        prim = stage.GetPrimAtPath(axis)
        prim.GetAttribute("drive:angular:physics:targetPosition").Set(math.degrees(turn))
    except Exception as error:
        _warn_once("axis", f"could not hold the telescope ({type(error).__name__}: {error})")


# --- the scene hook: the parts and joints the task-file API has no words for ------------------------------------
# yam_arena.scene.task_object(spec) turns each task object into an Arena object. Right after that module is
# imported, this wraps task_object, for this file's own object specs only: each of the instrument's bodies, once spawned, gets its other
# parts (more colliders and looks of the same rigid body), and the bubble's spawner, the last, adds the joints:
# the base held down on the plate by its central spring, the telescope on the base's vertical axis, each knob on
# its hinge and each screw on its thread, and the bubble on its pivot. On a machine without the simulator nothing
# imports yam_arena.scene, and this does nothing.
_SCENE_MODULE = "yam_arena.scene"
PARTS_FOLDER = "parts"
JOINTS_FOLDER = "level_joints"
_HOOK_MARK = "level_auto_level"


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _material(
    stage: Any, root: str, color: tuple[float, float, float], cache: dict[Any, Any]
) -> Any:
    if color in cache:
        return cache[color]
    gf = importlib.import_module("pxr.Gf")
    sdf = importlib.import_module("pxr.Sdf")
    shade = importlib.import_module("pxr.UsdShade")
    path = f"{root}/{PARTS_FOLDER}/look_{len(cache)}"
    material = shade.Material.Define(stage, path)
    shader = shade.Shader.Define(stage, f"{path}/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", sdf.ValueTypeNames.Color3f).Set(
        gf.Vec3f(*(_linear(c) for c in color))
    )
    shader.CreateInput("roughness", sdf.ValueTypeNames.Float).Set(0.45)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    cache[color] = material
    return material


def author_parts(stage: Any, env_root: str, body: str) -> list[str]:
    """Add a body's other parts under its prim in ``env_root``; the paths made."""
    gf = importlib.import_module("pxr.Gf")
    geom = importlib.import_module("pxr.UsdGeom")
    physics = importlib.import_module("pxr.UsdPhysics")
    shade = importlib.import_module("pxr.UsdShade")
    root = f"{env_root}/{body}"
    cache: dict[Any, Any] = {}
    made = []
    for part in PARTS:
        if part.body != body:
            continue
        path = f"{root}/{PARTS_FOLDER}/{part.name}"
        if part.shape == "box":
            prim = geom.Cube.Define(stage, path)
            prim.CreateSizeAttr(1.0)
            # The extent, in the shape's own units before its transform: the simulator's outline of the body is
            # measured from it (left unauthored it could read as the default 2-unit cube).
            prim.CreateExtentAttr([gf.Vec3f(-0.5, -0.5, -0.5), gf.Vec3f(0.5, 0.5, 0.5)])
            xform = geom.Xformable(prim.GetPrim())
            xform.AddTranslateOp().Set(gf.Vec3d(*part.centre))
            xform.AddOrientOp().Set(
                gf.Quatf(math.cos(part.yaw / 2), 0.0, 0.0, math.sin(part.yaw / 2))
            )
            xform.AddScaleOp().Set(gf.Vec3f(*part.size))
        else:
            prim = geom.Cylinder.Define(stage, path)
            prim.CreateRadiusAttr(part.radius)
            prim.CreateHeightAttr(part.length)
            prim.CreateAxisAttr(part.axis)
            half = [part.radius, part.radius, part.radius]
            half["XYZ".index(part.axis)] = part.length / 2
            prim.CreateExtentAttr([gf.Vec3f(*(-h for h in half)), gf.Vec3f(*half)])
            geom.Xformable(prim.GetPrim()).AddTranslateOp().Set(gf.Vec3d(*part.centre))
        if part.collide:
            physics.CollisionAPI.Apply(prim.GetPrim())
        shade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(
            _material(stage, root, part.color, cache)
        )
        made.append(path)
    return made


def _joint_frames(
    joint: Any,
    gf: Any,
    body0: str,
    body1: str,
    anchor: Sequence[float],
    roots: Mapping[str, Any],
    env_root: str,
    rot: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
) -> None:
    """Point a joint's two frames at the same world ``anchor``, both turned ``rot`` (w, x, y, z) from the bodies'
    own frames, which are level and unturned as every body spawns."""
    joint.CreateBody0Rel().SetTargets([f"{env_root}/{body0}"])
    joint.CreateBody1Rel().SetTargets([f"{env_root}/{body1}"])
    joint.CreateLocalPos0Attr(
        gf.Vec3f(*(float(a - o) for a, o in zip(anchor, roots[body0], strict=True)))
    )
    joint.CreateLocalPos1Attr(
        gf.Vec3f(*(float(a - o) for a, o in zip(anchor, roots[body1], strict=True)))
    )
    joint.CreateLocalRot0Attr(gf.Quatf(*rot))
    joint.CreateLocalRot1Attr(gf.Quatf(*rot))


# A quarter turn about y, taking a joint frame's X axis onto the bodies' vertical Z: PhysX turns a hinge about, and
# slides a prismatic joint along, the X axis of its frame. The knobs' hinges and the screws' threads both use it.
AXIS_FRAME = (math.sqrt(0.5), 0.0, -math.sqrt(0.5), 0.0)


def _damper(
    prim: Any, physics: Any, kind: str, damping: float, stiffness: float = 0.0, target: float = 0.0
) -> None:
    drive = physics.DriveAPI.Apply(prim, kind)
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(target)
    drive.CreateTargetVelocityAttr(0.0)


def author_joints(stage: Any, env_root: str) -> list[str]:
    """The instrument's joints and the bubble's physics, under ``env_root``; the paths made."""
    gf = importlib.import_module("pxr.Gf")
    physics = importlib.import_module("pxr.UsdPhysics")
    physx = importlib.import_module("pxr.PhysxSchema")
    usd = importlib.import_module("pxr.Usd")
    roots = JOINT_ROOTS
    folder = f"{env_root}/{JOINTS_FOLDER}"
    importlib.import_module("pxr.UsdGeom").Xform.Define(stage, folder)
    made = []

    # The base, held down on the plate by its central spring: it may tilt (2 degrees) and rise, but not slide or turn.
    stem = physics.Joint.Define(stage, f"{folder}/hold_down")
    _joint_frames(stem, gf, PLATE, BASE, HOLD_ANCHOR, roots, env_root)
    for dof, low, high in (
        ("transX", 1.0, -1.0),
        ("transY", 1.0, -1.0),
        ("rotZ", 1.0, -1.0),
        ("transZ", *HOLD_TRAVEL),
        ("rotX", -2.0, 2.0),
        ("rotY", -2.0, 2.0),
    ):
        limit = physics.LimitAPI.Apply(stem.GetPrim(), dof)
        limit.CreateLowAttr(low)
        limit.CreateHighAttr(high)
    _damper(stem.GetPrim(), physics, "transZ", 30.0, HOLD_STIFFNESS, -HOLD_DOWN / HOLD_STIFFNESS)
    made.append(str(stem.GetPath()))

    # The telescope on the vertical axis, with some friction, and balanced on it as a real one is: its centre of mass
    # on the axis, so a tilted instrument's weight does not turn it.
    axis = physics.RevoluteJoint.Define(stage, f"{folder}/vertical_axis")
    _joint_frames(axis, gf, BASE, SCOPE, AXIS_ANCHOR, roots, env_root)
    axis.CreateAxisAttr("Z")
    # Angular drives on the stage count per degree.
    _damper(
        axis.GetPrim(),
        physics,
        "angular",
        math.radians(SCOPE_DAMPING),
        math.radians(SCOPE_HOLD),
        0.0,
    )
    made.append(str(axis.GetPath()))
    scope = stage.GetPrimAtPath(f"{env_root}/{SCOPE}")
    physics.MassAPI.Apply(scope).CreateCenterOfMassAttr(gf.Vec3f(0.0, 0.0, (TUBE_Z - SCOPE_Z) / 2))
    for name in BODIES:  # enough solver iterations for the light knobs and screws on the heavy base
        for part in usd.PrimRange(stage.GetPrimAtPath(f"{env_root}/{name}")):
            if part.HasAPI(physics.RigidBodyAPI):
                solver = physx.PhysxRigidBodyAPI.Apply(part)
                solver.CreateSolverPositionIterationCountAttr(SOLVER_ITERATIONS[0])
                solver.CreateSolverVelocityIterationCountAttr(SOLVER_ITERATIONS[1])

    for s in SCREWS:
        knob = physics.RevoluteJoint.Define(stage, f"{folder}/knob_{s}")
        _joint_frames(knob, gf, BASE, KNOB[s], SPAWN[KNOB[s]], roots, env_root, AXIS_FRAME)
        knob.CreateAxisAttr("X")  # the frame's X: the vertical
        _damper(knob.GetPrim(), physics, "angular", math.radians(KNOB_DAMPING))
        # The spring washer: it slides up and down on the base without turning, pressed up against the knob.
        washer = physics.PrismaticJoint.Define(stage, f"{folder}/washer_{s}")
        _joint_frames(washer, gf, BASE, WASHER[s], SPAWN[WASHER[s]], roots, env_root)
        washer.CreateAxisAttr("Z")
        washer.CreateLowerLimitAttr(WASHER_TRAVEL[0])
        washer.CreateUpperLimitAttr(WASHER_TRAVEL[1])
        _damper(
            washer.GetPrim(),
            physics,
            "linear",
            5.0,
            WASHER_STIFFNESS,
            WASHER_PRESS / WASHER_STIFFNESS + 0.0002,
        )
        for part in usd.PrimRange(stage.GetPrimAtPath(f"{env_root}/{WASHER[s]}")):
            if part.HasAPI(physics.CollisionAPI):
                physx.PhysxCollisionAPI.Apply(part).CreateTorsionalPatchRadiusAttr(WASHER_PATCH)
        made.append(str(washer.GetPath()))
        thread = physics.PrismaticJoint.Define(stage, f"{folder}/thread_{s}")
        _joint_frames(thread, gf, BASE, SCREW[s], SPAWN[SCREW[s]], roots, env_root, AXIS_FRAME)
        thread.CreateAxisAttr("X")  # the frame's X: the vertical, + up into the base
        thread.CreateLowerLimitAttr(-TRAVEL)
        thread.CreateUpperLimitAttr(TRAVEL)
        made += [str(knob.GetPath()), str(thread.GetPath())]
        _damper(thread.GetPrim(), physics, "linear", SCREW_SPRING_DAMPING, SCREW_STIFFNESS, 0.0)

    # The bubble: the top of a pendulum hanging from the vial's centre of curvature, its weight 5 cm below the
    # pivot, so it rests where a bubble does (the vial's highest point) and stops at the vial's edge.
    pivot = physics.SphericalJoint.Define(stage, f"{folder}/bubble_pivot")
    _joint_frames(pivot, gf, SCOPE, BUBBLE, PIVOT_ANCHOR, roots, env_root)
    pivot.CreateAxisAttr("Z")
    limit = math.degrees(VIAL_REACH / VIAL_CURVE)
    pivot.CreateConeAngle0LimitAttr(limit)
    pivot.CreateConeAngle1LimitAttr(limit)
    made.append(str(pivot.GetPath()))
    bubble = stage.GetPrimAtPath(f"{env_root}/{BUBBLE}")
    for part in usd.PrimRange(bubble):
        if part.HasAPI(physics.CollisionAPI):
            physics.CollisionAPI(part).CreateCollisionEnabledAttr(False)
        if part.HasAPI(physics.RigidBodyAPI):
            mass = physics.MassAPI.Apply(part)
            mass.CreateMassAttr(BUBBLE_MASS)
            mass.CreateCenterOfMassAttr(gf.Vec3f(0.0, 0.0, -(VIAL_CURVE + BUBBLE_DROP)))
            mass.CreateDiagonalInertiaAttr(gf.Vec3f(1.0e-7, 1.0e-7, 1.0e-7))
            mass.CreatePrincipalAxesAttr(gf.Quatf(1.0, 0.0, 0.0, 0.0))
            body = physx.PhysxRigidBodyAPI.Apply(part)
            body.CreateLinearDampingAttr(BUBBLE_DAMPING)
            body.CreateAngularDampingAttr(BUBBLE_DAMPING)
    return made


def _env_roots(prim: Any, body: str) -> list[str]:
    """Every environment folder holding ``body``, given its prim just spawned (cloned copies included)."""
    stage = prim.GetStage()
    envs = prim.GetPath().GetParentPath().GetParentPath()
    return [
        str(env.GetPath())
        for env in stage.GetPrimAtPath(envs).GetChildren()
        if env.GetChild(body).IsValid()
    ]


def _with_parts(spawn: Callable[..., Any], body: str) -> Callable[..., Any]:
    def spawn_and_build(prim_path: str, cfg: Any, *args: Any, **kwargs: Any) -> Any:
        prim = spawn(prim_path, cfg, *args, **kwargs)
        stage, joints, first = None, [], ""
        try:
            stage = prim.GetStage()
            roots = _env_roots(prim, body)
            if not roots:
                raise RuntimeError("no environment holds it")
            # Environment 0 is the one scored.
            first = next((r for r in roots if r.endswith("/env_0")), roots[0])
            for root in roots:
                author_parts(stage, root, body)
                if body == BUBBLE:
                    made = author_joints(stage, root)
                    if root == first:
                        joints = made
        # An instrument without its parts or joints falls apart; the ledger says so, and nothing scores.
        except Exception as error:
            _record(stage, body, built=False)
            _warn_once(f"build-{body}", f"could not build {body} ({type(error).__name__}: {error})")
        else:
            _record(stage, body, built=True, joints=joints, root=first)
        return prim

    setattr(spawn_and_build, _HOOK_MARK, body)
    return spawn_and_build


def _spawner_cfgs(item: object) -> Iterator[Any]:
    """The spawner configs an Arena object holds (one level deep): objects with a callable ``func`` and a ``size``
    or a ``radius``."""
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


def _own(spec: Any) -> bool:
    """One of this file's instrument bodies, by value."""
    try:
        return any(spec == own for own in INSTRUMENT)
    except Exception:
        return False


def build_instrument(scene: Any) -> bool:
    """Wrap ``scene.task_object`` so the instrument's bodies get their parts and joints; True if it wrapped it now.
    It answers only for this file's own object specs, compared by value (a second load of this file makes equal
    specs and is served by the first wrapper; another task's object of the same name is left alone)."""
    original = getattr(scene, "task_object", None)
    if original is None or getattr(original, _HOOK_MARK, False):
        return False

    def task_object(spec: Any) -> Any:
        made = original(spec)
        if _own(spec):
            name = spec.name
            found = 0
            for cfg in _spawner_cfgs(made):
                found += 1
                if not getattr(cfg.func, _HOOK_MARK, None):
                    cfg.func = _with_parts(cfg.func, name)
            if not found:
                _warn_once(
                    f"spawner-{name}", f"found no spawner for {name}; the instrument will be wrong"
                )
        return made

    setattr(task_object, _HOOK_MARK, True)
    scene.task_object = task_object
    return True


class _SceneWatcher(importlib.abc.MetaPathFinder):
    """Waits for yam_arena.scene to be imported, lets it load normally, then applies build_instrument. It answers
    for that one module name only, and removes itself once the module has loaded."""

    marker = "level_auto_level.scene_watcher"

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
        except Exception as error:  # let the import go on without the hook: nothing will score
            self.disarm()
            _hook_failed(f"could not look up the scene module ({type(error).__name__}: {error})")
            return None
        finally:
            self._looking = False
        if spec is not None and spec.loader is not None:
            spec.loader = _BuildAfterLoad(spec.loader, self)
        return spec

    def disarm(self) -> None:
        if self in sys.meta_path:
            sys.meta_path.remove(self)


class _BuildAfterLoad(importlib.abc.Loader):
    """The module's own loader, plus build_instrument once the module has run."""

    def __init__(self, inner: importlib.abc.Loader, watcher: _SceneWatcher) -> None:
        self._inner = inner
        self._watcher = watcher

    def create_module(self, spec: importlib.machinery.ModuleSpec) -> types.ModuleType | None:
        return self._inner.create_module(spec)

    def exec_module(self, module: types.ModuleType) -> None:
        try:
            self._inner.exec_module(module)  # the module's own errors are its own: they propagate
            try:
                build_instrument(module)
            except Exception as error:  # never break importing the scene over the hook
                _hook_failed(f"could not wrap the scene builder ({type(error).__name__}: {error})")
        finally:
            self._watcher.disarm()

    def __getattr__(self, name: str) -> Any:  # get_source and friends, for tracebacks
        if name.startswith("__") or name in ("_inner", "_watcher"):  # a copy made without __init__
            raise AttributeError(name)
        return getattr(self._inner, name)


def install_hook() -> None:
    """Wrap the scene builder now if it is already imported; otherwise wait for its import (once)."""
    scene = sys.modules.get(_SCENE_MODULE)
    if scene is not None:
        try:
            build_instrument(scene)
        except Exception as error:
            _hook_failed(f"could not wrap the scene builder ({type(error).__name__}: {error})")
        return
    if not any(getattr(finder, "marker", None) == _SceneWatcher.marker for finder in sys.meta_path):
        sys.meta_path.insert(0, _SceneWatcher())


# --- the scripted demo -------------------------------------------------------------------------------------------
# Plain open-loop plans: nothing is read from the simulator or the cameras while they run. Every knob turn is planned
# from BUBBLE_READ, the bubble's offset at the fixed start as the left wrist camera shows it when the arm leans in
# over the circular level (the first thing the demo does, so the video shows it), by the textbook three-screw
# procedure and the instrument's published geometry (the vial's sensitivity, the screws' circle and pitch), never from
# the screws' hidden settings. Turning the front pair equally and oppositely moves the bubble parallel to the line
# through them (the left arm turns the front-left knob, then the right arm the front-right one, so the left arm can
# set off for the eyepiece meanwhile); the back screw then centres it. The back knob is under the eyepiece, so the
# left arm swings the telescope 60 degrees clockwise by its eyepiece and leans in over the level again while the right
# arm turns the back knob in two grips. Each knob is gripped from above, its pads 2 mm above the knob's foot, and
# turned by turning the wrist (at most 82.5 degrees per move). The right arm lets go of the back knob as soon as its
# last turn ends, so the fingers are open and rising before the bubble has been still for the 1 s that ends the trial.
GRIP_Z = KNOB_LOW + 0.002 + 0.013  # the TCP is 13 mm short of the pad tips
HOVER_Z = FP + 0.075
KNOB_OPEN = -0.026  # fingers 52 mm apart around a 30 mm knob
TURN_STEP = math.radians(82.5)
SWING = math.radians(-60.0)  # the telescope's turn (clockwise seen from above)
# The left arm grips the eyepiece this far from the vertical axis: 15 mm in from its end, the fingers clear of the
# wider tube.
EYE_REACH = -EYEPIECE_FROM - 0.015
EYE_BEHIND = 0.06  # it comes onto the eyepiece along the tube's axis from this far behind it
SIDE_OUT = 0.15  # its way round the instrument's left side, clear of the vial and the housing
PARK = {"left": (0.20, 0.25, TOP + 0.22), "right": (0.20, -0.25, TOP + 0.22)}
PARK_YAW = {"left": SWING, "right": math.radians(90.0)}
WAIT = {"right": (0.20, -0.22, FP + 0.15)}
# Leaning in over the circular level: the left wrist camera (70 mm beside the gripper, looking down and in) then sees
# the vial from 11 cm, the bubble about 12 pixels across. Each look is where the TCP goes from the vial's centre
# (world, m) with the wrist turned as given, worked out from the official rig's wrist-camera mount: the first with
# the telescope pointing away, the second with it swung, turned to keep clear of the right arm at the back knob.
FIRST_LOOK_YAW, FIRST_LOOK_FROM = math.radians(-90.0), (-0.0002, 0.0217, 0.040)
LOOK_YAW, LOOK_FROM = math.radians(-135.0), (0.0152, 0.0154, 0.040)
# The bubble's offset at the start as it settles (START_SETTLED), as the left wrist camera shows it at the first look:
# its centre 5.2 mm toward the robot and 3.2 mm to its left of the ring's centre, seen from above (read to 0.1 mm;
# the telescope points away, so the vial's axes are the world's).
BUBBLE_READ = (-0.0052, 0.0032)


def plan_from_bubble(bubble: Sequence[float]) -> dict[str, float]:
    """Each knob's turn (rad, + counter-clockwise) that centres a bubble seen ``bubble`` (m, x and y) from the ring's
    centre, by the textbook procedure: the front pair turned equally and oppositely takes out the bubble's offset
    along the line through them, and the back screw the rest. The bubble stands toward the high side, SENSITIVITY rad
    of tilt per m; a screw rises PITCH per turn."""
    tilt = SENSITIVITY * np.asarray(bubble, dtype=float)
    pair = seated_gradient(
        {"back": 0.0, "left": -1.0, "right": 1.0}
    )  # per m the left screw rises, the right drops
    back = seated_gradient({"back": -1.0, "left": 0.0, "right": 0.0})  # per m the back screw rises
    rise, back_rise = np.linalg.solve(np.column_stack([pair, back]), -tilt)
    per_m = 2.0 * math.pi / PITCH
    return {
        "left": float(rise * per_m),
        "right": float(-rise * per_m),
        "back": float(back_rise * per_m),
    }


PLAN = plan_from_bubble(BUBBLE_READ)
GRIPS = {
    "left": 1,
    "right": 1,
    "back": 2,
}  # each front knob in one grip, the back knob's longer turn in two
# The wrist's yaw (about the vertical) at which its roll joint is centred, gripping each knob with the arm that turns
# it (from the official IK: the left arm at the front-left knob, the right arm at the others).
WRIST_CENTRE = {
    "left": math.radians(-37.0),
    "right": math.radians(37.0),
    "back": math.radians(61.0),
}


def grip_yaw(screw: str, turn: float) -> float:
    """The yaw to grip a knob for a one-grip turn of ``turn`` (rad): the fingers cross the knob's line to the vertical
    axis halfway through the turn, while they are closed, so where they open around the knob, at either end, they lie
    half the turn off that line, clear of the housing. Of the two yaws that do so (half a turn
    apart), the one that keeps the wrist nearer the middle of its range."""
    toward_axis = SCREW_ANGLE[screw] + math.pi
    yaw = toward_axis - turn / 2 - math.pi / 2  # yaw 0 closes the fingers along world y
    candidates = [yaw + k * math.pi for k in range(-3, 4)]
    return min(candidates, key=lambda y: abs(y + turn / 2 - WRIST_CENTRE[screw]))


GRIP_YAW = {s: grip_yaw(s, PLAN[s] / GRIPS[s]) for s in SCREWS}


def _turns(turn: float) -> list[float]:
    """A turn split into moves of at most TURN_STEP."""
    n = max(1, math.ceil(abs(turn) / TURN_STEP - 1e-9))
    return [turn / n] * n


def knob_turn(screw: str, start_yaw: float, turn: float) -> list[Any]:
    """Grip a knob from above, turn it by ``turn`` (rad, + counter-clockwise) in one grip, let go and rise."""
    x, y = screw_xy(screw)
    plan = [
        Waypoint(1.6, (x, y, HOVER_Z), KNOB_OPEN, f"over the {screw} knob", start_yaw),
        Waypoint(1.0, (x, y, GRIP_Z), KNOB_OPEN, "down around it", start_yaw),
        Waypoint(0.7, (x, y, GRIP_Z), FINGER_CLOSED, "grip", start_yaw),
    ]
    yaw = start_yaw
    for step in _turns(turn):
        yaw += step
        plan.append(Waypoint(2.0, (x, y, GRIP_Z), FINGER_CLOSED, "turn", yaw))
    plan += [
        Waypoint(0.6, (x, y, GRIP_Z), KNOB_OPEN, "let go", yaw),
        Waypoint(0.8, (x, y, HOVER_Z), KNOB_OPEN, "up", yaw),
    ]
    return plan


def regripped_turn(screw: str, start_yaw: float, turn: float, grips: int) -> list[Any]:
    """A knob turn made in ``grips`` equal grips, the wrist turned back over the knob between them."""
    x, y = screw_xy(screw)
    plan: list[Any] = []
    for i in range(grips):
        part = knob_turn(screw, start_yaw, turn / grips)
        if i:
            yaw = plan[-1].yaw
            back = []
            for step in _turns(start_yaw - yaw):
                yaw += step
                back.append(Waypoint(0.8, (x, y, HOVER_Z), KNOB_OPEN, "turn the wrist back", yaw))
            part = back + part[1:]
        plan += part
    return plan


def _eyepiece(turn: float, reach: float = EYE_REACH) -> tuple[float, float]:
    """Where the left arm grips the eyepiece (or a point ``reach`` out along the telescope's axis behind it) with the
    telescope turned ``turn`` (rad) from pointing away."""
    return (X0 - reach * math.cos(turn), Y0 - reach * math.sin(turn))


def _vial(turn: float) -> tuple[float, float]:
    """The circular level's centre (world x, y) with the telescope turned ``turn``."""
    c, s = math.cos(turn), math.sin(turn)
    return (X0 + c * VIAL_XY[0] - s * VIAL_XY[1], Y0 + s * VIAL_XY[0] + c * VIAL_XY[1])


def lean_in_over_the_bubble() -> list[Any]:
    """Lean in over the circular level (the telescope pointing away) and hold, the bubble in the wrist camera."""
    x, y = _vial(0.0)
    look = (x + FIRST_LOOK_FROM[0], y + FIRST_LOOK_FROM[1], VIAL_TOP + FIRST_LOOK_FROM[2])
    return [
        Waypoint(2.4, look, FINGER_CLOSED, "lean in over the bubble", FIRST_LOOK_YAW),
        Waypoint(1.5, look, FINGER_CLOSED, "hold over the bubble", FIRST_LOOK_YAW),
    ]


def reach_the_eyepiece() -> list[Any]:
    """Round the instrument's left side to behind the eyepiece, then along the tube's axis onto it, the fingers
    wide open across it (the telescope pointing away, as it starts)."""
    x = screw_xy("left")[0]
    ex, ey = _eyepiece(0.0)
    return [
        Waypoint(1.2, (x, Y0 + SIDE_OUT, HOVER_Z), FINGER_OPEN, "out to the left", 0.0),
        Waypoint(
            1.6, (ex - EYE_BEHIND, Y0 + SIDE_OUT, HOVER_Z), FINGER_OPEN, "back past the vial", 0.0
        ),
        Waypoint(1.6, (ex - EYE_BEHIND, ey, TUBE_Z), FINGER_OPEN, "behind the eyepiece", 0.0),
        Waypoint(1.2, (ex, ey, TUBE_Z), FINGER_OPEN, "onto the eyepiece", 0.0),
    ]


def swing_telescope(angle: float) -> list[Any]:
    """Grip the eyepiece (the fingers across the tube), carry it round the vertical axis by ``angle``, let go and
    back off along the telescope's axis, clear of it."""
    plan = [Waypoint(0.7, (*_eyepiece(0.0), TUBE_Z), FINGER_CLOSED, "grip", 0.0)]
    steps = 3
    for i in range(1, steps + 1):
        a = angle * i / steps
        plan.append(Waypoint(1.2, (*_eyepiece(a), TUBE_Z), FINGER_CLOSED, "swing the telescope", a))
    end = _eyepiece(angle)
    plan += [
        Waypoint(0.6, (*end, TUBE_Z), FINGER_OPEN, "let go", angle),
        Waypoint(
            1.0, (*_eyepiece(angle, EYE_REACH + EYE_BEHIND), TUBE_Z), FINGER_OPEN, "back off", angle
        ),
    ]
    return plan


def lean_in_again(turn: float) -> list[Any]:
    """From behind the eyepiece (the fingers open across the telescope's axis), close the fingers, move out to the
    circular level's side and along the telescope to lean in over it."""
    u = (math.cos(turn), math.sin(turn))  # along the telescope, toward the sunshade
    n = (-u[1], u[0])  # toward its left side, where the circular level is
    vx, vy = _vial(turn)
    look = (vx + LOOK_FROM[0], vy + LOOK_FROM[1], VIAL_TOP + LOOK_FROM[2])
    behind = _eyepiece(turn, EYE_REACH + EYE_BEHIND)
    out = (look[0] - X0) * n[0] + (look[1] - Y0) * n[
        1
    ]  # how far the look is to the telescope's side
    beside = (behind[0] + n[0] * out, behind[1] + n[1] * out)
    return [
        Waypoint(0.5, (*behind, TUBE_Z), FINGER_CLOSED, "close the fingers", turn),
        Waypoint(1.2, (*beside, TUBE_Z), FINGER_CLOSED, "out to the level's side", turn),
        Waypoint(1.4, look, FINGER_CLOSED, "lean in over the bubble", LOOK_YAW),
    ]


def _duration(plan: Sequence[Any]) -> float:
    return float(sum(w.duration for w in plan))


def _hold(plan: list[Any], until: float, label: str) -> list[Any]:
    """Pad a plan with a wait where it ends, until ``until`` seconds."""
    left = until - _duration(plan)
    if left > 1e-9:
        last = plan[-1]
        plan.append(Waypoint(left, last.pos, last.finger, label, last.yaw))
    return plan


def demo() -> dict[str, list[Any]]:
    """Lean in over the bubble; the front pair (left, then right) equally and oppositely, as planned from the start's
    bubble offset; the telescope swung clear; the back knob in two grips while the left arm leans in again."""
    left = lean_in_over_the_bubble()
    left += knob_turn("left", GRIP_YAW["left"], PLAN["left"])
    right = [Waypoint(1.6, WAIT["right"], KNOB_OPEN, "out of the way", GRIP_YAW["right"])]
    right = _hold(
        right, _duration(left) + 1.2, "wait for the left arm"
    )  # until it is out to the left
    left += reach_the_eyepiece()
    right += knob_turn("right", GRIP_YAW["right"], PLAN["right"])
    right.append(Waypoint(1.6, WAIT["right"], KNOB_OPEN, "out of the way", right[-1].yaw))
    left = _hold(left, _duration(right), "wait for the right arm")
    left += swing_telescope(SWING)
    right = _hold(right, _duration(left), "wait for the telescope")
    left += lean_in_again(SWING)
    right += regripped_turn("back", GRIP_YAW["back"], PLAN["back"], grips=GRIPS["back"])
    right.append(Waypoint(1.6, PARK["right"], FINGER_OPEN, "park", PARK_YAW["right"]))
    end = max(_duration(left), _duration(right)) + 4.0
    return {"left": _hold(left, end, "hold"), "right": _hold(right, end, "hold")}


def _simulator_present() -> bool:
    """Whether Isaac Lab can be imported here: the hook is only installed where the scene can be built."""
    try:
        return importlib.util.find_spec("isaaclab") is not None
    except (ImportError, ValueError):
        return False


if _simulator_present():
    try:
        install_hook()
    # Never break loading the task over the hook; the instrument then falls apart.
    except Exception as error:
        _warn_once("install", f"could not install the scene hook ({type(error).__name__}: {error})")
