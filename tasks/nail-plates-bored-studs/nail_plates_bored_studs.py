"""On the station table: a short stud-wall panel lying on a framing table, as wall panels are built
flat in an off-site panel plant before they are stood up. Five pine 2x4 studs lie on edge, running
away from the robot, 120 mm apart, with a 38 mm face up: the face the drywall will be screwed to. A
yellow electrical cable runs left to right through a 22 mm hole bored across each stud, 270 mm in
front of the arms. Each hole shows as a dark square on both side faces of its stud, so its depth below
the face is read from the stud's side, as an electrician does. The holes were bored at different
depths, so the cable steps up and down between studs. In front of the studs, nearer the arms, thin
galvanized steel nail plates (76 x 38 mm) lie flat on narrow dark grey stands, one plate per stand.

The electrical code requires the edge of a bored hole to be at least 1-1/4 in (31.75 mm) from the
stud's face. Where it is closer, a steel nail plate goes flat on that face over the hole, so drywall
screws can't reach the cable. Plate only the studs that need it: a plate on a stud that doesn't need
one costs credit, and plates you don't need can stay on their stands.
"""

# No `from __future__ import annotations` here: the task loader imports this file without registering
# it in sys.modules, and dataclasses then cannot resolve string annotations.
import importlib
import itertools
import math
import sys
import warnings
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from types import FrameType
from typing import Any, Protocol

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP: float = TABLE_TOP[SCENE]

# --- the wall (world frame: x away from the arms, y to the left, z up; metres) ---------------------
STUD_WIDTH = 0.038  # a 2x4's 1-1/2 in face, up: the drywall face
STUD_DEPTH = 0.089  # its 3-1/2 in depth, standing on edge
STUD_X = (0.160, 0.440)  # near and far ends
STUD_Y = (-0.240, -0.120, 0.0, 0.120, 0.240)
STUD_TOP = TOP + STUD_DEPTH  # the up face
HOLE_X = 0.270  # every hole's centre, along the studs
HOLE_RADIUS = 0.011  # a 7/8 in bit, the usual size for one cable
# How far below the up face each hole's centre lies (one per stud, in STUD_Y's order). Their edges are
# 22, 25, 33.5, 17 and 34 mm below it.
HOLE_DEPTH = (0.033, 0.036, 0.0445, 0.028, 0.045)
CODE_EDGE = 0.03175  # NEC 300.4(A)(1): hole edge at least 1-1/4 in from the stud's nearest edge
BORING_EDGE = 0.016  # IRC R602.6 item 2: a bored hole's edge at least 5/8 in from the stud's edge
CABLE_SIZE = (0.011, 0.006)  # a flat two-wire NM cable: width (x) and thickness (z)
CABLE_END = 0.300  # it runs out to y -0.3 and +0.3

# --- the plates ------------------------------------------------------------------------------------
PLATE_SIZE = (0.076, 0.038, 0.0016)  # 3 x 1-1/2 in, 1/16 in steel (the code's minimum)
PLATE_MASS = 0.036  # steel: 7850 kg/m^3
PEDESTAL_SIZE = (0.060, 0.020, 0.040)  # narrower than a plate: the fingers close below its edges
RACK_X = 0.100
RACK_Y = (-0.150, -0.050, 0.050, 0.150)
PLATES = ("plate_0", "plate_1", "plate_2", "plate_3")  # on the stands at RACK_Y, right to left

# --- the rules --------------------------------------------------------------------------------------
FLAT_TILT = math.radians(5.0)
ON_FACE = 0.001  # a protecting plate's underside within this of the stud's up face
ALONG = math.radians(10.0)  # its long side within this of the stud's direction
ACROSS = 0.003  # its centre within this of the stud's centreline: it spans the 38 mm face
OVER_HOLE = 0.020  # and within this of the hole's centre along the stud: 38 - 20 >= 11, the hole
RESTS_ON = 0.005  # a plate resting on a stud's face (to within this) counts against a wrong stud
CLEAR = 0.05  # let go: every gripper's fingers and body at least this far from the plate
GRIPPER_BACK = 0.134  # the gripper reaches this far behind the TCP along its approach (its origin)
PAD_TIP = 0.013  # and this far ahead of it (the pad tips)
STILL = 0.01  # m/s
HOLD_S = 1.0  # full credit once everything has held for this long
DEFAULT_DT = 0.05  # the control step if the embodiment doesn't say (20 Hz)
STEP = 0.25

WOOD = (0.80, 0.66, 0.45)
HOLE_DARK = (0.05, 0.035, 0.025)
CABLE = (0.95, 0.80, 0.15)
STEEL = (0.72, 0.74, 0.76)
STAND = (0.25, 0.26, 0.28)


def top_edge(i: int) -> float:
    """How far stud i's hole edge lies below the up face (m)."""
    return HOLE_DEPTH[i] - HOLE_RADIUS


def bottom_edge(i: int) -> float:
    """How far stud i's hole edge lies above the down face (m)."""
    return STUD_DEPTH - HOLE_DEPTH[i] - HOLE_RADIUS


def needs_plate(i: int) -> bool:
    """The code's test: the hole's edge is closer than 1-1/4 in to the up face (the down face can't be
    plated here, and no hole is close to it)."""
    return top_edge(i) < CODE_EDGE


VIOLATING = tuple(i for i in range(len(STUD_Y)) if needs_plate(i))
COMPLIANT = tuple(i for i in range(len(STUD_Y)) if not needs_plate(i))


def _cable_boxes() -> list[Box]:
    """The cable as fixed boxes: straight through each stud at its hole's height, a short riser midway
    between studs, and tails out past the end studs."""
    width, thick = CABLE_SIZE
    heights = [STUD_TOP - d for d in HOLE_DEPTH]
    edges = [-CABLE_END] + [(a + b) / 2 for a, b in itertools.pairwise(STUD_Y)] + [CABLE_END]
    out = []
    for i, z in enumerate(heights):
        y0, y1 = edges[i], edges[i + 1]
        out.append(
            Box(
                name=f"cable_{i}",
                pos=(HOLE_X, (y0 + y1) / 2, z),
                size=(width, y1 - y0 + thick, thick),
                color=CABLE,
            )
        )
    for i in range(len(heights) - 1):
        low, high = sorted((heights[i], heights[i + 1]))
        out.append(
            Box(
                name=f"cable_riser_{i}",
                pos=(HOLE_X, edges[i + 1], (low + high) / 2),
                size=(width, thick, high - low + thick),
                color=CABLE,
            )
        )
    return out


def _objects() -> list[Box]:
    length = STUD_X[1] - STUD_X[0]
    out = []
    for i, y in enumerate(STUD_Y):
        out.append(
            Box(
                name=f"stud_{i}",
                pos=((STUD_X[0] + STUD_X[1]) / 2, y, TOP + STUD_DEPTH / 2),
                size=(length, STUD_WIDTH, STUD_DEPTH),
                color=WOOD,
            )
        )
        for side, sign in (("r", -1), ("l", 1)):  # the hole, drawn on both side faces
            out.append(
                Box(
                    name=f"hole_{i}_{side}",
                    pos=(HOLE_X, y + sign * (STUD_WIDTH / 2 + 0.0003), STUD_TOP - HOLE_DEPTH[i]),
                    size=(2 * HOLE_RADIUS, 0.0006, 2 * HOLE_RADIUS),
                    color=HOLE_DARK,
                )
            )
    out += _cable_boxes()
    for i, y in enumerate(RACK_Y):
        out.append(
            Box(
                name=f"pedestal_{i}",
                pos=(RACK_X, y, TOP + PEDESTAL_SIZE[2] / 2),
                size=PEDESTAL_SIZE,
                color=STAND,
            )
        )
    for name, y in zip(PLATES, RACK_Y, strict=True):
        out.append(
            Box(
                name=name,
                pos=(RACK_X, y, TOP + PEDESTAL_SIZE[2] + PLATE_SIZE[2] / 2),
                size=PLATE_SIZE,
                color=STEEL,
                mass=PLATE_MASS,
            )
        )
    return out


OBJECTS = _objects()


# --- scoring (pure NumPy) ---------------------------------------------------------------------------


class _State(Protocol):
    @property
    def pos(self) -> np.ndarray: ...
    @property
    def quat_xyzw(self) -> np.ndarray: ...
    @property
    def lin_vel(self) -> np.ndarray: ...
    @property
    def low(self) -> np.ndarray: ...


def rotation(quat_xyzw: Any) -> np.ndarray:
    """The rotation matrix of an (x, y, z, w) quaternion."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


@dataclass(frozen=True)
class PlateView:
    """What scoring needs from a plate's state (its pose from the box centre and rotation)."""

    centre: np.ndarray
    rot: np.ndarray
    underside: float
    tilt: float  # rad, its face normal from vertical
    heading_off: float  # rad, its long side from the studs' direction (x), either way round
    speed: float

    @staticmethod
    def of(state: _State) -> "PlateView":
        rot = rotation(state.quat_xyzw)
        long_side = rot[:, 0]
        off = abs(math.atan2(float(long_side[1]), float(long_side[0]))) % math.pi
        return PlateView(
            centre=np.asarray(state.pos, dtype=float),
            rot=rot,
            underside=float(state.low[2]),
            tilt=math.acos(min(1.0, abs(float(rot[2, 2])))),
            heading_off=min(off, math.pi - off),
            speed=float(np.linalg.norm(state.lin_vel)),
        )

    def distance(self, point: Any) -> float:
        """From a world point to the nearest point of the plate (m)."""
        local = self.rot.T @ (np.asarray(point, dtype=float) - self.centre)
        outside = np.maximum(np.abs(local) - np.asarray(PLATE_SIZE) / 2, 0.0)
        return float(np.linalg.norm(outside))


def protects(plate: PlateView, i: int) -> bool:
    """The plate lies flat on stud i's up face, along it, across the face and over its hole."""
    return (
        plate.tilt <= FLAT_TILT
        and abs(plate.underside - STUD_TOP) <= ON_FACE
        and plate.heading_off <= ALONG
        and abs(float(plate.centre[1]) - STUD_Y[i]) <= ACROSS
        and abs(float(plate.centre[0]) - HOLE_X) <= OVER_HOLE
    )


def rests_on(plate: PlateView, i: int) -> bool:
    """The plate lies on stud i's up face, anywhere and turned any way (its centre over the face,
    its underside on it)."""
    return (
        abs(float(plate.centre[1]) - STUD_Y[i]) <= STUD_WIDTH / 2
        and STUD_X[0] <= float(plate.centre[0]) <= STUD_X[1]
        and abs(plate.underside - STUD_TOP) <= RESTS_ON
    )


@dataclass(frozen=True)
class Gripper:
    """One arm's tool point (world, m: midway between the finger pads, 13 mm above their tips), its
    approach direction (unit, from the wrist toward the pad tips) and how far apart its fingers are."""

    tcp: np.ndarray
    approach: np.ndarray
    opening: float

    def reach(self, samples: int = 60) -> np.ndarray:
        """Points along the gripper from its origin (behind the TCP) to its pad tips."""
        s = np.linspace(-GRIPPER_BACK, PAD_TIP, samples)
        out: np.ndarray = self.tcp + s[:, None] * self.approach
        return out


def let_go(plate: PlateView, grippers: Sequence[Gripper] | None) -> bool:
    """No part of any gripper, from its origin to its pad tips, within CLEAR of the plate. Unknown
    grippers count as let go (the score then stops short of full credit)."""
    return grippers is None or all(
        min(plate.distance(p) for p in g.reach()) >= CLEAR for g in grippers
    )


@dataclass(frozen=True)
class Grade:
    protected: tuple[int, ...]  # studs that need a plate and have one, still and let go
    wrong: int  # plates resting on studs that don't need one
    done: bool  # every stud that needs one is protected, nothing wrong, the arms read
    score: float  # before the 1 s hold (see score())


def grade(plates: Sequence[PlateView], grippers: Sequence[Gripper] | None) -> Grade:
    def counts(p: PlateView, i: int) -> bool:
        return protects(p, i) and p.speed < STILL and let_go(p, grippers)

    protected = tuple(i for i in VIOLATING if any(counts(p, i) for p in plates))
    wrong = sum(1 for p in plates if any(rests_on(p, i) for i in COMPLIANT))
    done = len(protected) == len(VIOLATING) and wrong == 0 and grippers is not None
    return Grade(protected, wrong, done, max(0.0, STEP * (len(protected) - wrong)))


# --- the robot's state and the 1 s hold -------------------------------------------------------------

_warned: set[str] = set()
_held_s = 0.0  # seconds in a row with everything done
_clock: float | None = None  # the embodiment's episode clock at the last step
_last: tuple[object, Grade, float] | None = None  # the step's state dict (kept alive), grade, score
_ARMS = (("left_robot", "left"), ("right_robot", "right"))


def _warn_once(key: str, message: str) -> None:
    if key not in _warned:
        _warned.add(key)
        warnings.warn(f"nail_plates_bored_studs: {message}", RuntimeWarning, stacklevel=3)


def _calling_embodiment() -> Any:
    """The embodiment whose step is calling us, found by what it has (an episode clock, a task, a
    reset), not by method names, so a renamed private method upstream doesn't break it."""
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


def _numpy(values: Any) -> np.ndarray:
    for step in ("detach", "cpu", "numpy"):
        if hasattr(values, step):
            values = getattr(values, step)()
    return np.asarray(values, dtype=float)


def grippers_of(embodiment: Any) -> list[Gripper]:
    """Both arms' tool points, approach directions and finger openings, from the embodiment's joint
    positions and the framework's kinematics (the approach is the gripper frame's z axis)."""
    ik = importlib.import_module("yam_arena.ik")
    station = importlib.import_module("yam_arena.station")
    out = []
    for name, side in _ARMS:
        arm_ids, finger_ids = embodiment._ids[name]
        joints = embodiment._base.scene[name].data.joint_pos
        q = _numpy(getattr(joints, "torch", joints)[0])
        tcp, rot = ik.tcp_pose(q[list(arm_ids)], station.BASES[side])
        approach = np.asarray(rot, dtype=float)[:, 2]
        opening = float(-2.0 * q[list(finger_ids)].mean())
        out.append(Gripper(np.asarray(tcp, dtype=float), approach, opening))
    return out


def _robot() -> tuple[Any, list[Gripper] | None]:
    """The calling embodiment and its grippers, found afresh every step (a new trial or process gets
    its own), or (None, None) with a warning, once, if the arms can't be read this step."""
    try:
        embodiment = _calling_embodiment()
    except Exception as error:  # an odd caller on the stack
        _warn_once("find", f"could not look for the embodiment ({type(error).__name__}: {error})")
        embodiment = None
    if embodiment is None:
        _warn_once("find", "no embodiment found; letting go can't be checked, scores stop at 0.75")
        return None, None
    try:
        return embodiment, grippers_of(embodiment)
    except Exception as error:  # an unexpected embodiment: score the plates, short of full credit
        _warn_once(
            "robot", f"can't read the arms ({type(error).__name__}: {error}); scores stop at 0.75"
        )
        return embodiment, None


def _step_seconds(embodiment: Any) -> float:
    """The control step (s), from the embodiment; 20 Hz if it doesn't say."""
    try:
        dt = float(embodiment._base.step_dt)
    except (AttributeError, TypeError, ValueError):
        return DEFAULT_DT
    return dt if dt > 0 else DEFAULT_DT


def _update(objects: Mapping[str, Any]) -> tuple[Grade, float]:
    """Grade once per control step (success() and score() share the step's state dict)."""
    global _last, _held_s, _clock
    if _last is not None and _last[0] is objects:
        return _last[1], _last[2]
    plates = [PlateView.of(objects[name]) for name in PLATES]
    embodiment, grippers = _robot()
    result = grade(plates, grippers)
    clock = None if embodiment is None else getattr(embodiment, "_t", None)
    if clock is not None and _clock is not None and float(clock) < _clock:
        _held_s = 0.0  # the episode clock went back: a new trial
    _clock = None if clock is None else float(clock)
    _held_s = _held_s + _step_seconds(embodiment) if result.done else 0.0
    value = 1.0 if result.done and _held_s >= HOLD_S - 1e-9 else min(result.score, 3 * STEP)
    _last = (objects, result, value)
    return result, value


# --- what the embodiment calls ----------------------------------------------------------------------


def score(objects: Mapping[str, Any]) -> float:
    """0.25 per stud that needs a plate and has one (flat, still and let go), less 0.25 per plate on a
    stud that doesn't need one, at least 0; 1.0 once every stud that needs one is plated, nothing is
    wrong, and that has held for 1 s."""
    return _update(objects)[1]


def success(objects: Mapping[str, Any]) -> bool:
    """Every stud that needs a plate has one, none is on a wrong stud, and the grippers are clear.
    The embodiment ends the trial once this has held for 1 s."""
    return _update(objects)[0].done


# --- the scripted demo ------------------------------------------------------------------------------
# Each gripper closes across a plate's 38 mm width (yaw 0: the fingers close along y), its pad tips just
# below the plate, so the pads clear the narrow stand. It sets the plate down with the pad tips above
# the stud's face, so the fingers never straddle the stud, opens 2 mm up (the plate drops flat onto the
# face), and rises well clear of it. The fourth plate stays on its stand.

SIDES = ("left", "right")
TIPS_UNDER_PLATE = 0.0015  # the pad tips this far below a held plate's underside
DROP = 0.002  # let go with the plate's underside this far above the face
CLEARANCE = 0.015  # carry the plate's underside this far over the stud tops
RISE = 0.080  # after letting go the TCP rises this far over the face, clear of the plate
GRIP_YAW = 0.0
# Which stand each arm takes, in order, and the stud it plates.
JOBS = {"left": ((2, 3),), "right": ((0, 0), (1, 1))}
PARK = {"left": (0.20, 0.27, TOP + 0.17), "right": (0.20, -0.27, TOP + 0.17)}
SPEED = 1.0


def _tcp_over(underside: float) -> float:
    """The TCP height that holds a plate's underside at ``underside``."""
    return underside - TIPS_UNDER_PLATE + PAD_TIP


def pick_tcp(rack: int) -> tuple[float, float, float]:
    return (RACK_X, RACK_Y[rack], _tcp_over(TOP + PEDESTAL_SIZE[2]))


def place_tcp(stud: int) -> tuple[float, float, float]:
    return (HOLE_X, STUD_Y[stud], _tcp_over(STUD_TOP + DROP))


TRAVEL_Z = _tcp_over(STUD_TOP + CLEARANCE)
RISE_Z = STUD_TOP + RISE


def _job(rack: int, stud: int) -> list[Waypoint]:
    px, py, pz = pick_tcp(rack)
    qx, qy, qz = place_tcp(stud)
    k = 1.0 / SPEED
    return [
        Waypoint(1.5 * k, (px, py, TRAVEL_Z), FINGER_OPEN, "over the plate", GRIP_YAW),
        Waypoint(1.2 * k, (px, py, pz), FINGER_OPEN, "down to the plate", GRIP_YAW),
        Waypoint(0.6 * k, (px, py, pz), FINGER_CLOSED, "close on the plate", GRIP_YAW),
        Waypoint(1.0 * k, (px, py, TRAVEL_Z), FINGER_CLOSED, "lift the plate", GRIP_YAW),
        Waypoint(1.8 * k, (qx, qy, TRAVEL_Z), FINGER_CLOSED, "carry it over the hole", GRIP_YAW),
        Waypoint(1.0 * k, (qx, qy, qz), FINGER_CLOSED, "lower it to the face", GRIP_YAW),
        Waypoint(0.6 * k, (qx, qy, qz), FINGER_OPEN, "let go", GRIP_YAW),
        Waypoint(1.0 * k, (qx, qy, RISE_Z), FINGER_OPEN, "rise clear", GRIP_YAW),
    ]


def demo() -> dict[str, list[Waypoint]]:
    """Both arms at once: the left plates stud 3; the right plates stud 0, then stud 1."""
    plans: dict[str, list[Waypoint]] = {}
    for side in SIDES:
        steps: list[Waypoint] = []
        for rack, stud in JOBS[side]:
            steps += _job(rack, stud)
        steps.append(Waypoint(1.5 / SPEED, PARK[side], FINGER_OPEN, "park", GRIP_YAW))
        plans[side] = steps
    return plans
