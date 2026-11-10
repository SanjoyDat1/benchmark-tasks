"""On the station table: a cordless drill (a pistol-grip drill with its battery fitted, 0.9 kg) lies on its side
right of centre, turned at an angle: its chuck points back toward the robot's right arm and its battery away from the
robot. Across the table a coworker in a navy sleeve reaches toward the robot and holds out a gloved right hand (bright
yellow work glove), palm up, fingers pointing at the robot, thumb to the robot's left, resting on the bench just left
of centre. They are waiting to be handed the drill.

A drill is handed over grip first: its handle goes into the receiver's palm, running across the palm from the heel
of the hand to the gap between thumb and fingers, with the motor housing over the thumb and the chuck pointing out
along the fingers, away from the receiver's body. The giver uses both hands, one on the battery pack and one on the
motor housing, so the drill stays steady and the handle stays free; never points the chuck at the receiver; moves
slowly near their hand without bumping it; and lets go only once the hand carries the drill.
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
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from types import FrameType
from typing import Any, Protocol

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP: float = TABLE_TOP[SCENE]

# --- the drill (world frame: x away from the arms, y to the left, z up; metres) ------------------------------------
# Arena's library drill (the YCB power-drill scan). Lying flat in its native pose, in its own frame: chuck toward -x,
# motor housing (barrel) toward +y, battery toward -y, the handle in between; -z faces the table.
DRILL = "drill"
DRILL_ASSET = "cordless_drill_ycb_robolab"
DRILL_EXTENT = 0.1874  # its widest horizontal side (the handle direction): scale 1.0
# kg: the YCB power drill's own mass (Calli et al., IEEE RAM 2015). Held by the battery alone, 9 cm from its centre of
# mass, this much weight twisted in one YAM gripper (measured); held by the battery and the motor housing, with the
# centre of mass between the two grips, each arm carries about half.
DRILL_MASS = 0.895
DRILL_HALF = (0.0921, 0.0937, 0.02855)  # half its bounding box, in its own frame
REST_HEIGHT = 0.0286  # its origin (the box centre) above a flat surface it lies on
# Right of centre, where both arms reach both grips (official IK; the left arm's reach ends 2 cm farther right), its
# parts more than 3 cm clear of the coworker's little finger.
DRILL_START_XY = (0.20, -0.175)
# Turned 45 degrees from how it goes on the hand: chuck toward the robot's right arm (-x, -y), battery away from the
# robot and to the right. (Turned 90 degrees, the left arm's wrist would run out of travel in the turn: holding the
# motor housing, its last joint turns with the drill and with the arm's swing from the drill to the hand.)
DRILL_START_YAW = math.pi / 4
# Points and parts in the drill's own frame (measured from the asset's mesh).
GRIP_MID = (0.039, -0.016, -0.002)  # the middle of the handle
CHUCK_TIP = (-0.092, 0.068, 0.0)
CHUCK_AXIS = (-1.0, 0.0, 0.0)
HANDLE_AXIS = (0.0, 1.0, 0.0)  # along the handle, from the battery to the motor housing
PARTS = {  # (low, high) corners of each part's box
    "barrel": ((-0.0921, 0.024, -0.0287), (0.0921, 0.0937, 0.0239)),
    "handle": ((-0.007, -0.0559, -0.0194), (0.0628, 0.024, 0.0151)),
    "battery": ((-0.0372, -0.0937, -0.0283), (0.069, -0.056, 0.0287)),
}

# --- the coworker -----------------------------------------------------------------------------------------------
PALM_TOP = TOP + 0.040
# Where the handle's middle belongs: over the palm, between its heel and the thumb web. Just left of centre, where
# both arms reach their grips with the drill on the hand (official IK; the right arm's reach ends 1.5 cm farther
# left).
HAND_GRIP = (0.360, 0.065)
HAND_POINT = (HAND_GRIP[0], HAND_GRIP[1], PALM_TOP)
BODY_XY = (1.08, -0.170)  # the coworker's body: a vertical line beyond the far edge of the table
BODY_Z = (0.0, 1.75)
GLOVE_RGB = (0.85, 0.90, 0.15)  # hi-vis work glove
SLEEVE_RGB = (0.10, 0.14, 0.30)  # navy
KNUCKLES_X = 0.310
FINGERS = (  # thumb side first: (y from, y to, length toward the robot)
    ("index", 0.053, 0.073, 0.080),
    ("middle", 0.031, 0.051, 0.088),
    ("ring", 0.009, 0.029, 0.083),
    ("little", -0.013, 0.007, 0.066),
)
FINGER_TOP = PALM_TOP - 0.004
FINGER_THICKNESS = 0.018


def _static(
    name: str,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    rgb: tuple[float, float, float],
) -> Box:
    return Box(name, pos=center, size=size, color=rgb)


HAND = [
    _static("palm", (0.360, 0.030, PALM_TOP - 0.020), (0.100, 0.090, 0.040), GLOVE_RGB),
    *[
        _static(
            f"{name}_finger",
            (KNUCKLES_X - length / 2, (y0 + y1) / 2, FINGER_TOP - FINGER_THICKNESS / 2),
            (length, y1 - y0, FINGER_THICKNESS),
            GLOVE_RGB,
        )
        for name, y0, y1, length in FINGERS
    ],
    _static("thumb", (0.3475, 0.149, PALM_TOP - 0.011), (0.095, 0.032, 0.022), GLOVE_RGB),
    _static("thumb_web", (0.390, 0.104, TOP + 0.0175), (0.040, 0.058, 0.015), GLOVE_RGB),
    _static("glove_cuff", (0.430, 0.030, TOP + 0.018), (0.040, 0.070, 0.036), GLOVE_RGB),
    _static("forearm", (0.680, 0.030, TOP + 0.0325), (0.460, 0.080, 0.065), SLEEVE_RGB),
]
ARM_PARTS = ("glove_cuff", "forearm")  # the drill must not land on these
_HAND_BOXES = {
    box.name: (np.asarray(box.pos, dtype=float), np.asarray(box.size, dtype=float) / 2)
    for box in HAND
}

DRILL_OBJECT = Asset(
    DRILL, DRILL_ASSET, xy=DRILL_START_XY, extent=DRILL_EXTENT, mass=DRILL_MASS, yaw=DRILL_START_YAW
)
OBJECTS = [*HAND, DRILL_OBJECT]

# --- the rules ------------------------------------------------------------------------------------------------------
LIFT_HEIGHT = 0.020  # "lifted": the drill's lowest point at least this far above the table
# "Carried to the hand": the drill's box came within this distance of HAND_POINT. (Lifting the drill and turning it
# in place where it lay brings it no closer than 11.2 cm.)
CARRIED_NEAR = 0.07
# The speed limit holds while the drill's box is within this distance of HAND_POINT.
SLOW_ZONE = 0.15
ON_HAND_HEIGHT = 0.010  # resting on the hand: lowest point within this of the palm top...
ON_HAND_GRIP = 0.060  # ...and the handle's middle within this (horizontally) of HAND_GRIP
MAX_TILT = math.radians(10.0)
GRIP_TOLERANCE = 0.020  # handle first: the handle's middle within this of HAND_GRIP...
# ...the handle along +y and the chuck along -x, each within this.
AXIS_TOLERANCE = math.radians(20.0)
RELEASE_GAP = 0.070  # handed over: every gripper's TCP at least this far from the drill's box
# The chuck "aimed" at the coworker: within this of the direction to their body.
AIM_CONE = math.radians(30.0)
SLOW_SPEED = 0.25  # m/s near the hand (the 250 mm/s "reduced speed" for robots working near people)
RUN_STEPS = 2  # aiming and speeding count when they last this many control steps in a row
DROP = 0.020  # a fall of this much while nothing holds the drill is a drop
# No point of the drill is farther than this from its centre of mass or from its box centre (its box corners reach
# 15.5 cm from the scan's uniform-density centroid, at (0.015, 0.014, -0.002) in its frame), so the speed bound holds
# whichever of the two the simulator reports the velocity of.
REACH = 0.155
# A gripper holds the drill when its TCP is within this of the drill's own box (in the drill's frame, so a turned
# drill's larger world-aligned box doesn't count; the pads end 13 mm below the TCP, so they reach into its height)...
HELD_GAP = 0.010
# ...with its fingers this far apart (closed on the drill, not empty or wide open; the motor housing is 7 cm across at
# its widest, so a grip around it stays inside the range)...
HELD_OPENING = (0.025, 0.085)
# ...and since the last step the point of the drill where the TCP was is still at the TCP, within this: it moved
# with the gripper (a falling or slipping drill doesn't).
HELD_SLIP = 0.005
SETTLE_STEPS = (3, 5)  # "still" at the end: at rest in at least 3 of the last 5 control steps
FALLBACK_DROP_STEPS = 3  # without the robot's state: a fall of DROP within this many steps
# The finger pads (official YAM gripper): their tips end PAD_TIP past the TCP along the gripper's approach axis, and
# they are about PAD_WIDTH across; PAD_LENGTH of each pad, up from its tip, is what can touch the hand.
PAD_TIP = 0.013
PAD_LENGTH = 0.030
PAD_WIDTH = 0.027
# Holding it with a point of the pads' centre line (from the TCP to the pad tips) within this of the handle's box is
# holding it by the handle: the fingers close on the handle however deep or shallow the grip.
HANDLE_MARGIN = 0.005
# A TCP or finger pad this close to any part of the coworker's hand, glove cuff or sleeve touches them.
TOUCH_GAP = 0.003
# Both hands: while the drill is in the air (its lowest point AIR_HEIGHT or more above the table and not lying on the
# hand) both grippers must hold it. In the air with fewer than two holding it for this many control steps in all
# (0.5 s at 20 Hz) caps the score: a moment's slip or a regrasp is forgiven, carrying it one-handed is not, in one
# stretch or in many short ones, and not carried low either.
AIR_HEIGHT = 0.005
HANDS = 2
ONE_HAND_STEPS = 10
AIM_CAP, SPEED_CAP, BUMP_CAP, HANDLE_CAP, ONE_HAND_CAP, TOUCH_CAP = 0.25, 0.5, 0.5, 0.5, 0.5, 0.5
NO_ROBOT_CAP = 0.75
# A full score counts once it has held for 1 s (20 control steps); until then it reads 0.75, so a trial that only
# gets there in its last second (with no time to hold it) doesn't score 1.0.
HOLD_STEPS = 20
SCORE_OVER = "end"

# --- Arena's library drill loads from NVIDIA's production bucket ---------------------------------------------------
# Isaac Lab-Arena release/0.3.1 still points its object library at NVIDIA's staging S3 bucket (a TODO in its
# isaaclab_arena/assets/nucleus.py), and that bucket no longer has the library (HTTP 404, checked 2026-11-04); the
# production bucket has the same file. Right after Arena's object library is imported, this rewrites the drill's
# URL, and only the drill's, from the staging host to the production host. It does nothing when the URL is not the
# public staging one (a fixed Arena, a local mirror) and nothing on a machine without Arena.
_LIBRARY_MODULE = "isaaclab_arena.assets.object_library"
_DRILL_CLASS = "CordlessDrillYcbRobolab"
_STAGING = "https://omniverse-content-staging.s3-us-west-2.amazonaws.com/"
_PRODUCTION = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"


def fix_drill_url(library: object) -> bool:
    """Point the library's drill at the production bucket if it names the staging one; True if it changed."""
    drill = getattr(library, _DRILL_CLASS, None)
    path = getattr(drill, "usd_path", None)
    if drill is None or not isinstance(path, str) or not path.startswith(_STAGING):
        return False
    drill.usd_path = _PRODUCTION + path[len(_STAGING) :]
    return True


class _DrillUrlFix(importlib.abc.MetaPathFinder):
    """Waits for Arena's object library to be imported, lets it load normally, then applies fix_drill_url. It
    answers for that one module name only, and removes itself once the library has loaded."""

    marker = "hand_drill_handle_first.drill_url_fix"

    def __init__(self) -> None:
        self._looking = False

    def find_spec(
        self, fullname: str, path: Sequence[str] | None, target: types.ModuleType | None = None
    ) -> importlib.machinery.ModuleSpec | None:
        if fullname != _LIBRARY_MODULE or self._looking:
            return None
        self._looking = True  # find_spec below asks every finder again, this one included
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            self._looking = False
        if spec is not None and spec.loader is not None:
            spec.loader = _FixAfterLoad(spec.loader, self)
        return spec

    def disarm(self) -> None:
        if self in sys.meta_path:
            sys.meta_path.remove(self)


class _FixAfterLoad(importlib.abc.Loader):
    """The library's own loader, plus fix_drill_url once the module has run."""

    def __init__(self, inner: importlib.abc.Loader, watcher: _DrillUrlFix) -> None:
        self._inner = inner
        self._watcher = watcher

    def create_module(self, spec: importlib.machinery.ModuleSpec) -> types.ModuleType | None:
        return self._inner.create_module(spec)

    def exec_module(self, module: types.ModuleType) -> None:
        self._inner.exec_module(module)
        fix_drill_url(module)
        self._watcher.disarm()

    def __getattr__(self, name: str) -> Any:  # get_source and friends, for tracebacks
        if name.startswith("__") or name in ("_inner", "_watcher"):  # a copy made without __init__
            raise AttributeError(name)
        return getattr(self._inner, name)


def install_drill_url_fix() -> None:
    """Fix the drill now if the library is already imported; otherwise wait for its import (once)."""
    library = sys.modules.get(_LIBRARY_MODULE)
    if library is not None:
        fix_drill_url(library)
        return
    if not any(getattr(finder, "marker", None) == _DrillUrlFix.marker for finder in sys.meta_path):
        sys.meta_path.insert(0, _DrillUrlFix())


install_drill_url_fix()


# --- geometry ---------------------------------------------------------------------------------------------------------


def rotation(quat_xyzw: Any) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def yaw_quat(yaw: float) -> np.ndarray:
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def point_box_distance(point: Any, low: Any, high: Any) -> float:
    p, lo, hi = (np.asarray(v, dtype=float) for v in (point, low, high))
    return float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0)))


def boxes_overlap(
    center: np.ndarray,
    axes: np.ndarray,
    half: np.ndarray,
    box_center: np.ndarray,
    box_half: np.ndarray,
    touch: float = 0.001,
) -> bool:
    """An oriented box (centre, axes as columns, half sizes) against a world-aligned one, by separating axes. Boxes
    that only touch, or overlap by less than `touch`, don't count."""
    offset = center - box_center
    world = np.eye(3)
    candidates = [world[:, i] for i in range(3)] + [axes[:, j] for j in range(3)]
    candidates += [np.cross(world[:, i], axes[:, j]) for i in range(3) for j in range(3)]
    for axis in candidates:
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9:
            continue
        axis = axis / norm
        reach = float(
            np.sum(box_half * np.abs(world.T @ axis)) + np.sum(half * np.abs(axes.T @ axis))
        )
        if abs(float(offset @ axis)) >= reach - touch:
            return False
    return True


def _flat_angle(direction: np.ndarray, target: tuple[float, float]) -> float:
    """Angle between a direction's horizontal part and a horizontal unit target (pi if it has none)."""
    flat = np.asarray(direction[:2], dtype=float)
    norm = float(np.linalg.norm(flat))
    if norm < 1e-9:
        return math.pi
    return math.acos(max(-1.0, min(1.0, float(flat @ np.asarray(target)) / norm)))


class _State(Protocol):
    pos: Any
    quat_xyzw: Any
    lin_vel: Any
    ang_vel: Any
    low: Any
    high: Any

    def resting(self, max_speed: float = ..., max_spin: float = ...) -> bool: ...


@dataclass(frozen=True)
class DrillView:
    """The drill at one control step, with the points the rules use."""

    pos: np.ndarray
    rot: np.ndarray
    low: np.ndarray
    high: np.ndarray
    speed: float
    resting: bool

    @staticmethod
    def of(state: _State) -> "DrillView":
        return DrillView(
            pos=np.asarray(state.pos, dtype=float),
            rot=rotation(state.quat_xyzw),
            low=np.asarray(state.low, dtype=float),
            high=np.asarray(state.high, dtype=float),
            # The fastest any point of it can be moving: its centre's speed plus its spin times its reach.
            speed=float(
                np.linalg.norm(np.asarray(state.lin_vel, dtype=float))
                + np.linalg.norm(np.asarray(state.ang_vel, dtype=float)) * REACH
            ),
            resting=bool(state.resting()),
        )

    def point(self, local: Sequence[float]) -> np.ndarray:
        world: np.ndarray = self.pos + self.rot @ np.asarray(local, dtype=float)
        return world

    def direction(self, local: Sequence[float]) -> np.ndarray:
        world: np.ndarray = self.rot @ np.asarray(local, dtype=float)
        return world

    @property
    def tilt(self) -> float:
        """How far it is from lying flat on either side (rad)."""
        return math.acos(min(1.0, abs(float(self.rot[2, 2]))))

    @property
    def height(self) -> float:
        """Its lowest point above the table."""
        return float(self.low[2]) - TOP

    def hand_distance(self) -> float:
        return point_box_distance(HAND_POINT, self.low, self.high)

    def aim_angle(self) -> float:
        """The smallest angle between the chuck's direction and the direction from the chuck tip to any point of the
        coworker's body (a vertical line from the floor to the top of their head), feet to face."""
        tip = self.point(CHUCK_TIP)
        chuck = self.direction(CHUCK_AXIS)
        across = np.asarray(BODY_XY, dtype=float) - tip[:2]
        distance = float(np.linalg.norm(across))
        toward = float(chuck[:2] @ across)
        # Along the line, the angle is smallest at one end or where d/dz of the cosine is zero: dz = u_z d^2 / (u . h).
        rises = [BODY_Z[0] - float(tip[2]), BODY_Z[1] - float(tip[2])]
        if toward > 1e-12:
            rises.append(min(max(float(chuck[2]) * distance**2 / toward, rises[0]), rises[1]))
        best = max(
            (toward + float(chuck[2]) * dz) / max(math.hypot(distance, dz), 1e-9) for dz in rises
        )
        return math.acos(max(-1.0, min(1.0, best)))

    def touches_arm(self) -> bool:
        """A part of the drill overlapping the coworker's glove cuff or forearm."""
        for low, high in PARTS.values():
            lo, hi = np.asarray(low), np.asarray(high)
            center = self.point((lo + hi) / 2)
            for name in ARM_PARTS:
                box_center, box_half = _HAND_BOXES[name]
                if boxes_overlap(center, self.rot, (hi - lo) / 2, box_center, box_half):
                    return True
        return False


@dataclass(frozen=True)
class Gripper:
    """One arm's TCP (world, m), how far apart its fingers are (m), the direction its fingers point (approach) and
    the axis they close along, both unit vectors in the world (a straight-down grip by default)."""

    tcp: np.ndarray
    opening: float
    approach: tuple[float, float, float] = (0.0, 0.0, -1.0)
    closing: tuple[float, float, float] = (0.0, 1.0, 0.0)

    def pad_line(self) -> list[np.ndarray]:
        """Points on the line between the finger pads, from the TCP to the pad tips."""
        tcp, approach = np.asarray(self.tcp, dtype=float), np.asarray(self.approach, dtype=float)
        return [tcp + s * PAD_TIP * approach for s in (0.0, 0.5, 1.0)]

    def finger_points(self) -> list[np.ndarray]:
        """The TCP and a grid of points over both finger pads: at the tips, halfway up and PAD_LENGTH up, at both
        edges and the middle of each pad."""
        tcp = np.asarray(self.tcp, dtype=float)
        approach = np.asarray(self.approach, dtype=float)
        closing = np.asarray(self.closing, dtype=float)
        across = np.cross(approach, closing)
        points = [tcp]
        for finger in (-0.5, 0.5):
            for along in (PAD_TIP, PAD_TIP - PAD_LENGTH / 2, PAD_TIP - PAD_LENGTH):
                for edge in (-0.5, 0.0, 0.5):
                    points.append(
                        tcp
                        + finger * self.opening * closing
                        + along * approach
                        + edge * PAD_WIDTH * across
                    )
        return points


def hand_gap(grippers: Sequence[Gripper]) -> float:
    """The smallest distance from any gripper's TCP or finger pads to the coworker's hand, cuff or sleeve (m)."""
    best = math.inf
    for gripper in grippers:
        for point in gripper.finger_points():
            for center, half in _HAND_BOXES.values():
                best = min(best, point_box_distance(point, center - half, center + half))
    return best


# --- one episode -------------------------------------------------------------------------------------------------------


@dataclass
class Ledger:
    """What has happened so far this episode (the path rules) and the drill's last supported height."""

    steps: int = 0
    lifted: bool = False
    near: bool = False  # came near the hand after being lifted
    aimed: bool = False
    fast: bool = False
    bumped: bool = False
    by_handle: bool = False  # a gripper held it by the handle, the part the receiver needs free
    one_handed: bool = False  # it spent ONE_HAND_STEPS in the air held by fewer than both grippers
    touched_hand: bool = False  # a TCP or finger pad came within TOUCH_GAP of the coworker's hand
    dropped: bool = False
    aim_run: int = 0
    fast_run: int = 0
    holders: int = 0  # how many grippers held it at the last step
    one_hand_steps: int = 0  # steps so far in the air with fewer than HANDS holding it
    support_z: float | None = None  # its lowest point the last time it was held or at rest
    last_pose: tuple[np.ndarray, np.ndarray] | None = (
        None  # its position and rotation at the last step
    )
    last_tcps: list[np.ndarray] | None = None  # the TCPs at the last step
    recent_rest: list[bool] = field(default_factory=list)  # at rest, over the last few steps
    # Without the robot's state: the last few heights.
    recent_low: list[float] = field(default_factory=list)

    def observe(self, drill: DrillView, grippers: Sequence[Gripper] | None) -> None:
        self.steps += 1
        lifted_now = drill.height >= LIFT_HEIGHT
        distance = drill.hand_distance()
        self.lifted = self.lifted or lifted_now
        self.near = self.near or (self.lifted and distance <= CARRIED_NEAR)
        self.aim_run = self.aim_run + 1 if lifted_now and drill.aim_angle() <= AIM_CONE else 0
        self.aimed = self.aimed or self.aim_run >= RUN_STEPS
        in_slow_zone = distance <= SLOW_ZONE
        self.fast_run = self.fast_run + 1 if in_slow_zone and drill.speed > SLOW_SPEED else 0
        self.fast = self.fast or self.fast_run >= RUN_STEPS
        self.bumped = self.bumped or drill.touches_arm()
        self.recent_rest = [*self.recent_rest[1 - SETTLE_STEPS[1] :], drill.resting]
        low = float(drill.low[2])
        if grippers is not None:
            holders = [
                g
                for i, g in enumerate(grippers)
                if _holding(drill, g) and self._moved_with(drill, g, i)
            ]
            self.holders = len(holders)
            holding = bool(holders)
            # Carrying it by the handle: a gripper rigidly holding it there while it moves or is held up off the
            # table and the hand (not while it lies on the hand and the fingers open around it).
            on_a_support = drill.height < LIFT_HEIGHT or over_the_hand(drill)
            carrying = not drill.resting or not on_a_support
            self.by_handle = self.by_handle or (carrying and held_by_handle(drill, holders))
            self.touched_hand = self.touched_hand or hand_gap(grippers) <= TOUCH_GAP
            # In the air (even carried low) it needs both hands; on the table or the hand they may close or open one
            # at a time. Lying still with no gripper on it (set down somewhere else), it isn't being carried at all.
            in_air = drill.height >= AIR_HEIGHT and not over_the_hand(drill)
            set_down = drill.resting and not holders
            short_handed = in_air and not set_down and len(holders) < HANDS
            if short_handed:
                self.one_hand_steps += 1
            self.one_handed = self.one_handed or self.one_hand_steps >= ONE_HAND_STEPS
            if holding or drill.resting:
                self.support_z = low
            elif self.support_z is not None and low <= self.support_z - DROP:
                self.dropped = True
            self.last_tcps = [np.asarray(g.tcp, dtype=float) for g in grippers]
        else:
            self.recent_low = [*self.recent_low[-FALLBACK_DROP_STEPS:], low]
            if len(self.recent_low) > 1 and max(self.recent_low[:-1]) - low >= DROP:
                self.dropped = True
        self.last_pose = (drill.pos, drill.rot)

    def _moved_with(self, drill: DrillView, gripper: Gripper, i: int) -> bool:
        """The point of the drill where gripper i's TCP was at the last step is still at the TCP (rigidly held)."""
        if self.last_pose is None or self.last_tcps is None or i >= len(self.last_tcps):
            return True
        last_pos, last_rot = self.last_pose
        grasped = last_rot.T @ (self.last_tcps[i] - last_pos)  # in the drill's own frame
        return float(np.linalg.norm(drill.point(grasped) - gripper.tcp)) <= HELD_SLIP

    @property
    def settled(self) -> bool:
        return sum(self.recent_rest) >= SETTLE_STEPS[0]


def _holding(drill: DrillView, gripper: Gripper) -> bool:
    """The gripper's TCP at the drill (in its own frame) with its fingers closed on it. Geometry only: the task
    file sees no finger contact (the embodiment builds no contact sensors), so _moved_with checks the rest."""
    local = drill.rot.T @ (np.asarray(gripper.tcp, dtype=float) - drill.pos)
    half = np.asarray(DRILL_HALF)
    return (
        point_box_distance(local, -half, half) <= HELD_GAP
        and HELD_OPENING[0] <= gripper.opening <= HELD_OPENING[1]
    )


def held(drill: DrillView, grippers: Sequence[Gripper]) -> bool:
    return any(_holding(drill, g) for g in grippers)


def held_by_handle(drill: DrillView, grippers: Sequence[Gripper]) -> bool:
    """A gripper closed on the drill with the line between its pads (TCP to pad tips) at the handle, in the drill's
    own frame, so a shallow grip on top of the handle counts as well as a deep one. Callers pass the grippers that
    are rigidly holding it."""
    low, high = (np.asarray(corner) for corner in PARTS["handle"])
    for g in grippers:
        if _holding(drill, g):
            for point in g.pad_line():
                local = drill.rot.T @ (point - drill.pos)
                if point_box_distance(local, low, high) <= HANDLE_MARGIN:
                    return True
    return False


@dataclass(frozen=True)
class Grade:
    carried: bool
    on_hand: bool
    handle_first: bool
    released: bool
    robot_known: bool
    aimed: bool
    fast: bool
    bumped: bool
    by_handle: bool
    one_handed: bool
    touched_hand: bool
    dropped: bool
    score: float


def over_the_hand(drill: DrillView) -> bool:
    """Down on the coworker's hand: its box's lowest corner at palm height and its handle roughly over the palm (not
    merely as high as the palm somewhere else)."""
    grip = drill.point(GRIP_MID)
    return (
        abs(float(drill.low[2]) - PALM_TOP) <= ON_HAND_HEIGHT
        and math.dist(grip[:2], HAND_GRIP) <= ON_HAND_GRIP
    )


def on_hand(drill: DrillView, settled: bool) -> bool:
    """At rest on the coworker's hand: settled (still in most of the last few steps), lying flat, and down on the hand
    with its handle roughly over the palm."""
    return settled and drill.tilt <= MAX_TILT and over_the_hand(drill)


def handle_first(drill: DrillView) -> bool:
    """The handle's middle in the palm, the handle across the palm with the motor housing over the thumb (+y), and the
    chuck pointing out along the fingers (-x), away from the coworker."""
    grip = drill.point(GRIP_MID)
    return (
        math.dist(grip[:2], HAND_GRIP) <= GRIP_TOLERANCE
        and _flat_angle(drill.direction(HANDLE_AXIS), (0.0, 1.0)) <= AXIS_TOLERANCE
        and _flat_angle(drill.direction(CHUCK_AXIS), (-1.0, 0.0)) <= AXIS_TOLERANCE
    )


def grade(ledger: Ledger, drill: DrillView, grippers: Sequence[Gripper] | None) -> Grade:
    """The score ladder (0.25 carried, 0.5 on the hand, 0.75 handle first, 1.0 handed over) and its caps."""
    carried = ledger.lifted and ledger.near
    resting_on_hand = on_hand(drill, ledger.settled)
    first = handle_first(drill)
    released = grippers is not None and all(
        point_box_distance(g.tcp, drill.low, drill.high) >= RELEASE_GAP for g in grippers
    )
    score = 0.0
    if carried:
        score = 0.25
        if resting_on_hand:
            score = 0.5
            if first:
                score = 1.0 if released else 0.75
    if ledger.aimed:
        score = min(score, AIM_CAP)
    if ledger.fast:
        score = min(score, SPEED_CAP)
    if ledger.bumped:
        score = min(score, BUMP_CAP)
    if ledger.by_handle:
        score = min(score, HANDLE_CAP)
    if ledger.one_handed:
        score = min(score, ONE_HAND_CAP)
    if ledger.touched_hand:
        score = min(score, TOUCH_CAP)
    if grippers is None:
        score = min(score, NO_ROBOT_CAP)
    if ledger.dropped:
        score = 0.0
    return Grade(
        carried=carried,
        on_hand=resting_on_hand,
        handle_first=first,
        released=released,
        robot_known=grippers is not None,
        aimed=ledger.aimed,
        fast=ledger.fast,
        bumped=ledger.bumped,
        by_handle=ledger.by_handle,
        one_handed=ledger.one_handed,
        touched_hand=ledger.touched_hand,
        dropped=ledger.dropped,
        score=score,
    )


# --- episodes and the robot's state -----------------------------------------------------------------------------------

_LEDGER = Ledger()
_GRADE: Grade | None = None
_full_steps = 0  # control steps in a row with the full score's conditions met
_last_objects: object = None  # the step's state dict, kept alive so its id can't be reused
_embodiment: Any = None
_hooked = False
_robot_ok = True
_warned: set[str] = set()
_ARMS = (("left_robot", "left"), ("right_robot", "right"))


def _warn_once(key: str, message: str) -> None:
    if key not in _warned:
        _warned.add(key)
        warnings.warn(f"hand_drill_handle_first: {message}", RuntimeWarning, stacklevel=3)


def new_episode() -> None:
    global _LEDGER, _GRADE, _last_objects, _robot_ok, _full_steps
    _LEDGER = Ledger()
    _GRADE = None
    _full_steps = 0
    _last_objects = None
    _robot_ok = True  # try reading the arms again each trial


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
        _embodiment = embodiment
        original = embodiment.reset

        def reset(*args: Any, **kwargs: Any) -> Any:
            new_episode()
            return original(*args, **kwargs)

        try:
            embodiment.reset = reset
            _hooked = True
            return
        except (AttributeError, TypeError):
            pass
    _warn_once(
        "reset",
        "could not hook the embodiment's reset; a drill resting back at its start starts a new trial",
    )


def _numpy(values: Any) -> np.ndarray:
    for step in ("detach", "cpu", "numpy"):
        if hasattr(values, step):
            values = getattr(values, step)()
    return np.asarray(values, dtype=float)


def grippers_of(embodiment: Any) -> list[Gripper]:
    """Both arms' TCPs and finger openings, from the embodiment's joint positions and the framework's kinematics."""
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
    global _robot_ok
    if _embodiment is None or not _robot_ok:
        return None
    try:
        return grippers_of(_embodiment)
    # An unexpected embodiment: score without the robot (capped) rather than abort the trial.
    except Exception as error:
        _robot_ok = False
        _warn_once(
            "robot",
            f"can't read the arms' state ({type(error).__name__}: {error}); scores are capped at {NO_ROBOT_CAP}",
        )
        return None


def _back_at_start(drill: DrillView) -> bool:
    start_yaw = DRILL_START_YAW + math.pi / 2  # the handle axis's heading at the start
    return (
        math.dist(drill.pos[:2], DRILL_START_XY) <= 0.04
        and abs(drill.height) <= 0.003
        and _flat_angle(drill.direction(HANDLE_AXIS), (math.cos(start_yaw), math.sin(start_yaw)))
        <= math.radians(10)
        and drill.resting
    )


def _update(objects: Mapping[str, Any]) -> Grade:
    """Advance the episode once per control step (success() and score() share the step's state dict)."""
    global _last_objects, _GRADE, _full_steps
    if objects is _last_objects and _GRADE is not None:
        return _GRADE
    try:
        _hook_resets()
    except Exception as error:  # an odd caller on the stack: carry on without the hook
        _warn_once("hook", f"could not look for the embodiment ({type(error).__name__}: {error})")
    drill = DrillView.of(objects[DRILL])
    if not _hooked and _LEDGER.lifted and _back_at_start(drill):
        new_episode()
    grippers = _robot()
    _LEDGER.observe(drill, grippers)
    _GRADE = grade(_LEDGER, drill, grippers)
    _full_steps = _full_steps + 1 if _GRADE.score >= 1.0 - 1e-9 else 0
    _last_objects = objects
    return _GRADE


# --- what the embodiment calls -----------------------------------------------------------------------------------------


def score(objects: Mapping[str, Any]) -> float:
    """0.25 carried to the hand, 0.5 resting on it, 0.75 handle first, 1.0 handed over (once it has held for 1 s);
    see README.md for the caps."""
    value = _update(objects).score
    return value if _full_steps >= HOLD_STEPS else min(value, 0.75)


def success(objects: Mapping[str, Any]) -> bool:
    """Handed over handle first, chuck away, released and clear, with no path rule broken. The embodiment ends the
    trial once this has held for 1 s, which is when score() reads 1.0."""
    return _update(objects).score >= 1.0 - 1e-9


# --- the scripted demo -------------------------------------------------------------------------------------------------

# Both grippers hold the drill by its body, fingers closing across it (along the drill's own y), so the handle stays
# free for the hand: the right on the front of the battery pack (flat faces, 33 mm between the pads), the left on the
# back of the motor housing (58 mm between the pads; measured on the official stack). The drill's centre of mass lies
# between the two grips, 59% of the way from the battery grip to the housing grip, so the left arm carries about 59%
# of its weight and the right 41%, and neither grip has to stop it twisting on its own.
BATTERY_GRIP = (-0.022, -0.077, -0.002)  # the right TCP in the drill's frame while holding it
HOUSING_GRIP = (0.066, 0.061, -0.004)  # the left TCP
GRIPS = {"left": HOUSING_GRIP, "right": BATTERY_GRIP}
SIDES = ("left", "right")
# Each gripper's heading minus the drill's. The fingers close along the drill's y either way; these are the headings
# each arm reaches along the whole path within its wrist's limits (official IK).
GRIP_YAW = {"left": 0.0, "right": 0.0}
HAND_YAW = 0.0  # the drill's heading on the hand: chuck along -x, motor housing toward +y
APPROACH = 0.09  # the open grippers come down onto the drill from this far above their grips
CARRY_HEIGHT = TOP + 0.14  # the drill's origin while it is carried: its lowest point 11 cm up
# The turn and the carry to the hand happen together, in steps. Between waypoints each TCP moves in a straight line,
# so a turn cuts the corner and the two TCPs come closer together than the grips are; in 7.5-degree steps, by 0.4 mm.
TURN_STEPS = 6
TURN_STEP_SECONDS = 0.75
# Command the set-down this far above the resting height: the arms sag under the drill.
SET_DOWN_ABOVE = 0.003
# Open to 80 mm around the 58 mm housing and to 64 mm around the 33 mm battery: clear, without swinging wide.
RELEASE_FINGER = {"left": -0.040, "right": -0.032}
BACK_OFF = {"left": (0.22, 0.28, TOP + 0.15), "right": (0.20, -0.15, TOP + 0.15)}
PARK = {"left": (0.20, 0.27, TOP + 0.17), "right": (0.20, -0.27, TOP + 0.17)}
# The drill doesn't end up exactly where the commands put it: as the fingers close it shifts between them (on the
# official stack the TCPs ended 10 and 16 mm along the drill's y from their grip points, and stayed there; measured),
# and the arms sag under its weight (no payload compensation). Untrimmed, it came to rest 4 mm toward the robot and
# 10 mm toward the little finger from its target, still within GRIP_TOLERANCE. Carry it and set it down that much
# farther out, the same for both arms (the drill and its two grips move as one body), then let go at the untrimmed
# grip points: with the trim it came to rest 1 mm from its target and didn't move as the fingers opened (measured).
# The trim is tuned to this simulator.
PLACE_TRIM = (0.004, 0.010)


def _rz(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def grip_tcp(side: str, origin: Sequence[float] | np.ndarray, yaw: float) -> np.ndarray:
    """Where `side`'s TCP is while it holds the drill whose origin is at `origin`, heading `yaw`."""
    tcp: np.ndarray = np.asarray(origin, dtype=float) + _rz(yaw) @ np.asarray(GRIPS[side])
    return tcp


def start_origin() -> np.ndarray:
    return np.array([*DRILL_START_XY, TOP + REST_HEIGHT])


def hand_origin() -> np.ndarray:
    """Where the drill's origin sits when it lies on the hand handle first."""
    grip = _rz(HAND_YAW) @ np.asarray(GRIP_MID)
    origin: np.ndarray = np.array(
        [HAND_GRIP[0] - grip[0], HAND_GRIP[1] - grip[1], PALM_TOP + REST_HEIGHT]
    )
    return origin


def pick_tcp(side: str) -> np.ndarray:
    return grip_tcp(side, start_origin(), DRILL_START_YAW)


def place_tcp(side: str) -> np.ndarray:
    return grip_tcp(side, hand_origin(), HAND_YAW)


def drill_path() -> list[tuple[float, np.ndarray, float, bool, str]]:
    """The drill's poses from gripping to letting go: (seconds, origin, heading, fingers closed, label)."""
    start, hand = start_origin(), hand_origin()
    trim = np.array([*PLACE_TRIM, 0.0])  # aimed past the place point by the measured offset
    lifted = np.array([start[0], start[1], CARRY_HEIGHT])
    over_hand = np.array([hand[0], hand[1], CARRY_HEIGHT]) + trim
    down = hand + trim + np.array([0.0, 0.0, SET_DOWN_ABOVE])
    turn = DRILL_START_YAW, HAND_YAW
    path = [
        (2.0, start + np.array([0.0, 0.0, APPROACH]), turn[0], False, "above the drill"),
        (1.5, start, turn[0], False, "descend"),
        (0.8, start, turn[0], True, "grip the battery and the housing"),
        (0.4, start, turn[0], True, "settle the grip"),
        (1.5, lifted, turn[0], True, "lift together"),
    ]
    for k in range(1, TURN_STEPS + 1):
        f = k / TURN_STEPS
        origin = lifted + f * (over_hand - lifted)
        heading = turn[0] + f * (turn[1] - turn[0])
        path.append((TURN_STEP_SECONDS, origin, heading, True, "turn the chuck away and carry"))
    path += [
        (0.5, over_hand, turn[1], True, "steady over the hand"),
        (2.0, down, turn[1], True, "set down slowly"),
        (1.0, down, turn[1], True, "let it settle"),
    ]
    return path


def demo() -> dict[str, list[Waypoint]]:
    """Both arms, on one clock: grip the drill by its battery pack (right) and motor housing (left), lift it together,
    turn the chuck away while carrying it over the hand, set it down slowly handle first, let go, rise clear and park.
    Each arm's TCP is the drill's pose seen from that arm's grip, so the two plans move the drill as one rigid body."""
    hand = hand_origin()
    plans: dict[str, list[Waypoint]] = {}
    for side in SIDES:
        plan = []
        for seconds, origin, heading, closed, label in drill_path():
            p = grip_tcp(side, origin, heading)
            finger = FINGER_CLOSED if closed else FINGER_OPEN
            plan.append(Waypoint(seconds, _xyz(p), finger, label, heading + GRIP_YAW[side]))
        # Let go at the untrimmed grip point, rise straight up, back off to its own side.
        yaw = HAND_YAW + GRIP_YAW[side]
        release = grip_tcp(side, hand + np.array([0.0, 0.0, SET_DOWN_ABOVE]), HAND_YAW)
        up = grip_tcp(side, np.array([hand[0], hand[1], CARRY_HEIGHT]), HAND_YAW)
        plan += [
            Waypoint(0.8, _xyz(release), RELEASE_FINGER[side], "let go", yaw),
            Waypoint(1.5, _xyz(up), RELEASE_FINGER[side], "rise clear", yaw),
            Waypoint(1.5, BACK_OFF[side], FINGER_OPEN, "back off", yaw),
            Waypoint(2.0, PARK[side], FINGER_OPEN, "park", yaw),
        ]
        plans[side] = plan
    return plans


def _xyz(point: np.ndarray) -> tuple[float, float, float]:
    return (float(point[0]), float(point[1]), float(point[2]))
