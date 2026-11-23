"""A plain-top two-speed sheet winch is bolted to a deck board on the bench. Its sheet runs straight across the bench past
the drum's side: from the load (the sail's pull, springs under the bench's right-hand edge) over an edge roller,
through a fairlead on the deck board, three clockwise wraps round the drum, and on as the tail over the left-hand
edge to a hanging 1 kg weight that tails it (on a boat one person grinds while another tails; here the robot grinds
and the weight keeps the tail taut). A red mark on the sheet reaches the edge of the drum when the sail is fully
trimmed. A horn cleat stands on a vertical board beside the winch. The lock-in winch handle stands in its holder.

Turning the handle clockwise (seen from above) drives the drum in first gear (fast, direct drive); anticlockwise in
second gear (slow, 2.76 times stronger). The drum always turns the same way and its pawls stop it running back. The
lock-in handle goes in and comes out only while the red lock lever on top of its hub is held turned; let go, the lever
springs back and locks it in. Taking it out takes two hands: one holds the lever turned, the other lifts.
"""

# No `from __future__ import annotations` here: the task loader imports this file without registering it in
# sys.modules, and dataclasses then cannot resolve string annotations.
import contextlib
import importlib
import importlib.abc
import importlib.machinery
import importlib.util
import math
import sys
import types
import warnings
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from types import FrameType
from typing import Any

import numpy as np
from yam_arena.constants import FINGER_OPEN, READY
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP["station"]
DT = 0.05  # s: one control step at 20 Hz

# =====================================================================================================================
# Parameters
# =====================================================================================================================
# --- the winch (Harken Performa 20.2PTP, plain top, two speed; maker's figures via a retailer, read 2026-11-22) ------
GEAR_FAST = 1.0  # handle turns per drum turn in first gear (direct drive)
GEAR_SLOW = 2.76  # second gear
POWER_FAST = 6.95  # line pull per unit of force at the grip, with a 10 in handle (maker's power ratios)
POWER_SLOW = 19.20
BASE_DIA = 5.375 * 0.0254  # 136.5 mm
DRUM_DIA = 2.875 * 0.0254  # 73.0 mm
WINCH_HEIGHT = 5.0625 * 0.0254  # 128.6 mm
WINCH_MASS = 4.4 * 0.4536  # 2.0 kg
# --- the handle (a Lewmar 250 mm / 10 in grey alloy locking winch handle, part 29141111, per two retailers read
# 2026-11-22; its weight is not published, its grip and lock lever are drawn red as in the task's reference photo) ---
HANDLE_LENGTH = 0.254  # socket axis to grip axis: 10 in, the length the winch maker's power ratios are for (the named
# handle is listed as 250 mm, 1.6% shorter)
HANDLE_MASS = 0.600  # est: a comparable 254 mm aluminium lock-in handle (Harken B10ASG, 600 g, maker's figure)
# The radius the power ratios imply (handle length x gear / power): 36.5 mm, the drum's radius.
LOAD_RADIUS = HANDLE_LENGTH * GEAR_FAST / POWER_FAST
# First gear is clockwise seen from above (the maker's manual, from a summary: UNVERIFIED).
FAST_IS_CLOCKWISE = True

# --- estimates (not published; to be measured on a real winch) -------------------------------------------------------
EFFICIENCY = 1.0  # the gear train's efficiency (no figure found): the handle feels the ideal torque
IDLE_TORQUE = 0.05  # N m: the gear train's own drag with no load
BACKLASH = math.radians(3.0)  # a gear engages only after the spindle has turned this far past its last reversal
WINDUP_K = 50.0  # N m/rad: the handle-to-spindle windup spring that tells which way the arm pushes (4 deg at 3.5 N m)
WINDUP_D = 0.5  # N m s/rad
DEADBAND = 0.3  # N m: a push this small either way counts as no push (the brake then holds at the larger limit)
SPINDLE_K = 2000.0  # N m/rad: the spindle's hold at its brake angle (the brake: see step_brake)
SPINDLE_D = 5.0  # N m s/rad

# --- the sail's pull (estimates, sized from what the arm holds in GPU tests) -----------------------------------------
# Two extension springs from the sheet's end to a floor anchor: a soft one always on, a stiff one picked up by a slack
# link after LOAD_S1; past full trim a stopper ball on the sheet meets the fairlead's rubber snubber (bar-tight).
LOAD_T0 = 30.0  # N on the sheet at the start (the soft spring's pull; the sail luffing)
LOAD_K1 = 200.0  # N/m: the soft spring ...
LOAD_S1 = 0.150  # ... alone for this much sheet taken in
LOAD_K2 = 1134.0  # N/m: both springs (the sail filling). Sized so the grind turns the handle a whole number of turns
# overall (first gear about 1.04 turns one way, second about 2.0 the other): the handle comes out pointing as it went in,
# the direction in which the hub pinch holds its weight best (a v1 GPU test at 1,437 N/m: +262 deg)
F_REF = 18.0  # N at the grip in second gear at full trim (above the arm's rated-torque force; see the README)
LOAD_K3 = 20000.0  # N/m past full trim: the stopper ball on the snubber
TAIL_WEIGHT = 1.0  # kg hanging on the tail (it tails the sheet as it comes in)
G = 9.81
T_TAIL = TAIL_WEIGHT * G  # N on the tail
CAPSTAN_MU = 0.22  # polyester on an aluminium drum (Practical Sailor 2023; an estimate for this drum and sheet)
WRAPS = 3  # turns of sheet on the drum


def stage_two_tension_at(s: float) -> float:
    return LOAD_T0 + LOAD_K1 * LOAD_S1 + LOAD_K2 * (s - LOAD_S1)


def handle_torque(tension: float, gear: float) -> float:
    """The torque (N m) the handle needs to turn the drum against the sheet in a gear (the tail's pull helps)."""
    return IDLE_TORQUE + max(0.0, tension - T_TAIL) * LOAD_RADIUS / gear / EFFICIENCY


T_REF = (F_REF * HANDLE_LENGTH - IDLE_TORQUE) * GEAR_SLOW * EFFICIENCY / LOAD_RADIUS + T_TAIL  # N at full trim
S_REF = LOAD_S1 + (T_REF - LOAD_T0 - LOAD_K1 * LOAD_S1) / LOAD_K2  # m of sheet taken in at full trim


def sheet_tension(s: float) -> float:
    """Sheet tension (N) with s metres of sheet taken in from the start (luffing, filling, bar-tight)."""
    s = max(0.0, s)
    if s <= LOAD_S1:
        return LOAD_T0 + LOAD_K1 * s
    if s <= S_REF:
        return stage_two_tension_at(s)
    return T_REF + LOAD_K3 * (s - S_REF)


def line_per_turn(gear: float) -> float:
    """Sheet taken in per handle turn (m) in a gear (handle turns per drum turn)."""
    return 2.0 * math.pi * LOAD_RADIUS / gear


def stall_point(grip_force: float, gear: float) -> float:
    """Sheet taken in (m) where a steady grip force (N, at HANDLE_LENGTH) no longer turns the handle in a gear."""
    torque = grip_force * HANDLE_LENGTH - IDLE_TORQUE
    tension = torque * gear * EFFICIENCY / LOAD_RADIUS + T_TAIL
    if tension <= LOAD_T0:
        return 0.0
    if tension <= LOAD_T0 + LOAD_K1 * LOAD_S1:
        return (tension - LOAD_T0) / LOAD_K1
    if tension <= T_REF:
        return LOAD_S1 + (tension - LOAD_T0 - LOAD_K1 * LOAD_S1) / LOAD_K2
    return S_REF + (tension - T_REF) / LOAD_K3


def capstan_hold(tail: float = T_TAIL, mu: float = CAPSTAN_MU, wraps: int = WRAPS) -> float:
    """The largest sheet load (N) the wraps hold against the tail's pull before the sheet slips round the drum."""
    return tail * math.exp(mu * 2.0 * math.pi * wraps)


# --- the sheet behind one interface: tension in, line travel out, whether the tail is held -------------------------
@dataclass
class SheetStandIn:
    """The sheet as a taut line from the load over the edge roller and the fairlead onto the drum, three wraps, and
    the tail to the hanging weight (no rope simulated). The weight tails it: while it hangs free and the wraps hold
    (sheet load under the capstan limit), every bit of drum turn takes sheet in. A simulated rope with a tailing hand
    can replace this class behind the same three members."""

    taken_in: float = 0.0  # m of sheet wound onto the drum since the start
    weight_free: bool = True  # the tail weight hangs free (nothing lifts it or holds the tail)

    def tension(self) -> float:
        return sheet_tension(self.taken_in)

    def tail_held(self) -> bool:
        return self.weight_free and self.tension() <= capstan_hold()

    def take_in(self, ds: float) -> None:
        if ds > 0.0 and self.tail_held():
            self.taken_in += ds


# --- the two-speed coupling, updated every control step -------------------------------------------------------------
FAST, SLOW, NONE = 1, -1, 0


def pushed_way(windup_torque: float) -> int:
    """Which gear the arm's push asks for, from the handle-to-spindle windup torque (N m, positive anticlockwise seen
    from above). Read from the spring's twist, so it is known with the handle at rest too."""
    anticlockwise = windup_torque > DEADBAND
    clockwise = windup_torque < -DEADBAND
    if not (anticlockwise or clockwise):
        return NONE
    return FAST if clockwise == FAST_IS_CLOCKWISE else SLOW


REVERSAL_MIN = math.radians(0.5)  # the spindle counts as reversed only after it has gone back this far (solver jitter of
# a few thousandths of a degree must not reset the backlash: a GPU test lost every bit of a slow creep that way)


@dataclass
class Winch:
    """The drum's angle (only ever grows), the gear engaged, and the brake limit the spindle feels.

    Each control step: `step(spindle_angle, windup_torque)` with the spindle's measured angle (rad, about the winch's
    axis, anticlockwise positive seen from above, unwrapped by the caller) and the windup torque. The spindle drives the
    drum only when it goes past the furthest point it has reached since its last reversal, and after BACKLASH of travel
    from that reversal; going back by more than REVERSAL_MIN is a reversal (the other gear). The drum turns by the
    spindle's turn over that gear, takes that much sheet in, and the step returns the brake's torque limit for the next
    step: the load in the pushed direction's gear; with no clear push, the larger (first-gear) limit, so a resting or
    released handle never creeps. If the tail is not held the drum turns under the slipping turns: no load on the handle
    beyond the idle drag, and no sheet taken in."""

    sheet: SheetStandIn = field(default_factory=SheetStandIn)
    drum: float = 0.0  # rad the drum has turned (always the same way)
    last_spindle: float | None = None
    moving: int = 0  # the spindle's direction since its last reversal: +1 anticlockwise, -1 clockwise, 0 not yet
    extreme: float = 0.0  # the furthest angle it has reached in that direction since the reversal
    slack: float = BACKLASH  # backlash still to take up before the gear drives
    turned: dict[int, float] = field(default_factory=lambda: {FAST: 0.0, SLOW: 0.0})  # rad of driving turn per gear
    against_limit: int = 0  # steps the spindle drove first gear while the brake held only the second gear's load

    def gear_of(self, d_spindle: float) -> int:
        clockwise = d_spindle < 0.0
        return FAST if clockwise == FAST_IS_CLOCKWISE else SLOW

    def limit(self, way: int) -> float:
        tension = self.sheet.tension() if self.sheet.tail_held() else 0.0
        gear = GEAR_SLOW if way == SLOW else GEAR_FAST
        return handle_torque(tension, gear)

    def _drive(self, travel: float, way: int, fast_before: float, limit_used: float | None) -> None:
        take = max(0.0, travel - self.slack)
        self.slack = max(0.0, self.slack - travel)
        if take <= 0.0:
            return
        gear = GEAR_FAST if way == FAST else GEAR_SLOW
        before = self.drum
        self.drum += take / gear
        self.sheet.take_in(LOAD_RADIUS * (self.drum - before))
        self.turned[way] += take
        if way == FAST and limit_used is not None and limit_used < fast_before - 1e-6:
            self.against_limit += 1

    def step(self, spindle_angle: float, windup_torque: float, limit_used: float | None = None) -> float:
        if self.last_spindle is None:
            self.last_spindle = spindle_angle
            self.extreme = spindle_angle
            return self.limit(pushed_way(windup_torque) or FAST)
        self.last_spindle = spindle_angle
        fast_before = self.limit(FAST)
        offset = spindle_angle - self.extreme
        if self.moving != 0 and offset * self.moving > 0.0:  # past the furthest point: drive
            self.extreme = spindle_angle
            self._drive(abs(offset), self.gear_of(offset), fast_before, limit_used)
        elif abs(offset) > REVERSAL_MIN:  # gone back far enough (or the first motion): a reversal
            self.moving = 1 if offset > 0.0 else -1
            self.extreme = spindle_angle
            self.slack = BACKLASH
            self._drive(abs(offset), self.gear_of(offset), fast_before, limit_used)
        return self.limit(pushed_way(windup_torque) or FAST)


# =====================================================================================================================
# Layout (env frame, metres: x away from the robot, y to its left, z up)
# =====================================================================================================================
DECK = (0.48, 0.74, 0.018)  # est: the plywood deck board (x, y, thickness)
DECK_AT = (0.38, 0.23)  # its centre seen from above: x 0.14..0.62, y -0.14..0.60
DECK_TOP = TOP + DECK[2]
WINCH_XY = (0.40, 0.10)  # the winch's axis (an IK reach screen: the left arm turns full circles here)
WINCH_TOP = DECK_TOP + WINCH_HEIGHT  # the socket's mouth
DRUM_R = DRUM_DIA / 2
BASE_H = 0.012  # est: the base flange's height
HOUSING_R, HOUSING_TOP = 0.050, 0.034  # est: the gear housing under the drum
DRUM_TOP = 0.112  # est: the drum's top, above the deck
FLANGE_R = 0.042  # est: the top flange
SOCKET_DEPTH = 0.025  # est: the socket's depth (the handle's spigot)
BORE_R = 0.0125  # est: the socket's bore
SPIGOT_R = 0.0115  # est: the handle's spigot (1 mm a side clear in the bore)
ROPE_R = 0.005  # a 10 mm sheet
WRAP_Z0, WRAP_PITCH = 0.044, 0.011  # the lowest wrap's centre above the deck, and the wraps' pitch (est)
SHEET_X = WINCH_XY[0] - (DRUM_R + ROPE_R)  # the sheet runs along y past the drum's side nearer the robot
LOAD_Z = DECK_TOP + WRAP_Z0  # the load leg's height (it comes onto the lowest wrap)
TAIL_Z = DECK_TOP + WRAP_Z0 + WRAPS * WRAP_PITCH  # the tail leaves the top wrap
FAIRLEAD_Y = -0.120  # the fairlead on the deck board's edge (the sheet comes onto the drum straight through it)
LOAD_ROLLER_Y = -0.640  # the edge roller the load leg runs over (springs below the bench)
TAIL_ROLLER_Y = 0.640  # the edge roller the tail runs over (the weight below the bench)
WEIGHT_DROP0 = 0.120  # the weight hangs this far below the bench top at the start (it drops as the tail pays out)
CLEAT_BOARD = (0.018, 0.200, 0.120)  # est: the vertical board the horn cleat is on (x thick, y long, z high)
CLEAT_BOARD_AT = (0.569, 0.480)  # its centre seen from above (outside the handle's sweep and the stowed handle's)
CLEAT_LEN = 0.150  # est: a 6 in horn cleat (stage A leaves it unused)
HOLDER_XY = (0.250, 0.500)  # the handle holder (a socket on a post, outside the handle's sweep; IK margin 29 deg)
HOLDER_TOP = DECK_TOP + 0.100  # est: the holder's top (the stowed arm clears the drawn tail)
HOLDER_BORE_R = 0.0135  # est: 2 mm a side clear of the spigot (a stowage socket, not a drive)
HOLDER_SPIN_D = 0.5  # N m s/rad: est: the holder bore's drag on the handle turning in it (a GPU test: undamped, the
# handle kept turning after the hand let go)
BORE_MU = 0.10  # est: a smooth (greased or nylon) bore; at 0.5 the handle's off-centre weight jammed the spigot in the
# holder's bore as it was lifted (GPU tests)
LEFT_BASE_XY = (0.0, 0.305)  # the left arm's base (the official station's)
# The stowed handle's arm points across the bearing from the left arm's base: the hub pinch (closing level across
# that bearing) then holds the handle's off-centre weight against its pads' faces, not by friction alone (a GPU test: 38 deg off
# that line the handle swung 60 deg in the pinch).
HOLDER_HEADING = math.atan2(HOLDER_XY[1] - LEFT_BASE_XY[1], HOLDER_XY[0] - LEFT_BASE_XY[0]) - math.pi / 2
HANDLE_START_HEADING = HOLDER_HEADING

# --- the handle's shape (est, apart from its length and mass) -------------------------------------------------------
# Frame: origin on the socket axis at the winch's top when seated, z up, x along the arm toward the knob.
GAP = 0.002  # the boss's underside over the winch's top when seated
BOSS_R, BOSS_TOP = 0.018, 0.050  # the socket boss
ARM_W, ARM_Z0, ARM_Z1 = 0.026, 0.006, 0.022  # the arm's width, underside and top
KNOB_R, KNOB_H = 0.016, 0.095  # the free-spinning knob
KNOB_Z0 = ARM_Z1 + 0.002  # the knob's bottom
HANDLE_BODY_MASS = HANDLE_MASS - 0.060 - 0.020  # the knob 60 g and the lever 20 g (est split)
HANDLE_COM = (0.085, 0.0, 0.020)  # est (the steel socket and lock at one end, the grip at the other)
KNOB_MASS, LEVER_MASS = 0.060, 0.020
# The lock (no maker text found; an owners' forum describes a small steel piece under the socket that the lever turns:
# estimates). A short red lever on top of the hub turns a latch at the drive tip; sprung to locked. With the lever held
# turned past LEVER_FREE the latch is clear of the socket's splines; let go below LEVER_LOCK it is back.
LEVER_R = 0.040  # m: the lever's tab, from the drive axis
LEVER_DIR = math.radians(72.0)  # the lever lies across the hub's top, 72 deg anticlockwise from the arm (seen from above)
LEVER_MAX = 30.0  # deg: its travel, anticlockwise against its spring
LEVER_FREE = 20.0  # deg
LEVER_LOCK = 10.0  # deg
LEVER_PRELOAD, LEVER_FULL = 2.0, 5.0  # N at the tab: its spring at rest and fully turned
LEVER_Z = BOSS_TOP  # the lever's underside on the hub's top
LEVER_BAR = (LEVER_R + 0.006, 0.010, 0.006)  # the bar from the axis out (x), its width (y) and thickness (z)
LEVER_TAB = (0.016, 0.006, 0.024)  # the thumb tab rising at its end: 16 mm along the lever, 6 mm thick, 24 mm tall
LEVER_PUSH_Z = LEVER_Z + 0.020  # the tab is pushed this high (handle frame)
CATCH_K, CATCH_F = 200000.0, 300.0  # N/m and N: the lock while latched (nothing a hand can pull out)

# --- trim -----------------------------------------------------------------------------------------------------------
MARK_W = 0.012  # the red mark's width along the sheet
STOPPER_R = 0.012  # the stopper ball


def drum_edge_y() -> float:
    """Where the load leg first touches the drum (along y on the sheet's line)."""
    return WINCH_XY[1]


def mark_y(taken_in: float) -> float:
    """The mark's place along the sheet's line: it reaches the drum's edge at full trim."""
    return drum_edge_y() - max(S_REF - taken_in, 0.0)


def stopper_y(taken_in: float) -> float:
    """The stopper ball sits on the sheet the fairlead's distance beyond the mark: it meets the snubber at full trim."""
    return FAIRLEAD_Y - 0.008 - 0.006 - STOPPER_R - max(S_REF - taken_in, -0.010)


# =====================================================================================================================
# The scene hook: when yam_arena.scene is imported, the deck board's spawner also builds everything else
# =====================================================================================================================
_SCENE_MODULE = "yam_arena.scene"
HOOK_TAG = "sheet_winch_hook"
RECORD_NAME = "sheet_winch_record"
BASE = "deck_board"
OBJECTS = [Box(BASE, pos=(DECK_AT[0], DECK_AT[1], TOP + DECK[2] / 2), size=DECK, color=(0.72, 0.58, 0.40))]
CONTACT_OFFSET, REST_OFFSET = 0.0008, 0.0
INERTIA_FLOOR = 3e-7
DOFS = ("transX", "transY", "transZ", "rotX", "rotY", "rotZ")


def _record() -> dict[str, Any]:
    holder = sys.modules.get("yam_arena.task_file")
    if holder is None:
        return {}
    rec = getattr(holder, RECORD_NAME, None)
    if not isinstance(rec, dict):
        rec = {}
        setattr(holder, RECORD_NAME, rec)
    return rec


class _Pxr:
    def __init__(self) -> None:
        self.gf = importlib.import_module("pxr.Gf")
        self.sdf = importlib.import_module("pxr.Sdf")
        self.geom = importlib.import_module("pxr.UsdGeom")
        self.physics = importlib.import_module("pxr.UsdPhysics")
        self.shade = importlib.import_module("pxr.UsdShade")
        self.usd = importlib.import_module("pxr.Usd")
        self.vt = importlib.import_module("pxr.Vt")


def wxyz(r: np.ndarray) -> tuple[float, float, float, float]:
    m = np.asarray(r, dtype=float)
    t = float(np.trace(m))
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + m[i, i] - m[j, j] - m[k, k]) * 2
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return ((m[k, j] - m[j, k]) / s, q[0], q[1], q[2])


def xyzw(r: np.ndarray) -> list[float]:
    w, x, y, z = wxyz(r)
    return [x, y, z, w]


def quat_to_matrix(q_xyzw: Sequence[float] | np.ndarray) -> np.ndarray:
    x, y, z, w = (float(v) for v in q_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


UP = ry(-math.pi / 2)  # a joint frame whose x is the bodies' z (vertical axes use rotX, PhysX's twist)


def yaw_of(r: np.ndarray) -> float:
    return math.atan2(float(r[1, 0]), float(r[0, 0]))


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _material(px: _Pxr, stage: Any, path: str, rgb: Sequence[float], friction: tuple[float, float], rough: float = 0.6) -> Any:
    mat = px.shade.Material.Define(stage, path)
    shader = px.shade.Shader.Define(stage, f"{path}/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", px.sdf.ValueTypeNames.Color3f).Set(px.gf.Vec3f(*(_linear(float(c)) for c in rgb)))
    shader.CreateInput("roughness", px.sdf.ValueTypeNames.Float).Set(float(rough))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    api = px.physics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(float(friction[0]))
    api.CreateDynamicFrictionAttr(float(friction[1]))
    api.CreateRestitutionAttr(0.0)
    return mat


def _bind(px: _Pxr, prim: Any, mat: Any) -> None:
    api = px.shade.MaterialBindingAPI.Apply(prim)
    api.Bind(mat)
    api.Bind(mat, px.shade.Tokens.weakerThanDescendants, "physics")


def _collider(px: _Pxr, prim: Any) -> None:
    px.physics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", px.sdf.ValueTypeNames.Float).Set(float(CONTACT_OFFSET))
    prim.CreateAttribute("physxCollision:restOffset", px.sdf.ValueTypeNames.Float).Set(float(REST_OFFSET))


def _cube(px: _Pxr, stage: Any, path: str, center: Sequence[float] | np.ndarray, size: Sequence[float], mat: Any, collides: bool = True, rot: np.ndarray | None = None) -> str:
    cube = px.geom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([px.gf.Vec3f(-0.5, -0.5, -0.5), px.gf.Vec3f(0.5, 0.5, 0.5)])
    xf = px.geom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(px.gf.Vec3d(*(float(c) for c in center)))
    if rot is not None:
        xf.AddOrientOp().Set(px.gf.Quatf(*wxyz(rot)))
    xf.AddScaleOp().Set(px.gf.Vec3f(*(float(s) for s in size)))
    if collides:
        _collider(px, cube.GetPrim())
    _bind(px, cube.GetPrim(), mat)
    return path


def _cylinder(px: _Pxr, stage: Any, path: str, center: Sequence[float] | np.ndarray, radius: float, height: float, mat: Any, collides: bool = True, axis: str = "Z") -> str:
    cyl = px.geom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(float(radius))
    cyl.CreateHeightAttr(float(height))
    cyl.CreateAxisAttr(axis)
    half = {"X": (height / 2, radius, radius), "Y": (radius, height / 2, radius), "Z": (radius, radius, height / 2)}[axis]
    cyl.CreateExtentAttr([px.gf.Vec3f(*(-h for h in half)), px.gf.Vec3f(*half)])
    px.geom.Xformable(cyl.GetPrim()).AddTranslateOp().Set(px.gf.Vec3d(*(float(c) for c in center)))
    if collides:
        _collider(px, cyl.GetPrim())
    _bind(px, cyl.GetPrim(), mat)
    return path


def _sphere(px: _Pxr, stage: Any, path: str, center: Sequence[float], radius: float, mat: Any) -> str:
    sph = px.geom.Sphere.Define(stage, path)
    sph.CreateRadiusAttr(float(radius))
    sph.CreateExtentAttr([px.gf.Vec3f(-radius, -radius, -radius), px.gf.Vec3f(radius, radius, radius)])
    px.geom.Xformable(sph.GetPrim()).AddTranslateOp().Set(px.gf.Vec3d(*(float(c) for c in center)))
    _bind(px, sph.GetPrim(), mat)
    return path


def _mesh(px: _Pxr, stage: Any, path: str, points: np.ndarray, tris: np.ndarray, mat: Any) -> str:
    mesh = px.geom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([px.gf.Vec3f(*(float(c) for c in p)) for p in points])
    mesh.CreateFaceVertexCountsAttr([3] * len(tris))
    mesh.CreateFaceVertexIndicesAttr([int(i) for i in np.asarray(tris).ravel()])
    lo, hi = points.min(axis=0), points.max(axis=0)
    mesh.CreateExtentAttr([px.gf.Vec3f(*(float(c) for c in lo)), px.gf.Vec3f(*(float(c) for c in hi))])
    mesh.CreateSubdivisionSchemeAttr("none")
    _bind(px, mesh.GetPrim(), mat)
    return path


def tube(path_pts: np.ndarray, radius: float, sides: int = 10) -> tuple[np.ndarray, np.ndarray]:
    """A tube mesh along a polyline (drawn rope)."""
    pts, tris = [], []
    n = len(path_pts)
    for i in range(n):
        a = path_pts[min(i + 1, n - 1)] - path_pts[max(i - 1, 0)]
        a = a / max(float(np.linalg.norm(a)), 1e-9)
        ref = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        u = np.cross(a, ref)
        u /= float(np.linalg.norm(u))
        v = np.cross(a, u)
        for k in range(sides):
            t = 2 * math.pi * k / sides
            pts.append(path_pts[i] + radius * (math.cos(t) * u + math.sin(t) * v))
    for i in range(n - 1):
        for k in range(sides):
            a0, a1 = i * sides + k, i * sides + (k + 1) % sides
            b0, b1 = a0 + sides, a1 + sides
            tris += [(a0, b0, a1), (a1, b0, b1)]
    return np.array(pts), np.array(tris)


def wrap_path(n: int = 180) -> np.ndarray:
    """The three clockwise wraps (seen from above): from the drum's side nearer the robot, rising one pitch a turn."""
    c = np.array([WINCH_XY[0], WINCH_XY[1]])
    out = []
    for i in range(n + 1):
        t = i / n
        phi = math.pi - 2 * math.pi * WRAPS * t  # clockwise from the side nearer the robot (phi = 180 deg)
        r = DRUM_R + ROPE_R
        out.append([c[0] + r * math.cos(phi), c[1] + r * math.sin(phi), DECK_TOP + WRAP_Z0 + WRAP_PITCH * WRAPS * t])
    return np.array(out)


def _body(px: _Pxr, stage: Any, path: str, pos: Sequence[float] | np.ndarray, rot: np.ndarray, mass: float, inertia: Sequence[float], com: Sequence[float] | np.ndarray = (0.0, 0.0, 0.0), kinematic: bool = False) -> Any:
    xf = px.geom.Xform.Define(stage, path)
    xf.AddTranslateOp(precision=px.geom.XformOp.PrecisionDouble).Set(px.gf.Vec3d(*(float(v) for v in pos)))
    xf.AddOrientOp().Set(px.gf.Quatf(*wxyz(rot)))
    prim = xf.GetPrim()
    px.physics.RigidBodyAPI.Apply(prim)
    if kinematic:
        prim.GetAttribute("physics:kinematicEnabled").Set(True)
    prim.AddAppliedSchema("PhysxRigidBodyAPI")
    f = px.sdf.ValueTypeNames.Float
    prim.CreateAttribute("physxRigidBody:linearDamping", f).Set(0.05)
    prim.CreateAttribute("physxRigidBody:angularDamping", f).Set(0.05)
    prim.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", f).Set(0.5)
    prim.CreateAttribute("physxRigidBody:sleepThreshold", f).Set(0.0)
    prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", px.sdf.ValueTypeNames.Int).Set(32)
    prim.CreateAttribute("physxRigidBody:solverVelocityIterationCount", px.sdf.ValueTypeNames.Int).Set(4)
    mapi = px.physics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(mass))
    mapi.CreateCenterOfMassAttr(px.gf.Vec3f(*(float(c) for c in com)))
    mapi.CreateDiagonalInertiaAttr(px.gf.Vec3f(*(max(float(v), INERTIA_FLOOR) for v in inertia)))
    return prim


def _drive(px: _Pxr, prim: Any, axis: str, k: float, d: float, fmax: float, target: float = 0.0) -> None:
    api = px.physics.DriveAPI.Apply(prim, axis)
    api.CreateTypeAttr("force")
    api.CreateStiffnessAttr(float(k))
    api.CreateDampingAttr(float(d))
    api.CreateMaxForceAttr(float(fmax))  # finite: PhysX rejects an infinite drive limit
    api.CreateTargetPositionAttr(float(target))
    api.CreateTargetVelocityAttr(0.0)


def _joint(px: _Pxr, stage: Any, path: str, body0: str | None, body1: str, pos0: Sequence[float], rot0: np.ndarray, pos1: Sequence[float], limits: dict[str, tuple[float, float] | None], enabled: bool = True, rot1: np.ndarray | None = None) -> Any:
    """A D6 joint: every DOF locked except those in `limits` (None: free; (lo, hi): limited, deg for rotations). A turn
    about a vertical axis uses rotX (PhysX's twist) with both frames turned by UP (x along the bodies' z): a free rotZ is
    a swing axis in PhysX and does not drive as a plain revolute."""
    j = px.physics.Joint.Define(stage, path)
    if body0 is not None:
        j.CreateBody0Rel().SetTargets([body0])
    j.CreateBody1Rel().SetTargets([body1])
    j.CreateLocalPos0Attr(px.gf.Vec3f(*(float(c) for c in pos0)))
    j.CreateLocalRot0Attr(px.gf.Quatf(*wxyz(rot0)))
    j.CreateLocalPos1Attr(px.gf.Vec3f(*(float(c) for c in pos1)))
    j.CreateLocalRot1Attr(px.gf.Quatf(*wxyz(np.eye(3) if rot1 is None else rot1)))
    j.CreateExcludeFromArticulationAttr(True)
    j.CreateJointEnabledAttr(bool(enabled))
    j.CreateCollisionEnabledAttr(False)
    prim = j.GetPrim()
    for dof in DOFS:
        if dof in limits and limits[dof] is None:
            continue
        lim = px.physics.LimitAPI.Apply(prim, dof)
        lo_hi = limits.get(dof)
        if lo_hi is None:
            lim.CreateLowAttr(1.0)
            lim.CreateHighAttr(-1.0)  # low > high: locked
        else:
            lim.CreateLowAttr(float(lo_hi[0]))
            lim.CreateHighAttr(float(lo_hi[1]))
    return prim


def handle_pose_in_holder() -> tuple[np.ndarray, np.ndarray]:
    """The handle's frame standing in its holder (seated as in the winch)."""
    return np.array([HOLDER_XY[0], HOLDER_XY[1], HOLDER_TOP]), rz(HANDLE_START_HEADING)


def _bore_ring(px: _Pxr, stage: Any, path: str, centre: np.ndarray, top: float, depth: float, r_in: float, r_out: float, mat: Any, n: int = 16) -> None:
    """A socket's bore as a ring of boxes from top - depth to top (r_in to r_out), drawn dark."""
    width = 2 * r_in * math.tan(math.pi / n)
    for i in range(n):
        a = 2 * math.pi * i / n
        rm = (r_in + r_out) / 2
        c = (centre[0] + rm * math.cos(a), centre[1] + rm * math.sin(a), top - depth / 2)
        _cube(px, stage, f"{path}/ring{i:02d}", c, (r_out - r_in, width, depth), mat, rot=rz(a))


def author(stage: Any, env_root: str) -> None:
    px = _Pxr()
    rec = _record()
    root = f"{env_root}/winch_rig"
    px.geom.Xform.Define(stage, root)
    alu = _material(px, stage, f"{root}/alu", (0.70, 0.72, 0.74), (0.5, 0.4), 0.35)
    dark = _material(px, stage, f"{root}/dark", (0.12, 0.12, 0.13), (0.5, 0.4), 0.5)
    chrome = _material(px, stage, f"{root}/chrome", (0.86, 0.87, 0.88), (0.5, 0.4), 0.2)
    rope = _material(px, stage, f"{root}/rope", (0.93, 0.93, 0.88), (0.6, 0.5), 0.9)
    red = _material(px, stage, f"{root}/red", (0.85, 0.08, 0.08), (0.6, 0.5), 0.5)
    black = _material(px, stage, f"{root}/black", (0.05, 0.05, 0.05), (0.8, 0.7), 0.7)
    wood = _material(px, stage, f"{root}/wood", (0.62, 0.48, 0.32), (0.6, 0.5), 0.8)
    steel = _material(px, stage, f"{root}/steel", (0.55, 0.56, 0.58), (0.5, 0.4), 0.4)
    i3 = np.eye(3)
    wx, wy = WINCH_XY

    # The winch: a kinematic body (the joints' anchor) with its collision shapes; the drum is drawn on its own Xform
    # that the run turns (a visual: the drum's turn is computed from the gears).
    winch = f"{root}/winch"
    _body(px, stage, winch, (wx, wy, DECK_TOP), i3, WINCH_MASS, (0.01, 0.01, 0.01), kinematic=True)
    _cylinder(px, stage, f"{winch}/base", (0.0, 0.0, BASE_H / 2), BASE_DIA / 2, BASE_H, alu)
    _cylinder(px, stage, f"{winch}/housing", (0.0, 0.0, (BASE_H + HOUSING_TOP) / 2), HOUSING_R, HOUSING_TOP - BASE_H, alu)
    core_top = WINCH_HEIGHT - SOCKET_DEPTH
    _cylinder(px, stage, f"{winch}/core", (0.0, 0.0, (HOUSING_TOP + core_top) / 2), DRUM_R - 0.0005, core_top - HOUSING_TOP, alu)
    rec_ring = f"{winch}/socket"
    px.geom.Xform.Define(stage, rec_ring)
    bore = _material(px, stage, f"{root}/bore", (0.12, 0.12, 0.13), (BORE_MU, BORE_MU * 0.8), 0.3)
    _bore_ring(px, stage, rec_ring, np.zeros(2), WINCH_HEIGHT, SOCKET_DEPTH, BORE_R, 0.030, bore)
    drum = f"{root}/drum"
    dx = px.geom.Xform.Define(stage, drum)
    dx.AddTranslateOp().Set(px.gf.Vec3d(wx, wy, DECK_TOP))
    dx.AddRotateZOp().Set(0.0)
    _cylinder(px, stage, f"{drum}/barrel", (0.0, 0.0, (HOUSING_TOP + DRUM_TOP) / 2), DRUM_R, DRUM_TOP - HOUSING_TOP, chrome, collides=False)
    _cylinder(px, stage, f"{drum}/flange", (0.0, 0.0, (DRUM_TOP + WINCH_HEIGHT) / 2), FLANGE_R, WINCH_HEIGHT - DRUM_TOP - 0.0004, alu, collides=False)
    _cube(px, stage, f"{drum}/stripe", (DRUM_R - 0.0002, 0.0, (HOUSING_TOP + DRUM_TOP) / 2), (0.002, 0.008, DRUM_TOP - HOUSING_TOP - 0.004), dark, collides=False)
    _cube(px, stage, f"{drum}/flange_stripe", (FLANGE_R - 0.004, 0.0, WINCH_HEIGHT - 0.0002), (0.008, 0.006, 0.0006), red, collides=False)
    rec["drum"] = drum

    # The spindle: the winch's input shaft (no shapes), on the brake drive about the axis.
    spindle = f"{root}/spindle"
    _body(px, stage, spindle, (wx, wy, WINCH_TOP), i3, 0.2, (1e-2, 1e-2, 1e-2))
    for i in range(8):  # the socket's star, drawn at the bore's bottom (it turns with the spindle)
        _cube(px, stage, f"{spindle}/star{i}", (0.0, 0.0, -SOCKET_DEPTH + 0.0006), (2 * BORE_R - 0.001, 0.004, 0.001), steel, collides=False, rot=rz(i * math.pi / 8))
    brake = _joint(px, stage, f"{root}/spindle_brake", winch, spindle, (0.0, 0.0, WINCH_HEIGHT), UP, (0.0, 0.0, 0.0), {"rotX": None}, rot1=UP)
    _drive(px, brake, "rotX", SPINDLE_K * math.pi / 180.0, SPINDLE_D * math.pi / 180.0, 1000.0)
    rec.update(winch=winch, spindle=spindle, brake=f"{root}/spindle_brake")

    # The handle in its holder: spigot, hub, arm (colliding); the knob and the lock lever are their own bodies.
    hpos, hrot = handle_pose_in_holder()
    handle = f"{root}/handle"
    _body(px, stage, handle, hpos, hrot, HANDLE_BODY_MASS, (2e-4, 2.8e-3, 2.8e-3), com=HANDLE_COM)
    sp_len = SOCKET_DEPTH - 0.0005
    _cylinder(px, stage, f"{handle}/spigot", (0.0, 0.0, -sp_len / 2), SPIGOT_R, sp_len, steel)
    grey = _material(px, stage, f"{root}/handle_grey", (0.56, 0.58, 0.61), (0.5, 0.4), 0.4)  # the grey alloy body
    red_grip = _material(px, stage, f"{root}/handle_red", (0.80, 0.07, 0.07), (0.9, 0.8), 0.6)  # the grip and lever
    _cylinder(px, stage, f"{handle}/boss", (0.0, 0.0, (GAP + BOSS_TOP) / 2), BOSS_R, BOSS_TOP - GAP, grey)
    arm_x0 = BOSS_R - 0.004
    _cube(px, stage, f"{handle}/arm", ((arm_x0 + HANDLE_LENGTH) / 2, 0.0, (ARM_Z0 + ARM_Z1) / 2), (HANDLE_LENGTH - arm_x0, ARM_W, ARM_Z1 - ARM_Z0), grey)
    _cylinder(px, stage, f"{handle}/knob_seat", (HANDLE_LENGTH, 0.0, (ARM_Z0 + KNOB_Z0) / 2), 0.013, KNOB_Z0 - ARM_Z0, grey, collides=False)
    knob = f"{root}/knob"
    kpos = hpos + hrot @ np.array([HANDLE_LENGTH, 0.0, KNOB_Z0])
    _body(px, stage, knob, kpos, hrot, KNOB_MASS, (5e-5, 5e-5, 8e-6), com=(0.0, 0.0, KNOB_H / 2))
    _cylinder(px, stage, f"{knob}/grip", (0.0, 0.0, KNOB_H / 2), KNOB_R, KNOB_H, red_grip)
    _joint(px, stage, f"{root}/knob_spin", handle, knob, (HANDLE_LENGTH, 0.0, KNOB_Z0), UP, (0.0, 0.0, 0.0), {"rotX": None}, rot1=UP)
    # The lock lever: its frame on the drive axis at the hub's top, x along the lever; it turns about the axis.
    lever = f"{root}/lever"
    lrot = hrot @ rz(LEVER_DIR)
    lpos = hpos + hrot @ np.array([0.0, 0.0, LEVER_Z])
    _body(px, stage, lever, lpos, lrot, LEVER_MASS, (2e-6, 6e-6, 6e-6), com=(0.025, 0.0, 0.006))
    _cube(px, stage, f"{lever}/bar", (LEVER_BAR[0] / 2, 0.0, LEVER_BAR[2] / 2), LEVER_BAR, red_grip)
    _cube(px, stage, f"{lever}/tab", (LEVER_R, 0.0, LEVER_BAR[2] + LEVER_TAB[2] / 2), LEVER_TAB, red_grip)
    lturn = _joint(px, stage, f"{root}/lever_turn", handle, lever, (0.0, 0.0, LEVER_Z), rz(LEVER_DIR) @ UP, (0.0, 0.0, 0.0),
                   {"rotX": (0.0, LEVER_MAX)}, rot1=UP)
    k_lever = (LEVER_FULL - LEVER_PRELOAD) * LEVER_R / LEVER_MAX  # N m per deg
    _drive(px, lturn, "rotX", k_lever, k_lever * 0.02, 5.0, -LEVER_PRELOAD * LEVER_R / k_lever)
    # The lock-in socket joint: spindle to handle, enabled while the spigot is in the socket. transX is the lock (a stiff
    # drive while latched or while the latch is across, free while the lever is held turned); rotX is the windup spring
    # that reads the push (the lug's points are in the socket's from the moment it counts as in). Both frames are turned
    # by UP, so the joint's x (transX, rotX) is the socket's axis.
    sock = _joint(px, stage, f"{root}/handle_socket", spindle, handle, (0.0, 0.0, 0.0), UP, (0.0, 0.0, 0.0),
                  {"transX": (0.0, SOCKET_DEPTH + 0.003), "rotX": None}, enabled=False, rot1=UP)
    _drive(px, sock, "rotX", WINDUP_K * math.pi / 180.0, WINDUP_D * math.pi / 180.0, 200.0)
    _drive(px, sock, "transX", CATCH_K, 50.0, CATCH_F)
    rec.update(handle=handle, knob=knob, lever=lever, socket=f"{root}/handle_socket")

    # The holder: a socket on a post (static shapes).
    holder = f"{root}/holder"
    px.geom.Xform.Define(stage, holder)
    hc = np.array(HOLDER_XY)
    post_top = HOLDER_TOP - SOCKET_DEPTH
    _cylinder(px, stage, f"{holder}/post", (hc[0], hc[1], (DECK_TOP + post_top) / 2), 0.022, post_top - DECK_TOP, dark)
    _bore_ring(px, stage, holder, hc, HOLDER_TOP, SOCKET_DEPTH, HOLDER_BORE_R, 0.026, bore)
    # The holder's snug fit: while the spigot is seated a joint keeps the handle upright in the holder's bore (free
    # along and round the axis); lifted out, the handle is free. (A loose bore let it lean 10 deg and jam: GPU tests.)
    origin = np.asarray(rec.get("origin", (0.0, 0.0, 0.0)), dtype=float)
    hj = _joint(px, stage, f"{root}/holder_socket", None, handle, origin + np.array([hc[0], hc[1], HOLDER_TOP]), rz(HANDLE_START_HEADING) @ UP,
                (0.0, 0.0, 0.0), {"transX": (0.0, SOCKET_DEPTH + 0.003), "rotX": None}, enabled=True, rot1=UP)
    _drive(px, hj, "rotX", 0.0, HOLDER_SPIN_D * math.pi / 180.0, 5.0)  # the bore's friction: a let-go handle stops turning
    rec["holder_socket"] = f"{root}/holder_socket"

    # The sheet (drawn): the load leg from the edge roller through the fairlead to the drum, three wraps, the tail to
    # the far edge roller, and the drops to the springs and the weight below the bench top.
    load_pts = np.array([[SHEET_X, LOAD_ROLLER_Y, TOP + 0.012 + ROPE_R], [SHEET_X, FAIRLEAD_Y, LOAD_Z], [SHEET_X, wy, LOAD_Z]])
    for name, pts in (
        ("sheet_load", load_pts),
        ("sheet_wraps", wrap_path()),
        ("sheet_tail", np.array([[SHEET_X, wy, TAIL_Z], [SHEET_X, TAIL_ROLLER_Y, TOP + 0.012 + ROPE_R]])),
        ("sheet_load_drop", np.array([[SHEET_X, LOAD_ROLLER_Y - 0.012 - ROPE_R, TOP], [SHEET_X, LOAD_ROLLER_Y - 0.012 - ROPE_R, TOP - 0.25]])),
        ("sheet_tail_drop", np.array([[SHEET_X, TAIL_ROLLER_Y + 0.012 + ROPE_R, TOP], [SHEET_X, TAIL_ROLLER_Y + 0.012 + ROPE_R, TOP - WEIGHT_DROP0]])),
    ):
        p, t = tube(pts, ROPE_R)
        _mesh(px, stage, f"{root}/{name}", p, t, rope)
    _cylinder(px, stage, f"{root}/tail_weight", (SHEET_X, TAIL_ROLLER_Y + 0.012 + ROPE_R, TOP - WEIGHT_DROP0 - 0.025), 0.025, 0.050, black, collides=False)
    for name, y in (("load_roller", LOAD_ROLLER_Y), ("tail_roller", TAIL_ROLLER_Y)):
        _cylinder(px, stage, f"{root}/{name}", (SHEET_X, y, TOP + 0.012), 0.012, 0.040, steel, axis="X")
        _cube(px, stage, f"{root}/{name}_bracket", (SHEET_X, y, TOP + 0.006), (0.060, 0.030, 0.012), dark)
    # The fairlead: a block with an eye on the deck board's edge, its rubber snubber on the load side.
    _cube(px, stage, f"{root}/fairlead", (SHEET_X, FAIRLEAD_Y, DECK_TOP + 0.020), (0.040, 0.016, 0.040), dark)
    _cylinder(px, stage, f"{root}/snubber", (SHEET_X, FAIRLEAD_Y - 0.011, LOAD_Z), 0.010, 0.006, black, collides=False, axis="Y")
    # The mark and the stopper ball ride on the load leg (moved each step).
    for name, radius, length, mat in (("mark", ROPE_R + 0.0006, MARK_W, red), ("stopper", STOPPER_R, 2 * STOPPER_R, black)):
        x = px.geom.Xform.Define(stage, f"{root}/{name}")
        y0 = mark_y(0.0) if name == "mark" else stopper_y(0.0)
        x.AddTranslateOp().Set(px.gf.Vec3d(SHEET_X, y0, LOAD_Z))
        if name == "mark":
            _cylinder(px, stage, f"{root}/{name}/band", (0.0, 0.0, 0.0), radius, length, mat, collides=False, axis="Y")
        else:
            _sphere(px, stage, f"{root}/{name}/ball", (0.0, 0.0, 0.0), radius, mat)
        rec[name] = f"{root}/{name}"
    # The horn cleat on its vertical board (stage A leaves it unused).
    bx, by = CLEAT_BOARD_AT
    _cube(px, stage, f"{root}/cleat_board", (bx, by, DECK_TOP + CLEAT_BOARD[2] / 2), CLEAT_BOARD, wood)
    face = bx - CLEAT_BOARD[0] / 2
    cz = DECK_TOP + 0.070
    _cube(px, stage, f"{root}/cleat_base", (face - 0.008, by, cz), (0.016, 0.050, 0.020), alu)
    _cube(px, stage, f"{root}/cleat_horns", (face - 0.022, by, cz), (0.016, CLEAT_LEN, 0.016), alu)
    rec["init"] = {
        spindle: [wx, wy, WINCH_TOP, *xyzw(i3)],
        handle: [*map(float, hpos), *xyzw(hrot)],
        knob: [*map(float, kpos), *xyzw(hrot)],
        lever: [*map(float, lpos), *xyzw(lrot)],
    }
    rec.update(root=root, built=True, env_root=env_root)


def _with_build(spawn: Callable[..., Any]) -> Callable[..., Any]:
    def spawn_and_build(prim_path: str, cfg: Any, *args: Any, **kwargs: Any) -> Any:
        prim = spawn(prim_path, cfg, *args, **kwargs)
        try:
            env_root = str(prim.GetPath().GetParentPath())
            if not _record().get("built"):
                _record()["origin"] = _env_origin(prim.GetStage(), env_root)
                author(prim.GetStage(), env_root)
        except Exception as error:
            _record()["build_error"] = f"{type(error).__name__}: {error}"
        return prim

    setattr(spawn_and_build, HOOK_TAG, True)
    return spawn_and_build


def _env_origin(stage: Any, env_root: str) -> list[float]:
    px = _Pxr()
    m = px.geom.Xformable(stage.GetPrimAtPath(env_root)).ComputeLocalToWorldTransform(px.usd.TimeCode.Default())
    t = m.ExtractTranslation()
    return [float(t[0]), float(t[1]), float(t[2])]


def build_scene(scene: Any) -> None:
    original = getattr(scene, "task_object", None)
    if original is None or getattr(original, HOOK_TAG, None):
        return

    def task_object(spec: Any) -> Any:
        made = original(spec)
        if getattr(spec, "name", None) == BASE:
            stack = [made, *vars(made).values()] if hasattr(made, "__dict__") else [made]
            for cand in stack:
                for value in (cand, getattr(cand, "spawn", None), getattr(cand, "spawner_cfg", None)):
                    if value is not None and callable(getattr(value, "func", None)) and hasattr(value, "size") and not getattr(value.func, HOOK_TAG, None):
                        value.func = _with_build(value.func)
        return made

    setattr(task_object, HOOK_TAG, True)
    scene.task_object = task_object


class _Watcher(importlib.abc.MetaPathFinder):
    marker = "sheet_winch_watcher"

    def __init__(self) -> None:
        self._looking = False

    def find_spec(self, fullname: str, path: Sequence[str] | None, target: types.ModuleType | None = None) -> importlib.machinery.ModuleSpec | None:
        if fullname != _SCENE_MODULE or self._looking:
            return None
        self._looking = True
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            self._looking = False
        if spec is None or spec.loader is None:
            return spec
        spec.loader = _After(spec.loader, self)
        return spec

    def disarm(self) -> None:
        if self in sys.meta_path:
            sys.meta_path.remove(self)


class _After(importlib.abc.Loader):
    def __init__(self, inner: importlib.abc.Loader, watcher: _Watcher) -> None:
        self._inner, self._watcher = inner, watcher

    def create_module(self, spec: importlib.machinery.ModuleSpec) -> types.ModuleType | None:
        return self._inner.create_module(spec)

    def exec_module(self, module: types.ModuleType) -> None:
        try:
            self._inner.exec_module(module)
        finally:
            self._watcher.disarm()
        build_scene(module)

    def __getattr__(self, name: str) -> Any:
        if name.startswith("__") or name in ("_inner", "_watcher"):
            raise AttributeError(name)
        return getattr(self._inner, name)


def install_hook() -> None:
    scene = sys.modules.get(_SCENE_MODULE)
    if scene is not None:
        build_scene(scene)
    elif not any(getattr(f, "marker", None) == _Watcher.marker for f in sys.meta_path):
        sys.meta_path.insert(0, _Watcher())


install_hook()


# =====================================================================================================================
# Runtime: PhysX views and USD writes
# =====================================================================================================================
def _stage() -> Any:
    return importlib.import_module("omni.usd").get_context().get_stage()


_written: dict[tuple[str, str], Any] = {}


def set_attr(path: str, name: str, value: Any) -> None:
    key = (path, name)
    if _written.get(key) == value:
        return
    attr = _stage().GetPrimAtPath(path).GetAttribute(name)
    if not attr.IsValid():  # never create one while running (PhysX would re-read the stage): authored up front
        Live.failed = Live.failed or f"no attribute {name} on {path}"
        return
    attr.Set(value)
    _written[key] = value


def _sim_view() -> Any:
    try:
        return importlib.import_module("isaaclab_physx.physics").PhysxManager.get_physics_sim_view()
    except (ImportError, AttributeError):
        return importlib.import_module("isaaclab.sim").SimulationContext.instance().physics_sim_view


def _np(x: Any) -> np.ndarray:
    for step in ("detach", "cpu", "numpy"):
        if hasattr(x, step):
            x = getattr(x, step)()
    return np.asarray(x, dtype=float)


Bodies = dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]]


class Live:
    bodies: Any = None
    paths: list[str] = []  # noqa: RUF012
    failed: str = ""
    extra_bodies: list[str] = []  # noqa: RUF012 (a test rig may add its own bodies here before the first step)
    last: Bodies | None = None  # the last step's readings (the demo's live targets read them)
    start: dict[str, np.ndarray] = {}  # noqa: RUF012


def read_bodies() -> Bodies | None:
    """Our loose bodies: env-frame position, rotation matrix, velocity (6)."""
    rec = _record()
    if not rec.get("built"):
        return None
    if Live.bodies is None and not Live.failed:
        try:
            sv = _sim_view()
            if sv is None:
                return None
            want = [rec["spindle"], rec["handle"], rec["knob"], rec["lever"], *Live.extra_bodies]
            Live.bodies = sv.create_rigid_body_view(want)
            Live.paths = list(Live.bodies.prim_paths)
        except Exception as error:
            Live.failed = f"{type(error).__name__}: {error}"
    if Live.bodies is None:
        return None
    tf = _np(Live.bodies.get_transforms()).reshape(-1, 7)
    vel = _np(Live.bodies.get_velocities()).reshape(-1, 6)
    origin = np.asarray(rec.get("origin", (0.0, 0.0, 0.0)))
    out = {p: (tf[i, :3] - origin, quat_to_matrix(tf[i, 3:7]), vel[i]) for i, p in enumerate(Live.paths)}
    Live.last = out
    return out


def _like(ref: Any, values: np.ndarray, index: bool = False) -> Any:
    if type(ref).__module__.startswith("warp"):
        wp = importlib.import_module("warp")
        if index:
            return wp.array(values.astype(np.int32), dtype=wp.int32, device=ref.device)
        return wp.array(values.astype(np.float32), dtype=wp.float32, device=ref.device)
    if hasattr(ref, "clone"):
        torch = importlib.import_module("torch")
        return torch.as_tensor(values, dtype=torch.long if index else ref.dtype, device=ref.device)
    return values.astype(np.int32 if index else np.float32)


def restore_bodies() -> None:
    """A new trial: the spindle, the handle, its knob and lever back where they started, at rest (written through
    PhysX: these bodies are the scene hook's own and the simulator's reset does not know them)."""
    rec = _record()
    init = rec.get("init")
    if Live.bodies is None or not isinstance(init, dict):
        raise RuntimeError("the parts' view does not exist, so they cannot be put back")
    ref = Live.bodies.get_transforms()
    data = _np(ref).reshape(len(Live.paths), 7).copy()
    origin = np.asarray(rec.get("origin", (0.0, 0.0, 0.0)), dtype=float)
    ours = [i for i, path in enumerate(Live.paths) if path in init]
    if len(ours) != len(init):
        raise RuntimeError("not every part is in the view")
    for i in ours:
        data[i] = np.asarray(init[Live.paths[i]], dtype=float)
        data[i, :3] += origin
    idx = np.asarray(ours, dtype=np.int64)
    Live.bodies.set_transforms(_like(ref, data), _like(ref, idx, index=True))
    Live.bodies.set_velocities(_like(ref, np.zeros((len(Live.paths), 6))), _like(ref, idx, index=True))


RESTORE_TOL = 0.005  # m: at a new trial's third reading, every part within this of where it was at the first trial's
RESTORE_CHECK_STEP = 3


def restore_error(b: Bodies) -> str:
    """'' if every part is within RESTORE_TOL of where it was at the first trial's same reading, else which is off."""
    init = _record().get("init")
    if not isinstance(init, dict):
        return "no start poses"
    for path in init:
        if path not in b:
            return f"{path.rsplit('/', 1)[-1]} not read"
        ref = Live.start.get(path)
        if ref is None:
            return "no first-trial start recorded"
        off = float(np.linalg.norm(b[path][0] - ref))
        if not off <= RESTORE_TOL:
            return f"{path.rsplit('/', 1)[-1]} {off * 1000:.1f} mm off"
    return ""


# --- the handle in the socket ---------------------------------------------------------------------------------------
SEAT_R = 0.002  # m: the spigot's axis within this of the socket's to count as seated
SEAT_TILT = 5.0  # deg
SEAT_Z = 0.003  # m: the handle's frame within this above the socket's mouth
OUT_Z = SOCKET_DEPTH - 0.002  # m: once lifted this far (the spigot at the bore's mouth), the handle is free
HOLDER_R, HOLDER_TILT = 0.004, 8.0  # m, deg: seated in the holder (its bore is 2 mm a side clear)


def wrap_angle(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def lever_turn(hr: np.ndarray, lr: np.ndarray) -> float:
    """How far the lock lever is turned from its rest (deg, anticlockwise positive)."""
    return math.degrees(wrap_angle(yaw_of(lr) - yaw_of(hr) - LEVER_DIR))


def seated_in(hp: np.ndarray, hr: np.ndarray, centre: Sequence[float], top: float, tol_z: float = SEAT_Z,
              tol_r: float = SEAT_R, tol_tilt: float = SEAT_TILT) -> bool:
    off = math.hypot(float(hp[0] - centre[0]), float(hp[1] - centre[1]))
    tilt = math.degrees(math.acos(float(np.clip(hr[2, 2], -1.0, 1.0))))
    return off <= tol_r and tilt <= tol_tilt and -0.001 <= float(hp[2] - top) <= tol_z


@dataclass
class Socket:
    """The lock-in socket's state: free (the handle out), engaged (seated, the lock free) or latched."""

    engaged: bool = False
    offset: float = 0.0  # rad: the handle's turn on the spindle when it went in (the windup spring's zero)
    latched: bool = False
    inserts: int = 0
    removals: int = 0
    pulled_latched: float = 0.0  # m: the largest rise while latched (the lock must hold)
    blocked: bool = False  # in the socket with the latch across (lever not held): it can go neither in nor out
    block_at: float = 0.0  # m: the rise where the latch caught


@dataclass
class Holder:
    """The handle holder's snug fit: engaged while the spigot is seated in it (the handle stands upright)."""

    engaged: bool = True  # the handle starts in its holder
    yaw: float = HANDLE_START_HEADING  # the handle's heading when it went in
    removals: int = 0
    inserts: int = 0


@dataclass
class State:
    """One trial's memory (a new trial starts a new one)."""

    winch: Winch = field(default_factory=Winch)
    socket: Socket = field(default_factory=Socket)
    holder: Holder = field(default_factory=Holder)
    spindle_angle: float = 0.0  # rad, unwrapped
    brake_angle: float = 0.0  # rad, unwrapped: where the brake holds the spindle
    spindle_raw: float | None = None
    limit: float = 0.0
    windup: float = 0.0
    steps: int = 0
    hazard: str = ""
    void: str = ""
    held: int = 0
    trim_step: int = -1  # the first step with the mark within the 1.0 band
    trim_turns: float = -1.0  # the handle turns that drove the drum, up to that step
    restored: bool = False
    restore_checked: bool = False
    score: float = 0.0
    last: dict[str, Any] = field(default_factory=dict)


S = State()


def _socket_writes(sock: Socket) -> None:
    rec = _record()
    path = rec["socket"]
    gf = importlib.import_module("pxr.Gf")
    if sock.engaged:
        set_attr(path, "physics:localRot0", gf.Quatf(*wxyz(rz(sock.offset) @ UP)))
    held = sock.latched or sock.blocked
    set_attr(path, "drive:transX:physics:stiffness", float(CATCH_K if held else 0.0))
    set_attr(path, "drive:transX:physics:targetPosition", float(round(sock.block_at, 5) if sock.blocked else 0.0))
    set_attr(path, "physics:jointEnabled", bool(sock.engaged))


def _holder_writes(h: Holder) -> None:
    rec = _record()
    path = rec["holder_socket"]
    gf = importlib.import_module("pxr.Gf")
    if h.engaged:
        set_attr(path, "physics:localRot0", gf.Quatf(*wxyz(rz(h.yaw) @ UP)))
    set_attr(path, "physics:jointEnabled", bool(h.engaged))


def step_holder(b: Bodies, h: Holder) -> None:
    rec = _record()
    hp, hr, _ = b[rec["handle"]]
    rise = float(hp[2] - HOLDER_TOP)
    if not h.engaged:
        if seated_in(hp, hr, HOLDER_XY, HOLDER_TOP, tol_z=0.004, tol_r=HOLDER_R, tol_tilt=HOLDER_TILT):
            h.engaged, h.yaw = True, yaw_of(hr)
            h.inserts += 1
    elif rise >= OUT_Z:
        h.engaged = False
        h.removals += 1
    _holder_writes(h)


ENTER_Z = SOCKET_DEPTH - 0.006  # m: the handle counts as in the socket once its frame is this far above the mouth
# (the lug's tip 6 mm in), on the axis within SEAT_R and upright within ENTER_TILT; a latch across stops it there
ENTER_TILT = 8.0  # deg (a lug leaning more than this jams in the bore's mouth before the latch would meet the lip)


def step_socket(b: Bodies, sock: Socket) -> dict[str, Any]:
    """The lock-in socket. Out: a lug that enters the socket with the lever held turned goes in freely; without, its
    latch catches on the splines (blocked). In: let go of the lever with the lug fully in, it latches; let go with it
    part way, the latch catches where it is. Held turned, a latched or blocked handle is free; lifted to OUT_Z free, it
    is out."""
    rec = _record()
    hp, hr, _ = b[rec["handle"]]
    _, sr, _ = b[rec["spindle"]]
    _, lr, _ = b[rec["lever"]]
    turn = lever_turn(hr, lr)
    rise = float(hp[2] - WINCH_TOP)
    if not sock.engaged:
        if seated_in(hp, hr, WINCH_XY, WINCH_TOP, tol_z=ENTER_Z, tol_tilt=ENTER_TILT):
            sock.engaged = True
            sock.offset = wrap_angle(yaw_of(hr) - yaw_of(sr))
            free = turn >= LEVER_FREE
            sock.latched = not free and rise <= SEAT_Z  # found fully in with the latch across (a rig's start): locked
            sock.blocked = not free and not sock.latched
            sock.block_at = max(rise, 0.0)
            sock.inserts += 1
    elif sock.blocked:
        if turn >= LEVER_FREE:
            sock.blocked = False
    elif sock.latched:
        sock.pulled_latched = max(sock.pulled_latched, rise)
        if turn >= LEVER_FREE:
            sock.latched = False
    elif turn < LEVER_LOCK:
        if rise <= SEAT_Z:
            sock.latched = True
        else:
            sock.blocked, sock.block_at = True, max(rise, 0.0)
    if sock.engaged and not (sock.latched or sock.blocked) and rise >= OUT_Z:
        sock.engaged = False
        sock.removals += 1
    _socket_writes(sock)
    windup = 0.0
    if sock.engaged:
        windup = WINDUP_K * wrap_angle(yaw_of(hr) - yaw_of(sr) - sock.offset)
    return {"lever_deg": round(turn, 2), "rise_mm": round(rise * 1000, 2), "windup": windup, "blocked": sock.blocked}


def step_brake(brake_angle: float, windup: float, limit: float) -> float:
    """The gear train's resistance, as a brake on the spindle: the spindle is held at the brake's angle, and each
    control step the brake gives way just enough that the windup spring's torque (the push the handle passes on) is no
    more than the limit for the pushed direction (the sheet's load through that gear). Under the limit nothing moves.
    While the handle turns, the spring winds up a little more between steps (about 1.2 N m at 0.5 rad/s: a drag that
    grows with speed, as a gear train's friction does); at rest the push it takes is the limit exactly."""
    if abs(windup) <= limit:
        return brake_angle
    return brake_angle + (windup - math.copysign(limit, windup)) / WINDUP_K


def step_mechanics(b: Bodies) -> dict[str, Any]:
    """The socket and lock, the gears and the drum, once per control step."""
    rec = _record()
    step_holder(b, S.holder)
    sock = step_socket(b, S.socket)
    _, sr, _ = b[rec["spindle"]]
    raw = yaw_of(sr)
    if S.spindle_raw is not None:
        S.spindle_angle += wrap_angle(raw - S.spindle_raw)
    S.spindle_raw = raw
    used = S.limit
    S.limit = S.winch.step(S.spindle_angle, float(sock["windup"]), used if S.steps > 1 else None)
    S.windup = float(sock["windup"])
    S.brake_angle = step_brake(S.brake_angle, S.windup, S.limit)
    set_attr(rec["brake"], "drive:rotX:physics:targetPosition", float(round(math.degrees(wrap_angle(S.brake_angle)), 4)))
    gf = importlib.import_module("pxr.Gf")
    s_in = S.winch.sheet.taken_in
    set_attr(rec["drum"], "xformOp:rotateZ", float(-math.degrees(S.winch.drum)))
    set_attr(rec["mark"], "xformOp:translate", gf.Vec3d(SHEET_X, mark_y(s_in), LOAD_Z))
    set_attr(rec["stopper"], "xformOp:translate", gf.Vec3d(SHEET_X, stopper_y(s_in), LOAD_Z))
    return {
        **sock,
        "engaged": S.socket.engaged,
        "in_holder": S.holder.engaged,
        "latched": S.socket.latched,
        "spindle_deg": round(math.degrees(S.spindle_angle), 2),
        "drum_deg": round(math.degrees(S.winch.drum), 2),
        "taken_in_mm": round(s_in * 1000, 2),
        "short_mm": round((S_REF - s_in) * 1000, 2),
        "tension": round(S.winch.sheet.tension(), 1),
        "limit": round(S.limit, 4),
        "brake_deg": round(math.degrees(S.brake_angle), 2),
        "fast_turns": round(S.winch.turned[FAST] / (2 * math.pi), 3),
        "slow_turns": round(S.winch.turned[SLOW] / (2 * math.pi), 3),
        "against_limit": S.winch.against_limit,
    }


# =====================================================================================================================
# Scoring (README "Score"; the numbers are fixed from GPU runs)
# =====================================================================================================================
FULL_BAND = 0.005  # m: the mark within this of the drum's edge (or past it): full trim
HALF_BAND = 0.030  # m: about a third of a second-gear turn short
QUARTER_BAND = 0.080  # m: about one second-gear turn short (past the first-gear stall at a moderate push)
TRIM_TURNS = 4.0  # handle turns (both directions, those that drove the drum) to full trim, at most, for full credit:
# first gear while it is easy, then second (see the README for the demo's count and second gear alone's)
HOLD_STEPS = 20  # the finish held for 1 s (20 control steps)
HAND_CLEAR = 0.030  # every finger-pad point at least this far from the handle's shapes
STILL_V = 0.010  # m/s: the handle, knob, lock lever and spindle all slower than this
DROP_SPEED = 1.0  # m/s: the handle falling this fast with no gripper on it and not in the winch or holder (a 51 mm
# free fall): a dropped winch handle can go overboard or land on a foot
PAD_TIP, PAD_LENGTH, PAD_WIDTH = 0.013, 0.030, 0.027


@dataclass(frozen=True)
class Gripper:
    tcp: np.ndarray
    opening: float
    approach: np.ndarray
    closing: np.ndarray

    def finger_points(self) -> list[np.ndarray]:
        """The TCP and points over both finger pads (tip, middle, back; both edges and the middle)."""
        across = np.cross(self.approach, self.closing)
        points = [np.asarray(self.tcp, dtype=float)]
        for finger_side in (-0.5, 0.5):
            for along in (PAD_TIP, PAD_TIP - PAD_LENGTH / 2, PAD_TIP - PAD_LENGTH):
                for edge in (-0.5, 0.0, 0.5):
                    points.append(self.tcp + finger_side * self.opening * self.closing + along * self.approach + edge * PAD_WIDTH * across)
        return points


def handle_shapes() -> list[tuple[np.ndarray, np.ndarray]]:
    """The handle's colliding shapes as boxes in its own frame (low, high corners), with the knob and the square the lock
    lever can sweep."""
    arm_x0 = BOSS_R - 0.004
    return [
        (np.array([-SPIGOT_R, -SPIGOT_R, -SOCKET_DEPTH]), np.array([SPIGOT_R, SPIGOT_R, 0.0])),
        (np.array([-BOSS_R, -BOSS_R, GAP]), np.array([BOSS_R, BOSS_R, BOSS_TOP])),
        (np.array([-LEVER_R - 0.008, -LEVER_R - 0.008, LEVER_Z]), np.array([LEVER_R + 0.008, LEVER_R + 0.008, LEVER_Z + LEVER_BAR[2] + LEVER_TAB[2]])),
        (np.array([arm_x0, -ARM_W / 2, ARM_Z0]), np.array([HANDLE_LENGTH, ARM_W / 2, ARM_Z1])),
        (np.array([HANDLE_LENGTH - KNOB_R, -KNOB_R, ARM_Z0]), np.array([HANDLE_LENGTH + KNOB_R, KNOB_R, KNOB_Z0 + KNOB_H])),
    ]


def box_gap(p: np.ndarray, pos: np.ndarray, rot: np.ndarray, box: tuple[np.ndarray, np.ndarray]) -> float:
    q = rot.T @ (p - pos)
    gap = np.maximum(np.maximum(box[0] - q, q - box[1]), 0.0)
    return float(np.linalg.norm(gap))


def handle_gap(p: np.ndarray, hp: np.ndarray, hr: np.ndarray) -> float:
    return min(box_gap(p, hp, hr, box) for box in handle_shapes())


def hands_clear(hp: np.ndarray, hr: np.ndarray, grippers: Sequence[Gripper]) -> bool:
    return all(handle_gap(p, hp, hr) >= HAND_CLEAR for g in grippers for p in g.finger_points())


def gripping(hp: np.ndarray, hr: np.ndarray, grippers: Sequence[Gripper]) -> bool:
    """Some gripper has the handle between its pads: its pads closer than 60 mm, and a point between them within 5 mm
    of the handle's shapes."""
    for g in grippers:
        if g.opening > 0.060:
            continue
        for along in (PAD_TIP, PAD_TIP - PAD_LENGTH / 2):
            if handle_gap(g.tcp + along * g.approach, hp, hr) <= 0.005:
                return True
    return False


@dataclass(frozen=True)
class Snapshot:
    short: float  # m: how far the mark is short of the drum's edge (negative: past it)
    in_winch: bool
    stowed: bool  # the handle stands in its holder, out of the winch
    still: bool
    clear: bool
    late: bool  # full trim not reached, or reached after more than TRIM_TURNS handle turns

    @property
    def tidy(self) -> bool:
        return self.stowed and self.still and self.clear


def handle_turns(w: Winch) -> float:
    """The handle turns, in both directions, that drove the drum so far (a real trial counts the knob's turns on the
    video while the sheet comes in)."""
    return (w.turned[FAST] + w.turned[SLOW]) / (2 * math.pi)


def grade(now: Snapshot, st: State) -> float:
    """The ladder (README, "Score"). 0 for the whole trial after the named hazard (the handle dropped), or if the
    trial's state could not be read."""
    if st.void or st.hazard:
        st.held = 0
        return 0.0
    st.held = st.held + 1 if (now.short <= FULL_BAND and now.tidy and not now.late) else 0
    if st.held >= HOLD_STEPS:
        return 1.0
    if now.short <= FULL_BAND:
        return 0.75
    rung = 0.5 if now.short <= HALF_BAND else 0.25 if now.short <= QUARTER_BAND else 0.0
    return rung if now.tidy else max(rung - 0.25, 0.0)


def in_holder(hp: np.ndarray, hr: np.ndarray) -> bool:
    return seated_in(hp, hr, HOLDER_XY, HOLDER_TOP, tol_z=0.004, tol_r=HOLDER_R, tol_tilt=HOLDER_TILT)


_last_objects: object = None
_embodiment: Any = None
_last_clock: float | None = None
_hooked = False
_warned: set[str] = set()
_ARMS = (("left_robot", "left"), ("right_robot", "right"))
RESET_TAG = "trim_sheet_two_speed_winch_resets"


def _warn_once(key: str, message: str) -> None:
    if key not in _warned:
        _warned.add(key)
        with contextlib.suppress(Exception):
            warnings.warn(f"trim_sheet_two_speed_winch: {message}", RuntimeWarning, stacklevel=3)


def new_episode() -> None:
    """A new trial: fresh memory, the parts back at their start, the socket joint off."""
    global S, _last_objects, _last_clock
    S = State()
    Live.last = None
    _last_objects = None
    _last_clock = None
    _warned.difference_update(("robot", "void", "restore"))
    if not _record().get("built"):
        return
    try:
        restore_bodies()
        S.restored = True
        _socket_writes(S.socket)
        _holder_writes(S.holder)
        set_attr(_record()["brake"], "drive:rotX:physics:targetPosition", 0.0)  # the brake back at the spindle's start
    except Exception as error:
        S.void = f"could not set the new trial up ({type(error).__name__}: {error})"
        _warn_once("restore", S.void)


def _calling_embodiment() -> Any:
    frame: FrameType | None = sys._getframe(2)
    while frame is not None:
        candidate = frame.f_locals.get("self")
        if candidate is not None and hasattr(candidate, "_t") and hasattr(candidate, "task") and callable(getattr(candidate, "reset", None)):
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
    _warn_once("reset", "could not hook the embodiment's reset, so the arms' state is unknown: every trial is void (0)")


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


def _update(objects: Any) -> float:
    global _last_objects
    if objects is _last_objects:
        return S.score
    _last_objects = objects
    with contextlib.suppress(Exception):
        _hook_resets()
    _restart_if_clock_went_back()
    S.steps += 1
    b = read_bodies()
    if b is None:
        if S.steps > 3 or Live.failed:
            S.void = S.void or f"the parts could not be read ({Live.failed or 'no view'})"
        S.score = 0.0
        return 0.0
    if not all(np.all(np.isfinite(p)) and np.all(np.isfinite(v)) for p, _, v in b.values()):
        S.void = S.void or "the simulation went non-finite"
    if not S.restore_checked and S.steps >= RESTORE_CHECK_STEP:
        S.restore_checked = True
        if not S.restored:  # the first trial: where its parts are now is the start
            Live.start = {path: np.asarray(b[path][0], dtype=float).copy() for path in b}
        else:
            off = restore_error(b)
            if off:
                S.void = S.void or f"a new trial's parts were not back at their start ({off})"
    try:
        m = step_mechanics(b)
    except Exception as error:
        S.void = S.void or f"the parts could not be read ({type(error).__name__}: {error})"
        S.score = 0.0
        return 0.0
    if Live.failed:
        S.void = S.void or f"a write failed ({Live.failed})"
    grippers = _robot()
    if grippers is None:
        S.void = S.void or "the arms' state could not be read"
    rec = _record()
    hp, hr, hv = b[rec["handle"]]
    short = S_REF - S.winch.sheet.taken_in
    if short <= FULL_BAND and S.trim_step < 0:
        S.trim_step = S.steps
        S.trim_turns = handle_turns(S.winch)
    holder = S.holder.engaged and not S.socket.engaged
    held = grippers is not None and gripping(hp, hr, grippers)
    if not (S.socket.engaged or holder or held) and float(-hv[2]) >= DROP_SPEED:
        S.hazard = S.hazard or f"the handle dropped ({-hv[2]:.2f} m/s down, no gripper on it)"
    still = all(float(np.linalg.norm(v[:3])) < STILL_V for _, _, v in b.values())
    late = S.trim_step < 0 or S.trim_turns > TRIM_TURNS
    now = Snapshot(short, S.socket.engaged, holder, still, grippers is not None and hands_clear(hp, hr, grippers), late)
    if S.void:
        _warn_once("void", f"this trial is void (0): {S.void}")
    S.score = grade(now, S)
    S.last = {**m, "score": S.score, "held": S.held, "void": S.void, "hazard": S.hazard, "stowed": holder, "gripping": held,
              "still": still, "clear": now.clear, "late": late, "trim_t": round(S.trim_step * DT, 2) if S.trim_step >= 0 else None,
              "trim_turns": round(S.trim_turns, 3) if S.trim_step >= 0 else None}
    return S.score


def grippers_of(embodiment: Any) -> list[Gripper]:
    ik = importlib.import_module("yam_arena.ik")
    station = importlib.import_module("yam_arena.station")
    out = []
    for name, side in _ARMS:
        arm_ids, finger_ids = embodiment._ids[name]
        joints = embodiment._base.scene[name].data.joint_pos
        q = _np(getattr(joints, "torch", joints)[0])
        tcp, rot = ik.tcp_pose(q[list(arm_ids)], station.BASES[side])
        rot = np.asarray(rot, dtype=float)
        opening = float(-2.0 * q[list(finger_ids)].mean())
        if not (np.all(np.isfinite(tcp)) and np.all(np.isfinite(rot)) and math.isfinite(opening)):
            raise ValueError(f"{side} arm: not finite")
        out.append(Gripper(np.asarray(tcp, dtype=float), opening, approach=rot[:, 2].copy(), closing=rot[:, 0].copy()))
    return out


def _robot() -> list[Gripper] | None:
    if _embodiment is None:
        return None
    try:
        return grippers_of(_embodiment)
    except Exception as error:
        _warn_once("robot", f"can't read the arms' state ({type(error).__name__}: {error}): this trial is void (0)")
        return None


def score(objects: Any) -> float:
    """1.0: the mark at the drum's edge (within 5 mm, or past it) within 4 handle turns, the handle back in its holder, hands
    clear and all still for 1 s; 0.75 the mark there but late or the handle not stowed; 0.5 / 0.25 by distance short with
    a tidy finish (one rung lower if untidy); see README.md."""
    return _update(objects)


def success(objects: Any) -> bool:
    return _update(objects) >= 1.0


# =====================================================================================================================
# The scripted demo (both arms: the left cranks and carries, the right works the lock lever; its live targets read the
# handle's, knob's and lever's poses, which the rules
# allow for a scripted demo; a real robot would take the knob's place from its own joint encoders while it holds it)
# =====================================================================================================================
ARM = "left"
KJ = (80.0, 80.0, 80.0, 10.0, 10.0, 10.0)  # N m/rad: the official joint stiffness (joints 1-3, 4-6)
GRIP_H = 0.200  # m: the TCP on the knob's axis this far above the bench (the reach screen)
GRIP_TILT = 60.0  # deg: the approach tilted this far from straight down (the reach screen: 50 fails, 70 leaves 6 deg)
GRIP_TURN = 270.0  # deg: the closing axis's heading = the bearing from the arm's base + this (the reach screen)
KNOB_GAP = 0.016  # the pads' target gap on the 32 mm knob (8 mm a side into it: about 40 N a pad at 5,000 N/m)
KNOB_OPEN = 0.064
KNOB_BACK = 0.060  # pre-grasp: this far back along the approach
PSI_INS = math.radians(150.0)  # the handle's heading when the robot puts it in (its knob nearest the arm)
F_FAST = 22.0  # N: the push the servo allows in first gear before it calls it too hard (the shift)
F_SLOW = 22.0  # N: the push it allows in second gear (over F_REF = 18 N: it reaches the stopper)
OMEGA = 2 * math.pi / 12.0  # rad/s: the hand turns the knob at most this fast (12 s a turn)
LAM_MIN = 0.010  # m: the hand's lead on the knob along the circle's tangent (smallest)
LAM_UP, LAM_DOWN = 0.003, 0.0015  # m per control step
LAM_CAP = 0.200  # m: never more (a GPU test: a 37 deg lead along the circle pulled the knob out of the fingertips)
LAM_CUT = 0.8  # the lead is cut by this factor each step it is over the cap (a GPU test: a 118 mm lead built on the soft
# far side whipped the knob round 120 deg in a second at 37 N as it reached the stiff side)
STALL_WINDOW, STALL_DEG = 2.0, 3.0  # s, deg: stalled when the knob turned less than this over the window at full push (a GPU test: a stalled knob still creeps about 1 deg/s in the compliant grip)
STALL_F = 0.85  # full push: at least this share of the gear's cap
CRANK_S = 60.0  # s: the time the plan allows for the whole grind (a GPU test: 47.6 s; the servo stops early and holds)
# --- the hub pinch (the left hand lifts and carries the handle by its hub): the knob grip's family -------------------
PINCH_X, PINCH_Z = 0.0, 0.040  # the TCP on the socket's axis 40 mm up the hub: the pads (31 mm tall where they meet the
# hub) then clear the arm's root (its top 22 mm up), on the side the arm leaves the hub
PINCH_OPEN, PINCH_GAP = 0.060, 0.020  # round the 36 mm hub; closed 8 mm a side into it (about 40 N a pad)
# --- the lever hand (the right hand turns and holds the lock lever) --------------------------------------------------
LEVER_HAND = "right"
LEVER_TILT = 45.0  # deg: its approach tilted this far from straight down, pointing along the lever tab's travel
LEVER_BACK = 0.008  # m: its fingertips start this far behind the tab (on the side it is pushed from)
LEVER_OVER = 0.020  # m: and are sent this far past the tab's fully turned place (the arm's give holds it on its stop)
LIFT = 0.050  # m: lifted this far to clear the socket (25 mm deep)
SAG = 0.060  # m: the hand is sent this much higher than the handle should go when it lifts it (a GPU test: a 65 mm lift
# target barely moved the hand against the handle's weight; the arm gives about 350 N/m)
HOVER = 0.010  # m: the spigot this far over the socket while it is lined up
UPRIGHT_DEADBAND, UPRIGHT_STEP, UPRIGHT_MAX = 0.5, 1.0, 15.0  # deg: the holder line-up's lean correction (see InsertServo)
PUSH_IN = 0.015  # m: the target this far below seated (the arm's give pushes it home)
ZUP = np.array([0.0, 0.0, 1.0])


def knob_tcp(psi: float) -> np.ndarray:
    return np.array([WINCH_XY[0] + HANDLE_LENGTH * math.cos(psi), WINCH_XY[1] + HANDLE_LENGTH * math.sin(psi), TOP + GRIP_H])


def _rot_about(axis: np.ndarray, angle: float) -> np.ndarray:
    k = axis / float(np.linalg.norm(axis))
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    out: np.ndarray = np.eye(3) + math.sin(angle) * kx + (1 - math.cos(angle)) * (kx @ kx)
    return out


def base_of(side: str = ARM) -> np.ndarray:
    station = importlib.import_module("yam_arena.station")
    return np.asarray(station.BASES[side], dtype=float)


def knob_rot(p: np.ndarray, twin: bool) -> np.ndarray:
    """The knob grip (from the reach screen): closing axis level, heading the bearing from the arm's base + GRIP_TURN; the approach
    tilted GRIP_TILT from straight down about it. The knob spins on the handle, so the heading may change round the
    circle at no cost. twin: the same grip with the closing axis reversed."""
    b = base_of()
    h = math.atan2(float(p[1] - b[1]), float(p[0] - b[0])) + math.radians(GRIP_TURN)
    c = np.array([math.cos(h), math.sin(h), 0.0])
    a = _rot_about(c, math.radians(GRIP_TILT)) @ np.array([0.0, 0.0, -1.0])
    r = np.column_stack([c, np.cross(a, c), a])
    return r @ np.diag([-1.0, -1.0, 1.0]) if twin else r


@dataclass
class CircleModel:
    """The arm on the knob's circle, from the official IK at plan time: which grip twin reaches all round, and the
    arm's stiffness along the circle's tangent at 72 angles (official joint stiffness, J K^-1 J^T)."""

    twin: bool = False
    psi: np.ndarray = field(default_factory=lambda: np.zeros(0))
    k_t: np.ndarray = field(default_factory=lambda: np.zeros(0))
    worst_mm: float = 0.0

    def stiffness(self, psi: float) -> float:
        if len(self.psi) == 0:
            return 300.0
        return float(np.interp(psi % (2 * math.pi), self.psi, self.k_t, period=2 * math.pi))


def _rotvec(r: np.ndarray) -> np.ndarray:
    c = float(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))
    a = math.acos(c)
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return w / 2.0 if a < 1e-6 else w * (a / (2.0 * math.sin(a)))


def circle_model() -> CircleModel:
    ik = importlib.import_module("yam_arena.ik")
    base = base_of()
    best: CircleModel | None = None
    for twin in (False, True):
        q = np.asarray(READY, dtype=float)
        psis, kts, worst = [], [], 0.0
        for i in range(72):
            psi = 2 * math.pi * i / 72
            p = knob_tcp(psi)
            r = knob_rot(p, twin)
            for _ in range(2 if i == 0 else 1):
                q, pe, _ = ik.solve(p, ik.matrix_to_quat_xyzw(r), q, base_pos=base, iters=150)
            worst = max(worst, float(pe) * 1000)
            p0, r0 = ik.tcp_pose(q, base)
            jac = np.zeros((6, 6))
            for j in range(6):
                dq = np.zeros(6)
                dq[j] = 1e-5
                p1, r1 = ik.tcp_pose(q + dq, base)
                jac[:3, j] = (np.asarray(p1) - np.asarray(p0)) / 1e-5
                jac[3:, j] = _rotvec(np.asarray(r1) @ np.asarray(r0).T) / 1e-5
            comp = jac @ np.diag(1.0 / np.asarray(KJ)) @ jac.T
            t = np.array([-math.sin(psi), math.cos(psi), 0.0])
            psis.append(psi)
            kts.append(1.0 / float(t @ comp[:3, :3] @ t))
        m = CircleModel(twin, np.array(psis), np.array(kts), worst)
        if best is None or m.worst_mm < best.worst_mm:
            best = m
    assert best is not None
    return best


class Grip(float):
    """A Waypoint yaw that carries a full TCP rotation (3x3: closing axis, across, approach), fixed or live."""

    _rot: np.ndarray | Callable[[], np.ndarray]

    def __new__(cls, rot: np.ndarray | Callable[[], np.ndarray]) -> "Grip":
        obj = super().__new__(cls, 0.0)
        obj._rot = rot
        return obj

    @property
    def rot(self) -> np.ndarray:
        r = self._rot
        return np.asarray(r() if callable(r) else r, dtype=float)


GRIP_TAG = "trim_sheet_two_speed_winch_grip"


def install_grip_rotation() -> None:
    demo_mod: Any = importlib.import_module("yam_arena.demo")
    original = demo_mod.grip_rotation
    if getattr(original, GRIP_TAG, None) is not None:
        return

    def grip_rotation(yaw: float = 0.0) -> np.ndarray:
        rot = getattr(yaw, "rot", None)
        return np.array(rot, dtype=float) if rot is not None else original(yaw)

    setattr(grip_rotation, GRIP_TAG, original)
    demo_mod.grip_rotation = grip_rotation


def down_rot(heading: float) -> np.ndarray:
    """A straight-down grip closing along `heading` (the twin nearer the official grip's own, for joint 6)."""
    demo_mod: Any = importlib.import_module("yam_arena.demo")
    original = getattr(demo_mod.grip_rotation, GRIP_TAG, demo_mod.grip_rotation)
    yaw = wrap_angle(heading - math.pi / 2)
    if yaw > math.pi / 2:
        yaw -= math.pi
    elif yaw < -math.pi / 2:
        yaw += math.pi
    return np.asarray(original(yaw), dtype=float)


class LivePos:
    """A waypoint position worked out every control step."""

    def __init__(self, fn: Callable[[], np.ndarray]) -> None:
        self.fn = fn

    def __array__(self, dtype: Any = None, copy: Any = None) -> np.ndarray:
        return np.asarray(self.fn(), dtype=dtype)

    def __len__(self) -> int:
        return 3

    def __getitem__(self, index: Any) -> Any:
        return tuple(float(v) for v in self.fn())[index]

    def __iter__(self) -> Any:
        return iter(tuple(float(v) for v in self.fn()))


def finger(gap: float) -> float:
    """Finger joint target for a pad gap (m): 0 closed (1.1 mm), FINGER_OPEN = 95 mm."""
    return float(np.clip(-(gap - 0.0011) / (0.095 - 0.0011) * abs(FINGER_OPEN), FINGER_OPEN, 0.0))


def _phases(plan: Sequence[Waypoint]) -> list[tuple[float, float, str]]:
    out, start = [], 0.0
    for w in plan:
        out.append((start, start + float(w.duration), str(w.label)))
        start += float(w.duration)
    return out


def _now_label(phases: Sequence[tuple[float, float, str]]) -> str:
    t = S.steps * DT
    return next((lab for t0, t1, lab in phases if t0 <= t < t1), "")


def handle_now() -> tuple[np.ndarray, np.ndarray] | None:
    b = Live.last
    rec = _record()
    if b is None or rec.get("handle") not in b:
        return None
    hp, hr, _ = b[rec["handle"]]
    if not (np.all(np.isfinite(hp)) and np.all(np.isfinite(hr))):
        return None
    return np.asarray(hp, dtype=float), np.asarray(hr, dtype=float)


def knob_angle_now() -> float | None:
    b = Live.last
    rec = _record()
    if b is None or rec.get("knob") not in b:
        return None
    kp = b[rec["knob"]][0]
    if not np.all(np.isfinite(kp)):
        return None
    return math.atan2(float(kp[1] - WINCH_XY[1]), float(kp[0] - WINCH_XY[0]))


class CrankServo:
    """Turns the knob round its circle. Each control step the hand's target is set `lead` ahead of the knob along the
    circle; the lead grows while the knob turns slower than OMEGA and shrinks when faster or when the push it reads at the
    knob is over the gear's cap. First gear (clockwise) until the knob stalls at full push (too
    hard), then reverse: second gear (anticlockwise) until it stalls again (the sheet will come in no further); then
    the hand holds the knob still (the pawls hold the drum). `lam` is the lead in metres along the tangent."""

    def __init__(self, model: CircleModel, gears: Sequence[int] = (FAST, SLOW)) -> None:
        self.model = model
        self.gears = tuple(gears)
        self.phases: list[tuple[float, float, str]] = []
        self.restart()

    def restart(self) -> None:
        self.tick = -1
        self.mode = "idle"
        self.gear_i = 0
        self.lam = LAM_MIN
        self.unwrapped: float | None = None
        self.raw: float | None = None
        self.hist: list[float] = []
        self.mode_steps = 0
        self.cmd = PSI_INS
        self.push = 0.0  # m along the tangent (anticlockwise positive) the hand is sent past the knob
        self.force = 0.0  # N: the push on the knob it reads
        self.events: list[dict[str, Any]] = []

    def follow(self, plan: Sequence[Waypoint]) -> None:
        self.phases = _phases(plan)

    def _dir(self) -> int:
        gear = self.gears[min(self.gear_i, len(self.gears) - 1)]
        return -1 if (gear == FAST) == FAST_IS_CLOCKWISE else 1

    def _cap(self) -> float:
        gear = self.gears[min(self.gear_i, len(self.gears) - 1)]
        return F_FAST if gear == FAST else F_SLOW

    def update(self) -> None:
        if S.steps == self.tick:
            return
        if S.steps < self.tick:
            self.restart()
        self.tick = S.steps
        psi = knob_angle_now()
        if psi is None:
            return
        if self.raw is None or self.unwrapped is None:
            self.unwrapped = psi
        else:
            self.unwrapped += wrap_angle(psi - self.raw)
        self.raw = psi
        label = _now_label(self.phases)
        if label != "grind the sheet in" or self.mode == "done":
            self.cmd = self.unwrapped
            self.lam = LAM_MIN
            self.push = 0.0
            return
        if self.mode == "idle":
            self.mode, self.mode_steps, self.hist = "crank", 0, []
        d = self._dir()
        self.hist.append(self.unwrapped)
        self.mode_steps += 1
        n = max(1, int(0.5 / DT))
        rate = (self.hist[-1] - self.hist[-1 - n]) / (n * DT) * d if len(self.hist) > n else 0.0
        # the push the hand feels at the knob (the handle's torque over its length; a real robot reads it from its
        # motor currents): the lead grows while the knob is slower than OMEGA and the push under the gear's cap
        self.force = abs(S.windup) / HANDLE_LENGTH if S.socket.engaged else 0.0
        if self.force > self._cap() or rate > 1.5 * OMEGA:
            self.lam *= LAM_CUT  # over the cap, or the knob running away (the arm's stiffer side): back off fast
        elif rate < OMEGA:
            self.lam += LAM_UP
        else:
            self.lam -= LAM_DOWN
        self.lam = float(np.clip(self.lam, LAM_MIN, LAM_CAP))
        self.cmd = self.unwrapped
        self.push = d * self.lam
        w = int(STALL_WINDOW / DT)
        if self.mode_steps > w + 20 and abs(self.hist[-1] - self.hist[-1 - w]) < math.radians(STALL_DEG) and self.force >= STALL_F * self._cap():
            self.events.append({"t": round(S.steps * DT, 2), "gear": "fast" if self.gears[self.gear_i] == FAST else "slow",
                                "taken_in_mm": round(S.winch.sheet.taken_in * 1000, 1), "psi_deg": round(math.degrees(self.unwrapped), 1)})
            self.gear_i += 1
            self.lam, self.hist, self.mode_steps = LAM_MIN, [], 0
            self.push = 0.0
            if self.gear_i >= len(self.gears):
                self.mode = "done"
                self.cmd = self.unwrapped

    def target(self) -> np.ndarray:
        """The knob's place, then `push` along the circle's tangent there (the hand pushes the knob along its path; a
        target on the circle ahead would pull it inward, out of the fingertips)."""
        self.update()
        t = np.array([-math.sin(self.cmd), math.cos(self.cmd), 0.0])
        return knob_tcp(self.cmd) + t * self.push

    def rot(self) -> np.ndarray:
        self.update()
        return knob_rot(self.target(), self.model.twin)

    def report(self) -> dict[str, Any]:
        return {"crank": self.mode, "gear_i": self.gear_i, "lead_mm": round(self.push * 1000, 1),
                "cmd_deg": round(math.degrees(self.cmd), 2), "knob_deg": round(math.degrees(self.unwrapped or 0.0), 2),
                "k_t": round(self.model.stiffness(self.cmd), 1), "force_n": round(self.force, 2), "events": self.events}


class InsertServo:
    """Carries the handle to a socket and lines its spigot up there. While the hand carries it ('carry' phase) and holds
    it over the socket ('line up' phase) the target is shifted up by how far the handle hangs below where it should be
    (the arm gives under its weight), and across by the spigot's offset from the socket's axis; the shifts are kept for
    the push in. With `upright`, the line-up also turns the grip by the handle's lean in the pinch (read from its pose)
    until it stands upright over the socket, and keeps that turn for the push in."""

    def __init__(self, centre: Sequence[float], top: float, label: str, carry: str, upright: bool = False) -> None:
        self.centre = np.asarray(centre, dtype=float)
        self.top = float(top)
        self.label = label
        self.carry = carry
        self.upright = upright
        self.phases: list[tuple[float, float, str]] = []
        self.restart()

    def restart(self) -> None:
        self.tick = -1
        self.shift = np.zeros(2)
        self.dz = SAG
        self.fix = np.eye(3)

    def rot(self, base: np.ndarray) -> np.ndarray:
        """The grip rotation `base`, turned by the lean correction so far."""
        self.update()
        out: np.ndarray = self.fix @ np.asarray(base, dtype=float)
        return out

    def follow(self, plan: Sequence[Waypoint]) -> None:
        self.phases = _phases(plan)

    def update(self) -> None:
        if S.steps == self.tick:
            return
        if S.steps < self.tick:
            self.restart()
        self.tick = S.steps
        label = _now_label(self.phases)
        if label not in (self.label, self.carry):
            return
        h = handle_now()
        if h is None:
            return
        tip = h[0] + h[1] @ np.array([0.0, 0.0, -SOCKET_DEPTH])
        want = self.top + (HOVER if label == self.label else LIFT - SOCKET_DEPTH)
        self.dz = float(np.clip(self.dz + np.clip(0.3 * (want - float(tip[2])), -0.003, 0.003), 0.0, 0.150))
        if label == self.label:
            err = self.centre - tip[:2]
            self.shift = np.clip(self.shift + np.clip(0.3 * err, -0.002, 0.002), -0.020, 0.020)
            if self.upright:
                hz = h[1][:, 2]
                axis = np.cross(hz, np.array([0.0, 0.0, 1.0]))
                lean = math.asin(min(float(np.linalg.norm(axis)), 1.0))
                done = math.acos(float(np.clip((np.trace(self.fix) - 1.0) / 2.0, -1.0, 1.0)))
                if lean > math.radians(UPRIGHT_DEADBAND) and done < math.radians(UPRIGHT_MAX):
                    step = min(0.3 * lean, math.radians(UPRIGHT_STEP))
                    self.fix = _rot_about(axis, step) @ self.fix

    def at(self, p: np.ndarray, lift: bool = True) -> np.ndarray:
        """p shifted across by the spigot's offset, and up by the sag allowance if `lift`."""
        self.update()
        out: np.ndarray = np.asarray(p, dtype=float) + np.r_[self.shift, self.dz if lift else 0.0]
        return out


@dataclass(frozen=True)
class Options:
    """The routine's choices (the demo uses the defaults; the negatives and the gate rigs change them)."""

    active: bool = True
    start_seated: bool = False  # the handle starts in the winch (a gate rig places it): skip taking it from the holder
    gears: tuple[int, ...] = (FAST, SLOW)  # the grind's gears in order
    remove: bool = True  # take the handle out and put it back
    lever: bool = True  # False: the lever is not turned for the removal (the left hand pulls; the lock must hold)
    lever_early: bool = False  # True: the lever is turned and let go before the left hand pulls (the lock must hold)
    drop: bool = False  # True: the handle is let go 150 mm up after it comes out (it falls)
    crank_s: float = CRANK_S


SERVOS: dict[str, Any] = {}
MODEL: CircleModel | None = None


def pinch_at(hp: np.ndarray, heading: float) -> np.ndarray:
    out: np.ndarray = np.asarray(hp, dtype=float) + rz(heading) @ np.array([PINCH_X, 0.0, PINCH_Z])
    return out


def pinch_rot(p: np.ndarray) -> np.ndarray:
    """The hub pinch: the knob grip's family (closing level across the bearing from the arm's base, approach tilted
    GRIP_TILT from straight down), so its reach does not depend on where the handle points."""
    return knob_rot(p, MODEL.twin if MODEL is not None else False)


def bearing(p: Sequence[float] | np.ndarray) -> float:
    b = base_of()
    return math.atan2(float(p[1] - b[1]), float(p[0] - b[0]))


def carried_heading(heading: float, p_from: Sequence[float] | np.ndarray, p_to: Sequence[float] | np.ndarray) -> float:
    """The handle's heading after the hub pinch carries it from p_from to p_to: the pinch's closing axis follows the
    bearing from the arm's base, so the handle turns with it."""
    return wrap_angle(heading + bearing(p_to) - bearing(p_from))


def lever_frame(hp: np.ndarray, hr: np.ndarray, turn_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """The lever tab's middle (at push height) and the direction it moves when turned, for the handle at (hp, hr) and the
    lever turned turn_deg."""
    h = yaw_of(hr) + LEVER_DIR + math.radians(turn_deg)
    centre = np.asarray(hp, dtype=float) + np.asarray(hr, dtype=float) @ np.array([0.0, 0.0, LEVER_PUSH_Z])
    tab = centre + LEVER_R * np.array([math.cos(h), math.sin(h), 0.0])
    return tab, np.array([-math.sin(h), math.cos(h), 0.0])


def lever_hand_rot(p: np.ndarray) -> np.ndarray:
    """The lever hand's grip (the reach screen's family on the right arm): closing axis level across its bearing,
    approach tilted LEVER_TILT from straight down toward the bearing, so the closed fingertips poke forward and down."""
    b = base_of(LEVER_HAND)
    h = math.atan2(float(p[1] - b[1]), float(p[0] - b[0])) + math.radians(GRIP_TURN)
    c = np.array([math.cos(h), math.sin(h), 0.0])
    a = _rot_about(c, math.radians(LEVER_TILT)) @ np.array([0.0, 0.0, -1.0])
    return np.column_stack([c, np.cross(a, c), a])


def lever_now() -> float | None:
    """The lock lever's turn now (deg), from the last step's readings."""
    b = Live.last
    rec = _record()
    if b is None or rec.get("lever") not in b or rec.get("handle") not in b:
        return None
    return lever_turn(b[rec["handle"]][1], b[rec["lever"]][1])


LEVER_LAM0, LEVER_LAM_UP, LEVER_LAM_DOWN, LEVER_LAM_CAP = 0.015, 0.0015, 0.0010, 0.090  # m, m a step, m a step, m
LEVER_WANT = 27.0  # deg: the push grows until the lever is turned this far (free at LEVER_FREE, stop at LEVER_MAX)
LEVER_EASE = 29.0  # deg: and eases off past this (so the hand presses the lever's spring, not its stop)


class LeverHand:
    """The right hand on the lock lever, worked out each step from the handle's and the lever's places (the simulator's
    state, which the rules allow for a scripted demo). The fingertips go behind the tab, then push it along its arc: the
    target is the tab's place plus a push along its travel that grows (the arm is soft, 110-540 N/m) until the lever is
    turned LEVER_WANT and eases off past LEVER_EASE, so the hand presses about the spring's 5 N. Frozen once it lets
    go."""

    def __init__(self, fallback: tuple[np.ndarray, np.ndarray]) -> None:
        self.fallback = fallback
        self.phases: list[tuple[float, float, str]] = []
        self.restart()

    def restart(self) -> None:
        self.tick = -1
        self.lam = LEVER_LAM0
        self.frozen: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.max_turn = 0.0

    def follow(self, plan: Sequence[Waypoint]) -> None:
        self.phases = _phases(plan)

    def _pose(self) -> tuple[np.ndarray, np.ndarray]:
        h = handle_now()
        return h if h is not None else self.fallback

    def target(self, mode: str) -> tuple[np.ndarray, np.ndarray]:
        """(TCP, rotation) for a mode: 'over' (40 mm above the start), 'start' (fingertips behind the tab), 'push' (along
        its arc, the push servoed) or 'away' (backed off and up from where it let go)."""
        if S.steps < self.tick:
            self.restart()
        new_tick = S.steps != self.tick
        self.tick = S.steps
        if mode == "away" and "away" in self.frozen:
            return self.frozen["away"]
        hp, hr = self._pose()
        turn = lever_now()
        if mode in ("push", "away"):
            now = float(np.clip(turn if turn is not None else 0.0, 0.0, LEVER_MAX))
            self.max_turn = max(self.max_turn, now)
            if mode == "push" and new_tick and now < LEVER_WANT:
                self.lam = min(self.lam + LEVER_LAM_UP, LEVER_LAM_CAP)
            elif mode == "push" and new_tick and now > LEVER_EASE:
                self.lam = max(self.lam - LEVER_LAM_DOWN, LEVER_LAM0)
            tab, t = lever_frame(hp, hr, now)
            contact = tab - t * (LEVER_TAB[1] / 2) + t * (self.lam if mode == "push" else -0.030)
        else:
            tab, t = lever_frame(hp, hr, 0.0)
            contact = tab - t * (LEVER_TAB[1] / 2 + LEVER_BACK)
        r = lever_hand_rot(contact)
        tcp = contact - r[:, 2] * PAD_TIP
        if mode == "over":
            tcp = tcp + ZUP * 0.040
        if mode == "away":
            self.frozen["away"] = (tcp + ZUP * 0.060, r)
            return self.frozen["away"]
        return tcp, r


LEVER_LEAD = 2.5  # s: the right hand starts turning the lever this long before the left hand moves the handle
HIGH_Z = WINCH_TOP + 0.150  # m: the right hand crosses over the handle's knob (its top 119 mm over the winch) this high
LEVER_APPROACH = 6.0  # s: from the right hand's ready pose to behind the lever's tab
RIGHT_VIA: tuple[float, float, float] | None = (0.50, -0.20, 0.12)  # (x, y, z over the winch's top): a point the
# right hand passes on its way across, out beyond the winch, so its arm stays clear of the knob (the knob stands between
# the right arm's base and the hub; planner check: the arm's link axes 80 mm or more from the knob)


def lever_waypoints(hand: LeverHand, t_now: float, t_move: float, t_release: float, label: str) -> list[Waypoint]:
    """The right hand from its ready pose (at t_now, s into the plan): up, across high over the handle's knob, down over
    the lever, behind the tab LEVER_LEAD before t_move (when the left hand starts moving the handle), the lever turned by
    t_move, held until t_release, then let go, up, and back across high to its ready pose."""
    rpark, rrot = ready_pose(LEVER_HAND)
    rhigh = np.array([rpark[0], rpark[1], max(float(rpark[2]), HIGH_Z)])
    closed = finger(0.0011)
    t_start = t_move - LEVER_LEAD  # behind the tab from here

    def high(mode: str) -> Callable[[], np.ndarray]:
        def fn() -> np.ndarray:
            p = hand.target(mode)[0]
            out: np.ndarray = np.array([p[0], p[1], max(float(p[2]), HIGH_Z)])
            return out

        return fn

    g_over = Grip(lambda: hand.target("over")[1])
    g_away = Grip(lambda: hand.target("away")[1])
    out = [Waypoint(max(t_start - LEVER_APPROACH - t_now, 0.2), tuple(rpark), FINGER_OPEN, "hold still", Grip(rrot))]
    via = [] if RIGHT_VIA is None else [np.array([RIGHT_VIA[0], RIGHT_VIA[1], WINCH_TOP + RIGHT_VIA[2]])]
    out += [Waypoint(1.5, tuple(rhigh), closed, f"up ({label})", Grip(rrot))]
    out += [Waypoint(1.2, tuple(v), closed, f"across over the knob ({label})", Grip(rrot)) for v in via]
    out += [
        Waypoint(2.5 - 1.2 * len(via), LivePos(high("over")), closed, f"across over the knob ({label})", g_over),
        Waypoint(2.0, LivePos(lambda: hand.target("over")[0]), closed, f"over the lever ({label})", g_over),
        Waypoint(1.0, LivePos(lambda: hand.target("start")[0]), closed, f"behind the lever's tab ({label})", Grip(lambda: hand.target("start")[1])),
        Waypoint(1.5, LivePos(lambda: hand.target("push")[0]), closed, f"turn the lever ({label})", Grip(lambda: hand.target("push")[1])),
    ]
    t = t_now + sum(w.duration for w in out)
    out.append(Waypoint(max(t_release - t, 0.3), LivePos(lambda: hand.target("push")[0]), closed, f"hold the lever turned ({label})",
                        Grip(lambda: hand.target("push")[1])))
    out.append(Waypoint(1.0, LivePos(lambda: hand.target("away")[0]), closed, f"let go of the lever ({label})", g_away))
    out.append(Waypoint(1.0, LivePos(high("away")), closed, f"up off the lever ({label})", g_away))
    out += [Waypoint(1.0, tuple(v), closed, f"back across over the knob ({label})", Grip(rrot)) for v in via]
    out.append(Waypoint(2.0 - 1.0 * len(via), tuple(rhigh), closed, f"back across over the knob ({label})", Grip(rrot)))
    out.append(Waypoint(1.5, tuple(rpark), FINGER_OPEN, "back to ready", Grip(rrot)))
    return out


def removal_heading() -> float:
    h = handle_now()
    return yaw_of(h[1]) if h is not None else PSI_INS


def routine(o: Options | None = None) -> dict[str, list[Waypoint]]:
    global MODEL
    o = o or Options()
    install_grip_rotation()
    park, park_rot = ready_pose(ARM)
    if not o.active:
        return {side: [Waypoint(5.0, tuple(ready_pose(side)[0]), FINGER_OPEN, "hold still")] for side in ("left", "right")}
    MODEL = circle_model()
    crank = CrankServo(MODEL, o.gears)
    seat_w = np.array([WINCH_XY[0], WINCH_XY[1], WINCH_TOP])
    seat_h = np.array([HOLDER_XY[0], HOLDER_XY[1], HOLDER_TOP])
    ins_w = InsertServo(WINCH_XY, WINCH_TOP, "line it up over the socket", "carry it over the winch")
    ins_h = InsertServo(HOLDER_XY, HOLDER_TOP, "line it up over the holder", "carry it to the holder", upright=True)
    pinch_open, pinch, pinch_loose = finger(PINCH_OPEN), finger(PINCH_GAP), finger(PINCH_GAP + 0.020)
    out: list[Waypoint] = []

    def on_hub() -> np.ndarray:
        """The hub pinch's TCP on the handle where it is now (the hand follows the handle while it is loose)."""
        h = handle_now()
        if h is None:
            return np.asarray(ins_w.at(at_w - ZUP * PUSH_IN, lift=False), dtype=float)
        out_p: np.ndarray = h[0] + h[1] @ np.array([PINCH_X, 0.0, PINCH_Z])
        return out_p

    psi_ins = PSI_INS if o.start_seated else carried_heading(HOLDER_HEADING, seat_h, seat_w)
    g_ins = Grip(pinch_rot(pinch_at(seat_w, psi_ins)))
    if not o.start_seated:
        g_h = Grip(pinch_rot(pinch_at(seat_h, HOLDER_HEADING)))
        at_h = pinch_at(seat_h, HOLDER_HEADING)
        at_w = pinch_at(seat_w, psi_ins)
        lift = ZUP * (LIFT + SAG)
        out += [
            Waypoint(3.0, tuple(at_h + ZUP * 0.060), pinch_open, "over the handle in its holder", g_h),
            Waypoint(1.5, tuple(at_h), pinch_open, "down round the hub", g_h),
            Waypoint(1.0, tuple(at_h), pinch, "pinch the hub", g_h),
            Waypoint(1.5, tuple(at_h + lift), pinch, "lift it out of the holder", g_h),
            Waypoint(3.0, LivePos(lambda: ins_w.at(at_w + ZUP * LIFT)), pinch, "carry it over the winch", g_ins),
            Waypoint(1.5, LivePos(lambda: ins_w.at(at_w + ZUP * HOVER)), pinch, "line it up over the socket", g_ins),
            Waypoint(1.5, LivePos(lambda: ins_w.at(at_w - ZUP * PUSH_IN, lift=False)), pinch, "set it on the socket (the latch stops it)", g_ins),
            # the socket now holds it upright and across, so the lever's push cannot shove it off the axis: the left hand
            # loosens its pinch and stays round the hub; the right hand sets off only now, when the knob it crosses
            # over has stopped moving
            Waypoint(0.8, LivePos(on_hub), pinch_loose, "loosen the pinch: the socket holds it", g_ins),
            Waypoint(LEVER_APPROACH + LEVER_LEAD - 0.8, LivePos(on_hub), pinch_loose, "hold it there while the lever is turned", g_ins),
        ]
        t_move_in = sum(w.duration for w in out)
        out += [
            Waypoint(1.5, LivePos(lambda: ins_w.at(at_w - ZUP * PUSH_IN, lift=False)), pinch, "pinch it and push it into the socket", g_ins),
            Waypoint(1.5, LivePos(lambda: ins_w.at(at_w - ZUP * PUSH_IN, lift=False)), pinch, "hold it in while the lever is let go", g_ins),
        ]
        t_release_in = t_move_in + 2.4  # let go once the lug is fully in (let go part way, the latch would catch there)
        out += [
            Waypoint(0.8, LivePos(lambda: ins_w.at(at_w - ZUP * PUSH_IN, lift=False)), pinch_open, "let go: it is locked in", g_ins),
            Waypoint(1.0, LivePos(lambda: ins_w.at(at_w + ZUP * 0.060, lift=False)), pinch_open, "up off the hub", g_ins),
        ]
    knob_at: dict[str, Any] = {}

    def knob_start() -> tuple[np.ndarray, np.ndarray]:
        """The knob grip's place and rotation for where the knob is when the hand sets off for it (read once a trial:
        the lever's push may have turned the handle a little since it went in)."""
        if "k" not in knob_at or S.steps < knob_at["t"]:
            h = handle_now()
            k = knob_tcp(yaw_of(h[1]) if h is not None else psi_ins)
            knob_at.update(k=k, r=knob_rot(k, MODEL.twin if MODEL is not None else False), t=S.steps)
        return knob_at["k"], knob_at["r"]

    g0 = Grip(lambda: knob_start()[1])
    g_live = Grip(crank.rot)
    live = LivePos(crank.target)
    out += [
        Waypoint(2.5, LivePos(lambda: knob_start()[0] - knob_start()[1][:, 2] * KNOB_BACK), finger(KNOB_OPEN), "over the knob", g0),
        Waypoint(1.5, LivePos(lambda: knob_start()[0]), finger(KNOB_OPEN), "onto the knob", g0),
        Waypoint(0.8, LivePos(lambda: knob_start()[0]), finger(KNOB_GAP), "grip the knob", g0),
        Waypoint(0.5, live, finger(KNOB_GAP), "take up the knob", g_live),
        Waypoint(o.crank_s, live, finger(KNOB_GAP), "grind the sheet in", g_live),
        Waypoint(0.8, live, finger(KNOB_OPEN), "let go of the knob", g_live),
        Waypoint(1.0, LivePos(lambda: crank.target() - crank.rot()[:, 2] * KNOB_BACK), finger(KNOB_OPEN), "back off the knob", g_live),
    ]
    if o.remove:
        frozen: dict[str, float] = {}

        def heading() -> float:
            if "h" not in frozen or S.steps < frozen.get("k", 0):
                frozen["h"], frozen["k"] = removal_heading(), S.steps
            return frozen["h"]

        grab = np.array([PINCH_X, 0.0, PINCH_Z])

        def at_boss(dz: float) -> Callable[[], np.ndarray]:
            def fn() -> np.ndarray:
                out: np.ndarray = seat_w + rz(heading()) @ grab + ZUP * dz
                return out

            return fn

        g_rm = Grip(pinch_rot(pinch_at(seat_w, 0.0)))
        r_h = pinch_rot(pinch_at(seat_h, 0.0))
        g_h = Grip(lambda: ins_h.rot(r_h))  # turned by the handle's lean in the pinch while it is lined up over the holder
        at_h = pinch_at(seat_h, HOLDER_HEADING)
        grip_close = pinch
        out += [
            Waypoint(2.0, LivePos(at_boss(0.060)), pinch_open, "over the hub", g_rm),
            Waypoint(1.5, LivePos(at_boss(0.0)), pinch_open, "down round the hub", g_rm),
            Waypoint(1.0, LivePos(at_boss(0.0)), grip_close, "pinch the hub", g_rm),
            Waypoint(LEVER_LEAD + (2.5 if o.lever_early else 0.0), LivePos(at_boss(0.0)), grip_close, "hold it while the lever is turned", g_rm),
        ]
        t_move_out = sum(w.duration for w in out)
        out += [
            Waypoint(1.5, LivePos(at_boss(LIFT + SAG)), grip_close, "pull it up out of the winch", g_rm),
            Waypoint(1.0, LivePos(at_boss(LIFT + SAG)), grip_close, "hold it up while the lever is let go", g_rm),
        ]
        t_release_out = t_move_out + 1.6  # held until the lug is out (let go part way, the latch would catch there)
        if o.drop:
            out += [
                Waypoint(1.5, LivePos(at_boss(0.150 + SAG)), grip_close, "raise it", g_rm),
                Waypoint(0.8, LivePos(at_boss(0.150 + SAG)), pinch_open, "let it fall", g_rm),
                Waypoint(1.0, LivePos(at_boss(0.250)), pinch_open, "up", g_rm),
            ]
    if o.remove and not o.drop:
        out += [
            Waypoint(3.0, LivePos(lambda: ins_h.at(at_h + ZUP * LIFT)), grip_close, "carry it to the holder", g_h),
            Waypoint(1.5, LivePos(lambda: ins_h.at(at_h + ZUP * HOVER)), grip_close, "line it up over the holder", g_h),
            Waypoint(1.5, LivePos(lambda: ins_h.at(at_h - ZUP * 0.005, lift=False)), grip_close, "lower it into the holder", g_h),
            Waypoint(0.8, LivePos(lambda: ins_h.at(at_h - ZUP * 0.005, lift=False)), pinch_open, "let go of it", g_h),
            Waypoint(1.0, LivePos(lambda: ins_h.at(at_h + ZUP * 0.060, lift=False)), pinch_open, "up off it", g_h),
        ]
    out.append(Waypoint(2.5, tuple(park), FINGER_OPEN, "park", Grip(park_rot)))
    # The right hand: the lever, for putting the handle in and for taking it out (the lock needs it held both ways).
    hand_in = LeverHand((seat_w + ZUP * (HOVER + SOCKET_DEPTH), rz(psi_ins)))
    hand_out = LeverHand((seat_w, rz(psi_ins)))
    right: list[Waypoint] = []
    t_r = 0.0
    if not o.start_seated:
        right += lever_waypoints(hand_in, t_r, t_move_in, t_release_in, "in")
        t_r = sum(w.duration for w in right)
    if o.remove and o.lever:
        t_release = t_move_out - 1.0 if o.lever_early else t_release_out
        right += lever_waypoints(hand_out, t_r, t_move_out - (2.5 if o.lever_early else 0.0), t_release, "out")
    for sv in (crank, ins_w, ins_h, hand_in, hand_out):
        sv.follow(out)
    SERVOS.clear()
    SERVOS.update(crank=crank, ins_w=ins_w, ins_h=ins_h, lever_in=hand_in, lever_out=hand_out)
    end = max(sum(w.duration for w in out), sum(w.duration for w in right)) + 2.0
    out.append(Waypoint(end - sum(w.duration for w in out), tuple(park), FINGER_OPEN, "rest", Grip(park_rot)))
    rpark, rrot = ready_pose("right")
    right.append(Waypoint(end + 2.0 - sum(w.duration for w in right), tuple(rpark), FINGER_OPEN, "hold still", Grip(rrot)))
    return {"left": out, "right": right}


def ready_pose(side: str) -> tuple[np.ndarray, np.ndarray]:
    ik = importlib.import_module("yam_arena.ik")
    station = importlib.import_module("yam_arena.station")
    pos, rot = ik.tcp_pose(np.asarray(READY, dtype=float), station.BASES[side])
    return np.asarray(pos, dtype=float), np.asarray(rot, dtype=float)


def demo() -> dict[str, list[Waypoint]]:
    """The left arm takes the handle from its holder and sets it in the winch; the right arm holds the lock lever turned
    while the left pushes it home; the left grinds in first gear to the stall and reverses into second gear to the stall;
    then the right holds the lever turned while the left lifts the handle out and puts it back in its holder."""
    return routine()
