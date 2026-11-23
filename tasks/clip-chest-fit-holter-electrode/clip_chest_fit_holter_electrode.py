"""A CPR training manikin (adult torso with head) lies on its back on the bench, its head to the robot's left. A square of
dark faux fur with a 10 mm pile is taped over its breastbone: it stands in for chest hair. Four snap ECG electrodes are
already on bare skin on the chest, each with a lead of a 5-lead snap cable snapped on. The cable's fifth lead is
snapped onto the last electrode, which still sits on its liner card on the bench in front of the patient's right side.
A powered surgical hair clipper, switched off, lies on a tray beyond the patient's hips.

The last electrode goes in the middle of the fur, over the middle of the breastbone. Hair under an electrode stops it
sticking, so the hair must be clipped off there first, and the electrode then pressed onto the bare patch. The clipper
is switched on and off with the push button on its top; it cuts only with its blade's edge down on the skin.
"""

import contextlib
import importlib
import importlib.abc
import importlib.machinery
import importlib.util
import itertools
import math
import sys
import types
import warnings
from collections.abc import Callable, Iterator, Sequence
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
# PARAMETERS. Sources: makers' and distributors' listings read 2026-11-22 (cited in the README). "est" = estimate (not found or unverified); measure on the
# real kit before submission. Millimetre values are written in metres.
# =====================================================================================================================
# --- the manikin: Prestan Professional Adult PP-AM-100M-MS (61 x 35.6 x 17.8 cm and 7 lb are UNVERIFIED) ------------
HEAD_DIR = (0.0, 1.0)  # the body's axis on the bench, toward the head (head to the robot's left)
SITE_XY = (0.310, -0.050)  # the electrode site: mid-sternum (Welch Allyn H3+ diagram), the fur's middle
STERNUM_H = 0.150  # est: the skin over the mid-sternum above the bench (17.8 cm overall depth includes the head)
CHEST_HALF_W = 0.165  # est: half the chest's width (35.6 cm across the shoulders)
R_ALONG = 0.60  # est: the chest's curve along the body (m)
NECK_SIDE = 0.120  # est: site to the base of the neck
HIP_SIDE = 0.300  # est: site to the torso's lower end (61 cm head to pelvis, less the head and neck)
HEAD_R = 0.095  # est: the head, drawn as a sphere, face up
SKIN = (0.86, 0.70, 0.60)
# --- the hair: Shannon Fabrics Luxe Cuddle, 10 mm pile, dark solid colour (price UNVERIFIED) -------------------------
PANEL = (0.160, 0.120)  # along the body, across it (cut from the bolt)
PILE = 0.010  # 10 mm pile (retail listings)
STUBBLE = 0.0004  # est: what a cut leaves (0.2-0.5 mm for other makers' blades, unverified; the 9680's value not found)
CELL = 0.002  # the hair field's cells: 2 mm
STRANDS = 3  # drawn strands per cell
STRAND_R = 0.00035  # drawn strand radius
NAP_DEG = 35.0  # the drawn strands lean this far from the skin's normal, toward the hips (the nap; not in the cut rule)
HAIR_RGB = (0.09, 0.075, 0.065)
BACKING_RGB = (0.42, 0.38, 0.35)  # est: the knit backing seen through clipped pile (lighter than the pile)
# --- the clipper: 3M Surgical Clipper Professional 9681 with blade 9680 (distributor listing and 3M's how-to guide) ---
CLIP_LEN = 0.180  # about 18 cm long (listed 7.09 in)
CLIP_W = 0.060  # about 6 cm wide (listed 2.36 in)
CLIP_T = 0.035  # est: thickness NOT FOUND
CLIP_MASS = 0.200  # est: mass not found (150-250 g guessed)
BLADE_W = 0.0364  # blade width 36.4 mm (CME listing, verified)
SOLE = 0.012  # est: the blade's flat sole, front edge (the cutting edge) to its heel
HEAD_ANGLE = 25.0  # est (deg): the blade's sole is set at this angle to the body, so a pencil hold puts it flat
BODY_LIFT = 0.002  # est: the body's underside above the cutting edge (the body level)
HEAD_TAPER = 0.030  # est: the body narrows to the blade over its front 30 mm
BUTTON_R = 0.008  # est: one round push button on the top face (location and type NOT FOUND)
BUTTON_AT = 0.115  # est: its centre this far behind the cutting edge
BUTTON_TRAVEL = 0.0015  # est
BUTTON_FORCE = 4.0  # est: N to press it fully home
BUTTON_PRELOAD = 1.0  # est: N holding it up
BUTTON_MASS = 0.003  # est
LATCH_ON, LATCH_REARM = 0.70, 0.30  # fractions of travel: push-on/push-off with hysteresis
CLIPPER_AT = (0.180, -0.470)  # the cutting edge's middle, the clipper lying on the tray, teeth toward the robot (-x)
# --- the cut rule (est: no clipping speed or force is published; 3M: "Do not use excessive pressure") ---------------
GAP_MAX = 0.0015  # the cutting edge must be within this of the skin
SOLE_TILT_MAX = 15.0  # est (deg): "blade flat or at a slight angle"; steeper is raking ("Do not rake")
SEAT_N = 0.3  # N: smallest contact force between the clipper and the chest that counts as seated
FEED_MAX = 0.060  # m/s: est cap on the feed speed (faster passes cut nothing)
TEETH_COS = 0.5  # the motion must be within 60 deg of the teeth's direction
DRAG_PER_CM2_S = 0.04  # N per (cm^2/s) of hair cut: est feed drag (the motor cuts; the hand only guides)
DRAG_MAX = 1.0  # N
# --- the electrode: 3M Red Dot 2560 foam, 4 x 3.5 cm (verified); thickness and stud NOT FOUND ---------------------------
EL_SIZE = (0.040, 0.035)
EL_T = 0.0012  # est
EL_CORNER = 0.006  # est
STUD_R, STUD_H = 0.002, 0.004  # est (standard snap stud about 4 mm, unverified)
SNAP = (0.016, 0.011, 0.008)  # est: the lead's snap connector (clipped onto the stud)
EL_MASS = 0.0015 + 0.003  # est: electrode 1.5 g + snap connector 3 g
LINER = (0.070, 0.048, 0.0006)  # est: one electrode on its own liner card (cut from the strip), a 25 mm bare tab at one end
EL_ON_CARD = 0.010  # the electrode's centre sits this far from the card's centre, away from the tab
LINER_MASS = 0.002  # est
CARD_AT = (0.100, -0.020)  # the liner card's centre, on the bench beside the patient's right flank
CARD_YAW = 90.0  # deg: the card's long side (and the electrode's) along the body; its bare tab toward the hips (-y)
# --- the bond rule (est: no on-skin peel value published for this electrode; 0.1-1 N/cm at 90 deg in studies) ---------
LINER_RELEASE = 0.4  # N: est (a silicone release liner, well under 0.5 N: unverified)
SKIN_HOLD = 2.5  # N: est hold on bare skin (0.1-1 N/cm over a 35-40 mm edge: about 0.4-4 N)
BARE_EXP = 6.0  # hold = SKIN_HOLD * bare_fraction ** BARE_EXP (steep: hair under the gel stops it sticking)
PRESS_N = 3.0  # N: est press that makes the bond
PRESS_S = 1.0  # s: est, held
BOND_K = 4000.0  # N/m: the bond's spring (the hold force over it is under the break distance)
BOND_BREAK = 0.002  # m: a bond is broken once the electrode has moved this far from where it stuck
PEEL_ARM = 0.010  # m: the bond's torque limit is its force limit times this (a peel from one edge)
FACE_DOWN_DEG = 30.0  # it sticks only gel side down: its back within this of the skin's normal
# --- the tray ---------------------------------------------------------------------------------------------------------
TRAY = (0.240, 0.120, 0.004)  # est: a flat plastic instrument tray beyond the patient's hip
TRAY_AT = (0.270, -0.470)
# --- the four electrodes already on the chest (fixed dressing): same electrode, each with its lead's snap on ----------
DRESS_UV = ((0.100, 0.080), (0.100, -0.080), (-0.170, 0.095), (-0.170, -0.095))  # est: illustrative hairless places
LEAD_R = 0.002  # the lead wire's radius (a 5-lead snap cable lists "Lead Wire Diameter 4 mm")
LEAD_TAIL = 0.040  # each lead is drawn this far past the chest's side, along the bench (the wire beyond is not modelled)

# =====================================================================================================================
# Geometry (pure)
# =====================================================================================================================
AXIS = np.array(HEAD_DIR, dtype=float) / float(np.linalg.norm(HEAD_DIR))
ACROSS = np.array([-AXIS[1], AXIS[0]])
SITE = np.array(SITE_XY, dtype=float)


def to_uv(xy: Any) -> np.ndarray:
    """Bench points (x, y) to body coordinates (u toward the head, v across), from the site."""
    d = np.asarray(xy, dtype=float)[..., :2] - SITE
    return np.stack([d @ AXIS, d @ ACROSS], axis=-1)


def to_xy(uv: Any) -> np.ndarray:
    uv = np.asarray(uv, dtype=float)
    return np.asarray(SITE + uv[..., :1] * AXIS + uv[..., 1:2] * ACROSS, dtype=float)


def _end_round(u: Any) -> np.ndarray:
    mid, half = (NECK_SIDE - HIP_SIDE) / 2, (NECK_SIDE + HIP_SIDE) / 2
    s = np.clip(np.abs((np.asarray(u, dtype=float) - mid) / half), 0.0, 1.0)
    return (1.0 - s**6) ** (1.0 / 6.0)


def depth(u: Any) -> np.ndarray:
    """The chest's height over the bench along its middle line."""
    u = np.asarray(u, dtype=float)
    return np.asarray((STERNUM_H - u * u / (2 * R_ALONG)) * _end_round(u) / float(_end_round(0.0)), dtype=float)


def half_width(u: Any) -> np.ndarray:
    return np.asarray(CHEST_HALF_W * np.maximum(_end_round(u), 1e-3) ** 0.5, dtype=float)


def skin_z(uv: Any) -> np.ndarray:
    """The skin's height (world z) at body coordinates: an elliptic cross-section of depth(u) and half_width(u)."""
    uv = np.asarray(uv, dtype=float)
    u, v = uv[..., 0], uv[..., 1]
    s = np.clip(v / half_width(u), -1.0, 1.0)
    return np.asarray(TOP + depth(u) * np.sqrt(1.0 - s * s), dtype=float)


def skin_normal(uv: Any, eps: float = 0.0005) -> np.ndarray:
    uv = np.asarray(uv, dtype=float)
    du = (skin_z(uv + (eps, 0.0)) - skin_z(uv - (eps, 0.0))) / (2 * eps)
    dv = (skin_z(uv + (0.0, eps)) - skin_z(uv - (0.0, eps))) / (2 * eps)
    g = du[..., None] * AXIS + dv[..., None] * ACROSS
    n = np.concatenate([-g, np.ones(g.shape[:-1] + (1,))], axis=-1)
    return np.asarray(n / np.linalg.norm(n, axis=-1, keepdims=True), dtype=float)


def surface_point(uv: Any) -> np.ndarray:
    uv = np.asarray(uv, dtype=float)
    return np.concatenate([to_xy(uv), skin_z(uv)[..., None]], axis=-1)


# =====================================================================================================================
# The hair field and the cut rule (pure; unit-tested on the CPU)
# =====================================================================================================================
@dataclass
class HairField:
    """Cells of CELL over the fur panel, centred on the site: each has a pile height (PILE, or STUBBLE once cut)."""

    cell: float = CELL
    size: tuple[float, float] = PANEL
    nu: int = 0
    nv: int = 0
    height: np.ndarray = field(default_factory=lambda: np.zeros(0))

    def __post_init__(self) -> None:
        self.nu = round(self.size[0] / self.cell)
        self.nv = round(self.size[1] / self.cell)
        if self.height.size == 0:
            self.height = np.full(self.nu * self.nv, PILE)

    @property
    def centres(self) -> np.ndarray:
        """(n, 2) cell centres in body coordinates (u, v), index = iu * nv + iv."""
        iu, iv = np.meshgrid(np.arange(self.nu), np.arange(self.nv), indexing="ij")
        u = (iu.ravel() + 0.5) * self.cell - self.size[0] / 2
        v = (iv.ravel() + 0.5) * self.cell - self.size[1] / 2
        return np.stack([u, v], axis=-1)

    @property
    def cut(self) -> np.ndarray:
        return self.height <= STUBBLE + 1e-9

    def index(self, uv: Any) -> np.ndarray:
        """Cell index for each body point, -1 outside the panel."""
        uv = np.asarray(uv, dtype=float).reshape(-1, 2)
        iu = np.floor((uv[:, 0] + self.size[0] / 2) / self.cell).astype(int)
        iv = np.floor((uv[:, 1] + self.size[1] / 2) / self.cell).astype(int)
        inside = (iu >= 0) & (iu < self.nu) & (iv >= 0) & (iv < self.nv)
        return np.where(inside, iu * self.nv + iv, -1)


@dataclass
class Edge:
    """The cutting edge at one moment: its two ends in body coordinates and their heights above the skin (m)."""

    uv: np.ndarray  # (2, 2)
    gap: np.ndarray  # (2,)


@dataclass
class CutResult:
    cut: int = 0  # cells cut this step
    area_cm2: float = 0.0
    why: str = ""  # why nothing was cut ("" when something was)
    speed: float = 0.0
    teeth_cos: float = 0.0


def _seated_span(e: Edge) -> tuple[float, float] | None:
    """The part of the edge (0..1 from end 0 to end 1) within GAP_MAX of the skin (gap linear along it)."""
    g0, g1 = float(e.gap[0]), float(e.gap[1])
    lo, hi = 0.0, 1.0
    for t_gap, slope in ((g0, g1 - g0),):
        if abs(slope) < 1e-12:
            if abs(t_gap) > GAP_MAX:
                return None
            continue
        a, b = (-GAP_MAX - t_gap) / slope, (GAP_MAX - t_gap) / slope
        lo, hi = max(lo, min(a, b)), min(hi, max(a, b))
    return (lo, hi) if hi - lo > 1e-6 else None


def _in_triangle(p: np.ndarray, a: np.ndarray, b: np.ndarray, c: np.ndarray) -> np.ndarray:
    def cross(o: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        return np.asarray((x[0] - o[0]) * (y[..., 1] - o[1]) - (x[1] - o[1]) * (y[..., 0] - o[0]), dtype=float)

    d1, d2, d3 = cross(a, b, p), cross(b, c, p), cross(c, a, p)
    neg = (d1 < -1e-12) | (d2 < -1e-12) | (d3 < -1e-12)
    pos = (d1 > 1e-12) | (d2 > 1e-12) | (d3 > 1e-12)
    return ~(neg & pos)


def cut_step(
    hair: HairField,
    motor_on: bool,
    force: float | None,
    prev: Edge | None,
    now: Edge,
    teeth_uv: np.ndarray,
    dt: float = DT,
    tilt_deg: float = 0.0,
) -> CutResult:
    """One control step of the cut rule. A cell is cut (pile -> STUBBLE, never back) only if the motor is on, the
    clipper is pressed on the chest (force >= SEAT_N), the cutting edge was within GAP_MAX of the skin both at the
    last step and now (only the seated part of the edge cuts), it moved teeth first (within 60 deg of the teeth's
    direction) at no more than FEED_MAX with the sole within SOLE_TILT_MAX of the skin, and the cell's centre lies in
    the area the seated edge swept."""
    if not motor_on:
        return CutResult(why="motor off")
    if force is None or force < SEAT_N:
        return CutResult(why="not pressed on the chest")
    if prev is None:
        return CutResult(why="no previous step")
    if tilt_deg > SOLE_TILT_MAX:
        return CutResult(why="blade raked (sole too steep to the skin)")
    span_p, span_n = _seated_span(prev), _seated_span(now)
    if span_p is None or span_n is None:
        return CutResult(why="edge not on the skin")
    lo, hi = max(span_p[0], span_n[0]), min(span_p[1], span_n[1])
    if hi - lo < 1e-6:
        return CutResult(why="edge not on the skin")

    def at(e: Edge, t: float) -> np.ndarray:
        return np.asarray(e.uv[0] + t * (e.uv[1] - e.uv[0]), dtype=float)

    p0, p1, q1, q0 = at(prev, lo), at(prev, hi), at(now, hi), at(now, lo)
    move = (q0 + q1) / 2 - (p0 + p1) / 2
    dist = float(np.linalg.norm(move))
    speed = dist / dt
    if dist < 1e-4:
        return CutResult(why="not moving", speed=speed)
    teeth = np.asarray(teeth_uv, dtype=float)
    teeth = teeth / max(float(np.linalg.norm(teeth)), 1e-9)
    tc = float(move @ teeth) / dist
    if tc < TEETH_COS:
        return CutResult(why="not teeth first", speed=speed, teeth_cos=tc)
    if speed > FEED_MAX:
        return CutResult(why="too fast", speed=speed, teeth_cos=tc)
    quad = np.array([p0, p1, q1, q0])
    lo_uv, hi_uv = quad.min(axis=0) - hair.cell, quad.max(axis=0) + hair.cell
    c = hair.centres
    near = np.flatnonzero(np.all((c >= lo_uv) & (c <= hi_uv), axis=1) & ~hair.cut)
    if near.size == 0:
        return CutResult(why="nothing left to cut there", speed=speed, teeth_cos=tc)
    pts = c[near]
    inside = _in_triangle(pts, p0, p1, q1) | _in_triangle(pts, p0, q1, q0)
    hit = near[inside]
    hair.height[hit] = STUBBLE
    return CutResult(cut=int(hit.size), area_cm2=hit.size * (hair.cell * 100) ** 2, speed=speed, teeth_cos=tc)


def feed_drag(result: CutResult, dt: float = DT) -> float:
    """The drag on the clipper while cutting (N): proportional to the hair cut per second, capped (an estimate)."""
    return min(DRAG_MAX, DRAG_PER_CM2_S * result.area_cm2 / dt)


# =====================================================================================================================
# The bond rule (pure)
# =====================================================================================================================
def footprint_points(centre_uv: Any, axis_uv: Any, step: float = CELL / 2) -> np.ndarray:
    """Points over the electrode's footprint (a 40 x 35 mm rounded rectangle) in body coordinates."""
    a = np.asarray(axis_uv, dtype=float)
    a = a / max(float(np.linalg.norm(a)), 1e-9)
    b = np.array([-a[1], a[0]])
    hx, hy = EL_SIZE[0] / 2, EL_SIZE[1] / 2
    xs = np.arange(-hx + step / 2, hx, step)
    ys = np.arange(-hy + step / 2, hy, step)
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    gx, gy = gx.ravel(), gy.ravel()
    cx, cy = np.clip(np.abs(gx), 0, hx - EL_CORNER), np.clip(np.abs(gy), 0, hy - EL_CORNER)
    keep = (np.abs(gx) - cx) ** 2 + (np.abs(gy) - cy) ** 2 <= EL_CORNER**2
    gx, gy = gx[keep], gy[keep]
    return np.asarray(np.asarray(centre_uv, dtype=float) + gx[:, None] * a + gy[:, None] * b, dtype=float)


def bare_fraction(hair: HairField, centre_uv: Any, axis_uv: Any) -> float:
    """Share of the electrode's footprint over bare skin: clipped cells, or skin outside the fur."""
    idx = hair.index(footprint_points(centre_uv, axis_uv))
    bare = (idx < 0) | hair.cut[np.maximum(idx, 0)]
    return float(np.mean(bare))


def skin_hold(bare: float) -> float:
    """The bond's force limit on the chest (N): steep in the bare share (est)."""
    return float(SKIN_HOLD * float(np.clip(bare, 0.0, 1.0)) ** BARE_EXP)


@dataclass
class PressTimer:
    """Counts how long the electrode has been pressed on with at least PRESS_N (consecutive control steps)."""

    held: float = 0.0

    def update(self, force: float | None, dt: float = DT) -> bool:
        self.held = self.held + dt if force is not None and force >= PRESS_N else 0.0
        return self.held >= PRESS_S - 1e-9


@dataclass
class ButtonLatch:
    """Push-on, push-off: a press past LATCH_ON of the travel toggles the motor once; it re-arms below LATCH_REARM."""

    on: bool = False
    armed: bool = True
    toggles: int = 0

    def update(self, travel: float) -> bool:
        frac = travel / BUTTON_TRAVEL
        if self.armed and frac >= LATCH_ON:
            self.on, self.armed = not self.on, False
            self.toggles += 1
        elif not self.armed and frac <= LATCH_REARM:
            self.armed = True
        return self.on


# =====================================================================================================================
# The clipper's frame (pure): x toward the teeth, y left, z up from its sole; origin at the middle of the cutting edge
# =====================================================================================================================
EDGE_LOCAL = np.array([[0.0, -BLADE_W / 2, 0.0], [0.0, BLADE_W / 2, 0.0]])
CLIP_COM = np.array([-CLIP_LEN * 0.55, 0.0, BODY_LIFT + CLIP_T / 2])  # est: a little behind the middle (the battery)
BUTTON_LOCAL = np.array([-BUTTON_AT, 0.0, BODY_LIFT + CLIP_T])


def _ry(deg: float) -> np.ndarray:
    t = math.radians(deg)
    return np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])


SOLE_ROT = _ry(-HEAD_ANGLE)  # the sole's frame in the clipper's: level when the body is pitched HEAD_ANGLE rear-up


def sole_tilt_deg(rot: np.ndarray, uv: Any) -> float:
    """Angle between the blade's sole and the skin under the edge's middle (deg)."""
    n = rot @ SOLE_ROT[:, 2]
    return math.degrees(math.acos(float(np.clip(n @ skin_normal(np.asarray(uv, dtype=float)), -1.0, 1.0))))


def edge_now(pos: np.ndarray, rot: np.ndarray) -> Edge:
    """The cutting edge's ends in body coordinates and their heights above the skin, for the clipper's origin pose."""
    w = pos + EDGE_LOCAL @ rot.T
    uv = to_uv(w[:, :2])
    return Edge(uv=uv, gap=w[:, 2] - skin_z(uv))


def teeth_uv(rot: np.ndarray) -> np.ndarray:
    """The teeth's direction (the clipper's x) in body coordinates."""
    return np.array([rot[:2, 0] @ AXIS, rot[:2, 0] @ ACROSS])


# =====================================================================================================================
# The routine's geometry (pure): every pose is checked with the official IK
# =====================================================================================================================
PAD_TIP = 0.013  # the YAM's pad tips are this far past the TCP along the approach (official PAD_TIP_PAST_TCP)
GRIP_BACK = 0.090  # est: the pinch's centre behind the cutting edge (clear of the button; held "like a pencil")
CLIP_PITCH = HEAD_ANGLE + 10.0  # the body above the skin while clipping: the sole 10 deg off flat, edge down
PASSES_U = (-0.012, 0.012)  # two passes along the body axis offsets: 2 x 36.4 mm covers u -30..+30 mm
PASS_V = (-0.028, 0.040)  # each pulls the clipper toward the robot (teeth first) across the site, 68 mm


def clipper_frame(edge: np.ndarray, normal: np.ndarray, teeth: np.ndarray, pitch: float) -> np.ndarray:
    """The clipper's rotation with its cutting edge on the skin, teeth along `teeth`, body `pitch` deg rear-up."""
    d = teeth - float(teeth @ normal) * normal
    d = d / float(np.linalg.norm(d))
    a = math.radians(pitch)
    x = math.cos(a) * d - math.sin(a) * normal
    z = math.sin(a) * d + math.cos(a) * normal
    return np.column_stack([x, np.cross(z, x), z])


def grip_tcp(edge: np.ndarray, rot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The right hand's top pinch across the clipper's width (TCP rotation columns: closing, across, approach)."""
    tcp = edge + rot @ np.array([-GRIP_BACK, 0.0, BODY_LIFT + CLIP_T / 2])
    return tcp, np.column_stack([rot[:, 1], rot[:, 0], -rot[:, 2]])


def _tilted(tilt: float, az: float) -> np.ndarray:
    t, a = math.radians(tilt), math.radians(az)
    approach = np.array([math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), -math.cos(t)])
    closing = np.array([-math.sin(a), math.cos(a), 0.0])
    closing = closing - float(closing @ approach) * approach
    closing /= float(np.linalg.norm(closing))
    return np.column_stack([closing, np.cross(approach, closing), approach])


def REACH_POSES() -> dict[str, tuple[str, tuple[float, float, float], np.ndarray]]:
    """Named TCP poses of the routine (arm, position, rotation): every waypoint of the demo's plans."""
    out: dict[str, tuple[str, tuple[float, float, float], np.ndarray]] = {}
    plans = routine()
    for side in ("right", "left"):
        for k, w in enumerate(plans[side]):
            rot = getattr(w.yaw, "rot", None)
            if rot is None:
                continue
            p = np.asarray(w.pos, dtype=float)
            out[f"{side[0].upper()} {k:02d} {w.label}"] = (side, (float(p[0]), float(p[1]), float(p[2])), np.asarray(rot, dtype=float))
    return out


# =====================================================================================================================
# The scene hook: when yam_arena.scene is imported, the tray's spawner also builds everything else
# =====================================================================================================================
_SCENE_MODULE = "yam_arena.scene"
HOOK_TAG = "clip_holter_hook"
RECORD_NAME = "clip_holter_record"
BASE = "instrument_tray"
OBJECTS = [Box(BASE, pos=(TRAY_AT[0], TRAY_AT[1], TOP + TRAY[2] / 2), size=TRAY, color=(0.80, 0.84, 0.88))]
CONTACT_OFFSET, REST_OFFSET = 0.0008, 0.0
INERTIA_FLOOR = 3e-7  # kg m^2: light bodies need it to stay stable (masses unchanged)


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


def quat_to_matrix(xyzw: Sequence[float] | np.ndarray) -> np.ndarray:
    x, y, z, w = (float(v) for v in xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


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


def _collider(px: _Pxr, prim: Any, offset: float = CONTACT_OFFSET) -> None:
    px.physics.CollisionAPI.Apply(prim)
    prim.AddAppliedSchema("PhysxCollisionAPI")
    prim.CreateAttribute("physxCollision:contactOffset", px.sdf.ValueTypeNames.Float).Set(float(offset))
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


def _cylinder(px: _Pxr, stage: Any, path: str, center: Sequence[float], radius: float, height: float, mat: Any, collides: bool = True, axis: str = "Z") -> str:
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


def _mesh(px: _Pxr, stage: Any, path: str, points: np.ndarray, tris: np.ndarray, mat: Any, collides: bool = False) -> str:
    mesh = px.geom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([px.gf.Vec3f(*(float(c) for c in p)) for p in points])
    mesh.CreateFaceVertexCountsAttr([3] * len(tris))
    mesh.CreateFaceVertexIndicesAttr([int(i) for i in np.asarray(tris).ravel()])
    lo, hi = points.min(axis=0), points.max(axis=0)
    mesh.CreateExtentAttr([px.gf.Vec3f(*(float(c) for c in lo)), px.gf.Vec3f(*(float(c) for c in hi))])
    mesh.CreateSubdivisionSchemeAttr("none")
    if collides:
        _collider(px, mesh.GetPrim())
        px.physics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
    _bind(px, mesh.GetPrim(), mat)
    return path


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
    prim.CreateAttribute("physxRigidBody:linearDamping", f).Set(0.2)
    prim.CreateAttribute("physxRigidBody:angularDamping", f).Set(0.2)
    prim.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", f).Set(0.5)
    prim.CreateAttribute("physxRigidBody:sleepThreshold", f).Set(0.0)
    prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", px.sdf.ValueTypeNames.Int).Set(32)
    prim.CreateAttribute("physxRigidBody:solverVelocityIterationCount", px.sdf.ValueTypeNames.Int).Set(4)
    mapi = px.physics.MassAPI.Apply(prim)
    mapi.CreateMassAttr(float(mass))
    mapi.CreateCenterOfMassAttr(px.gf.Vec3f(*(float(c) for c in com)))
    mapi.CreateDiagonalInertiaAttr(px.gf.Vec3f(*(max(float(v), INERTIA_FLOOR) for v in inertia)))
    return prim


def _box_inertia(m: float, size: Sequence[float]) -> tuple[float, float, float]:
    a, b, c = size
    return (m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12)


def _drive(px: _Pxr, prim: Any, axis: str, k: float, d: float, fmax: float, target: float = 0.0) -> None:
    api = px.physics.DriveAPI.Apply(prim, axis)
    api.CreateTypeAttr("force")
    api.CreateStiffnessAttr(float(k))
    api.CreateDampingAttr(float(d))
    api.CreateMaxForceAttr(float(fmax))  # finite: PhysX rejects an infinite drive limit
    api.CreateTargetPositionAttr(float(target))
    api.CreateTargetVelocityAttr(0.0)


DOFS = ("transX", "transY", "transZ", "rotX", "rotY", "rotZ")


def torso_mesh(nu: int = 120, nt: int = 72) -> tuple[np.ndarray, np.ndarray]:
    """The torso's skin as a closed mesh in world coordinates: elliptic sections over u, a flat underside."""
    us = np.linspace(-HIP_SIDE, NECK_SIDE, nu)
    ts = np.linspace(0.0, math.pi, nt)
    pts = []
    for u in us:
        d, w = float(depth(u)), float(half_width(u))
        for t in ts:
            uv = np.array([u, w * math.cos(t)])
            xy = to_xy(uv)
            pts.append((xy[0], xy[1], TOP + 0.0005 + max(d - 0.0005, 0.0) * math.sin(t)))
    p = np.array(pts)
    tris = []
    for i in range(nu - 1):
        for j in range(nt):
            a, b = i * nt + j, i * nt + (j + 1) % nt
            c, e = (i + 1) * nt + j, (i + 1) * nt + (j + 1) % nt
            tris += [(a, c, b), (b, c, e)]
    # end caps: fans from each end's centre
    for i, flip in ((0, False), (nu - 1, True)):
        centre = len(p)
        ring = p[i * nt : (i + 1) * nt]
        p = np.vstack([p, ring.mean(axis=0)])
        for j in range(nt):
            a, b = i * nt + j, i * nt + (j + 1) % nt
            tris.append((centre, b, a) if not flip else (centre, a, b))
    return p, np.array(tris)


SLICE = 0.010  # the torso's collider: solid convex slices this long along the body (a thin mesh shell lets parts tunnel)


def torso_slices(top_pts: int = 28) -> list[tuple[np.ndarray, np.ndarray]]:
    """Convex pieces of the torso: between two cross-sections SLICE apart, each the skin's elliptic arc sampled at
    top_pts points evenly across plus the two bottom corners (60 points: inside the GPU's 64-vertex hull limit).
    Returned as point clouds with a fan of triangles (PhysX takes their convex hull)."""
    out = []
    edges = np.arange(-HIP_SIDE, NECK_SIDE + 1e-9, SLICE)
    for u0, u1 in itertools.pairwise(edges):
        pts = []
        for u in (u0, u1):
            w = float(half_width(u))
            for v in np.linspace(-w, w, top_pts):
                pts.append(surface_point(np.array([u, v])))
            for edge_v in (-w, w):
                pts.append(np.r_[to_xy(np.array([u, edge_v])), TOP + 0.0005])
        p = np.array(pts)
        tris = np.array([(0, i, i + 1) for i in range(1, len(p) - 1)])
        out.append((p, tris))
    return out


def strand_layout(hair: HairField, seed: int = 7) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Drawn strands: (n, 3) roots on the skin, (n, 4) orientations (w, x, y, z), (n,) cell of each."""
    rng = np.random.default_rng(seed)
    c = np.repeat(hair.centres, STRANDS, axis=0)
    cell = np.repeat(np.arange(len(hair.centres)), STRANDS)
    uv = c + rng.uniform(-0.5, 0.5, c.shape) * hair.cell
    roots = surface_point(uv)
    n = skin_normal(uv)
    nap = np.concatenate([-AXIS, [0.0]])
    out = []
    for k in range(len(uv)):
        lean = math.radians(NAP_DEG + rng.uniform(-12, 12))
        turn = math.radians(rng.uniform(-30, 30))
        side = np.cross(n[k], nap)
        side /= max(float(np.linalg.norm(side)), 1e-9)
        fwd = np.cross(side, n[k])
        dirn = math.cos(turn) * fwd + math.sin(turn) * side
        z = math.cos(lean) * n[k] + math.sin(lean) * dirn
        x = np.cross(side, z)
        x /= max(float(np.linalg.norm(x)), 1e-9)
        y = np.cross(z, x)
        out.append(wxyz(np.column_stack([x, y, z])))
    return roots, np.array(out), cell


def skin_frame(uv: Any) -> np.ndarray:
    """A frame on the skin at uv: x along the body (toward the head), z the skin's normal."""
    n = skin_normal(np.asarray(uv, dtype=float))
    a = np.r_[AXIS, 0.0]
    x = a - float(a @ n) * n
    x /= float(np.linalg.norm(x))
    return np.column_stack([x, np.cross(n, x), n])


def dress_layout() -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """The placed electrodes: (pad centre, frame, lead polyline). Each lead runs from its snap across the chest to the
    nearer side (v > 0 is the robot's side), down onto the bench and LEAD_TAIL along it."""
    out = []
    for u, v in DRESS_UV:
        uv = np.array([u, v])
        frame = skin_frame(uv)
        centre = surface_point(uv) + frame[:, 2] * (EL_T / 2)
        side = 1.0 if v > 0 else -1.0
        w = float(half_width(u))
        pts = [centre + frame[:, 2] * (EL_T / 2 + SNAP[2] / 2)]
        for vv in np.linspace(v + side * 0.010, side * (w - 0.004), 8):
            q = np.array([u, vv])
            pts.append(surface_point(q) + skin_normal(q) * LEAD_R)
        edge = np.r_[to_xy(np.array([u, side * (w + 0.012)])), TOP + LEAD_R]
        tail = np.r_[to_xy(np.array([u, side * (w + 0.012 + LEAD_TAIL)])), TOP + LEAD_R]
        pts += [edge, tail]
        out.append((centre, frame, np.array(pts)))
    return out


def _segment(px: _Pxr, stage: Any, path: str, a: np.ndarray, b: np.ndarray, radius: float, mat: Any) -> None:
    """A cylinder from a to b (a lead's piece), as a scaled unit cylinder along z."""
    d = b - a
    length = float(np.linalg.norm(d))
    z = d / max(length, 1e-9)
    x = np.cross(z, (0.0, 0.0, 1.0) if abs(z[2]) < 0.9 else (1.0, 0.0, 0.0))
    x /= float(np.linalg.norm(x))
    rot = np.column_stack([x, np.cross(z, x), z])
    cyl = px.geom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(1.0)
    cyl.CreateHeightAttr(1.0)
    cyl.CreateAxisAttr("Z")
    cyl.CreateExtentAttr([px.gf.Vec3f(-1.0, -1.0, -0.5), px.gf.Vec3f(1.0, 1.0, 0.5)])
    xf = px.geom.Xformable(cyl.GetPrim())
    xf.AddTranslateOp().Set(px.gf.Vec3d(*(float(c) for c in (a + b) / 2)))
    xf.AddOrientOp().Set(px.gf.Quatf(*wxyz(rot)))
    xf.AddScaleOp().Set(px.gf.Vec3f(float(radius), float(radius), length))
    _collider(px, cyl.GetPrim())
    _bind(px, cyl.GetPrim(), mat)


def card_rot() -> np.ndarray:
    t = math.radians(CARD_YAW)
    return np.array([[math.cos(t), -math.sin(t), 0.0], [math.sin(t), math.cos(t), 0.0], [0.0, 0.0, 1.0]])


def xyzw(r: np.ndarray) -> list[float]:
    w, x, y, z = wxyz(r)
    return [x, y, z, w]


HAIR = HairField()


def author(stage: Any, env_root: str) -> None:
    px = _Pxr()
    rec = _record()
    root = f"{env_root}/holter"
    px.geom.Xform.Define(stage, root)
    skin = _material(px, stage, f"{root}/skin", SKIN, (0.9, 0.8), 0.7)
    backing = _material(px, stage, f"{root}/backing", BACKING_RGB, (0.9, 0.8), 0.9)
    hair_mat = _material(px, stage, f"{root}/hair", HAIR_RGB, (0.9, 0.8), 0.8)
    grey = _material(px, stage, f"{root}/clipper_grey", (0.55, 0.58, 0.62), (0.6, 0.5), 0.4)
    blue = _material(px, stage, f"{root}/clipper_blue", (0.12, 0.33, 0.62), (0.6, 0.5), 0.4)
    steel = _material(px, stage, f"{root}/steel", (0.78, 0.79, 0.80), (0.4, 0.35), 0.3)
    foam = _material(px, stage, f"{root}/foam", (0.95, 0.95, 0.93), (0.9, 0.8), 0.9)
    red = _material(px, stage, f"{root}/red", (0.80, 0.10, 0.10), (0.6, 0.5), 0.5)
    card = _material(px, stage, f"{root}/liner", (0.97, 0.97, 0.99), (0.4, 0.35), 0.4)
    snap_m = _material(px, stage, f"{root}/snap", (0.10, 0.45, 0.20), (0.8, 0.7), 0.5)
    i3 = np.eye(3)

    # The manikin: a kinematic body (so contact views can read force against it), triangle-mesh skin, a head.
    torso = f"{root}/torso"
    _body(px, stage, torso, (0.0, 0.0, 0.0), i3, 3.2, (0.1, 0.1, 0.1), kinematic=True)
    pts, tris = torso_mesh()
    _mesh(px, stage, f"{torso}/skin", pts, tris, skin)  # drawn; it collides through the solid slices below
    for i, (sp, st) in enumerate(torso_slices()):
        path = _mesh(px, stage, f"{torso}/slice{i:02d}", sp, st, skin)
        prim = stage.GetPrimAtPath(path)
        _collider(px, prim)
        px.physics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        px.geom.Imageable(prim).MakeInvisible()
    neck_c = np.r_[to_xy(np.array([NECK_SIDE + 0.03, 0.0])), TOP + 0.06]
    head_c = np.r_[to_xy(np.array([NECK_SIDE + 0.03 + HEAD_R + 0.02, 0.0])), TOP + HEAD_R]
    sph = px.geom.Sphere.Define(stage, f"{torso}/head")
    sph.CreateRadiusAttr(float(HEAD_R))
    sph.CreateExtentAttr([px.gf.Vec3f(-HEAD_R, -HEAD_R, -HEAD_R), px.gf.Vec3f(HEAD_R, HEAD_R, HEAD_R)])
    px.geom.Xformable(sph.GetPrim()).AddTranslateOp().Set(px.gf.Vec3d(*(float(c) for c in head_c)))
    _collider(px, sph.GetPrim())
    _bind(px, sph.GetPrim(), skin)
    _cylinder(px, stage, f"{torso}/neck", neck_c, 0.055, 0.07, skin, axis="Y" if abs(AXIS[1]) > 0.5 else "X")

    # The fur: the backing drawn just over the skin (visual), the pile as instanced strands.
    hb = HAIR
    us = np.linspace(-PANEL[0] / 2, PANEL[0] / 2, 41)
    vs = np.linspace(-PANEL[1] / 2, PANEL[1] / 2, 31)
    grid = np.array([[u, v] for u in us for v in vs])
    bp = surface_point(grid) + skin_normal(grid) * 0.0003
    bt = []
    for i in range(len(us) - 1):
        for j in range(len(vs) - 1):
            a, b, c, d = i * len(vs) + j, i * len(vs) + j + 1, (i + 1) * len(vs) + j, (i + 1) * len(vs) + j + 1
            bt += [(a, c, b), (b, c, d)]
    _mesh(px, stage, f"{root}/fur_backing", bp, np.array(bt), backing)
    roots, orients, _ = strand_layout(hb)
    inst = px.geom.PointInstancer.Define(stage, f"{root}/fur_pile")
    proto = px.geom.Xform.Define(stage, f"{root}/fur_pile/strand")
    strand = px.geom.Cylinder.Define(stage, f"{root}/fur_pile/strand/shape")
    strand.CreateRadiusAttr(1.0)
    strand.CreateHeightAttr(1.0)
    strand.CreateAxisAttr("Z")
    strand.CreateExtentAttr([px.gf.Vec3f(-1, -1, -0.5), px.gf.Vec3f(1, 1, 0.5)])
    px.geom.Xformable(strand.GetPrim()).AddTranslateOp().Set(px.gf.Vec3d(0.0, 0.0, 0.5))
    _bind(px, strand.GetPrim(), hair_mat)
    inst.CreatePrototypesRel().SetTargets([proto.GetPath()])
    inst.CreateProtoIndicesAttr([0] * len(roots))
    inst.CreatePositionsAttr(px.vt.Vec3fArray.FromNumpy(roots.astype(np.float32)))
    inst.CreateOrientationsAttr(px.vt.QuathArray([px.gf.Quath(*(float(c) for c in q)) for q in orients]))
    scales = np.tile(np.array([STRAND_R, STRAND_R, PILE], dtype=np.float32), (len(roots), 1))
    inst.CreateScalesAttr(px.vt.Vec3fArray.FromNumpy(scales))
    lo, hi = roots.min(axis=0) - PILE, roots.max(axis=0) + PILE
    inst.CreateExtentAttr([px.gf.Vec3f(*(float(c) for c in lo)), px.gf.Vec3f(*(float(c) for c in hi))])
    rec["pile"] = f"{root}/fur_pile"

    # The clipper: one rigid body (the 9681's head is fixed), origin at the middle of the cutting edge, x toward the
    # teeth, z up from the sole. Lying flat on the tray, teeth toward the robot.
    clip = f"{root}/clipper"
    crot = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])  # teeth toward the robot
    cpos = np.array([CLIPPER_AT[0], CLIPPER_AT[1], TOP + TRAY[2] + SOLE * math.sin(math.radians(HEAD_ANGLE)) + 0.0004])
    _body(px, stage, clip, cpos, crot, CLIP_MASS, _box_inertia(CLIP_MASS, (CLIP_LEN, CLIP_W, CLIP_T)), com=CLIP_COM)
    body_len = CLIP_LEN - HEAD_TAPER
    _cube(px, stage, f"{clip}/body", (-HEAD_TAPER - body_len / 2, 0.0, BODY_LIFT + CLIP_T / 2), (body_len, CLIP_W, CLIP_T), blue)
    neck_len = HEAD_TAPER - SOLE * math.cos(math.radians(HEAD_ANGLE))
    _cube(px, stage, f"{clip}/neck", (-HEAD_TAPER + neck_len / 2, 0.0, BODY_LIFT + 0.012), (neck_len, (CLIP_W + BLADE_W) / 2, 0.020), grey)
    for name, centre, size, mat, hit in (
        ("blade", (-SOLE / 2, 0.0, 0.0045), (SOLE, BLADE_W + 0.004, 0.006), grey, True),
        ("sole", (-SOLE / 2, 0.0, 0.00075), (SOLE, BLADE_W, 0.0015), steel, True),
    ):
        _cube(px, stage, f"{clip}/{name}", SOLE_ROT @ np.array(centre), size, mat, collides=hit, rot=SOLE_ROT)
    for k in range(9):  # the comb's teeth, drawn
        y = (k - 4) * BLADE_W / 9
        _cube(px, stage, f"{clip}/tooth{k}", SOLE_ROT @ np.array([0.0012, y, 0.0015]), (0.0025, 0.0018, 0.003), steel, collides=False, rot=SOLE_ROT)
    rec["clipper"] = clip
    btn = f"{root}/button"
    bpos = cpos + crot @ (BUTTON_LOCAL + (0.0, 0.0, 0.0025))
    _body(px, stage, btn, bpos, crot, BUTTON_MASS, (1e-7, 1e-7, 1e-7))
    _cylinder(px, stage, f"{btn}/cap", (0.0, 0.0, 0.0), BUTTON_R, 0.005, red)
    j = px.physics.Joint.Define(stage, f"{root}/button_slide")
    j.CreateBody0Rel().SetTargets([clip])
    j.CreateBody1Rel().SetTargets([btn])
    j.CreateLocalPos0Attr(px.gf.Vec3f(*(float(c) for c in BUTTON_LOCAL + (0.0, 0.0, 0.0025))))
    j.CreateLocalRot0Attr(px.gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateLocalPos1Attr(px.gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(px.gf.Quatf(1.0, 0.0, 0.0, 0.0))
    jp = j.GetPrim()
    for dof in ("transX", "transY", "rotX", "rotY", "rotZ"):
        lim = px.physics.LimitAPI.Apply(jp, dof)
        lim.CreateLowAttr(1.0)
        lim.CreateHighAttr(-1.0)  # low > high: locked
    lim = px.physics.LimitAPI.Apply(jp, "transZ")
    lim.CreateLowAttr(-BUTTON_TRAVEL)
    lim.CreateHighAttr(0.0)
    k_btn = (BUTTON_FORCE - BUTTON_PRELOAD) / BUTTON_TRAVEL
    _drive(px, jp, "transZ", k_btn, 2.0, 20.0, BUTTON_PRELOAD / k_btn)
    rec["button"] = btn

    # The electrode (pad, stud and the lead's snap connector: one body) on its liner card on the bench.
    liner = f"{root}/liner"
    lpos = (CARD_AT[0], CARD_AT[1], TOP + LINER[2] / 2 + 0.0001)
    yaw = card_rot()
    _body(px, stage, liner, lpos, yaw, LINER_MASS, _box_inertia(LINER_MASS, LINER))
    _cube(px, stage, f"{liner}/card", (0.0, 0.0, 0.0), LINER, card)
    el = f"{root}/electrode"
    on_card = yaw @ np.array([EL_ON_CARD, 0.0, 0.0])
    epos = (CARD_AT[0] + on_card[0], CARD_AT[1] + on_card[1], lpos[2] + LINER[2] / 2 + EL_T / 2 + 0.0001)
    _body(px, stage, el, epos, yaw, EL_MASS, _box_inertia(EL_MASS, (EL_SIZE[0], EL_SIZE[1], 0.01)), com=(0.0, 0.0, 0.003))
    _cube(px, stage, f"{el}/pad", (0.0, 0.0, 0.0), (EL_SIZE[0], EL_SIZE[1], EL_T), foam)
    _cylinder(px, stage, f"{el}/stud", (0.0, 0.0, EL_T / 2 + STUD_H / 2), STUD_R, STUD_H, steel, collides=False)
    _cube(px, stage, f"{el}/snap", (0.0, 0.0, EL_T / 2 + SNAP[2] / 2), SNAP, snap_m)
    rec["electrode"], rec["liner"] = el, liner
    # Bonds: D6 joints whose six drives are springs capped at the bond's hold; enabled when a bond forms.
    for name, b0, local0, enabled, hold in (
        ("bond_liner", liner, (EL_ON_CARD, 0.0, LINER[2] / 2 + EL_T / 2 + 0.0001), True, LINER_RELEASE),
        ("bond_skin", torso, (0.0, 0.0, 0.0), False, 0.0),
    ):
        bj = px.physics.Joint.Define(stage, f"{root}/{name}")
        bj.CreateBody0Rel().SetTargets([b0])
        bj.CreateBody1Rel().SetTargets([el])
        bj.CreateLocalPos0Attr(px.gf.Vec3f(*(float(c) for c in local0)))
        bj.CreateLocalRot0Attr(px.gf.Quatf(1.0, 0.0, 0.0, 0.0))
        bj.CreateLocalPos1Attr(px.gf.Vec3f(0.0, 0.0, 0.0))
        bj.CreateLocalRot1Attr(px.gf.Quatf(1.0, 0.0, 0.0, 0.0))
        bj.CreateExcludeFromArticulationAttr(True)
        bj.CreateJointEnabledAttr(enabled)
        bj.CreateCollisionEnabledAttr(True)
        for dof in DOFS:
            trans = dof.startswith("trans")
            _drive(px, bj.GetPrim(), dof, BOND_K if trans else 2.0, 2.0 if trans else 0.01, max(hold if trans else hold * PEEL_ARM, 1e-6))
        rec[name] = f"{root}/{name}"
    # The four electrodes already placed, with their leads: one fixed body (it never moves), drawn and colliding.
    dress = f"{root}/dressing"
    _body(px, stage, dress, (0.0, 0.0, 0.0), i3, 0.1, (0.01, 0.01, 0.01), kinematic=True)
    lead_m = _material(px, stage, f"{root}/lead", (0.15, 0.15, 0.17), (0.6, 0.5), 0.5)
    for k, (pad_c, frame, lead) in enumerate(dress_layout()):
        _cube(px, stage, f"{dress}/pad{k}", pad_c, (EL_SIZE[0], EL_SIZE[1], EL_T), foam, rot=frame)
        _cube(px, stage, f"{dress}/snap{k}", pad_c + frame[:, 2] * (EL_T / 2 + SNAP[2] / 2), SNAP, snap_m, rot=frame)
        for j, (a, b) in enumerate(itertools.pairwise(lead)):
            _segment(px, stage, f"{dress}/lead{k}_{j:02d}", a, b, LEAD_R, lead_m)
    rec["dressing"] = dress
    # Where each loose body starts (env frame: x, y, z, then the quaternion x, y, z, w): a new trial puts them back.
    rec["init"] = {
        clip: [*map(float, cpos), *xyzw(crot)],
        btn: [*map(float, bpos), *xyzw(crot)],
        liner: [*map(float, lpos), *xyzw(yaw)],
        el: [*map(float, epos), *xyzw(yaw)],
    }
    # Contact reports on the bodies whose contact forces are read (PhysX contact views need them).
    for body in (torso, clip, el, dress):
        prim = stage.GetPrimAtPath(body)
        prim.AddAppliedSchema("PhysxContactReportAPI")
        prim.CreateAttribute("physxContactReport:threshold", px.sdf.ValueTypeNames.Float).Set(0.0)
    # The feed drag: a PhysX force on the clipper, authored now (zero, off) so that the run only sets its values (adding
    # a schema or an attribute while the simulation runs makes PhysX re-read the stage and drops its views).
    cprim = stage.GetPrimAtPath(clip)
    cprim.AddAppliedSchema("PhysxForceAPI")
    cprim.CreateAttribute("physxForce:force", px.sdf.ValueTypeNames.Float3).Set(px.gf.Vec3f(0.0, 0.0, 0.0))
    cprim.CreateAttribute("physxForce:torque", px.sdf.ValueTypeNames.Float3).Set(px.gf.Vec3f(0.0, 0.0, 0.0))
    cprim.CreateAttribute("physxForce:worldFrameEnabled", px.sdf.ValueTypeNames.Bool).Set(True)
    cprim.CreateAttribute("physxForce:forceEnabled", px.sdf.ValueTypeNames.Bool).Set(False)
    cprim.CreateAttribute("physxForce:mode", px.sdf.ValueTypeNames.Token).Set("force")
    rec.update(root=root, torso=torso, built=True, env_root=env_root)


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
    marker = "clip_holter_watcher"

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
# Runtime: PhysX views (bodies, contact forces) and USD writes (the bonds, the pile's drawing, the feed drag)
# =====================================================================================================================
def _stage() -> Any:
    return importlib.import_module("omni.usd").get_context().get_stage()


_written: dict[tuple[str, str], Any] = {}


def set_attr(path: str, name: str, value: Any, kind: str = "Float") -> None:
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
    clip_contact: Any = None
    el_contact: Any = None
    dress_contact: Any = None
    failed: str = ""
    extra_bodies: list[str] = []  # noqa: RUF012 (a test rig may add its own bodies here before the first step)
    last: Bodies | None = None  # the last step's readings (the demo's live targets read them)
    start: dict[str, np.ndarray] = {}  # noqa: RUF012 (each loose part's place at the first trial's third reading)


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
            want = [rec["clipper"], rec["button"], rec["electrode"], rec["liner"], *Live.extra_bodies]
            Live.bodies = sv.create_rigid_body_view(want)
            Live.paths = list(Live.bodies.prim_paths)
            Live.clip_contact = sv.create_rigid_contact_view([rec["clipper"]], filter_patterns=[[rec["torso"]]], max_contact_data_count=64)
            Live.el_contact = sv.create_rigid_contact_view([rec["electrode"]], filter_patterns=[[rec["torso"]]], max_contact_data_count=64)
            Live.dress_contact = sv.create_rigid_contact_view([rec["clipper"]], filter_patterns=[[rec["dressing"]]], max_contact_data_count=64)
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


def _contact_force(view: Any) -> float | None:
    if view is None:
        return None
    try:
        f = _np(view.get_contact_force_matrix(dt=1.0 / 240.0)).reshape(-1, 3)
        v = float(np.linalg.norm(f.sum(axis=0)))
    except Exception as error:
        Live.failed = Live.failed or f"contact: {type(error).__name__}: {error}"
        return None
    if not math.isfinite(v):
        Live.failed = Live.failed or "contact: not finite"
        return None
    return v


def restore_bodies() -> None:
    """A new trial: every loose body back where it started, at rest. Written through PhysX, because these bodies are
    the scene hook's own and the simulator's reset does not know them."""
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


RESTORE_TOL = 0.005  # m: at a new trial's third reading, every loose part within this of where it was at the first
RESTORE_CHECK_STEP = 3  # trial's third reading (the parts settle a little from their authored poses; the first
# reading after a reset can come before the simulation has stepped with the written poses)


def restore_error(b: Bodies) -> str:
    """'' if every loose part is within RESTORE_TOL of where it was at the first trial's same reading, else which part
    is how far off."""
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


def restored_in_place(b: Bodies) -> bool:
    return restore_error(b) == ""


def _like(ref: Any, values: np.ndarray, index: bool = False) -> Any:
    """Values as an array of the view's own kind (warp, torch or numpy) on its device: float32 data, int32 indices."""
    if type(ref).__module__.startswith("warp"):
        wp = importlib.import_module("warp")
        if index:
            return wp.array(values.astype(np.int32), dtype=wp.int32, device=ref.device)
        return wp.array(values.astype(np.float32), dtype=wp.float32, device=ref.device)
    if hasattr(ref, "clone"):
        torch = importlib.import_module("torch")
        return torch.as_tensor(values, dtype=torch.long if index else ref.dtype, device=ref.device)
    return values.astype(np.int32 if index else np.float32)


@dataclass
class State:
    """One trial's memory (a new trial starts a new one)."""

    latch: ButtonLatch = field(default_factory=ButtonLatch)
    prev_edge: Edge | None = None
    press: PressTimer = field(default_factory=PressTimer)
    skin_bond: bool = False
    skin_hold_n: float = 0.0
    bond_bare: float = 0.0
    bond_at: np.ndarray | None = None  # the electrode's centre (env frame) when it stuck
    liner_bond: bool = True
    liner_at: np.ndarray | None = None
    cut_total: int = 0
    last_cut: CutResult = field(default_factory=CutResult)
    drawn_cut: int = -1  # cells drawn cut (-1: redraw)
    steps: int = 0
    hazard: str = ""  # why this trial scores 0 for good (a hazard)
    void: str = ""  # why this trial is void (scores 0): its state could not be read
    held: int = 0  # control steps the 1.0 state has held
    restored: bool = False  # a new trial's parts were written back to their start (checked at its first reading)
    restore_checked: bool = False
    score: float = 0.0
    last: dict[str, Any] = field(default_factory=dict)  # the last step's readings (traces)


S = State()


def _redraw_pile() -> None:
    cut = HAIR.cut
    n = int(cut.sum())
    if n == S.drawn_cut:
        return
    rec = _record()
    px = _Pxr()
    scales = np.tile(np.array([STRAND_R, STRAND_R, PILE], dtype=np.float32), (len(cut) * STRANDS, 1))
    scales[np.repeat(cut, STRANDS), 2] = STUBBLE
    with contextlib.suppress(Exception):
        _stage().GetPrimAtPath(rec["pile"]).GetAttribute("scales").Set(px.vt.Vec3fArray.FromNumpy(scales))
        S.drawn_cut = n


def _set_bond(name: str, on: bool, hold: float, local0: np.ndarray | None = None, rot0: np.ndarray | None = None) -> None:
    rec = _record()
    path = rec[name]
    gf = importlib.import_module("pxr.Gf")
    if on and local0 is not None and rot0 is not None:
        set_attr(path, "physics:localPos0", gf.Vec3f(*(float(c) for c in local0)), "Float3")
        set_attr(path, "physics:localRot0", gf.Quatf(*wxyz(rot0)), "Quatf")
    for dof in DOFS:
        trans = dof.startswith("trans")
        set_attr(path, f"drive:{dof}:physics:maxForce", float(max(hold if trans else hold * PEEL_ARM, 1e-6)))
    set_attr(path, "physics:jointEnabled", bool(on), "Bool")


def _set_drag(force: np.ndarray) -> None:
    rec = _record()
    gf = importlib.import_module("pxr.Gf")
    set_attr(rec["clipper"], "physxForce:force", gf.Vec3f(*(float(c) for c in force)), "Float3")
    set_attr(rec["clipper"], "physxForce:forceEnabled", bool(np.linalg.norm(force) > 0), "Bool")


def button_travel(cp: np.ndarray, cr: np.ndarray, bp: np.ndarray) -> float:
    """How far the button is pressed into the clipper (m)."""
    return float(-((bp - cp) @ cr[:, 2] - (BUTTON_LOCAL[2] + 0.0025)))


def step_mechanics(b: Bodies) -> dict[str, Any]:
    """One control step of the mechanisms: the button's latch, the cut rule (and its drag), the bonds."""
    rec = _record()
    out: dict[str, Any] = {}
    cp, cr, cv = b[rec["clipper"]]
    bp, _, _ = b[rec["button"]]
    travel = button_travel(cp, cr, bp)
    motor = S.latch.update(travel)
    f_clip = _contact_force(Live.clip_contact)
    edge = edge_now(cp, cr)
    tilt = sole_tilt_deg(cr, edge.uv.mean(axis=0))
    res = cut_step(HAIR, motor, f_clip, S.prev_edge, edge, teeth_uv(cr), tilt_deg=tilt)
    S.prev_edge = edge
    S.cut_total += res.cut
    S.last_cut = res
    drag = feed_drag(res)
    v = cv[:3] - float(cv[:3] @ np.r_[0.0, 0.0, 1.0]) * np.r_[0.0, 0.0, 1.0]
    try:
        _set_drag(-drag * v / max(float(np.linalg.norm(v)), 1e-6) if drag > 0 else np.zeros(3))
    except Exception as error:  # the drag not applied would make cutting easier: never silent (the trial is void)
        Live.failed = Live.failed or f"drag: {type(error).__name__}: {error}"
    _redraw_pile()
    f_dress = _contact_force(Live.dress_contact)
    out.update(button_mm=round(travel * 1000, 3), motor=motor, toggles=S.latch.toggles, clip_n=f_clip, gap_mm=[round(float(g) * 1000, 2) for g in edge.gap],
               cut=res.cut, cut_total=S.cut_total, why=res.why, feed_mm_s=round(res.speed * 1000, 1), teeth_cos=round(res.teeth_cos, 2), drag_n=round(drag, 3),
               tilt_deg=round(tilt, 1), dress_n=f_dress, edge_uv_mm=[round(float(c) * 1000, 1) for c in edge.uv.mean(axis=0)])
    # Bonds
    ep, er, _ = b[rec["electrode"]]
    lp, lr, _ = b[rec["liner"]]
    f_el = _contact_force(Live.el_contact)
    if S.liner_bond:
        rel = lr.T @ (ep - lp)
        if S.liner_at is None:
            S.liner_at = rel
        if float(np.linalg.norm(rel - S.liner_at)) > BOND_BREAK:
            S.liner_bond = False
            _set_bond("bond_liner", False, LINER_RELEASE)
    pressed = S.press.update(f_el if not S.liner_bond else None)
    uv = to_uv(ep[:2])
    if S.skin_bond and S.bond_at is not None:
        if float(np.linalg.norm(ep - S.bond_at)) > BOND_BREAK:
            S.skin_bond = False
            _set_bond("bond_skin", False, 0.0)
    elif pressed and not S.liner_bond and float(er[:, 2] @ skin_normal(uv)) >= math.cos(math.radians(FACE_DOWN_DEG)):
        S.bond_bare = bare_fraction(HAIR, uv, el_axis_uv(er))
        S.skin_hold_n = skin_hold(S.bond_bare)
        S.skin_bond, S.bond_at = True, ep.copy()
        _set_bond("bond_skin", True, S.skin_hold_n, local0=ep, rot0=er)  # the torso's frame is the world (env) frame
    out.update(el_n=f_el, liner_bond=S.liner_bond, skin_bond=S.skin_bond, hold_n=round(S.skin_hold_n, 3), bare=round(S.bond_bare, 3),
               press_s=round(S.press.held, 2), el_mm=[round(float(c) * 1000, 1) for c in ep], el_uv_mm=[round(float(c) * 1000, 1) for c in uv])
    return out


def el_axis_uv(er: np.ndarray) -> np.ndarray:
    """The electrode's long side (its x) in body coordinates."""
    return np.array([er[:2, 0] @ AXIS, er[:2, 0] @ ACROSS])


# =====================================================================================================================
# Scoring (README "Score" and task.json "scoring" say exactly this)
# =====================================================================================================================
PLACE_TOL = 0.015  # the electrode's centre (its stud) within this of the site's mark (the fur square's centre)
BARE_NEED = 0.95  # share of an outline that must lie over clipped pile or skin outside the fur
CLEAR_CAP_CM2 = 70.0  # est: the clipped area may not exceed this: about 5 times the electrode's outline (13.7 cm^2); the
# demo clips about 45 (90 was first used, but the arm can reach only about 97 of the square, so it could hardly be exceeded)
HOLD_STEPS = 20  # the 1.0 state held for 1 s (20 control steps at 20 Hz)
HAND_CLEAR = 0.030  # every finger point at least this far from the clipper, the electrode and the liner card
STILL_V, STILL_W = 0.010, 0.5  # m/s and rad/s: the clipper, its button, the electrode and the card all slower
CARD_DOWN = 0.006  # the card lies on the bench: every corner within this of the bench top
TRAY_DOWN = 0.010  # the clipper lies on its tray: its lowest point within this of the tray's top
DRESS_TOUCH_N = 0.05  # N: the running clipper touching a placed electrode or a lead (a hazard: it can cut a lead)


def clipper_points(pos: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """The clipper's outline points (env frame): the body's eight corners, the blade's sole corners and its teeth."""
    body = [(x, y, z) for x in (-CLIP_LEN, -HEAD_TAPER) for y in (-CLIP_W / 2, CLIP_W / 2) for z in (BODY_LIFT, BODY_LIFT + CLIP_T)]
    sole = [tuple(SOLE_ROT @ np.array([x, y, 0.0])) for x in (-SOLE, 0.0) for y in (-BLADE_W / 2, BLADE_W / 2)]
    local = np.array(body + sole, dtype=float)
    return np.asarray(pos + local @ rot.T, dtype=float)


def on_tray(pos: np.ndarray, rot: np.ndarray) -> bool:
    """Seen from above, every outline point of the clipper inside the tray's edges, and its lowest point within
    TRAY_DOWN of the tray's top (it lies on the tray)."""
    pts = clipper_points(pos, rot)
    lo = np.array(TRAY_AT) - np.array(TRAY[:2]) / 2
    hi = np.array(TRAY_AT) + np.array(TRAY[:2]) / 2
    inside = bool(np.all(pts[:, :2] >= lo) and np.all(pts[:, :2] <= hi))
    return inside and float(pts[:, 2].min()) <= TOP + TRAY[2] + TRAY_DOWN


def card_down(pos: np.ndarray, rot: np.ndarray) -> bool:
    """Every corner of the liner card within CARD_DOWN of the bench top: it lies on the bench, not on the patient."""
    corners = np.array([(x, y, z) for x in (-LINER[0] / 2, LINER[0] / 2) for y in (-LINER[1] / 2, LINER[1] / 2) for z in (-LINER[2] / 2, LINER[2] / 2)])
    return bool(np.all(pos[2] + corners @ rot[2] <= TOP + CARD_DOWN))


# The parts' boxes for the hands' clearance (own frames: low and high corners).
CLIP_BOX = (np.array([-CLIP_LEN, -CLIP_W / 2, -0.007]), np.array([0.004, CLIP_W / 2, BODY_LIFT + CLIP_T + 0.005]))
EL_BOX = (np.array([-EL_SIZE[0] / 2, -EL_SIZE[1] / 2, -EL_T / 2]), np.array([EL_SIZE[0] / 2, EL_SIZE[1] / 2, EL_T / 2 + SNAP[2]]))
CARD_BOX = (-np.array(LINER) / 2, np.array(LINER) / 2)


def box_gap(p: np.ndarray, pos: np.ndarray, rot: np.ndarray, box: tuple[np.ndarray, np.ndarray]) -> float:
    """Distance from p to a box given in a body's own frame (0 inside)."""
    q = rot.T @ (p - pos)
    gap = np.maximum(np.maximum(box[0] - q, q - box[1]), 0.0)
    return float(np.linalg.norm(gap))


PAD_LENGTH, PAD_WIDTH = 0.030, 0.027


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
        for finger in (-0.5, 0.5):
            for along in (PAD_TIP, PAD_TIP - PAD_LENGTH / 2, PAD_TIP - PAD_LENGTH):
                for edge in (-0.5, 0.0, 0.5):
                    points.append(self.tcp + finger * self.opening * self.closing + along * self.approach + edge * PAD_WIDTH * across)
        return points


def hands_clear(b: Bodies, grippers: Sequence[Gripper]) -> bool:
    rec = _record()
    parts = ((rec["clipper"], CLIP_BOX), (rec["electrode"], EL_BOX), (rec["liner"], CARD_BOX))
    for g in grippers:
        for p in g.finger_points():
            for path, box in parts:
                pos, rot, _ = b[path]
                if box_gap(p, pos, rot, box) < HAND_CLEAR:
                    return False
    return True


@dataclass(frozen=True)
class Snapshot:
    """What the scorer reads at one control step."""

    motor_on: bool
    on_tray: bool
    card_down: bool
    cleared_cm2: float
    site_bare: float  # share of the site's outline over clipped pile
    stuck: bool  # the electrode is stuck to the chest (it was pressed on and has not been pulled off)
    el_bare: float  # share of the electrode's outline over clipped pile or skin, when it stuck
    el_off: float  # its centre's distance from the site's mark (m)
    still: bool
    clear: bool

    @property
    def placed(self) -> bool:
        return self.stuck and self.el_bare >= BARE_NEED and self.el_off <= PLACE_TOL

    @property
    def tidy(self) -> bool:
        return not self.motor_on and self.on_tray and self.card_down and self.cleared_cm2 <= CLEAR_CAP_CM2

    @property
    def site_clear(self) -> bool:
        return self.site_bare >= BARE_NEED


def grade(now: Snapshot, st: State) -> float:
    """The ladder (README, "Score"). 0 for the whole trial after a hazard, or if the trial's state could not be read."""
    if st.void or st.hazard:
        st.held = 0
        return 0.0
    st.held = st.held + 1 if now.placed and now.tidy and now.still and now.clear else 0
    if st.held >= HOLD_STEPS:
        return 1.0
    if now.placed:
        return 0.75
    if now.site_clear:
        return 0.5 if now.tidy else 0.25
    return 0.0


def snapshot(b: Bodies, grippers: Sequence[Gripper] | None, motor_on: bool) -> Snapshot:
    rec = _record()
    cp, cr, _ = b[rec["clipper"]]
    ep, _, _ = b[rec["electrode"]]
    lp, lr, _ = b[rec["liner"]]
    still = all(float(np.linalg.norm(v[:3])) < STILL_V and float(np.linalg.norm(v[3:])) < STILL_W for _, _, v in b.values())
    return Snapshot(
        motor_on=motor_on,
        on_tray=on_tray(cp, cr),
        card_down=card_down(lp, lr),
        cleared_cm2=float(HAIR.cut.sum()) * (CELL * 100) ** 2,
        site_bare=bare_fraction(HAIR, np.zeros(2), np.array([1.0, 0.0])),
        stuck=S.skin_bond,
        el_bare=S.bond_bare if S.skin_bond else 0.0,
        el_off=float(np.linalg.norm(ep[:2] - SITE)),
        still=still,
        clear=grippers is not None and hands_clear(b, grippers),
    )


# --- trials and the robot's state ------------------------------------------------------------------------------------
_last_objects: object = None
_embodiment: Any = None
_last_clock: float | None = None
_hooked = False
_warned: set[str] = set()
_ARMS = (("left_robot", "left"), ("right_robot", "right"))
RESET_TAG = "clip_chest_fit_holter_electrode_resets"


def _warn_once(key: str, message: str) -> None:
    if key not in _warned:
        _warned.add(key)
        with contextlib.suppress(Exception):
            warnings.warn(f"clip_chest_fit_holter_electrode: {message}", RuntimeWarning, stacklevel=3)


def new_episode() -> None:
    """A new trial: fresh memory, the fur uncut, every loose body back at its start, the bonds and the drag as built."""
    global S, _last_objects, _last_clock
    S = State()
    HAIR.height[:] = PILE
    Live.last = None  # the demo's live targets must not read the last trial's parts
    _last_objects = None
    _last_clock = None
    _warned.difference_update(("robot", "void", "restore"))
    if not _record().get("built"):
        return
    try:
        restore_bodies()
        S.restored = True
        _set_bond("bond_skin", False, 0.0)
        _set_bond("bond_liner", True, LINER_RELEASE)
        _set_drag(np.zeros(3))
        _redraw_pile()
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
    """Task files get no reset hook, so hook the embodiment's reset once: every new trial calls new_episode()."""
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


def _update(objects: Any) -> float:
    """Advance the trial once per control step (success() and score() share the step)."""
    global _last_objects
    if objects is _last_objects:
        return S.score
    _last_objects = objects
    with contextlib.suppress(Exception):
        _hook_resets()
    _restart_if_clock_went_back()
    S.steps += 1
    grippers = _robot()
    if grippers is None:
        S.void = S.void or "the arms' state could not be read"
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
        if not S.restored:  # the first trial: where its parts are now (they settle from their authored poses) is the start
            Live.start = {path: np.asarray(b[path][0], dtype=float).copy() for path in b}
        else:
            off = restore_error(b)
            if off:
                S.void = S.void or f"a new trial's parts were not back at their start ({off})"
    try:
        m = step_mechanics(b)
        now = snapshot(b, grippers, bool(m["motor"]))
    except Exception as error:
        S.void = S.void or f"the parts could not be read ({type(error).__name__}: {error})"
        S.score = 0.0
        return 0.0
    if m["dress_n"] is None or Live.failed:  # a reading that failed is never taken as safe
        S.void = S.void or f"a contact reading or a write failed ({Live.failed or 'clipper on the placed electrodes and leads'})"
    elif m["motor"] and m["dress_n"] >= DRESS_TOUCH_N:
        S.hazard = S.hazard or "the running clipper touched a placed electrode or a lead"
    if S.void:
        _warn_once("void", f"this trial is void (0): {S.void}")
    S.score = grade(now, S)
    S.last = {**m, **{k: getattr(now, k) for k in now.__dataclass_fields__}, "placed": now.placed, "tidy": now.tidy, "site_clear": now.site_clear,
              "held": S.held, "hazard": S.hazard, "void": S.void, "score": S.score}
    return S.score


def score(objects: Any) -> float:
    """1.0: the electrode stuck on bare skin at the site and a tidy finish, still and hands clear for 1 s; 0.75 the
    electrode stuck on bare skin at the site; 0.5 the site clipped and a tidy finish; 0.25 the site clipped; see README.md."""
    return _update(objects)


def success(objects: Any) -> bool:
    return _update(objects) >= 1.0


# =====================================================================================================================
# The scripted demo: both arms (it moves only the arms; one live target reads the button where it lies)
# =====================================================================================================================
class Grip(float):
    """A Waypoint yaw that carries a full TCP rotation (3x3, columns: closing axis, across, approach)."""

    rot: np.ndarray

    def __new__(cls, rot: np.ndarray) -> "Grip":
        obj = super().__new__(cls, 0.0)
        obj.rot = np.asarray(rot, dtype=float)
        return obj


GRIP_TAG = "clip_chest_fit_holter_electrode_grip"


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
    demo_mod.grip_rotation = grip_rotation


class LivePos:
    """A waypoint position worked out every control step (from the parts' last readings, or a fixed fallback)."""

    def __init__(self, fn: Callable[[], np.ndarray]) -> None:
        self.fn = fn

    def __array__(self, dtype: Any = None, copy: Any = None) -> np.ndarray:
        return np.asarray(self.fn(), dtype=dtype)

    def __len__(self) -> int:
        return 3

    def __getitem__(self, index: Any) -> Any:
        return tuple(float(v) for v in self.fn())[index]

    def __iter__(self) -> Iterator[float]:
        return iter(tuple(float(v) for v in self.fn()))


def finger(gap: float) -> float:
    """Finger joint target for a pad gap (m): 0 closed (1.1 mm), FINGER_OPEN = 95 mm."""
    return float(np.clip(-(gap - 0.0011) / (0.095 - 0.0011) * abs(FINGER_OPEN), FINGER_OPEN, 0.0))


CLOSED = 0.0
# --- the right hand and the clipper (an early GPU test: button 1.77 mm and latch on; pinched, lifted 86 mm; seated
# 51/51 pull steps at 1.4-3.0 N; one swath of 25.8 cm^2) -------------------------------------------------------------
BUTTON_PAST = 0.015  # the closed fingertips are sent this far past the button's top: the arm's give sets the push
PINCH_GAP = 0.054  # the pads' target gap on the 60 mm body (3 mm a side)
OVERTRAVEL = 0.008  # the cutting edge sent this far into the skin along its normal (the arm's give seats the blade)
PULL_SPEED = 0.027  # m/s along each pass
LIFT_START = 0.012  # the edge this far over the skin before a pass (30 mm puts joint 4 at its limit for the far pass)
LIFT = 0.030  # and this far after it
CARRY_UP = 0.080  # the clipper lifted this far off its place on the tray
CARRY_HIGH = 0.130  # then raised this high over its place (official IK: higher runs out of reach) and carried to
# OVER_PATCH over the patch (a straight carry from 80 mm skimmed the abdomen with the running blade on the GPU)
OVER_PATCH = 0.060  # the cutting edge this far over the patch's near end before and after the passes
TRAY_SET = 0.003  # set down this far over where it lay
UP = 0.060
TEETH = np.array([-1.0, 0.0, 0.0])  # the teeth point at the robot: each pass pulls the clipper toward it
DOWN = np.diag([-1.0, -1.0, 1.0]) @ _tilted(0.0, 0.0)  # a straight-down grip (closed fingers: either roll)
TAB_PRESS = 0.006  # the closed fingertips sent this far into the card's tab (pins it while the electrode is peeled)
TAB_IN = 0.008  # from the card's end (the left hand's fingers are at the snap, 37 mm away)
TAB_TILT, TAB_AZ = 35.0, 90.0  # the right hand's fingertips point toward +y, so the two hands' bodies are on either side
# --- the left hand and the electrode -----------------------------------------------------------------------------------
EL_TILT, EL_AZ = 30.0, -90.0  # its approach: tilted from straight down toward -y (official IK: 28 deg joint margin)
SNAP_OPEN = 0.030  # the pads open round the 11 mm snap connector
SNAP_SQUEEZE = 0.006  # and close to this (the 11 mm snap stops them: a firm pinch)
SNAP_DOWN = -0.001  # the pads' tips this far below the snap's middle (2 mm below: the tilted fingertips' low corners
# met the electrode's pad and pushed electrode and card 11 mm before the pinch, on the GPU)
PEEL_UP = 0.040  # the peel: straight up
SET_OVER = 0.003  # the electrode set onto the site this far into the skin
# The presses: the fingers open PRESS_OPEN either side of the snap connector, so their two tips land on the pad beside
# it; the approach is tilted toward the hips, so each fingertip's lowest corner (toward the head) is what touches,
# 6-12 mm toward the head from the tip's middle (the tip is 15-27 mm wide). With the tips' middles at EDGE_AT along the
# body from the electrode's centre, they press its head half (about 6-12 mm from the centre) and then its hip half
# (about 8-14 mm from it), either side of the snap. (On the GPU closed fingertips, tilted, were held off by the snap and
# by the skin beyond the pad's end, 25 mm over the skin.)
PRESS_OPEN = 0.018
EDGE_AT = (0.0, -0.020)
EDGE_OVER = 0.022  # the fingertips sent this far down past where they touch the pad, at each press (on the GPU 16 mm
# along the tilted approach gave 2.7-2.8 N, under the 3 N that makes it stick)
EDGE_HOLD = 1.4  # s at each press (the bond needs 3 N for 1 s)
TAB_VIA = (0.150, -0.400, 0.150)  # the right hand goes round the hips' end to the card (over the bench top)
HOLD_UP = 3.0  # s the left hand holds the peeled electrode up while the right hand leaves the card


def snap_grip() -> np.ndarray:
    """The left hand's TCP rotation for the snap connector: approach tilted EL_TILT toward EL_AZ, closing across the
    snap's 11 mm faces (the electrode's y)."""
    return el_grip_rot(EL_TILT, EL_AZ)


def el_grip_rot(tilt: float, az: float) -> np.ndarray:
    t, a = math.radians(tilt), math.radians(az)
    approach = np.array([math.sin(t) * math.cos(a), math.sin(t) * math.sin(a), -math.cos(t)])
    closing = el_frame0()[:, 1]
    closing = closing - float(closing @ approach) * approach
    closing /= float(np.linalg.norm(closing))
    return np.column_stack([closing, np.cross(approach, closing), approach])


def el_frame0() -> np.ndarray:
    """The electrode's frame on its card (its x along the 40 mm side)."""
    return card_rot()


def snap_centre0() -> np.ndarray:
    on_card = card_rot() @ np.array([EL_ON_CARD, 0.0, 0.0])
    return np.array([CARD_AT[0] + on_card[0], CARD_AT[1] + on_card[1], TOP + LINER[2] + EL_T + SNAP[2] / 2])


def tcp_for_snap(snap: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """The TCP that puts the pads' tips SNAP_DOWN below the snap's middle (pads either side of it)."""
    return np.asarray(snap - rot[:, 2] * (PAD_TIP - SNAP_DOWN), dtype=float)


def clip_at(uv: np.ndarray, lift: float) -> tuple[np.ndarray, np.ndarray]:
    """The right hand's TCP and rotation with the cutting edge at uv, `lift` along the skin's normal (negative: into
    it), the body CLIP_PITCH above the skin, teeth toward the robot."""
    n = skin_normal(uv)
    e = surface_point(uv) + n * lift
    return grip_tcp(e, clipper_frame(e, n, TEETH, CLIP_PITCH))


def tray_pinch() -> tuple[np.ndarray, np.ndarray]:
    edge0 = np.array([CLIPPER_AT[0], CLIPPER_AT[1], TOP + TRAY[2] + SOLE * math.sin(math.radians(HEAD_ANGLE))])
    return grip_tcp(edge0, np.diag([-1.0, -1.0, 1.0]))


def button_top0() -> np.ndarray:
    edge0 = np.array([CLIPPER_AT[0], CLIPPER_AT[1], TOP + TRAY[2] + SOLE * math.sin(math.radians(HEAD_ANGLE))])
    return np.asarray(edge0 + np.diag([-1.0, -1.0, 1.0]) @ (BUTTON_LOCAL + (0.0, 0.0, 0.005)), dtype=float)


def button_top_now() -> np.ndarray:
    """The button's top where it lies now (the demo may read the simulator's state), else where it started."""
    b = Live.last
    rec = _record()
    if b is not None and rec.get("button") in b:
        bp, br, _ = b[rec["button"]]
        top = bp + br[:, 2] * 0.0025
        if np.all(np.isfinite(top)) and float(np.linalg.norm(top[:2] - button_top0()[:2])) < 0.03:
            return np.asarray(top, dtype=float)
    return button_top0()


def button_press_tcp(past: float) -> np.ndarray:
    return np.asarray(button_top_now() + np.array([0.0, 0.0, PAD_TIP - past]), dtype=float)


def card_tab() -> np.ndarray:
    """Where the right hand pins the card's bare tab: TAB_IN from the card's end."""
    d = card_rot() @ np.array([-LINER[0] / 2 + TAB_IN, 0.0, 0.0])
    return np.array([CARD_AT[0] + d[0], CARD_AT[1] + d[1], TOP + LINER[2]])


# The blade kept seated (the demo may read the simulator's state; it only moves the arm). On the GPU the clipper pitched
# in the pinch during the first pass (the feed drag 90 mm ahead of the pads), so an open-loop second pass rode 1.4-3.8 mm
# over the skin and cut nothing; and the edge landed 10-18 mm nearer the robot than planned (the clipper sits in the
# pinch a little differently each time), so the second pass began inside the site. During each pass the servo lowers
# the hand along the skin's normal until the cutting edge's mean gap is SERVO_GAP, and turns the clipper's pitch until
# its sole is TILT_GOAL off flat, heel up; over and on the skin before each pull it moves the hand until the edge's
# middle is where the plan puts it (during the pull, only across the pull).
SERVO_GAP = 0.0003
SERVO_GAIN = 0.3  # of the error per control step
DZ_STEP, DZ_MIN, DZ_MAX = 0.0005, -0.004, 0.012  # m: per step, and the range beyond OVERTRAVEL
TILT_GOAL, DP_GAIN, DP_STEP, DP_MAX = 5.0, 0.1, 0.3, 10.0  # deg
UV_GAIN, UV_STEP, UV_MAX = 0.3, 0.001, 0.020  # m


def heel_up_deg(rot: np.ndarray, uv: Any) -> float:
    """The sole's tilt from the skin about the cutting edge (deg): positive with the heel up (the edge down)."""
    n = skin_normal(np.asarray(uv, dtype=float))
    ns = rot @ SOLE_ROT[:, 2]
    d = rot[:, 0] - float(rot[:, 0] @ n) * n
    d /= max(float(np.linalg.norm(d)), 1e-9)
    return math.degrees(math.atan2(float(ns @ d), float(ns @ n)))


class ClipServo:
    """Watches the clipper during the passes and corrects the right hand's live targets (depth and pitch)."""

    def __init__(self) -> None:
        self.phases: list[tuple[float, float, str, np.ndarray]] = []
        self.restart()

    def restart(self) -> None:
        self.tick = -1
        self.dz = 0.0
        self.dp = 0.0
        self.duv = np.zeros(2)
        self.mode = ""
        self.seen: dict[str, Any] = {}

    def follow(self, plan: Sequence[Waypoint]) -> None:
        start = 0.0
        self.phases = []
        for w in plan:
            ours = getattr(w.pos, "servo", None) is self
            mode = getattr(w.pos, "mode", "") if ours else ""
            uv = np.asarray(getattr(w.pos, "uv", np.zeros(2)), dtype=float) if ours else np.zeros(2)
            self.phases.append((start, start + float(w.duration), mode, uv))
            start += float(w.duration)

    def update(self) -> None:
        steps = S.steps
        if steps == self.tick:
            return
        if steps < self.tick:
            self.restart()
        self.tick = steps
        t = steps * DT
        mode, frac, uv = "", 0.0, np.zeros(2)
        for t0, t1, m, w_uv in self.phases:
            if t0 <= t < t1:
                mode, frac, uv = m, (t - t0) / max(t1 - t0, 1e-9), w_uv
                break
        self.mode = mode
        if mode == "over":
            self.dz = 0.0
        if mode not in ("over", "seat", "pull") or (mode == "over" and frac < 0.5):
            return
        b = Live.last
        path = _record().get("clipper")
        if b is None or path not in b:
            return
        cp, cr, _ = b[path]
        e = edge_now(cp, cr)
        mid = e.uv.mean(axis=0)
        gap, heel = float(np.mean(e.gap)), heel_up_deg(cr, mid)
        if not (math.isfinite(gap) and math.isfinite(heel) and np.all(np.isfinite(mid))):
            return
        err = uv - mid
        if mode == "pull":
            err[1] = 0.0  # along the pull the plan rules; across it, the edge is held on its line
        self.duv = np.clip(self.duv + np.clip(UV_GAIN * err, -UV_STEP, UV_STEP), -UV_MAX, UV_MAX)
        if mode == "over" or (mode == "seat" and frac < 0.6):
            return
        self.dz = float(np.clip(self.dz + np.clip(SERVO_GAIN * (gap - SERVO_GAP), -DZ_STEP, DZ_STEP), DZ_MIN, DZ_MAX))
        if mode == "pull":
            self.dp = float(np.clip(self.dp + np.clip(DP_GAIN * (heel - TILT_GOAL), -DP_STEP, DP_STEP), -DP_MAX, DP_MAX))
        self.seen = {"servo_gap_mm": round(gap * 1000, 2), "heel_deg": round(heel, 1)}

    def frame(self, uv: np.ndarray, lift: float, deep: bool) -> tuple[np.ndarray, np.ndarray]:
        self.update()
        uv = np.asarray(uv, dtype=float) + self.duv
        n = skin_normal(uv)
        e = surface_point(uv) + n * (lift - (self.dz if deep else 0.0))
        return grip_tcp(e, clipper_frame(e, n, TEETH, CLIP_PITCH - self.dp))

    def report(self) -> dict[str, Any]:
        return {"servo": self.mode, "dz_mm": round(self.dz * 1000, 2), "dp_deg": round(self.dp, 2),
                "duv_mm": [round(float(c) * 1000, 1) for c in self.duv], **self.seen}


class ClipPos(LivePos):
    """A pass's waypoint: the TCP for the cutting edge at uv, `lift` along the normal, with the servo's corrections."""

    def __init__(self, servo: ClipServo, uv: np.ndarray, lift: float, mode: str) -> None:
        self.servo, self.uv, self.lift, self.mode = servo, np.asarray(uv, dtype=float), lift, mode
        super().__init__(lambda: self.servo.frame(self.uv, self.lift, self.mode in ("seat", "pull"))[0])


class ClipGrip(float):
    servo: ClipServo
    uv: np.ndarray

    def __new__(cls, servo: ClipServo, uv: np.ndarray) -> "ClipGrip":
        obj = super().__new__(cls, 0.0)
        obj.servo, obj.uv = servo, np.asarray(uv, dtype=float)
        return obj

    @property
    def rot(self) -> np.ndarray:
        return self.servo.frame(self.uv, 0.0, False)[1]


SERVO: ClipServo | None = None  # the last plan's servo (traces read it)


@dataclass(frozen=True)
class Options:
    """The routine's choices (the demo uses the defaults; the negative tests change them)."""

    active: bool = True  # False: both arms hold still (doing nothing)
    clip: bool = True  # False: the clipper is never used (the electrode goes on the fur)
    press_on: bool = True  # False: the button is never pressed on (the motor stays off)
    press_off: bool = True  # False: the clipper is put back running
    passes_u: tuple[float, ...] = PASSES_U
    pass_v: tuple[float, float] = PASS_V
    air_s: float = 0.0  # extra seconds holding the running clipper in the air before the first pass
    set_over: float = SET_OVER
    edge_over: float = EDGE_OVER


def _hold(seconds: float, last: Waypoint, label: str) -> Waypoint:
    return Waypoint(seconds, last.pos, last.finger, label, last.yaw)


def right_plan(o: Options) -> list[Waypoint]:
    """Take the clipper off the tray, switch it on, clip the site in pulls toward the robot, put it back, switch it
    off; then pin the liner card's tab while the left hand peels the electrode."""
    install_grip_rotation()
    pinch, prot = tray_pinch()
    gd, gp = Grip(DOWN), Grip(prot)
    up = np.array([0.0, 0.0, UP])
    held, opened = finger(PINCH_GAP), FINGER_OPEN
    out: list[Waypoint] = []
    if o.clip:
        if o.press_on:
            press = button_press_tcp(BUTTON_PAST)
            out += [
                Waypoint(2.5, tuple(press + up), CLOSED, "over the button", gd),
                Waypoint(1.5, tuple(press), CLOSED, "press the button (on)", gd),
                Waypoint(0.8, tuple(press), CLOSED, "hold the press", gd),
                Waypoint(1.2, tuple(press + up), CLOSED, "up off the button", gd),
            ]
        out += [
            Waypoint(1.0 if o.press_on else 3.5, tuple(pinch + up), opened, "open, over the pinch", gp),
            Waypoint(2.0, tuple(pinch), opened, "down round the clipper", gp),
            Waypoint(1.0, tuple(pinch), held, "pinch", gp),
            Waypoint(2.0, tuple(pinch + (0.0, 0.0, CARRY_UP)), held, "lift the clipper", gp),
        ]
        if o.air_s > 0:
            out.append(_hold(o.air_s, out[-1], "running in the air"))
        v0, v1 = o.pass_v
        pull_s = (v1 - v0) / PULL_SPEED
        servo = ClipServo()
        u_first, u_last = o.passes_u[0], o.passes_u[-1]
        out += [
            Waypoint(1.5, tuple(pinch + (0.0, 0.0, CARRY_HIGH)), held, "up over the tray", gp),
            Waypoint(2.5, ClipPos(servo, np.array([u_first, v1]), OVER_PATCH, "over"), held, "over the patch", ClipGrip(servo, np.array([u_first, v1]))),
        ]
        for k, u in enumerate(o.passes_u):
            a, z = np.array([u, v0]), np.array([u, v1])
            out += [
                Waypoint(1.5, ClipPos(servo, a, LIFT_START, "over"), held, f"over pass {k + 1}", ClipGrip(servo, a)),
                Waypoint(1.5, ClipPos(servo, a, -OVERTRAVEL, "seat"), held, f"seat for pass {k + 1}", ClipGrip(servo, a)),
                Waypoint(pull_s, ClipPos(servo, z, -OVERTRAVEL, "pull"), held, f"pass {k + 1}: pull toward the robot", ClipGrip(servo, z)),
                Waypoint(1.0, ClipPos(servo, z, LIFT, "lift"), held, f"lift after pass {k + 1}", ClipGrip(servo, z)),
            ]
        out += [
            Waypoint(1.0, ClipPos(servo, np.array([u_last, v1]), OVER_PATCH, "lift"), held, "up off the patch", ClipGrip(servo, np.array([u_last, v1]))),
            Waypoint(2.5, tuple(pinch + (0.0, 0.0, CARRY_HIGH)), held, "carry back over the tray", gp),
            Waypoint(2.0, tuple(pinch + (0.0, 0.0, CARRY_UP)), held, "down over the tray", gp),
            Waypoint(2.0, tuple(pinch + (0.0, 0.0, TRAY_SET)), held, "set it on the tray", gp),
            Waypoint(0.8, tuple(pinch + (0.0, 0.0, TRAY_SET)), opened, "let go", gp),
            Waypoint(1.0, tuple(pinch + up), opened, "up off the clipper", gp),
        ]
        if o.press_off:
            out += [
                Waypoint(1.2, LivePos(lambda: button_press_tcp(BUTTON_PAST) + up), CLOSED, "over the button", gd),
                Waypoint(1.5, LivePos(lambda: button_press_tcp(BUTTON_PAST)), CLOSED, "press the button (off)", gd),
                Waypoint(0.8, LivePos(lambda: button_press_tcp(BUTTON_PAST)), CLOSED, "hold the press", gd),
                Waypoint(1.0, LivePos(lambda: button_press_tcp(BUTTON_PAST) + up), CLOSED, "up off the button", gd),
            ]
    tab_rot = el_grip_rot(TAB_TILT, TAB_AZ) @ np.diag([-1.0, -1.0, 1.0])  # closed fingers: this twin reaches (22 deg margin)
    gt = Grip(tab_rot)
    tab = card_tab() - tab_rot[:, 2] * PAD_TIP
    out += [
        Waypoint(1.5, tuple(np.array(TAB_VIA) + (0.0, 0.0, TOP)), CLOSED, "round the hips' end", gd),
        Waypoint(2.0, tuple(tab + up), CLOSED, "over the card's tab", gt),
        Waypoint(1.5, tuple(tab - (0.0, 0.0, TAB_PRESS)), CLOSED, "pin the card's tab", gt),
    ]
    return out


RIGHT_PEEL_WAIT = 4.0  # s the right hand pins the tab while the left hand takes and peels the electrode


class ElServo:
    """Watches the electrode (the demo may read the simulator's state; it only moves the arm): before the pinch, where
    its snap connector lies (the card can shift when the other hand pins it: 11 mm on the GPU); while the hand holds it
    up still, how far it sits from where the pinch put it (its grip offset); after it is let go, where it landed. The
    pinch is aimed at the snap, the hand's targets over the site are shifted by the grip offset, and the corner presses
    are aimed round the electrode where it lies."""

    def __init__(self) -> None:
        self.phases: list[tuple[float, float, str]] = []
        self.restart()

    def restart(self) -> None:
        self.tick = -1
        self.pick = np.zeros(2)
        self.shift = np.zeros(2)
        self.landed = np.zeros(2)
        self.mode = ""

    def follow(self, plan: Sequence[Waypoint]) -> None:
        start = 0.0
        self.phases = []
        for w in plan:
            self.phases.append((start, start + float(w.duration), str(getattr(w.pos, "mode", ""))))
            start += float(w.duration)

    def update(self) -> None:
        steps = S.steps
        if steps == self.tick:
            return
        if steps < self.tick:
            self.restart()
        self.tick = steps
        t = steps * DT
        self.mode = next((m for t0, t1, m in self.phases if t0 <= t < t1), "")
        if self.mode not in ("pick", "measure", "landed"):
            return
        b = Live.last
        path = _record().get("electrode")
        if b is None or path not in b:
            return
        ep = b[path][0]
        if not np.all(np.isfinite(ep)):
            return
        if self.mode == "pick":
            self.pick = np.clip(ep[:2] - snap_centre0()[:2], -0.020, 0.020)
        elif self.mode == "measure":
            self.shift = np.clip(ep[:2] - snap_centre0()[:2] - self.pick, -0.020, 0.020)
        else:
            self.landed = np.clip(to_uv(ep[:2]), -0.020, 0.020)

    def report(self) -> dict[str, Any]:
        return {"el_servo": self.mode, "pick_mm": [round(float(c) * 1000, 1) for c in self.pick], "grip_shift_mm": [round(float(c) * 1000, 1) for c in self.shift],
                "landed_uv_mm": [round(float(c) * 1000, 1) for c in self.landed]}


class ElPos(LivePos):
    def __init__(self, servo: ElServo, fn: Callable[[ElServo], np.ndarray], mode: str = "") -> None:
        self.servo, self.mode = servo, mode

        def value() -> np.ndarray:
            servo.update()
            return np.asarray(fn(servo), dtype=float)

        super().__init__(value)


EL_SERVO: ElServo | None = None  # the last plan's electrode servo (traces read it)


def left_plan(o: Options, start: float) -> list[Waypoint]:
    """From `start` (s; the right hand is pinning the card): take the electrode by its lead's snap connector, peel it
    off its liner, set it on the site, let go, press its head half and then its hip half with the fingertips opened
    either side of the snap, and move clear."""
    global EL_SERVO
    install_grip_rotation()
    rot = snap_grip()
    g = Grip(rot)
    snap = snap_centre0()
    at_card = tcp_for_snap(snap, rot)
    site = surface_point(np.zeros(2))
    snap_site = site + np.array([0.0, 0.0, EL_T + SNAP[2] / 2])
    at_site = tcp_for_snap(snap_site, rot)
    up = np.array([0.0, 0.0, 1.0])
    park, park_rot = ready_pose("left")
    sv = ElServo()
    EL_SERVO = sv

    def over_site(lift: np.ndarray) -> Callable[[ElServo], np.ndarray]:
        return lambda s_: at_site + lift - np.r_[s_.shift, 0.0]

    def on_card(lift: np.ndarray) -> Callable[[ElServo], np.ndarray]:
        return lambda s_: at_card + lift + np.r_[s_.pick, 0.0]

    held, opened = finger(SNAP_SQUEEZE), finger(SNAP_OPEN)
    out = [
        Waypoint(max(start - 1.5, 0.5), tuple(park), FINGER_OPEN, "wait clear", Grip(park_rot)),
        Waypoint(2.5, ElPos(sv, on_card(up * 0.040), "pick"), opened, "over the snap connector", g),
        Waypoint(1.5, ElPos(sv, on_card(up * 0.0), "pick"), opened, "down round the snap", g),
        Waypoint(1.0, ElPos(sv, on_card(up * 0.0)), held, "pinch the snap", g),
        Waypoint(1.5, ElPos(sv, on_card(up * PEEL_UP)), held, "peel it off the liner", g),
        Waypoint(1.0, ElPos(sv, on_card(up * 0.080)), held, "lift", g),
        Waypoint(HOLD_UP, ElPos(sv, on_card(up * 0.080), "measure"), held, "hold it up", g),
        Waypoint(3.0, ElPos(sv, over_site(up * 0.030)), held, "over the site", g),
        Waypoint(1.5, ElPos(sv, over_site(rot[:, 2] * o.set_over)), held, "set it on the site", g),
        Waypoint(0.8, ElPos(sv, over_site(rot[:, 2] * o.set_over)), opened, "let go", g),
        Waypoint(0.8, ElPos(sv, over_site(up * 0.025), "landed"), opened, "up", g),
        Waypoint(0.5, ElPos(sv, over_site(up * 0.025), "landed"), finger(PRESS_OPEN), "fingers either side of the snap", g),
    ]

    def edge(du: float, extra: np.ndarray) -> Callable[[ElServo], np.ndarray]:
        def at(s_: ElServo) -> np.ndarray:
            uv = s_.landed + np.array([du, 0.0])
            return np.asarray(surface_point(uv) + skin_normal(uv) * (EL_T + SNAP[2] / 2) - rot[:, 2] * (PAD_TIP - SNAP_DOWN) + extra, dtype=float)
        return at

    pressing = finger(PRESS_OPEN)
    for du, end in zip(EDGE_AT, ("head", "hip"), strict=True):
        out += [
            Waypoint(0.8, ElPos(sv, edge(du, up * 0.015)), pressing, f"over the electrode's {end} end", g),
            Waypoint(0.8, ElPos(sv, edge(du, -up * o.edge_over)), pressing, f"press its {end} end", g),
            Waypoint(EDGE_HOLD, ElPos(sv, edge(du, -up * o.edge_over)), pressing, f"press its {end} end: hold", g),
            Waypoint(0.5, ElPos(sv, edge(du, up * 0.015)), pressing, "up", g),
        ]
    out += [
        Waypoint(1.0, tuple(site + up * 0.090 - rot[:, 2] * PAD_TIP), CLOSED, "clear of the chest", g),
        Waypoint(2.0, tuple(park), FINGER_OPEN, "park", Grip(park_rot)),
    ]
    sv.follow(out)
    return out


def routine(o: Options | None = None) -> dict[str, list[Waypoint]]:
    o = o or Options()
    install_grip_rotation()
    if not o.active:
        return {side: [Waypoint(5.0, tuple(_ready_tcp(side)), FINGER_OPEN, "hold still")] for side in ("left", "right")}
    global SERVO
    right = right_plan(o)
    servos = [w.pos.servo for w in right if isinstance(w.pos, ClipPos)]
    SERVO = servos[0] if servos else None
    t_pinned = sum(w.duration for w in right)
    left = left_plan(o, t_pinned)
    t_peeled = sum(w.duration for w in left[:6])  # through "lift"
    park, park_rot = ready_pose("right")
    right += [
        Waypoint(max(t_peeled - t_pinned, 0.5), right[-1].pos, CLOSED, "hold the card down", right[-1].yaw),
        Waypoint(1.0, tuple(np.asarray(right[-1].pos, dtype=float) + (0.0, 0.0, UP)), CLOSED, "up off the card", right[-1].yaw),
        Waypoint(2.0, tuple(park), FINGER_OPEN, "park", Grip(park_rot)),
    ]
    end = max(sum(w.duration for w in right), sum(w.duration for w in left)) + 4.0
    for plan in (left, right):
        plan.append(_hold(end - sum(w.duration for w in plan), plan[-1], "rest"))
    if SERVO is not None:
        SERVO.follow(right)
    return {"left": left, "right": right}


def ready_pose(side: str) -> tuple[np.ndarray, np.ndarray]:
    """The arm's ready pose (where it starts; both arms park there at the end)."""
    ik = importlib.import_module("yam_arena.ik")
    station = importlib.import_module("yam_arena.station")
    pos, rot = ik.tcp_pose(np.asarray(READY, dtype=float), station.BASES[side])
    return np.asarray(pos, dtype=float), np.asarray(rot, dtype=float)


def _ready_tcp(side: str) -> np.ndarray:
    return ready_pose(side)[0]


def demo() -> dict[str, list[Waypoint]]:
    """Both arms: the right hand clips the site and puts the clipper back switched off, then pins the liner card while
    the left hand peels the electrode, sets it on the bare patch and presses round its edge."""
    return routine()
