"""On the station table, in front of the right arm: a piece of bathroom vanity top (white solid surface, 22 cm deep,
20 cm wide, 12 mm thick) with a single-handle faucet whose handle has been taken off. What is left is its brass
valve body, a round column 4 cm across standing about 7.5 cm above the vanity, with a short chrome spout pointing
toward the arms. The water supplies come in low on its sides: hot on the left (a brass shank with a red nut) and
cold on the right (a blue nut), as a user facing the faucet sees it. The removed lever handle lies behind the
faucet as a fixed reference, its red (hot) mark on its left and blue (cold) on its right.

The old cartridge is still in the valve body: a used grey-tan plastic cartridge about 2.4 cm across with a brass
stem standing 3 cm out of its top. Its two plastic tabs sit in the cut-outs at the front and back of the body's
rim, and a U-shaped copper retainer clip holds it down: the clip's two legs pass through slots near the top of the
valve body, and its small tab stands up at the front, toward the arms. Each stem stands down (off); it turns in
its cartridge and lifts toward on, and a small spring ball in the cartridge's top holds it where it is left. The
old stem is stiff: it binds 2 mm up (the handle won't open fully) and 15 degrees either way. The new one lifts 6 mm
to full-on and turns freely. Inside, each stem ends in a brass piston with a single port, which sets which supply
the handle's swing opens. A small dark notch on the stem's top, on the same side as the port, shows where it
faces.

On the table to the right of the vanity, nearer the arms, a new white cartridge of the same kind stands upright
in a short holder cup, stem up; further out, an empty grey parts tray waits for the old cartridge. The holder,
the tray and the handle are fixed.
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
from dataclasses import dataclass
from types import FrameType
from typing import Any, Protocol

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere

SCENE = "station"
SCORE_OVER = "end"
TOP: float = TABLE_TOP[SCENE]

# --- layout (world frame: x away from the arms, y to their left, z up; metres) --------------------------------
TABLE_X = (-0.19, 0.91)
TABLE_Y = (-0.65, 0.65)
LOST_BELOW = (
    0.05  # a part whose outline's lowest point is this far below the table top has fallen off
)

# A piece of 12 mm solid-surface vanity top with the faucet on it, in front of the right arm (whose straight-down
# grips reach highest there); the new cartridge's holder cup and the parts tray stand on the table beside it.
VANITY = "vanity_top"
VANITY_SIZE = (0.22, 0.20, 0.012)
VANITY_CENTER = (0.29, -0.10, TOP + VANITY_SIZE[2] / 2)
DECK = TOP + VANITY_SIZE[2]  # the vanity's top surface
MARBLE_RGB = (0.93, 0.93, 0.91)

# The valve body: a brass column with a bore for the cartridge. Its rim has two cut-outs (front and back) that
# key the cartridge's tabs, and just below the rim two straight slots (left and right of the bore, running front
# to back) carry the clip's legs over the cartridge's top shoulder. Sizes follow a 1225-type cartridge (Moen
# publishes no drawing; they are approximate).
FAUCET_XY = (0.28, -0.12)
R_OUT = 0.020  # the column's radius
R_BORE = 0.0125  # the bore's radius
FLOOR_THICK = 0.005
Z_FLOOR = DECK + FLOOR_THICK  # the bore's floor
# The supply inlets: a port through the bore's wall on each side, low down, hot on the left (+y, as a user facing
# the faucet sees it) and cold on the right. Outside, each takes a brass supply shank with a red (hot) or blue
# (cold) nut.
Z_INLET = Z_FLOOR + 0.012  # the ports' centre height
INLET_HALF_H = 0.003  # 6 mm tall ...
INLET_HALF_ANGLE = math.radians(14.0)  # ... and 28 degrees wide (6 mm at the bore)
HOT_BEARING, COLD_BEARING = math.pi / 2, -math.pi / 2  # where the inlets face, from the bore's axis
CART_R = 0.012  # the cartridge body's radius (0.5 mm radial clearance in the bore)
CART_LEN = 0.065  # the cartridge body, bottom to top shoulder
SEAT_GAP = 0.001  # seated, the tabs bear on the cut-outs' floor and the body stands this far off the bore's floor
Z_SHOULDER = Z_FLOOR + SEAT_GAP + CART_LEN  # a seated cartridge's top shoulder: the cut-outs' floor
SLOT_LIFT = 0.0005  # the clip slots' floor stands this far above the shoulder, so the legs slide clear of it
Z_SLOT_FLOOR = Z_SHOULDER + SLOT_LIFT
SLOT_H = 0.0025  # the clip slots' height
Z_SLOT_TOP = Z_SLOT_FLOOR + SLOT_H
TAB_H = 0.005  # the tabs (and the boss round the stem) rise this far above the shoulder: flush with the rim
Z_RIM = Z_SHOULDER + TAB_H
CUT_HALF = 0.006  # the cut-outs: |y| < 6 mm, through the wall, from the shoulder to the rim
CH_IN, CH_OUT = (
    0.007,
    0.012,
)  # the clip slots: 7 mm < |y| < 12 mm, at slot height (0.5 mm round each leg)
CHAMFER_DEPTH, CHAMFER_MOUTH = (
    0.0015,
    0.014,
)  # lead-ins at the rim and at the cut-outs' floor: 14 mm radius at the top
BODY_HEIGHT = Z_RIM - DECK

# O-ring drag: two spring-loaded steel balls in pockets on the bore's left and right walls press on the cartridge
# as it passes (ball plungers), so pushing a cartridge home or pulling it out takes a few newtons.
PAD_R = 0.003
PAD_Z = Z_FLOOR + 0.028
PAD_POCKET_HALF_Z = 0.0045
PAD_POCKET_HALF_ANGLE = math.radians(20.0)
PAD_EXTENDED = R_BORE - 0.001 + PAD_R  # the ball's centre at full extension: 1 mm proud of the bore
PAD_TRAVEL = 0.003  # how far a ball can be pushed back into its pocket
PAD_STIFFNESS = 3000.0  # N/m; 4.5 N per ball with a cartridge in the bore: about 10 N to push or pull one
PAD_PRELOAD = 0.001  # the spring's target lies 1 mm past full extension (3 N preload)
PAD_DAMPING = 8.0  # N s/m: about critical for the ball's mass
PAD_MASS = 0.005
PAD_FRICTION = (1.0, 0.9)
BORE_FRICTION = (0.3, 0.25)  # greased brass

# The cartridge: a plastic body with two tabs and a boss round the stem on its top shoulder.
CART_MASS = 0.040
BOSS_R = 0.006
TAB_SIZE = (0.0085, 0.008, TAB_H)  # radial, across, tall
TAB_X = (
    0.009 + TAB_SIZE[0] / 2
)  # the tabs run from 9 to 17.5 mm out from the axis, along the body's x
O_RING_Z = (-0.012, -0.030)  # two dark O-ring bands (looks only), sealing above and below the inlet windows
O_RING_R, O_RING_H = CART_R + 0.0002, 0.002
PLASTIC_FRICTION = (0.5, 0.4)
# A cartridge and its stem turn on each other only against the stem's O-rings: both bodies' angular damping (1/s)
# stands in for that friction, so neither keeps spinning on the other once let go.
CART_ANGULAR_DAMPING = 10.0

# The stem: brass, turns freely (damped) and slides 6 mm up to a stop.
STEM_R = 0.004
STEM_LEN = 0.045
STEM_ABOVE = 0.030  # its top stands this far above the rim when down
STEM_TRAVEL = 0.006
STEM_MASS = 0.020
STEM_DAMPING = 0.06  # N m s/rad: the O-rings' drag on turning (30 mN m at the demo's turn; a bump meets more)
# The stem's joint frames turn the joint's x onto the bodies' z (a -90 degree turn about y; w, x, y, z), so the
# stem's spin is the joint's twist (PhysX's free axis) and its 6 mm slide is along the joint's x.
TWIST_FRAME = (math.cos(-math.pi / 4), 0.0, math.sin(-math.pi / 4), 0.0)
# The work order: the handle is stiff and won't open fully. The old cartridge's scaled spool binds: its stem slides
# up only 2 of its 6 mm and turns only 15 degrees either way (limits on its joint).
OLD_STEM_TRAVEL = 0.002
OLD_TWIST_BIND = 15.0  # degrees, either way from where it stands
# A spring-loaded detent ball in each cartridge's boss presses on its stem (the stem's O-ring friction): it holds the
# stem where it is set (a lifted stem stays on) and adds a little drag to sliding and turning it. Each ball is a
# task object of its own (so the framework resets it with the rest); the hook adds its joint to its cartridge.
DETENT_R = 0.0015
DETENT_IN = 0.0003  # the joint's zero stands the ball this far into the stem's path
DETENT_X = STEM_R + DETENT_R - DETENT_IN  # the joint's zero, from the stem's axis, in the cartridge's +x
DETENT_REST_X = STEM_R + DETENT_R  # where the ball starts: touching the stem (its joint DETENT_IN out)
DETENT_Z = CART_LEN / 2 + TAB_H / 2  # its height in the cartridge's frame: mid-boss
DETENT_TRAVEL = 0.001
# N/m: about 1.2 N on the stem; with the objects' grippy friction (1.2), 1.4 N holds the stem's 0.2 N weight.
DETENT_STIFFNESS = 1000.0
DETENT_PRELOAD = 0.0009
DETENT_MASS = 0.001
STEM_CENTER_ABOVE_RIM = STEM_ABOVE - STEM_LEN / 2  # 7.5 mm
CART_CENTER_BELOW_RIM = TAB_H + CART_LEN / 2  # 37.5 mm
NOTCH = (1.0, 0.0)  # the notch's side of the stem, in the stem's own frame (x, y)
# Inside the shell the stem ends in a brass piston with one port on the notch's side (looks only: no water flows).
# Pulled up to its 6 mm stop (open), the port stands level with the inlets; turned, it faces one inlet or the
# other. With the stem's notch to the front the port faces the front, between the inlets; swinging the handle to
# the left (the stem a quarter turn clockwise, seen from above) brings the port onto the hot inlet.
PORT = (1.0, 0.0)  # the port's side of the piston, in the stem's own frame (x, y)
PISTON_R = 0.009
PORT_W = 0.0065  # the port's width (6.5 mm: 42 degrees round the piston) ...
PORT_HALF_H = 0.003  # ... and half its height
HANDLE_SWING = math.pi / 2  # the handle's swing from the centre to full hot (or full cold)
# The dark notch (looks only), as (centre, size) boxes in the stem's frame: down its +x side and across its top.
NOTCH_SIDE = ((STEM_R - 0.0009, 0.0, STEM_LEN / 2 - 0.0035), (0.0021, 0.0024, 0.007))
NOTCH_TOP = ((STEM_R / 2, 0.0, STEM_LEN / 2 + 0.0002), (STEM_R, 0.0024, 0.0006))

# The retainer clip: copper, a U of two legs and a bight, with a grab tab standing up from the bight.
CLIP = "retainer_clip"
CLIP_T = 0.0015  # thickness
LEG_W = 0.004
LEG_Y = 0.0095  # the legs' centre lines, |y|
LEG_LEN = 0.0526
TAB_UP = 0.010  # the grab tab's height above the bight
CLIP_MASS = 0.007
CLIP_Z = Z_SLOT_FLOOR + CLIP_T / 2  # its centre plane, resting on the slots' floor: 0.5 mm above a seated shoulder
GRAB_TAB_W = 0.008
SPOUT_TOP = Z_SLOT_FLOOR - 0.0002  # the spout's top, 0.2 mm under the clip slots' floor
BIGHT_SIZE = (CLIP_T, 2 * LEG_Y + LEG_W, CLIP_T)
CLIP_HOME_X = (
    FAUCET_XY[0] - 0.033 - CLIP_T / 2
)  # the bight's centre with the clip home (legs right across)
COPPER_FRICTION = (0.6, 0.5)

# The holder cup for the new cartridge, and the parts tray for the old one.
HOLDER_XY = (0.20, -0.27)
HOLDER_R = (0.0128, 0.016)
HOLDER_H = 0.020
HOLDER_FLOOR = 0.003
TRAY_XY = (0.33, -0.28)
TRAY_INNER = 0.11
TRAY_WALL = 0.004
TRAY_HEIGHT = 0.030
TRAY_FLOOR = 0.004
TRAY_RGB = (0.45, 0.47, 0.50)

NEW_CART, OLD_CART = "new_cartridge", "old_cartridge"
NEW_STEM, OLD_STEM = "new_stem", "old_stem"
CARTRIDGES = {NEW_CART: NEW_STEM, OLD_CART: OLD_STEM}
NEW_DETENT, OLD_DETENT = "new_detent_ball", "old_detent_ball"
DETENT_BALLS = {NEW_CART: NEW_DETENT, OLD_CART: OLD_DETENT}
DETENT_RGB = (0.75, 0.76, 0.78)  # steel
NEW_RGB = (0.94, 0.94, 0.92)  # new white plastic
OLD_RGB = (0.62, 0.58, 0.50)  # used, grey-tan
STEM_RGB = (0.80, 0.63, 0.30)  # brass
OLD_STEM_RGB = (0.58, 0.48, 0.30)  # tarnished brass
CLIP_RGB = (0.76, 0.45, 0.22)  # copper
BRASS_RGB = (0.72, 0.58, 0.30)
CHROME_RGB = (0.80, 0.82, 0.85)
NOTCH_RGB = (0.04, 0.04, 0.04)
HOT_RGB, COLD_RGB = (0.80, 0.08, 0.08), (0.10, 0.30, 0.85)
SHANK_SIZE = (0.008, 0.012, 0.008)  # a supply shank, reaching 11 mm out from the column
NUT_SIZE = (0.013, 0.008, 0.013)  # its coloured compression nut

OLD_CART_POS = (FAUCET_XY[0], FAUCET_XY[1], Z_RIM - CART_CENTER_BELOW_RIM)
NEW_CART_POS = (HOLDER_XY[0], HOLDER_XY[1], TOP + HOLDER_FLOOR + 0.0005 + CART_LEN / 2)
STEM_ABOVE_CART = (
    CART_CENTER_BELOW_RIM + STEM_CENTER_ABOVE_RIM
)  # a stem's centre above its cartridge's, when down
OLD_STEM_YAW = math.pi  # notch toward the arms (the front), as the faucet came
STEM_DOWN_Z = Z_RIM + STEM_CENTER_ABOVE_RIM  # a seated stem's centre when down (closed)
PORT_Z = Z_INLET - STEM_TRAVEL - STEM_DOWN_Z  # the port's centre in the stem's frame: level with the inlets when up
PISTON_BOTTOM = PORT_Z - PORT_HALF_H - 0.0005  # the piston's bottom, in the stem's frame
WINDOW_Z = Z_INLET - OLD_CART_POS[2]  # the shell's inlet windows, in a cartridge's frame
# (centre, size) boxes in their body's frame: the shell's two inlet windows (dark, flush with the shell), the piston
# (a cylinder, as its bounding box) and its port.
SHELL_WINDOWS = tuple(
    ((0.0, sign * (CART_R - 0.0005), WINDOW_Z), (0.006, 0.0012, 2 * INLET_HALF_H)) for sign in (1.0, -1.0)
)
PISTON = (
    (0.0, 0.0, (PISTON_BOTTOM - STEM_LEN / 2) / 2),
    (2 * PISTON_R, 2 * PISTON_R, -STEM_LEN / 2 - PISTON_BOTTOM),
)
PISTON_PORT = ((PISTON_R - 0.0005, 0.0, PORT_Z), (0.0012, PORT_W, 2 * PORT_HALF_H))
NEW_STEM_YAW = math.pi / 2  # notch to the left, as the new cartridge comes out of its box


def _box(
    name: str, center: Sequence[float], size: Sequence[float], color: tuple[float, float, float]
) -> Box:
    return Box(
        name,
        pos=(float(center[0]), float(center[1]), float(center[2])),
        size=tuple(size),
        color=color,
    )


def _tray() -> list[Box]:
    x, y = TRAY_XY
    half = TRAY_INNER / 2 + TRAY_WALL / 2
    wall_z = TOP + TRAY_HEIGHT / 2
    outer = TRAY_INNER + 2 * TRAY_WALL
    return [
        _box(
            "parts_tray_floor", (x, y, TOP + TRAY_FLOOR / 2), (outer, outer, TRAY_FLOOR), TRAY_RGB
        ),
        _box("parts_tray_front", (x - half, y, wall_z), (TRAY_WALL, outer, TRAY_HEIGHT), TRAY_RGB),
        _box("parts_tray_back", (x + half, y, wall_z), (TRAY_WALL, outer, TRAY_HEIGHT), TRAY_RGB),
        _box(
            "parts_tray_left", (x, y + half, wall_z), (TRAY_INNER, TRAY_WALL, TRAY_HEIGHT), TRAY_RGB
        ),
        _box(
            "parts_tray_right",
            (x, y - half, wall_z),
            (TRAY_INNER, TRAY_WALL, TRAY_HEIGHT),
            TRAY_RGB,
        ),
    ]


def _spout_and_handle() -> list[Box | Cylinder]:
    fx, fy = FAUCET_XY
    hx, hy = 0.355, -0.04  # the removed handle, lying on the vanity behind the faucet, to its left
    return [
        # The spout joins the valve body just under the clip slots: a clip slid out rests on it, level.
        _box("faucet_spout", (fx - 0.050, fy, SPOUT_TOP - 0.008), (0.060, 0.022, 0.016), CHROME_RGB),
        Cylinder(
            "spout_aerator",
            pos=(fx - 0.072, fy, SPOUT_TOP - 0.016 - 0.007),
            radius=0.0105,
            height=0.014,
            color=CHROME_RGB,
        ),
        Cylinder(
            "handle_hub", pos=(hx, hy, DECK + 0.014), radius=0.017, height=0.028, color=CHROME_RGB
        ),
        _box("handle_lever", (hx, hy - 0.050, DECK + 0.007), (0.014, 0.080, 0.012), CHROME_RGB),
        _box("handle_hot_mark", (hx, hy + 0.006, DECK + 0.0285), (0.010, 0.009, 0.001), HOT_RGB),
        _box("handle_cold_mark", (hx, hy - 0.006, DECK + 0.0285), (0.010, 0.009, 0.001), COLD_RGB),
    ]


def _supplies() -> list[Box]:
    """The two supply shanks screwed into the inlets from outside, with red (hot) and blue (cold) nuts."""
    fx, fy = FAUCET_XY
    out = []
    for name, sign, rgb in (("hot", 1.0, HOT_RGB), ("cold", -1.0, COLD_RGB)):
        out.append(_box(f"{name}_supply", (fx, fy + sign * (R_OUT + 0.005), Z_INLET), SHANK_SIZE, BRASS_RGB))
        out.append(_box(f"{name}_supply_nut", (fx, fy + sign * (R_OUT + 0.014), Z_INLET), NUT_SIZE, rgb))
    return out


VANITY_TOP = _box(VANITY, VANITY_CENTER, VANITY_SIZE, MARBLE_RGB)
OLD_CARTRIDGE = Cylinder(
    OLD_CART, pos=OLD_CART_POS, radius=CART_R, height=CART_LEN, mass=CART_MASS, color=OLD_RGB
)
NEW_CARTRIDGE = Cylinder(
    NEW_CART, pos=NEW_CART_POS, radius=CART_R, height=CART_LEN, mass=CART_MASS, color=NEW_RGB
)
OLD_STEM_SPEC = Cylinder(
    OLD_STEM,
    pos=(OLD_CART_POS[0], OLD_CART_POS[1], OLD_CART_POS[2] + STEM_ABOVE_CART),
    radius=STEM_R,
    height=STEM_LEN,
    mass=STEM_MASS,
    color=OLD_STEM_RGB,
    yaw=OLD_STEM_YAW,
)
NEW_STEM_SPEC = Cylinder(
    NEW_STEM,
    pos=(NEW_CART_POS[0], NEW_CART_POS[1], NEW_CART_POS[2] + STEM_ABOVE_CART),
    radius=STEM_R,
    height=STEM_LEN,
    mass=STEM_MASS,
    color=STEM_RGB,
    yaw=NEW_STEM_YAW,
)
# Each cartridge's detent ball, touching its stem in the cartridge's top (the hook joins it to the cartridge).
OLD_DETENT_SPEC = Sphere(
    OLD_DETENT,
    pos=(OLD_CART_POS[0] + DETENT_REST_X, OLD_CART_POS[1], OLD_CART_POS[2] + DETENT_Z),
    radius=DETENT_R,
    mass=DETENT_MASS,
    color=DETENT_RGB,
)
NEW_DETENT_SPEC = Sphere(
    NEW_DETENT,
    pos=(NEW_CART_POS[0] + DETENT_REST_X, NEW_CART_POS[1], NEW_CART_POS[2] + DETENT_Z),
    radius=DETENT_R,
    mass=DETENT_MASS,
    color=DETENT_RGB,
)
# The clip's own box is its bight; the scene hook adds the legs and the grab tab to the same body.
CLIP_SPEC = Box(
    CLIP, pos=(CLIP_HOME_X, FAUCET_XY[1], CLIP_Z), size=BIGHT_SIZE, mass=CLIP_MASS, color=CLIP_RGB
)
TRAY = _tray()
# The vanity first (its hook builds the valve body, the holder and the ball plungers); each cartridge before its
# stem and its detent ball (their hooks join each to it); the clip last.
OBJECTS = [
    VANITY_TOP,
    *_spout_and_handle(),
    *_supplies(),
    *TRAY,
    OLD_CARTRIDGE,
    NEW_CARTRIDGE,
    OLD_STEM_SPEC,
    NEW_STEM_SPEC,
    OLD_DETENT_SPEC,
    NEW_DETENT_SPEC,
    CLIP_SPEC,
]
PARTS = (OLD_CART, NEW_CART, OLD_STEM, NEW_STEM, CLIP)
NEW_PARTS = (NEW_CART, NEW_STEM, CLIP)  # what goes back in: losing one ends the job (the old one is scrap)


# --- geometry: closed triangle meshes of rings and ring pieces (pure NumPy) ----------------------------------------
Bounds = Callable[[float, float], tuple[float, float]]


def chamfered(z_top: float) -> Callable[[float], float]:
    """The bore's radius at height z under a top edge at ``z_top`` with a 45-degree-ish lead-in."""

    def radius(z: float) -> float:
        start = z_top - CHAMFER_DEPTH
        if z <= start:
            return R_BORE
        return R_BORE + (CHAMFER_MOUTH - R_BORE) * min(1.0, (z - start) / CHAMFER_DEPTH)

    return radius


def strip_bounds(
    y_lo: float, y_hi: float, r_in: Callable[[float], float], r_out: float = R_OUT
) -> Bounds:
    """Radial bounds along the ray at angle theta of the ring piece between y_lo < y < y_hi (either may be
    infinite), r_in(z) < r < r_out."""

    def bounds(theta: float, z: float) -> tuple[float, float]:
        s = math.sin(theta)
        lo, hi = r_in(z), r_out
        if s > 1e-12:
            if math.isfinite(y_lo):
                lo = max(lo, y_lo / s)
            if math.isfinite(y_hi):
                hi = min(hi, y_hi / s)
        elif s < -1e-12:
            if math.isfinite(y_hi):
                lo = max(lo, y_hi / s)
            if math.isfinite(y_lo):
                hi = min(hi, y_lo / s)
        elif not (y_lo < 0.0 < y_hi):
            return lo, lo
        return lo, hi

    return bounds


def theta_span(
    bounds: Bounds, z: float, lo: float, hi: float, n: int = 3600
) -> tuple[float, float]:
    """The interval of angles in [lo, hi] where the piece is at least 0.01 mm thick at height z, refined by
    bisection at both ends."""
    thetas = np.linspace(lo, hi, n + 1)
    inside = np.array([bounds(float(t), z)[1] - bounds(float(t), z)[0] > 1e-5 for t in thetas])
    if not inside.any():
        raise ValueError("empty ring piece")
    first, last = int(np.argmax(inside)), int(len(inside) - 1 - np.argmax(inside[::-1]))
    if not inside[first : last + 1].all():
        raise ValueError("ring piece is not one interval of angles")

    def edge(a: float, b: float) -> float:  # a outside, b inside
        for _ in range(40):
            m = (a + b) / 2
            if bounds(m, z)[1] - bounds(m, z)[0] > 1e-5:
                b = m
            else:
                a = m
        return b

    start = (
        float(thetas[first]) if first == 0 else edge(float(thetas[first - 1]), float(thetas[first]))
    )
    end = float(thetas[last]) if last == n else edge(float(thetas[last + 1]), float(thetas[last]))
    return start, end


def _oriented(tri: list[int], points: np.ndarray, toward: np.ndarray) -> list[int] | None:
    a, b, c = (points[i] for i in tri)
    normal = np.cross(b - a, c - a)
    if np.linalg.norm(normal) < 1e-14:
        return None
    return tri if float(normal @ toward) >= 0.0 else [tri[0], tri[2], tri[1]]


def polar_solid(
    theta0: float, theta1: float, bounds: Bounds, zs: Sequence[float], n: int, closed: bool = False
) -> tuple[np.ndarray, np.ndarray]:
    """A closed triangle mesh of the solid {theta0 <= angle <= theta1, lo <= r <= hi, zs[0] <= z <= zs[-1]},
    with (lo, hi) = bounds(angle, z) varying linearly between the heights in ``zs``. ``closed`` makes a full
    ring (theta1 = theta0 + 2 pi, no end faces). Every face points out of the solid."""
    m = len(zs)
    count = n if closed else n + 1
    thetas = [theta0 + (theta1 - theta0) * i / n for i in range(count)]
    points = np.zeros((count * m * 2, 3))

    def idx(i: int, j: int, side: int) -> int:
        return ((i % count) * m + j) * 2 + side

    for i, t in enumerate(thetas):
        c, s = math.cos(t), math.sin(t)
        for j, z in enumerate(zs):
            lo, hi = bounds(t, z)
            hi = max(hi, lo + 1e-5)
            points[idx(i, j, 0)] = (lo * c, lo * s, z)
            points[idx(i, j, 1)] = (hi * c, hi * s, z)
    tris: list[list[int]] = []

    def add(quad: Sequence[int], toward: np.ndarray) -> None:
        for tri in ([quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]]):
            fixed = _oriented(tri, points, toward)
            if fixed is not None:
                tris.append(fixed)

    spans = n
    for i in range(spans):
        tm = theta0 + (theta1 - theta0) * (i + 0.5) / n
        radial = np.array([math.cos(tm), math.sin(tm), 0.0])
        for j in range(m - 1):
            add([idx(i, j, 0), idx(i + 1, j, 0), idx(i + 1, j + 1, 0), idx(i, j + 1, 0)], -radial)
            add([idx(i, j, 1), idx(i + 1, j, 1), idx(i + 1, j + 1, 1), idx(i, j + 1, 1)], radial)
        add(
            [idx(i, 0, 0), idx(i, 0, 1), idx(i + 1, 0, 1), idx(i + 1, 0, 0)],
            np.array([0.0, 0.0, -1.0]),
        )
        add(
            [idx(i, m - 1, 0), idx(i, m - 1, 1), idx(i + 1, m - 1, 1), idx(i + 1, m - 1, 0)],
            np.array([0.0, 0.0, 1.0]),
        )
    if not closed:
        for i, t, sign in ((0, theta0, -1.0), (n, theta1, 1.0)):
            tangent = sign * np.array([-math.sin(t), math.cos(t), 0.0])
            for j in range(m - 1):
                add([idx(i, j, 0), idx(i, j + 1, 0), idx(i, j + 1, 1), idx(i, j, 1)], tangent)
    return points, np.asarray(tris, dtype=np.int64)


def _levels(z0: float, z1: float, chamfer_top: bool) -> list[float]:
    return [z0, z1 - CHAMFER_DEPTH, z1] if chamfer_top else [z0, z1]


def _piece(
    y_lo: float, y_hi: float, z0: float, z1: float, side: float, chamfer_top: bool, n: int
) -> tuple[np.ndarray, np.ndarray]:
    """The ring piece between y_lo < y < y_hi on one side of the bore (side: -1 front, x < 0; +1 back; 0 either)."""
    r_in = chamfered(z1) if chamfer_top else (lambda z: R_BORE)
    bounds = strip_bounds(y_lo, y_hi, r_in)
    if side < 0:
        lo, hi = (math.pi / 2, 3 * math.pi / 2)
    elif side > 0:
        lo, hi = (-math.pi / 2, math.pi / 2)
    else:
        lo, hi = (0.0, math.pi) if y_lo >= 0 else (-math.pi, 0.0)
    start, end = theta_span(bounds, z0, lo, hi)
    return polar_solid(start, end, bounds, _levels(z0, z1, chamfer_top), n)


def valve_body_pieces(n: int = 48) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """The valve body above its floor disc, as closed triangle meshes in the faucet's frame (x, y from its axis;
    z world): the bore wall below the inlets, round the hot and cold inlet ports (left and right), up to the ball
    pockets, round the pockets, up to the shoulder (lead-in at the top), a
    0.5 mm ledge under the slots, at slot height (minus the cut-outs and the clip slots) and up to the rim (minus
    the cut-outs; lead-in)."""
    inf = math.inf
    pocket_lo, pocket_hi = PAD_Z - PAD_POCKET_HALF_Z, PAD_Z + PAD_POCKET_HALF_Z
    ring = strip_bounds(-inf, inf, lambda z: R_BORE)
    a, b = PAD_POCKET_HALF_ANGLE, INLET_HALF_ANGLE
    inlet = [Z_INLET - INLET_HALF_H, Z_INLET + INLET_HALF_H]
    pieces = {
        "wall_base": polar_solid(0.0, 2 * math.pi, ring, [Z_FLOOR, inlet[0]], 2 * n, closed=True),
        "wall_inlets_back": polar_solid(-math.pi / 2 + b, math.pi / 2 - b, ring, inlet, n),
        "wall_inlets_front": polar_solid(math.pi / 2 + b, 3 * math.pi / 2 - b, ring, inlet, n),
        "wall_low": polar_solid(0.0, 2 * math.pi, ring, [inlet[1], pocket_lo], 2 * n, closed=True),
        "wall_pockets_back": polar_solid(
            -math.pi / 2 + a, math.pi / 2 - a, ring, [pocket_lo, pocket_hi], n
        ),
        "wall_pockets_front": polar_solid(
            math.pi / 2 + a, 3 * math.pi / 2 - a, ring, [pocket_lo, pocket_hi], n
        ),
        "wall_high": polar_solid(
            0.0,
            2 * math.pi,
            strip_bounds(-inf, inf, chamfered(Z_SHOULDER)),
            _levels(pocket_hi, Z_SHOULDER, True),
            2 * n,
            closed=True,
        ),
        "ledge_left": _piece(CUT_HALF, inf, Z_SHOULDER, Z_SLOT_FLOOR, 0.0, False, n),
        "ledge_right": _piece(-inf, -CUT_HALF, Z_SHOULDER, Z_SLOT_FLOOR, 0.0, False, n),
        "slot_left": _piece(CH_OUT, inf, Z_SLOT_FLOOR, Z_SLOT_TOP, 0.0, False, n),
        "slot_right": _piece(-inf, -CH_OUT, Z_SLOT_FLOOR, Z_SLOT_TOP, 0.0, False, n),
        "slot_left_front": _piece(CUT_HALF, CH_IN, Z_SLOT_FLOOR, Z_SLOT_TOP, -1.0, False, n // 4),
        "slot_left_back": _piece(CUT_HALF, CH_IN, Z_SLOT_FLOOR, Z_SLOT_TOP, 1.0, False, n // 4),
        "slot_right_front": _piece(-CH_IN, -CUT_HALF, Z_SLOT_FLOOR, Z_SLOT_TOP, -1.0, False, n // 4),
        "slot_right_back": _piece(-CH_IN, -CUT_HALF, Z_SLOT_FLOOR, Z_SLOT_TOP, 1.0, False, n // 4),
        "rim_left": _piece(CUT_HALF, inf, Z_SLOT_TOP, Z_RIM, 0.0, True, n),
        "rim_right": _piece(-inf, -CUT_HALF, Z_SLOT_TOP, Z_RIM, 0.0, True, n),
    }
    return pieces


def holder_ring(n: int = 48) -> tuple[np.ndarray, np.ndarray]:
    bounds = strip_bounds(-math.inf, math.inf, lambda z: HOLDER_R[0], HOLDER_R[1])
    return polar_solid(
        0.0, 2 * math.pi, bounds, [TOP + HOLDER_FLOOR, TOP + HOLDER_H], n, closed=True
    )


# --- the scene hook: what the task-file API can't express -------------------------------------------------------
# yam_arena.scene.task_object(spec) turns each task object into an Arena object. Right after that module is imported,
# this wraps task_object, matching this file's own object specs, so that after Arena spawns one its spawner also
# builds:
#  - the vanity: the valve body (static triangle meshes) and its floor, the holder cup, and the two ball plungers
#    (small rigid balls on PhysX prismatic joints with spring drives, anchored to the vanity);
#  - each cartridge: its two tabs and its stem boss (more colliders of the same rigid body) and two O-ring bands;
#  - each stem: its dark notch (looks only) and a joint to its cartridge: free to turn (a damped drive), free to
#    slide up 6 mm to a stop, locked otherwise (jointed bodies don't collide, so the stem passes through the boss);
#    the stiff old stem's joint stops it 2 mm up and 15 degrees either way;
#  - each detent ball: a prismatic joint to its cartridge, its spring drive pressing the ball onto the stem;
#  - the clip: its two legs and grab tab (more colliders of the clip's body).
# On a machine without the simulator nothing imports yam_arena.scene, and this does nothing. What was built is
# recorded on the scene module; scoring gives 0 unless every hooked object's parts were built (it never fails open).
_SCENE_MODULE = "yam_arena.scene"
PARTS_FOLDER = "faucet_parts"
PLUNGERS = "ball_plungers"
HOOK_TAG = "replace_faucet_cartridge_parts"
BUILD_RECORD = "replace_faucet_cartridge_build"  # an attribute of the scene module: what the hook built


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


def _material(
    px: _Pxr,
    stage: Any,
    path: str,
    rgb: Sequence[float],
    metallic: float,
    roughness: float,
    friction: tuple[float, float] | None,
) -> Any:
    material = px.shade.Material.Define(stage, path)
    shader = px.shade.Shader.Define(stage, f"{path}/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", px.sdf.ValueTypeNames.Color3f).Set(
        px.gf.Vec3f(*(_linear(c) for c in rgb))
    )
    shader.CreateInput("metallic", px.sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness", px.sdf.ValueTypeNames.Float).Set(roughness)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    if friction is not None:
        api = px.physics.MaterialAPI.Apply(material.GetPrim())
        api.CreateStaticFrictionAttr(friction[0])
        api.CreateDynamicFrictionAttr(friction[1])
        api.CreateRestitutionAttr(0.0)
    return material


def _bind(px: _Pxr, prim: Any, material: Any, collides: bool) -> None:
    api = px.shade.MaterialBindingAPI.Apply(prim)
    api.Bind(material)
    if collides:
        api.Bind(material, px.shade.Tokens.weakerThanDescendants, "physics")


# --- solver settings for the millimetre parts (README.md, Fidelity) ---------------------------------------------
# The clip is solved with more iterations than PhysX's default for a body (4 position, 1 velocity). Its speed out of
# an overlap is left at PhysX's default (unbounded): capped at 5 cm/s in the simulator, a cartridge pulled up hard
# went straight through the clip's legs, as the fingers went through capped stems. The other moving parts keep all
# of PhysX's defaults.
PART_POSITION_ITERATIONS = 32
PART_VELOCITY_ITERATIONS = 4
# The valve body's colliders and the clip's legs and grab tab start contacts FIT_CONTACT_OFFSET out and rest at zero
# separation, so a pair of them looks 0.4 mm out: less than the valve body's thinnest wall (the 0.5 mm ledge under
# the slots) and the clip's side play in its slots (0.5 mm). Left unauthored, PhysX picks a default offset, and a
# pair that looks further out than a thin wall is thick can meet the wall's far face, which pushes the wrong way.
FIT_CONTACT_OFFSET = 0.0002
FIT_REST_OFFSET = 0.0


def _tune_body(px: _Pxr, prim: Any) -> None:
    """The clip's solver settings, on its rigid body (the framework applies PhysX's rigid-body API to it)."""
    prim.AddAppliedSchema("PhysxRigidBodyAPI")
    ints = px.sdf.ValueTypeNames.Int
    prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", ints).Set(PART_POSITION_ITERATIONS)
    prim.CreateAttribute("physxRigidBody:solverVelocityIterationCount", ints).Set(PART_VELOCITY_ITERATIONS)


def _tune_collider(px: _Pxr, prim: Any) -> None:
    """A close-fitting collider's contact and rest offsets."""
    prim.AddAppliedSchema("PhysxCollisionAPI")
    floats = px.sdf.ValueTypeNames.Float
    prim.CreateAttribute("physxCollision:contactOffset", floats).Set(FIT_CONTACT_OFFSET)
    prim.CreateAttribute("physxCollision:restOffset", floats).Set(FIT_REST_OFFSET)


def _place(
    px: _Pxr, prim: Any, center: Sequence[float], scale: Sequence[float] | None = None
) -> None:
    xform = px.geom.Xformable(prim)
    xform.AddTranslateOp().Set(px.gf.Vec3d(*(float(c) for c in center)))
    if scale is not None:
        xform.AddScaleOp().Set(px.gf.Vec3f(*(float(s) for s in scale)))


def _cube(
    px: _Pxr,
    stage: Any,
    path: str,
    center: Sequence[float],
    size: Sequence[float],
    material: Any,
    collides: bool,
    fit: bool = False,
) -> str:
    """A box; ``collides`` makes it a collider, ``fit`` a close-fitting one (_tune_collider)."""
    cube = px.geom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    # Authored extents: the framework's outlines come from USD bounds, which then needn't be computed.
    cube.CreateExtentAttr([px.gf.Vec3f(-0.5, -0.5, -0.5), px.gf.Vec3f(0.5, 0.5, 0.5)])
    _place(px, cube.GetPrim(), center, size)
    if collides:
        px.physics.CollisionAPI.Apply(cube.GetPrim())
        if fit:
            _tune_collider(px, cube.GetPrim())
    _bind(px, cube.GetPrim(), material, collides)
    return path


def _cylinder(
    px: _Pxr,
    stage: Any,
    path: str,
    center: Sequence[float],
    radius: float,
    height: float,
    material: Any,
    collides: bool,
) -> str:
    cylinder = px.geom.Cylinder.Define(stage, path)
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(height)
    cylinder.CreateAxisAttr("Z")
    cylinder.CreateExtentAttr(
        [px.gf.Vec3f(-radius, -radius, -height / 2), px.gf.Vec3f(radius, radius, height / 2)]
    )
    _place(px, cylinder.GetPrim(), center)
    if collides:
        px.physics.CollisionAPI.Apply(cylinder.GetPrim())
    _bind(px, cylinder.GetPrim(), material, collides)
    return path


def _mesh(
    px: _Pxr,
    stage: Any,
    path: str,
    points: np.ndarray,
    tris: np.ndarray,
    offset: Sequence[float],
    material: Any,
    fit: bool = False,
) -> str:
    """A static triangle-mesh collider (no rigid body above it), points shifted by ``offset``; ``fit`` makes it a
    close-fitting one (_tune_collider)."""
    mesh = px.geom.Mesh.Define(stage, path)
    local = points + np.asarray(offset, dtype=float)
    mesh.CreatePointsAttr([px.gf.Vec3f(*(float(v) for v in p)) for p in local])
    mesh.CreateFaceVertexCountsAttr([3] * len(tris))
    mesh.CreateFaceVertexIndicesAttr([int(i) for i in tris.reshape(-1)])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateExtentAttr(
        [
            px.gf.Vec3f(*(float(v) for v in local.min(axis=0))),
            px.gf.Vec3f(*(float(v) for v in local.max(axis=0))),
        ]
    )
    px.physics.CollisionAPI.Apply(mesh.GetPrim())
    px.physics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
    if fit:
        _tune_collider(px, mesh.GetPrim())
    _bind(px, mesh.GetPrim(), material, True)
    return path


def _quat(px: _Pxr, yaw: float) -> Any:
    return px.gf.Quatf(math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


Vec3 = tuple[float, float, float]


def _vanity_layout() -> tuple[Vec3, Vec3, Vec3, Vec3]:
    """In the vanity's frame: the offset of the valve body's meshes (built in the faucet's frame), its floor
    disc's centre, the offset of the holder ring's mesh, and the holder floor's centre."""
    vx, vy, vz = VANITY_CENTER
    to_faucet = (FAUCET_XY[0] - vx, FAUCET_XY[1] - vy, -vz)
    to_holder = (HOLDER_XY[0] - vx, HOLDER_XY[1] - vy, -vz)
    floor_center = (to_faucet[0], to_faucet[1], DECK + FLOOR_THICK / 2 - vz)
    holder_floor = (to_holder[0], to_holder[1], TOP + HOLDER_FLOOR / 2 - vz)
    return to_faucet, floor_center, to_holder, holder_floor


def vanity_outline() -> tuple[np.ndarray, np.ndarray]:
    """The vanity's outline in its own frame, as built: its slab, and the valve body, its floor, the holder cup
    and the holder's floor that the hook adds under it (the framework's outline takes them all in)."""
    to_faucet, floor_center, to_holder, holder_floor = _vanity_layout()
    half = np.divide(VANITY_SIZE, 2)
    lows, highs = [-half], [half]
    for points, _ in valve_body_pieces().values():
        lows.append(points.min(axis=0) + to_faucet)
        highs.append(points.max(axis=0) + to_faucet)
    ring, _ = holder_ring()
    lows.append(ring.min(axis=0) + to_holder)
    highs.append(ring.max(axis=0) + to_holder)
    for center, radius, height in ((floor_center, R_OUT, FLOOR_THICK), (holder_floor, HOLDER_R[1], HOLDER_FLOOR)):
        lows.append(np.subtract(center, (radius, radius, height / 2)))
        highs.append(np.add(center, (radius, radius, height / 2)))
    return np.min(lows, axis=0), np.max(highs, axis=0)


def author_vanity(stage: Any, env_root: str) -> list[str]:
    """The valve body, its floor, the holder cup and the ball plungers, under ``env_root``; the paths made."""
    px = _Pxr()
    vanity = f"{env_root}/{VANITY}"
    folder = f"{vanity}/{PARTS_FOLDER}"
    brass = _material(px, stage, f"{folder}/brass", BRASS_RGB, 0.8, 0.35, BORE_FRICTION)
    pvc = _material(px, stage, f"{folder}/pvc", (0.86, 0.86, 0.84), 0.0, 0.6, PLASTIC_FRICTION)
    steel = _material(px, stage, f"{folder}/steel", (0.75, 0.76, 0.78), 0.9, 0.3, PAD_FRICTION)
    to_faucet, floor_center, to_holder, holder_floor = _vanity_layout()
    made = [f"{folder}/brass", f"{folder}/pvc", f"{folder}/steel"]
    for name, (points, tris) in valve_body_pieces().items():
        made.append(_mesh(px, stage, f"{folder}/valve_{name}", points, tris, to_faucet, brass, fit=True))
    made.append(
        _cylinder(px, stage, f"{folder}/valve_floor", floor_center, R_OUT, FLOOR_THICK, brass, True)
    )
    points, tris = holder_ring()
    made.append(_mesh(px, stage, f"{folder}/holder_ring", points, tris, to_holder, pvc))
    made.append(
        _cylinder(
            px, stage, f"{folder}/holder_floor", holder_floor, HOLDER_R[1], HOLDER_FLOOR, pvc, True
        )
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        ball = f"{env_root}/{PLUNGERS}/ball_{side}"
        center = (FAUCET_XY[0], FAUCET_XY[1] + sign * PAD_EXTENDED, PAD_Z)
        sphere = px.geom.Sphere.Define(stage, ball)
        sphere.CreateRadiusAttr(PAD_R)
        sphere.CreateExtentAttr([px.gf.Vec3f(-PAD_R, -PAD_R, -PAD_R), px.gf.Vec3f(PAD_R, PAD_R, PAD_R)])
        _place(px, sphere.GetPrim(), center)
        px.physics.RigidBodyAPI.Apply(sphere.GetPrim())
        px.physics.MassAPI.Apply(sphere.GetPrim()).CreateMassAttr(PAD_MASS)
        px.physics.CollisionAPI.Apply(sphere.GetPrim())
        _bind(px, sphere.GetPrim(), steel, True)
        joint = px.physics.PrismaticJoint.Define(stage, f"{ball}_joint")
        joint.CreateAxisAttr("Y")
        joint.CreateBody0Rel().SetTargets([vanity])
        joint.CreateBody1Rel().SetTargets([ball])
        frame = _quat(px, 0.0 if sign > 0 else math.pi)  # the joint's +y points out of the bore
        joint.CreateLocalPos0Attr(
            px.gf.Vec3f(*(float(c - v) for c, v in zip(center, VANITY_CENTER, strict=True)))
        )
        joint.CreateLocalRot0Attr(frame)
        joint.CreateLocalPos1Attr(px.gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot1Attr(frame)
        joint.CreateLowerLimitAttr(0.0)
        joint.CreateUpperLimitAttr(PAD_TRAVEL)
        drive = px.physics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(PAD_STIFFNESS)
        drive.CreateDampingAttr(PAD_DAMPING)
        drive.CreateTargetPositionAttr(-PAD_PRELOAD)
        made += [ball, f"{ball}_joint"]
    return made


def author_cartridge(stage: Any, env_root: str, name: str) -> list[str]:
    """The tabs and the stem boss (colliders of the cartridge's body); two O-ring bands and the shell's two inlet
    windows (looks only: dark openings on the body's left and right, level with the inlets when seated)."""
    px = _Pxr()
    root = f"{env_root}/{name}"
    folder = f"{root}/{PARTS_FOLDER}"
    rgb = NEW_RGB if name == NEW_CART else OLD_RGB
    plastic = _material(px, stage, f"{folder}/plastic", rgb, 0.0, 0.5, PLASTIC_FRICTION)
    rubber = _material(px, stage, f"{folder}/rubber", (0.05, 0.05, 0.05), 0.0, 0.8, None)
    top_z = CART_LEN / 2 + TAB_H / 2
    made = [f"{folder}/plastic", f"{folder}/rubber"]
    for label, sign in (("tab_front", -1.0), ("tab_back", 1.0)):
        made.append(
            _cube(
                px, stage, f"{folder}/{label}", (sign * TAB_X, 0.0, top_z), TAB_SIZE, plastic, True
            )
        )
    made.append(
        _cylinder(px, stage, f"{folder}/boss", (0.0, 0.0, top_z), BOSS_R, TAB_H, plastic, True)
    )
    for k, z in enumerate(O_RING_Z):
        made.append(
            _cylinder(
                px, stage, f"{folder}/o_ring_{k}", (0.0, 0.0, z), O_RING_R, O_RING_H, rubber, False
            )
        )
    for label, (center, size) in zip(("window_left", "window_right"), SHELL_WINDOWS, strict=True):
        made.append(_cube(px, stage, f"{folder}/{label}", center, size, rubber, False))
    body = stage.GetPrimAtPath(root)  # its rigid body (the framework applies PhysX's rigid-body API to it)
    body.CreateAttribute("physxRigidBody:angularDamping", px.sdf.ValueTypeNames.Float).Set(
        CART_ANGULAR_DAMPING
    )
    return made


def author_detent(stage: Any, env_root: str, name: str) -> list[str]:
    """A detent ball's joint to its cartridge's boss: prismatic along the cartridge's x, its spring drive pressing the
    ball onto the stem. The ball is a task object of its own, beside the cartridge (a rigid body's subtree must hold
    one body), so the framework resets it with the other parts; jointed to the cartridge, it touches only the
    stem."""
    px = _Pxr()
    cartridge = next(c for c, b in DETENT_BALLS.items() if b == name)
    path = f"{env_root}/{name}_joint"
    joint = px.physics.PrismaticJoint.Define(stage, path)
    joint.CreateAxisAttr("X")  # +x: out from the stem
    joint.CreateBody0Rel().SetTargets([f"{env_root}/{cartridge}"])
    joint.CreateBody1Rel().SetTargets([f"{env_root}/{name}"])
    joint.CreateLocalPos0Attr(px.gf.Vec3f(DETENT_X, 0.0, DETENT_Z))
    joint.CreateLocalRot0Attr(px.gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(px.gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(px.gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(DETENT_TRAVEL)
    drive = px.physics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(DETENT_STIFFNESS)
    drive.CreateDampingAttr(PAD_DAMPING)
    drive.CreateTargetPositionAttr(-DETENT_PRELOAD)
    return [path]


def _qmul(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float, float]:
    """The Hamilton product of two (w, x, y, z) quaternions: a's turn after b's."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def stem_frame(name: str) -> tuple[float, float, float, float]:
    """The stem's joint frame (w, x, y, z) in its own body: the twist frame, turned back by the stem's spawn yaw on its
    cartridge."""
    yaw = OLD_STEM_YAW if name == OLD_STEM else NEW_STEM_YAW
    return _qmul((math.cos(-yaw / 2), 0.0, 0.0, math.sin(-yaw / 2)), TWIST_FRAME)


def stem_travel(name: str) -> float:
    """How far a stem slides up: the stiff old one binds after 2 mm."""
    return OLD_STEM_TRAVEL if name == OLD_STEM else STEM_TRAVEL


def author_stem(stage: Any, env_root: str, name: str) -> list[str]:
    """The stem's notch and its brass piston with its port, inside the shell (looks only), and its joint to its
    cartridge."""
    px = _Pxr()
    cartridge = next(c for c, s in CARTRIDGES.items() if s == name)
    stem = f"{env_root}/{name}"
    folder = f"{stem}/{PARTS_FOLDER}"
    dark = _material(px, stage, f"{folder}/notch_mark", NOTCH_RGB, 0.0, 0.9, None)
    made = [f"{folder}/notch_mark"]
    # A dark notch on the stem's +x side at its top: down its side and across its top face.
    for label, (center, size) in (("notch_side", NOTCH_SIDE), ("notch_top", NOTCH_TOP)):
        made.append(_cube(px, stage, f"{folder}/{label}", center, size, dark, False))
    stage.GetPrimAtPath(stem).CreateAttribute(
        "physxRigidBody:angularDamping", px.sdf.ValueTypeNames.Float
    ).Set(CART_ANGULAR_DAMPING)  # the O-rings' friction, as on its cartridge
    brass = _material(px, stage, f"{folder}/piston_brass", STEM_RGB, 0.8, 0.35, None)
    made.append(f"{folder}/piston_brass")
    (piston_c, (piston_d, _, piston_h)), (port_c, port_size) = PISTON, PISTON_PORT
    made.append(
        _cylinder(px, stage, f"{folder}/piston", piston_c, piston_d / 2, piston_h, brass, False)
    )
    made.append(_cube(px, stage, f"{folder}/piston_port", port_c, port_size, dark, False))
    joint = px.physics.Joint.Define(stage, f"{env_root}/{name}_joint")
    joint.CreateBody0Rel().SetTargets([f"{env_root}/{cartridge}"])
    joint.CreateBody1Rel().SetTargets([stem])
    joint.CreateLocalPos0Attr(px.gf.Vec3f(0.0, 0.0, CART_LEN / 2 + TAB_H))  # the boss's top centre
    joint.CreateLocalRot0Attr(px.gf.Quatf(*TWIST_FRAME))  # the joint's x along the cartridge's z
    # The same point and axis seen from the stem, which spawns turned on its cartridge: its frame is turned back by
    # that much, so the joint's twist reads 0 where the stem starts (PhysX twist limits must lie within +-180).
    joint.CreateLocalPos1Attr(px.gf.Vec3f(0.0, 0.0, -STEM_CENTER_ABOVE_RIM))
    joint.CreateLocalRot1Attr(px.gf.Quatf(*stem_frame(name)))
    for axis in ("transY", "transZ", "rotY", "rotZ"):  # locked: low above high (no swing, no sideways)
        limit = px.physics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr(1.0)
        limit.CreateHighAttr(-1.0)
    slide = px.physics.LimitAPI.Apply(joint.GetPrim(), "transX")  # up the bodies' z
    slide.CreateLowAttr(0.0)
    slide.CreateHighAttr(stem_travel(name))
    if name == OLD_STEM:  # the stiff old spool: it turns only so far (angular limits are in degrees)
        bind = px.physics.LimitAPI.Apply(joint.GetPrim(), "rotX")
        bind.CreateLowAttr(-OLD_TWIST_BIND)
        bind.CreateHighAttr(OLD_TWIST_BIND)
    turn = px.physics.DriveAPI.Apply(joint.GetPrim(), "rotX")  # the twist: free, damped
    turn.CreateTypeAttr("force")
    turn.CreateTargetPositionAttr(0.0)
    turn.CreateTargetVelocityAttr(0.0)  # damped toward standing still (relative to the cartridge)
    turn.CreateStiffnessAttr(0.0)
    turn.CreateDampingAttr(STEM_DAMPING * math.pi / 180.0)  # USD angular drives are per degree
    made.append(f"{env_root}/{name}_joint")
    return made


def author_clip(stage: Any, env_root: str) -> list[str]:
    """The clip's legs and grab tab: more colliders of the clip's body (its own box is the bight), close-fitting in
    the slots; and the clip's solver settings."""
    px = _Pxr()
    _tune_body(px, stage.GetPrimAtPath(f"{env_root}/{CLIP}"))
    folder = f"{env_root}/{CLIP}/{PARTS_FOLDER}"
    copper = _material(px, stage, f"{folder}/copper", CLIP_RGB, 0.9, 0.35, COPPER_FRICTION)
    made = [f"{folder}/copper"]
    leg_x = CLIP_T / 2 + LEG_LEN / 2
    for label, sign in (("leg_left", 1.0), ("leg_right", -1.0)):
        made.append(
            _cube(
                px,
                stage,
                f"{folder}/{label}",
                (leg_x, sign * LEG_Y, 0.0),
                (LEG_LEN, LEG_W, CLIP_T),
                copper,
                True,
                fit=True,
            )
        )
    made.append(
        _cube(
            px,
            stage,
            f"{folder}/grab_tab",
            (0.0, 0.0, CLIP_T / 2 + TAB_UP / 2),
            (CLIP_T, GRAB_TAB_W, TAB_UP),
            copper,
            True,
            fit=True,
        )
    )
    return made


AUTHORS: dict[str, Callable[[Any, str], list[str]]] = {
    VANITY: author_vanity,
    OLD_CART: functools.partial(author_cartridge, name=OLD_CART),
    NEW_CART: functools.partial(author_cartridge, name=NEW_CART),
    OLD_STEM: functools.partial(author_stem, name=OLD_STEM),
    NEW_STEM: functools.partial(author_stem, name=NEW_STEM),
    OLD_DETENT: functools.partial(author_detent, name=OLD_DETENT),
    NEW_DETENT: functools.partial(author_detent, name=NEW_DETENT),
    CLIP: author_clip,
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


def _build_record(scene: Any) -> dict[str, str]:
    """What the hook built, kept on the scene module so every copy of this file loaded in the process reads the
    same record: object name -> "built" (its parts authored in every environment) or "failed"."""
    record = getattr(scene, BUILD_RECORD, None)
    if not isinstance(record, dict):
        record = {}
        setattr(scene, BUILD_RECORD, record)
    return record


def _with_parts(spawn: Callable[..., Any], name: str, record: dict[str, str]) -> Callable[..., Any]:
    def spawn_and_build(prim_path: str, cfg: Any, *args: Any, **kwargs: Any) -> Any:
        prim = spawn(prim_path, cfg, *args, **kwargs)
        try:
            roots = _env_roots(prim, name)
            if not roots:
                raise RuntimeError("no environment holds it")
            for root in roots:
                AUTHORS[name](prim.GetStage(), root)
        except Exception as error:  # the faucet is incomplete: scoring sees the record and gives 0
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
    """Whether ``spec`` is one of this file's hooked objects. Specs are frozen dataclasses, so this compares every
    field by value: it holds across a second load of this file, and another task's object that merely shares a
    name is left alone."""
    try:
        return getattr(spec, "name", None) in AUTHORS and any(spec == own for own in OBJECTS)
    except Exception:
        return False


def _spawner_cfgs(item: object) -> Iterator[Any]:
    """The spawner configs an Arena object holds (one level deep): objects with a callable ``func`` and a
    ``size`` (a box) or a ``radius`` (a cylinder)."""
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


def build_faucet(scene: Any) -> bool:
    """Wrap ``scene.task_object`` so this file's hooked objects come out with their parts; True if it wrapped it
    now. It matches whole specs (see _ours), so a second load of this file is served by the first wrapper and
    other tasks' objects are untouched."""
    original = getattr(scene, "task_object", None)
    if original is None or getattr(original, HOOK_TAG, None) is not None:
        return False
    record = _build_record(scene)

    def task_object(spec: Any) -> Any:
        if _ours(spec) and spec.name == VANITY:  # the first of this task's objects: a new build starts
            record.clear()
        made = original(spec)
        if _ours(spec):
            name = spec.name
            found = 0
            for cfg in _spawner_cfgs(made):
                found += 1
                if getattr(cfg.func, HOOK_TAG, None) is None:
                    cfg.func = _with_parts(cfg.func, name, record)
            if not found:
                record[name] = "failed"
                _warn_once(
                    f"spawner-{name}", f"found no spawner for {name}; its parts will be missing"
                )
        return made

    setattr(task_object, HOOK_TAG, "task_object")
    scene.task_object = task_object
    return True


class _FaucetWatcher(importlib.abc.MetaPathFinder):
    """Waits for yam_arena.scene to be imported, lets it load normally, then applies build_faucet. It answers for
    that one module name only, and removes itself once the module has loaded."""

    marker = "replace_faucet_cartridge.scene_watcher"

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
        except Exception as error:  # never break the framework's import: let the other finders answer
            self.disarm()  # first, so nothing below can leave the finder armed
            _warn_once("finder", f"could not watch the scene module ({type(error).__name__}: {error})")
            return None
        finally:
            self._looking = False
        if spec is None:  # nothing to watch: stand down
            self.disarm()
            return None
        if spec.loader is not None:
            spec.loader = _BuildAfterLoad(spec.loader, self)
        return spec

    def disarm(self) -> None:
        if self in sys.meta_path:
            sys.meta_path.remove(self)


class _BuildAfterLoad(importlib.abc.Loader):
    """The module's own loader, plus build_faucet once the module has run."""

    def __init__(self, inner: importlib.abc.Loader, watcher: _FaucetWatcher) -> None:
        self._inner = inner
        self._watcher = watcher

    def create_module(self, spec: importlib.machinery.ModuleSpec) -> types.ModuleType | None:
        return self._inner.create_module(spec)

    def exec_module(self, module: types.ModuleType) -> None:
        try:
            self._inner.exec_module(module)  # the module's own import: its errors are its own
        finally:
            self._watcher.disarm()
        try:
            build_faucet(module)
        except Exception as error:  # the import must still succeed; the faucet then scores 0
            with contextlib.suppress(Exception):  # record first, then say so
                _build_record(module).update(dict.fromkeys(AUTHORS, "failed"))
            _warn_once("wrap", f"could not wrap the scene builder ({type(error).__name__}: {error})")

    def __getattr__(self, name: str) -> Any:  # get_source and friends, for tracebacks
        if name.startswith("__") or name in ("_inner", "_watcher"):  # a copy made without __init__
            raise AttributeError(name)
        return getattr(self._inner, name)


def install_faucet() -> None:
    """Wrap the scene builder now if it is already imported; otherwise wait for its import (once)."""
    scene = sys.modules.get(_SCENE_MODULE)
    if scene is not None:
        build_faucet(scene)
        return
    if not any(
        getattr(finder, "marker", None) == _FaucetWatcher.marker for finder in sys.meta_path
    ):
        sys.meta_path.insert(0, _FaucetWatcher())


# --- reading the parts -----------------------------------------------------------------------------------------
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


@dataclass(frozen=True)
class PartView:
    """One part as the simulator reports it: the centre of its world-aligned outline (its bounding box), the
    outline's corners, its body's rotation and how fast it moves."""

    center: np.ndarray
    low: np.ndarray
    high: np.ndarray
    rot: np.ndarray
    speed: float
    spin: float

    @staticmethod
    def of(state: _State) -> "PartView":
        return PartView(
            np.asarray(state.pos, dtype=float),
            np.asarray(state.low, dtype=float),
            np.asarray(state.high, dtype=float),
            rotation(state.quat_xyzw),
            float(np.linalg.norm(np.asarray(state.lin_vel, dtype=float))),
            float(np.linalg.norm(np.asarray(state.ang_vel, dtype=float))),
        )

    @property
    def tilt(self) -> float:
        """Its body's z axis from the vertical (rad)."""
        return math.acos(max(-1.0, min(1.0, float(self.rot[2, 2]))))

    def heading(self, local: Sequence[float] = (1.0, 0.0)) -> float:
        """The world bearing (rad, from +x toward +y) of a horizontal direction fixed in its body."""
        d = self.rot @ np.array([local[0], local[1], 0.0])
        return math.atan2(float(d[1]), float(d[0]))

    @property
    def still(self) -> bool:
        return self.speed < MAX_SPEED and self.spin < MAX_SPIN

    @property
    def lost(self) -> bool:
        x, y = float(self.center[0]), float(self.center[1])
        off = not (TABLE_X[0] <= x <= TABLE_X[1] and TABLE_Y[0] <= y <= TABLE_Y[1])
        return off or float(self.low[2]) < TOP - LOST_BELOW or not np.all(np.isfinite(self.center))


def views(objects: Mapping[str, _State]) -> dict[str, PartView]:
    return {name: PartView.of(objects[name]) for name in PARTS}


def angle_between(a: float, b: float) -> float:
    """The smaller angle between two bearings (rad)."""
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


# --- the rules ---------------------------------------------------------------------------------------------------
MAX_SPEED, MAX_SPIN = 0.01, 0.5  # still: m/s, rad/s
SEAT_TOL = 0.001  # seated: the body's bottom within 1 mm of where the tabs on the cut-outs' floor put it
SEAT_AXIS_TOL = 0.0015  # and its axis within 1.5 mm of the bore's
SEAT_TILT = math.radians(3.0)
TAB_TOL = math.radians(8.0)  # its tabs within 8 degrees of front-back (either way round)
OUT_CLEAR = 0.002  # out of the valve body: its lowest point 2 mm above the rim ...
OUT_AWAY = 0.025  # ... or its centre more than this from the bore's axis (in plan; one set down against the
# column stands 32 mm off)
CLIP_TILT = math.radians(5.0)
CLIP_YAW = math.radians(5.0)  # the clip's legs along x (from the front or from the back)
# Retained: the clip's outline spans the bore's axis by this much on both sides (the shoulder under the legs
# reaches 9.4 mm either side at the legs' inner edges).
CLIP_ACROSS = 0.0115
CLIP_Y_TOL = 0.002
# Clipped: the clip's lowest point rests on the slots' floor (0.5 mm below it to 0.2 mm above) ...
CLIP_Z_BAND = (Z_SLOT_FLOOR - 0.0005, Z_SLOT_FLOOR + 0.0002)
# ... and the new cartridge's bottom within 0.5 mm of its seat: the clip's legs pass over a cartridge only that far
# home, and a clip tipped over one standing prouder (still inside the band and CLIP_TILT) doesn't count.
CLIPPED_DEPTH = SLOT_LIFT
# Retaining, while the after-test lifts the stem: the legs in their slots across the bore, however high in the
# slots, and the cartridge risen no further than they let it. A lifted stem can draw a clipped cartridge up the
# 0.5 mm gap under the legs, then the legs up their 1 mm of play to the slots' ceiling: 1.5 mm. RETAIN_RISE allows
# 0.5 mm more. Without its clip a cartridge rose 4.5 mm in the simulator.
CLIP_PLAY = SLOT_H - CLIP_T
RETAIN_TOL = 0.0005
RETAIN_RISE = SLOT_LIFT + CLIP_PLAY + RETAIN_TOL
# Forced past the clip (README.md, "In order"): a cartridge's shoulder 3 mm above its seat is at the slots' ceiling,
# past wherever the legs can be in their slots; FORCED_RISE allows 1 mm more for the solver's give and the readings.
# With the clip home, the old cartridge rose at most 1.55 mm in the demo's before-test on the GPU (1.72 mm in an
# earlier build's held pull). A cartridge forced through the clip crosses every depth from FORCED_RISE up to
# CLEAR_OF_SLOTS, where its bottom passes the slots' top (68 mm), so no lower threshold would catch more.
FORCED_RISE = SLOT_LIFT + SLOT_H + 0.001
CLEAR_OF_SLOTS = Z_SLOT_TOP - (Z_FLOOR + SEAT_GAP)
# ... and only a clip over its shoulder can force it: some part of the clip within FORCED_GAP of the bore's axis in
# plan. That is the cartridge's radius (CART_R, which "out of its way" needs the clip clear of) less its 0.5 mm side
# play in the bore and 0.5 mm for a contact's give, so a clip that only touches a cartridge's side isn't over it. (On
# the GPU, leg tips stopped against a cartridge 16.8 mm proud and 0.5 mm off the axis read 11.5 to 11.6 mm.)
FORCED_GAP = CART_R - (R_BORE - CART_R) - 0.0005
# A clip whose lowest point is no lower lies on the rim or above it (0.5 mm allowed for a resting contact's give);
# legs in their slots reach no higher than Z_SLOT_TOP - CLIP_T, 3 mm below that.
ON_RIM = Z_RIM - 0.0005
PORT_TOL = math.radians(20.0)  # hot on the left: the handle's left swing brings the piston's port this close to
# the hot inlet's centre (within 20 degrees the port uncovers more than half of the inlet)
FRONT = math.pi  # toward the arms (-x): where the stem's notch and the piston's port face when set
TRAY_RIM_Z = TOP + TRAY_FLOOR + 0.015  # in the tray: lowest point below this
HOLD_STEPS = 20  # 1.0 once its conditions have held 1 s: this many control steps at 20 Hz (or the embodiment's rate)
HAND_CLEAR = 0.01  # every gripper's tool point and finger pads at least this far from the new parts and the clip
LADDER = (0.25, 0.5, 0.75, 1.0)

TRAY_X = (TRAY_XY[0] - TRAY_INNER / 2, TRAY_XY[0] + TRAY_INNER / 2)
TRAY_Y = (TRAY_XY[1] - TRAY_INNER / 2, TRAY_XY[1] + TRAY_INNER / 2)


def axis_offset(part: PartView) -> float:
    """How far its outline's centre is from the bore's axis, in plan (m)."""
    return math.hypot(float(part.center[0]) - FAUCET_XY[0], float(part.center[1]) - FAUCET_XY[1])


def out_of_body(cart: PartView) -> bool:
    return float(cart.low[2]) > Z_RIM + OUT_CLEAR or axis_offset(cart) > OUT_AWAY


def in_body(cart: PartView) -> bool:
    """Still in the valve body: out_of_body's complement, with finite comparisons that a NaN or infinite reading
    fails."""
    low, off = float(cart.low[2]), axis_offset(cart)
    return math.isfinite(low) and math.isfinite(off) and low <= Z_RIM + OUT_CLEAR and off <= OUT_AWAY


def keyed(cart: PartView) -> bool:
    """In the bore: on its axis, upright, its tabs in the cut-outs (at any depth)."""
    tabs = angle_between(cart.heading(), 0.0)
    return (
        axis_offset(cart) <= SEAT_AXIS_TOL
        and cart.tilt <= SEAT_TILT
        and min(tabs, math.pi - tabs) <= TAB_TOL
    )


def seated(cart: PartView) -> bool:
    """Pushed fully home: the body's bottom where the tabs on the cut-outs' floor put it, upright, on the bore's
    axis, its tabs in the cut-outs."""
    return abs(depth(cart)) <= SEAT_TOL and keyed(cart)


def depth(cart: PartView) -> float:
    """How far a cartridge's body stands above its seat, measured on its axis: the centre of its bottom face over
    where the tabs on the cut-outs' floor put it (negative if lower). Not the outline's lowest point: the outline is
    the turned corners of its local box, so on a cocked cartridge that point is lower by the box's half-width (up to
    17.5 mm, across the tabs) times the tilt's sine."""
    low, high = local_outline(NEW_CART)
    origin = cart.center - cart.rot @ ((low + high) / 2)
    bottom = origin + cart.rot @ np.array([0.0, 0.0, -CART_LEN / 2])
    return float(bottom[2]) - (Z_FLOOR + SEAT_GAP)


def _clip_in_slots(clip: PartView, highest: float) -> bool:
    """Legs along x, reaching right across the cartridge's top both ways, centred, its lowest point from the slots'
    floor (less the band's tolerance) up to ``highest``. A NaN or infinite outline never is."""
    readings = (*np.asarray(clip.low, dtype=float), *np.asarray(clip.high, dtype=float), float(clip.center[1]))
    if not all(math.isfinite(v) for v in readings):
        return False
    yaw = angle_between(clip.heading(), 0.0)
    return (
        min(yaw, math.pi - yaw) <= CLIP_YAW
        and float(clip.low[0]) <= FAUCET_XY[0] - CLIP_ACROSS
        and float(clip.high[0]) >= FAUCET_XY[0] + CLIP_ACROSS
        and abs(float(clip.center[1]) - FAUCET_XY[1]) <= CLIP_Y_TOL
        and CLIP_Z_BAND[0] <= float(clip.low[2]) <= highest
    )


def clip_home(clip: PartView) -> bool:
    """Retaining at rest: flat in the slots, legs on their floors, reaching right across the cartridge's top."""
    return clip.tilt <= CLIP_TILT and _clip_in_slots(clip, CLIP_Z_BAND[1])


def clip_retaining(clip: PartView) -> bool:
    """Retaining while a lift draws the cartridge up into it: as clip_home, but the legs anywhere in their slots'
    play (their lowest point up to CLIP_PLAY higher)."""
    return clip.tilt <= CLIP_TILT and _clip_in_slots(clip, CLIP_Z_BAND[1] + CLIP_PLAY)


def held_down(cart: PartView, clip: PartView) -> bool:
    """The cartridge no higher than the clip's legs let it stand (its shoulder at most RETAIN_TOL into their
    underside) and never past RETAIN_RISE: a cartridge well into legs resting on the floor isn't held by them."""
    legs = SLOT_LIFT + float(clip.low[2]) - Z_SLOT_FLOOR  # the legs' underside, over a seated shoulder
    d = depth(cart)
    # Explicit comparisons, so a NaN anywhere fails closed (min() would pass a NaN through to RETAIN_RISE), and
    # an infinite clip reading allows nothing.
    return math.isfinite(legs) and d >= -SEAT_TOL and d <= RETAIN_RISE and d <= legs + RETAIN_TOL


def clip_across(clip: PartView) -> bool:
    """In its slots across the bore, however high in them: the pose that should hold the cartridge down. (Unlike
    clip_home, it still holds while a rising cartridge lifts the legs toward the slots' ceiling.)"""
    return _clip_in_slots(clip, Z_SLOT_FLOOR + SLOT_H)


# Out of a cartridge's way (README.md, "Out of its way") is read from the clip's own shapes, turned and tipped as it
# lies, not from its outline: a box square to the world, which round a clip set down turned on the vanity takes in up
# to 21 mm of empty space. No part of the clip may come within the cartridge's radius (CART_R) of the bore's axis in
# plan. With its legs along the slots, that is where their inner edges meet the shoulder's edge: 9.37 mm either side
# of the axis with the clip centred in its slots, CLIP_FREE_X with the legs at either end of their side play.
CLIP_FREE_X = math.sqrt(CART_R**2 - (LEG_Y - LEG_W / 2 - (CH_OUT - CH_IN - LEG_W) / 2) ** 2)
_CORNERS = np.array([(x, y, z) for x in (-0.5, 0.5) for y in (-0.5, 0.5) for z in (-0.5, 0.5)])


def _hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """The convex hull of points in plan, anticlockwise (Andrew's monotone chain)."""
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def turn(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    upper: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and turn(lower[-2], lower[-1], p) <= 0.0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and turn(upper[-2], upper[-1], p) <= 0.0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def plan_gap(corners: np.ndarray, point: Sequence[float]) -> float:
    """How far (m), in plan, a point is from the shadow of a box given by its corners (world): 0 if it covers it."""
    hull = _hull([(float(c[0]), float(c[1])) for c in corners])
    px, py = float(point[0]), float(point[1])
    inside = len(hull) >= 3
    gap = math.inf
    for (ax, ay), (bx, by) in zip(hull, hull[1:] + hull[:1], strict=True):
        ex, ey = bx - ax, by - ay
        inside = inside and ex * (py - ay) - ey * (px - ax) >= 0.0
        length = ex * ex + ey * ey
        k = 0.0 if length == 0.0 else max(0.0, min(1.0, ((px - ax) * ex + (py - ay) * ey) / length))
        gap = min(gap, math.hypot(px - ax - k * ex, py - ay - k * ey))
    return 0.0 if inside else gap


def readable(part: PartView) -> bool:
    """Every reading of a part's pose is finite: its outline's centre and corners, and its rotation."""
    values = (part.center, part.low, part.high, part.rot)
    return all(bool(np.all(np.isfinite(np.asarray(v, dtype=float)))) for v in values)


def clip_gap(clip: PartView) -> float:
    """How near (m) the clip comes to the bore's axis in plan: from the axis to the nearest of its shapes (bight,
    legs, grab tab), turned and tipped as it lies; 0 where one covers the axis. NaN if a reading isn't finite."""
    if not readable(clip):
        return math.nan
    origin = body_origin(CLIP, clip)
    return min(
        plan_gap(origin + (np.asarray(centre) + _CORNERS * np.asarray(size)) @ clip.rot.T, FAUCET_XY)
        for centre, size in part_shapes(CLIP)
    )


def clip_out(clip: PartView) -> bool:
    """Out of the way of a cartridge going in: no part of the clip within the cartridge's radius of the bore's axis
    in plan, however high it is (clip_gap). Only then can a cartridge pass it. A NaN or infinite reading never is.
    (For the old cartridge coming out, State.clip_out_old allows more.)"""
    gap = clip_gap(clip)
    return math.isfinite(gap) and gap >= CART_R


def notch_bearing(stem: PartView) -> float:
    return stem.heading(NOTCH)


def port_bearing(stem: PartView) -> float:
    """Where the piston's port faces (its bearing from the bore's axis), with the stem as it is."""
    return stem.heading(PORT)


def hot_error(stem: PartView) -> float:
    """How far (rad) the handle's swing to the left (the stem a quarter turn clockwise, seen from above) leaves the
    piston's port from the hot inlet's centre."""
    return angle_between(port_bearing(stem) - HANDLE_SWING, HOT_BEARING)


def in_tray(part: PartView) -> bool:
    x, y = float(part.center[0]), float(part.center[1])
    return (
        TRAY_X[0] <= x <= TRAY_X[1]
        and TRAY_Y[0] <= y <= TRAY_Y[1]
        and float(part.low[2]) <= TRAY_RIM_Z
    )


# --- the robot's grippers (from the embodiment's joints) -------------------------------------------------------------
# The YAM's finger pads (official constants): their tips end PAD_TIP_PAST_TCP past the TCP along the approach axis,
# they are about PAD_WIDTH across, and PAD_LENGTH of each pad up from its tip can touch.
PAD_TIP_PAST_TCP = 0.013
PAD_LENGTH = 0.030
PAD_WIDTH = 0.027


@dataclass(frozen=True)
class Gripper:
    """One arm's tool point (TCP, midway between the fingertips; world, m), how far apart its fingers are (m), the
    direction its fingers point (approach) and the axis they close along (unit vectors)."""

    tcp: np.ndarray
    opening: float
    approach: tuple[float, float, float] = (0.0, 0.0, -1.0)
    closing: tuple[float, float, float] = (0.0, 1.0, 0.0)
    joints: tuple[float, ...] = ()  # the joint positions it was computed from

    def finger_points(self) -> list[np.ndarray]:
        """The TCP and a grid over both finger pads: at the tips, halfway up and PAD_LENGTH up, at both edges and
        the middle of each pad."""
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


def point_box_distance(point: Any, low: Any, high: Any) -> float:
    p = np.asarray(point, dtype=float)
    d = np.maximum(
        np.maximum(np.asarray(low, dtype=float) - p, p - np.asarray(high, dtype=float)), 0.0
    )
    return float(np.linalg.norm(d))


def hands_clear(parts: Mapping[str, PartView], grippers: Sequence[Gripper]) -> bool:
    """Every gripper's tool point and finger pads at least HAND_CLEAR from the new cartridge, its stem and the clip."""
    return all(
        point_box_distance(point, parts[name].low, parts[name].high) >= HAND_CLEAR
        for name in (NEW_CART, NEW_STEM, CLIP)
        for gripper in grippers
        for point in gripper.finger_points()
    )


# --- the trial's preconditions: the parts were built, the readings are finite, the parts are as built ----------
GEOMETRY_TOL = 0.0005  # a trial's first step: outlines and positions within 0.5 mm of what was built ...
SETTLE = 0.001  # ... except that a part may have settled this far onto its support
STEM_TILT_TOL = math.radians(1.0)  # a stem's axis along its cartridge's (its joint locks the tilt)
_STATE_FIELDS = ("pos", "quat_xyzw", "lin_vel", "ang_vel", "low", "high")
QUAT_NORM = (0.5, 2.0)
Shape = tuple[tuple[float, float, float], tuple[float, float, float]]  # (centre, size) in a body's own frame


def part_shapes(name: str) -> list[Shape]:
    """Every shape of a part's body as a box in its own frame: the primitive the task-file API makes and what the
    hook adds to it (looks-only shapes too: the framework's outline takes in all of a body's geometry)."""
    if name in CARTRIDGES:
        top = CART_LEN / 2 + TAB_H / 2
        return [
            ((0.0, 0.0, 0.0), (2 * CART_R, 2 * CART_R, CART_LEN)),
            ((-TAB_X, 0.0, top), TAB_SIZE),
            ((TAB_X, 0.0, top), TAB_SIZE),
            ((0.0, 0.0, top), (2 * BOSS_R, 2 * BOSS_R, TAB_H)),
            *(((0.0, 0.0, z), (2 * O_RING_R, 2 * O_RING_R, O_RING_H)) for z in O_RING_Z),
            *SHELL_WINDOWS,
        ]
    if name in DETENT_BALLS.values():
        return [((0.0, 0.0, 0.0), (2 * DETENT_R, 2 * DETENT_R, 2 * DETENT_R))]
    if name in CARTRIDGES.values():
        stem = ((0.0, 0.0, 0.0), (2 * STEM_R, 2 * STEM_R, STEM_LEN))
        return [stem, NOTCH_SIDE, NOTCH_TOP, PISTON, PISTON_PORT]
    leg_x = CLIP_T / 2 + LEG_LEN / 2
    return [
        ((0.0, 0.0, 0.0), BIGHT_SIZE),
        ((leg_x, LEG_Y, 0.0), (LEG_LEN, LEG_W, CLIP_T)),
        ((leg_x, -LEG_Y, 0.0), (LEG_LEN, LEG_W, CLIP_T)),
        ((0.0, 0.0, CLIP_T / 2 + TAB_UP / 2), (CLIP_T, GRAB_TAB_W, TAB_UP)),
    ]


def local_outline(name: str) -> tuple[np.ndarray, np.ndarray]:
    """A part's outline in its own frame (low and high corners), as built."""
    shapes = part_shapes(name)
    lows = [np.subtract(c, np.divide(size, 2)) for c, size in shapes]
    highs = [np.add(c, np.divide(size, 2)) for c, size in shapes]
    return np.min(lows, axis=0), np.max(highs, axis=0)


def parts_built() -> bool:
    """Whether the hook built every hooked object's parts (read from the scene module's record; no record, no
    faucet)."""
    scene = sys.modules.get(_SCENE_MODULE)
    record = getattr(scene, BUILD_RECORD, None) if scene is not None else None
    return isinstance(record, dict) and all(record.get(name) == "built" for name in AUTHORS)


def all_finite(objects: Mapping[str, Any], grippers: Sequence[Gripper] | None) -> bool:
    """Every object's position, orientation, velocities and outline, and every gripper's joint readings, tool
    point, opening and axes, are finite numbers."""
    try:
        values = [np.asarray(getattr(st, f), dtype=float) for st in objects.values() for f in _STATE_FIELDS]
        for g in grippers or ():
            values += [np.asarray(v, dtype=float) for v in (g.joints, g.tcp, g.opening, g.approach, g.closing)]
        if not all(bool(np.all(np.isfinite(v))) for v in values):
            return False
        # An orientation must be a rotation: a quaternion far from unit length (a zero one, say) is not.
        norms = [float(np.linalg.norm(np.asarray(st.quat_xyzw, dtype=float))) for st in objects.values()]
        return all(QUAT_NORM[0] <= n <= QUAT_NORM[1] for n in norms)
    except (LookupError, AttributeError, TypeError, ValueError, ArithmeticError):
        return False


def _mm(v: Any) -> str:
    return "(" + ", ".join(f"{1000 * float(x):.2f}" for x in np.ravel(v)) + ")"


def geometry_problems(objects: Mapping[str, Any]) -> list[str]:
    """A trial's first step: each part's reported outline against the shapes built, its body where it was put (or
    settled at most SETTLE lower), each stem and detent ball where its joint holds it on its cartridge, and the
    vanity's outline taking in the valve body and the holder."""
    spawn = {o.name: np.asarray(o.pos, dtype=float) for o in OBJECTS}
    origins: dict[str, np.ndarray] = {}
    rots: dict[str, np.ndarray] = {}
    problems = []
    try:
        for name in PARTS:
            st = objects[name]
            rot = rotation(st.quat_xyzw)
            low, high = local_outline(name)
            built = np.abs(rot) @ (high - low)  # the world outline of the turned shapes' outline
            seen = np.asarray(st.high, dtype=float) - np.asarray(st.low, dtype=float)
            if float(np.max(np.abs(seen - built))) > GEOMETRY_TOL:
                problems.append(f"{name}'s outline is {_mm(seen)} mm, built {_mm(built)} mm")
            middle = (np.asarray(st.low, dtype=float) + np.asarray(st.high, dtype=float)) / 2
            origins[name], rots[name] = middle - rot @ ((low + high) / 2), rot
            d = origins[name] - spawn[name]
            placed = math.hypot(d[0], d[1]) <= GEOMETRY_TOL and -SETTLE - GEOMETRY_TOL <= d[2] <= GEOMETRY_TOL
            if name not in CARTRIDGES.values() and not placed:  # a stem: its joint, checked below
                problems.append(f"{name} starts {_mm(d)} mm from where it was put")
        for cart, stem in CARTRIDGES.items():
            rel = rots[cart].T @ (origins[stem] - origins[cart])
            slide = float(rel[2]) - STEM_ABOVE_CART
            tilt = math.acos(max(-1.0, min(1.0, float(rots[cart][:, 2] @ rots[stem][:, 2]))))
            if (
                math.hypot(rel[0], rel[1]) > GEOMETRY_TOL
                or not -GEOMETRY_TOL <= slide <= STEM_TRAVEL + GEOMETRY_TOL
                or tilt > STEM_TILT_TOL
            ):
                problems.append(
                    f"{stem} is off its joint: {_mm(rel)} mm from {cart}, tilted {math.degrees(tilt):.1f} deg"
                )
        for cart, ball in DETENT_BALLS.items():  # each detent ball in its cartridge's top, against the stem
            st = objects[ball]
            low, high = local_outline(ball)
            built = np.abs(rotation(st.quat_xyzw)) @ (high - low)
            seen = np.asarray(st.high, dtype=float) - np.asarray(st.low, dtype=float)
            if float(np.max(np.abs(seen - built))) > GEOMETRY_TOL:
                problems.append(f"{ball}'s outline is {_mm(seen)} mm, built {_mm(built)} mm")
            middle = (np.asarray(st.low, dtype=float) + np.asarray(st.high, dtype=float)) / 2
            rel = rots[cart].T @ (middle - origins[cart]) - np.array([DETENT_X, 0.0, DETENT_Z])
            if not (
                -GEOMETRY_TOL <= rel[0] <= DETENT_TRAVEL + GEOMETRY_TOL
                and math.hypot(rel[1], rel[2]) <= GEOMETRY_TOL
            ):
                problems.append(f"{ball} is off its joint: {_mm(rel)} mm from its place in {cart}")
        problems += _vanity_problems(objects[VANITY])
    except (LookupError, AttributeError, TypeError, ValueError, ArithmeticError) as error:
        problems.append(f"can't read the parts ({type(error).__name__}: {error})")
    return problems


_VANITY_OUTLINE: list[tuple[np.ndarray, np.ndarray]] = []  # computed once, on the first trial


def _vanity_problems(st: Any) -> list[str]:
    """The vanity's reported outline must take in the valve body and the holder the hook built under it, and stand
    where it was put."""
    if not _VANITY_OUTLINE:
        _VANITY_OUTLINE.append(vanity_outline())
    low, high = _VANITY_OUTLINE[0]
    rot = rotation(st.quat_xyzw)
    built = np.abs(rot) @ (high - low)
    seen = np.asarray(st.high, dtype=float) - np.asarray(st.low, dtype=float)
    problems = []
    if float(np.max(np.abs(seen - built))) > GEOMETRY_TOL:
        problems.append(
            f"the vanity's outline (with the valve body and holder) is {_mm(seen)} mm, built {_mm(built)} mm"
        )
    middle = (np.asarray(st.low, dtype=float) + np.asarray(st.high, dtype=float)) / 2
    if float(np.linalg.norm(middle - rot @ ((low + high) / 2) - np.asarray(VANITY_CENTER))) > GEOMETRY_TOL:
        problems.append("the vanity is not where it was put")
    return problems


# --- the score ---------------------------------------------------------------------------------------------------
# The tests a plumber makes (README.md): before, lift the old stem (the fault: it binds); after, with the clip home,
# lift the new stem to full-on, swing it to the full-hot and full-cold ends, and leave it off.
BEFORE_LIFT = 0.0015  # the before-test: the old stem lifted this far while the clip is still where it was put
AFTER_LIFT = 0.0055  # the after-test: the new stem lifted this far (full-on is 6 mm) while clipped
END_TOL = math.radians(10.0)  # ... and swung to within this of the full-hot and the full-cold end
OFF_TOL = 0.001  # off: the new stem down within this of its lower stop
CLIP_MOVED = 0.002  # the clip has been moved once its body is this far from where it was put


def body_origin(name: str, part: PartView) -> np.ndarray:
    """A part's body origin, from its outline's centre and its turn (the outline is its shapes' box)."""
    low, high = local_outline(name)
    return np.asarray(part.center - part.rot @ ((low + high) / 2), dtype=float)


def stem_lift(parts: Mapping[str, PartView], cart: str) -> float:
    """How far a cartridge's stem stands up its slide (0 down, off; 6 mm full-on), measured along the cartridge."""
    stem = CARTRIDGES[cart]
    rel = parts[cart].rot.T @ (body_origin(stem, parts[stem]) - body_origin(cart, parts[cart]))
    return float(rel[2]) - STEM_ABOVE_CART


@dataclass(frozen=True)
class State:
    """One step's facts. ``clear`` is None when the arms' state is unknown."""

    lost: tuple[str, ...]
    old_out: bool
    old_in_tray: bool
    new_seated: bool
    clip_home: bool  # the clip retaining the new cartridge (its legs on the slots' floors, the cartridge home)
    hot_left: bool  # the piston's port set so that the handle's left swing opens the hot inlet
    still: bool
    clear: bool | None
    old_lift: float = 0.0  # the old stem's lift up its slide (m)
    new_lift: float = 0.0  # the new stem's
    clip_untouched: bool = False  # the clip still where it was put
    new_depth: float = 0.0  # the new cartridge's depth above its seat, on its axis (m)
    new_in_bore: bool = False
    at_hot_end: bool = False  # the new stem swung to the full-hot end (the port on the hot inlet)
    at_cold_end: bool = False
    stem_off: bool = False  # the new stem down: the faucet off
    clip_across: bool = False  # the clip in its slots across the bore, however high in them
    retained: bool = False  # the new cartridge keyed in the bore and its clip retaining it, within the slots' play
    clip_out: bool = False  # the clip out of a cartridge's way going in: clear of the bore in plan (clip_out)
    clip_low: float = math.nan  # the clip's lowest point (m; NaN unless all its readings are finite) ...
    clip_high: float = math.nan  # ... and its highest
    clip_gap: float = math.nan  # how near the clip comes to the bore's axis in plan (m; clip_gap)
    old_in_body: bool = False  # the old cartridge still in the valve body (out_of_body's complement, NaN-safe)
    old_depth: float = 0.0  # the old cartridge's depth above its seat, on its axis (m)

    @property
    def clip_out_old(self) -> bool:
        """Out of the old cartridge's way as it comes out: clip_out, or the clip wholly on or above the rim (the
        valve body doesn't hold it there: the rising cartridge would lift it off), or wholly below the slots' floor
        (over the bore, only the old cartridge and the valve body are there). Unread (NaN) heights leave clip_out."""
        return self.clip_out or self.clip_low >= ON_RIM or self.clip_high < Z_SLOT_FLOOR

    @property
    def clip_over_new(self) -> bool:
        """The clip over a cartridge's shoulder going in, as the forced checks read it: not out of its way, and
        within FORCED_GAP of the bore's axis (more than just touching its side). An unread (NaN) gap counts."""
        return not (self.clip_out or self.clip_gap >= FORCED_GAP)

    @property
    def clip_over_old(self) -> bool:
        """The same for the old cartridge coming out: not out of its way (clip_out_old), and within FORCED_GAP."""
        return not (self.clip_out_old or self.clip_gap >= FORCED_GAP)

    @property
    def full(self) -> bool:
        return (
            not self.lost
            and self.old_out
            and self.old_in_tray
            and self.new_seated
            and self.clip_home
            and self.hot_left
            and self.stem_off
            and self.still
            and self.clear is True
        )


def state(parts: Mapping[str, PartView], grippers: Sequence[Gripper] | None) -> State:
    new = parts[NEW_CART]
    new_lift = stem_lift(parts, NEW_CART)
    clip_at = body_origin(CLIP, parts[CLIP])
    return State(
        lost=tuple(name for name in NEW_PARTS if parts[name].lost),
        old_out=out_of_body(parts[OLD_CART]),
        old_in_tray=in_tray(parts[OLD_CART]),
        new_seated=seated(new),
        clip_home=clip_home(parts[CLIP]) and depth(new) <= CLIPPED_DEPTH,
        hot_left=hot_error(parts[NEW_STEM]) <= PORT_TOL,
        still=all(parts[name].still for name in (NEW_CART, NEW_STEM, CLIP)),
        clear=None if grippers is None else hands_clear(parts, grippers),
        old_lift=stem_lift(parts, OLD_CART),
        new_lift=new_lift,
        clip_untouched=float(np.linalg.norm(clip_at - np.asarray(CLIP_SPEC.pos, dtype=float))) <= CLIP_MOVED,
        new_depth=depth(new),
        new_in_bore=axis_offset(new) <= SEAT_AXIS_TOL * 2,
        at_hot_end=angle_between(port_bearing(parts[NEW_STEM]), HOT_BEARING) <= END_TOL,
        at_cold_end=angle_between(port_bearing(parts[NEW_STEM]), COLD_BEARING) <= END_TOL,
        stem_off=new_lift <= OFF_TOL,
        clip_across=clip_across(parts[CLIP]),
        retained=clip_retaining(parts[CLIP]) and keyed(new) and held_down(new, parts[CLIP]),
        clip_out=clip_out(parts[CLIP]),
        clip_low=float(parts[CLIP].low[2]) if readable(parts[CLIP]) else math.nan,
        clip_high=float(parts[CLIP].high[2]) if readable(parts[CLIP]) else math.nan,
        clip_gap=clip_gap(parts[CLIP]),
        old_in_body=in_body(parts[OLD_CART]),
        old_depth=depth(parts[OLD_CART]),
    )


UNREADABLE = State(
    lost=(),
    old_out=False,
    old_in_tray=False,
    new_seated=False,
    clip_home=False,
    hot_left=False,
    still=False,
    clear=None,
    new_depth=math.nan,
    old_depth=math.nan,
)  # what a step whose parts can't be read stands for (the trial is void by then; it clears no latch either)


@dataclass
class Ledger:
    """What carries over between steps this trial."""

    steps: int = 0
    full_steps: int = 0
    lost: bool = False
    void: str | None = None  # why this trial can't be scored (a non-finite reading, a geometry mismatch)
    clip_moved: bool = False  # the clip has left where it was put: the before-test's window has closed
    before_tested: bool = False  # before that, the old stem lifted in the valve body (the fault seen)
    clipped_seen: bool = False  # the new cartridge has been clipped home
    after_lifted: bool = False  # since then: the new stem lifted to full-on while clipped ...
    after_hot: bool = False  # ... and, lifted, swung to the full-hot end ...
    after_cold: bool = False  # ... and to the full-cold end
    retention_failed: bool = False  # since then, the new cartridge has risen past its slots' play under the clip
    # The order a cartridge can only be changed in (README.md): the clip comes out of the old cartridge's way
    # (State.clip_out_old) before it can come out, the new one goes home with the clip out of its way (clip_out), and
    # the clip goes back across it once it is home. A state reached any other way (a solver letting a cartridge
    # through a clip that is in its way) earns nothing.
    clip_freed: bool = False  # the clip seen out of the way while the old cartridge was still in the valve body
    old_forced: bool = False  # the old cartridge forced past the clip in its way
    new_forced: bool = False  # the new one forced past it since it last stood clear above the slots or off the axis
    homed_open: bool = False  # the new cartridge seen home while the clip was out of its way, and not forced
    clip_over_home: bool = False  # the clip's latest return into a cartridge's way found the new cartridge home
    clip_was_out: bool = False  # last step: the clip out of a cartridge's way going in (for its return) ...
    clip_was_over_new: bool = False  # ... over a cartridge's shoulder going in (for the forced checks) ...
    clip_was_over_old: bool = False  # ... and over the old one's
    old_was_down: bool = False  # last step: the old cartridge in the body, its shoulder under the clip's level
    new_was_up: bool = False  # last step: the new one clear above the slots, or off the bore's axis

    @property
    def after_tested(self) -> bool:
        return self.after_lifted and self.after_hot and self.after_cold

    @property
    def old_out_in_order(self) -> bool:
        """The old cartridge's way out was opened first, and it was never forced past the clip."""
        return self.clip_freed and not self.old_forced

    @property
    def in_order(self) -> bool:
        """The clip out of the way before the old cartridge, the new one home with it out, the clip back over it."""
        return self.old_out_in_order and self.homed_open and self.clip_over_home

    def complete(self, now: State) -> bool:
        """Everything 1.0 needs this step: the job done in order, both tests made, the faucet off."""
        return (
            now.full
            and self.in_order
            and self.before_tested
            and self.after_tested
            and not self.retention_failed
        )

    def observe(self, now: State) -> None:
        self.steps += 1
        self.lost = self.lost or bool(now.lost)
        self.clip_moved = self.clip_moved or not now.clip_untouched
        if not self.clip_moved and not now.old_out and now.old_lift >= BEFORE_LIFT:
            self.before_tested = True
        self.clip_freed = self.clip_freed or (now.clip_out_old and now.old_in_body)
        # A cartridge forced past the clip (a solver let it through): seen with its shoulder more than FORCED_RISE
        # above its seat while its bottom is still below the slots' top (CLEAR_OF_SLOTS) and the clip over its
        # shoulder (State.clip_over_new, clip_over_old: closer than FORCED_GAP, not just touching), or gone from one
        # side of that span to the other in one step with the clip over its shoulder at either end. (Crossing the
        # 64 mm between takes the demo dozens of steps, each checked; the one-step check is a backstop for a step the
        # readings skipped.) The old one earns nothing for good, whatever was opened before.
        old_down = now.old_in_body and now.old_depth <= FORCED_RISE
        old_across = now.old_in_body and FORCED_RISE < now.old_depth < CLEAR_OF_SLOTS
        old_jumped = self.old_was_down and not (old_down or old_across)
        if (old_across and now.clip_over_old) or (old_jumped and (now.clip_over_old or self.clip_was_over_old)):
            self.old_forced = True
        self.old_was_down = old_down
        self.clip_was_over_old = now.clip_over_old
        # The new one, forced past it, loses any earlier credit for being home and clipped, and counts as home again
        # only once it has stood clear above the slots (or off the bore's axis) and gone home with the clip out of
        # its way.
        new_down = now.new_in_bore and now.new_depth <= FORCED_RISE
        new_across = now.new_in_bore and FORCED_RISE < now.new_depth < CLEAR_OF_SLOTS
        new_up = math.isfinite(now.new_depth) and not (new_down or new_across)
        new_jumped = self.new_was_up and new_down
        if (new_across and now.clip_over_new) or (new_jumped and (now.clip_over_new or self.clip_was_over_new)):
            self.new_forced = True
            self.homed_open = False
            self.clip_over_home = False
        elif new_up:
            self.new_forced = False
        self.new_was_up = new_up
        # Forced more than the slots' play up its bore with the clip in its way, it goes back to needing to go home
        # with the clip out of its way, and to be clipped again.
        if now.new_in_bore and now.new_depth > RETAIN_RISE and not now.clip_out:
            self.homed_open = False
            self.clip_over_home = False
        self.homed_open = self.homed_open or (now.clip_out and now.new_seated and not self.new_forced)
        if self.clip_was_out and not now.clip_out:  # the clip came back into a cartridge's way: over what?
            self.clip_over_home = now.new_seated
        self.clip_was_out = now.clip_out
        self.clip_was_over_new = now.clip_over_new
        if now.clip_home and now.new_seated and self.clip_over_home:
            self.clipped_seen = True
        if self.clipped_seen:
            if now.clip_across and now.new_in_bore and now.new_depth > RETAIN_RISE:
                self.retention_failed = True  # the clip, in its slots, let it rise past their play: it didn't hold
            if now.retained:  # in the faucet and held by its clip, however far the lift draws it up the slots' play
                on = now.new_lift >= AFTER_LIFT  # full-on: the ends are checked with the faucet open
                self.after_lifted = self.after_lifted or on
                self.after_hot = self.after_hot or (on and now.at_hot_end)
                self.after_cold = self.after_cold or (on and now.at_cold_end)
        self.full_steps = self.full_steps + 1 if self.complete(now) else 0

    def spoil(self, reason: str) -> None:
        if self.void is None:
            self.void = reason


@dataclass(frozen=True)
class Grade:
    state: State
    full_steps: int
    lost: bool
    score: float
    void: str | None = None
    built: bool = False
    complete: bool = False  # everything 1.0 needs, this step (before the 1 s hold)

    @property
    def counts(self) -> bool:
        """The trial can be scored: the parts were built, and nothing has voided it or left the table."""
        return self.built and self.void is None and not self.lost


def grade(ledger: Ledger, now: State, built: bool, hold_steps: int = HOLD_STEPS) -> Grade:
    """The ladder (README.md): 0 unless the faucet's parts were built, once the trial is void (a non-finite reading,
    or a part not as built on its first step), once the new cartridge, its stem or the clip has left the table,
    while the old cartridge is still in the valve body, or when the arms can't be read. Otherwise 0.25 with the old
    cartridge out; 0.5 with the new one pushed fully home (keyed by its tabs), however its stem is set; 0.75 with
    the clip retaining it, at rest (and it has not risen past the slots' play since it was first clipped), and its
    piston's port set for hot on the left; 1.0 when also the faucet was tested before (the old stem lifted while
    the clip was untouched) and after (the new stem lifted to full-on and swung to both ends while its clip retained
    the cartridge), it is left off, the old cartridge is in
    the parts tray, the hands are clear and everything is still, held for 1 s. Each rung counts only if reached in
    the order a cartridge can be changed in: the old cartridge out once the clip had come out, the new one home with
    the bore open, clipped by the clip's return across it once home (Ledger.in_order)."""
    if (
        not built
        or ledger.void is not None
        or ledger.lost
        or now.lost
        or not (now.old_out and ledger.old_out_in_order)
        or now.clear is None
    ):
        score = 0.0
    else:
        score = LADDER[0]
        if now.new_seated and ledger.homed_open:
            score = LADDER[1]
            if (
                now.clip_home
                and ledger.clip_over_home
                and now.hot_left
                and not ledger.retention_failed
            ):
                score = LADDER[2]
                if ledger.complete(now) and ledger.full_steps >= hold_steps:
                    score = LADDER[3]
    return Grade(now, ledger.full_steps, ledger.lost, score, ledger.void, built, ledger.complete(now))


# --- episodes and the robot's state ----------------------------------------------------------------------------------
_LEDGER = Ledger()
_GRADE: Grade | None = None
_last_objects: object = None  # the step's state dict, kept alive so its id can't be reused
_embodiment: Any = None
_last_clock: float | None = None  # the embodiment's episode clock at the last step
_hooked = False
_warned: set[str] = set()
_ARMS = (("left_robot", "left"), ("right_robot", "right"))
RESET_TAG = "replace_faucet_cartridge_resets"  # the callback list on a hooked reset


def _warn_once(key: str, message: str) -> None:
    """Warn once per key. Never raises: with warnings turned into errors (-W error), a warning must not
    become the failure it reports."""
    if key not in _warned:
        _warned.add(key)
        with contextlib.suppress(Exception):
            warnings.warn(f"replace_faucet_cartridge: {message}", RuntimeWarning, stacklevel=3)


def new_episode() -> None:
    global _LEDGER, _GRADE, _last_objects, _last_clock
    _LEDGER = Ledger()
    _GRADE = None
    _last_objects = None
    _last_clock = None
    _warned.difference_update(("robot", "void"))  # warn again in every trial this happens in


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


def _reset_callbacks(embodiment: Any) -> list[Callable[[], None]] | None:
    """The list of new-trial callbacks on the embodiment's reset, wrapping it the first time. Every copy of this
    file loaded in the process adds its own new_episode to the one list, so each copy's ledger resets."""
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
    """Task files get no reset hook, so hook the embodiment's reset once: every new trial then starts a fresh ledger.
    The embodiment is also where the arms' joint positions come from."""
    global _hooked, _embodiment
    if _hooked:
        return
    embodiment = _calling_embodiment()
    if embodiment is not None:
        callbacks = _reset_callbacks(embodiment)
        if callbacks is not None:  # only an embodiment whose reset is hooked is read from
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
    """A second way to see a new trial, besides the hooked reset: the embodiment's episode clock (``_t``) running
    backwards."""
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


def _hold_steps() -> int:
    """Control steps in the 1 s a full score must hold: from the embodiment's control rate when it can be read,
    else HOLD_STEPS (20 Hz, as the run commands set)."""
    try:
        return max(1, round(float(_embodiment.info.control_hz)))
    except (LookupError, AttributeError, TypeError, ValueError, ArithmeticError):  # inf, nan, missing
        return HOLD_STEPS


def _numpy(values: Any) -> np.ndarray:
    for step in ("detach", "cpu", "numpy"):
        if hasattr(values, step):
            values = getattr(values, step)()
    return np.asarray(values, dtype=float)


def _xyz(v: Any) -> tuple[float, float, float]:
    return (float(v[0]), float(v[1]), float(v[2]))


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
                joints=tuple(float(v) for v in q),
            )
        )
    return out


def _robot() -> list[Gripper] | None:
    if _embodiment is None:
        return None
    try:
        return grippers_of(_embodiment)
    # An unexpected embodiment: without the arms "hands clear" can't be checked, so that step scores 0. Read again
    # next step (a one-off failure doesn't switch reading off), and warn in every trial it happens in.
    except Exception as error:
        _warn_once(
            "robot",
            f"can't read the arms' state ({type(error).__name__}: {error}); each such step scores 0",
        )
        return None


def _update(objects: Mapping[str, Any]) -> Grade:
    """Advance the episode once per control step (success() and score() share the step's state dict)."""
    global _last_objects, _GRADE
    if objects is _last_objects and _GRADE is not None:
        return _GRADE
    try:
        _hook_resets()
    except Exception as error:  # an odd caller on the stack: carry on without the hook
        _warn_once("hook", f"could not look for the embodiment ({type(error).__name__}: {error})")
    _restart_if_clock_went_back()
    grippers = _robot()
    if not all_finite(objects, grippers):
        _LEDGER.spoil("a non-finite (or degenerate) position, orientation, velocity, outline or joint reading")
    elif _LEDGER.steps == 0:
        problems = geometry_problems(objects)
        if problems:
            _LEDGER.spoil("the parts are not as built: " + "; ".join(problems))
    if _LEDGER.void is not None:
        _warn_once("void", f"this trial scores 0: {_LEDGER.void}")
    built = parts_built()
    if not built:
        _warn_once("built", "the faucet's parts were not built (see earlier warnings): every trial scores 0")
    try:
        now = state(views(objects), grippers)
    except (LookupError, AttributeError, TypeError, ValueError, ArithmeticError) as error:  # unreadable: void
        _LEDGER.spoil(f"can't read the parts ({type(error).__name__}: {error})")
        _warn_once("void", f"this trial scores 0: {_LEDGER.void}")
        now = UNREADABLE
    _LEDGER.observe(now)
    _GRADE = grade(_LEDGER, now, built, _hold_steps())
    _last_objects = objects
    return _GRADE


# --- what the embodiment calls ----------------------------------------------------------------------------------------
def score(objects: Mapping[str, Any]) -> float:
    """1.0: the new cartridge home under its clip, its stem set for hot on the left, the old one in the tray; see
    README.md."""
    return _update(objects).score


def success(objects: Mapping[str, Any]) -> bool:
    """Everything 1.0 needs, this step (the embodiment ends the trial once this holds for 1 s, when score() reaches
    1.0 too)."""
    grade_now = _update(objects)
    return grade_now.complete and grade_now.counts


# --- the scripted demo ------------------------------------------------------------------------------------------------
# The right arm does the job (the faucet stands in front of it, where its straight-down grips reach highest; the
# left arm's can't reach across there), as a plumber does it one-handed: test the old stem (it binds 2 mm up and 15
# degrees round: the fault), slide the clip out by its tab (fingers
# closing across the tab), pull the old cartridge out by its stem and drop it in the parts tray, carry the new one
# over by its body with its tabs lined up, let it go part way into the bore and push it home by its stem's top with
# closed fingers, turn its stem a quarter turn so
# the notch faces the front, push the clip back in with closed fingers (the spout keeps the slid-out clip level),
# and test the new stem (full-on, full hot and back, full cold and back, off). The left arm stays parked. Waypoint
# yaw 0 closes the fingers along world y, pi/2 along world x. Every point comes from the layout above; nothing is
# read from the simulator.
PARK = {"left": (0.20, 0.27, TOP + 0.17), "right": (0.20, -0.27, TOP + 0.17)}
CARRY_Z = (
    TOP + 0.200
)  # TCP height while carrying a cartridge by its stem: its bottom 2 cm above the rim
PULL_Z = (
    TOP + 0.205
)  # where the pull out of the bore aims (the arm lags behind it while the O-rings drag)
STEM_GRIP_BELOW_TOP = 0.002  # the TCP 2 mm below a stem's top: the pads hold its top 15 mm
TAB_AJAR = FINGER_OPEN * 0.30  # about 28 mm apart round the clip's 1.5 mm tab
TAB_RELEASE = -0.012 / 2  # let go of the tab by opening to 12 mm: clear of it, and of the stem beyond it
BODY_AJAR = FINGER_OPEN * 0.40  # about 38 mm apart round a cartridge's 24 mm body (clear of its tabs)
# The clip goes home pushed, not pinched: closed fingers push its tab's front face, so the slots set its yaw. (A
# pinch locks the clip's yaw to the wrist's, and the legs in the front and back slots allow only 1.6 degrees: a
# pinched clip a degree and a half off wedged and sprang out of its slots in the simulator.) Nothing then pulls it
# back as the fingers leave. Pushed on the tab's flat face, 3 mm above the bight, the clip stops within 0.1 mm of
# the fingers' meshes' mark in the simulator. But pushed across the back wall's outer face (which the legs' tips
# meet from 16.4 mm behind the axis, at their outer edges), their ends caught on it and the clip hopped: 1 degree
# at 17.5 mm, and at 17.9 mm a leg was thrown out of its slot onto the rim. (Placed there, as it starts, it sits
# still.) So the push slows for its last 8 mm and stops with the tips at 16 mm, inside that face: 3.6 mm short of
# home, the legs still reaching 4.5 mm past what "clipped" needs.
CLIP_SHORT = 0.0036  # the push stops this far short of home (by the fingers' meshes)
CLIP_SLOW = 0.008  # ... and covers its last 8 mm slowly
PUSH_FACE = 0.0039  # closed, the fingertips' outer faces stand this far either side of the TCP along their closing
PUSH_GAP = 0.004  # the fingers come down this far in front of the tab, and back off this far before rising
STEM_AJAR = FINGER_OPEN * 0.40  # about 38 mm apart round an 8 mm stem
CLIP_GRIP_Z = (
    Z_RIM + 0.002 + PAD_TIP_PAST_TCP
)  # the pad tips 2 mm above the rim: the tab's top 5 mm in the pads
CLIP_PUSH_Z = CLIP_Z + CLIP_T / 2 + 0.003 + PAD_TIP_PAST_TCP  # pushing: the fingertips 3 mm over the bight
CLIP_PULL = (
    0.031  # far enough for the legs' tips to clear the cartridge's shoulder, still in their slots
)
PRESS = 0.032  # push this far past "home": the arm's position control is compliant (about 2 mm per N here)
END_HOLD = 2.5
TURN_S = 3.0  # a quarter turn of the stem, against its damping
# The tests: a stem is lifted to aim this far past its full-on stop (6 mm), for the arm's lag against the detent's
# drag (the stiff old one binds well short of it); pushed off aiming this far past its lower stop; and the old one
# is tried with a 30 degree turn (it binds at 15).
TEST_OVER = 0.006
OFF_PRESS = 0.006
# The after-test's lift aims 1 mm past full-on and holds there, from the lags measured in the simulator: the arm
# settled 0.5 mm short of its aim, and the stem slipped up to 0.5 mm down the grip as a swing began, so the stem
# still ends at its stop (aiming 0.3 mm past, it reached only 5.49 mm). Pressing on the stop draws the clipped
# cartridge up, lifting the clip's legs off their floors and letting the clip creep out of its slots: aiming 2.5 mm
# past, the arm pressed with 1.2 mm of error beyond its free 0.5 mm, and the cartridge rose 1 mm. Aiming 1 mm past
# presses with at most 0.5 mm, under half that.
AFTER_OVER = 0.001
GRIP_S = 1.2  # the after-test closes its fingers on the stem this slowly ...
SETTLE_S = 0.5  # ... and lets the grip, each lift and each swing's end settle this long
# The new cartridge's way in. Held by its 5 mm boss it pivoted in the grip: in three trials of the same demo in the
# simulator (the resets differ by tenths of a millimetre) it hung 2.6, 0.5 and 4.1 degrees off vertical, and the bore's lead-in is
# only 1.5 mm (14 mm at the rim over a 12.5 mm bore). At 4.1 degrees it jammed in the bore's mouth; the push tipped
# it to 12 degrees, and it dropped in turned 6.1 degrees and stayed 0.7 mm proud on its tabs, so the clip's legs
# met its shoulder and the clip couldn't go home. Hurried, the same happened earlier (lifted out of its holder at
# full speed, the arm's sideways lag dragged it on the holder's wall; lowered at full speed, it twisted 8 degrees).
# So it is held by its body: the pads on its top BODY_GRIP_BELOW, three times the boss's height, closing across it
# (yaw 0, clear of its tabs). It is drawn out of the holder slowly, hung still just over the rim and eased in, then
# lowered until it is RELEASE_DEPTH in the bore, still above the ball plungers: there the bore's 0.5 mm play holds
# it within 1.5 degrees of upright whatever the grip did. It is let go, and settles on the plungers' balls. The
# closed fingertips then push it home by its stem's top, straight down its axis, which can't tip it; the stem
# rests on its lower stop, so the push goes into the cartridge.
HOLDER_CLEAR = 0.025  # drawn up this far slowly: out of its 20 mm holder
ENTRY_ABOVE = 0.003  # it hangs with its bottom this far above the rim ...
ENTRY_DEPTH = 0.015  # ... is eased this far down, into the bore ...
RELEASE_DEPTH = 0.039  # ... and lowered on until its bottom is this far below the rim, where it is let go
BODY_GRIP_BELOW = 0.014  # holding the body: the pad tips this far below its top shoulder
BODY_ABOVE = 0.055  # the fingers come down round it from this far above the grip: the tips 6 mm over the stem's top
# Let go in the bore, a cartridge comes to rest on the two ball plungers: its bottom edge (the body's radius) meets
# the balls, which stand 1 mm proud of the bore, this high.
REST_BOTTOM = PAD_Z + math.sqrt(PAD_R**2 - (PAD_EXTENDED - CART_R) ** 2)
TRY_TURN = math.radians(30.0)
STEM_TOP_SEATED = Z_RIM + STEM_ABOVE
STEM_ABOVE_BOTTOM = STEM_TOP_SEATED - (Z_FLOOR + SEAT_GAP)  # a lowered stem's top above its cartridge's bottom
STEM_PUSH_TCP = STEM_TOP_SEATED + PAD_TIP_PAST_TCP  # the closed fingertips on a seated cartridge's stem top
# The old cartridge hanging from its stem (pulled up to its stop, which the stiff old stem reaches at 2 mm) reaches
# this far below the TCP. (The new one is carried by its body: see body_tcp.)
HANG = STEM_TOP_SEATED - STEM_GRIP_BELOW_TOP - (Z_FLOOR + SEAT_GAP) - OLD_STEM_TRAVEL


def body_tcp(bottom_z: float) -> float:
    """The TCP's height holding a cartridge by its body (the pad tips BODY_GRIP_BELOW under its top shoulder) with
    its bottom at ``bottom_z``."""
    return bottom_z + CART_LEN - BODY_GRIP_BELOW + PAD_TIP_PAST_TCP


def _pad(plan: list[Waypoint], until: float, label: str) -> list[Waypoint]:
    """Hold the plan's last pose until ``until`` seconds."""
    total = sum(w.duration for w in plan)
    if until > total + 1e-9:
        last = plan[-1]
        plan = [*plan, Waypoint(until - total, last.pos, last.finger, label, last.yaw)]
    return plan


def _at(x: float, y: float, z: float) -> tuple[float, float, float]:
    return (float(x), float(y), float(z))


def clip_moves(start_x: float, end_x: float, label: str) -> list[Waypoint]:
    """Pinch the clip's tab at ``start_x`` (its bight's centre), slide it to ``end_x``, let go and rise."""
    yaw = math.pi / 2
    y, z = FAUCET_XY[1], CLIP_GRIP_Z
    return [
        Waypoint(1.5, _at(start_x, y, z + 0.04), TAB_AJAR, f"above the clip's tab ({label})", yaw),
        Waypoint(1.2, _at(start_x, y, z), TAB_AJAR, "round the tab", yaw),
        Waypoint(0.8, _at(start_x, y, z), FINGER_CLOSED, "pinch the tab", yaw),
        Waypoint(2.0, _at(end_x, y, z), FINGER_CLOSED, label, yaw),
        Waypoint(0.6, _at(end_x, y, z), TAB_RELEASE, "let go of the tab", yaw),
        Waypoint(0.8, _at(end_x, y, z + 0.04), TAB_RELEASE, "rise from the tab", yaw),
    ]


def push_tcp_x(bight_x: float) -> float:
    """The TCP's x with the closed fingers' face on the tab's front face, the clip's bight at ``bight_x``."""
    return bight_x - CLIP_T / 2 - PUSH_FACE


def clip_push(start_x: float, end_x: float, label: str) -> list[Waypoint]:
    """Push the clip from ``start_x`` (its bight's centre) to ``end_x`` with closed fingers on its tab's flat front
    face, slowing for the last CLIP_SLOW; back off and rise. Nothing holds the clip, so its slots set its yaw and
    nothing pulls it back."""
    yaw = math.pi / 2  # the fingers close along x: their broad outer faces face the tab
    y, z = FAUCET_XY[1], CLIP_PUSH_Z
    start, end = push_tcp_x(start_x) - PUSH_GAP, push_tcp_x(end_x)
    slow = max(start, end - CLIP_SLOW)
    return [
        Waypoint(1.5, _at(start, y, z + 0.04), FINGER_CLOSED, f"above the clip's tab ({label})", yaw),
        Waypoint(1.2, _at(start, y, z), FINGER_CLOSED, "in front of the tab", yaw),
        Waypoint(2.0, _at(slow, y, z), FINGER_CLOSED, "push the clip toward home", yaw),
        Waypoint(2.0, _at(end, y, z), FINGER_CLOSED, label, yaw),
        Waypoint(0.8, _at(end - PUSH_GAP, y, z), FINGER_CLOSED, "back off the tab", yaw),
        Waypoint(0.8, _at(end - PUSH_GAP, y, z + 0.04), FINGER_CLOSED, "rise from the clip", yaw),
    ]


def test_before(stem_tcp: float) -> list[Waypoint]:
    """Check the fault: grip the old stem, lift it (it binds about 2 mm up: it won't open fully), try a turn (it binds
    too), turn it back, push it down (off) and let go."""
    fx, fy = FAUCET_XY
    lift = _at(fx, fy, stem_tcp + STEM_TRAVEL + TEST_OVER)
    return [
        Waypoint(1.5, _at(fx, fy, stem_tcp + 0.04), STEM_AJAR, "above the old stem (test)"),
        Waypoint(1.2, _at(fx, fy, stem_tcp), STEM_AJAR, "round the old stem (test)"),
        Waypoint(0.8, _at(fx, fy, stem_tcp), FINGER_CLOSED, "grip the old stem (test)"),
        Waypoint(1.5, lift, FINGER_CLOSED, "lift it: it binds"),
        Waypoint(2.0, lift, FINGER_CLOSED, "try a turn: it binds", TRY_TURN),
        Waypoint(1.5, lift, FINGER_CLOSED, "turn it back"),
        Waypoint(1.5, _at(fx, fy, stem_tcp - OFF_PRESS), FINGER_CLOSED, "push it down (off)"),
        Waypoint(0.8, _at(fx, fy, stem_tcp), FINGER_CLOSED, "ease off the old stem"),
        Waypoint(0.6, _at(fx, fy, stem_tcp), STEM_AJAR, "let go of the old stem (test)"),
        Waypoint(0.8, _at(fx, fy, stem_tcp + 0.04), STEM_AJAR, "rise from the old stem (test)"),
    ]


def test_after(stem_tcp: float) -> list[Waypoint]:
    """Make sure it works, with the clip home: grip the new stem, lift it to full-on, swing it to the full-hot end and
    back to the centre, push it off; grip it again, lift it, swing it to the full-cold end and back, push it off;
    let go. The right arm's straight-down grips turn from yaw 0 to +90 degrees here, so each end takes its own grip.
    The two swings cancel: the stem ends set as it was (notch to the front). Each grip closes slowly and settles
    before the lift, the open fingers swing back slowly and settle before the second grip, and each swing holds at
    its end: in the simulator every quick close or reversal turned the stem in the grip."""
    fx, fy = FAUCET_XY
    q = math.pi / 2
    up = _at(fx, fy, stem_tcp + STEM_TRAVEL + AFTER_OVER)
    off = _at(fx, fy, stem_tcp - OFF_PRESS)
    down = _at(fx, fy, stem_tcp)
    return [
        Waypoint(1.5, _at(fx, fy, stem_tcp + 0.04), STEM_AJAR, "above the new stem (test)", q),
        Waypoint(1.2, down, STEM_AJAR, "round the new stem (test)", q),
        Waypoint(GRIP_S, down, FINGER_CLOSED, "grip the new stem (test)", q),
        Waypoint(SETTLE_S, down, FINGER_CLOSED, "settle the grip", q),
        Waypoint(1.5, up, FINGER_CLOSED, "lift it to full-on", q),
        Waypoint(SETTLE_S, up, FINGER_CLOSED, "hold it at full-on", q),
        Waypoint(TURN_S, up, FINGER_CLOSED, "swing it to full hot", 0.0),
        Waypoint(SETTLE_S, up, FINGER_CLOSED, "hold it at full hot", 0.0),
        Waypoint(TURN_S, up, FINGER_CLOSED, "back to the centre", q),
        Waypoint(1.5, off, FINGER_CLOSED, "push it down (off)", q),
        Waypoint(0.8, down, FINGER_CLOSED, "ease off the new stem", q),
        Waypoint(0.6, down, STEM_AJAR, "let go of the new stem (test)", q),
        Waypoint(2.0, down, STEM_AJAR, "swing the open fingers back", 0.0),
        Waypoint(SETTLE_S, down, STEM_AJAR, "settle the open fingers", 0.0),
        Waypoint(GRIP_S, down, FINGER_CLOSED, "grip the new stem again (test)", 0.0),
        Waypoint(SETTLE_S, down, FINGER_CLOSED, "settle the grip again", 0.0),
        Waypoint(1.5, up, FINGER_CLOSED, "lift it to full-on again", 0.0),
        Waypoint(SETTLE_S, up, FINGER_CLOSED, "hold it at full-on again", 0.0),
        Waypoint(TURN_S, up, FINGER_CLOSED, "swing it to full cold", q),
        Waypoint(SETTLE_S, up, FINGER_CLOSED, "hold it at full cold", q),
        Waypoint(TURN_S, up, FINGER_CLOSED, "back to the centre again", 0.0),
        Waypoint(1.5, off, FINGER_CLOSED, "push it down (off) again", 0.0),
        Waypoint(0.8, down, FINGER_CLOSED, "ease off the new stem again", 0.0),
        Waypoint(0.6, down, STEM_AJAR, "let go of the new stem", 0.0),
        Waypoint(0.8, _at(fx, fy, stem_tcp + 0.04), STEM_AJAR, "rise from the new stem", 0.0),
    ]


def demo() -> dict[str, list[Waypoint]]:
    fx, fy = FAUCET_XY
    hx, hy = HOLDER_XY
    tx, ty = TRAY_XY
    stem_tcp = STEM_TOP_SEATED - STEM_GRIP_BELOW_TOP
    # The new cartridge is carried by its body, so the grip holds its tabs' bearing (its stem turns freely in it
    # and can't). Holding it so, the TCP: in its holder (settled 0.5 mm into it), with its bottom ENTRY_ABOVE over
    # the rim, and with its bottom RELEASE_DEPTH into the bore.
    grip_tcp = body_tcp(NEW_CART_POS[2] - 0.0005 - CART_LEN / 2)
    entry_tcp = body_tcp(Z_RIM + ENTRY_ABOVE)
    release_tcp = body_tcp(Z_RIM - RELEASE_DEPTH)
    # Let go, it rests on the plungers; the closed fingertips come down to 2 mm over its stem's top (the stem
    # lowered, as it came), push it home by the stem, aiming PRESS past home, and ease off to touching it there.
    rest_stem_tcp = REST_BOTTOM + STEM_ABOVE_BOTTOM + PAD_TIP_PAST_TCP
    drop_tcp = (
        TOP + TRAY_FLOOR + 0.010 + HANG
    )  # the old cartridge's bottom 1 cm above the tray's floor
    right = test_before(stem_tcp)
    right += clip_moves(CLIP_HOME_X, CLIP_HOME_X - CLIP_PULL, "slide the clip out")
    right += [
        Waypoint(1.5, _at(fx, fy, stem_tcp + 0.04), STEM_AJAR, "above the old stem"),
        Waypoint(1.2, _at(fx, fy, stem_tcp), STEM_AJAR, "round the old stem"),
        Waypoint(0.8, _at(fx, fy, stem_tcp), FINGER_CLOSED, "grip the old stem"),
        Waypoint(
            1.0,
            _at(fx, fy, stem_tcp + OLD_STEM_TRAVEL + 0.004),
            FINGER_CLOSED,
            "raise the stem to its stop",
        ),
        Waypoint(3.0, _at(fx, fy, PULL_Z), FINGER_CLOSED, "pull the old cartridge out"),
        Waypoint(2.0, _at(tx, ty, CARRY_Z), FINGER_CLOSED, "carry it to the tray"),
        Waypoint(1.5, _at(tx, ty, drop_tcp), FINGER_CLOSED, "lower it into the tray"),
        Waypoint(0.6, _at(tx, ty, drop_tcp), STEM_AJAR, "let go of the old cartridge"),
        Waypoint(0.8, _at(tx, ty, CARRY_Z), STEM_AJAR, "rise from the tray"),
        Waypoint(1.8, _at(hx, hy, grip_tcp + BODY_ABOVE), BODY_AJAR, "above the new cartridge"),
        Waypoint(1.5, _at(hx, hy, grip_tcp), BODY_AJAR, "round its body"),
        Waypoint(0.8, _at(hx, hy, grip_tcp), FINGER_CLOSED, "grip the new cartridge by its body"),
        Waypoint(1.5, _at(hx, hy, grip_tcp + HOLDER_CLEAR), FINGER_CLOSED, "draw it up out of its holder"),
        Waypoint(1.5, _at(hx, hy, CARRY_Z), FINGER_CLOSED, "lift the new cartridge out"),
        Waypoint(2.0, _at(fx, fy, CARRY_Z), FINGER_CLOSED, "over the valve body"),
        Waypoint(1.5, _at(fx, fy, entry_tcp), FINGER_CLOSED, "lower it over the bore"),
        Waypoint(SETTLE_S, _at(fx, fy, entry_tcp), FINGER_CLOSED, "let it hang still over the bore"),
        Waypoint(2.0, _at(fx, fy, entry_tcp - ENTRY_DEPTH), FINGER_CLOSED, "ease it into the bore"),
        Waypoint(2.0, _at(fx, fy, release_tcp), FINGER_CLOSED, "lower it down the bore"),
        Waypoint(SETTLE_S, _at(fx, fy, release_tcp), FINGER_CLOSED, "let it settle in the bore"),
        Waypoint(0.6, _at(fx, fy, release_tcp), BODY_AJAR, "let go of the new cartridge"),
        Waypoint(1.0, _at(fx, fy, rest_stem_tcp + 0.010), BODY_AJAR, "rise from the new cartridge"),
        Waypoint(0.4, _at(fx, fy, rest_stem_tcp + 0.010), FINGER_CLOSED, "close the fingers over its stem"),
        Waypoint(0.8, _at(fx, fy, rest_stem_tcp + 0.002), FINGER_CLOSED, "down onto its stem"),
        Waypoint(3.0, _at(fx, fy, STEM_PUSH_TCP - PRESS), FINGER_CLOSED, "push the new cartridge home"),
        Waypoint(0.8, _at(fx, fy, STEM_PUSH_TCP - PRESS), FINGER_CLOSED, "hold it home"),
        Waypoint(0.8, _at(fx, fy, STEM_PUSH_TCP), FINGER_CLOSED, "ease off"),
        Waypoint(0.8, _at(fx, fy, stem_tcp + 0.04), FINGER_CLOSED, "rise from its stem"),
        Waypoint(0.6, _at(fx, fy, stem_tcp + 0.04), STEM_AJAR, "open above the new stem"),
        Waypoint(1.2, _at(fx, fy, stem_tcp), STEM_AJAR, "round the new stem"),
        Waypoint(0.8, _at(fx, fy, stem_tcp), FINGER_CLOSED, "grip the new stem"),
        # The notch starts to the left (bearing +90 deg); a quarter turn anticlockwise brings it to the front.
        Waypoint(
            TURN_S, _at(fx, fy, stem_tcp), FINGER_CLOSED, "turn the notch to the front", math.pi / 2
        ),
        Waypoint(0.6, _at(fx, fy, stem_tcp), STEM_AJAR, "let go of the stem", math.pi / 2),
        Waypoint(0.8, _at(fx, fy, stem_tcp + 0.04), STEM_AJAR, "rise from the stem", math.pi / 2),
    ]
    right += clip_push(CLIP_HOME_X - CLIP_PULL, CLIP_HOME_X - CLIP_SHORT, "push the clip home")
    right += test_after(stem_tcp)
    right.append(Waypoint(1.5, PARK["right"], FINGER_OPEN, "park"))
    end = sum(w.duration for w in right) + END_HOLD
    left = [Waypoint(2.0, PARK["left"], FINGER_OPEN, "park")]
    return {"left": _pad(left, end, "hold still"), "right": _pad(right, end, "hold still")}


def _simulator_present() -> bool:
    """Whether Isaac Lab can be imported here: the scene hook is only installed where the scene can be built."""
    try:
        return importlib.util.find_spec("isaaclab") is not None
    except (ImportError, ValueError):
        return False


if _simulator_present():
    try:
        install_faucet()
    except (
        Exception
    ) as error:  # never break loading the task over the hook; the faucet then has no valve body
        _warn_once("faucet", f"could not install the scene hook ({type(error).__name__}: {error})")
