"""On the table in front of the arms stands a black locking post-mount mailbox (an Oasis Classic-type 12.0 in wide, 15.2
in tall, 18.1 in deep galvanized-steel box), its front toward the robot, bolted through a 4x4 post adapter plate onto
the top of a short 4x4 post on a plywood base. Its front has two doors, both hinged along their bottom edge. The upper
one is the carrier's hopper door: a bar handle along its top and a spring outgoing-mail clip on its inside face; a
magnet holds it shut. The lower one, near the bottom of the box, is the resident's access door (26.5 x 10.5 cm). A
cam lock sits near its right end: a chrome bezel with a brass plug and a horizontal keyway. The key turns the plug a
quarter turn: counter-clockwise (seen from the front) unlocks, swinging the steel cam behind the plug up out of the
way; clockwise locks, swinging the cam out to the right behind the frame, where it stops the door from opening, and
where, with the door open, it meets the frame before the door can shut. The key comes out only in the locked
position. A small magnet holds the access door shut; let go open, it falls open to its stop. A red outgoing-mail flag
is bolted to the right side near the front top, lying down along the side.

Inside, on the floor of the box just behind the access door, lies the delivered parcel: a small white corrugated
mailer box (12.1 x 8.6 x 6.0 cm) on one of its long sides, its length front to back. In front of the mailbox a
rectangle of blue painter's tape on the table marks where the parcel goes. The mailbox's key (nickel, 4.2 cm long)
lies flat on a black silicone mat on the table to the right of the mailbox, its blade toward the mailbox.
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
from dataclasses import dataclass, field
from types import FrameType
from typing import Any, Protocol

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
SCORE_OVER = "end"
TOP: float = TABLE_TOP[SCENE]
INCH = 0.0254
G = 9.81
Vec3 = tuple[float, float, float]
Pt = Sequence[float] | np.ndarray

# --- the mailbox (world frame: x away from the arms, y to their left, z up; metres) ------------------------------
# Outer size from the maker's page (15.2 in tall, 12.0 in wide, 18.1 in deep). Every inner dimension is an estimate
# from the maker's drawings (README, "Sources"): none is published.
MB_W = 12.0 * INCH  # 0.305 (y)
MB_H = 15.2 * INCH  # 0.386 (z)
MB_D = 18.1 * INCH  # 0.460 (x)
X_FRONT = 0.320  # the front panel's outer face
Y_MB = 0.0
SKIN = 0.002  # 14-16 gauge steel, as drawn
WALL = 0.004  # collider thickness of the box's walls (inward from the outer face)
BASE_PLY = 0.019  # 3/4 in plywood base, clamped to the table
POST_STUB = 0.035  # the top of a 4x4 post (89 mm square), cut 35 mm long
ADAPTER = 0.006  # the 4x4 adapter plate
Z_BOT = TOP + BASE_PLY + POST_STUB + ADAPTER  # the box's bottom: 60 mm over the table
FLOOR_ZB = 0.020  # the inside floor, above the box's bottom (the decorative base pan below it)
# Front panel layout, heights above the box's bottom (estimates from the maker's drawings).
AD_OPEN = (FLOOR_ZB, 0.115)  # access opening: floor to 115 mm
AD_ZB = (FLOOR_ZB, 0.125)  # the access door: 105 mm tall, its top 10 mm over the opening's top
AD_W = 0.265  # door width (overlaps the 245 mm opening 10 mm a side)
OPEN_HALF_W = 0.1225  # both openings: 245 mm wide
AD_T = 0.006  # the door's depth: a 1.6 mm panel with folded edges
AD_MID = 0.120  # the door body's own box: the panel's middle; the rest is built round the lock's hole
LOCK_HOLE = 0.0066  # half the square hole the lock's cylinder passes through (hidden under the bezel)
HD_COM = (0.020, 0.0, -0.010)  # the hopper door's centre of mass, its frame: inside the hinge line (its wings)
HD_OPEN = (0.140, 0.335)  # hopper opening
HD_ZB = (0.135, 0.345)  # the hopper door: 210 mm tall
HD_T = 0.006
HINGE_R = 0.004
AD_OPEN_LIMIT = 95.0  # degrees: the access door's stop (estimate)
HD_OPEN_LIMIT = 70.0  # degrees: the hopper's wings stop it (estimate)
CATCH_DEG = 3.0  # within this of shut, the door's magnet pulls it shut and holds it
CATCH_TORQUE = (0.15, 0.30)  # N m: the access door's and the hopper door's magnets (estimates)
CATCH_K = 0.05  # N m per degree, up to the catch torque
# The cam lock: a 5/8 in cylinder through the door, a 19 mm chrome bezel, a 13 mm plug, a horizontal keyway, a flat
# steel cam behind; a quarter turn (CompX National's catalogue for USPS-approved mailbox locks: "90 deg cam turn", "Key
# removal in locked position only").
LOCK_Y = -0.1005  # the lock's axis: 22 mm in from the opening's right edge
LOCK_ZB = FLOOR_ZB + (AD_ZB[1] - AD_ZB[0]) / 2  # the door's mid-height
BEZEL_R, BEZEL_T = 0.0095, 0.003
PLUG_R = 0.0065
PLUG_LEN = 0.0159  # 5/8 in
X_FACE = X_FRONT - AD_T - BEZEL_T  # the plug's face (the keyway's mouth)
CAM_T, CAM_W, CAM_L = 0.0025, 0.010, 0.039  # the cam: 2.5 mm steel, 10 wide, 39 long (34 past the axis)
CAM_BACK = 0.005  # it reaches this far behind the axis
X_CAM = X_FACE + PLUG_LEN + CAM_T / 2  # the cam's mid-plane, behind the front panel
KEYWAY_T, KEYWAY_W = 0.0026, 0.0085  # the keyway slot (estimate; the blade is 2.0 x 7.5)
UNLOCKED_DEG = -90.0  # plug angle (about +x): counter-clockwise seen from the front unlocks
FULL_IN = 0.0010  # the key is fully in when its shoulder is within this of the plug's face ...
ON_AXIS = 0.0010  # ... on the lock's axis within this ...
ALIGNED_DEG = 4.0  # ... and square to the face within this
AT_LOCKED_DEG = 3.0  # the key comes out only within this of locked
PLUG_HOLD_K = 1.0  # N m per degree: the wafers hold the plug while the key is not fully in (stiff)
PLUG_HOLD_TORQUE = 5.0  # N m
PLUG_FREE_DAMPING = 0.0005  # N m s per degree: the plug's drag while the key turns it
SNAP_DEG = 6.0  # within this of either end, a light snap draws it to the end (the wafers dropping in)
SNAP_K, SNAP_TORQUE = 0.004, 0.03
RETAIN_K, RETAIN_FORCE = 3000.0, 40.0  # N/m, N: the wafers keep the key in while the plug is turned
WAFER_K, WAFER_FORCE = 300.0, 0.3  # N/m, N: their light grip on a key home at locked (estimate, to measure)
# The flag: a 12 x 3 mm red steel flat bar, its head an outline, pivoting on its bracket at the right side's front top.
FLAG_PIVOT = (X_FRONT + 0.040, Y_MB - MB_W / 2 - 0.011, 0.300)  # x, y, height above the box's bottom
FLAG_LEN, FLAG_BAR, FLAG_T = 0.200, 0.012, 0.003
FLAG_HEAD = (0.075, 0.055)  # the head: along the arm, out from it
FLAG_MASS = 0.10  # kg (estimate: 12 x 3 mm steel bar, about 0.45 m in all)
FLAG_K, FLAG_TORQUE = 0.04, 0.40  # N m per degree, N m: its sprung catch (STD-7C: under 2 lbf to move it)
FLAG_SWITCH = 45.0  # degrees: past this the catch carries it on
FLAG_UP = 80.0  # degrees: "up" for scoring
# The outgoing-mail clip on the hopper door's inside face (spring steel, 80 x 30 mm, its free edge toward the hinge).
CLIP_SIZE = (0.0012, 0.080, 0.030)
CLIP_TOP_ZB = 0.300  # its hinged edge, above the box's bottom (door shut)
CLIP_OPEN = 25.0
CLIP_K, CLIP_TORQUE, CLIP_PRELOAD = 0.015, 0.10, 2.0  # N m/deg, N m, degrees past shut

# --- the loose things -------------------------------------------------------------------------------------------------
KEY_BOW = (0.026, 0.024, 0.002)  # nickel-silver key (estimate): bow 26 x 24 x 2 mm with a 5 mm hole
KEY_HOLE_R, KEY_HOLE_X = 0.0025, -0.006
BLADE = (0.0155, 0.0075, 0.0020)  # the blade: 15.5 long (to the plug's back), 7.5 wide, 2.0 thick
KEY_MASS = 0.006
KEY_MAT = (0.100, 0.100, 0.003)  # black silicone mat
KEY_AT = (0.215, -0.255)  # the bow's centre on the mat
# The parcel: a white corrugated mailer box (ULINE S-425, 4 x 3 x 2 in inside; outside 4-3/4 x 3-3/8 x 2-3/8 in, the
# maker's listing), lying on one of its long sides: its length along x, 2-3/8 in across, 3-3/8 in tall.
PARCEL_SIZE = (4.75 * INCH, 2.375 * INCH, 3.375 * INCH)
PARCEL_MASS = 0.24  # kg: the box (0.09 lb, the listing) and a 200 g calibration weight taped inside
PARCEL_Y = 0.050  # it lies left of the box's middle, 12 mm behind the access opening
PARCEL_PLACE = (0.235, 0.050)  # the centre of its marked place on the table, in front of the mailbox
PLACE_SIZE = (0.160, 0.100)  # the tape outline's inside: 160 x 100 mm

# --- colours (sRGB) and friction (static, dynamic) ------------------------------------------------------------------
BLACK = (0.060, 0.062, 0.068)  # the doors: black satin powder coat (lit, it reads dark grey with highlights)
FRAME = (0.125, 0.128, 0.136)  # the box: the same black powder coat, lit
EDGE_LINE = (0.36, 0.37, 0.39)  # the doors' folded edges catch the light (a thin line round each door)
DARK = (0.02, 0.02, 0.022)
RED = (0.80, 0.06, 0.05)
CHROME = (0.82, 0.83, 0.85)
BRASS = (0.80, 0.66, 0.36)
NICKEL = (0.88, 0.88, 0.85)
POST_WOOD = (0.60, 0.47, 0.32)
PLY = (0.83, 0.71, 0.52)
CARD = (0.93, 0.93, 0.91)
LABEL = (0.99, 0.99, 0.97)
INK = (0.25, 0.25, 0.27)
SILICONE = (0.11, 0.11, 0.12)
STEEL_F = (0.40, 0.35)
CARD_F = (0.50, 0.45)
RUBBER_F = (1.00, 0.90)
KEY_F = (0.35, 0.30)

MB_CENTER = (X_FRONT + MB_D / 2, Y_MB, Z_BOT + FLOOR_ZB - WALL / 2)  # the mailbox body: its floor slab's centre
AD_CENTER = (X_FRONT - AD_T / 2, Y_MB, Z_BOT + (AD_ZB[0] + AD_ZB[1]) / 2)
HD_CENTER = (X_FRONT - HD_T / 2, Y_MB, Z_BOT + (HD_ZB[0] + HD_ZB[1]) / 2)
PLUG_CENTER = (X_CAM, LOCK_Y - (CAM_L / 2 - CAM_BACK), Z_BOT + LOCK_ZB)  # the plug's body: its cam's centre
KEY_CENTER = (KEY_AT[0], KEY_AT[1], TOP + KEY_MAT[2] + KEY_BOW[2] / 2)
CLIP_CENTER = (X_FRONT + 0.0010 + CLIP_SIZE[0] / 2, Y_MB, Z_BOT + CLIP_TOP_ZB - CLIP_SIZE[2] / 2)
FLAG_CENTER = (FLAG_PIVOT[0] + FLAG_LEN / 2, FLAG_PIVOT[1], Z_BOT + FLAG_PIVOT[2])
PARCEL_CENTER = (X_FRONT + WALL + 0.012 + PARCEL_SIZE[0] / 2, PARCEL_Y, Z_BOT + FLOOR_ZB + PARCEL_SIZE[2] / 2)
MAT_CENTER = (KEY_AT[0] + 0.012, KEY_AT[1], TOP + KEY_MAT[2] / 2)

MAILBOX, KEY_MAT_N = "mailbox", "key_mat"
ACCESS_DOOR, PLUG, KEY = "access_door", "lock_plug", "key"
HOPPER, CLIP, FLAG = "hopper_door", "mail_clip", "flag"
PARCEL = "parcel"


def _t(values: Any) -> Vec3:
    return (float(values[0]), float(values[1]), float(values[2]))


def _v(values: Any) -> np.ndarray:
    return np.asarray(values, dtype=float)


def _box(name: str, center: Pt, size: Pt, rgb: Pt, mass: float | None = None) -> Box:
    return Box(name, pos=_t(center), size=_t(size), color=_t(rgb), mass=mass)


OBJECTS = [
    _box(MAILBOX, MB_CENTER, (MB_D - WALL, MB_W - 2 * WALL, WALL), BLACK),
    _box(KEY_MAT_N, MAT_CENTER, KEY_MAT, SILICONE),
    _box(ACCESS_DOOR, AD_CENTER, (AD_T, AD_MID, AD_ZB[1] - AD_ZB[0]), BLACK, 0.40),
    _box(PLUG, PLUG_CENTER, (CAM_T, CAM_L, CAM_W), CHROME, 0.020),
    _box(KEY, KEY_CENTER, KEY_BOW, NICKEL, KEY_MASS),
    _box(HOPPER, HD_CENTER, (HD_T, AD_W, HD_ZB[1] - HD_ZB[0]), BLACK, 1.00),
    _box(CLIP, CLIP_CENTER, CLIP_SIZE, CHROME, 0.010),
    _box(FLAG, FLAG_CENTER, (FLAG_LEN, FLAG_T, FLAG_BAR), RED, FLAG_MASS),
    _box(PARCEL, PARCEL_CENTER, PARCEL_SIZE, CARD, PARCEL_MASS),
]
SPECS = {o.name: o for o in OBJECTS}
FIXED = (MAILBOX, KEY_MAT_N)
FREE = (ACCESS_DOOR, PLUG, KEY, HOPPER, CLIP, FLAG, PARCEL)


# --- geometry as data: every shape of every body, in the body's own frame -------------------------------------------
@dataclass(frozen=True, eq=False)
class Part:
    """One shape under a body: a box (``size``), a cylinder (``radius``, ``height`` along ``axis``) or a triangle mesh
    (``points``, ``tris``); its look (``look``: a key of LOOKS) and whether it collides (``fit``: a close-fitting
    collider with small contact offsets)."""

    name: str
    kind: str
    center: Vec3
    look: str
    collides: bool = False
    size: Vec3 = (0.0, 0.0, 0.0)
    radius: float = 0.0
    height: float = 0.0
    axis: str = "Z"
    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    tris: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), dtype=np.int64))
    fit: bool = False

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        c = _v(self.center)
        if self.kind == "box":
            half = _v(self.size) / 2
        elif self.kind == "cyl":
            r, h = self.radius, self.height / 2
            half = _v({"X": (h, r, r), "Y": (r, h, r), "Z": (r, r, h)}[self.axis])
        else:
            return c + self.points.min(axis=0), c + self.points.max(axis=0)
        return c - half, c + half


# Looks: (rgb, metallic, roughness). Physics materials: friction per body (the colliders a body has share one).
LOOKS: dict[str, tuple[Vec3, float, float]] = {
    "powder": (BLACK, 0.25, 0.42),
    "frame": (FRAME, 0.25, 0.45),
    "edge": (EDGE_LINE, 0.6, 0.30),
    "dark": (DARK, 0.0, 0.9),
    "red": (RED, 0.1, 0.4),
    "chrome": (CHROME, 1.0, 0.15),
    "brass": (BRASS, 1.0, 0.3),
    "nickel": (NICKEL, 1.0, 0.16),
    "post": (POST_WOOD, 0.0, 0.8),
    "ply": (PLY, 0.0, 0.75),
    "card": (CARD, 0.0, 0.85),
    "label": (LABEL, 0.0, 0.7),
    "ink": (INK, 0.0, 0.8),
    "silicone": (SILICONE, 0.0, 0.9),
    "galv": ((0.62, 0.64, 0.66), 0.9, 0.4),
    "tape": ((0.20, 0.45, 0.80), 0.0, 0.8),
}
FRICTION = {
    MAILBOX: STEEL_F,
    KEY_MAT_N: RUBBER_F,
    ACCESS_DOOR: STEEL_F,
    PLUG: KEY_F,
    KEY: KEY_F,
    HOPPER: STEEL_F,
    CLIP: STEEL_F,
    FLAG: STEEL_F,
    PARCEL: CARD_F,
}


def _bx(name: str, lo: Pt, hi: Pt, look: str, collides: bool = False, fit: bool = False) -> Part:
    """A box from its low and high corners."""
    lo_v, hi_v = _v(lo), _v(hi)
    return Part(name, "box", _t((lo_v + hi_v) / 2), look, collides, size=_t(hi_v - lo_v), fit=fit)


def _cy(name: str, center: Pt, radius: float, height: float, axis: str, look: str, collides: bool = False) -> Part:
    return Part(name, "cyl", _t(center), look, collides, radius=radius, height=height, axis=axis)


def _mesh(name: str, points: np.ndarray, tris: np.ndarray, look: str, collides: bool = False) -> Part:
    return Part(name, "mesh", (0.0, 0.0, 0.0), look, collides, points=points, tris=tris)


def prism(outline: Sequence[tuple[float, float]], lo: float, hi: float, axis: str) -> tuple[np.ndarray, np.ndarray]:
    """A closed triangle mesh: the convex polygon ``outline`` (counter-clockwise in the plane of the other two axes, in
    order) extruded from ``lo`` to ``hi`` along ``axis``."""
    n = len(outline)
    a, b = {"X": (1, 2), "Y": (2, 0), "Z": (0, 1)}[axis]
    k = {"X": 0, "Y": 1, "Z": 2}[axis]
    points = np.zeros((2 * n, 3))
    for i, (u, w) in enumerate(outline):
        for j, h in enumerate((lo, hi)):
            p = points[2 * i + j]
            p[a], p[b], p[k] = u, w, h
    tris: list[list[int]] = []
    for i in range(1, n - 1):
        tris.append([0, 2 * (i + 1), 2 * i])  # the low cap, facing -axis
        tris.append([1, 2 * i + 1, 2 * (i + 1) + 1])  # the high cap
    for i in range(n):
        j = (i + 1) % n
        tris.append([2 * i, 2 * j, 2 * j + 1])
        tris.append([2 * i, 2 * j + 1, 2 * i + 1])
    return points, np.asarray(tris, dtype=np.int64)


def rounded_rect(w: float, h: float, r: float, n: int = 6) -> list[tuple[float, float]]:
    """A w x h rectangle centred on the origin with corners rounded to r, counter-clockwise."""
    out: list[tuple[float, float]] = []
    for cx, cy, start in ((w / 2 - r, -h / 2 + r, -90.0), (w / 2 - r, h / 2 - r, 0.0), (-w / 2 + r, h / 2 - r, 90.0), (-w / 2 + r, -h / 2 + r, 180.0)):
        for i in range(n + 1):
            t = math.radians(start + 90.0 * i / n)
            out.append((cx + r * math.cos(t), cy + r * math.sin(t)))
    return out


def ring(r_in: float, outer: Callable[[float], float], z0: float, z1: float, n: int = 48) -> tuple[np.ndarray, np.ndarray]:
    """A closed mesh between a circle of r_in and a star-shaped outer edge r = outer(angle), from z0 to z1 (about z)."""
    pts: list[tuple[float, float, float]] = []
    for i in range(n):
        t = 2 * math.pi * i / n
        c, s = math.cos(t), math.sin(t)
        ro = outer(t)
        for z in (z0, z1):
            pts.append((r_in * c, r_in * s, z))
            pts.append((ro * c, ro * s, z))
    points = np.asarray(pts)

    def idx(i: int, z: int, side: int) -> int:
        return (i % n) * 4 + z * 2 + side

    tris: list[list[int]] = []
    for i in range(n):
        j = i + 1
        tris += [[idx(i, 0, 1), idx(j, 0, 1), idx(j, 1, 1)], [idx(i, 0, 1), idx(j, 1, 1), idx(i, 1, 1)]]  # outer
        tris += [[idx(i, 0, 0), idx(j, 1, 0), idx(j, 0, 0)], [idx(i, 0, 0), idx(i, 1, 0), idx(j, 1, 0)]]  # inner
        tris += [[idx(i, 1, 0), idx(i, 1, 1), idx(j, 1, 1)], [idx(i, 1, 0), idx(j, 1, 1), idx(j, 1, 0)]]  # top
        tris += [[idx(i, 0, 0), idx(j, 0, 1), idx(i, 0, 1)], [idx(i, 0, 0), idx(j, 0, 0), idx(j, 0, 1)]]  # bottom
    return points, np.asarray(tris, dtype=np.int64)


def strip(xs: Sequence[float], lo: Callable[[float], float], hi: Callable[[float], float], z0: float, z1: float) -> tuple[np.ndarray, np.ndarray]:
    """A closed mesh of the plate {x in xs' span, lo(x) <= y <= hi(x), z0 <= z <= z1} (a key blade's bitted profile)."""
    pts: list[tuple[float, float, float]] = []
    for x in xs:
        for y in (lo(x), hi(x)):
            for z in (z0, z1):
                pts.append((x, y, z))
    points = np.asarray(pts)
    m = len(xs)

    def idx(i: int, side: int, z: int) -> int:
        return i * 4 + side * 2 + z

    tris: list[list[int]] = []
    for i in range(m - 1):
        j = i + 1
        tris += [[idx(i, 0, 0), idx(j, 0, 0), idx(j, 0, 1)], [idx(i, 0, 0), idx(j, 0, 1), idx(i, 0, 1)]]
        tris += [[idx(i, 1, 0), idx(j, 1, 1), idx(j, 1, 0)], [idx(i, 1, 0), idx(i, 1, 1), idx(j, 1, 1)]]
        tris += [[idx(i, 0, 1), idx(j, 0, 1), idx(j, 1, 1)], [idx(i, 0, 1), idx(j, 1, 1), idx(i, 1, 1)]]
        tris += [[idx(i, 0, 0), idx(j, 1, 0), idx(j, 0, 0)], [idx(i, 0, 0), idx(i, 1, 0), idx(j, 1, 0)]]
    last = m - 1
    tris += [[idx(0, 0, 0), idx(0, 0, 1), idx(0, 1, 1)], [idx(0, 0, 0), idx(0, 1, 1), idx(0, 1, 0)]]
    tris += [[idx(last, 0, 0), idx(last, 1, 1), idx(last, 0, 1)], [idx(last, 0, 0), idx(last, 1, 0), idx(last, 1, 1)]]
    return points, np.asarray(tris, dtype=np.int64)


def _local(center: Pt, body: Pt) -> np.ndarray:
    return np.asarray(_v(center) - _v(body), dtype=float)


def mailbox_parts() -> list[Part]:
    """The box (its frame: the floor slab's centre). Colliders where the robot or the parts can touch it: walls, top,
    front frame, the base; looks: the bevelled top, the base pan, the hinge knuckles, the bolt heads, the flag bracket."""
    c = _v(MB_CENTER)
    x0, x1 = X_FRONT, X_FRONT + MB_D
    yl, yr = Y_MB + MB_W / 2, Y_MB - MB_W / 2
    zb, zt = Z_BOT, Z_BOT + MB_H

    def w(lo: Pt, hi: Pt) -> tuple[np.ndarray, np.ndarray]:
        return _local(lo, c), _local(hi, c)

    parts: list[Part] = []

    def slab(name: str, lo: Pt, hi: Pt, look: str = "frame", collides: bool = True) -> None:
        a, b = w(lo, hi)
        parts.append(_bx(name, a, b, look, collides))

    bevel = 0.030  # the top's bevelled front edge
    slab("wall_left", (x0, yl - WALL, zb), (x1, yl, zt - bevel))
    slab("wall_right", (x0, yr, zb), (x1, yr + WALL, zt - bevel))
    slab("back", (x1 - WALL, yr, zb), (x1, yl, zt))
    slab("roof", (x0 + bevel, yr, zt - WALL), (x1, yl, zt))
    slab("base_pan", (x0, yr, zb), (x1, yl, Z_BOT + FLOOR_ZB - WALL))
    # The bevel: a sloped plate across the front top (looks: a prism; collider: its box under it).
    a = _local((x0, 0.0, zt - bevel), c)
    b = _local((x0 + bevel, 0.0, zt), c)
    outline = [(a[2], a[0]), (b[2], b[0]), (b[2] - 0.004, b[0]), (a[2], a[0] + 0.004)]  # (z, x)
    pts, tris = prism(outline, yr - c[1], yl - c[1], "Y")
    parts.append(_mesh("bevel", pts, tris, "powder"))
    slab("bevel_block", (x0, yr, zt - bevel), (x0 + WALL, yl, zt - bevel + 0.004), "powder", True)
    # Front panel round the two openings (collides): the strip under the access opening, the jambs, the rail, the top.
    fz = (Z_BOT + AD_OPEN[0], Z_BOT + AD_OPEN[1], Z_BOT + HD_OPEN[0], Z_BOT + HD_OPEN[1])
    slab("front_low", (x0, yr, zb), (x0 + WALL, yl, fz[0]))
    for side, (ya, yb) in (("right", (yr, -OPEN_HALF_W)), ("left", (OPEN_HALF_W, yl))):
        slab(f"jamb_low_{side}", (x0, ya, fz[0]), (x0 + WALL, yb, fz[1]))
        slab(f"jamb_high_{side}", (x0, ya, fz[2]), (x0 + WALL, yb, fz[3]))
    slab("rail", (x0, yr, fz[1]), (x0 + WALL, yl, fz[2]))
    slab("front_top", (x0, yr, fz[3]), (x0 + WALL, yl, zt - bevel))
    # The gap round each door (looks): a dark line on the frame just outside the door's outline.
    for name, (z0, z1) in (("access", AD_ZB), ("hopper", HD_ZB)):
        g, xf = 0.0025, x0 - 0.0002
        za, zb_ = Z_BOT + z0 - g, Z_BOT + z1 + g
        slab(f"gap_{name}_top", (xf, -AD_W / 2 - g, zb_ - g), (x0, AD_W / 2 + g, zb_), "dark", False)
        slab(f"gap_{name}_bottom", (xf, -AD_W / 2 - g, za), (x0, AD_W / 2 + g, za + g), "dark", False)
        slab(f"gap_{name}_left", (xf, AD_W / 2, za), (x0, AD_W / 2 + g, zb_), "dark", False)
        slab(f"gap_{name}_right", (xf, -AD_W / 2 - g, za), (x0, -AD_W / 2, zb_), "dark", False)
    # The galvanized inside (looks): a liner on the side walls, the back and under the roof.
    zin0, zin1 = Z_BOT + FLOOR_ZB, zt - WALL
    slab("liner_left", (x0 + WALL, yl - WALL - 0.0005, zin0), (x1 - WALL, yl - WALL, zin1), "galv", False)
    slab("liner_right", (x0 + WALL, yr + WALL, zin0), (x1 - WALL, yr + WALL + 0.0005, zin1), "galv", False)
    slab("liner_back", (x1 - WALL - 0.0005, yr + WALL, zin0), (x1 - WALL, yl - WALL, zin1), "galv", False)
    slab("liner_roof", (x0 + WALL, yr + WALL, zin1 - 0.0005), (x1 - WALL, yl - WALL, zin1), "galv", False)
    # The decorative base: a flared skirt round the bottom (looks).
    skirt = 0.006
    slab("skirt", (x0 - skirt, yr - skirt, zb), (x1 + skirt, yl + skirt, zb + 0.012), "powder", False)
    # Hinge knuckles: each door's hinge along its bottom edge (looks).
    for name, zh, door_t in (("hinge_access", Z_BOT + AD_ZB[0], AD_T), ("hinge_hopper", Z_BOT + HD_ZB[0], HD_T)):
        for k, ym in enumerate((-0.095, 0.0, 0.095)):
            parts.append(_cy(f"{name}_{k}", _local((x0 - door_t / 2, ym, zh), c), HINGE_R, 0.030, "Y", "powder"))
    # The flag's bracket: a black plate on the right side, two bolts through it (looks).
    px, py, pz = FLAG_PIVOT
    bracket_lo = (px - 0.022, yr - 0.006, Z_BOT + pz - 0.020)
    bracket_hi = (px + 0.022, yr, Z_BOT + pz + 0.012)
    slab("flag_bracket", bracket_lo, bracket_hi, "powder", False)
    for k, dx in enumerate((-0.012, 0.012)):
        parts.append(_cy(f"flag_bolt_{k}", _local((px + dx, yr - 0.008, Z_BOT + pz - 0.006), c), 0.004, 0.004, "Y", "galv"))
    # Rivets and a drip lip on the roof (looks).
    for k, (dx, dy) in enumerate(((0.05, 0.11), (0.05, -0.11), (MB_D - 0.05, 0.11), (MB_D - 0.05, -0.11))):
        parts.append(_cy(f"roof_rivet_{k}", _local((x0 + dx, dy, zt + 0.0008), c), 0.004, 0.0016, "Z", "powder"))
    # The mount: the 4x4 adapter plate, the post's top and the plywood base (collide: they hold the box up).
    mid_x = x0 + MB_D / 2
    slab("adapter_plate", (mid_x - 0.127, -0.051, zb - ADAPTER), (mid_x + 0.127, 0.051, zb), "powder")
    slab("post_top", (mid_x - 0.0445, -0.0445, TOP + BASE_PLY), (mid_x + 0.0445, 0.0445, zb - ADAPTER), "post")
    slab("ply_base", (mid_x - 0.150, -0.130, TOP), (mid_x + 0.150, 0.130, TOP + BASE_PLY), "ply")
    # The parcel's marked place: an outline of 24 mm painter's tape on the table, PLACE_SIZE inside (looks).
    (px, py), (lx, ly), tw = PARCEL_PLACE, PLACE_SIZE, 0.024
    x0p, x1p, y0p, y1p = px - lx / 2, px + lx / 2, py - ly / 2, py + ly / 2
    for name, lo, hi in (
        ("place_near", (x0p - tw, y0p - tw), (x0p, y1p + tw)),
        ("place_far", (x1p, y0p - tw), (x1p + tw, y1p + tw)),
        ("place_right", (x0p, y0p - tw), (x1p, y0p)),
        ("place_left", (x0p, y1p), (x1p, y1p + tw)),
    ):
        slab(name, (lo[0], lo[1], TOP), (hi[0], hi[1], TOP + 0.0003), "tape", False)
    return parts


def access_door_parts() -> list[Part]:
    """The access door (its frame: the panel's centre, door shut): its folded-edge panel is the body's own box; the
    lock's bezel, the plug's face and the keyway are looks on its face; a dark gasket line round it."""
    c = _v(AD_CENTER)
    lock = _v((X_FRONT - AD_T, LOCK_Y, Z_BOT + LOCK_ZB))
    parts = [
        _cy("bezel", _local(lock - (BEZEL_T / 2, 0, 0), c), BEZEL_R, BEZEL_T, "X", "chrome"),
        _cy("bezel_inner", _local(lock - (BEZEL_T + 0.0002, 0, 0), c), PLUG_R + 0.0008, 0.0004, "X", "dark"),
    ]
    h = AD_ZB[1] - AD_ZB[0]
    t2, ly, lz, s = AD_T / 2, LOCK_Y - c[1], Z_BOT + LOCK_ZB - c[2], LOCK_HOLE
    m = AD_MID / 2
    parts += [  # the rest of the panel (colliders), round the lock's hole
        _bx("panel_left", (-t2, m, -h / 2), (t2, AD_W / 2, h / 2), "powder", True),
        _bx("panel_right_edge", (-t2, -AD_W / 2, -h / 2), (t2, ly - s, h / 2), "powder", True),
        _bx("panel_right_in", (-t2, ly + s, -h / 2), (t2, -m, h / 2), "powder", True),
        _bx("panel_over_lock", (-t2, ly - s, lz + s), (t2, ly + s, h / 2), "powder", True),
        _bx("panel_under_lock", (-t2, ly - s, -h / 2), (t2, ly + s, lz - s), "powder", True),
        _bx("edge_top", (-t2 - 0.0004, -AD_W / 2, h / 2 - 0.004), (-t2, AD_W / 2, h / 2), "powder"),
    ]
    return parts + door_edges(AD_W, h, AD_T)


def door_edges(w: float, h: float, t: float) -> list[Part]:
    """A door's folded edges as a thin light line round its face (looks), so the door reads against the box."""
    f, d = -t / 2 - 0.0003, 0.0015
    return [
        _bx("edge_line_top", (f, -w / 2, h / 2 - d), (f + 0.0003, w / 2, h / 2), "edge"),
        _bx("edge_line_bottom", (f, -w / 2, -h / 2), (f + 0.0003, w / 2, -h / 2 + d), "edge"),
        _bx("edge_line_left", (f, w / 2 - d, -h / 2), (f + 0.0003, w / 2, h / 2), "edge"),
        _bx("edge_line_right", (f, -w / 2, -h / 2), (f + 0.0003, -w / 2 + d, h / 2), "edge"),
    ]


def plug_parts() -> list[Part]:
    """The lock's plug and cam (its frame: the cam's centre, locked): the cam is the body's own box; the plug's
    cylinder is four colliders round the keyway slot (they guide the blade and are what it turns), its face a look."""
    c = _v(PLUG_CENTER)
    axis_y, axis_z = LOCK_Y - c[1], Z_BOT + LOCK_ZB - c[2]
    x_lo, x_hi = X_FACE - c[0], X_FACE + PLUG_LEN - c[0]
    t2, w2, r = KEYWAY_T / 2, KEYWAY_W / 2, PLUG_R
    parts = [
        _bx("plug_above", (x_lo, axis_y - r, axis_z + t2), (x_hi, axis_y + r, axis_z + r), "brass", True, True),
        _bx("plug_below", (x_lo, axis_y - r, axis_z - r), (x_hi, axis_y + r, axis_z - t2), "brass", True, True),
        _bx("plug_side_l", (x_lo, axis_y + w2, axis_z - t2), (x_hi, axis_y + r, axis_z + t2), "brass", True, True),
        _bx("plug_side_r", (x_lo, axis_y - r, axis_z - t2), (x_hi, axis_y - w2, axis_z + t2), "brass", True, True),
        _cy("plug_face", (x_lo - 0.0002, axis_y, axis_z), PLUG_R, 0.0004, "X", "brass"),
        _bx("keyway", (x_lo - 0.0005, axis_y - w2, axis_z - t2), (x_lo + 0.0001, axis_y + w2, axis_z + t2), "dark"),
        _cy("cam_boss", (-c[0] + X_CAM, axis_y, axis_z), 0.0055, CAM_T + 0.001, "X", "chrome"),
    ]
    return parts


def key_parts() -> list[Part]:
    """The key (its frame: the bow's centre; the blade along +x): the bow is the body's own box (a collider); its
    rounded outline with the ring hole, the shoulder and the bitted blade are looks; the blade is also a close-fitting
    collider box (what enters the keyway)."""
    bl, bw, bt = KEY_BOW
    lx, ly, lt = BLADE

    def outer(t: float) -> float:  # the bow's outline seen from its hole: a rounded square (superellipse, power 4)
        cx, cy = math.cos(t), math.sin(t)
        a = (bl / 2 - KEY_HOLE_X) if cx > 0 else (bl / 2 + KEY_HOLE_X)  # the hole to the bow's front or back
        b = bw / 2
        return float((abs(cx / a) ** 4 + abs(cy / b) ** 4) ** -0.25)

    pts, tris = ring(KEY_HOLE_R, outer, -bt / 2, bt / 2)
    pts = pts + _v((KEY_HOLE_X, 0.0, 0.0))
    x0 = bl / 2

    def hi(x: float) -> float:  # the bitting: five cuts along the top edge
        u = (x - x0 - 0.002) / (lx - 0.003)
        if u <= 0.0 or u >= 1.0:
            return ly / 2
        cut = (0.0016, 0.0009, 0.0022, 0.0012, 0.0019)[min(4, int(u * 5))]
        frac = u * 5 - int(u * 5)
        return ly / 2 - cut * (1.0 - abs(2.0 * frac - 1.0))

    def lo(x: float) -> float:  # the bottom edge, tapered to the tip
        return -ly / 2 + max(0.0, (x - (x0 + lx - 0.003)) / 0.003) * 0.0025

    xs = [x0 + lx * i / 40 for i in range(41)]
    bpts, btris = strip(xs, lo, hi, -lt / 2, lt / 2)
    return [
        _mesh("bow", pts, tris, "nickel"),
        _mesh("blade", bpts, btris, "nickel"),
        _bx("shoulder", (x0 - 0.002, -0.0055, -bt / 2), (x0 + 0.0005, 0.0055, bt / 2), "nickel"),
        _bx("blade_collider", (x0, -ly / 2, -lt / 2), (x0 + lx, ly / 2, lt / 2), "nickel", True, True),
    ]


def hopper_parts() -> list[Part]:
    """The hopper door (its frame: the panel's centre, door shut): its panel is the body's own box; the bar handle
    (collides), the two side wings inside (looks), and the clip's backing plate (looks)."""
    h = HD_ZB[1] - HD_ZB[0]
    parts = [
        _cy("handle_bar", (-HD_T / 2 - 0.022, 0.0, h / 2 - 0.018), 0.005, 0.150, "Y", "chrome", True),
        _bx("handle_post_l", (-HD_T / 2 - 0.022, 0.068, h / 2 - 0.023), (-HD_T / 2, 0.076, h / 2 - 0.013), "chrome", True),
        _bx("handle_post_r", (-HD_T / 2 - 0.022, -0.076, h / 2 - 0.023), (-HD_T / 2, -0.068, h / 2 - 0.013), "chrome", True),
    ]
    for side, sign in (("l", 1.0), ("r", -1.0)):
        y = sign * (OPEN_HALF_W - 0.003)
        outline = [(-h / 2, HD_T / 2), (h / 2 - 0.010, HD_T / 2), (h / 2 - 0.010, HD_T / 2 + 0.085)]  # (z, x)
        pts, tris = prism(outline, y - 0.0008, y + 0.0008, "Y")
        parts.append(_mesh(f"wing_{side}", pts, tris, "powder"))
    return parts + door_edges(AD_W, h, HD_T)


def clip_parts() -> list[Part]:
    """The outgoing-mail clip: its plate is the body's own box; a turned-out lip at its free (lower) edge (looks)."""
    sx, sy, sz = CLIP_SIZE
    return [_bx("lip", (sx / 2, -sy / 2, -sz / 2 - 0.0005), (sx / 2 + 0.003, sy / 2, -sz / 2 + 0.0015), "chrome")]


def flag_parts() -> list[Part]:
    """The flag (its frame: the arm's centre, flag down, the arm along +x from its pivot): the arm is the body's own
    box; its plate head (a collider, as in the maker's 2010 drawing) hangs below its end when it is down; a pivot
    boss (looks)."""
    half = FLAG_LEN / 2
    hl, hw = FLAG_HEAD
    t, b = FLAG_T, FLAG_BAR
    return [
        _bx("head", (half - hl, -t / 2, -hw), (half, t / 2, -b / 2), "red", True),
        _cy("pivot_boss", (-half, 0.0, 0.0), 0.009, t + 0.002, "Y", "red"),
    ]


def parcel_parts() -> list[Part]:
    """The parcel: the box is the body's own; a shipping label on the face toward the arms' left, its address lines,
    and the tuck flap's edge along the top (looks)."""
    sx, sy, sz = PARCEL_SIZE
    face = sy / 2
    out = [_bx("label", (-0.045, face, -0.030), (0.045, face + 0.0004, 0.030), "label")]
    for k in range(4):
        z = 0.018 - 0.011 * k
        out.append(_bx(f"line_{k}", (-0.035, face + 0.0004, z), (0.025 - 0.010 * k, face + 0.0007, z + 0.003), "ink"))
    out.append(_bx("flap_edge", (-sx / 2, -sy / 2, sz / 2), (sx / 2, -sy / 2 + 0.002, sz / 2 + 0.0004), "ink"))
    return out


PARTS: dict[str, Callable[[], list[Part]]] = {
    MAILBOX: mailbox_parts,
    KEY_MAT_N: lambda: [],
    ACCESS_DOOR: access_door_parts,
    PLUG: plug_parts,
    KEY: key_parts,
    HOPPER: hopper_parts,
    CLIP: clip_parts,
    FLAG: flag_parts,
    PARCEL: parcel_parts,
}


def local_outline(name: str) -> tuple[np.ndarray, np.ndarray]:
    """A body's outline in its own frame (low and high corners): its own box and every part the hook adds."""
    half = _v(SPECS[name].size) / 2
    lows, highs = [-half], [half]
    for part in PARTS[name]():
        lo, hi = part.bounds()
        lows.append(lo)
        highs.append(hi)
    return np.min(lows, axis=0), np.max(highs, axis=0)


# --- joints (frames in each body's own frame; every body starts unturned) ------------------------------------------
@dataclass(frozen=True)
class JointSpec:
    """A joint from body0 to body1 at a point (world, as built) with its axis; limits in degrees; its drive."""

    name: str
    kind: str  # "revolute" or "d6"
    body0: str
    body1: str
    point: Vec3  # body1's frame point on the joint, world, as built
    body1_point: Vec3  # the same point for body1 when it differs (the d6 key retainer's shoulder)
    axis: str
    limits: tuple[float, float]
    stiffness: float
    damping: float
    target: float
    max_force: float
    collides: bool = False


AD_HINGE = (AD_CENTER[0], Y_MB, Z_BOT + AD_ZB[0])
HD_HINGE = (HD_CENTER[0], Y_MB, Z_BOT + HD_ZB[0])
LOCK_AXIS_POINT = (X_CAM, LOCK_Y, Z_BOT + LOCK_ZB)
FACE_POINT = (X_FACE, LOCK_Y, Z_BOT + LOCK_ZB)
CLIP_HINGE = (CLIP_CENTER[0], Y_MB, Z_BOT + CLIP_TOP_ZB)
FLAG_AXIS = (FLAG_PIVOT[0], FLAG_PIVOT[1], Z_BOT + FLAG_PIVOT[2])
SHOULDER_LOCAL = (KEY_BOW[0] / 2, 0.0, 0.0)
JOINTS = (
    JointSpec("access_hinge", "revolute", MAILBOX, ACCESS_DOOR, AD_HINGE, AD_HINGE, "Y", (-AD_OPEN_LIMIT, 0.0), CATCH_K, 0.0005, 0.0, CATCH_TORQUE[0]),
    JointSpec("lock_plug", "revolute", ACCESS_DOOR, PLUG, LOCK_AXIS_POINT, LOCK_AXIS_POINT, "X", (UNLOCKED_DEG, 0.0), PLUG_HOLD_K, 0.002, 0.0, PLUG_HOLD_TORQUE),
    JointSpec("key_retainer", "d6", PLUG, KEY, FACE_POINT, (0.0, 0.0, 0.0), "X", (0.0, 0.0), 0.0, 0.0, 0.0, RETAIN_FORCE, True),
    JointSpec("hopper_hinge", "revolute", MAILBOX, HOPPER, HD_HINGE, HD_HINGE, "Y", (-HD_OPEN_LIMIT, 0.0), CATCH_K, 0.001, 0.0, CATCH_TORQUE[1]),
    JointSpec("clip_hinge", "revolute", HOPPER, CLIP, CLIP_HINGE, CLIP_HINGE, "Y", (-CLIP_OPEN, 0.0), CLIP_K, 0.0002, CLIP_PRELOAD, CLIP_TORQUE),
    JointSpec("flag_pivot", "revolute", MAILBOX, FLAG, FLAG_AXIS, FLAG_AXIS, "Y", (-90.0, 0.0), FLAG_K, 0.0005, 0.0, FLAG_TORQUE),
)
JOINT_OF = {j.body1: j for j in JOINTS}  # each joint is built with its body1 (spawned after its body0)
CENTERS = {o.name: _t(o.pos) for o in OBJECTS}


def joint_frames(spec: JointSpec) -> tuple[Vec3, Vec3]:
    """The joint's frame origin in body0's and in body1's own frame."""
    pos0 = _t(_v(spec.point) - _v(CENTERS[spec.body0]))
    pos1 = SHOULDER_LOCAL if spec.kind == "d6" else _t(_v(spec.body1_point) - _v(CENTERS[spec.body1]))
    return pos0, pos1


# --- the scene hook: what the task-file API can't express -------------------------------------------------------
# yam_arena.scene.task_object(spec) turns each task object into an Arena object. Right after that module is imported,
# this wraps task_object, matching this file's own object specs by value, so that after Arena spawns one its spawner
# also builds that body's parts (PARTS: colliders and looks, each Gprim with its extent) and the joint that hangs it
# on the body spawned before it (JOINTS). On a machine without the simulator nothing imports yam_arena.scene, and this
# does nothing. What was built is recorded on the scene module; scoring gives 0 unless every part was built.
_SCENE_MODULE = "yam_arena.scene"
PARTS_FOLDER = "parts"
JOINTS_FOLDER = "mailbox_joints"
HOOK_TAG = "get_mail_locking_mailbox_parts"
BUILD_RECORD = "get_mail_locking_mailbox_build"
FIT_CONTACT_OFFSET = 0.0004
FIT_REST_OFFSET = 0.0
SOLVER = (32, 4)
OWN_LOOK = {
    MAILBOX: "galv",
    KEY_MAT_N: "silicone",
    ACCESS_DOOR: "powder",
    PLUG: "chrome",
    KEY: "nickel",
    HOPPER: "powder",
    CLIP: "chrome",
    FLAG: "red",
    PARCEL: "card",
}


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


def _material(px: _Pxr, stage: Any, path: str, look: str, friction: tuple[float, float]) -> Any:
    rgb, metallic, roughness = LOOKS[look]
    material = px.shade.Material.Define(stage, path)
    shader = px.shade.Shader.Define(stage, f"{path}/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    floats = px.sdf.ValueTypeNames.Float
    shader.CreateInput("diffuseColor", px.sdf.ValueTypeNames.Color3f).Set(px.gf.Vec3f(*(_linear(c) for c in rgb)))
    shader.CreateInput("metallic", floats).Set(float(metallic))
    shader.CreateInput("roughness", floats).Set(float(roughness))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    api = px.physics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(float(friction[0]))
    api.CreateDynamicFrictionAttr(float(friction[1]))
    api.CreateRestitutionAttr(0.0)
    return material


def spec_colliders(px: _Pxr, stage: Any, body: str) -> list[Any]:
    """The colliders the framework made for a spec (its own primitive), not the parts this file adds."""
    folder = f"{body}/{PARTS_FOLDER}"
    return [
        prim
        for prim in px.usd.PrimRange(stage.GetPrimAtPath(body))
        if prim.HasAPI(px.physics.CollisionAPI) and not (str(prim.GetPath()) == folder or str(prim.GetPath()).startswith(folder + "/"))
    ]


def _place(px: _Pxr, prim: Any, center: Pt, scale: Pt | None = None) -> None:
    xform = px.geom.Xformable(prim)
    xform.AddTranslateOp().Set(px.gf.Vec3d(*(float(c) for c in center)))
    if scale is not None:
        xform.AddScaleOp().Set(px.gf.Vec3f(*(float(s) for s in scale)))


def _author_part(px: _Pxr, stage: Any, folder: str, part: Part, materials: dict[str, Any]) -> str:
    path = f"{folder}/{part.name}"
    f3 = px.gf.Vec3f
    if part.kind == "box":
        gprim = px.geom.Cube.Define(stage, path)
        gprim.CreateSizeAttr(1.0)
        gprim.CreateExtentAttr([f3(-0.5, -0.5, -0.5), f3(0.5, 0.5, 0.5)])
        _place(px, gprim.GetPrim(), part.center, part.size)
    elif part.kind == "cyl":
        gprim = px.geom.Cylinder.Define(stage, path)
        gprim.CreateRadiusAttr(float(part.radius))
        gprim.CreateHeightAttr(float(part.height))
        gprim.CreateAxisAttr(part.axis)
        lo, hi = part.bounds()
        c = _v(part.center)
        gprim.CreateExtentAttr([f3(*(float(v) for v in lo - c)), f3(*(float(v) for v in hi - c))])
        _place(px, gprim.GetPrim(), part.center)
    else:
        gprim = px.geom.Mesh.Define(stage, path)
        gprim.CreatePointsAttr([f3(*(float(v) for v in p)) for p in part.points])
        gprim.CreateFaceVertexCountsAttr([3] * len(part.tris))
        gprim.CreateFaceVertexIndicesAttr([int(i) for i in part.tris.reshape(-1)])
        gprim.CreateSubdivisionSchemeAttr("none")
        gprim.CreateDoubleSidedAttr(True)
        lo, hi = part.points.min(axis=0), part.points.max(axis=0)
        gprim.CreateExtentAttr([f3(*(float(v) for v in lo)), f3(*(float(v) for v in hi))])
        _place(px, gprim.GetPrim(), part.center)
    prim = gprim.GetPrim()
    material = materials[part.look]
    binding = px.shade.MaterialBindingAPI.Apply(prim)
    binding.Bind(material)
    if part.collides:
        px.physics.CollisionAPI.Apply(prim)
        binding.Bind(material, px.shade.Tokens.weakerThanDescendants, "physics")
        if part.fit:
            prim.AddAppliedSchema("PhysxCollisionAPI")
            floats = px.sdf.ValueTypeNames.Float
            prim.CreateAttribute("physxCollision:contactOffset", floats).Set(FIT_CONTACT_OFFSET)
            prim.CreateAttribute("physxCollision:restOffset", floats).Set(FIT_REST_OFFSET)
    return path


def _tune_body(px: _Pxr, prim: Any) -> None:
    prim.AddAppliedSchema("PhysxRigidBodyAPI")
    ints = px.sdf.ValueTypeNames.Int
    prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", ints).Set(SOLVER[0])
    prim.CreateAttribute("physxRigidBody:solverVelocityIterationCount", ints).Set(SOLVER[1])


def _drive(px: _Pxr, prim: Any, kind: str, spec: JointSpec) -> None:
    drive = px.physics.DriveAPI.Apply(prim, kind)
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(spec.stiffness))
    drive.CreateDampingAttr(float(spec.damping))
    drive.CreateTargetPositionAttr(float(spec.target))
    drive.CreateTargetVelocityAttr(0.0)
    drive.CreateMaxForceAttr(float(spec.max_force))  # always finite (an infinite one is rejected on this stack)


def author_joint(px: _Pxr, stage: Any, env_root: str, spec: JointSpec) -> str:
    path = f"{env_root}/{JOINTS_FOLDER}/{spec.name}"
    pos0, pos1 = joint_frames(spec)
    joint = (px.physics.RevoluteJoint if spec.kind == "revolute" else px.physics.Joint).Define(stage, path)
    joint.CreateBody0Rel().SetTargets([f"{env_root}/{spec.body0}"])
    joint.CreateBody1Rel().SetTargets([f"{env_root}/{spec.body1}"])
    joint.CreateLocalPos0Attr(px.gf.Vec3f(*pos0))
    joint.CreateLocalRot0Attr(px.gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(px.gf.Vec3f(*pos1))
    joint.CreateLocalRot1Attr(px.gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateCollisionEnabledAttr(bool(spec.collides))
    if spec.kind == "revolute":
        joint.CreateAxisAttr(spec.axis)
        joint.CreateLowerLimitAttr(float(spec.limits[0]))
        joint.CreateUpperLimitAttr(float(spec.limits[1]))
        _drive(px, joint.GetPrim(), "angular", spec)
    else:  # the key retainer: free in every direction, a drive along the keyway that is off until the plug turns
        _drive(px, joint.GetPrim(), "transX", spec)
    return path


def author(stage: Any, env_root: str, name: str) -> list[str]:
    """Build a body's parts (and its joint to the body it hangs on); the paths made."""
    px = _Pxr()
    body = f"{env_root}/{name}"
    folder = f"{body}/{PARTS_FOLDER}"
    friction = FRICTION[name]
    looks = {OWN_LOOK[name], *(p.look for p in PARTS[name]())}
    materials = {look: _material(px, stage, f"{folder}/mat_{look}", look, friction) for look in sorted(looks)}
    own = spec_colliders(px, stage, body)
    if not own:
        raise RuntimeError(f"no collider under {body}")
    for prim in own:
        binding = px.shade.MaterialBindingAPI.Apply(prim)
        binding.Bind(materials[OWN_LOOK[name]], px.shade.Tokens.strongerThanDescendants)
        binding.Bind(materials[OWN_LOOK[name]], px.shade.Tokens.strongerThanDescendants, "physics")
    made = [str(m.GetPath()) for m in materials.values()]
    for part in PARTS[name]():
        made.append(_author_part(px, stage, folder, part, materials))
    if name in FREE:
        _tune_body(px, stage.GetPrimAtPath(body))
    if name == HOPPER:  # its wings are inside: its weight sits behind the hinge line, so shut it stays shut
        px.physics.MassAPI.Apply(stage.GetPrimAtPath(body)).CreateCenterOfMassAttr(px.gf.Vec3f(*HD_COM))
    if name in JOINT_OF:
        made.append(author_joint(px, stage, env_root, JOINT_OF[name]))
    return made


def _author_of(name: str) -> Callable[[Any, str], list[str]]:
    def build(stage: Any, root: str) -> list[str]:
        return author(stage, root, name)

    return build


AUTHORS: dict[str, Callable[[Any, str], list[str]]] = {name: _author_of(name) for name in PARTS}


def _env_roots(prim: Any, name: str) -> list[str]:
    """Every environment folder holding the object ``name``, given its prim just spawned (cloned copies included)."""
    stage = prim.GetStage()
    envs = prim.GetPath().GetParentPath().GetParentPath()
    return [str(env.GetPath()) for env in stage.GetPrimAtPath(envs).GetChildren() if env.GetChild(name).IsValid()]


def _build_record(scene: Any) -> dict[str, Any]:
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
            _warn_once(f"build-{name}", f"could not build {name}'s parts ({type(error).__name__}: {error})")
        else:
            if record.get(name) != "failed":
                record[name] = "built"
        return prim

    setattr(spawn_and_build, HOOK_TAG, name)
    return spawn_and_build


def _ours(spec: Any) -> bool:
    try:
        return getattr(spec, "name", None) in AUTHORS and any(spec == own for own in OBJECTS)
    except Exception:
        return False


def _spawner_cfgs(item: object) -> Iterator[Any]:
    seen: set[int] = set()
    stack = [item, *vars(item).values()] if hasattr(item, "__dict__") else [item]
    for candidate in stack:
        for value in (candidate, getattr(candidate, "spawn", None), getattr(candidate, "spawner_cfg", None)):
            if value is None or id(value) in seen:
                continue
            seen.add(id(value))
            if callable(getattr(value, "func", None)) and (hasattr(value, "size") or hasattr(value, "radius")):
                yield value


def build_scene(scene: Any) -> bool:
    """Wrap ``scene.task_object`` so this file's objects come out with their parts; True if it wrapped it now."""
    original = getattr(scene, "task_object", None)
    if original is None or getattr(original, HOOK_TAG, None) is not None:
        return False
    record = _build_record(scene)

    def task_object(spec: Any) -> Any:
        if _ours(spec) and spec.name == OBJECTS[0].name:  # the first of this task's objects: a new build starts
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
                _warn_once(f"spawner-{spec.name}", f"found no spawner for {spec.name}; its parts will be missing")
        return made

    setattr(task_object, HOOK_TAG, "task_object")
    scene.task_object = task_object
    return True


class _SceneWatcher(importlib.abc.MetaPathFinder):
    """Waits for yam_arena.scene to be imported, lets it load normally, then applies build_scene."""

    marker = "get_mail_locking_mailbox.scene_watcher"

    def __init__(self) -> None:
        self._looking = False

    def find_spec(self, fullname: str, path: Sequence[str] | None, target: types.ModuleType | None = None) -> importlib.machinery.ModuleSpec | None:
        if fullname != _SCENE_MODULE or self._looking:
            return None
        self._looking = True
        try:
            spec = importlib.util.find_spec(fullname)
        except Exception as error:
            self.disarm()
            _warn_once("finder", f"could not watch the scene module ({type(error).__name__}: {error})")
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
        except Exception as error:
            with contextlib.suppress(Exception):
                _build_record(module).update(dict.fromkeys(AUTHORS, "failed"))
            _warn_once("wrap", f"could not wrap the scene builder ({type(error).__name__}: {error})")

    def __getattr__(self, name: str) -> Any:
        if name.startswith("__") or name in ("_inner", "_watcher"):
            raise AttributeError(name)
        return getattr(self._inner, name)


def install_hook() -> None:
    scene = sys.modules.get(_SCENE_MODULE)
    if scene is not None:
        build_scene(scene)
        return
    if not any(getattr(finder, "marker", None) == _SceneWatcher.marker for finder in sys.meta_path):
        sys.meta_path.insert(0, _SceneWatcher())


def parts_built() -> bool:
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


MAX_SPEED, MAX_SPIN = 0.01, 0.5
TABLE_X = (-0.19, 0.91)
TABLE_Y = (-0.65, 0.65)
LOST_BELOW = 0.05


@dataclass(frozen=True)
class PartView:
    """One body as the simulator reports it: its outline's corners and centre, its rotation, its body origin (from
    its outline and the parts it was built with) and how fast it moves."""

    name: str
    center: np.ndarray
    low: np.ndarray
    high: np.ndarray
    rot: np.ndarray
    speed: float
    spin: float

    @staticmethod
    def of(name: str, state: _State) -> "PartView":
        return PartView(
            name,
            _v(state.pos),
            _v(state.low),
            _v(state.high),
            rotation(state.quat_xyzw),
            float(np.linalg.norm(_v(state.lin_vel))),
            float(np.linalg.norm(_v(state.ang_vel))),
        )

    @property
    def origin(self) -> np.ndarray:
        low, high = local_outline(self.name)
        return np.asarray(self.center - self.rot @ ((low + high) / 2), dtype=float)

    def world(self, local: Pt) -> np.ndarray:
        return np.asarray(self.origin + self.rot @ _v(local), dtype=float)

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


def hinge_open_deg(door: PartView) -> float:
    """A bottom-hinged door's opening (degrees; 0 shut, its top edge swung toward the arms positive)."""
    z_axis = door.rot[:, 2]
    return math.degrees(math.atan2(-float(z_axis[0]), float(z_axis[2])))


def plug_deg(door: PartView, plug: PartView) -> float:
    """The plug's turn in the door about the lock's axis (degrees; 0 locked, -90 unlocked)."""
    rel = door.rot.T @ plug.rot
    return math.degrees(math.atan2(float(rel[2, 1]), float(rel[1, 1])))


def flag_up_deg(flag: PartView) -> float:
    """The flag's rise (degrees; 0 down along the side, 90 straight up)."""
    arm = flag.rot[:, 0]
    return math.degrees(math.atan2(float(arm[2]), float(arm[0])))


def clip_lift_deg(hopper: PartView, clip: PartView) -> float:
    """How far the outgoing-mail clip stands off the hopper door (degrees; 0 shut on it)."""
    rel = hopper.rot.T @ clip.rot
    return -math.degrees(math.atan2(float(rel[0, 2]), float(rel[2, 2])))


@dataclass(frozen=True)
class KeyFit:
    """The key against the lock: its shoulder's distance in front of the plug's face (m; negative past it), its
    distance off the lock's axis (m), its blade's angle to the axis (degrees), and whether its tip is in the keyway."""

    gap: float
    radial: float
    tilt: float

    @property
    def full(self) -> bool:
        return self.gap < FULL_IN and self.radial < ON_AXIS and self.tilt < ALIGNED_DEG

    @property
    def entered(self) -> bool:
        return self.gap < BLADE[0] and self.radial < 0.003 and self.tilt < 15.0


def key_fit(plug: PartView, key: PartView) -> KeyFit:
    face = plug.world(_v(FACE_POINT) - _v(PLUG_CENTER))
    axis = plug.rot[:, 0]
    shoulder = key.world(SHOULDER_LOCAL)
    d = shoulder - face
    along = float(d @ axis)
    radial = float(np.linalg.norm(d - along * axis))
    c = float(np.clip(key.rot[:, 0] @ axis, -1.0, 1.0))
    return KeyFit(-along, radial, math.degrees(math.acos(c)))


# --- the mechanisms' rules, applied every control step (README, "How the file works, and its fidelity") --------------
@dataclass
class Mechanisms:
    """What the rules last wrote, so a write happens only on a change."""

    last: dict[tuple[str, str], float] = field(default_factory=dict)

    def step(self, parts: Mapping[str, PartView]) -> list[tuple[str, str, float]]:
        door, plug, key = parts[ACCESS_DOOR], parts[PLUG], parts[KEY]
        hopper, flag = parts[HOPPER], parts[FLAG]
        turn = plug_deg(door, plug)
        fit = key_fit(plug, key)
        end = 0.0 if turn > UNLOCKED_DEG / 2 else UNLOCKED_DEG
        want: list[tuple[str, str, float]] = []
        # The plug: held by its wafers unless the key is fully in; then free (its blade in the slot turns it), with a
        # light snap into either end.
        if fit.full:
            snap = abs(turn - end) < SNAP_DEG
            want += [
                ("lock_plug", "drive:angular:physics:stiffness", SNAP_K if snap else 0.0),
                ("lock_plug", "drive:angular:physics:damping", PLUG_FREE_DAMPING),
                ("lock_plug", "drive:angular:physics:targetPosition", end),
                ("lock_plug", "drive:angular:physics:maxForce", SNAP_TORQUE),
            ]
        else:
            want += [
                ("lock_plug", "drive:angular:physics:stiffness", PLUG_HOLD_K),
                ("lock_plug", "drive:angular:physics:damping", 0.002),
                ("lock_plug", "drive:angular:physics:targetPosition", end),
                ("lock_plug", "drive:angular:physics:maxForce", PLUG_HOLD_TORQUE),
            ]
        # The key stays in while the plug is turned off locked (key removal in the locked position only); home at
        # locked, the wafers' springs grip it lightly (a light pull takes it out; it does not slide out by itself).
        if turn < -AT_LOCKED_DEG:
            k, c, cap = RETAIN_K, 5.0, RETAIN_FORCE
        elif fit.gap < FULL_IN and fit.radial < ON_AXIS:
            k, c, cap = WAFER_K, 1.0, WAFER_FORCE
        else:
            k, c, cap = 0.0, 0.0, RETAIN_FORCE
        want.append(("key_retainer", "drive:transX:physics:stiffness", k))
        want.append(("key_retainer", "drive:transX:physics:damping", c))
        want.append(("key_retainer", "drive:transX:physics:maxForce", cap))
        # Each door's magnet: within CATCH_DEG of shut it pulls the door shut.
        for joint, part in (("access_hinge", door), ("hopper_hinge", hopper)):
            near = hinge_open_deg(part) < CATCH_DEG
            want.append((joint, "drive:angular:physics:stiffness", CATCH_K if near else 0.0))
        # The flag's sprung catch: past halfway it carries the flag on to the stop; held there.
        want.append(("flag_pivot", "drive:angular:physics:targetPosition", -90.0 if flag_up_deg(flag) > FLAG_SWITCH else 0.0))
        out = []
        for joint, attr, value in want:
            if self.last.get((joint, attr)) != value:
                self.last[(joint, attr)] = value
                out.append((joint, attr, value))
        return out


def _stage() -> Any:
    return importlib.import_module("omni.usd").get_context().get_stage()


def apply_writes(writes: Sequence[tuple[str, str, float]]) -> None:
    """Make the step's writes on the live stage (only where the simulator runs): drive set points and gains."""
    if not writes or not _simulator_present():
        return
    try:
        stage = _stage()
        root = _env_root()
        for joint, attr, value in writes:
            stage.GetPrimAtPath(f"{root}/{JOINTS_FOLDER}/{joint}").GetAttribute(attr).Set(float(value))
    except Exception as error:
        _warn_once("writes", f"could not write the mechanisms' joints ({type(error).__name__}: {error})")


# --- the robot's grippers ------------------------------------------------------------------------------------------
PAD_TIP_PAST_TCP = 0.013
PAD_LENGTH = 0.030
PAD_WIDTH = 0.027


@dataclass(frozen=True)
class Gripper:
    tcp: np.ndarray
    opening: float
    approach: tuple[float, float, float] = (0.0, 0.0, -1.0)
    closing: tuple[float, float, float] = (0.0, 1.0, 0.0)

    def finger_points(self) -> list[np.ndarray]:
        tcp = _v(self.tcp)
        approach, closing = _v(self.approach), _v(self.closing)
        across = np.cross(approach, closing)
        points = [tcp]
        for finger in (-0.5, 0.5):
            for along in (PAD_TIP_PAST_TCP, PAD_TIP_PAST_TCP - PAD_LENGTH / 2, PAD_TIP_PAST_TCP - PAD_LENGTH):
                for edge in (-0.5, 0.0, 0.5):
                    points.append(tcp + finger * self.opening * closing + along * approach + edge * PAD_WIDTH * across)
        return points


HAND_CLEAR = 0.030


def hands_clear(parts: Mapping[str, PartView], grippers: Sequence[Gripper]) -> bool:
    """Every finger point at least HAND_CLEAR from every loose part's outline (the doors', flag's and lock's too)."""
    for g in grippers:
        for p in g.finger_points():
            for part in parts.values():
                gap = np.maximum(np.maximum(part.low - p, p - part.high), 0.0)
                if float(np.linalg.norm(gap)) < HAND_CLEAR:
                    return False
    return True


# --- the outcome checks (a ruler, a protractor, photos and the trial video; README, "Score") --------------------------
SHUT_DEG = 1.5
LOCKED_DEG = 5.0  # the plug within this of locked
OPENED_DEG = 30.0
MAT_MARGIN = 0.005
DOWN_TOL = 0.006
HOLD_STEPS = 20


def parcel_placed(parcel: PartView) -> bool:
    """The parcel is on the table inside its taped place: seen from above, the rectangle that bounds it (sides
    parallel to the table's edges) inside the tape's inside edges, and its lowest point within DOWN_TOL of the table."""
    lo, hi = _v(PARCEL_PLACE) - _v(PLACE_SIZE) / 2, _v(PARCEL_PLACE) + _v(PLACE_SIZE) / 2
    inside = bool(np.all(parcel.low[:2] >= lo) and np.all(parcel.high[:2] <= hi))
    return inside and float(parcel.low[2]) < TOP + DOWN_TOL


def key_home(key: PartView) -> bool:
    """The key lies on its mat (its outline over the mat, its underside on the mat)."""
    lo, hi = _v(MAT_CENTER) - _v(KEY_MAT) / 2, _v(MAT_CENTER) + _v(KEY_MAT) / 2
    over = bool(np.all(key.low[:2] >= lo[:2] - MAT_MARGIN) and np.all(key.high[:2] <= hi[:2] + MAT_MARGIN))
    return over and float(key.low[2]) < TOP + KEY_MAT[2] + DOWN_TOL


@dataclass(frozen=True)
class State:
    door_open: float
    plug: float
    key_full: bool
    hopper_open: float
    flag_up: float
    opened: bool
    parcel_placed: bool
    key_home: bool
    still: bool
    clear: bool
    lost: tuple[str, ...]

    @property
    def shut_and_locked(self) -> bool:
        return self.door_open < SHUT_DEG and self.plug > -LOCKED_DEG


def state(parts: Mapping[str, PartView], grippers: Sequence[Gripper] | None) -> State:
    door, plug, key = parts[ACCESS_DOOR], parts[PLUG], parts[KEY]
    hopper = parts[HOPPER]
    door_open, turn = hinge_open_deg(door), plug_deg(door, plug)
    return State(
        door_open=door_open,
        plug=turn,
        key_full=key_fit(plug, key).full,
        hopper_open=hinge_open_deg(hopper),
        flag_up=flag_up_deg(parts[FLAG]),
        opened=door_open > OPENED_DEG,
        parcel_placed=parcel_placed(parts[PARCEL]),
        key_home=key_home(key),
        still=all(p.still for p in parts.values()),
        clear=grippers is not None and hands_clear(parts, grippers),
        lost=tuple(n for n in (KEY, PARCEL) if parts[n].lost),
    )


@dataclass
class Ledger:
    steps: int = 0
    opened: bool = False
    void: str | None = None
    held: int = 0

    def observe(self, now: State) -> None:
        self.steps += 1
        self.opened = self.opened or (now.opened and now.plug < UNLOCKED_DEG / 2)
        if now.lost and self.void is None:
            self.void = f"{', '.join(now.lost)} off the table"


def grade(ledger: Ledger, now: State, built: bool) -> float:
    """The ladder (README, "Score"). Every rung above 0 needs the access door to have been opened past 30 degrees
    with the plug turned to unlocked (the key did it)."""
    if not built or ledger.void is not None:
        return 0.0
    placed = ledger.opened and now.parcel_placed
    locked = placed and now.shut_and_locked
    done = locked and now.key_home
    if done and now.still and now.clear:
        ledger.held += 1
    else:
        ledger.held = 0
    if done and ledger.held >= HOLD_STEPS:
        return 1.0
    if locked:
        return 0.75
    if placed:
        return 0.5
    return 0.25 if ledger.opened else 0.0


# --- episodes and the robot's state ------------------------------------------------------------------------------------
_LEDGER = Ledger()
_MECH = Mechanisms()
_SCORE = 0.0
_last_objects: object = None
_embodiment: Any = None
_last_clock: float | None = None
_hooked = False
_warned: set[str] = set()
_ARMS = (("left_robot", "left"), ("right_robot", "right"))
RESET_TAG = "get_mail_locking_mailbox_resets"
LAST_STATE: dict[str, Any] = {}  # the last step's readings (for traces)


def _warn_once(key: str, message: str) -> None:
    if key not in _warned:
        _warned.add(key)
        with contextlib.suppress(Exception):
            warnings.warn(f"get_mail_locking_mailbox: {message}", RuntimeWarning, stacklevel=3)


def reset_writes() -> list[tuple[str, str, float]]:
    """A new trial: every drive back to its built value."""
    out: list[tuple[str, str, float]] = []
    for j in JOINTS:
        kind = "angular" if j.kind == "revolute" else "transX"
        out += [
            (j.name, f"drive:{kind}:physics:stiffness", j.stiffness),
            (j.name, f"drive:{kind}:physics:damping", j.damping),
            (j.name, f"drive:{kind}:physics:targetPosition", j.target),
            (j.name, f"drive:{kind}:physics:maxForce", j.max_force),
        ]
    return out


def new_episode() -> None:
    global _LEDGER, _MECH, _SCORE, _last_objects, _last_clock
    _LEDGER = Ledger()
    _MECH = Mechanisms()
    _SCORE = 0.0
    _last_objects = None
    _last_clock = None
    _warned.difference_update(("robot", "void"))
    apply_writes(reset_writes())


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
    _warn_once("reset", "could not hook the embodiment's reset, so the arms' state is unknown: no trial can reach 1.0")


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
        out.append(Gripper(_v(tcp), float(-2.0 * q[list(finger_ids)].mean()), approach=_t(rot[:, 2]), closing=_t(rot[:, 0])))
    return out


def _robot() -> list[Gripper] | None:
    if _embodiment is None:
        return None
    try:
        return grippers_of(_embodiment)
    except Exception as error:
        _warn_once("robot", f"can't read the arms' state ({type(error).__name__}: {error}); the hands can't be checked clear, so 1.0 can't be reached")
        return None


def _update(objects: Mapping[str, Any]) -> float:
    """Advance the trial once per control step (success() and score() share the step's state dict)."""
    global _last_objects, _SCORE
    if objects is _last_objects:
        return _SCORE
    with contextlib.suppress(Exception):
        _hook_resets()
    _restart_if_clock_went_back()
    grippers = _robot()
    built = parts_built()
    try:
        parts = views(objects)
        apply_writes(_MECH.step(parts))
        now = state(parts, grippers)
        LAST_STATE.update(now.__dict__)
    except (LookupError, AttributeError, TypeError, ValueError, ArithmeticError) as error:
        _LEDGER.void = _LEDGER.void or f"can't read the parts ({type(error).__name__}: {error})"
        _warn_once("void", f"this trial scores 0: {_LEDGER.void}")
        _last_objects = objects
        _SCORE = 0.0
        return _SCORE
    _LEDGER.observe(now)
    _SCORE = grade(_LEDGER, now, built)
    _last_objects = objects
    return _SCORE


def score(objects: Mapping[str, Any]) -> float:
    """1.0: the parcel on its taped place, the access door shut and locked, the key back on its mat, everything still
    and both hands clear for 1 s; 0.75 without the key on its mat; 0.5 with the parcel placed; 0.25 once the door was
    opened by the key; see README.md."""
    return _update(objects)


def success(objects: Mapping[str, Any]) -> bool:
    return _update(objects) >= 1.0


# --- the scripted demo: both arms; it reads the parts' state and only moves the arms -----------------------------------
FINGER_MAX = -FINGER_OPEN * 2


class Grip(float):
    """A Waypoint yaw that carries a full tool rotation (3x3, columns: closing axis, across, approach)."""

    rot: np.ndarray

    def __new__(cls, rot: np.ndarray) -> "Grip":
        obj = super().__new__(cls, 0.0)
        obj.rot = np.asarray(rot, dtype=float)
        return obj


GRIP_TAG = "get_mail_locking_mailbox_grip"


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


def aim(approach: Pt, closing: Pt = (0.0, 0.0, 1.0)) -> np.ndarray:
    """A tool rotation pointing along ``approach``, closing as near ``closing`` as it can."""
    a = _v(approach) / float(np.linalg.norm(_v(approach)))
    c = _v(closing) - float(_v(closing) @ a) * a
    c = c / float(np.linalg.norm(c))
    return np.column_stack([c, np.cross(a, c), a])


def rx(deg: float) -> np.ndarray:
    t = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])


def ry(deg: float) -> np.ndarray:
    t = math.radians(deg)
    return np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])


def rz(deg: float) -> np.ndarray:
    t = math.radians(deg)
    return np.array([[math.cos(t), -math.sin(t), 0.0], [math.sin(t), math.cos(t), 0.0], [0.0, 0.0, 1.0]])


def rotvec(rot: np.ndarray) -> np.ndarray:
    """The rotation vector (axis times angle, radians) of a rotation matrix."""
    c = float(np.clip((np.trace(rot) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(c)
    if angle < 1e-9:
        return np.zeros(3)
    w = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
    return np.asarray(w / (2.0 * math.sin(angle)) * angle, dtype=float)


def turned(vec: Pt) -> np.ndarray:
    """The rotation of a rotation vector (radians)."""
    v = _v(vec)
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.eye(3)
    k = v / angle
    kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.asarray(np.eye(3) + math.sin(angle) * kx + (1.0 - math.cos(angle)) * (kx @ kx), dtype=float)


def _clip(vec: np.ndarray, most: float) -> np.ndarray:
    n = float(np.linalg.norm(vec))
    return vec if n <= most or n == 0.0 else np.asarray(vec * (most / n), dtype=float)


def _finger(opening: float) -> float:
    return float(-min(max(opening, 0.0), FINGER_MAX) / 2)


# First the right hand takes the key off its mat, puts it into the lock, turns it to unlocked and pulls the access
# door open by it, while the left arm watches from its park; the rest of the routine is further down.
#
# The key's places (its bow's centre): on the mat, and home in the lock (shoulder on the plug's face, on the axis).
KEY_HOME_AT = (X_FACE - KEY_BOW[0] / 2, LOCK_Y, Z_BOT + LOCK_ZB)
LOCK_AXIS_AT = (X_FACE, LOCK_Y, Z_BOT + LOCK_ZB)
# The right hand's grips on the key (TCP rotations, columns: closing, across, approach). Measured against the official
# URDF's collision meshes: a YAM finger is 15 mm wide at its tip and 40 mm wide 20 mm back, so a
# straight-down pinch can't bring a key to a lock on a flat face (the fingers meet the door first).
# - EDGE: off the mat and into the keyway. Across the bow's edges (24 mm apart: the fingers close on it hard), the hand
#   pitched EDGE_PITCH from straight down toward the lock, so its fingertips lead and the rest of the hand stays clear of
#   the door with the key home (30 deg: 2.8 mm clear at home with every link of the arm, 6 mm of each fingertip on the
#   bow's edge at the pick; straight down to 15 deg the fingers meet the door, and at 20-25 deg the forearm's wrist link
#   meets the hopper door as the key goes home, on the GPU). The bow's centre sits
#   EDGE_DROP straight below the TCP, with the fingertips' lowest points 1.5 mm into the silicone mat at the pick (they
#   stop on it: a light press down). The fingers close to EDGE_SQUEEZE, 2 mm inside the bow's edges each (about 10 N a
#   finger): squeezed at the full 50 N, the fingertips' rounded ends rode up over the 2 mm edges (on the GPU).
EDGE_PITCH = 30.0
EDGE_DROP = 0.0115
EDGE_SQUEEZE = 0.020
EDGE = aim((math.sin(math.radians(EDGE_PITCH)), 0.0, -math.cos(math.radians(EDGE_PITCH))), (0.0, -1.0, 0.0))
EDGE_OPEN = 0.036  # the fingers' opening round the 24 mm bow
# - FACE: for the turn. On the bow's flat faces (form closure about the blade: the bow can't turn between flat pads
#   without pushing them apart), level, pointing along the keyway, its pad tips FACE_SHORT short of the plug's face; the
#   quarter turn is then the wrist's own roll. The pads close on a 2 mm bow, so the turn rests on the bow's shape, not
#   on the grip's force.
FACE = aim((1.0, 0.0, 0.0), (0.0, 0.0, -1.0))  # this way round the wrist rolls from +90 to 0 deg (its limit is 120)
FACE_SHORT = 0.0025
FACE_TCP = (KEY_HOME_AT[0] + KEY_BOW[0] / 2 - FACE_SHORT - PAD_TIP_PAST_TCP, LOCK_Y, Z_BOT + LOCK_ZB)
FACE_OPEN = 0.016
# - PULL: for the door. After the turn the bow stands upright; the hand takes it again on its faces from above the
#   lock, pitched PULL_PITCH down, so that only its fingertips are in front of the door's upper half when it drops.
PULL_PITCH = 45.0
PULL = aim((math.cos(math.radians(PULL_PITCH)), 0.0, -math.sin(math.radians(PULL_PITCH))), (0.0, -1.0, 0.0))
PULL_SHORT = 0.008  # its fingertips' front corner this far short of the plug's face (5 mm: the dropping door brushed them)
PULL_OPEN = 0.016
# The pull: the hand draws the key round the hinge at PULL_SPEED (the arm is soft: the magnet's 0.15 N m needs about
# 15 mm of the arm's give before the door moves) until the door is PULL_AT open, past its magnet's 3 degrees; then it
# holds at most PULL_LEAD ahead of the door. Then it lets go by sliding its pads off the bow, up and back along its own
# axis, as its fingers open: the door drops to its stop behind the hand.
PULL_SPEED, PULL_AT, PULL_LEAD, PULL_MAX = 8.0, 5.0, 2.0, 30.0
RELEASE_S, RELEASE_BACK, RELEASE_UP = 0.28, 0.065, 0.025  # the slide off: its time, and how far back along the axis and up
# The search for the keyway (a key held in a parallel grip turns a little, so it needs a search; it finishes against a stop): the key is lined up
# LINE_UP in front of the face, fed in at FEED_SPEED; if its tip stops on the face (the key lags its feed by BLOCKED),
# the feed waits and the key's tip circles SEARCH_R about its aim until it drops in; it is fed OVERTRAVEL past home
# (the shoulder stops it).
LINE_UP = BLADE[0] + 0.003
FEED_SPEED = 0.004
BLOCKED = 0.0015
SEARCH_R = 0.0005
SEARCH_HZ = 1.2
OVERTRAVEL = 0.003
# The turn: the wrist rolls at TURN_SPEED, never more than TURN_LEAD ahead of the plug, and holds TURN_OVER past the
# plug once the plug is at its unlocked end (the end's stop takes the rest).
TURN_SPEED = 30.0
TURN_LEAD = 10.0
TURN_OVER = 4.0
PRESS = 0.0010  # the face pinch's forward push on the key while it turns (keeps the shoulder home)
SERVO_GAIN = 0.10  # of the error per control step (the arm takes a few steps to follow a change)
STEP_SHIFT, MAX_SHIFT = 0.0003, 0.010  # m
STEP_TURN, MAX_TURN = math.radians(0.4), math.radians(10.0)
_READ_ERRORS = (LookupError, AttributeError, TypeError, ValueError, ArithmeticError, ImportError)
# Modes of the servo (what it does while a waypoint is the current one).
PICK, LINEUP, INSERT, HOLD, FACE_ON, TURN, PULL_ON = "pick", "lineup", "insert", "hold", "face", "turn", "pull"
REGRIP_FACE, TURN_BACK, REGRIP = "regrip_face", "turn_back", "regrip"
PARCEL_ON, PARCEL_DRAW, DOOR_UNDER, DOOR_UP = "parcel", "parcel_draw", "door_under", "door_up"
EDGE_MODES = (PICK, LINEUP, INSERT, HOLD, REGRIP)
# The door lifted shut (right hand, below): the servo leads the door by at most DOOR_LEAD (the soft arm presses only as
# hard as it is held off its target) and pushes DOOR_PAST past shut once the magnet has the door.
DOOR_SPEED, DOOR_LEAD, DOOR_PAST = 15.0, 20.0, 1.0  # on the GPU, with 8 degrees the hand stopped 3-5 mm short of
# its target, pressing too lightly (about 2 N on the soft arm), and the door stayed on its stop


def _control_hz() -> float:
    try:
        hz = float(_embodiment.info.control_hz)
        return hz if math.isfinite(hz) and hz > 0 else 20.0
    except (LookupError, AttributeError, TypeError, ValueError, ArithmeticError):
        return 20.0


class KeyServo:
    """The demo's watching of the key (a scripted demo may read the simulator's state; it never moves a part). Every
    control step it reads the last step's parts and corrects the right hand's live targets, by the current waypoint's
    mode:
    - PICK: the hand's aim over the key as it lies (its place and turn on the mat);
    - LINEUP: the key held LINE_UP in front of the plug's face, on the lock's axis, square to it (position and turn);
    - INSERT: the key fed along the axis (the feed waits while the tip is stopped on the face, and the tip circles to
      find the slot), its line and turn corrected until the tip is in the slot (then the slot guides it);
    - HOLD: nothing changes;
    - FACE_ON: the face pinch centred on the key as it sits in the lock;
    - TURN: the wrist's roll led ahead of the plug, held past it at the unlocked end.
    """

    def __init__(self) -> None:
        self.phases: list[tuple[float, float, str | None]] = []
        self.labels: list[tuple[float, str]] = []
        self.restart()

    def restart(self) -> None:
        self.tick = -1
        self.mode: str | None = None
        self.pick_shift = np.zeros(3)
        self.pick_yaw = 0.0
        self.shift = np.zeros(3)
        self.turn = np.eye(3)
        self.feed = LINE_UP
        self.lag_steps = 0
        self.search_t = 0.0
        self.face_shift = np.zeros(3)
        self.roll = 0.0
        self.tip_in = False
        self.pull_deg = 0.0
        self.pulled = False
        self.parcel_shift = np.zeros(3)
        self.parcel_dz = 0.0
        self.door_phi = AD_OPEN_LIMIT
        self.regrip_shift = np.zeros(3)
        self.seen: dict[str, Any] = {}

    def follow(self, plan: Sequence[Waypoint]) -> None:
        out: list[tuple[float, float, str | None]] = []
        self.labels = []
        start = 0.0
        for w in plan:
            pos = w.pos
            mode = getattr(pos, "mode", None) if getattr(pos, "servo", None) is self else None
            out.append((start, start + float(w.duration), mode))
            self.labels.append((start + float(w.duration), str(w.label)))
            start += float(w.duration)
        self.phases = out

    def label_at(self, t: float) -> str:
        for end, label in self.labels:
            if t < end:
                return label
        return "done"

    def mode_at(self, t: float) -> str | None:
        for start, end, mode in self.phases:
            if start <= t < end:
                return mode
        return None

    # --- the targets the plan reads ---
    def edge_rot(self, mode: str | None) -> np.ndarray:
        self.update()
        if mode == PICK:
            return np.asarray(rz(self.pick_yaw) @ EDGE, dtype=float)
        if mode in (LINEUP, INSERT, HOLD):
            return np.asarray(self.turn @ EDGE, dtype=float)
        return EDGE

    def edge_tcp(self, key_at: Pt, mode: str | None) -> np.ndarray:
        """The TCP that puts the key (as held) at ``key_at`` (its bow's centre), corrected as the mode says."""
        self.update()
        want = _v(key_at)
        if mode == PICK:
            want = _v(KEY_CENTER) + rz(self.pick_yaw) @ (want - _v(KEY_CENTER)) + self.pick_shift
        elif mode == INSERT:
            want = _v(KEY_HOME_AT) - _v((self.feed, 0.0, 0.0)) + self.search() + self.shift
        elif mode == HOLD:  # home exactly (no overtravel), so that the hand doesn't spring on when it lets go
            want = _v(KEY_HOME_AT) - _v((max(self.feed, 0.0), 0.0, 0.0)) + self.search() + self.shift
        elif mode == LINEUP:
            want = want + self.shift
        elif mode in (REGRIP, None):
            want = want + self.regrip_shift
        held = EDGE.T @ _v((0.0, 0.0, -EDGE_DROP))  # the bow's centre in the grip's own frame
        return np.asarray(want - self.edge_rot(mode) @ held, dtype=float)

    def search(self) -> np.ndarray:
        if self.search_t <= 0.0:
            return np.zeros(3)
        a = 2.0 * math.pi * SEARCH_HZ * self.search_t
        r = SEARCH_R * min(1.0, self.search_t / 0.5)
        return _v((0.0, r * math.cos(a), r * math.sin(a)))

    def face_rot(self, rot: np.ndarray) -> np.ndarray:
        self.update()
        return np.asarray(rx(self.roll) @ rot, dtype=float)

    def face_tcp(self, tcp: Pt) -> np.ndarray:
        self.update()
        pivot = _v(LOCK_AXIS_AT)
        return np.asarray(pivot + rx(self.roll) @ (_v(tcp) - pivot) + self.face_shift, dtype=float)

    def pull_place(self, back: float, up: float) -> np.ndarray:
        self.update()
        tcp, rot = pull_tcp(self.pull_deg)
        return np.asarray(tcp - back * rot[:, 2] + _v((0.0, 0.0, up)), dtype=float)

    def pull_rot(self) -> np.ndarray:
        self.update()
        return pull_tcp(self.pull_deg)[1]

    # --- the watching ---
    def update(self) -> None:
        steps = _LEDGER.steps
        if steps == self.tick:
            return
        if steps < self.tick:
            self.restart()
        self.tick = steps
        objects = _last_objects
        if not isinstance(objects, Mapping):
            return
        hz = _control_hz()
        self.mode = self.mode_at((steps + 1) / hz)
        if self.mode is None:
            return
        saved = self._state()
        try:
            parts = views(objects)
            self.correct(self.mode, parts, 1.0 / hz)
        except _READ_ERRORS:
            self._restore(saved)
            return
        values = [v for k, v in self._state().items() if k not in ("tip_in", "pulled", "lag_steps")]
        if not all(np.all(np.isfinite(np.asarray(v, dtype=float))) for v in values):
            self._restore(saved)

    _KEPT = ("pick_shift", "pick_yaw", "shift", "turn", "feed", "lag_steps", "search_t", "face_shift", "roll", "tip_in",
             "pull_deg", "pulled", "parcel_shift", "parcel_dz", "door_phi", "regrip_shift")

    def _state(self) -> dict[str, Any]:
        return {k: (getattr(self, k).copy() if isinstance(getattr(self, k), np.ndarray) else getattr(self, k)) for k in self._KEPT}

    def _restore(self, saved: Mapping[str, Any]) -> None:
        for k, v in saved.items():
            setattr(self, k, v)

    def correct(self, mode: str, parts: Mapping[str, PartView], dt: float) -> None:
        key, plug, door = parts[KEY], parts[PLUG], parts[ACCESS_DOOR]
        if mode == PICK:
            off = key.origin - _v(KEY_CENTER)
            self.pick_shift = _clip(_v((off[0], off[1], 0.0)), 0.015)
            self.pick_yaw = float(np.clip(math.degrees(math.atan2(key.rot[1, 0], key.rot[0, 0])), -20.0, 20.0))
            return
        fit = key_fit(plug, key)
        self.seen = {"gap": fit.gap, "radial": fit.radial, "tilt": fit.tilt}
        if mode in (LINEUP, INSERT):
            face = plug.world(_v(FACE_POINT) - _v(PLUG_CENTER))
            axis = plug.rot[:, 0]
            gap = LINE_UP if mode == LINEUP else self.feed
            want = face - (KEY_BOW[0] / 2 + gap) * axis
            err = want - key.origin
            err_rot = rotvec(plug.rot @ key.rot.T)
            tip = fit.gap - BLADE[0]  # the blade's tip in front of the face (negative: in the slot)
            self.tip_in = self.tip_in or tip < -0.0015
            if mode == INSERT:
                err = err - float(err @ axis) * axis  # along the axis, the feed rules
                if fit.gap > self.feed + BLOCKED and not self.tip_in:
                    self.lag_steps += 1  # the key lags its feed: wait for it (and if the tip is on the face, search)
                else:
                    self.lag_steps = 0
                    self.feed = max(self.feed - FEED_SPEED * dt, -OVERTRAVEL)
                if self.lag_steps > 4 and tip < 0.0006:
                    self.search_t += dt  # the circle grows, then keeps where the tip drops in
            if mode == LINEUP or not self.tip_in:
                self.shift = _clip(self.shift + _clip(SERVO_GAIN * err, STEP_SHIFT), MAX_SHIFT)
                step = _clip(SERVO_GAIN * err_rot, STEP_TURN)
                candidate = turned(step) @ self.turn
                if float(np.linalg.norm(rotvec(candidate))) <= MAX_TURN:
                    self.turn = candidate
            self.seen.update(err_mm=[round(1000 * float(v), 2) for v in err], err_deg=round(math.degrees(float(np.linalg.norm(err_rot))), 2), tip_mm=round(1000 * tip, 2))
            return
        if mode == FACE_ON:
            axis_at = plug.world(_v(LOCK_AXIS_AT) - _v(PLUG_CENTER))
            off = key.origin - axis_at
            lateral = off - float(off @ plug.rot[:, 0]) * plug.rot[:, 0]
            self.face_shift = _clip(lateral, 0.004)
            return
        if mode == PULL_ON:
            opened = hinge_open_deg(door)
            self.pulled = self.pulled or opened >= PULL_AT
            if self.pulled:
                self.pull_deg = max(0.0, min(self.pull_deg, opened + PULL_LEAD))
            else:
                self.pull_deg = min(self.pull_deg + PULL_SPEED * dt, PULL_MAX)
            self.seen.update(door=round(opened, 2), pull=round(self.pull_deg, 2))
            return
        if mode == TURN:
            turn_now = plug_deg(door, plug)
            if turn_now <= UNLOCKED_DEG + 1.5:
                self.roll = max(min(self.roll, turn_now - TURN_OVER), UNLOCKED_DEG - 12.0)
            else:
                self.roll = max(self.roll - TURN_SPEED * dt, turn_now - TURN_LEAD, UNLOCKED_DEG - 12.0)
            self.seen.update(plug=round(turn_now, 2), roll=round(self.roll, 2))
            return
        if mode == REGRIP_FACE:  # the face pinch taken again on the turned key, as it sits
            self.roll = float(np.clip(plug_deg(door, plug), UNLOCKED_DEG - 6.0, 6.0))
            axis_at = plug.world(_v(LOCK_AXIS_AT) - _v(PLUG_CENTER))
            off = key.origin - axis_at
            lateral = off - float(off @ plug.rot[:, 0]) * plug.rot[:, 0]
            self.face_shift = _clip(lateral, 0.004)
            return
        if mode == TURN_BACK:
            turn_now = plug_deg(door, plug)
            if turn_now >= -1.5:
                self.roll = min(max(self.roll, turn_now + TURN_OVER), 12.0)
            else:
                self.roll = min(self.roll + TURN_SPEED * dt, turn_now + TURN_LEAD, 12.0)
            self.seen.update(plug=round(turn_now, 2), roll=round(self.roll, 2))
            return
        if mode == REGRIP:  # the edge grip centred on the key as it sits in the lock, locked
            self.regrip_shift = _clip(key.origin - _v(KEY_HOME_AT), 0.004)
            return
        if mode == PARCEL_ON:  # the parcel pinch aimed at the parcel as it lies
            self.parcel_shift = _clip(parts[PARCEL].origin - _v(PARCEL_CENTER), 0.015)
            return
        if mode == PARCEL_DRAW:  # held PARCEL_LIFT off the floor while it is drawn out (the arm sags under its weight)
            lift = float(parts[PARCEL].low[2]) - (Z_BOT + FLOOR_ZB)
            rise = float(np.clip(SERVO_GAIN * (PARCEL_LIFT - lift), -STEP_SHIFT, STEP_SHIFT))
            self.parcel_dz = float(np.clip(self.parcel_dz + rise, -0.004, PARCEL_SAG_MAX))
            self.seen.update(parcel_lift_mm=round(1000 * lift, 2))
            return
        if mode in (DOOR_UNDER, DOOR_UP):
            opened = hinge_open_deg(door)
            if mode == DOOR_UNDER:  # the fingertip follows the door where it lies
                self.door_phi = float(np.clip(opened, 0.0, AD_OPEN_LIMIT))
            elif opened < CATCH_DEG:  # in its catch: the fingertip just past shut, so the hand rests on it
                self.door_phi = max(min(self.door_phi - DOOR_SPEED * dt, opened), -DOOR_PAST)
            else:  # lift: never more than DOOR_LEAD ahead of the door (the soft arm lags under its weight)
                self.door_phi = max(self.door_phi - DOOR_SPEED * dt, opened - DOOR_LEAD, -DOOR_PAST)
            self.seen.update(door=round(opened, 2), door_phi=round(self.door_phi, 2))

    def report(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "pick_mm": [round(1000 * float(v), 2) for v in self.pick_shift],
            "pick_yaw": round(self.pick_yaw, 2),
            "shift_mm": [round(1000 * float(v), 2) for v in self.shift],
            "turn_deg": round(math.degrees(float(np.linalg.norm(rotvec(self.turn)))), 2),
            "feed_mm": round(1000 * self.feed, 2),
            "searching": self.search_t > 0.0,
            "tip_in": self.tip_in,
            "face_mm": [round(1000 * float(v), 2) for v in self.face_shift],
            "roll": round(self.roll, 2),
            "pull_deg": round(self.pull_deg, 2),
            "pulled": self.pulled,
            "parcel_mm": [round(1000 * float(v), 2) for v in self.parcel_shift],
            "parcel_dz_mm": round(1000 * self.parcel_dz, 2),
            "door_phi": round(self.door_phi, 2),
            "regrip_mm": [round(1000 * float(v), 2) for v in self.regrip_shift],
            **self.seen,
        }


class EdgePos:
    """A waypoint position for the edge grip: the TCP that puts the held key's bow centre at ``key_at``."""

    def __init__(self, servo: KeyServo, key_at: Pt, mode: str | None) -> None:
        self.servo, self.key_at, self.mode = servo, _v(key_at), mode

    def value(self) -> np.ndarray:
        return self.servo.edge_tcp(self.key_at, self.mode)

    def __array__(self, dtype: Any = None, copy: Any = None) -> np.ndarray:
        return np.asarray(self.value(), dtype=dtype)

    def __len__(self) -> int:
        return 3

    def __getitem__(self, index: Any) -> Any:
        return tuple(float(v) for v in self.value())[index]

    def __iter__(self) -> Iterator[float]:
        return iter(tuple(float(v) for v in self.value()))


class EdgeGrip(float):
    servo: KeyServo
    mode: str | None

    def __new__(cls, servo: KeyServo, mode: str | None) -> "EdgeGrip":
        obj = super().__new__(cls, 0.0)
        obj.servo, obj.mode = servo, mode
        return obj

    @property
    def rot(self) -> np.ndarray:
        return self.servo.edge_rot(self.mode)


class FacePos(EdgePos):
    """A waypoint position for the face grip: ``key_at`` is the TCP before the turn; rolled with the servo's roll
    about the lock's axis and centred on the key."""

    def value(self) -> np.ndarray:
        return self.servo.face_tcp(self.key_at)


class FaceGrip(float):
    servo: KeyServo
    nominal: np.ndarray

    def __new__(cls, servo: KeyServo, rot: np.ndarray) -> "FaceGrip":
        obj = super().__new__(cls, 0.0)
        obj.servo, obj.nominal = servo, np.asarray(rot, dtype=float)
        return obj

    @property
    def rot(self) -> np.ndarray:
        return self.servo.face_rot(self.nominal)


class PullPos(EdgePos):
    """A waypoint position for the pull grip: ``key_at`` holds (back along the grip's axis, up, 0) from its grip on the
    key, with the door at the servo's pull angle."""

    def value(self) -> np.ndarray:
        return self.servo.pull_place(float(self.key_at[0]), float(self.key_at[1]))


class PullGrip(float):
    servo: KeyServo

    def __new__(cls, servo: KeyServo) -> "PullGrip":
        obj = super().__new__(cls, 0.0)
        obj.servo = servo
        return obj

    @property
    def rot(self) -> np.ndarray:
        return self.servo.pull_rot()


SERVO: KeyServo | None = None  # the last plan's servo (traces read it)
# Parks (official IK: 12 and 19 degrees from every joint limit): the right hand level beside the lock, the left one
# from above the front; their wrist cameras watch the lock and the front.
LEFT_PARK = ((0.12, 0.08, TOP + 0.28), aim((1.0, -0.25, -0.45), (0.0, 0.0, -1.0)))
RIGHT_PARK = ((0.16, -0.12, TOP + 0.17), aim((1.0, 0.1, -0.2)))
FINGER_TIP_HALF = 0.0077  # half a YAM fingertip's width (official URDF collision mesh)
PAD_MID = 0.004  # the middle of the pads' flat faces, this far along the approach from the TCP


def pull_tcp(door_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """The pull grip's TCP and rotation with the door open ``door_deg`` (the key turned, home in its lock): the
    fingertips' front corner PULL_SHORT short of the plug's face, the pads' middle on the bow's middle."""
    p = math.radians(PULL_PITCH)
    front = PAD_TIP_PAST_TCP * math.cos(p) + FINGER_TIP_HALF * math.sin(p)
    tcp = _v((X_FACE - PULL_SHORT - front, LOCK_Y, Z_BOT + LOCK_ZB + PAD_MID * math.sin(p)))
    hinge = _v(AD_HINGE)
    t = ry(-door_deg)
    return np.asarray(hinge + t @ (tcp - hinge), dtype=float), np.asarray(t @ PULL, dtype=float)


def key_plan(servo: KeyServo) -> list[Waypoint]:
    """The right arm: the key off its mat, into the lock, the quarter turn, the door pulled open by the key."""
    start = _v(KEY_CENTER)
    lifted = start + _v((0.0, 0.0, 0.045))
    over = start + _v((0.0, 0.0, 0.050))
    lineup = _v(KEY_HOME_AT) - _v((LINE_UP, 0.0, 0.0))
    near = lineup - _v((0.030, 0.0, -0.010))
    held = _v(KEY_HOME_AT)
    e = {m: EdgeGrip(servo, m) for m in (None, PICK, LINEUP, INSERT, HOLD)}
    shut, edge, face_open, pull_open = _finger(0.0), _finger(EDGE_OPEN), _finger(FACE_OPEN), _finger(PULL_OPEN)
    squeeze = _finger(EDGE_SQUEEZE)
    face_back = _v(FACE_TCP) - _v((0.030, 0.0, 0.0))
    fg = FaceGrip(servo, FACE)
    plan = [
        Waypoint(2.5, EdgePos(servo, over, PICK), edge, "over the key", e[PICK]),
        Waypoint(1.5, EdgePos(servo, start, PICK), edge, "down round the key's bow", e[PICK]),
        Waypoint(1.0, EdgePos(servo, start, PICK), squeeze, "pinch the bow's edges", e[PICK]),
        Waypoint(1.2, EdgePos(servo, lifted, None), squeeze, "lift the key", e[None]),
        Waypoint(2.5, EdgePos(servo, near, None), squeeze, "carry it to the lock", e[None]),
        # Each servo phase starts with a short blend into its live target; the next waypoint has the same live target,
        # so the plan's own interpolation then follows the servo directly.
        Waypoint(1.5, EdgePos(servo, lineup, None), squeeze, "line it up with the keyway", e[None]),
        Waypoint(0.1, EdgePos(servo, lineup, LINEUP), squeeze, "watch the key", e[LINEUP]),
        Waypoint(1.8, EdgePos(servo, lineup, LINEUP), squeeze, "square it to the keyway", e[LINEUP]),
        Waypoint(0.2, EdgePos(servo, held, INSERT), squeeze, "start the feed", e[INSERT]),
        Waypoint(7.0, EdgePos(servo, held, INSERT), squeeze, "feed it into the keyway", e[INSERT]),
        Waypoint(0.6, EdgePos(servo, held, HOLD), squeeze, "hold it home", e[HOLD]),
        Waypoint(1.0, EdgePos(servo, held, HOLD), edge, "let go of the key", e[HOLD]),
        Waypoint(1.2, EdgePos(servo, held - _v((0.030, 0.0, -0.020)), None), edge, "back off", e[None]),
        Waypoint(2.0, FacePos(servo, face_back, FACE_ON), face_open, "turn the hand level", fg),
        Waypoint(1.5, FacePos(servo, _v(FACE_TCP), FACE_ON), face_open, "round the bow's faces", fg),
        Waypoint(1.0, FacePos(servo, _v(FACE_TCP), FACE_ON), shut, "pinch the bow's faces", fg),
        Waypoint(0.5, FacePos(servo, _v(FACE_TCP) + _v((PRESS, 0.0, 0.0)), None), shut, "press it home", fg),
        Waypoint(4.0, FacePos(servo, _v(FACE_TCP) + _v((PRESS, 0.0, 0.0)), TURN), shut, "turn the key to unlocked", fg),
        Waypoint(0.8, FacePos(servo, _v(FACE_TCP) + _v((PRESS, 0.0, 0.0)), TURN), shut, "hold it at its stop", fg),
        Waypoint(1.0, FacePos(servo, _v(FACE_TCP), None), face_open, "let go of the key", fg),
        Waypoint(1.0, FacePos(servo, face_back, None), face_open, "back off", fg),
    ]
    pg = PullGrip(servo)
    plan += [
        Waypoint(2.0, PullPos(servo, (0.030, 0.0, 0.0), None), pull_open, "over the turned key", pg),
        Waypoint(1.2, PullPos(servo, (0.0, 0.0, 0.0), None), pull_open, "round the bow's faces", pg),
        Waypoint(1.0, PullPos(servo, (0.0, 0.0, 0.0), None), shut, "pinch the bow", pg),
        Waypoint(0.1, PullPos(servo, (0.0, 0.0, 0.0), PULL_ON), shut, "draw the door open by the key", pg),
        Waypoint(3.4, PullPos(servo, (0.0, 0.0, 0.0), PULL_ON), shut, "draw the door open by the key", pg),
        # Let go: slide the pads off the bow, up and back along the grip's axis, as the fingers open.
        Waypoint(RELEASE_S, PullPos(servo, (RELEASE_BACK, RELEASE_UP, 0.0), None), _finger(0.060), "slide off and lift away: the door drops", pg),
        Waypoint(0.8, PullPos(servo, (0.090, 0.040, 0.0), None), _finger(0.060), "clear of the door", pg),
        Waypoint(2.0, RIGHT_PARK[0], _finger(0.06), "park", Grip(RIGHT_PARK[1])),
    ]
    return plan


# --- the routine's second half: the parcel out, the door shut, the key turned back, out and on its mat ------------------
# The parcel pinch (left hand): across the parcel's 60 mm width at its near end, the hand pitched PARCEL_PITCH down so
# that the fingers, which widen back from their tips, stay clear of the floor and the door's lip (official URDF meshes).
PARCEL_PITCH = 8.0
PARCEL_GRIP = aim((math.cos(math.radians(PARCEL_PITCH)), 0.0, -math.sin(math.radians(PARCEL_PITCH))), (0.0, -1.0, 0.0))
PARCEL_PAD_IN = 0.058  # the pads' middle this far behind the parcel's near end: over its centre of mass (on the GPU,
# pinched 40 mm in, it pivoted 17-23 degrees in the pads as it was drawn, its far end dragging on the floor, and slipped)
PARCEL_OPEN = 0.080
PARCEL_SQUEEZE = 0.030  # the fingers' target: 15 mm inside each face (the gripper's 50 N limit is the squeeze)
PARCEL_LIFT = 0.005  # off the floor while it is drawn out: the opening leaves 9 mm over the parcel's 86 mm, the door's
# hinge edge stands 3 mm over the floor. The servo (PARCEL_DRAW) holds it there by the parcel's measured height (on the GPU,
# commanded 5 mm, the arm sagged under the 0.24 kg and lifted it 0.6 mm; its near edge met the door's lip, the parcel
# tilted 10 degrees and the hand was pushed up 9 mm).
PARCEL_SAG_MAX = 0.016  # the servo's most extra lift (10.0-11.7 mm were needed on the GPU)
PARCEL_OUT = 0.165  # drawn out this far: its far end then 22 mm in front of the hopper door's face
PARCEL_RAISE = 0.130  # its underside then over the door's top edge as the door is shut
PARCEL_SET = 0.002  # set down from this high (it drops the rest)
PARCEL_HOLD_Y = 0.110  # held up here, clear of the right hand, while the door is shut


def parcel_tcp(center: Pt) -> np.ndarray:
    """The pinch's TCP with the parcel's centre at ``center`` (the parcel as it lies, not turned)."""
    pads = _v(center) - _v((PARCEL_SIZE[0] / 2 - PARCEL_PAD_IN, 0.0, 0.0))
    return np.asarray(pads - PAD_MID * PARCEL_GRIP[:, 2], dtype=float)


class ParcelPos(EdgePos):
    """A waypoint position for the parcel pinch: ``key_at`` is where the parcel's centre is to be, shifted by how far
    from its start the servo found the parcel lying."""

    def value(self) -> np.ndarray:
        self.servo.update()
        return parcel_tcp(self.key_at + self.servo.parcel_shift + _v((0.0, 0.0, self.servo.parcel_dz)))


# The door lifted shut (right hand, fingers closed, closing across y). The door lies open at 95 degrees, its free edge
# 71 mm over the table: the hand comes at it from above and in front, pointing down and forward at DOOR_PITCH, and
# slips its fingertips under the door; the door's outer corner rests on the fingers DOOR_SC back from their tips. The
# hand keeps that pitch and follows the edge up its arc (the servo leads the door by at most DOOR_LEAD: the door's
# 0.4 kg on the soft arm); near shut the fingertips push on the door's face below its top edge, the rest of the hand
# up and back, clear of the hopper door (a YAM finger widens about 0.4 mm per mm back from its tip, official meshes).
DOOR_Y = -0.055  # between the lock's key (y -0.10) and the middle
DOOR_PITCH = 45.0
DOOR_GRIP = aim((math.cos(math.radians(DOOR_PITCH)), 0.0, -math.sin(math.radians(DOOR_PITCH))), (0.0, -1.0, 0.0))
DOOR_SC_SHUT, DOOR_SC_OPEN = 0.006, 0.045  # the corner's distance back from the fingertips, at 20 deg and less, at 60 and
# more (in between, linear). Open, well back on the fingers' upper face, so the tips are under the door even with the
# soft arm 7 mm short of its target (GPU: 8 mm back, the tip's rounded end pushed the door's edge onto its stop; 20 and
# 35 mm back, the tips came up in front of the edge and passed it; 45 mm back, the door rose and shut in 6 s); near
# shut, nearer the tips, clear of the rail behind the door.
# A closed YAM finger pair's half-width across the closing, by distance back from the fingertips (official URDF meshes).
FINGER_TAPER = ((0.0, 0.0077), (0.008, 0.0110), (0.013, 0.0128), (0.023, 0.0169), (0.033, 0.0206), (0.043, 0.0246), (0.053, 0.0285), (0.068, 0.0340))


def _taper(s: float) -> float:
    return float(np.interp(s, [a for a, _ in FINGER_TAPER], [b for _, b in FINGER_TAPER]))


def door_edge(phi: float) -> np.ndarray:
    """The access door's outer free-edge corner (its top edge when shut) with the door ``phi`` open, at DOOR_Y."""
    hinge = _v(AD_HINGE)
    corner = _v((X_FRONT - AD_T, DOOR_Y, Z_BOT + AD_ZB[1]))
    return np.asarray(hinge + ry(-phi) @ (corner - hinge), dtype=float)


def door_sc(phi: float) -> float:
    return float(np.interp(phi, [20.0, 60.0], [DOOR_SC_SHUT, DOOR_SC_OPEN]))


def door_hand(phi: float, back: float = 0.0) -> np.ndarray:
    """The door-lifting hand's TCP with the door ``phi`` open: the door's corner on the fingers' upper face door_sc(phi)
    back from their tips; ``back`` drawn back along the hand's own axis."""
    a, up = DOOR_GRIP[:, 2], -DOOR_GRIP[:, 1]
    sc = door_sc(phi)
    tip = door_edge(phi) + sc * a - _taper(sc) * up
    return np.asarray(tip - (PAD_TIP_PAST_TCP + back) * a, dtype=float)


class DoorPos(EdgePos):
    """A waypoint position for the door lift: ``key_at`` holds (degrees more open than the servo's door angle, back
    along the hand's axis, 0)."""

    def value(self) -> np.ndarray:
        self.servo.update()
        return door_hand(self.servo.door_phi + float(self.key_at[0]), float(self.key_at[1]))


def left_plan(servo: KeyServo, start: float, door_shut: float) -> list[Waypoint]:
    """The left arm: watches from its park until ``start``, then takes the parcel out through the access opening and
    holds it up while the right hand shuts the door (until ``door_shut``), then sets it on its marked place."""
    park, park_grip, park_open = LEFT_PARK[0], Grip(LEFT_PARK[1]), _finger(0.07)
    pc = _v(PARCEL_CENTER)
    pg = Grip(PARCEL_GRIP)
    popen, squeeze = _finger(PARCEL_OPEN), _finger(PARCEL_SQUEEZE)
    lifted = pc + _v((0.0, 0.0, PARCEL_LIFT))
    out = lifted - _v((PARCEL_OUT, 0.0, 0.0))
    raised = out + _v((0.0, 0.0, PARCEL_RAISE))
    aside = _v((raised[0], PARCEL_HOLD_Y, raised[2]))
    place = _v((PARCEL_PLACE[0], PARCEL_PLACE[1], TOP + PARCEL_SIZE[2] / 2 + PARCEL_SET))
    plan = [
        Waypoint(3.0, park, park_open, "to its watching pose", park_grip),
        Waypoint(max(start - 3.0, 0.1), park, park_open, "watch", park_grip),
        Waypoint(2.5, ParcelPos(servo, pc - _v((0.070, 0.0, 0.0)), None), popen, "to the access opening", pg),
        Waypoint(0.1, ParcelPos(servo, pc - _v((0.070, 0.0, 0.0)), PARCEL_ON), popen, "watch the parcel", pg),
        Waypoint(2.0, ParcelPos(servo, pc, PARCEL_ON), popen, "in round the parcel's end", pg),
        Waypoint(1.0, ParcelPos(servo, pc, PARCEL_ON), squeeze, "pinch it", pg),
        Waypoint(0.6, ParcelPos(servo, lifted, None), squeeze, "lift it off the floor", pg),
        Waypoint(0.1, ParcelPos(servo, lifted, PARCEL_DRAW), squeeze, "watch its height", pg),
        Waypoint(1.0, ParcelPos(servo, lifted, PARCEL_DRAW), squeeze, "hold it 5 mm off the floor", pg),
        Waypoint(3.5, ParcelPos(servo, out, PARCEL_DRAW), squeeze, "draw it out through the opening", pg),
        Waypoint(1.5, ParcelPos(servo, raised, None), squeeze, "lift it clear", pg),
        Waypoint(1.2, ParcelPos(servo, aside, None), squeeze, "hold it aside", pg),
    ]
    held_until = sum(w.duration for w in plan)
    plan += [
        Waypoint(max(door_shut - held_until, 0.1), ParcelPos(servo, aside, None), squeeze, "hold it up while the door is shut", pg),
        Waypoint(2.0, ParcelPos(servo, place + _v((0.0, 0.0, 0.030)), None), squeeze, "over its place", pg),
        Waypoint(1.2, ParcelPos(servo, place, None), squeeze, "set it down", pg),
        Waypoint(0.8, ParcelPos(servo, place, None), popen, "let go", pg),
        Waypoint(1.2, ParcelPos(servo, place + _v((0.0, 0.0, 0.070)), None), popen, "lift away", pg),
        Waypoint(2.0, park, park_open, "park", park_grip),
    ]
    return plan


def door_plan(servo: KeyServo) -> list[Waypoint]:
    """The right arm: the access door lifted from its stop back into its catch (the cam is clear: the key is still at
    unlocked)."""
    dg, closed = Grip(DOOR_GRIP), _finger(0.0)
    return [
        Waypoint(2.5, DoorPos(servo, (12.0, 0.040, 0.0), DOOR_UNDER), closed, "to the door's free edge", dg),
        Waypoint(1.5, DoorPos(servo, (3.0, 0.0, 0.0), DOOR_UNDER), closed, "fingertips under its edge", dg),
        Waypoint(0.1, DoorPos(servo, (0.0, 0.0, 0.0), DOOR_UP), closed, "lift the door", dg),
        Waypoint(8.0, DoorPos(servo, (0.0, 0.0, 0.0), DOOR_UP), closed, "lift the door shut", dg),
        Waypoint(1.0, DoorPos(servo, (0.0, 0.040, 0.0), None), closed, "draw back from the door", dg),
    ]


def key_back_plan(servo: KeyServo) -> list[Waypoint]:
    """The right arm, once the door is shut: the face pinch on the turned key again, the quarter turn back to locked,
    the edge grip, the key drawn out (it comes out only at locked) and laid on its mat."""
    face_back = _v(FACE_TCP) - _v((0.030, 0.0, 0.0))
    press = _v(FACE_TCP) + _v((PRESS, 0.0, 0.0))
    fg = FaceGrip(servo, FACE)
    face_open, shut = _finger(FACE_OPEN), _finger(0.0)
    edge, squeeze = _finger(EDGE_OPEN), _finger(EDGE_SQUEEZE)
    held = _v(KEY_HOME_AT)
    on_mat = _v(KEY_CENTER) + _v((0.0, 0.0, 0.002))
    over_mat = _v(KEY_CENTER) + _v((0.0, 0.0, 0.050))
    e = {m: EdgeGrip(servo, m) for m in (None, REGRIP)}
    return [
        Waypoint(2.5, FacePos(servo, face_back, REGRIP_FACE), face_open, "to the turned key, level", fg),
        Waypoint(1.5, FacePos(servo, _v(FACE_TCP), REGRIP_FACE), face_open, "round the bow's faces", fg),
        Waypoint(1.0, FacePos(servo, _v(FACE_TCP), REGRIP_FACE), shut, "pinch the bow's faces", fg),
        Waypoint(0.5, FacePos(servo, press, None), shut, "press it home", fg),
        Waypoint(4.0, FacePos(servo, press, TURN_BACK), shut, "turn the key back to locked", fg),
        Waypoint(0.8, FacePos(servo, press, TURN_BACK), shut, "hold it at locked", fg),
        Waypoint(1.0, FacePos(servo, _v(FACE_TCP), None), face_open, "let go of the key", fg),
        Waypoint(1.0, FacePos(servo, face_back, None), face_open, "back off", fg),
        Waypoint(2.0, EdgePos(servo, held - _v((0.030, 0.0, -0.020)), None), edge, "turn the hand to the edge grip", e[None]),
        Waypoint(0.1, EdgePos(servo, held, REGRIP), edge, "watch the key", e[REGRIP]),
        Waypoint(1.5, EdgePos(servo, held, REGRIP), edge, "round the bow's edges", e[REGRIP]),
        Waypoint(1.0, EdgePos(servo, held, None), squeeze, "pinch the bow's edges", e[None]),
        Waypoint(2.0, EdgePos(servo, held - _v((0.030, 0.0, 0.0)), None), squeeze, "draw the key out", e[None]),
        Waypoint(1.5, EdgePos(servo, held - _v((0.060, 0.0, -0.030)), None), squeeze, "clear of the door", e[None]),
        Waypoint(2.5, EdgePos(servo, over_mat, None), squeeze, "carry it to its mat", e[None]),
        Waypoint(1.5, EdgePos(servo, on_mat, None), squeeze, "lay it on its mat", e[None]),
        Waypoint(0.8, EdgePos(servo, on_mat, None), edge, "let go", e[None]),
        Waypoint(1.0, EdgePos(servo, over_mat, None), edge, "lift away", e[None]),
        Waypoint(2.0, RIGHT_PARK[0], _finger(0.06), "park", Grip(RIGHT_PARK[1])),
    ]


def _pad(plan: list[Waypoint], until: float, label: str) -> list[Waypoint]:
    total = sum(w.duration for w in plan)
    if until > total + 1e-9:
        last = plan[-1]
        plan = [*plan, Waypoint(until - total, last.pos, last.finger, label, last.yaw)]
    return plan


def _until(plan: Sequence[Waypoint], label: str) -> float:
    """The time at which the last waypoint labelled ``label`` ends."""
    t, end = 0.0, 0.0
    for w in plan:
        t += float(w.duration)
        if w.label == label:
            end = t
    return end


END_HOLD = 2.0
LEFT_SERVO: KeyServo | None = None


def demo() -> dict[str, list[Waypoint]]:
    global SERVO, LEFT_SERVO
    install_grip_rotation()
    servo, lservo = KeyServo(), KeyServo()
    right = key_plan(servo)
    first = sum(w.duration for w in right)
    out = _until(left_plan(lservo, first, 0.0), "hold it aside")  # the parcel is out and held up aside
    right = _pad(right, out, "wait for the parcel") + door_plan(servo)
    shut = sum(w.duration for w in right)
    left = left_plan(lservo, first, shut)
    right = right + key_back_plan(servo)
    end = max(sum(w.duration for w in right), sum(w.duration for w in left)) + END_HOLD
    right, left = _pad(right, end, "hold still"), _pad(left, end, "hold still")
    servo.follow(right)
    lservo.follow(left)
    SERVO, LEFT_SERVO = servo, lservo
    return {"left": left, "right": right}


def _simulator_present() -> bool:
    try:
        return importlib.util.find_spec("isaaclab") is not None
    except (ImportError, ValueError):
        return False


if _simulator_present():
    try:
        install_hook()
    except Exception as error:  # never break loading the task over the hook; the scene then scores 0
        _warn_once("hook", f"could not install the scene hook ({type(error).__name__}: {error})")
