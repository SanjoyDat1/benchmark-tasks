"""Three black curbside mailboxes stand side by side on the table in front of the arms, their doors toward the robot:
standard post-mount mailboxes of the classic arched shape (Elite type, 6.92 in wide, 8.88 in tall, 20.05 in deep,
galvanized steel), bolted 11.3 in apart on a 34 in three-box steel spreader bar that sits on a short 4x4 post on a
plywood base. Each mailbox has a front door hinged along its bottom edge, held shut by a latch at its top; a small
pull tab sticks out from the top of the door. Unlatched, a door falls open toward the robot and rests on its stop,
tilted down past level. On each mailbox's right side (as the robot sees it), near the front, a red carrier signal
flag pivots on a bracket: down, it lies along the side pointing to the back; up, it stands straight up above the
mailbox.

The middle mailbox has its red flag up: the customer has left a letter in it to be collected. That letter, a 5 x 7
in greeting card in a white A7 envelope (18.4 x 13.3 cm), lies flat on the floor of that mailbox, just inside the
door. The other two mailboxes, left and right of it, are shut with their flags down. A black plastic letter tray (the
mail tray, 31.8 x 24.8 cm, 5.6 cm deep) sits on the table between the arms, nearer the robot than the mailboxes.

The scene is fixed: it is always the middle mailbox that is flagged. The two side mailboxes look the same and are
there to be left alone (opening a mailbox whose flag is down scores 0).
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
Quat = tuple[float, float, float, float]
Pt = Sequence[float] | np.ndarray

# --- the layout: the middle mailbox has its flag up (a fixed scene) -------------------------------------------------
BOXES = ("left", "middle", "right")  # as the robot sees them: left is +y
FLAGGED = "middle"

# --- the mailboxes (world frame: x away from the arms, y to their left, z up; metres) ------------------------------
# Outer size from the maker's page (8.88 in tall, 6.92 in wide, 20.05 in deep). The arch is drawn as a half circle
# over straight sides (USPS-STD-7C Fig. 1A: a traditional box's arch radius is 3.12-5.50 in). Every inner dimension
# is an estimate (README, "Sources").
MB_W = 6.92 * INCH  # 0.176 (y)
MB_H = 8.88 * INCH  # 0.226 (z)
MB_D = 20.05 * INCH  # 0.509 (x)
ARCH_R = MB_W / 2
SIDE_H = MB_H - ARCH_R  # the straight sides' height, to where the arch springs
X_FRONT = 0.380  # the boxes' front edge (each door's inner face, shut)
SPACING = 34.0 / 3.0 * INCH  # centre to centre: a 34 in three-box spreader bar (derived: 34 / 3; to measure)
Y_OF = {"left": SPACING, "middle": 0.0, "right": -SPACING}
WALL = 0.004  # collider thickness of the walls (inward from the outer face; the steel is under 1 mm)
FLOOR_T = 0.004  # the floor's collider (its top is the inside floor)
# The bench stand-in for the post (the real boxes stand 41-45 in above the road): a 3/4 in plywood base clamped to the
# table, a 4x4 post offcut and the spreader bar; the boxes' bottoms then stand 18 cm above the table, so an open door
# (115 degrees) hangs clear of it.
BASE_PLY = 0.019
SPREADER = (7.25 * INCH, 34.0 * INCH, 1.0 * INCH)  # x (depth), y (length), z (height): Salsbury 4883 listing
Z_BOT = TOP + 0.180  # each box's outer bottom
POST_H = Z_BOT - SPREADER[2] - TOP - BASE_PLY
FLOOR_TOP = Z_BOT + 0.0015  # the inside floor
# The door: a pressed steel panel the box's front outline plus a 2 mm rim, its flange 6 mm deep; hinged along its
# bottom edge; a latch tab at the top. USPS-STD-7C §3.3: opens with no more than 5 lb, a positive mechanical latch, not
# spring-loaded, stays open until pushed shut, opens at least 100 degrees (120 or less preferred).
DOOR_T = 0.006
DOOR_RIM = 0.002
HINGE_DROP = 0.003  # the hinge pin sits this far under the box's bottom edge (the door's rolled edge wraps it), so an open
# door's face lies under the floor's edge and mail slides out over it
DOOR_MASS = 0.28  # kg (estimate: 0.8 mm steel over the door's area plus its flange)
DOOR_OPEN = 115.0  # degrees: where it rests open (estimate within STD-7C's 100-120; to measure)
DOOR_DAMPING = 0.0008  # N m s per degree: the hinge's drag (estimate)
CATCH_DEG = 3.0  # within this of shut the latch's hook is over the lip: it pulls the door shut and holds it
CATCH_TORQUE = 1.0  # N m: about 4.4 N (1 lb) at the tab to pull it open: STD-7C's lowest allowed (estimate, to measure)
CATCH_K = CATCH_TORQUE / CATCH_DEG  # N m per degree
TAB = (0.015, 0.030, 0.003)  # the pull tab: sticks out 15 mm from the door's face, 30 mm wide, 3 mm (folded) thick
# The flag: a 16 x 3 mm red steel arm on a pivot 30 mm behind the front (STD-7C: within 2 in), 95 mm (about 3.75 in, the
# low end of STD-7C's 3.75-8.50 in) above the box's bottom, 190 mm long, with a 57 x 51 mm signal plate at its end
# (above the box when up).
FLAG_X = X_FRONT + 0.030
FLAG_DZ = 0.095
FLAG_OFF = 0.008  # its arm's middle, out from the side
FLAG_LEN, FLAG_BAR, FLAG_T = 0.190, 0.016, 0.003
FLAG_HEAD = (0.057, 0.051)  # along the arm, across it
FLAG_MASS = 0.06  # kg (estimate)
FLAG_K, FLAG_TORQUE = 0.03, 0.35  # N m per degree, N m: its detent (STD-7C: 2 lb or less to lower it; estimate)
FLAG_DAMPING = 0.0004
FLAG_SWITCH = 45.0  # degrees: past this the detent carries it on
FLAG_UP_START = 90.0

# --- the loose things -------------------------------------------------------------------------------------------------
# The letter: a 5 x 7 in greeting card in an A7 envelope (7-1/4 x 5-1/4 in); 2 mm and 25 g are estimates. It lies flat,
# face up, long side along the mailbox, centred, its near edge LETTER_IN behind the door's inner face.
LETTER = (7.25 * INCH, 5.25 * INCH, 0.002)
LETTER_MASS = 0.025
LETTER_IN = 0.025
# The mail tray: Officemate 21040 stackable letter tray (maker's drawing 12.5 x 9.75 x 2.2 in), RPET plastic; its long
# side along y; a low front where papers slide in, toward the robot.
TRAY = (9.75 * INCH, 12.5 * INCH, 2.2 * INCH)  # x, y, z
TRAY_WALL, TRAY_FLOOR, TRAY_FRONT_H = 0.003, 0.003, 0.025
TRAY_MASS = 0.43  # kg (the maker lists 1.90 lb for two)
TRAY_AT = (-0.050, 0.0)  # its centre on the table: clear of where an open door rests, and of the hand lifting it

# --- colours (sRGB) and friction (static, dynamic) ------------------------------------------------------------------
BLACK = (0.060, 0.062, 0.068)
FRAME = (0.110, 0.113, 0.120)
EDGE_LINE = (0.30, 0.31, 0.33)
DARK = (0.02, 0.02, 0.022)
RED = (0.82, 0.07, 0.05)
GALV = (0.62, 0.64, 0.66)
POST_WOOD = (0.60, 0.47, 0.32)
PLY = (0.83, 0.71, 0.52)
ENVELOPE = (0.96, 0.95, 0.92)
STAMP = (0.20, 0.42, 0.75)
INK = (0.30, 0.30, 0.33)
TRAY_RGB = (0.10, 0.10, 0.11)
STEEL_F = (0.40, 0.35)
FLOOR_F = (0.30, 0.25)  # galvanized sheet under paper (estimate); combined by "min" so paper slides on it
PAPER_F = (0.60, 0.55)
PLASTIC_F = (0.45, 0.40)
WOOD_F = (0.50, 0.45)


def _t(values: Any) -> Vec3:
    return (float(values[0]), float(values[1]), float(values[2]))


def _v(values: Any) -> np.ndarray:
    return np.asarray(values, dtype=float)


def box_name(b: str) -> str:
    return f"mailbox_{b}"


def door_name(b: str) -> str:
    return f"door_{b}"


def flag_name(b: str) -> str:
    return f"flag_{b}"


STAND, LETTER_N, TRAY_N = "stand", "letter", "tray"


def raised(b: str) -> bool:
    return b == FLAGGED


def box_center(b: str) -> Vec3:
    return (X_FRONT + MB_D / 2, Y_OF[b], FLOOR_TOP - FLOOR_T / 2)


def door_center(b: str) -> Vec3:
    return (X_FRONT - DOOR_T / 2, Y_OF[b], Z_BOT + SIDE_H / 2)


def flag_pivot(b: str) -> Vec3:
    return (FLAG_X, Y_OF[b] - MB_W / 2 - FLAG_OFF, Z_BOT + FLAG_DZ)


FLAG_BOSS = (0.016, FLAG_T + 0.004, 0.016)
LETTER_CENTER = (X_FRONT + LETTER_IN + LETTER[0] / 2, Y_OF[FLAGGED], FLOOR_TOP + LETTER[2] / 2)
TRAY_CENTER = (TRAY_AT[0], TRAY_AT[1], TOP + TRAY_FLOOR / 2)
STAND_CENTER = (X_FRONT + MB_D / 2, 0.0, TOP + BASE_PLY / 2)
PLY_SIZE = (0.300, 0.300, BASE_PLY)


def _box(name: str, center: Pt, size: Pt, rgb: Pt, mass: float | None = None) -> Box:
    return Box(name, pos=_t(center), size=_t(size), color=_t(rgb), mass=mass)


def _objects() -> list[Box]:
    out = [_box(STAND, STAND_CENTER, PLY_SIZE, PLY)]
    for b in BOXES:
        out.append(_box(box_name(b), box_center(b), (MB_D, MB_W, FLOOR_T), FRAME))
        out.append(_box(door_name(b), door_center(b), (DOOR_T, MB_W + 2 * DOOR_RIM, SIDE_H), BLACK, DOOR_MASS))
        out.append(_box(flag_name(b), flag_pivot(b), FLAG_BOSS, RED, FLAG_MASS))
    out.append(_box(LETTER_N, LETTER_CENTER, LETTER, ENVELOPE, LETTER_MASS))
    out.append(_box(TRAY_N, TRAY_CENTER, (TRAY[0], TRAY[1], TRAY_FLOOR), TRAY_RGB, TRAY_MASS))
    return out


OBJECTS = _objects()
SPECS = {o.name: o for o in OBJECTS}
DOORS = tuple(door_name(b) for b in BOXES)
FLAGS = tuple(flag_name(b) for b in BOXES)
FIXED = (STAND, *(box_name(b) for b in BOXES))
FREE = (*DOORS, *FLAGS, LETTER_N, TRAY_N)


# --- geometry as data: every shape of every body, in the body's own frame -------------------------------------------
@dataclass(frozen=True, eq=False)
class Part:
    """One shape under a body: a box (``size``), a cylinder (``radius``, ``height`` along ``axis``) or a triangle mesh
    (``points``, ``tris``); its look (``look``: a key of LOOKS); whether it collides; ``hidden``: a collider that is
    not drawn (inside a drawn mesh)."""

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
    hidden: bool = False

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


LOOKS: dict[str, tuple[Vec3, float, float]] = {
    "powder": (BLACK, 0.25, 0.42),
    "frame": (FRAME, 0.25, 0.45),
    "edge": (EDGE_LINE, 0.6, 0.30),
    "dark": (DARK, 0.0, 0.9),
    "red": (RED, 0.1, 0.40),
    "galv": (GALV, 0.9, 0.40),
    "post": (POST_WOOD, 0.0, 0.8),
    "ply": (PLY, 0.0, 0.75),
    "envelope": (ENVELOPE, 0.0, 0.85),
    "stamp": (STAMP, 0.0, 0.7),
    "ink": (INK, 0.0, 0.8),
    "tray": (TRAY_RGB, 0.0, 0.6),
    "steel": ((0.16, 0.17, 0.18), 0.6, 0.45),
}
FRICTION: dict[str, tuple[float, float]] = {STAND: WOOD_F, LETTER_N: PAPER_F, TRAY_N: PLASTIC_F}
FRICTION.update({box_name(b): FLOOR_F for b in BOXES})
FRICTION.update({door_name(b): STEEL_F for b in BOXES})
FRICTION.update({flag_name(b): STEEL_F for b in BOXES})
COMBINE_MIN = {box_name(b) for b in BOXES}  # the boxes' galvanized steel: the lower friction of a pair wins


def _bx(name: str, lo: Pt, hi: Pt, look: str, collides: bool = False, hidden: bool = False) -> Part:
    lo_v, hi_v = _v(lo), _v(hi)
    return Part(name, "box", _t((lo_v + hi_v) / 2), look, collides, size=_t(hi_v - lo_v), hidden=hidden)


def _cy(name: str, center: Pt, radius: float, height: float, axis: str, look: str, collides: bool = False) -> Part:
    return Part(name, "cyl", _t(center), look, collides, radius=radius, height=height, axis=axis)


def _mesh(name: str, points: np.ndarray, tris: np.ndarray, look: str) -> Part:
    return Part(name, "mesh", (0.0, 0.0, 0.0), look, False, points=points, tris=tris)


def arch_band(y0: float, z0: float, r_in: float, r_out: float, x0: float, x1: float, n: int = 24) -> tuple[np.ndarray, np.ndarray]:
    """A closed mesh: the half ring r_in..r_out about the line (y0, z0) along x, over the top (y from +r to -r),
    from x0 to x1. With r_in = 0 it is a half disc."""
    pts: list[tuple[float, float, float]] = []
    for i in range(n + 1):
        t = math.pi * i / n
        c, s = math.cos(t), math.sin(t)
        for x in (x0, x1):
            pts.append((x, y0 + r_in * c, z0 + r_in * s))
            pts.append((x, y0 + r_out * c, z0 + r_out * s))
    points = np.asarray(pts)

    def idx(i: int, xi: int, side: int) -> int:
        return i * 4 + xi * 2 + side

    tris: list[list[int]] = []
    for i in range(n):
        j = i + 1
        tris += [[idx(i, 0, 1), idx(j, 0, 1), idx(j, 1, 1)], [idx(i, 0, 1), idx(j, 1, 1), idx(i, 1, 1)]]  # outer
        tris += [[idx(i, 0, 0), idx(j, 1, 0), idx(j, 0, 0)], [idx(i, 0, 0), idx(i, 1, 0), idx(j, 1, 0)]]  # inner
        tris += [[idx(i, 1, 0), idx(i, 1, 1), idx(j, 1, 1)], [idx(i, 1, 0), idx(j, 1, 1), idx(j, 1, 0)]]  # x1 face
        tris += [[idx(i, 0, 0), idx(j, 0, 1), idx(i, 0, 1)], [idx(i, 0, 0), idx(j, 0, 0), idx(j, 0, 1)]]  # x0 face
    for i in (0, n):  # the two flat ends (where the band meets the sides)
        tris += [[idx(i, 0, 0), idx(i, 0, 1), idx(i, 1, 1)], [idx(i, 0, 0), idx(i, 1, 1), idx(i, 1, 0)]]
    return points, np.asarray(tris, dtype=np.int64)


def _local(center: Pt, body: Pt) -> np.ndarray:
    return np.asarray(_v(center) - _v(body), dtype=float)


ARCH_STEPS = 7


def arch_steps(r: float, z_spring: float, y0: float) -> list[tuple[float, float, float]]:
    """Horizontal slabs inside a half circle of radius r springing at z_spring: (z_low, z_high, half width at z_high)."""
    out = []
    for k in range(ARCH_STEPS):
        z0 = z_spring + r * k / ARCH_STEPS
        z1 = z_spring + r * (k + 1) / ARCH_STEPS
        half = math.sqrt(max(r * r - (z1 - z_spring) ** 2, 0.0))
        out.append((z0, z1, max(half, 0.004)))
    return out


def mailbox_parts(b: str) -> list[Part]:
    """A mailbox (its frame: the floor slab's centre). Colliders: the sides, the back, the arch (slabs, hidden under
    the drawn arch); looks: the arch, the galvanized inside, the front bead, the flag's bracket."""
    c = _v(box_center(b))
    yc = Y_OF[b]
    x0, x1 = X_FRONT, X_FRONT + MB_D
    yl, yr = yc + MB_W / 2, yc - MB_W / 2
    zs = Z_BOT + SIDE_H
    parts: list[Part] = []

    def slab(name: str, lo: Pt, hi: Pt, look: str = "frame", collides: bool = True, hidden: bool = False) -> None:
        parts.append(_bx(name, _local(lo, c), _local(hi, c), look, collides, hidden))

    slab("side_left", (x0, yl - WALL, Z_BOT), (x1, yl, zs))
    slab("side_right", (x0, yr, Z_BOT), (x1, yr + WALL, zs))
    slab("back", (x1 - WALL, yr, Z_BOT), (x1, yl, Z_BOT + MB_H))
    # The arch's colliders: slabs between the arch's inner surface and its outer surface at each slab's top (inside the
    # drawn shell; their inner corners show a little inside the box's roof). They are drawn: GPU runs B and C suggest an
    # invisible collider does not collide on this stack.
    for k, (z0, z1, half) in enumerate(arch_steps(ARCH_R - WALL, zs, yc)):
        outer = math.sqrt(max(ARCH_R * ARCH_R - (z1 - zs) ** 2, 0.0))  # inside the drawn shell at the step's top
        if half > 0.006 and outer > half + 0.001:
            slab(f"arch_l{k}", (x0, yc + half, z0), (x1, yc + outer, z1), "frame", True)
            slab(f"arch_r{k}", (x0, yc - outer, z0), (x1, yc - half, z1), "frame", True)
    slab("arch_top", (x0, yc - 0.010, Z_BOT + MB_H - WALL), (x1, yc + 0.010, Z_BOT + MB_H), "frame", True)
    pts, tris = arch_band(yc - c[1], zs - c[2], ARCH_R - 0.001, ARCH_R, x0 - c[0], x1 - c[0])
    parts.append(_mesh("arch", pts, tris, "frame"))
    pts, tris = arch_band(yc - c[1], zs - c[2], ARCH_R - WALL - 0.0005, ARCH_R - WALL, x0 + 0.004 - c[0], x1 - WALL - c[0])
    parts.append(_mesh("arch_inside", pts, tris, "galv"))
    # The galvanized inside (looks): floor, sides and back.
    slab("floor_inside", (x0 + 0.003, yr + WALL, FLOOR_TOP - 0.0003), (x1 - WALL, yl - WALL, FLOOR_TOP), "galv", False)
    slab("liner_left", (x0 + 0.003, yl - WALL - 0.0005, FLOOR_TOP), (x1 - WALL, yl - WALL, zs), "galv", False)
    slab("liner_right", (x0 + 0.003, yr + WALL, FLOOR_TOP), (x1 - WALL, yr + WALL + 0.0005, zs), "galv", False)
    slab("liner_back", (x1 - WALL - 0.0005, yr + WALL, FLOOR_TOP), (x1 - WALL, yl - WALL, zs), "galv", False)
    # The front bead the latch hooks over (looks): a rolled lip round the front edge.
    pts, tris = arch_band(yc - c[1], zs - c[2], ARCH_R - 0.0005, ARCH_R + 0.0025, x0 + 0.006 - c[0], x0 + 0.011 - c[0])
    parts.append(_mesh("bead_arch", pts, tris, "powder"))
    slab("bead_left", (x0 + 0.006, yl, Z_BOT), (x0 + 0.011, yl + 0.0025, zs), "powder", False)
    slab("bead_right", (x0 + 0.006, yr - 0.0025, Z_BOT), (x0 + 0.011, yr, zs), "powder", False)
    # The flag's bracket (looks): a small black plate on the right side, a bolt through it.
    px, py, pz = flag_pivot(b)
    slab("flag_bracket", (px - 0.020, yr - 0.004, pz - 0.022), (px + 0.020, yr, pz + 0.014), "powder", False)
    parts.append(_cy("flag_bolt", _local((px, yr - 0.005, pz), c), 0.004, 0.003, "Y", "galv"))
    # The bottom flange where it bolts to the spreader (looks).
    slab("flange", (x0 + 0.02, yr - 0.004, Z_BOT), (x1 - 0.02, yl + 0.004, Z_BOT + 0.003), "powder", False)
    return parts


def door_parts(b: str) -> list[Part]:
    """A door (its frame: the panel's lower rectangle's centre, shut). Its own box is the panel below the arch; the
    arch is a drawn half disc with hidden slabs inside it as colliders; the pull tab collides; the flange round the
    edge, the hinge knuckles and an embossed line are looks."""
    c = _v(door_center(b))
    yc = Y_OF[b]
    xa, xb = X_FRONT - DOOR_T, X_FRONT
    zs = Z_BOT + SIDE_H
    r = ARCH_R + DOOR_RIM
    parts: list[Part] = []
    for k, (z0, z1, half) in enumerate(arch_steps(r, zs, yc)):
        lo = _local((xa + 0.0004, yc - half, z0), c)
        hi = _local((xb - 0.0004, yc + half, z1), c)
        parts.append(_bx(f"arch_{k}", lo, hi, "powder", True))  # inside the drawn half disc: not seen
    pts, tris = arch_band(yc - c[1], zs - c[2], 0.0, r, xa - c[0], xb - c[0])
    parts.append(_mesh("arch", pts, tris, "powder"))
    # The flange: the door's edge turned back over the box (looks).
    pts, tris = arch_band(yc - c[1], zs - c[2], ARCH_R + 0.0003, r, xb - c[0], xb + 0.009 - c[0])
    parts.append(_mesh("flange_arch", pts, tris, "powder"))
    hw = MB_W / 2
    parts.append(_bx("flange_left", _local((xb, yc + hw + 0.0003, Z_BOT), c), _local((xb + 0.009, yc + hw + DOOR_RIM, zs), c), "powder"))
    parts.append(_bx("flange_right", _local((xb, yc - hw - DOOR_RIM, Z_BOT), c), _local((xb + 0.009, yc - hw - 0.0003, zs), c), "powder"))
    # An embossed line round the face (looks), 12 mm in from the edge.
    pts, tris = arch_band(yc - c[1], zs - c[2], r - 0.0135, r - 0.012, xa - 0.0003 - c[0], xa - c[0])
    parts.append(_mesh("emboss_arch", pts, tris, "edge"))
    for side, y in (("left", yc + r - 0.0135), ("right", yc - r + 0.012)):
        parts.append(_bx(f"emboss_{side}", _local((xa - 0.0003, y, Z_BOT + 0.012), c), _local((xa, y + 0.0015, zs), c), "edge"))
    parts.append(_bx("emboss_bottom", _local((xa - 0.0003, yc - r + 0.012, Z_BOT + 0.012), c), _local((xa, yc + r - 0.012, Z_BOT + 0.0135), c), "edge"))
    # The pull tab at the top (collides): the latch's folded tab sticking out of the face.
    ztop = zs + r
    parts.append(_bx("tab", _local((xa - TAB[0], yc - TAB[1] / 2, ztop - TAB[2]), c), _local((xa, yc + TAB[1] / 2, ztop), c), "powder", True))
    # The hinge's knuckles along the bottom (looks).
    for k, dy in enumerate((-0.06, 0.0, 0.06)):
        parts.append(_cy(f"knuckle_{k}", _local((X_FRONT - DOOR_T / 2, yc + dy, Z_BOT), c), 0.003, 0.025, "Y", "powder"))
    return parts


def flag_parts(b: str) -> list[Part]:
    """A flag (its frame: the pivot). Its own box is the pivot boss; the arm and the signal plate collide. Built as it
    starts: up (arm along +z, the plate toward the back) on the flagged mailbox, down (arm along +x, the plate hanging
    below its end) on the others; the joint's frame makes 'down' its zero either way."""
    hl, hw = FLAG_HEAD
    t, bar, n = FLAG_T / 2, FLAG_BAR / 2, FLAG_LEN
    if raised(b):
        return [
            _bx("arm", (-bar, -t, 0.0), (bar, t, n - hl), "red", True),
            _bx("plate", (-bar, -t, n - hl), (hw - bar, t, n), "red", True),
        ]
    return [
        _bx("arm", (0.0, -t, -bar), (n - hl, t, bar), "red", True),
        _bx("plate", (n - hl, -t, -(hw - bar)), (n, t, bar), "red", True),
    ]


def letter_parts() -> list[Part]:
    """The envelope (looks on its top face): a stamp, the address lines and a return address."""
    lx, ly, lz = LETTER
    z0, z1 = lz / 2, lz / 2 + 0.0002
    parts = [_bx("stamp", (lx / 2 - 0.034, ly / 2 - 0.030, z0), (lx / 2 - 0.010, ly / 2 - 0.008, z1), "stamp")]
    for k in range(3):  # the address, across the middle (the envelope read with its long side left to right)
        y = 0.010 - 0.010 * k
        parts.append(_bx(f"address_{k}", (-0.040 + 0.004 * k, y - 0.0012, z0), (0.035 - 0.012 * k, y + 0.0012, z1), "ink"))
    for k in range(2):  # the return address, top left
        y = ly / 2 - 0.012 - 0.007 * k
        parts.append(_bx(f"return_{k}", (-lx / 2 + 0.010, y - 0.0008, z0), (-lx / 2 + 0.045 - 0.008 * k, y + 0.0008, z1), "ink"))
    return parts


def tray_parts() -> list[Part]:
    """The letter tray: its floor is the body's own box; the back, the sides and the low front collide."""
    tx, ty, tz = TRAY
    w, f = TRAY_WALL, TRAY_FLOOR
    z0 = -f / 2
    return [
        _bx("back", (tx / 2 - w, -ty / 2, z0), (tx / 2, ty / 2, z0 + tz), "tray", True),
        _bx("side_left", (-tx / 2, ty / 2 - w, z0), (tx / 2, ty / 2, z0 + tz), "tray", True),
        _bx("side_right", (-tx / 2, -ty / 2, z0), (tx / 2, -ty / 2 + w, z0 + tz), "tray", True),
        _bx("front", (-tx / 2, -ty / 2, z0), (-tx / 2 + w, ty / 2, z0 + TRAY_FRONT_H), "tray", True),
    ]


def stand_parts() -> list[Part]:
    """The bench stand-in for the post (its frame: the plywood base's centre): a 4x4 post offcut and the three-box
    spreader bar (collide), the bolts (looks)."""
    c = _v(STAND_CENTER)
    sx, sy, sz = SPREADER
    xm = X_FRONT + MB_D / 2
    top = TOP + BASE_PLY
    parts = [
        _bx("post", _local((xm - 0.0445, -0.0445, top), c), _local((xm + 0.0445, 0.0445, top + POST_H), c), "post", True),
        _bx("spreader", _local((xm - sx / 2, -sy / 2, top + POST_H), c), _local((xm + sx / 2, sy / 2, Z_BOT), c), "steel", True),
    ]
    for b in BOXES:
        for k, dx in enumerate((-0.06, 0.06)):
            parts.append(_cy(f"bolt_{b}_{k}", _local((xm + dx, Y_OF[b], top + POST_H - 0.002), c), 0.006, 0.004, "Z", "galv"))
    return parts


def _parts_of(name: str) -> list[Part]:
    for b in BOXES:
        if name == box_name(b):
            return mailbox_parts(b)
        if name == door_name(b):
            return door_parts(b)
        if name == flag_name(b):
            return flag_parts(b)
    return {LETTER_N: letter_parts, TRAY_N: tray_parts, STAND: stand_parts}[name]()


def _parts_fn(name: str) -> Callable[[], list[Part]]:
    def parts() -> list[Part]:
        return _parts_of(name)

    return parts


PARTS: dict[str, Callable[[], list[Part]]] = {o.name: _parts_fn(o.name) for o in OBJECTS}


def local_outline(name: str) -> tuple[np.ndarray, np.ndarray]:
    """A body's outline in its own frame (low and high corners): its own box and every part the hook adds."""
    half = _v(SPECS[name].size) / 2
    lows, highs = [-half], [half]
    for part in PARTS[name]():
        lo, hi = part.bounds()
        lows.append(lo)
        highs.append(hi)
    return np.min(lows, axis=0), np.max(highs, axis=0)


# --- joints (frames in each body's own frame) -------------------------------------------------------------------------
@dataclass(frozen=True)
class JointSpec:
    """A revolute joint from body0 to body1 at a point (world, as built) about an axis; limits in degrees; its drive.
    ``rot1``: the joint's frame in body1 (w, x, y, z); a flag built standing up has its frame turned so that its zero is
    still 'down'."""

    name: str
    body0: str
    body1: str
    point: Vec3
    axis: str
    limits: tuple[float, float]
    stiffness: float
    damping: float
    target: float
    max_force: float
    rot1: Quat = (1.0, 0.0, 0.0, 0.0)


FLAG_UP_ROT1: Quat = (math.cos(math.radians(-45.0)), 0.0, math.sin(math.radians(-45.0)), 0.0)  # -90 deg about y


def _joints() -> tuple[JointSpec, ...]:
    out = []
    for b in BOXES:
        hinge = (X_FRONT - DOOR_T / 2, Y_OF[b], Z_BOT - HINGE_DROP)
        out.append(JointSpec(f"hinge_{b}", box_name(b), door_name(b), hinge, "Y", (-DOOR_OPEN, 0.0), CATCH_K, DOOR_DAMPING, 0.0, CATCH_TORQUE))
        up = raised(b)
        out.append(
            JointSpec(
                f"flag_{b}_pivot", box_name(b), flag_name(b), flag_pivot(b), "Y", (-90.0, 0.0), FLAG_K, FLAG_DAMPING,
                -90.0 if up else 0.0, FLAG_TORQUE, FLAG_UP_ROT1 if up else (1.0, 0.0, 0.0, 0.0),
            )
        )
    return tuple(out)


JOINTS = _joints()
JOINT_OF = {j.body1: j for j in JOINTS}  # each joint is built with its body1 (spawned after its body0)
CENTERS = {o.name: _t(o.pos) for o in OBJECTS}


def joint_frames(spec: JointSpec) -> tuple[Vec3, Vec3]:
    """The joint's frame origin in body0's and in body1's own frame."""
    return _t(_v(spec.point) - _v(CENTERS[spec.body0])), _t(_v(spec.point) - _v(CENTERS[spec.body1]))


# --- the scene hook: what the task-file API can't express -------------------------------------------------------
# yam_arena.scene.task_object(spec) turns each task object into an Arena object. Right after that module is imported,
# this wraps task_object, matching this file's own object specs by value, so that after Arena spawns one its spawner
# also builds that body's parts (PARTS: colliders and looks, each Gprim with its extent) and the joint that hangs it
# on the body spawned before it (JOINTS). On a machine without the simulator nothing imports yam_arena.scene, and this
# does nothing. What was built is recorded on the scene module; scoring gives 0 unless every part was built.
_SCENE_MODULE = "yam_arena.scene"
PARTS_FOLDER = "parts"
JOINTS_FOLDER = "mailbox_joints"
HOOK_TAG = "collect_flagged_mailbox_letter_parts"
BUILD_RECORD = "collect_flagged_mailbox_letter_build"
LETTER_CONTACT_OFFSET = 0.0008
LETTER_REST_OFFSET = 0.0
SOLVER = (32, 4)


def own_look(name: str) -> str:
    if name in DOORS:
        return "powder"
    if name in FLAGS:
        return "red"
    return {STAND: "ply", LETTER_N: "envelope", TRAY_N: "tray"}.get(name, "galv")


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


def _material(px: _Pxr, stage: Any, path: str, look: str, friction: tuple[float, float], combine_min: bool) -> Any:
    rgb, metallic, roughness = LOOKS[look]
    material = px.shade.Material.Define(stage, path)
    shader = px.shade.Shader.Define(stage, f"{path}/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    floats = px.sdf.ValueTypeNames.Float
    shader.CreateInput("diffuseColor", px.sdf.ValueTypeNames.Color3f).Set(px.gf.Vec3f(*(_linear(c) for c in rgb)))
    shader.CreateInput("metallic", floats).Set(float(metallic))
    shader.CreateInput("roughness", floats).Set(float(roughness))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    prim = material.GetPrim()
    api = px.physics.MaterialAPI.Apply(prim)
    api.CreateStaticFrictionAttr(float(friction[0]))
    api.CreateDynamicFrictionAttr(float(friction[1]))
    api.CreateRestitutionAttr(0.0)
    if combine_min:
        prim.AddAppliedSchema("PhysxMaterialAPI")
        prim.CreateAttribute("physxMaterial:frictionCombineMode", px.sdf.ValueTypeNames.Token).Set("min")
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


def _thin(px: _Pxr, prim: Any) -> None:
    prim.AddAppliedSchema("PhysxCollisionAPI")
    floats = px.sdf.ValueTypeNames.Float
    prim.CreateAttribute("physxCollision:contactOffset", floats).Set(LETTER_CONTACT_OFFSET)
    prim.CreateAttribute("physxCollision:restOffset", floats).Set(LETTER_REST_OFFSET)


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
    if part.hidden:
        gprim.CreateVisibilityAttr(px.geom.Tokens.invisible)
    return path


def _tune_body(px: _Pxr, prim: Any) -> None:
    prim.AddAppliedSchema("PhysxRigidBodyAPI")
    ints = px.sdf.ValueTypeNames.Int
    prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", ints).Set(SOLVER[0])
    prim.CreateAttribute("physxRigidBody:solverVelocityIterationCount", ints).Set(SOLVER[1])


def _drive(px: _Pxr, prim: Any, spec: JointSpec) -> None:
    drive = px.physics.DriveAPI.Apply(prim, "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(float(spec.stiffness))
    drive.CreateDampingAttr(float(spec.damping))
    drive.CreateTargetPositionAttr(float(spec.target))
    drive.CreateTargetVelocityAttr(0.0)
    drive.CreateMaxForceAttr(float(spec.max_force))  # always finite (an infinite one is rejected on this stack)


def author_joint(px: _Pxr, stage: Any, env_root: str, spec: JointSpec) -> str:
    path = f"{env_root}/{JOINTS_FOLDER}/{spec.name}"
    pos0, pos1 = joint_frames(spec)
    joint = px.physics.RevoluteJoint.Define(stage, path)
    joint.CreateBody0Rel().SetTargets([f"{env_root}/{spec.body0}"])
    joint.CreateBody1Rel().SetTargets([f"{env_root}/{spec.body1}"])
    joint.CreateLocalPos0Attr(px.gf.Vec3f(*pos0))
    joint.CreateLocalRot0Attr(px.gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(px.gf.Vec3f(*pos1))
    joint.CreateLocalRot1Attr(px.gf.Quatf(*(float(v) for v in spec.rot1)))
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateAxisAttr(spec.axis)
    joint.CreateLowerLimitAttr(float(spec.limits[0]))
    joint.CreateUpperLimitAttr(float(spec.limits[1]))
    _drive(px, joint.GetPrim(), spec)
    return path


def author(stage: Any, env_root: str, name: str) -> list[str]:
    """Build a body's parts (and its joint to the body it hangs on); the paths made."""
    px = _Pxr()
    body = f"{env_root}/{name}"
    folder = f"{body}/{PARTS_FOLDER}"
    friction = FRICTION[name]
    own = own_look(name)
    looks = {own, *(p.look for p in PARTS[name]())}
    materials = {look: _material(px, stage, f"{folder}/mat_{look}", look, friction, name in COMBINE_MIN) for look in sorted(looks)}
    colliders = spec_colliders(px, stage, body)
    if not colliders:
        raise RuntimeError(f"no collider under {body}")
    for prim in colliders:
        binding = px.shade.MaterialBindingAPI.Apply(prim)
        binding.Bind(materials[own], px.shade.Tokens.strongerThanDescendants)
        binding.Bind(materials[own], px.shade.Tokens.strongerThanDescendants, "physics")
        if name == LETTER_N:
            _thin(px, prim)
    made = [str(m.GetPath()) for m in materials.values()]
    for part in PARTS[name]():
        made.append(_author_part(px, stage, folder, part, materials))
    if name in FREE:
        _tune_body(px, stage.GetPrimAtPath(body))
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

    marker = "collect_flagged_mailbox_letter.scene_watcher"

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
LOST_BELOW = 0.05  # the letter is lost once any of it is this far below the table top (it has fallen off the table)


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
        """Fallen off the table: any of it LOST_BELOW under the table top (held in the air past an edge is not lost)."""
        return float(self.low[2]) < TOP - LOST_BELOW or not np.all(np.isfinite(self.center))


def views(objects: Mapping[str, _State]) -> dict[str, PartView]:
    return {name: PartView.of(name, objects[name]) for name in FREE}


def door_open_deg(door: PartView) -> float:
    """A bottom-hinged door's opening (degrees; 0 shut, its top swung toward the arms positive)."""
    z_axis = door.rot[:, 2]
    return math.degrees(math.atan2(-float(z_axis[0]), float(z_axis[2])))


def flag_up_deg(flag: PartView) -> float:
    """A flag's rise (degrees; 0 down along the side pointing back, 90 straight up)."""
    b = flag.name.removeprefix("flag_")
    arm = flag.rot @ (_v((0.0, 0.0, 1.0)) if raised(b) else _v((1.0, 0.0, 0.0)))
    return math.degrees(math.atan2(float(arm[2]), float(arm[0])))


def letter_corners(letter: PartView) -> np.ndarray:
    half = _v(LETTER) / 2
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float)
    return np.asarray(letter.origin + (signs * half) @ letter.rot.T, dtype=float)


TRAY_INNER_Z = 0.030  # the letter counts as in the tray when its lowest corner is within this of the tray's floor


def letter_in_tray(letter: PartView, tray: PartView) -> bool:
    """Every corner of the envelope inside the tray's walls (seen from above, in the tray's own frame) and its
    lowest corner within TRAY_INNER_Z of the tray's floor."""
    local = (letter_corners(letter) - tray.origin) @ tray.rot
    hx, hy = TRAY[0] / 2 - TRAY_WALL, TRAY[1] / 2 - TRAY_WALL
    inside = bool(np.all(np.abs(local[:, 0]) <= hx) and np.all(np.abs(local[:, 1]) <= hy))
    floor = TRAY_FLOOR / 2
    low = float(local[:, 2].min())
    return inside and floor - 0.003 <= low <= floor + TRAY_INNER_Z


def letter_out(letter: PartView) -> bool:
    """The whole envelope is in front of the mailboxes' fronts (out of its mailbox)."""
    return bool(np.all(letter_corners(letter)[:, 0] < X_FRONT - DOOR_T))


def letter_in_box(letter: PartView, b: str) -> bool:
    """Some of the envelope is inside mailbox ``b`` (behind its front, within its sides)."""
    c = letter_corners(letter)
    yc = Y_OF[b]
    behind = c[:, 0] > X_FRONT - DOOR_T
    within = np.abs(c[:, 1] - yc) < MB_W / 2 + 0.01
    return bool(np.any(behind & within))


# --- the mechanisms' rules, applied every control step (README, "Fidelity") ------------------------------------------
@dataclass
class Mechanisms:
    """What the rules last wrote, so a write happens only on a change."""

    last: dict[tuple[str, str], float] = field(default_factory=dict)

    def step(self, parts: Mapping[str, PartView]) -> list[tuple[str, str, float]]:
        want: list[tuple[str, str, float]] = []
        for b in BOXES:
            # The latch: within CATCH_DEG of shut its hook is over the bead: it pulls the door shut and holds it.
            near = door_open_deg(parts[door_name(b)]) < CATCH_DEG
            want.append((f"hinge_{b}", "drive:angular:physics:stiffness", CATCH_K if near else 0.0))
            # The flag's detent: past halfway it carries the flag on to its stop and holds it there.
            up = flag_up_deg(parts[flag_name(b)]) > FLAG_SWITCH
            want.append((f"flag_{b}_pivot", "drive:angular:physics:targetPosition", -90.0 if up else 0.0))
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
    """Every finger point at least HAND_CLEAR from the envelope's, the flagged mailbox's door's and flag's outlines."""
    near = [parts[LETTER_N], parts[door_name(FLAGGED)], parts[flag_name(FLAGGED)]]
    for g in grippers:
        for p in g.finger_points():
            for part in near:
                gap = np.maximum(np.maximum(part.low - p, p - part.high), 0.0)
                if float(np.linalg.norm(gap)) < HAND_CLEAR:
                    return False
    return True


# --- the outcome checks (a protractor or phone level, a ruler, photos and the trial video; README, "Scoring") -------
SHUT_DEG = 1.5  # a door within this of shut is latched (the latch's hook catches within 3 degrees)
FLAG_DOWN_DEG = 10.0  # a flag within this of its rest along the side is down
OPENED_WRONG_DEG = 10.0  # a mailbox whose flag was down, opened past this at any time: the trial scores 0
HOLD_STEPS = 20


@dataclass(frozen=True)
class State:
    door_open: dict[str, float]
    flag_up: dict[str, float]
    in_tray: bool
    out: bool
    inside: bool
    still: bool
    clear: bool
    lost: bool

    def others_untouched(self) -> bool:
        return all(self.door_open[b] < SHUT_DEG and self.flag_up[b] < FLAG_DOWN_DEG for b in BOXES if b != FLAGGED)


def state(parts: Mapping[str, PartView], grippers: Sequence[Gripper] | None) -> State:
    letter = parts[LETTER_N]
    return State(
        door_open={b: door_open_deg(parts[door_name(b)]) for b in BOXES},
        flag_up={b: flag_up_deg(parts[flag_name(b)]) for b in BOXES},
        in_tray=letter_in_tray(letter, parts[TRAY_N]),
        out=letter_out(letter),
        inside=letter_in_box(letter, FLAGGED),
        still=all(p.still for p in parts.values()),
        clear=grippers is not None and hands_clear(parts, grippers),
        lost=letter.lost,
    )


@dataclass
class Ledger:
    steps: int = 0
    taken: bool = False
    void: str | None = None
    held: int = 0

    def observe(self, now: State) -> None:
        self.steps += 1
        self.taken = self.taken or now.out
        for b in BOXES:
            if b != FLAGGED and now.door_open[b] > OPENED_WRONG_DEG and self.void is None:
                self.void = f"the {b} mailbox (flag down) was opened"
        if now.lost and self.void is None:
            self.void = "the letter fell off the table"


def grade(ledger: Ledger, now: State, built: bool) -> float:
    """The ladder (README, "Scoring"). Every rung needs the other two mailboxes shut with their flags down at the
    time. The whole trial scores 0 (void) only for a real failure: opening a mailbox whose flag is down, the letter
    falling off the table; or if the arms' state or the parts couldn't be read, or a reading wasn't finite."""
    if not built or ledger.void is not None or not now.others_untouched():
        ledger.held = 0
        return 0.0
    latched = now.door_open[FLAGGED] < SHUT_DEG
    lowered = now.flag_up[FLAGGED] < FLAG_DOWN_DEG
    done = now.in_tray and latched and lowered
    if done and now.still and now.clear:
        ledger.held += 1
    else:
        ledger.held = 0
    if done and ledger.held >= HOLD_STEPS:
        return 1.0
    if now.in_tray and latched:
        return 0.75
    if now.in_tray:
        return 0.5
    if ledger.taken and not now.inside:
        return 0.25
    return 0.0


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
RESET_TAG = "collect_flagged_mailbox_letter_resets"
LAST_STATE: dict[str, Any] = {}  # the last step's readings (for traces)


def _warn_once(key: str, message: str) -> None:
    if key not in _warned:
        _warned.add(key)
        with contextlib.suppress(Exception):
            warnings.warn(f"collect_flagged_mailbox_letter: {message}", RuntimeWarning, stacklevel=3)


def reset_writes() -> list[tuple[str, str, float]]:
    """A new trial: every drive back to its built value."""
    out: list[tuple[str, str, float]] = []
    for j in JOINTS:
        out += [
            (j.name, "drive:angular:physics:stiffness", j.stiffness),
            (j.name, "drive:angular:physics:damping", j.damping),
            (j.name, "drive:angular:physics:targetPosition", j.target),
            (j.name, "drive:angular:physics:maxForce", j.max_force),
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
    _warn_once("reset", "could not hook the embodiment's reset, so the arms' state is unknown: every trial scores 0")


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
        _warn_once("robot", f"can't read the arms' state ({type(error).__name__}: {error}); the trial scores 0")
        return None


READINGS = ("pos", "quat_xyzw", "lin_vel", "ang_vel", "low", "high")


def check_finite(objects: Mapping[str, Any], grippers: Sequence[Gripper]) -> None:
    """Every reading the score uses is a finite number (the moving parts' poses, outlines and speeds; the hands)."""
    values = [getattr(objects[name], key) for name in FREE for key in READINGS]
    values += [v for g in grippers for v in (g.tcp, g.opening, g.approach, g.closing)]
    if not all(bool(np.all(np.isfinite(np.asarray(v, dtype=float)))) for v in values):
        raise ValueError("a non-finite reading")


def _update(objects: Mapping[str, Any]) -> float:
    """Advance the trial once per control step (success() and score() share the step's state dict)."""
    global _last_objects, _SCORE
    if objects is _last_objects:
        return _SCORE
    with contextlib.suppress(Exception):
        _hook_resets()
    _restart_if_clock_went_back()
    grippers = _robot()
    if grippers is None:
        _LEDGER.void = _LEDGER.void or "the arms' state couldn't be read"
        _warn_once("void", f"this trial scores 0: {_LEDGER.void}")
    built = parts_built()
    try:
        check_finite(objects, grippers or [])
        parts = views(objects)
        apply_writes(_MECH.step(parts))
        now = state(parts, grippers)
        LAST_STATE.update(now.__dict__)
    except Exception as error:  # any unreadable part voids the trial; nothing escapes the scorer
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
    """1.0: the letter in the tray, the flagged mailbox latched shut with its flag down, the other two mailboxes shut
    with their flags down, everything still and both hands clear for 1 s; 0.75 with the letter in the tray and the door
    latched but not that (the flag up, or not yet still, clear and held); 0.5 with the letter in the tray; 0.25 once
    the letter is out of its mailbox and not back in it; see README.md."""
    return _update(objects)


def success(objects: Mapping[str, Any]) -> bool:
    return _update(objects) >= 1.0


# --- the scripted demo ----------------------------------------------------------------------------------------------
FINGER_MAX = -FINGER_OPEN * 2


class Grip(float):
    """A Waypoint yaw that carries a full tool rotation (3x3, columns: closing axis, across, approach)."""

    rot: np.ndarray

    def __new__(cls, rot: np.ndarray) -> "Grip":
        obj = super().__new__(cls, 0.0)
        obj.rot = np.asarray(rot, dtype=float)
        return obj


GRIP_TAG = "collect_flagged_mailbox_letter_grip"


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


def ry(deg: float) -> np.ndarray:
    t = math.radians(deg)
    return np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])


def rz(deg: float) -> np.ndarray:
    t = math.radians(deg)
    return np.array([[math.cos(t), -math.sin(t), 0.0], [math.sin(t), math.cos(t), 0.0], [0.0, 0.0, 1.0]])


def _clip(vec: np.ndarray, most: float) -> np.ndarray:
    n = float(np.linalg.norm(vec))
    return vec if n <= most or n == 0.0 else np.asarray(vec * (most / n), dtype=float)


def _finger(opening: float) -> float:
    return float(-min(max(opening, 0.0), FINGER_MAX) / 2)


# The routine, by the right arm (official IK, README "The demo"): pinch the pull tab and draw the door open past its latch, let it fall open; press the
# shut fingertips on the envelope and draw it toward the sill until it overhangs; pinch the overhang, lift its near
# end (the envelope pivots on its far end) and draw it out along its length; lay it in the tray; lift the door shut
# from under its free edge into the latch; push the flag down past its detent's middle; park. The other arm watches.
# The demo reads the parts' poses from the simulator every control step (a scripted demo may); it never moves a part.
ARM = "right"  # the working arm; the left arm watches from its park
HINGE_X = X_FRONT - DOOR_T / 2
_READ_ERRORS = (LookupError, AttributeError, TypeError, ValueError, ArithmeticError, ImportError)
SERVO_GAIN = 0.15
# The pull tab pinch: level, pointing at the door, closing up-down; the pads' tips TAB_SHORT short of the door's face.
TAB_SHORT = 0.003
TAB_OPEN = 0.016
# The pull: the hand draws the tab round the hinge at PULL_SPEED, never more than PULL_LEAD ahead of the door (the arm
# is soft: the latch's 1 N m needs some lead), until the door is PULL_AT open, past the latch's 3 degrees; then it
# lets go, sliding back and up as its fingers open, and the door falls open to its stop.
PULL_SPEED, PULL_LEAD, PULL_AT, PULL_MAX = 8.0, 6.0, 8.0, 14.0
RELEASE_S, RELEASE_BACK, RELEASE_UP = 0.30, 0.050, 0.035
# The press and drag: fingers shut (closing across y), pitched PRESS_PITCH down; their lowest point PRESS_DEPTH into
# the envelope's top, PRESS_IN behind its near edge; drawn toward the robot at DRAG_SPEED, never more than DRAG_LEAD
# ahead of the envelope, until it overhangs the hinge line by OVERHANG.
PRESS_PITCH = 35.0
PRESS_YAW_MAX = 15.0
TIP_LOW, TIP_AHEAD = 0.0127, 0.0050  # the shut fingertips' lowest point from the TCP: down, ahead (official meshes)
PRESS_IN, PRESS_DEPTH = (0.050, 0.100), 0.006  # two strokes; each press stays inside the box, behind the sill
OVERHANG_1 = 0.035  # the first stroke ends with this much over the sill (its press then 15 mm inside)
DRAG_SPEED, DRAG_LEAD = 0.020, 0.008
OVERHANG = 0.080
# The pinch: pitched PINCH_PITCH down, closing across the envelope's thickness, the pads' tips PINCH_TIP in front of
# the hinge line (the open door slopes down under the overhang: the lower finger must clear it).
PINCH_PITCH = 0.0  # level: the pads meet the flat envelope square
PINCH_TIP = 0.032
PINCH_OPEN = 0.008
# Out of the mailbox: the near end lifted (the envelope pivots on its far end) by TILT, then drawn out along its own
# length by DRAW_OUT and lifted LIFT_OUT; laid in the tray with its lowest corner TRAY_DROP over the tray's floor.
TILT = 30.0
DRAW_OUT = 0.150
LIFT_OUT = 0.0
TRAY_DROP = 0.012
TRAY_YAW = 90.0  # the envelope is turned to lie along the tray (its long side across the robot) as it is laid in
TRAY_TILT = 40.0  # the hand's extra pitch over the tray (official IK margin)
TRAY_PLACE = (-0.010, 0.0)  # where the envelope's middle is let go, inside the tray
# The door lifted shut (carried on the shut fingers' upper face, the grip's pitch easing from DOOR_PITCH[0] open to DOOR_PITCH[1] shut): shut
# fingers across y, the door's free edge on their upper face DOOR_SC back from their tips; the servo leads the door by
# at most DOOR_LEAD and goes DOOR_PAST past shut once the latch has it.
DOOR_PITCH = (45.0, 10.0)  # open, shut (official IK)
DOOR_DY = -0.040  # the lift is this far toward the working arm's side of the door's middle (clear of the tab)
DOOR_SPEED, DOOR_LEAD, DOOR_PAST = 15.0, 20.0, 4.0
DOOR_FRONT = 0.080  # the hand first comes this far in front of where it goes under the door (GPU run F: coming down
# from above, it landed on the open door's top)
DOOR_IN = 0.040  # the fingers carry the door this far in from its edge, under its face (GPU runs C-E: carried at the
# edge itself, the fingers' sloping face only grazed the edge and rose past it; the open door's face slopes 25 deg)
DOOR_SC = ((10.0, -0.010), (60.0, 0.045))  # open: the door's edge 45 mm back on the fingers' upper face (GPU run B: at
# 20 mm the fingers came up just in front of the edge and missed the door); near shut the fingertips push the door's
# face 10 mm under its edge
FINGER_TAPER = ((0.0, 0.0077), (0.008, 0.0110), (0.013, 0.0128), (0.023, 0.0169), (0.033, 0.0206), (0.043, 0.0246), (0.053, 0.0285), (0.068, 0.0340))
# The flag pushed down: shut fingertips on the arm's front edge FLAG_R from the pivot, the hand pitched FLAG_PITCH;
# the hand's angle leads the flag down at FLAG_SPEED, at most FLAG_LEAD ahead, to FLAG_END (past the detent's middle).
FLAG_PITCH_MAX = 30.0  # (steeper is out of the official IK's reach there)
FLAG_R, FLAG_PITCH, FLAG_YAW, FLAG_DY = 0.160, 10.0, 25.0, 0.002  # (GPU run B: at -5 mm the fingertips'
# rounded edge only grazed the 3 mm arm and the push stalled at 86 deg; yawed and pitched so the gripper and wrist clear the
# box beside the flag: official URDF meshes, dry run)
# (yawed: the hand comes from the flag's outer side, so its body stays clear of the box the flag is on)
FLAG_SPEED, FLAG_LEAD, FLAG_END = 20.0, 20.0, 36.0  # (GPU run C: the flag lagged the hand by about 4 deg)
# Modes of the servo (what it does while a waypoint is the current one).
TAB_ON, PULL, PRESS_ON, DRAG, PINCH_ON, DOOR_UNDER, DOOR_UP, FLAG_PUSH = (
    "tab", "pull", "press", "drag", "pinch", "door_under", "door_up", "flag",
)
PRESS2_ON, DRAG2 = "press2", "drag2"


def _control_hz() -> float:
    try:
        hz = float(_embodiment.info.control_hz)
        return hz if math.isfinite(hz) and hz > 0 else 20.0
    except (LookupError, AttributeError, TypeError, ValueError, ArithmeticError):
        return 20.0


def _closing_sign(side: str) -> float:
    return 1.0 if side == "right" else -1.0


MIRROR = np.diag([1.0, -1.0, 1.0])


def _for_arm(side: str, rot: np.ndarray) -> np.ndarray:
    """A grip planned for the right arm, as the left arm takes it (only its park): mirrored through the arms' middle
    plane."""
    if side == "right":
        return np.asarray(rot, dtype=float)
    return np.asarray(MIRROR @ rot @ MIRROR, dtype=float)


ARM_BASE_Y = -0.305  # the working (right) arm's base


def press_yaw(b: str) -> float:
    """The press's turn about the vertical (degrees): along the line from the working arm's base to the box, at most
    PRESS_YAW_MAX (straight in, the forearm meets the arch on one side; at 34 the hand meets the other: official URDF
    meshes, dry run)."""
    return float(np.clip(math.degrees(math.atan2(Y_OF[b] - ARM_BASE_Y, X_FRONT + 0.07)), -PRESS_YAW_MAX, PRESS_YAW_MAX))


def tab_point(b: str, phi: float) -> np.ndarray:
    """The pull tab's middle with the door ``phi`` open."""
    tab = _v((X_FRONT - DOOR_T - TAB[0] / 2, Y_OF[b], Z_BOT + SIDE_H + ARCH_R + DOOR_RIM - TAB[2] / 2))
    h = _v((HINGE_X, Y_OF[b], Z_BOT - HINGE_DROP))
    return np.asarray(h + ry(-phi) @ (tab - h), dtype=float)


def tab_grip(side: str, phi: float) -> np.ndarray:
    """Closing down for the right arm (its wrist then turns away from its -120 degree stop as the door opens)."""
    return _for_arm(side, ry(-phi) @ aim((1.0, 0.0, 0.0), (0.0, 0.0, -1.0)))


def tab_tcp(b: str, side: str, phi: float, back: float = 0.0, up: float = 0.0) -> np.ndarray:
    """The tab pinch's TCP with the door ``phi`` open (the grip turns with the door), drawn ``back`` along the grip's
    axis and lifted ``up``."""
    rot = tab_grip(side, phi)
    a = rot[:, 2]
    face = tab_point(b, phi) + (TAB[0] / 2) * a  # the door's face behind the tab's middle
    return np.asarray(face - (TAB_SHORT + PAD_TIP_PAST_TCP + back) * a + _v((0.0, 0.0, up)), dtype=float)


def press_grip(side: str, yaw: float = 0.0) -> np.ndarray:
    p = math.radians(PRESS_PITCH)
    return np.asarray(rz(yaw) @ _for_arm(side, aim((math.cos(p), 0.0, -math.sin(p)), (0.0, -1.0, 0.0))), dtype=float)


def press_tcp(contact: Pt) -> np.ndarray:
    """The TCP that puts the shut fingertips' lowest point at ``contact``."""
    return np.asarray(_v(contact) + _v((-TIP_AHEAD, 0.0, TIP_LOW)), dtype=float)


def pinch_grip(side: str) -> np.ndarray:
    return _for_arm(side, ry(PINCH_PITCH) @ aim((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)))


def pinch_tcp(y: float, z_mid: float, side: str) -> np.ndarray:
    """The pinch's TCP: the pads' tips PINCH_TIP in front of the hinge line, their middle at the envelope's middle."""
    a = pinch_grip(side)[:, 2]
    tips = _v((HINGE_X - PINCH_TIP, y, z_mid))
    return np.asarray(tips - PAD_TIP_PAST_TCP * a, dtype=float)


def letter_near_x(letter: PartView) -> float:
    return float(letter_corners(letter)[:, 0].min())


def _taper(s: float) -> float:
    return float(np.interp(s, [a for a, _ in FINGER_TAPER], [b for _, b in FINGER_TAPER]))


def door_grip(side: str, phi: float) -> np.ndarray:
    s = min(max(phi / DOOR_OPEN, 0.0), 1.0)
    p_open, p_shut = DOOR_PITCH
    p = math.radians(p_shut + (p_open - p_shut) * s)
    return _for_arm(side, aim((math.cos(p), 0.0, -math.sin(p)), (0.0, -1.0, 0.0)))


def door_edge(b: str, phi: float) -> np.ndarray:
    """The door's outer free-edge corner (its top edge when shut, at DOOR_DY from the middle) with the door ``phi``
    open (the edge where the panel's lower rectangle meets the arch: the arch above it is narrower)."""
    dy = DOOR_DY * _closing_sign(ARM)
    h = _v((HINGE_X, Y_OF[b] + dy, Z_BOT - HINGE_DROP))
    r = ARCH_R + DOOR_RIM
    top = Z_BOT + SIDE_H + math.sqrt(max(r * r - dy * dy, 0.0))
    along = top - (Z_BOT - HINGE_DROP)
    corner = _v((X_FRONT - DOOR_T, Y_OF[b] + dy, Z_BOT - HINGE_DROP + along))
    return np.asarray(h + ry(-phi) @ (corner - h), dtype=float)


def door_sc(phi: float) -> float:
    (a0, s0), (a1, s1) = DOOR_SC
    return float(np.interp(phi, [a0, a1], [s0, s1]))


def door_hand(b: str, side: str, phi: float, back: float = 0.0) -> np.ndarray:
    """The door-lifting hand's TCP with the door ``phi`` open: the door's free edge on the shut fingers' upper face,
    door_sc(phi) back from their tips; ``back`` drawn back along the hand's own axis."""
    rot = door_grip(side, phi)
    a, up = rot[:, 2], -rot[:, 1]
    up = up if up[2] >= 0.0 else -up  # the shut fingers' upper face
    sc = door_sc(phi)
    hinge = _v((HINGE_X, Y_OF[b], Z_BOT - HINGE_DROP))
    edge = door_edge(b, phi)
    inward = _v((hinge[0] - edge[0], 0.0, hinge[2] - edge[2]))
    inward = inward / float(np.linalg.norm(inward))
    tip = edge + DOOR_IN * inward + sc * a - _taper(sc) * up
    return np.asarray(tip - (PAD_TIP_PAST_TCP + back) * a, dtype=float)


def flag_grip(side: str, theta: float = 90.0) -> np.ndarray:
    """Shut fingers stacked up-down, pointing at the flag from its outer side (yawed FLAG_YAW toward the box), pitched
    down as the flag goes down so the push stays square to the arm (GPU run C: pushed level, the fingertips slid up the
    tilting arm and off it at 48 degrees)."""
    p = math.radians(min(max(90.0 - theta, 0.0) + FLAG_PITCH, FLAG_PITCH_MAX))
    return np.asarray(rz(FLAG_YAW) @ _for_arm(side, aim((math.cos(p), 0.0, -math.sin(p)), (0.0, 0.0, 1.0))), dtype=float)


def _flag_retreat(b: str) -> np.ndarray:
    """Where the hand goes back to from the flag, high in front of the boxes."""
    return _v((0.280, Y_OF[b] - MB_W / 2 - 0.030, TOP + 0.450))


def flag_tcp(b: str, side: str, theta: float, back: float = 0.0) -> np.ndarray:
    """The flag-pushing hand's TCP with its fingertips on the arm's front edge FLAG_R from the pivot, the flag at
    ``theta`` (degrees up); ``back`` drawn back along the hand's axis."""
    t = math.radians(theta)
    arm = _v((math.cos(t), 0.0, math.sin(t)))
    toward_down = _v((math.sin(t), 0.0, -math.cos(t)))
    contact = _v(flag_pivot(b)) + FLAG_R * arm - (FLAG_BAR / 2) * toward_down + _v((0.0, FLAG_DY, 0.0))
    a = flag_grip(side, theta)[:, 2]
    return np.asarray(contact - (PAD_TIP_PAST_TCP + 0.004 + back) * a, dtype=float)


class Servo:
    """The demo's watching of the parts for one arm (a scripted demo may read the simulator's state; it never moves a
    part). Every control step it reads the last step's parts and corrects the arm's live targets, by the current
    waypoint's mode:
    - TAB_ON: the pinch aimed at the tab as the door is;
    - PULL: the door's angle led by at most PULL_LEAD until it is PULL_AT open, then held there;
    - PRESS_ON, PINCH_ON: aimed at the envelope as it lies;
    - DRAG: the press drawn toward the robot, never more than DRAG_LEAD ahead of the envelope, until it overhangs;
    - DOOR_UNDER, DOOR_UP: the door's free edge followed, then led up into its latch;
    - FLAG_PUSH: the flag led down past its detent's middle.
    """

    def __init__(self, side: str, b: str) -> None:
        self.side, self.b = side, b
        self.phases: list[tuple[float, float, str | None]] = []
        self.labels: list[tuple[float, str]] = []
        self.restart()

    def restart(self) -> None:
        self.tick = -1
        self.mode: str | None = None
        self.tab_phi = 0.0
        self.pull = 0.0
        self.pulled = False
        self.letter_shift = np.zeros(3)
        self.letter_shift2 = np.zeros(3)
        self.pinch_shift = np.zeros(3)
        self.drag = 0.0
        self.dragged = False
        self.drag2 = 0.0
        self.dragged2 = False
        self.door_phi = DOOR_OPEN
        self.flag_deg = FLAG_UP_START + 4.0
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
            self.correct(self.mode, views(objects), 1.0 / hz)
        except _READ_ERRORS:
            self._restore(saved)
            return
        values = [v for k, v in self._state().items() if k not in ("pulled", "dragged", "dragged2")]
        if not all(np.all(np.isfinite(np.asarray(v, dtype=float))) for v in values):
            self._restore(saved)

    _KEPT = ("tab_phi", "pull", "pulled", "letter_shift", "letter_shift2", "pinch_shift", "drag", "dragged", "drag2", "dragged2", "door_phi", "flag_deg")

    def _state(self) -> dict[str, Any]:
        return {k: (getattr(self, k).copy() if isinstance(getattr(self, k), np.ndarray) else getattr(self, k)) for k in self._KEPT}

    def _restore(self, saved: Mapping[str, Any]) -> None:
        for k, v in saved.items():
            setattr(self, k, v)

    def correct(self, mode: str, parts: Mapping[str, PartView], dt: float) -> None:
        door, letter, flag = parts[door_name(self.b)], parts[LETTER_N], parts[flag_name(self.b)]
        opened = door_open_deg(door)
        if mode == TAB_ON:
            self.tab_phi = float(np.clip(opened, -1.0, 6.0))
            return
        if mode == PULL:
            self.pulled = self.pulled or opened >= PULL_AT
            if self.pulled:
                self.pull = min(self.pull, opened + 1.0)
            else:
                self.pull = min(self.pull + PULL_SPEED * dt, opened + PULL_LEAD, PULL_MAX)
            self.seen.update(door=round(opened, 2), pull=round(self.pull, 2))
            return
        if mode in (PRESS_ON, PRESS2_ON, PINCH_ON):
            off = _clip(letter.origin - _v(LETTER_CENTER), 0.15)
            if mode == PRESS_ON:
                self.letter_shift = off
            elif mode == PRESS2_ON:
                self.letter_shift2 = off
            else:
                self.pinch_shift = off
            return
        if mode in (DRAG, DRAG2):
            near = letter_near_x(letter)
            first = mode == DRAG
            shift = self.letter_shift if first else self.letter_shift2
            start = LETTER_CENTER[0] - LETTER[0] / 2 + float(shift[0])
            done = near <= HINGE_X - (OVERHANG_1 if first else OVERHANG)
            if first:
                self.dragged = self.dragged or done
                if not self.dragged:
                    self.drag = max(self.drag - DRAG_SPEED * dt, near - start - DRAG_LEAD)
            else:
                self.dragged2 = self.dragged2 or done
                if not self.dragged2:
                    self.drag2 = max(self.drag2 - DRAG_SPEED * dt, near - start - DRAG_LEAD)
            self.seen.update(near_mm=round(1000 * (near - HINGE_X), 1), drag_mm=round(1000 * (self.drag if first else self.drag2), 1))
            return
        if mode in (DOOR_UNDER, DOOR_UP):
            if mode == DOOR_UNDER:
                self.door_phi = float(np.clip(opened, 0.0, DOOR_OPEN))
            elif opened < CATCH_DEG:
                self.door_phi = max(min(self.door_phi - DOOR_SPEED * dt, opened), -DOOR_PAST)
            else:
                self.door_phi = max(self.door_phi - DOOR_SPEED * dt, opened - DOOR_LEAD, -DOOR_PAST)
            self.seen.update(door=round(opened, 2), door_phi=round(self.door_phi, 2))
            return
        if mode == FLAG_PUSH:
            now = flag_up_deg(flag)
            self.flag_deg = max(self.flag_deg - FLAG_SPEED * dt, now - FLAG_LEAD, FLAG_END)
            self.seen.update(flag=round(now, 2), flag_cmd=round(self.flag_deg, 2))

    def report(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "tab_phi": round(self.tab_phi, 2),
            "pull": round(self.pull, 2),
            "pulled": self.pulled,
            "letter_mm": [round(1000 * float(v), 2) for v in self.letter_shift],
            "letter2_mm": [round(1000 * float(v), 2) for v in self.letter_shift2],
            "pinch_mm": [round(1000 * float(v), 2) for v in self.pinch_shift],
            "drag_mm": round(1000 * self.drag, 2),
            "dragged": self.dragged,
            "drag2_mm": round(1000 * self.drag2, 2),
            "dragged2": self.dragged2,
            "door_phi": round(self.door_phi, 2),
            "flag_deg": round(self.flag_deg, 2),
            **self.seen,
        }


class Live:
    """A waypoint position read every control step: ``fn(servo)`` after the servo has looked at the parts."""

    def __init__(self, servo: Servo, mode: str | None, fn: Callable[[Servo], np.ndarray]) -> None:
        self.servo, self.mode, self.fn = servo, mode, fn

    def value(self) -> np.ndarray:
        self.servo.update()
        return np.asarray(self.fn(self.servo), dtype=float)

    def __array__(self, dtype: Any = None, copy: Any = None) -> np.ndarray:
        return np.asarray(self.value(), dtype=dtype)

    def __len__(self) -> int:
        return 3

    def __getitem__(self, index: Any) -> Any:
        return tuple(float(v) for v in self.value())[index]

    def __iter__(self) -> Iterator[float]:
        return iter(tuple(float(v) for v in self.value()))


class LiveGrip(float):
    """A Waypoint yaw whose rotation is read every control step: ``fn(servo)``."""

    servo: Servo
    fn: Callable[[Servo], np.ndarray]

    def __new__(cls, servo: Servo, fn: Callable[[Servo], np.ndarray]) -> "LiveGrip":
        obj = super().__new__(cls, 0.0)
        obj.servo, obj.fn = servo, fn
        return obj

    @property
    def rot(self) -> np.ndarray:
        self.servo.update()
        return np.asarray(self.fn(self.servo), dtype=float)


def _park(side: str) -> tuple[np.ndarray, np.ndarray]:
    """Each arm's park: out to its own side and up, its wrist camera on the mailboxes."""
    s = 1.0 if side == "left" else -1.0
    return _v((0.15, s * 0.20, TOP + 0.25)), _for_arm(side, aim((1.0, 0.5, -0.5), (0.0, 0.0, 1.0)))


def _contact(sv: Servo, stroke: int) -> np.ndarray:
    """A stroke's first contact on the envelope as it lies at the stroke's start: PRESS_IN behind its near edge,
    PRESS_DEPTH into its top."""
    shift = sv.letter_shift if stroke == 0 else sv.letter_shift2
    near = LETTER_CENTER[0] - LETTER[0] / 2 + float(shift[0])
    return _v((near + PRESS_IN[stroke], LETTER_CENTER[1] + float(shift[1]), FLOOR_TOP + LETTER[2] - PRESS_DEPTH))


def _pinch_pose(sv: Servo) -> tuple[np.ndarray, np.ndarray]:
    """The pinch's TCP and rotation on the envelope as it lies (after the drag)."""
    y = LETTER_CENTER[1] + float(sv.pinch_shift[1])
    z = LETTER_CENTER[2] + float(sv.pinch_shift[2])
    return pinch_tcp(y, z, sv.side), pinch_grip(sv.side)


def _far_edge(sv: Servo) -> np.ndarray:
    """The envelope's far bottom edge (its pivot when its near end is lifted), as it lay at the pinch."""
    near = LETTER_CENTER[0] - LETTER[0] / 2 + float(sv.pinch_shift[0])
    return _v((near + LETTER[0], LETTER_CENTER[1] + float(sv.pinch_shift[1]), FLOOR_TOP))


def _tilted(sv: Servo, tilt: float, out: float, lift: float) -> tuple[np.ndarray, np.ndarray]:
    """The pinch turned ``tilt`` degrees about the envelope's far edge (near end up), then drawn ``out`` along the
    envelope's length toward the robot and lifted ``lift``."""
    tcp, rot = _pinch_pose(sv)
    far = _far_edge(sv)
    turn = ry(tilt)
    along = turn @ _v((1.0, 0.0, 0.0))
    return far + turn @ (tcp - far) - out * along + _v((0.0, 0.0, lift)), turn @ rot


def _in_tray(sv: Servo, drop: float, at: Pt = TRAY_PLACE) -> tuple[np.ndarray, np.ndarray]:
    """The hand over the tray holding the envelope as it was pinched (rigidly), turned as when it was drawn out, with
    the envelope's middle over the tray's middle and its lowest corner ``drop`` over the tray's floor."""
    tcp, rot = _pinch_pose(sv)
    centre = _v(LETTER_CENTER) + sv.pinch_shift
    turn = rz(TRAY_YAW if sv.side == "right" else -TRAY_YAW) @ ry(TRAY_TILT)
    new_rot = turn @ rot
    held_centre = turn @ (centre - tcp)  # the envelope's middle from the TCP, as held
    half = _v(LETTER) / 2
    corners = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], dtype=float) * half
    low = float((corners @ turn.T)[:, 2].min())
    want = _v((float(at[0]), float(at[1]), TOP + TRAY_FLOOR + drop - low))
    return want - held_centre, new_rot


def _tilt_pos(frac: float) -> Callable[[Servo], np.ndarray]:
    def pos(s: Servo) -> np.ndarray:
        return _tilted(s, TILT * frac, 0.0, 0.0)[0]

    return pos


def _tilt_rot(frac: float) -> Callable[[Servo], np.ndarray]:
    def rot(s: Servo) -> np.ndarray:
        return _tilted(s, TILT * frac, 0.0, 0.0)[1]

    return rot


def _stroke_contact(stroke: int) -> Callable[[Pt, int | None], Callable[[Servo], np.ndarray]]:
    """Waypoint positions for a press stroke: the TCP over the stroke's contact, offset, and drawn by the stroke's
    drag when ``drawn`` is set."""

    def at(offset: Pt, drawn: int | None) -> Callable[[Servo], np.ndarray]:
        def pos(s: Servo) -> np.ndarray:
            d = 0.0 if drawn is None else (s.drag if stroke == 0 else s.drag2)
            return press_tcp(_contact(s, stroke) + _v(offset) + _v((d, 0.0, 0.0)))

        return pos

    return at


ROUTINE = ("open", "letter", "tray", "shut", "flag", "park")


def routine(sv: Servo, steps: Sequence[str] = ROUTINE, tray_at: Pt = TRAY_PLACE) -> list[Waypoint]:
    """The working arm's plan for mailbox ``sv.b``: the named steps of ROUTINE, in its order (the negative tests leave
    some out; ``tray_at`` is where the envelope is let go)."""
    b, side = sv.b, sv.side
    shut, tab_open = _finger(0.0), _finger(TAB_OPEN)
    plan: list[Waypoint] = []

    def w(duration: float, mode: str | None, pos: Callable[[Servo], np.ndarray], grip: Callable[[Servo], np.ndarray], finger: float, label: str) -> None:
        plan.append(Waypoint(duration, Live(sv, mode, pos), finger, label, LiveGrip(sv, grip)))

    def tab_g(s: Servo) -> np.ndarray:
        return tab_grip(side, s.tab_phi)

    def pull_g(s: Servo) -> np.ndarray:
        return tab_grip(side, s.pull)

    # 1. The door, opened by its pull tab.
    if "open" in steps:
        w(2.5, None, lambda s: tab_tcp(b, side, 0.0, back=0.050), tab_g, tab_open, "to the door's pull tab")
        w(0.1, TAB_ON, lambda s: tab_tcp(b, side, s.tab_phi, back=0.050), tab_g, tab_open, "watch the door")
        w(1.5, TAB_ON, lambda s: tab_tcp(b, side, s.tab_phi), tab_g, tab_open, "pads round the tab")
        w(1.0, TAB_ON, lambda s: tab_tcp(b, side, s.tab_phi), tab_g, shut, "pinch the tab")
        w(0.1, PULL, lambda s: tab_tcp(b, side, s.pull), pull_g, shut, "pull the door past its latch")
        w(3.0, PULL, lambda s: tab_tcp(b, side, s.pull), pull_g, shut, "pull the door past its latch")
        w(RELEASE_S, None, lambda s: tab_tcp(b, side, s.pull, back=RELEASE_BACK, up=RELEASE_UP), pull_g, _finger(0.030), "let go: the door falls open")
        w(1.0, None, lambda s: tab_tcp(b, side, s.pull, back=RELEASE_BACK + 0.04, up=RELEASE_UP + 0.04), pull_g, _finger(0.030), "clear of the falling door")
    # 2. The envelope drawn to the sill by a fingertip press.
    if "letter" in steps:
        pg = press_grip(side, press_yaw(b))

        def press_g(s: Servo) -> np.ndarray:
            return pg

        for k, (on, drag_mode) in enumerate(((PRESS_ON, DRAG), (PRESS2_ON, DRAG2))):
            n = f" ({k + 1})"
            c = _stroke_contact(k)
            w(2.0, None, c((-0.060, 0.0, 0.035), None), press_g, shut, "over the envelope" + n)
            w(0.1, on, c((-0.060, 0.0, 0.035), None), press_g, shut, "watch the envelope" + n)
            w(1.2, on, c((0.0, 0.0, 0.015), None), press_g, shut, "fingertips over it" + n)
            w(1.0, on, c((0.0, 0.0, 0.0), None), press_g, shut, "press on the envelope" + n)
            w(0.1, drag_mode, c((0.0, 0.0, 0.0), k), press_g, shut, "draw it toward the sill" + n)
            w(4.0, drag_mode, c((0.0, 0.0, 0.0), k), press_g, shut, "draw it toward the sill" + n)
            w(1.0, None, c((-0.010, 0.0, 0.040), k), press_g, shut, "lift the fingertips" + n)
        # 3. The overhang pinched, the near end lifted, the envelope drawn out along its length.
        def pinch_g(s: Servo) -> np.ndarray:
            return _pinch_pose(s)[1]

        w(2.0, None, lambda s: _pinch_pose(s)[0] - 0.080 * _pinch_pose(s)[1][:, 2] + _v((0.0, 0.0, 0.030)), pinch_g, _finger(PINCH_OPEN), "in front of the overhang")
        w(1.0, None, lambda s: _pinch_pose(s)[0] - 0.080 * _pinch_pose(s)[1][:, 2], pinch_g, _finger(PINCH_OPEN), "level with it")
        w(0.1, PINCH_ON, lambda s: _pinch_pose(s)[0] - 0.080 * _pinch_pose(s)[1][:, 2], pinch_g, _finger(PINCH_OPEN), "watch the envelope")
        w(1.5, PINCH_ON, lambda s: _pinch_pose(s)[0], pinch_g, _finger(PINCH_OPEN), "pads round the overhang")
        w(1.0, None, lambda s: _pinch_pose(s)[0], pinch_g, shut, "pinch the envelope")
        for k, frac in enumerate((0.34, 0.67, 1.0)):
            w(0.6, None, _tilt_pos(frac), _tilt_rot(frac), shut, f"lift its near end {k + 1}")
        w(2.5, None, lambda s: _tilted(s, TILT, DRAW_OUT, 0.0)[0], lambda s: _tilted(s, TILT, DRAW_OUT, 0.0)[1], shut, "draw it out of the mailbox")
        w(1.0, None, lambda s: _tilted(s, TILT, DRAW_OUT, LIFT_OUT)[0], lambda s: _tilted(s, TILT, DRAW_OUT, LIFT_OUT)[1], shut, "lift it clear")
    # 4. Into the tray.
    if "tray" in steps:
        w(2.5, None, lambda s: _in_tray(s, TRAY_DROP + 0.050, tray_at)[0], lambda s: _in_tray(s, 0.0)[1], shut, "over the tray")
        w(1.2, None, lambda s: _in_tray(s, TRAY_DROP, tray_at)[0], lambda s: _in_tray(s, 0.0)[1], shut, "lower it into the tray")
        w(0.8, None, lambda s: _in_tray(s, TRAY_DROP, tray_at)[0], lambda s: _in_tray(s, 0.0)[1], _finger(0.030), "let go")
        w(1.2, None, lambda s: _in_tray(s, TRAY_DROP + 0.080, tray_at)[0] - 0.040 * _in_tray(s, 0.0)[1][:, 2], lambda s: _in_tray(s, 0.0)[1], _finger(0.030), "lift away")
    # 5. The door lifted shut from under its free edge, into the latch.
    if "shut" in steps:
        def door_g(s: Servo) -> np.ndarray:
            return door_grip(side, s.door_phi)

        # come at it low, in front of its free edge, then in under it (GPU run F: coming down from above, the hand
        # landed on the open door's top and pressed it onto its stop)
        w(2.5, DOOR_UNDER, lambda s: door_hand(b, side, s.door_phi + 12.0, back=0.040) - _v((DOOR_FRONT, 0.0, 0.0)), lambda s: door_grip(side, s.door_phi + 12.0), shut, "low in front of the door")
        w(1.5, DOOR_UNDER, lambda s: door_hand(b, side, s.door_phi + 12.0, back=0.040), lambda s: door_grip(side, s.door_phi + 12.0), shut, "in under the door")
        w(1.5, DOOR_UNDER, lambda s: door_hand(b, side, s.door_phi + 3.0), lambda s: door_grip(side, s.door_phi + 3.0), shut, "fingertips under its edge")
        w(0.1, DOOR_UP, lambda s: door_hand(b, side, s.door_phi), door_g, shut, "lift the door shut")
        w(12.0, DOOR_UP, lambda s: door_hand(b, side, s.door_phi), door_g, shut, "lift the door shut")
        w(1.0, None, lambda s: door_hand(b, side, max(s.door_phi, 0.0), back=0.050) - _v((0.0, 0.0, 0.020)), door_g, shut, "draw back from the door")
    # 6. The flag pushed down.
    if "flag" in steps:
        def flag_g(s: Servo) -> np.ndarray:
            return flag_grip(side, s.flag_deg)

        w(2.5, None, lambda s: flag_tcp(b, side, FLAG_UP_START + 4.0, back=0.040), flag_g, shut, "to the flag")
        w(1.0, FLAG_PUSH, lambda s: flag_tcp(b, side, s.flag_deg), flag_g, shut, "fingertips on the flag")
        w(4.0, FLAG_PUSH, lambda s: flag_tcp(b, side, s.flag_deg), flag_g, shut, "push the flag down")
        w(0.8, None, lambda s: flag_tcp(b, side, s.flag_deg) + _v((-0.050, 0.0, 0.0)), flag_g, shut, "back off the flag")
        w(0.8, None, lambda s: flag_tcp(b, side, s.flag_deg) + _v((-0.070, 0.0, 0.050)), flag_g, shut, "up off the flag")
        w(1.5, None, lambda s: _flag_retreat(b), flag_g, _finger(0.04), "back toward the robot")
    if "park" in steps:
        park, park_rot = _park(side)
        plan.append(Waypoint(2.5, park, _finger(0.04), "park", Grip(park_rot)))
    return plan


def _pad(plan: list[Waypoint], until: float, label: str) -> list[Waypoint]:
    total = sum(w.duration for w in plan)
    if until > total + 1e-9:
        last = plan[-1]
        plan = [*plan, Waypoint(until - total, last.pos, last.finger, label, last.yaw)]
    return plan


END_HOLD = 2.0
SERVOS: dict[str, Servo] = {}  # the last plan's servos, per arm (traces read them)


def demo() -> dict[str, list[Waypoint]]:
    install_grip_rotation()
    worker = ARM
    idle = "left" if worker == "right" else "right"
    sv = Servo(worker, FLAGGED)
    plans = {worker: routine(sv)}
    park, park_rot = _park(idle)
    plans[idle] = [Waypoint(3.0, park, _finger(0.04), "watch", Grip(park_rot))]
    end = max(sum(w.duration for w in p) for p in plans.values()) + END_HOLD
    plans = {side: _pad(p, end, "hold still") for side, p in plans.items()}
    sv.follow(plans[worker])
    SERVOS.clear()
    SERVOS[worker] = sv
    return {"left": plans["left"], "right": plans["right"]}


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
