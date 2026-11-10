"""On the station table, in front of the arms: a flat 30 x 40 cm panel, 1.2 cm thick and fixed to the table,
long side running left to right. A coworker has started painting it blue: a 4 cm border all the way round,
and the whole left end of the panel, 15 cm of it. The rest, a bare off-white rectangle 22 cm deep and 21 cm
wide, still needs paint, and the coworker also missed a few small patches in their own paint at the left end.
To the right of the panel, in front of the right arm, a 4-inch mini paint roller lies on a grey drip tray: a
blue foam cover loaded with paint, 10 cm long and 4 cm across, its axle running front to back, on a thin metal
frame whose yellow plastic handle points toward the right arm and rests on a small block.

The cover spins freely on its axle. It paints the panel along the line where it rolls on it, a 10 cm wide
strip, and wet paint shows up on the panel as it is applied. Only the cover paints the panel; the frame, the
handle and the grippers never do. Any part of the roller touching the table counts as paint on the table.
"""

# No `from __future__ import annotations` here: the task loader imports this file without registering it in
# sys.modules, and dataclasses then cannot resolve string annotations.
import functools
import importlib
import importlib.abc
import importlib.machinery
import importlib.util
import itertools
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
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP: float = TABLE_TOP[SCENE]

# --- layout (world frame: x away from the arms, y to the left, z up; metres) --------------------------------
PANEL_CENTER = (
    0.235,
    -0.02,
)  # clear of the camera stand's foot, inside the right arm's straight-down reach
PANEL_SIZE = (0.30, 0.40)  # x (depth), y (width)
PANEL_THICKNESS = 0.012
PANEL_TOP = TOP + PANEL_THICKNESS
# The bare area, in the panel frame (origin at the panel's centre, axes along the world's): a 4 cm border all
# round, and the left 15 cm of the panel (that side's border included). x_min, x_max, y_min, y_max.
TARGET = (-0.11, 0.11, -0.16, 0.05)
# Patches the coworker missed in their own paint at the left end ("holidays", in coating inspection): 6 cm
# along x and 3 cm along y, at least 3 cm from the bare area, from each other and from the panel's edge, all
# within the right arm's reach. 6 cm so the roller's 10 cm cover paints one with its ends inside the free
# overlap band (no waste when centred within 5 mm); the roller rolls across a patch's 3 cm.
HOLIDAYS = (
    (-0.10, -0.04, 0.09, 0.12),
    (-0.01, 0.05, 0.14, 0.17),
    (0.04, 0.10, 0.08, 0.11),
)
PANEL_RGB = (0.93, 0.92, 0.89)  # off-white primer
PAINT_RGB = (0.16, 0.36, 0.78)  # blue
HANDLE_RGB = (0.95, 0.72, 0.10)  # yellow plastic
METAL_RGB = (0.62, 0.64, 0.66)  # zinc-plated steel wire
TRAY_RGB = (0.45, 0.46, 0.48)
TRAY_SIZE = (0.14, 0.20, 0.01)
TRAY_CENTER = (0.20, -0.33)
# The station's table (Dream Labs' replica): 1.10 x 1.30 m, centred at x 0.36, y 0.
TABLE_X = (0.36 - 0.55, 0.36 + 0.55)
TABLE_Y = (-0.65, 0.65)

# --- the roller: a 4-inch mini trim roller, built from a cylinder and boxes ----------------------------------
# The roller frame: origin at the cover's centre, x along the axle, the handle pointing to -y, z up.
COVER_RADIUS = 0.020  # a 40 mm foam cover
COVER_LENGTH = 0.10  # 4 inches
COVER_MASS = 0.02
FRAME_MASS = 0.07  # handle and wire frame together
HANDLE_SIZE = (0.024, 0.110, 0.024)  # x, y (along the handle), z
HANDLE_CENTER = (
    0.0,
    -0.095,
    0.025,
)  # the handle runs from y -0.040 to -0.150, 2.5 cm above the axle
GRIP = (0.0, -0.085, 0.025)  # where the demo holds the handle: 1 cm in front of its middle
WIRE = 0.005  # the frame's square wire
_ARM_X = COVER_LENGTH / 2 + 0.0055  # the side arms run just outside the cover's ends
_ARM_RISE = (
    0.040,
    0.025,
)  # each arm runs 4 cm back (-y) and 2.5 cm up, from the axle to the cross bar
_ARM_LENGTH = math.hypot(*_ARM_RISE)
_ARM_ROLL = -math.atan2(
    _ARM_RISE[1], _ARM_RISE[0]
)  # turn about x taking a box's +y onto the arm's direction
# The frame's parts besides the handle: (name, centre, size, turn about x in rad), in the roller frame. All are
# one rigid body with the handle; the cover is a second body, joined to it by a revolute joint on the axle.
FRAME_PARTS = (
    (
        "arm_far",
        (_ARM_X, -_ARM_RISE[0] / 2, _ARM_RISE[1] / 2),
        (WIRE, _ARM_LENGTH, WIRE),
        _ARM_ROLL,
    ),
    (
        "arm_near",
        (-_ARM_X, -_ARM_RISE[0] / 2, _ARM_RISE[1] / 2),
        (WIRE, _ARM_LENGTH, WIRE),
        _ARM_ROLL,
    ),
    ("cross_bar", (0.0, -_ARM_RISE[0], _ARM_RISE[1]), (2 * _ARM_X + WIRE, WIRE, WIRE), 0.0),
    ("axle_far", (COVER_LENGTH / 2 + 0.004, 0.0, 0.0), (0.008, 0.004, 0.004), 0.0),
    ("axle_near", (-COVER_LENGTH / 2 - 0.004, 0.0, 0.0), (0.008, 0.004, 0.004), 0.0),
)
# At rest: the cover on the tray (spawned 1 mm above it), the handle's far end on a small block, so the handle is
# level and the roller cannot tip or roll off.
COVER_START = (TRAY_CENTER[0], -0.255, TOP + TRAY_SIZE[2] + COVER_RADIUS + 0.001)
_HANDLE_END_Y = COVER_START[1] + HANDLE_CENTER[1] - HANDLE_SIZE[1] / 2
REST_SIZE = (0.04, 0.025, 0.033)  # its top 1 mm under the handle as spawned

# --- painting rules ------------------------------------------------------------------------------------------
CELL = 0.01  # the panel is scored as a 30 x 40 grid of 1 cm cells
# A short overlap into the coworker's paint is free: painters blend new paint into the edge they meet, but
# rolling far over paint that has already dried leaves lap marks and wastes paint. 2 cm, the task's own value.
WET_BAND = 0.02
CONTACT_GAP = (
    0.0015  # the cover's lowest line at most 1.5 mm above the panel top counts as touching it
)
BELOW_LIMIT = 0.003  # ... and at most 3 mm below it (pressed into the foam)
CONTACT_HALF_WIDTH = CELL / 2  # the strip it wets: cells whose centres lie within 5 mm of that line
MAX_TILT = math.radians(10.0)  # the axle must be within 10 degrees of level
MAX_SWEEP = 0.04  # fill in cells between consecutive touching steps whose cover centres are up to 4 cm apart
SWEEP_SAMPLES = 8
COVERAGE_STEPS = ((0.98, 0.8), (0.95, 0.6), (0.80, 0.4), (0.50, 0.2))
# Wasted cells: the coworker's cells beyond the free band that the robot painted, counted, not as a share.
EXCESS_STEPS = ((20, 1.0), (60, 0.75), (120, 0.5))
EXCESS_FLOOR = 0.0  # more than 120 wasted cells (blanket painting) scores 0
BENCH_CAP = 0.5
HOLD_STEPS = 20  # 1 s at 20 Hz: a full score must have held this long before score() reports 1.0
HELD_BACK = 0.8  # what score() reports for a full score that has not held yet
SCORE_OVER = "end"


def _box(
    name: str, x0: float, x1: float, y0: float, y1: float, rgb: tuple[float, float, float]
) -> Box:
    """A fixed 12 mm slab of the panel, given in the panel frame."""
    cx, cy = PANEL_CENTER
    return Box(
        name,
        pos=(cx + (x0 + x1) / 2, cy + (y0 + y1) / 2, TOP + PANEL_THICKNESS / 2),
        size=(x1 - x0, y1 - y0, PANEL_THICKNESS),
        color=rgb,
    )


_HX, _HY = PANEL_SIZE[0] / 2, PANEL_SIZE[1] / 2
_TX0, _TX1, _TY0, _TY1 = TARGET


def _left_end_slabs() -> list[Box]:
    """The coworker's paint at the left end (between the near and far borders), tiled around the missed
    patches: one column of slabs per run of x between patch edges, cut around the patches in that column."""
    edges = sorted({_TX0, _TX1, *(h[0] for h in HOLIDAYS), *(h[1] for h in HOLIDAYS)})
    slabs = []
    for x0, x1 in itertools.pairwise(edges):
        holes = sorted(
            (h for h in HOLIDAYS if h[0] <= x0 + 1e-9 and h[1] >= x1 - 1e-9), key=lambda h: h[2]
        )
        y = _TY1
        for hole in holes:
            slabs.append((x0, x1, y, hole[2]))
            y = hole[3]
        slabs.append((x0, x1, y, _HY))
    return [
        _box(f"paint_left_{i}", x0, x1, y0, y1, PAINT_RGB)
        for i, (x0, x1, y0, y1) in enumerate(slabs)
    ]


# The panel is flush slabs: the bare area, the missed patches, and the coworker's paint around them.
PANEL_PARTS = [
    _box("panel_bare", _TX0, _TX1, _TY0, _TY1, PANEL_RGB),
    *(
        _box(f"missed_spot_{k + 1}", x0, x1, y0, y1, PANEL_RGB)
        for k, (x0, x1, y0, y1) in enumerate(HOLIDAYS)
    ),
    _box("paint_near_edge", -_HX, _TX0, -_HY, _HY, PAINT_RGB),
    _box("paint_far_edge", _TX1, _HX, -_HY, _HY, PAINT_RGB),
    _box("paint_right_edge", _TX0, _TX1, -_HY, _TY0, PAINT_RGB),
    *_left_end_slabs(),
]
TRAY = Box(
    "drip_tray",
    pos=(TRAY_CENTER[0], TRAY_CENTER[1], TOP + TRAY_SIZE[2] / 2),
    size=TRAY_SIZE,
    color=TRAY_RGB,
)
ROLLER_REST = Box(
    "roller_rest",
    pos=(COVER_START[0], _HANDLE_END_Y + REST_SIZE[1] / 2, TOP + TRAY_SIZE[2] + REST_SIZE[2] / 2),
    size=REST_SIZE,
    color=TRAY_RGB,
)
COVER = "roller_cover"
FRAME = "roller_frame"
# The cover is an upright cylinder in the task-file API; the scene hook below lays its axis along x.
ROLLER_COVER = Cylinder(
    COVER,
    pos=COVER_START,
    radius=COVER_RADIUS,
    height=COVER_LENGTH,
    mass=COVER_MASS,
    color=PAINT_RGB,
)
# The handle is the frame body's own box; the hook adds the wire parts to it and the axle joint.
ROLLER_FRAME = Box(
    FRAME,
    pos=tuple(c + h for c, h in zip(COVER_START, HANDLE_CENTER, strict=True)),
    size=HANDLE_SIZE,
    mass=FRAME_MASS,
    color=HANDLE_RGB,
)
OBJECTS = [
    *PANEL_PARTS,
    TRAY,
    ROLLER_REST,
    ROLLER_COVER,
    ROLLER_FRAME,
]  # the frame last: it hosts the joint


# --- the roller's joint and wire frame: the task-file API has neither -----------------------------------------
# yam_arena.scene.task_object(spec) turns each task object into an Arena object. Right after that module is
# imported, this wraps task_object, by object name: the cover's cylinder is spawned lying along x (Isaac Lab's
# CylinderCfg.axis), and the frame's spawner, after spawning the handle, adds the wire parts as more colliders
# of the same rigid body and a UsdPhysics.RevoluteJoint on the axle between the frame and the cover. Jointed
# bodies don't collide with each other (UsdPhysics default), so the cover spins freely inside the frame. On a
# machine without the simulator nothing imports yam_arena.scene, and this does nothing.
_SCENE_MODULE = "yam_arena.scene"
AXLE = "roller_axle"
PARTS_FOLDER = "frame_parts"


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def author_roller(stage: Any, env_root: str) -> list[str]:
    """Add the wire parts to the frame body and the axle joint under ``env_root``; the paths it made."""
    gf = importlib.import_module("pxr.Gf")
    sdf = importlib.import_module("pxr.Sdf")
    geom = importlib.import_module("pxr.UsdGeom")
    physics = importlib.import_module("pxr.UsdPhysics")
    shade = importlib.import_module("pxr.UsdShade")
    frame_path = f"{env_root}/{FRAME}"
    metal_path = f"{frame_path}/{PARTS_FOLDER}/metal"
    material = shade.Material.Define(stage, metal_path)
    shader = shade.Shader.Define(stage, f"{metal_path}/shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", sdf.ValueTypeNames.Color3f).Set(
        gf.Vec3f(*(_linear(c) for c in METAL_RGB))
    )
    shader.CreateInput("metallic", sdf.ValueTypeNames.Float).Set(0.9)
    shader.CreateInput("roughness", sdf.ValueTypeNames.Float).Set(0.35)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    made = [metal_path]
    for name, centre, size, roll in FRAME_PARTS:
        path = f"{frame_path}/{PARTS_FOLDER}/{name}"
        cube = geom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        xform = geom.Xformable(cube.GetPrim())
        local = [c - h for c, h in zip(centre, HANDLE_CENTER, strict=True)]
        xform.AddTranslateOp().Set(gf.Vec3d(*local))
        xform.AddOrientOp().Set(gf.Quatf(math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0))
        xform.AddScaleOp().Set(gf.Vec3f(*size))
        physics.CollisionAPI.Apply(cube.GetPrim())
        shade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
        made.append(path)
    joint_path = f"{env_root}/{AXLE}"
    joint = physics.RevoluteJoint.Define(stage, joint_path)
    joint.CreateAxisAttr("X")
    joint.CreateBody0Rel().SetTargets([frame_path])
    joint.CreateBody1Rel().SetTargets([f"{env_root}/{COVER}"])
    joint.CreateLocalPos0Attr(
        gf.Vec3f(*(-h for h in HANDLE_CENTER))
    )  # the axle's centre, seen from the handle
    joint.CreateLocalRot0Attr(gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(gf.Quatf(1.0, 0.0, 0.0, 0.0))
    made.append(joint_path)
    return made


def _env_roots(prim: Any) -> list[str]:
    """Every environment folder holding the frame, given the frame prim just spawned (cloned copies included)."""
    stage = prim.GetStage()
    envs = prim.GetPath().GetParentPath().GetParentPath()
    return [
        str(env.GetPath())
        for env in stage.GetPrimAtPath(envs).GetChildren()
        if env.GetChild(FRAME).IsValid()
    ]


def _with_roller(spawn: Callable[..., Any]) -> Callable[..., Any]:
    def spawn_and_build(prim_path: str, cfg: Any, *args: Any, **kwargs: Any) -> Any:
        prim = spawn(prim_path, cfg, *args, **kwargs)
        try:
            for root in _env_roots(prim):
                author_roller(prim.GetStage(), root)
        except (
            Exception
        ) as error:  # a roller without its axle falls apart; scoring then sees no flat cover
            _warn_once("roller", f"could not build the roller ({type(error).__name__}: {error})")
        return prim

    spawn_and_build.paint_roller = True  # type: ignore[attr-defined]
    return spawn_and_build


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


def build_roller(scene: Any) -> bool:
    """Wrap ``scene.task_object`` so the roller's two objects come out jointed; True if it wrapped it now.
    It dispatches on the objects' names, so a second load of this file is served by the first wrapper."""
    original = getattr(scene, "task_object", None)
    if original is None or getattr(original, "paint_roller", False):
        return False

    def task_object(spec: Any) -> Any:
        made = original(spec)
        name = getattr(spec, "name", None)
        if name in (COVER, FRAME):
            found = 0
            for cfg in _spawner_cfgs(made):
                found += 1
                if name == COVER:
                    if hasattr(cfg, "axis"):
                        cfg.axis = "X"
                    else:
                        _warn_once(
                            "cover-axis",
                            "the cover's spawner has no axis setting; the roller will be wrong",
                        )
                elif name == FRAME and not getattr(cfg.func, "paint_roller", False):
                    cfg.func = _with_roller(cfg.func)
            if not found:
                _warn_once("roller", f"found no spawner for {name}; the roller will be wrong")
        return made

    task_object.paint_roller = True  # type: ignore[attr-defined]
    scene.task_object = task_object
    return True


class _RollerWatcher(importlib.abc.MetaPathFinder):
    """Waits for yam_arena.scene to be imported, lets it load normally, then applies build_roller. It answers
    for that one module name only, and removes itself once the module has loaded."""

    marker = "finish_painting_panel.roller_watcher"

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
    """The module's own loader, plus build_roller once the module has run."""

    def __init__(self, inner: importlib.abc.Loader, watcher: _RollerWatcher) -> None:
        self._inner = inner
        self._watcher = watcher

    def create_module(self, spec: importlib.machinery.ModuleSpec) -> types.ModuleType | None:
        return self._inner.create_module(spec)

    def exec_module(self, module: types.ModuleType) -> None:
        self._inner.exec_module(module)
        build_roller(module)
        self._watcher.disarm()

    def __getattr__(self, name: str) -> Any:  # get_source and friends, for tracebacks
        if name.startswith("__") or name in ("_inner", "_watcher"):  # a copy made without __init__
            raise AttributeError(name)
        return getattr(self._inner, name)


def install_roller() -> None:
    """Wrap the scene builder now if it is already imported; otherwise wait for its import (once)."""
    scene = sys.modules.get(_SCENE_MODULE)
    if scene is not None:
        build_roller(scene)
        return
    if not any(
        getattr(finder, "marker", None) == _RollerWatcher.marker for finder in sys.meta_path
    ):
        sys.meta_path.insert(0, _RollerWatcher())


# --- the paint grid --------------------------------------------------------------------------------------------
NX, NY = round(PANEL_SIZE[0] / CELL), round(PANEL_SIZE[1] / CELL)
_gx, _gy = np.meshgrid(
    -_HX + CELL * (np.arange(NX) + 0.5), -_HY + CELL * (np.arange(NY) + 0.5), indexing="ij"
)
CELLS = np.stack([_gx.ravel(), _gy.ravel()], axis=1)  # cell centres, panel frame; index i * NY + j
_CELLS_WORLD = CELLS + np.asarray(PANEL_CENTER)


def _distance_to_rect(xy: np.ndarray, rect: tuple[float, float, float, float]) -> np.ndarray:
    x0, x1, y0, y1 = rect
    dx = np.maximum(np.maximum(x0 - xy[:, 0], xy[:, 0] - x1), 0.0)
    dy = np.maximum(np.maximum(y0 - xy[:, 1], xy[:, 1] - y1), 0.0)
    distance: np.ndarray = np.hypot(dx, dy)
    return distance


def _distance_to_target(xy: np.ndarray) -> np.ndarray:
    """Distance (m) from each point to the nearest bare area: the big rectangle or a missed patch."""
    distance: np.ndarray = np.min(
        [_distance_to_rect(xy, rect) for rect in (TARGET, *HOLIDAYS)], axis=0
    )
    return distance


_D = _distance_to_target(CELLS)
IS_TARGET: np.ndarray = _D == 0.0
IS_HOLIDAY: np.ndarray = IS_TARGET & (_distance_to_rect(CELLS, TARGET) > 0.0)
IS_WET_BAND: np.ndarray = ~IS_TARGET & (_D <= WET_BAND + 1e-9)
IS_EXCESS: np.ndarray = ~IS_TARGET & ~IS_WET_BAND
N_TARGET = int(IS_TARGET.sum())  # 516: 462 in the big rectangle, 18 in each missed patch
N_COWORKER = int((~IS_TARGET).sum())  # 684
N_EXCESS = int(IS_EXCESS.sum())


def rotation(quat_xyzw: Any) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def roller_contact(pos: np.ndarray, rot: np.ndarray) -> tuple[float, np.ndarray]:
    """(tilt, cells) for the cover: its axle's tilt from level (rad) and, per panel cell, whether the cell lies
    under the cover's lowest line (its centre within CONTACT_HALF_WIDTH of it, along the cover's 10 cm) with
    that line between BELOW_LIMIT below and CONTACT_GAP above the panel top there.

    ``rot`` is the cover's own rotation; its x axis is the axle, so spinning about it changes nothing here: a
    rolling cover and a skidding one paint the same strip. Heights are taken along the line, so a cover with
    one end lifted paints only the part that touches."""
    axle = rot[:, 0]
    tilt = math.asin(min(1.0, abs(float(axle[2]))))
    if tilt >= MAX_TILT:
        return tilt, np.zeros(len(CELLS), dtype=bool)
    up = np.array([0.0, 0.0, 1.0]) - float(axle[2]) * axle
    base = pos - COVER_RADIUS * up / np.linalg.norm(up)  # a point on the lowest line
    a_xy = axle[:2]
    rel = _CELLS_WORLD - base[:2]
    s = rel @ a_xy / float(a_xy @ a_xy)  # position along the axle of the line point over each cell
    across = rel - np.outer(s, a_xy)
    inside = (np.abs(s) <= COVER_LENGTH / 2 + 1e-9) & (
        np.hypot(across[:, 0], across[:, 1]) <= CONTACT_HALF_WIDTH + 1e-9
    )
    height = base[2] + s * float(axle[2]) - PANEL_TOP
    touching: np.ndarray = inside & (height <= CONTACT_GAP) & (height >= -BELOW_LIMIT)
    return tilt, touching


def _box_corners(centre: Sequence[float], size: Sequence[float], roll: float) -> np.ndarray:
    half = np.asarray(size) / 2
    c, s = math.cos(roll), math.sin(roll)
    turn = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    signs = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)], dtype=float)
    corners: np.ndarray = (signs * half) @ turn.T + np.asarray(centre)
    return corners


# Every corner of the frame body (handle and wire parts), in the roller frame.
FRAME_CORNERS = np.vstack(
    [
        _box_corners(HANDLE_CENTER, HANDLE_SIZE, 0.0),
        *(_box_corners(c, size, roll) for _, c, size, roll in FRAME_PARTS),
    ]
)


def roller_lowest(cover_pos: np.ndarray, cover_rot: np.ndarray, frame_rot: np.ndarray) -> float:
    """Height of the roller's lowest point (any part), from the two bodies' poses and the roller's sizes, not
    from bounding boxes. The frame is placed from the cover's centre, which the axle joint keeps fixed in it."""
    a_z = float(cover_rot[2, 0])
    cover_low = (
        cover_pos[2]
        - COVER_LENGTH / 2 * abs(a_z)
        - COVER_RADIUS * math.sqrt(max(0.0, 1.0 - a_z * a_z))
    )
    frame_low = float((cover_pos + FRAME_CORNERS @ frame_rot.T)[:, 2].min())
    return float(min(cover_low, frame_low))


def on_bench(pos: np.ndarray, lowest_z: float, touching_panel: bool) -> bool:
    """Paint on the table: any part of the roller within CONTACT_GAP of the table top, over the table, while
    the cover is not on the panel."""
    over_table = TABLE_X[0] <= pos[0] <= TABLE_X[1] and TABLE_Y[0] <= pos[1] <= TABLE_Y[1]
    return bool(over_table and abs(lowest_z - TOP) <= CONTACT_GAP and not touching_panel)


@dataclass
class Breakdown:
    covered: int
    coverage: float
    base: float
    wasted: int
    multiplier: float
    bench: bool
    score: float  # base x multiplier, then the table cap; before the 1 s hold (see held_score)


def score_ledger(painted: np.ndarray, robot_paint: np.ndarray, bench: bool) -> Breakdown:
    """The score, from which cells carry paint and which of those the robot put there."""
    covered = int((painted & IS_TARGET).sum())
    coverage = covered / N_TARGET
    base = 1.0 if covered >= N_TARGET else 0.0
    if covered < N_TARGET:
        for threshold, value in COVERAGE_STEPS:
            if coverage >= threshold - 1e-12:
                base = value
                break
    wasted = int((robot_paint & IS_EXCESS).sum())
    multiplier = EXCESS_FLOOR
    for limit, value in EXCESS_STEPS:
        if wasted <= limit:
            multiplier = value
            break
    total = base * multiplier
    if bench:
        total = min(total, BENCH_CAP)
    return Breakdown(covered, coverage, base, wasted, multiplier, bench, total)


def held_score(raw: float, full_steps: int) -> float:
    """A full score counts once it has held for HOLD_STEPS control steps (1 s); until then it reports
    HELD_BACK, so a trial that only reaches it in its last second does not score 1.0."""
    if raw >= 1.0 - 1e-9 and full_steps < HOLD_STEPS:
        return HELD_BACK
    return raw


# --- one episode's paint ---------------------------------------------------------------------------------------


@dataclass
class Ledger:
    painted: np.ndarray = field(default_factory=lambda: ~IS_TARGET.copy())
    robot_paint: np.ndarray = field(default_factory=lambda: np.zeros(len(CELLS), dtype=bool))
    bench: bool = False
    last_pos: np.ndarray | None = None  # cover centre at the last step that touched the panel
    steps: int = 0
    full_steps: int = 0  # consecutive steps at full score (before the hold), counting this one

    def apply(self, pos: np.ndarray, rot: np.ndarray, frame_rot: np.ndarray) -> None:
        """One control step of the roller: the cover's centre and rotation, and the frame's rotation (world
        frame, env-relative)."""
        self.steps += 1
        _, hits = roller_contact(pos, rot)
        touching = bool(hits.any())
        if on_bench(pos, roller_lowest(pos, rot, frame_rot), touching):
            self.bench = True
        if (
            touching
            and self.last_pos is not None
            and np.linalg.norm(pos - self.last_pos) <= MAX_SWEEP
        ):
            for k in range(1, SWEEP_SAMPLES):
                s = k / SWEEP_SAMPLES
                hits |= roller_contact(self.last_pos + s * (pos - self.last_pos), rot)[1]
        self.painted |= hits
        self.robot_paint |= hits
        self.last_pos = pos.copy() if touching else None
        full = score_ledger(self.painted, self.robot_paint, self.bench).score >= 1.0 - 1e-9
        self.full_steps = self.full_steps + 1 if full else 0


_LEDGER = Ledger()
_last_objects: object = None  # the step's state dict, kept alive so its id can't be reused
_hooked = False
_hooked_to: Any = None  # the embodiment whose reset is wrapped
_warned: set[str] = set()
RESET_TAG = "finish_painting_panel_new_episode"  # on a wrapped reset: the new_episode it calls


def _warn_once(key: str, message: str) -> None:
    if key not in _warned:
        _warned.add(key)
        warnings.warn(f"finish_painting_panel: {message}", RuntimeWarning, stacklevel=3)


def new_episode() -> None:
    """Start an empty ledger and clear the wet-paint tiles."""
    global _LEDGER, _last_objects
    _LEDGER = Ledger()
    _last_objects = None
    _TILES.hide()


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
    """Task files get no reset hook, so wrap the calling embodiment's reset: every new trial then starts a fresh
    ledger and hides the last trial's tiles before its first frame is rendered. The wrapper keeps the original's
    name and signature and is tagged, so this file never wraps the same reset twice; a different embodiment is
    hooked in turn. If no embodiment can be found, fall back to treating the roller resting at its start pose as
    a new trial."""
    global _hooked, _hooked_to
    embodiment = _calling_embodiment()
    if embodiment is None:
        if not _hooked:
            _warn_once(
                "reset",
                "could not find the embodiment's reset; a roller resting at its start pose starts a new trial",
            )
        return
    if embodiment is _hooked_to:
        return
    original = embodiment.reset
    if getattr(original, RESET_TAG, None) is new_episode:  # already wrapped by this file
        _hooked, _hooked_to = True, embodiment
        return

    @functools.wraps(original)
    def reset(*args: Any, **kwargs: Any) -> Any:
        new_episode()
        return original(*args, **kwargs)

    setattr(reset, RESET_TAG, new_episode)
    try:
        embodiment.reset = reset
        _hooked, _hooked_to = True, embodiment
    except (AttributeError, TypeError):
        _warn_once(
            "reset",
            "could not hook the embodiment's reset; a roller resting at its start pose starts a new trial",
        )


def _update(objects: Mapping[str, Any]) -> Ledger:
    """Advance the ledger once per control step (success() and score() share the step's state dict)."""
    global _last_objects
    if objects is _last_objects:
        return _LEDGER
    try:
        _hook_resets()
    except Exception as error:  # an odd caller on the stack: carry on without the hook
        _warn_once("hook", f"could not look for the embodiment ({type(error).__name__}: {error})")
    cover, frame = objects[COVER], objects[FRAME]
    pos = np.asarray(cover.pos, dtype=float)
    back_at_start = np.linalg.norm(pos - np.asarray(COVER_START)) < 0.003
    resting = np.linalg.norm(np.asarray(cover.lin_vel, dtype=float)) < 0.01
    if not _hooked and back_at_start and resting and _LEDGER.steps > 0:
        new_episode()
    _last_objects = objects
    _LEDGER.apply(pos, rotation(cover.quat_xyzw), rotation(frame.quat_xyzw))
    _TILES.draw(objects, _LEDGER.robot_paint & IS_TARGET)
    return _LEDGER


# --- wet paint the cameras can see ---------------------------------------------------------------------------


class _Tiles:
    """Painted cells drawn as thin tiles with Isaac Lab's VisualizationMarkers, so camera-based policies see their
    own paint. Imported lazily: the simulator is running by the time score() is first called. The tiles are only
    a picture of the ledger: if drawing fails for any reason, they turn off with one warning and scoring goes on.
    Outside the simulator (unit tests), nothing is drawn."""

    def __init__(self) -> None:
        self._markers: Any = None
        self._origin = np.zeros(3)
        self._shown = -1
        self._off = False

    def hide(self) -> None:
        if self._markers is None or self._off:
            return
        try:
            self._markers.set_visibility(False)
            self._shown = 0
        except Exception as error:  # cosmetic only: never abort a trial over the picture
            self._disable(error)

    def draw(self, objects: Mapping[str, Any], cells: np.ndarray) -> None:
        if self._off:
            return
        count = int(cells.sum())
        if count == self._shown:
            return
        try:
            if self._markers is None:
                origin = self._env_origin(objects)
                self._markers = self._make()
                self._origin = origin
            self._shown = count
            if count == 0:
                self._markers.set_visibility(False)
                return
            xy = CELLS[cells] + np.asarray(PANEL_CENTER)
            z = np.full((count, 1), PANEL_TOP + 0.0007)
            points = np.hstack([xy, z]) + self._origin
            quats = np.tile([0.0, 0.0, 0.0, 1.0], (count, 1))
            self._markers.set_visibility(True)
            self._markers.visualize(
                translations=points.astype(np.float32), orientations=quats.astype(np.float32)
            )
        except ImportError:
            self._off = True  # no simulator: unit tests
        except Exception as error:  # cosmetic only: never abort a trial over the picture
            self._disable(error)

    def _disable(self, error: Exception) -> None:
        self._off = True
        _warn_once(
            "tiles",
            f"wet-paint tiles turned off ({type(error).__name__}: {error}); scoring is unaffected",
        )

    @staticmethod
    def _make() -> Any:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        # Tiles overlap their neighbours slightly, so no seams show between painted cells.
        tile = sim_utils.CuboidCfg(
            size=(CELL * 1.02, CELL * 1.02, 0.0005),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=tuple(_linear(c) for c in PAINT_RGB), roughness=0.4
            ),
        )
        return VisualizationMarkers(
            VisualizationMarkersCfg(prim_path="/Visuals/wet_paint", markers={"paint": tile})
        )

    @staticmethod
    def _env_origin(objects: Mapping[str, Any]) -> np.ndarray:
        """World offset of the env (object states are env-relative; markers live in world coordinates)."""
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        cache = UsdGeom.XformCache()
        for prim in stage.Traverse():
            if prim.GetName() == "panel_bare":
                world = np.array(
                    cache.GetLocalToWorldTransform(prim).ExtractTranslation(), dtype=float
                )
                offset: np.ndarray = world - np.asarray(objects["panel_bare"].pos, dtype=float)
                return offset
        return np.zeros(3)


_TILES = _Tiles()


# --- what the embodiment calls ---------------------------------------------------------------------------------


def score(objects: Mapping[str, Any]) -> float:
    """1.0 for every bare cell painted (missed patches included) with little wasted overlap, held for 1 s;
    see README.md for the ladder."""
    ledger = _update(objects)
    raw = score_ledger(ledger.painted, ledger.robot_paint, ledger.bench).score
    return held_score(raw, ledger.full_steps)


def success(objects: Mapping[str, Any]) -> bool:
    """Full score: every bare cell painted, at most 20 wasted cells, no paint on the table. The embodiment
    ends the trial once this has held for 1 s."""
    ledger = _update(objects)
    return score_ledger(ledger.painted, ledger.robot_paint, ledger.bench).score >= 1.0 - 1e-9


# --- the scripted demo -----------------------------------------------------------------------------------------

GRIP_YAW = math.pi / 2  # fingers close along world x, across the handle's 24 mm width
# The cover's lowest line rides 0.3 mm above the panel (inside the 1.5 mm contact rule), as the pad did; the
# cover rolls, so contact drags far less than a pad.
PRESS = -0.0003
ROLL_SPEED = 0.05  # m/s
HOVER = 0.03
LANE_X = (-0.065, 0.0, 0.065)  # panel frame: the cover's centre on three overlapping 10 cm lanes
LANE_Y = (
    0.06,
    -0.17,
)  # the cover rolls from the bare rectangle's left (+y) edge to past its right edge
LANE_ORDER = (0, 1, 2)
OVERRUN = 0.005  # a patch is rolled from 5 mm before it to 5 mm past it (1 cm would wet corner cells 2.1 cm out)
TOUCH_UP_ORDER = (2, 1, 0)  # the missed patches first, while nothing else has been rolled
# Measured: the roller turns 4 to 5 degrees in the grip as it is lifted, and with the cover 8.5 cm in front of the
# grip that puts the cover 7 to 11 mm toward -x of where it is commanded at the start of a roll (easing to 0 to 5
# mm along it). Every roll is aimed this much toward +x to take it out.
ROLL_TRIM_X = 0.008


def grip_for(cover: Sequence[float]) -> tuple[float, float, float]:
    """The gripper's point (TCP) that holds the cover's centre at ``cover``, handle level."""
    return (cover[0] + GRIP[0], cover[1] + GRIP[1], cover[2] + GRIP[2])


def demo() -> dict[str, list[Waypoint]]:
    """Right arm: pick the roller up by its handle, roll across each patch the coworker missed, then roll three
    lanes over the bare rectangle, each toward the right arm, and lift clear."""
    yaw = GRIP_YAW
    rest = (COVER_START[0], COVER_START[1], COVER_START[2] - 0.001)  # settled on the tray
    grip = grip_for(rest)
    plan = [
        Waypoint(1.5, (grip[0], grip[1], grip[2] + 0.08), FINGER_OPEN, "above the handle", yaw),
        Waypoint(1.5, grip, FINGER_OPEN, "descend", yaw),
        Waypoint(0.8, grip, FINGER_CLOSED, "grip", yaw),
        Waypoint(1.0, (grip[0], grip[1], grip[2] + 0.05), FINGER_CLOSED, "lift", yaw),
    ]
    cx, cy = PANEL_CENTER
    roll_z = (
        PANEL_TOP + COVER_RADIUS - PRESS
    )  # the cover's centre while its line rides on the panel

    def roll(x: float, y0: float, y1: float, label: str, travel: float) -> list[Waypoint]:
        x += ROLL_TRIM_X
        return [
            Waypoint(
                travel, grip_for((x, y0, roll_z + HOVER)), FINGER_CLOSED, f"over {label}", yaw
            ),
            Waypoint(1.0, grip_for((x, y0, roll_z)), FINGER_CLOSED, "set the roller down", yaw),
            Waypoint(
                abs(y1 - y0) / ROLL_SPEED,
                grip_for((x, y1, roll_z)),
                FINGER_CLOSED,
                f"roll {label}",
                yaw,
            ),
            Waypoint(0.8, grip_for((x, y1, roll_z + HOVER)), FINGER_CLOSED, "lift", yaw),
        ]

    for n, k in enumerate(TOUCH_UP_ORDER):
        hx0, hx1, hy0, hy1 = HOLIDAYS[k]
        plan += roll(
            cx + (hx0 + hx1) / 2,
            cy + hy1 + OVERRUN,
            cy + hy0 - OVERRUN,
            f"missed spot {k + 1}",
            2.5 if n == 0 else 1.5,  # the first leg comes all the way from the tray
        )
    for k in LANE_ORDER:
        plan += roll(cx + LANE_X[k], cy + LANE_Y[0], cy + LANE_Y[1], f"lane {k + 1}", 1.5)
    return {"right": plan}


def _simulator_present() -> bool:
    """Whether Isaac Lab can be imported here: the roller hook is only installed where the scene can be built."""
    try:
        return importlib.util.find_spec("isaaclab") is not None
    except (ImportError, ValueError):
        return False


if _simulator_present():
    try:
        install_roller()
    except (
        Exception
    ) as error:  # never break loading the task over the hook; the roller then falls apart
        _warn_once("roller", f"could not install the roller hook ({type(error).__name__}: {error})")
