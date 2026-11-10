# Finish painting a panel a coworker started, without repainting their work

One task file, `finish_painting_panel.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.
A coworker has painted a panel's border and its left end, and missed three small patches in their own paint;
the robot finishes the bare rectangle and touches up the patches with a 4-inch mini paint roller, without missing
spots and without repainting finished areas beyond a short overlap.

## Requirements

- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`
  (Isaac Lab-Arena `release/0.3.1`, here at `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`; `inspect-robots` 1.0.0,
  `inspect-robots-yam` 0.38.0 and `inspect-robots-dream` at `b5e936a201cec66c54c297e522c920d98445e339`, with
  Arena's assets read from the production bucket, as in Robocurve's November 5 launchable). Tested on
  Isaac Lab 3.0 / Isaac Sim 6.1 as pinned there.
- `source $ARENA_DIR/yam.env` in the shell.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/finish-painting-panel/finish_painting_panel.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Your coworker started painting this panel. Finish it with the paint roller: cover every bare spot, including any they missed, without repainting their work beyond a small overlap, and keep paint off the table." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/paint-dream
```

The scripted demo (full score): the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/paint-demo`.

Videos: `for log in ~/yam-eval/paint-*/*.json; do inspect-robots video "$log"; done`.

## Score

`--scorer task_score` gives 1.0 when the task succeeds (a full score held for 1 s) and otherwise the task
file's `score()` at the end of the trial. Every step also reports `info["score"]`. The rules are in
`task.json` and in the file's `score_ledger()`; in short:

- **Coverage** of the 516 bare cells (1 cm), 462 in the rectangle and 18 in each of the three missed patches:
  1.0 all, 0.8 at 98% or more (at most 10 missed), 0.6 at 95%, 0.4 at 80%, 0.2 at 50%, else 0. Skipping one
  patch scores 0.6; skipping all three, 0.4.
- **Waste**, two-sided on purpose: the coworker's cells more than 2 cm from any bare area that the robot
  painted. Up to 20 keep the score, 21 to 60 multiply it by 0.75, 61 to 120 by 0.5, more than 120 score 0.
  Painting the whole panel covers everything but wastes 367 cells, so it scores 0.
- **Paint on the table** (any part of the roller within 1.5 mm of the table top while the cover is off the
  panel) caps the score at 0.5.
- **Hold**: a full score reads 0.8 until it has held for 1 s (20 control steps), so a trial that only gets
  there in its last second doesn't score 1.0.

**Scoring a real trial:** photograph the panel from above through a 1 cm grid and count bare cells (patches
included); read the robot's strokes over the coworker's paint off the overhead video, since the new paint
matches theirs; watch the roller for contact with the table.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes). The scripted demo (a roll across each missed patch,
  then three lanes, each rolled toward the right arm) reaches 1.0 after about 40.3 s of simulated time and
  ends as a success 1 s later. On an RTX PRO 6000 Blackwell GPU shared with other runs, one trial took about
  4.5 minutes of wall-clock time (about 2 minutes of it startup), and each further trial in the same process
  about 2.5 minutes.
- No randomness: the layout is fixed (`-E object_jitter=0`), so no seed is needed.

## How the file works

- **The roller.** The task-file API has upright cylinders and boxes but no joints, so the roller is two task
  objects joined by a hook: `roller_cover` (a cylinder) and `roller_frame` (the handle's box). Right after the
  official `yam_arena.scene` is imported, the file wraps its `task_object()` by object name: the cover's
  cylinder is spawned lying along x (Isaac Lab's `CylinderCfg.axis`), and the frame's spawner, after spawning
  the handle, adds the wire arms, cross bar and axle stubs as more colliders of the same rigid body and a
  revolute joint on the axle between the frame and the cover. Jointed bodies don't collide with each other, so
  the cover spins freely and rolls on the panel. The hook is installed only where Isaac Lab can be imported, it
  does nothing to other objects, and a second load of the file is served by the first wrapper. Keep
  `-E object_jitter=0`: jitter would move the two bodies apart.
- **Paint ledger.** `score()` is called after every control step with each object's state. The file keeps a
  1 cm grid of the panel. The cover paints the cells under its lowest line (centres within 5 mm of it, along its
  10 cm) where that line is between 3 mm below and 1.5 mm above the panel top, as long as the axle is within
  10 degrees of level; spinning doesn't matter, so a rolling and a skidding cover paint the same strip. Cells
  passed over since the last touching step (cover centres up to 4 cm apart) are filled in. `success()` and
  `score()` share each step's state dict, so a step counts once.
- **The panel** is built from flush fixed slabs: the bare rectangle, the three missed patches (also bare
  primer) and the coworker's paint tiled around them.
- **Table contact** is judged from the two bodies' poses and the roller's sizes (its lowest point), not from
  the simulator's bounding boxes.
- **Episodes.** Task files get no reset hook. The file finds the calling embodiment (the caller with an
  episode clock, a task and a `reset`) and wraps its `reset` (keeping its name and signature, and never twice),
  so every new trial starts a fresh ledger and hides the last trial's tiles before its first frame renders. If
  no such caller is found, it warns once and treats the roller resting at its start pose as a new trial.
- **Visible paint.** New paint is drawn as thin tiles with Isaac Lab's `VisualizationMarkers`, imported inside
  `score()` once the simulator is running (the module itself imports nothing from Isaac Sim, as task files
  must), so camera-based policies see what they have painted. The tiles are only a picture of the ledger: if
  drawing fails, they turn off with one warning and scoring carries on.
- **No `from __future__ import annotations`.** The task loader imports the file without registering it in
  `sys.modules`, and dataclasses then cannot resolve string annotations.

## Checking it without a GPU

The file loads without Isaac Sim, so `task_file.load()` checks its objects and hooks on any machine with
`yam-isaac-lab-arena` installed:

```bash
python -B -I -c "from yam_arena import task_file; t = task_file.load('tasks/finish-painting-panel/finish_painting_panel.py'); print(t.names)"
```
