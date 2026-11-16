# Level a builder's automatic level on its three foot screws

One task file, `level_auto_level.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.
A builder's automatic level stands on a fixed stand on the bench, tilted 24.4' toward the back-left. The robot
levels it the way it is done on site, the first step of every setup for an elevation shot: by turning its three
foot screws until the circular bubble sits in its ring, reading the bubble as it goes. Turning one screw tilts the
whole instrument, so the screws interact, and the start tilts between the screws' lines, so no single knob can
level it. Any correct way of levelling it scores.

## Requirements

- **`-E object_jitter=0` is required.** The instrument is twelve bodies held together by joints authored where each
  body is spawned; jitter would spawn them apart from their joints, and the run-time checks (below) then score 0.
- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`
  (Isaac Lab-Arena `release/0.3.1`, here at `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`; `inspect-robots` 1.0.0,
  `inspect-robots-yam` 0.38.0 and `inspect-robots-dream` at `b5e936a201cec66c54c297e522c920d98445e339`, with
  Arena's assets read from the production bucket, as in Robocurve's November 5 launchable). Tested on
  Isaac Lab 3.0 / Isaac Sim 6.1 as pinned there.
- `source "$ARENA_DIR/yam.env"` in the shell.
- No downloads and no third-party assets: every part is built from primitives by the task file.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/level-auto-level/level_auto_level.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Level the instrument so its circular bubble is centred in the ring." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/level-dream
```

The scripted demo (full score): the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/level-demo`.

Videos: `for log in ~/yam-eval/level-*/*.json; do inspect-robots video "$log"; done`.

## Score

`--scorer task_score` gives 1.0 when the task succeeds (level, held for 1 s) and otherwise the task file's
`score()` at the end of the trial. Every step also reports `info["score"]`.

Everything is read from the instrument: the base's tilt (the angle between its vertical axis and the vertical) and
how high each foot screw's rounded tip stands above the base plate. On a real instrument the tilt is read off the
bubble (2 mm of bubble travel for every 8' of tilt) and the screws' tips with a feeler gauge.

| Score | What must be true |
|---|---|
| 0 | Any screw tip more than 0.05 mm above the base plate (the instrument held up, not standing on its own screws) or more than 0.05 mm into it (pushed down), or the base tilted more than 2 degrees |
| 1.0 | The bubble in the ring (tilt 4' or less: its centre within 1 mm of the ring's), every tip on the plate, the base still (under 5 mm/s and 0.02 rad/s) and no knob being turned (none turned 0.1 degrees over the last control step), held for 1 s; this ends the trial |
| 0.75 | The bubble in the ring at the end, but not yet for 1 s |
| 0.5 | Tilt 8' or less: the bubble touching the ring |
| 0.25 | Tilt 15' or less: within the automatic compensator's working range |
| 0 | Anything else, including doing nothing (24.4') |

Why these lines: a builder's automatic level keeps its line of sight level only while its compensator is within its
working range (about 15'), and the operator levels it until the circular bubble is inside the ring, which on an
8'/2 mm vial is about 4'. The start tilts toward 148 degrees, between two screws' lines: any one knob turned alone
leaves at least 11.5' (0.25 at most), so at least two must move. Holding a corner up, or pressing one down, takes out
at most 1.1' before the 0.05 mm limits are reached (0.05 mm over the 15 cm to the other two screws); even with every
corner anywhere in that band, a single knob still leaves 9.5'. Pushing on the instrument can bend a thread's spring
at most 0.15 mm (3.4' at one corner, about 5' combined with the seat band) before the step scores 0; a hand resting
on the instrument is not otherwise detected, but no single knob plus such a press reaches the full-score band.

## Reading the bubble

The circular level is real size: an 8 mm bubble in a 28 mm window. From the top camera at 320 x 176 it is about two
pixels across, too small to read. Like a surveyor, the robot has to lean in: a wrist camera 11 cm from the vial sees
the bubble about 12 pixels across, 1.5 pixels per millimetre of bubble travel, so a 1 mm offset (4') is visible.

## The scripted demo

The demo is open-loop: its waypoints are fixed, and nothing is read from the simulator or the cameras while it runs.
Its turns are planned from the fixed start's bubble offset as the left wrist camera shows it when the arm leans in
over the circular level, which the demo does first so the video's wrist frames show the bubble: its centre 5.2 mm
toward the robot and 3.2 mm to its left of the ring's, a tilt of 24.4' toward the back-left on the vial's 8'/2 mm.
(The screws are set for 24.0' toward 150 degrees; the threads' springs give a little under the weight, and the
instrument settles at 24.4' toward 148 degrees, which is what the camera shows.)
The plan never uses the screws' hidden settings. By the textbook three-screw procedure, the front pair is turned
equally and oppositely to move the bubble parallel to the line through them (116.1 degrees each: the left arm, then
the right arm), and the back screw then centres it (326.7 degrees in two grips, the wrist turned back between them).
The back knob is under the eyepiece, so the left arm first swings the telescope 60 degrees clockwise by its eyepiece,
then leans in over the level again while the right arm turns the back knob. The right arm lets go as soon as its last
turn ends. The bubble is in the ring, the base still and no knob turning for 1 s, which ends the trial. In the
simulator the full score first holds at control step 927 and the trial ends as a success at step 947 (47.35 s); three
trials run back to back each took the same 947 steps, ending 0.6' from level.

- Run limit: 2,400 control steps at 20 Hz (2 minutes). Wall clock on an RTX PRO 6000 Blackwell GPU shared with other
  runs: the first trial took about 5.5 minutes (about 2 of them startup), and each further trial in the same process
  3 to 4 minutes.
- No randomness: the layout is fixed (`-E object_jitter=0`), so no seed is needed.

## How the file works

- **The instrument** is twelve rigid bodies on PhysX joints: the base (body, three arms and their bosses), the
  telescope (housing, standard, tube, sunshade, eyepiece, the circular level's bracket and window, and look-only
  focusing and horizontal-drive knobs), three knobs, three spring washers, three screws and the bubble. The
  task-file API can only list boxes, cylinders and spheres, so each body is listed as one of those and a scene hook
  adds its other parts and the joints when the scene is built (see "The scene hook").
- **The base** is held on the plate by a central spring joint (as a tribrach's spring plate holds it): it may tilt
  2 degrees and rise, but cannot slide or turn. Its tilt comes only from where its three screw tips stand on the
  plate.
- **The screws' threads.** Each knob turns on a hinge on its arm and its screw slides on a prismatic joint along the
  same axis, 4 mm either way, held by a very stiff spring (1,000 N/mm) whose set point the file moves to the knob's
  turn times the thread's pitch (1 mm per turn; a right-hand thread, so clockwise drives the screw down and lifts
  that corner) from `score()` and `success()`, which the embodiment calls every control step; the screws follow
  only because they do. PhysX turns a hinge about, and slides a prismatic joint along, the X axis of its frame, so
  both joints' frames put X on the screw's vertical axis. A spring washer (2.7 N) under each knob gives the thread
  its holding friction. A PhysX rack-and-pinion joint was tried first and locked the knobs, so it is not used.
- **The telescope** turns on the base's vertical axis with some friction, its centre of mass on the axis as a real
  one is balanced. Like a real axis's friction, a spring holds its heading: the file moves the spring's set point
  along only where the telescope has been turned more than half a degree past it, so a hand swings it (against at
  most 0.17 N m) but it does not creep.
- **The bubble** is the top of a pendulum hung from the vial's centre of curvature (0.86 m below the window, for an
  8'/2 mm vial), its weight 5 cm below the pivot and damped like the vial's liquid. It therefore rests at the vial's
  highest point, exactly where a real bubble floats, moves 2 mm for every 8' of tilt, and stops at the window's
  edge. It has no collider; heavily damped, as the vial's liquid damps a bubble, it settles in two or three seconds.
- **Checks before anything scores.** The scene hook notes in a ledger (kept on `yam_arena.scene`, so every load of
  the file sees the same one) that each body got its parts and every joint was authored, and which joints; until it
  has, and while the stage being simulated does not hold those joints, nothing scores. A hook that cannot be
  installed or run is recorded the same way, and never stops the scene from loading; its warnings never raise. Every
  part the hook makes has its extent authored, which the simulator measures the bodies' outlines from. Then, every
  step:
  - every object is reported, each field with its shape, every number finite and every orientation a unit
    quaternion (otherwise the rest of the trial scores 0);
  - each body's reported bounding box matches the shape built, its parts included (otherwise the rest of the trial
    scores 0);
  - each joint holds its two bodies within 0.5 mm of where it should;
  - each screw stands within 0.15 mm of where its knob's turns put it, judged from where it settled at the trial's
    first step (each screw must stay within 1.2 mm of where it was set, checked at every step), the file following each knob's turn step by step, so a thread
    that did not follow, or a hand pushing the base level, scores 0.
  The file reads only the objects' states, no joint readings or gripper poses.
- **Episodes.** The task-file API gives no signal when a trial starts, so the file finds the embodiment that is
  scoring by walking the Python call stack (the first caller with an episode clock `_t`, a `task` and a `reset`).
  It starts a new trial when that embodiment's clock has not moved on since the last step (each episode's first
  scored step has the clock at one control step) or a different embodiment calls, and it also wraps the
  embodiment's `reset` once, keeping a list of callbacks on the wrapper, one per load of the file. A new trial clears
  the record of a spoiled trial and every knob's turn count.
- **The scene hook.** Right after `yam_arena.scene` is imported, the file wraps its `task_object` so that each of
  the instrument's bodies, once spawned, gets its other parts, and the last one (the bubble) also adds the joints.
  It answers only for this file's own object specs, compared by value, so other tasks' objects are left alone.
  Where Isaac Lab is not installed, nothing is hooked.
- **No `from __future__ import annotations`.** The task loader imports the file without registering it in
  `sys.modules`, and dataclasses then cannot resolve string annotations.

## Checking it without a GPU

The file imports only the standard library, NumPy and `yam_arena`'s `constants`, `demo` and `task_file`, so its
geometry, scoring and demo plan can be exercised on any machine by standing in small fakes for those three modules.
The instruction the runs use is in `task.json`:

```bash
python -B -I -c "import json; print(json.load(open('tasks/level-auto-level/task.json'))['instruction'])"
```

## Sources

- Compensator working range about 15', circular level 8'/2 mm to 10'/2 mm, 1.5 to 1.7 kg: makers' listings for
  builder's automatic levels (for example the Leica NA320, whose retail listings give a working range of +-15', and
  the Topcon AT-B series).
- Levelling with three foot screws as the first step of setting up (turn two together in opposite directions, then
  the third; the bubble follows the left thumb): US Navy Engineering Aid training manual (NAVEDTRA 14069) and
  Johnson and Berger instrument manuals.
- This task's own choices, not published by makers: the foot screws' pitch (1 mm per turn), the spring washers, and
  the enlarged screw circle (10 cm) and plate; a real builder's level has its foot screws closer together.
