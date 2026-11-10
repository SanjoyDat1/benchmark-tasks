# Fit nail plates where a bored hole is too close to the stud face

One task file, `nail_plates_bored_studs.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.
A short stud-wall panel lies on a framing table, as in an off-site panel plant, with a cable run through
holes bored in its five studs. The electrical code wants every hole's edge at least 1-1/4 in (31.75 mm)
from the stud's face; where it is closer, a drywall screw could reach the cable, so a steel nail plate
goes over the hole. The robot reads each hole's depth from the stud's side, as an electrician does,
and fits plates flat on the faces of the studs that need them, and no others.

## Requirements

- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`
  (Isaac Lab-Arena `release/0.3.1`, here at `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`; `inspect-robots` 1.0.0,
  `inspect-robots-yam` 0.38.0 and `inspect-robots-dream` at `b5e936a201cec66c54c297e522c920d98445e339`, with
  Arena's assets read from the production bucket, as in Robocurve's November 5 launchable). Tested on
  Isaac Lab 3.0 / Isaac Sim 6.1 as pinned there.
- `source $ARENA_DIR/yam.env` in the shell.
- Every object is a plain box, so no asset downloads are needed.
- Run with `-E object_jitter=0`: the plates sit on narrow stands, and jitter would knock them off.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/nail-plates-bored-studs/nail_plates_bored_studs.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "A cable runs through holes bored in the five studs. Fit a steel nail plate flat on the top face of each stud whose hole edge is less than 1-1/4 in (31.75 mm) below that face, centred over the hole, and plate no other stud. Then let go of the plates." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/plates-dream
```

The scripted demo (full score): the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/plates-demo`.

Videos: `for log in ~/yam-eval/plates-*/*.json; do inspect-robots video "$log"; done`.

## Score

`--scorer task_score` gives 1.0 when the task succeeds and otherwise the task file's `score()` at the end
of the trial. Every step also reports `info["score"]`.

| Stud (y, mm) | Hole centre below the face | Hole edge below the face | Needs a plate |
|---|---|---|---|
| -240 | 33 mm | 22 mm | yes |
| -120 | 36 mm | 25 mm | yes (6.75 mm short) |
| 0 | 44.5 mm | 33.5 mm | no |
| 120 | 28 mm | 17 mm | yes |
| 240 | 45 mm | 34 mm | no |

A 22 mm hole in an 89 mm stud passes on both faces only if its top edge is 31.75 to 35.25 mm below the
face. Every hole's edge here is at least 33 mm from the down face, so only the up face needs plates,
and at least 16 mm (5/8 in) from both faces, the closest the framing code lets a hole be bored (IRC
R602.6 item 2), so every stud is legal once plated.

A plate **protects** a stud when it lies flat on the stud's up face (underside within 1 mm of the face,
tilted under 5 degrees), runs along the stud (within 10 degrees), and its centre is within 3 mm of the
stud's centreline (so it spans the 38 mm face) and within 20 mm of the hole's centre along the stud
(so the 76 mm plate covers the 22 mm hole's length). It must also be still (under 1 cm/s), and no point on
either gripper's centre line, from its wrist mount (13.4 cm behind the tool point along the approach) to its pad
tips (1.3 cm ahead), may be within 5 cm of any point of it. A plate still in the fingers, or pressed
down by the palm or the gripper body, doesn't count.

| Score | What must be true at the end |
|---|---|
| +0.25 per stud | Each stud that needs a plate and has a protecting one |
| -0.25 per plate | Each plate resting on the face of a stud that doesn't need one (its centre over the face, its underside within 5 mm of it, turned any way); a plate is a limited part, and one on the wrong stud leaves a needed stud bare. The score never goes below 0 |
| 1.0 | Every stud that needs a plate is protected, no plate is on a wrong stud, held for 1 s (until then at most 0.75) |

There is a spare plate, so "use every plate" plates a wrong stud and loses a quarter, and plating studs
without reading the holes doesn't pay: no way of putting one plate on each of four studs reaches 1.0.
Stacked, dropped or misplaced plates earn nothing; plates left on their stands are fine. The studs,
cable and stands are fixed to the bench.

## Seeing the holes

The call is made from each stud's side face, where the hole shows as a near-black 22 mm square. From the
fixed top camera it is not legible: at 320 x 176 the 25 mm and 33.5 mm edges differ by under a pixel,
and some side faces face away from it. The wrist cameras can read it: each sits 7 cm beside the
tool point and 6 cm above it, looking 25 degrees off vertical toward it, so with the gripper turned 90
degrees over a stud the camera looks down into the bay at the stud's side face from about 13 cm (tool
point 4 mm to the stud's left of its centreline, 20 mm over its face; both arms reach it). The setup
screenshot's second panel is two 320 x 176 right-wrist frames from that pose, over the stud at y -120 and
over the one at y 0, side by side. In them the hole's near edge sits 22 px from the line of the stud's
face at y -120 (25 mm) and 28 px at y 0 (33.5 mm), as the camera model predicts: about 6 px apart, on a
face 53 px deep in the image.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes). The scripted demo runs both arms at once (the left arm
  plates the stud at y 120; the right arm plates the stud at y -240, then the one at y -120) and leaves the
  spare plate on its stand. Every plate counts once its gripper has risen clear, at about 17 s; the score
  reaches 1.0 a second later, at about 18 s, and the trial ends as a success.
- Wall-clock time on one RTX PRO 6000 Blackwell GPU: about 20 s to start the simulator once its caches are warm
  (2 to 3 minutes cold), then about 70 s per demo trial at 320 x 176 cameras.
- No randomness: the layout is fixed (`-E object_jitter=0`), so no seed is needed.

## How the file works

- **The wall** is fixed boxes: five studs, a near-black square on both side faces of each stud where its
  hole is, and the cable as straight runs through the holes joined by short risers midway between studs.
- **Scoring** is the end state only: `score()` reads the four plates' poses after every control step,
  counts the studs that need a plate and have one, and takes off a quarter per plate on a wrong stud. It
  returns 1.0 only after everything has held for 1 s of the embodiment's own control steps (its
  `step_dt`; 20 Hz if it doesn't say), so a trial that gets there in its last second scores 0.75;
  `success()` reports the condition itself and the embodiment ends the trial once it has held for 1 s. The
  hold restarts on any break and when the episode clock goes back (a new trial). `success()` and `score()`
  share each step's state dict.
- **Letting go.** A plate counts only with every gripper clear of it, so the file needs the grippers as well
  as the plates. Task files get only the objects' states, so on every step the file finds the calling
  embodiment (the caller with an episode clock, a task and a `reset`), reads both arms' joint positions from
  it, and turns them into tool points and approach directions with the framework's own forward kinematics
  (`yam_arena.ik.tcp_pose`). It measures from points along each gripper, wrist mount to pad tips, to the
  nearest point of the plate in the plate's own frame. If the arms can't be read on a step, the file warns
  once, counts the plates without that test, holds the score at 0.75 or less, and tries again the next step.
- **The demo** closes each gripper across a plate's 38 mm width with the pad tips just below the plate, which
  the narrow stands leave room for. It lowers the plate to 2 mm above the stud's face, so the fingers never
  straddle the stud, opens (the plate drops flat), and rises 8 cm over the face, clear of the plate.
- **No `from __future__ import annotations`.** The task loader imports the file without registering it in
  `sys.modules`, and dataclasses then cannot resolve string annotations.

## Checking it without a GPU

The file loads without Isaac Sim, so `task_file.load()` checks its objects and hooks on any machine with
`yam-isaac-lab-arena` installed:

```bash
python -B -I -c "from yam_arena import task_file; t = task_file.load('tasks/nail-plates-bored-studs/nail_plates_bored_studs.py'); print(t.names)"
```
