# Pass a drill by its body in both hands, handle into a coworker's palm

One task file, `hand_drill_handle_first.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.
A cordless drill lies on the bench; a coworker holds out a gloved hand, palm up. The robot hands the drill over
the safe way: handle into the palm, motor housing over the thumb, chuck pointing out along the fingers and never at
the coworker, held by its body in both hands (its balance point is inside the handle, which must stay free), slow
near the hand, without bumping it, and released only once it rests in the hand.

## Requirements

- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`
  (Isaac Lab-Arena `release/0.3.1`, here at `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`; `inspect-robots` 1.0.0,
  `inspect-robots-yam` 0.38.0 and `inspect-robots-dream` at `b5e936a201cec66c54c297e522c920d98445e339`, with
  Arena's assets read from the production bucket, as in Robocurve's November 5 launchable). Tested on
  Isaac Lab 3.0 / Isaac Sim 6.1 as pinned there.
- `source $ARENA_DIR/yam.env` in the shell.
- Network access to NVIDIA's public asset bucket (the drill is an Arena library asset; see "The drill's URL").

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/hand-drill-handle-first/hand_drill_handle_first.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Your coworker holds out their right hand, palm up, for the drill. Holding it by its body with both hands, never by the handle, lay the handle across their palm with the chuck pointing out along their fingers, never at them. Move slowly near their hand without bumping it, and let go once the drill rests in it." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/drill-dream
```

The scripted demo (full score): the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/drill-demo`.

Videos: `for log in ~/yam-eval/drill-*/*.json; do inspect-robots video "$log"; done`.

## Score

`--scorer task_score` gives 1.0 when the task succeeds (a full score held for 1 s) and otherwise the task file's
`score()` at the end of the trial. Every step also reports `info["score"]`.

| Score | What must be true at the end |
|---|---|
| 0.25 | Carried: lifted 2 cm or more, then brought within 7 cm of the palm |
| 0.5 | Resting on the hand: still, flat within 10 degrees, at palm height, handle within 6 cm of the palm point |
| 0.75 | Handle first: handle middle within 2 cm of the palm point, handle across the palm toward the thumb, chuck along the fingers (20 degrees each) |
| 1.0 | Handed over: both grippers at least 7 cm from the drill, held for 1 s (until then it reads 0.75) |

Caps for the rest of the trial: the chuck aimed at any point of the coworker, feet to face, while lifted (0.25);
any point of the drill faster than 0.25 m/s within 15 cm of the hand (0.5); carried by its handle, judged along the
line between the finger pads so a shallow grip counts too (0.5); carried in the air (5 mm or more up, not on the
hand) by fewer than both grippers for 0.5 s in all (0.5); the drill touching the coworker's cuff or sleeve (0.5); a
TCP or finger pad within 3 mm of the coworker's hand, cuff or sleeve (0.5); dropped, i.e. falling 2 cm or more while
no gripper is moving it (0). The grippers may close on the drill one at a time while it lies on the table and let
go one at a time once it lies on the hand. The exact rules are in `task.json` and in the file's `grade()`.

**Scoring a real trial:** grips, speed, aim and contact from 20 fps video of the arms; the end pose from a top
photo and a tape (handle middle to the palm point, the handle and chuck directions).

## Runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (1 minute). The scripted demo hands the drill over in 20 s: the full
  score's conditions are met at about 17.4 s, and after the 1 s hold the score reads 1.0 and the trial ends as a
  success at about 18.4 s.
- Wall-clock time on an RTX PRO 6000 Blackwell GPU: about 1.5 minutes for one trial (most of it startup), about
  3.3 minutes for three trials in one process (`--epochs 3`).
- No randomness: the layout is fixed (`-E object_jitter=0`), so no seed is needed.
- The drill's USD as tested (`cordless_drill.usd` on NVIDIA's production bucket, see below) has SHA-256
  `358ddbab8ba44a5d4766785db04195758779c118a5760b2c372f4401036ba449`.

## How the file works

- **The coworker's hand** is a set of fixed boxes: palm, four fingers, thumb and the web between them, glove cuff
  and sleeve. The drill handed over correctly rests on the palm (battery) and the thumb (motor housing), with its
  handle over the hollow of the palm.
- **Episode rules.** `score()` is called after every control step. The file keeps, per trial, what has happened
  so far (lifted, carried near the hand, aimed at the coworker, too fast near the hand, carried by the handle,
  carried with fewer than two grippers, touched the sleeve, a gripper touched the hand, dropped) and grades the
  final pose against the ladder.
  `success()` and `score()` share each step's state dict, so a step counts once.
- **The robot's grippers.** Handing over means letting go, holding by the body means not gripping the handle, and
  carrying with both hands means both grippers holding it, so the file needs the grippers as well as the drill. Task files get only the objects' states, so on its first
  step the file finds the calling embodiment (the caller with an episode clock, a task and a `reset`), reads both
  arms' joint positions from it, and turns them into TCP positions with the framework's own forward kinematics
  (`yam_arena.ik.tcp_pose`). If that fails, it warns once and caps the score at 0.75.
- **Both hands.** A gripper holds the drill when its TCP is at the drill (within 1 cm of the drill's own box, in
  the drill's frame), its fingers are closed on it (2.5 to 8.5 cm apart) and the drill moved with it since the last
  step. While the drill is in the air (5 mm or more above the table and not down on the hand), fewer than two
  holding it for 10 control steps in all caps the score; lying still with no gripper on it, it isn't being carried.
- **Fingers off the hand.** Each gripper's finger pads are modelled from the TCP pose the kinematics return: tips
  13 mm past the TCP along the approach axis, 27 mm wide, checked from the tips up 30 mm, at the fingers' current
  opening. Any of those points, or a TCP, within 3 mm of a box of the hand, cuff or sleeve caps the score at 0.5. The embodiment
  senses no finger contact, so holding is judged from geometry and motion only. The scripted demo holds the drill by
  the front of the battery pack (right arm) and the back of the motor housing (left arm), with its centre of mass
  between them, and moves both arms on one clock as one rigid body: each arm's waypoints are the drill's pose seen
  from its own grip.
- **Episodes.** Task files get no reset hook, so the file wraps the embodiment's `reset` once: every new trial
  starts a fresh record. If no embodiment is found, it warns and treats the drill resting back at its start pose
  after a lift as a new trial.
- **The drill's URL.** Isaac Lab-Arena `release/0.3.1` points its object library at NVIDIA's staging asset bucket
  (a TODO in `isaaclab_arena/assets/nucleus.py`), and that bucket no longer holds the library; the production
  bucket holds the same files. When the task file is loaded it installs a one-shot import hook that waits for
  `isaaclab_arena.assets.object_library`, lets it load normally, and then rewrites this drill's `usd_path`, and
  nothing else, from the staging host to the production host, for the rest of the process. It does nothing if the
  URL points anywhere else (a fixed Arena or a local mirror), removes itself once the library has loaded, and never
  fires without Arena.
- **No `from __future__ import annotations`.** The task loader imports the file without registering it in
  `sys.modules`, and dataclasses then cannot resolve string annotations.

## Checking it without a GPU

The file loads without Isaac Sim, so `task_file.load()` checks its objects and hooks on any machine with
`yam-isaac-lab-arena` installed:

```bash
python -B -I -c "from yam_arena import task_file; t = task_file.load('tasks/hand-drill-handle-first/hand_drill_handle_first.py'); print(t.names)"
```
