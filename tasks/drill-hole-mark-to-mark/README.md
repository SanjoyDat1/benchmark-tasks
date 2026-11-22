# Drill a straight 1/4 in hole through a 2x4 with a handheld drill, in at the top mark and out at the bottom mark

One task file, `drill_hole_mark_to_mark.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.

A 2x4 stands on edge on the bench, clamped and raised on two offcuts over a gap, with a pencil X on the middle of
its top edge and a matching X on its bottom edge. A 12 V pistol drill/driver with a 1/4 in wood bit lies on its side
next to it. The robot has to pick the drill up by its handle without running it (the pinch that holds the handle
also squeezes the trigger), turn it bit-down, set the point on the top X, square the bit and push until the point
bites, squeeze the trigger and feed against the reaction twist, draw the bit out and put the drill down.

The skill is **precise control of position and force with a handheld power tool**. A shallow hole hides a lean of a
few degrees; a through-bore turns it into a miss on the far face (2.6 degrees through 89 mm misses the bottom X by
4 mm), so the hand must hold the line under a sustained push until breakthrough. No drill press does the aiming,
and the bit sits 55 mm beside the grip, so the push twists the wrist: the bit must be squared under load. The task
doesn't prescribe how.

## Requirements

- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`
  (Isaac Lab-Arena `release/0.3.1`; `inspect-robots` and `inspect-robots-yam` as it installs them);
  `source "$ARENA_DIR/yam.env"` in the shell.
- Nothing to download: the task file builds its geometry from boxes, cylinders, a cone and a sphere.
- `-E control_hz=20`: the file reads the control rate from the embodiment when it can, and otherwise counts its
  1 s hold as 20 steps.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/drill-hole-mark-to-mark/drill_hole_mark_to_mark.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Drill a hole straight down through the board, in at the X on its top edge and out at the X on its bottom edge, then put the drill down on the bench and let go of it." \
  --scorer task_score --max-steps 6000 --store-frames --log-dir ~/yam-eval/drill-dream
```

To run the scripted demo, use the same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/drill-demo`.

## The job, as the trade does it

- **Occupations** (O*NET read 2026-11-16 and 2026-11-17; BLS OEWS May 2025, national). Both list this work as a
  Core task (relevant to at least 67% of workers, mean importance at least 3.0):

  | SOC | Occupation | O*NET Core task | OEWS May 2025 |
  |---|---|---|---|
  | 47-2031 | Carpenters | "Bore boltholes in timber, masonry or concrete walls, using power drill." | 670,090 |
  | 47-3012 | Helpers, carpenters | "Drill holes in timbers or lumber." | 21,680 |

  Together: **691,770**. Occupations that drill in installed framing, or whose O*NET entries name drills only
  as a tool, are not counted.
- **Mark to mark.** A bolt or threaded rod has to leave on its mark on the far face. Boring such holes at a bench
  or on sawhorses before the piece goes in is common trade practice; no single published source was found.
- **The bit** is a 1/4 x 6 in spade bit with a 1/4 in hex shank that fits a quick-change holder, Irwin Speedbor
  88804: a centre point and two outer cutting spurs (retailers' listings, read 2026-11-21, about $3 to $7).
  No 6 in brad-point bit with a 1/4 in hex shank was found on sale; those found are 4 to 5 in long, too short for
  an 89 mm bore plus the holder. The task file's comments say "brad point": it models a centre point that holds
  when pushed square, which is the model's **estimate**, not a maker's claim.

## Setup and objects

Millimetres; x away from the robot, y to its left, z up from the bench.

| Object | Make, size, material | Place |
|---|---|---|
| Board | Kiln-dried SPF 2x4, 38.1 x 88.9 x 300, about 0.48 kg, on edge along y | Centre line x 330, centred at y -53 |
| Supports | Two 2x4 offcuts laid flat (89 x 90 x 38.1), fixed with double-sided carpet tape, not screws (a shared bench) | Centred at x 330, y -118 and +12; a 40 gap under the X |
| Clamps | Two 12 in bar clamps along x across the board's faces, jaws resting on the offcut; fixed jaw on the robot's side, bar and handle running away from the robot | One over each offcut; jaws at x 300 and 360 |
| Scrap | 3 mm hardboard: takes a breakthrough lunge, catches the chips that fall through | On the bench in the gap |
| Marks | Pencil Xs, 22 mm strokes | Top and bottom edges, x 330, y -53 |
| Drill | Bosch PS21-2A kit (PS21N pocket driver, 2.0 Ah battery), or any 12 V pocket drill/driver with a 1/4 in hex holder; 150 x 64 x 178, 846 g with battery and bit; forward, high speed (0-1,300 rpm), clutch low (for example 5 of 20) | On its side, bit toward the robot, handle's centre at (364, -312) |
| Bit | Irwin Speedbor 88804 spade bit, 1/4 x 6 in, 1/4 in grooved hex shank (retailer listings, read 2026-11-21); about 30 g (an **estimate**) | In the holder, about 127 exposed |
| Cameras | Two phones on tripods, for scoring | At the bench's end looking along the board; across the bench, facing the robot |

Not simulated: the clamps' bars and handles (only their jaws, as four fixed steel blocks, 22 x 40 x 50), the
hardboard, the chips and the phones; the bit is drawn as a plain 1/4 in rod with a pointed tip. Check on the real
bench that the clamps' bars and handles clear the drill's path and the place it is laid down.

## Score

Every score above 0 needs the finish, checked at the end: the drill resting on the bench (its lowest point within
5 mm of it; and, seen from above, the rectangle that bounds the drill and its bit, with its sides parallel to the
bench's edges, clear of the strip x 286 to 374, y -203 to 97 that holds the board and its offcuts), its trigger released, and every finger at least 50 mm from the
drill's parts. A drill left in the hole or in the hand scores 0.

| Score | When |
|---|---|
| 1.0 | Through; entry within 3 mm of the top X; exit within 4 mm of the bottom X; everything still (under 10 mm/s and 0.5 rad/s) for 1 s |
| 0.75 | Through; entry within 3 mm; exit within 8 mm |
| 0.5 | Through; entry within 6 mm; exit within 12 mm |
| 0.25 | A hole at least 20 mm deep, entry within 6 mm |
| 0 | Anything else, or if at any time the drill fell, the bit was levered, the bit spun in the air, or the drill left the bench top |

- **Entry, exit:** where the hole's line crosses the top and bottom edges, from each X's centre. **Through:** the
  point got 4 mm past the bottom edge.
- **Fell:** the drill dropping faster than 0.5 m/s (a free fall reaches that in about 13 mm) while no gripper is
  closed on its handle (fingers at most 55 mm apart, within 35 mm of the handle's pinch point). A hand on the
  housing alone is not holding it; a breakthrough lunge with the handle in hand is not a fall.
- **Levered:** more than 6 degrees between the bit and the line its hole started on (its line when 3 mm in), until
  the point is drawn back out.
- **Spun in the air:** the bit turning for 0.2 s with its point more than 10 mm from the board and out of its hole.
- **One hole per trial:** once the bit is drawn back out, that hole is the trial's.
- **Real trial:** a 1/4 in rod pushed into the hole and a ruler from each X's centre give entry and exit (turn the
  board over for the exit); the rod passing shows it is through, and its depth gives the 20 mm. The trial video
  shows the set-down, a fall (a drop of about 13 mm or more with no gripper closed on the handle) and a spin in
  the air (bit and holder seen turning, motor heard, counted in frames). Levering: in each tripod video, read the
  drill's lean from the board's top edge with a protractor on a paused frame, just after the bit has entered and
  at its greatest lean in the wood; a change of more than 6 degrees in either view is levered (if it changes in
  both, the root of the sum of their squares: the scorer measures the full angle).
- **Guards:** a trial is void (0) unless every part was built and starts where it was built (the trigger and the
  point at rest), or on a non-finite reading, or if the arms' state can't be read.

## How the file works, and its fidelity

Wood can't be cut in PhysX, so the hole is joints and a feed law, driven only by run-time writes of drive set
points, stiffness, damping and force limits. The drill is rigid at its real size and mass; the contacts, the push,
the skate, the twist and the lunge are physics; the cutting is modelled.
- **Trigger:** its own body on a sprung joint, at the drill's own place and size (8 mm proud, 14 tall, 16 wide,
  16 mm under the housing; not an enlarged stand-in): 15 N to start, 18 N at its switch (1.5 mm in), 27 N fully
  in (6 mm), speed in proportion to travel past the switch. In the real trial the pad presses the real trigger, with no adapter.
- **Point:** a 6.4 mm steel ball on a 20 kN/m spring along the bit, the only part of the bit that touches the
  wood; its give reads the push. Pushed more than 8 degrees off square it skates; pushed 20 N or more within
  8 degrees, it is held where it is (the bite).
- **Feed:** with the trigger on and the push above 20 N, the depth grows at (push − 20 N) × 0.14 mm/s per N × the
  trigger's speed. It cuts from about 20 N; a harder push only cuts faster.
- **Line lock:** from 3 mm deep, two springs (at the point and 60 mm up the bit, 20 and 8 kN/m) hold the bit on
  the line it went in on, where the entry is recorded. Leaned more than 2 degrees off it, the bit binds (no feed,
  0.6 N m of drag). The lock ends when the point is drawn back out.
- **Twist:** a torque against the bit's turn, 0.05 N m + 0.18 N m per mm/rev of feed. **Breakthrough:** at 89 +
  4 mm the exit is recorded and the drill lunges until the arm catches it or its holder is 10 mm from the board.
- **Springs for wood** (6 traced demo trials, two GPU types): while cutting, the point stayed within 0.41 mm of
  its line and the bit within 0.18 degrees. At breakthrough, with the trigger still on, the lunge tilted the bit
  up to 0.67 degrees for under a second (1.2 mm at the hole's mouth): a real bit would ream the mouth there. With
  the trigger off, while the bit is drawn out, it stayed within 0.31 degrees and 0.5 mm: it would rub, not cut.
- **Looks only:** the bit's rod and tip, the pencil Xs. **Left out:** tear-out, the bit's flex, the clutch.
- **Levering on the GPU:** a commanded 30 degree lever tilted the bit only 3.73 degrees off its line against the line lock, so
  the 6 degree rule is shown by the offline dry run and the unit tests, not by a GPU run.

## Sources and estimates

| Value | Used | Source / status |
|---|---|---|
| 2x4 | 38.1 x 88.9 mm; about 470 kg/m³ | Dry dressed size (American Softwood Lumber Standard PS 20); density an **estimate** (G ≈ 0.42, American Wood Council FAQ) |
| Drill | 0.816 kg with battery (846 g with the 30 g bit); 150 x 64 x 178 mm (the sim's shape: 156 x 60 x 178) | Bosch PS21N page: 1.4 lb tool only, 5.6 in head length, 0–350/0–1,300 rpm, 265 in-lb, 1/4 in hex holder; the PS21-2A kit is 1.8 lb. Mass and inertia from our own model |
| Feed, torque | 119 N and 0.18 N m per mm/rev; + 0.05 N m; 1,000 rpm loaded | McMillin, C. W. and Woodson, G. E. (1972), *Forest Products Journal* 22(11):55–59: a 1 in spur bit in southern pine at about 1 mm a turn averaged 138 lb thrust and 33 in-lb torque. Scaled to 1/4 in (thrust ∝ diameter, torque ∝ diameter²) and to SPF. **Estimates**, and from a spur bit, not a spade bit |
| Bite | 20 N | **Estimate.** The same paper, p. 59: "95 percent of the thrust force observed was attributable to spurs and brad". Szwajka and Trzepiecinski (2018), *BioResources* 13(4):8263–8278, Fig. 6: 40–54 N at zero feed (9.3 mm drill, MDF); scaled, 15–25 N |
| Walk angle; trigger; line lock; grip friction | 8 degrees; 15 / 18 / 27 N; 20 and 8 kN/m; 1.0 / 0.9 | **Estimates**; no published figures were found |

At the demo's 30 N the bit feeds 0.084 mm a turn (1.40 mm/s) and goes through in a little over a minute; at 22 N it would take
over 5 minutes. A bench measurement (the board on a scale under the drill, the trigger pressed against a scale)
would replace the bite, feed and trigger estimates.

### The push, the pads and the wrists

- The 30 N on the wood is the drill's weight (8.3 N) plus 21.7 N from the hand on the handle, whose pinch also
  holds the trigger in: about 66 N across its two pads.
- **Measured pad loads** while cutting (the finger drives' readings; 3 traced trials on each of two GPU types):
  the right hand's pads averaged 29.7-38.2 N a pad and peaked at 40.4 N (on each GPU type, one control step of
  about 4,600 was over 40 N). The real gripper's driver caps its grip at 50 N (the official stack's
  `yam_arena/robot.py`, line 88: "One DM4310 drives both fingers through a rack; i2rt's driver caps grip force at
  50 N").
- The push reaches the pads mostly as friction: about 21.7 N from 66 N of normal load, a coefficient of 0.33 or
  more on the rubber handle. An **estimate**, not a measurement.
- The second hand only braces: about 30 N a pad on the motor housing, 19 mm from the turning hex holder.
- **Wrist** (modelled statics): its pitch drive's mean torque fell from 2.9-3.0 N m holding the drill over the X
  to 2.1-2.2 N m cutting (DM4310: 3 N m rated, 7 N m peak; Damiao's 2025 selection manual). So holding the drill
  up is at about its rated torque; this has not been measured on hardware.

## The scripted demo

It reads the simulator's state, as a scripted demo may, and only changes where the arms are sent; it never moves a
part or writes to the simulation.
- The right hand pinches the handle lightly (about 14 N, under the trigger's 15 N preload), carries the drill to
  the board and turns it bit-down. From then on, every control step it reads the drill's pose, the push and the
  hole's state and corrects the arm's target by 15% of the error, as a person watches the bit.
- It rests the point at 3 N, squares the bit and slides the point onto the X; the left hand pinches the housing;
  the right hand squeezes the trigger in and raises the push slowly to 21 N until the point bites. Until then
  only friction holds the point, so the entries were 2.43-2.95 mm: inside the 3 mm rung, with little to spare.
- It feeds at 30 N (24 N for the last 8 mm) for a set time: the bit came through 28-32 s before the trigger was
  let go. Then the left hand parks, and the right hand draws the bit out, moves back clear of the board, lays
  the drill on its side on the bench in front of the board, lets go and parks.

## Results

From real runs of this task file on the official stack (3 trials in one process for each demo run; one Dream-1
trial):

| Run | GPU | Trials | Score |
|---|---|---|---|
| Scripted demo | NVIDIA L40S | 3 | 1.0 on 3 of 3 |
| Scripted demo | NVIDIA RTX PRO 6000 | 3 | 1.0 on 3 of 3 |
| Dream-1 (`dream`) | NVIDIA RTX PRO 6000 | 1 | 0.0 |

- The same demo, traced step by step (3 trials on each GPU type, on a copy of this file that differs only in
  comments), scored 1.0 in all 6: entry 2.43-2.95 mm from the top X, exit 2.54-3.13 mm from the bottom X, the drill
  laid on the bench with its trigger off and the hands clear.
- Runtime: each demo trial reaches its 1 s hold after about 3,226 control steps (161 s of simulated time; its plan
  is 165 s, the run limit 6,000 steps); three trials took about 79 minutes of wall time on the L40S and 105 on the
  RTX PRO 6000, which was shared with other jobs.
- Dream-1 ran all 6,000 steps (300 s of simulated time, about 66 minutes of wall time). Its right gripper went to
  the drill and stayed there; the drill was never lifted and no hole was started, so it scored 0.

## Real trial: safety and reset

- **People:** nobody in the cell while the bit can turn, and an operator at the robot's power cut-off all that
  time (a score of 0 for spinning in the air does not stop a real bit). The bit breaks out into the open gap over
  the hardboard scrap, and the clutch is set low, so a bound bit slips instead of twisting the arm.
- **Gripper pads** touch only the rubber handle and the motor housing, 19 mm clear of the turning holder and more
  than 100 mm from the bit. Vacuum and wipe them after each trial; replace a worn pad.
- **Chips** (about 3 cm³ a hole, dry; no liquid or sparks) land on the board, the bench and the hardboard: clear
  them with a shop vac after each trial.
- **Wood:** a kiln-dried SPF 2x4 stud cut into 300 mm pieces, about 3 trials a piece (holes 30 mm apart with the
  board still fully on both offcuts).
- **Reset** (about 3–4 minutes; an estimate, not timed): unclamp; slide the board so clear, knot-free wood is over
  the gap; mark a fresh pair of Xs with a combination square; clamp again at the places marked on the bench; fit
  a fresh bit if the last one is bent or dull; vacuum; wipe the pads; lay the drill at its start mark, charged,
  on forward, high speed and the low clutch setting (check it still drills a test hole).

## Assets and licences

- **This task** adds no third-party assets: every shape is built in the task file from primitives. The drill's
  proportions follow our own model of a Bosch PS21-class drill/driver, made from a CC0 (public-domain) 3D scan
  (Printables model 1109547). No logo or trademark is drawn.
- **The robot** is not in this folder; it comes with the official package under Requirements. That package's
  README (at the commit above) says its YAM robot model (`yam_arena/assets/yam/`) "comes from i2rt under the MIT
  license"; the file beside it, `LICENSE.i2rt`, reads "MIT License, Copyright (c) I2RT Robotics". The same README
  says the package's own code has no licence yet.
