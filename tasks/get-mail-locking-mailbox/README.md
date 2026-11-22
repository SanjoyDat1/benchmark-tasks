# Get the parcel out of a locked curbside mailbox with its key, then shut and lock it and put the key back

One task file, `get_mail_locking_mailbox.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.

A locking post-mount parcel mailbox stands on the bench on a short cut of 4x4 post, its front toward the robot. A
small parcel lies inside, behind the resident's access door, which is locked by a cam lock. The key lies on a mat
beside the mailbox. The robot has to take the key, put it into the keyway and turn it a quarter turn, open the door
(it falls open with the key riding in its lock), take the parcel out through the 245 x 95 mm opening and set it on
its marked place, lift the door shut, turn the key back to locked, take the key out and lay it back on its mat.

The skill is **working a real lock that gates a real door, in the order the lock forces**: the plug turns only with
the key fully home; turned, its steel cam clears the frame and the door opens; the door will not shut on a thrown
cam, so it must be shut before the key is turned back; the key comes out only when the lock is locked. The turn and
the cam are contact: the blade turns the plug through the keyway's walls, and the cam is a collider behind the frame.
The plug's key-home gate and the key's hold while the plug is turned are drive rules (see How the file works).

## Requirements

- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`
  (Isaac Lab-Arena `release/0.3.1`; `inspect-robots` and `inspect-robots-yam` as it installs them);
  `source "$ARENA_DIR/yam.env"` in the shell.
- Nothing to download: the task file builds its geometry from boxes, cylinders and closed triangle meshes.
- `-E control_hz=20`: the scorer counts its 1 s hold as 20 control steps.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/get-mail-locking-mailbox/get_mail_locking_mailbox.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Get the parcel out of the locked mailbox: unlock the access door with its key, take the parcel out and put it inside the taped rectangle on the table, then shut the access door, lock it, and put the key back on its mat." \
  --scorer task_score --max-steps 4000 --store-frames --log-dir ~/yam-eval/mailbox-dream
```

To run the scripted demo, use the same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/mailbox-demo`.

## Why it matters, and the closest known tasks

- **Everyday and recognisable.** Curbside delivery is the most common kind in the US: "curbside delivery points
  were the most common type of delivery, comprising 41 percent of all delivery points" (FY2021; USPS Office of
  Inspector General, RISC-WP-22-004, p. 7), "around 40 percent" in FY2025 (RISC-WP-26-002, p. 29). Mail theft is
  why people lock them: "From FY 2023 through FY 2025, the Postal Inspection Service received over 800,000 mail
  theft complaints" (USPS OIG 25-137-R26, pp. 3-4). Carriers never open a locked box: USPS Postal Operations Manual
  632.21, "Carriers are prohibited from accepting keys for locks on private mail receptacles". So getting the mail
  out, and locking up again, is the resident's job. No published share of households with a locking box was found,
  and none is claimed.
- **Closest known tasks, and what differs.**
  - RoboDojo `insert_key` ("Pick up the key, hand it over to the other hand, insert it into the keyhole, then turn
    it"): the key ends turned in a fixed block that unlocks nothing. Here the turn moves a cam that holds a door,
    the door then has to be opened, shut and locked again, and the key comes back out only at locked.
  - BEHAVIOR-1K's mailbox tasks (for example `bringing_in_mail`, `collecting_mail_from_the_letterbox`): the mailbox
    is "openable", with no key, lock or door to shut again, and the mail is taken symbolically.
  - Meta-World's door lock and unlock turn a knob; there is no key.
- **The order is the difficulty.** Done out of order the job fails physically: a door shut with the key already
  turned back stops on the cam about 11 degrees open (measured in the simulation with test drives); a key pulled
  before it is turned back to locked stays in (held with up to 40 N while the plug is turned); a door pulled without the turn
  stops at 3 degrees (the cam on the frame).

## Setup and objects

Millimetres; x away from the robot (from the line through the arms' bases), y to its left, z up from the bench.

| Object | Make, size, material | Place |
|---|---|---|
| Mailbox | Architectural Mailboxes Oasis Classic 6200B-10 (black): 15.2 in tall, 12.0 wide, 18.1 deep (386 x 305 x 460), 22.44 lb, galvanized steel, powder coated, keyed cam lock, two keys | Front face at x 320, centred at y 0 |
| Mount | The maker's 4x4 adapter plate (item 7530) bolted to a 35 mm cut of 4x4 post (89 x 89), screwed to a 300 x 260 x 19 plywood base clamped to the bench | Under the box; the box's bottom 60 above the bench |
| Access door | The box's own: 265 x 105, hinged along its bottom edge, a magnet holds it shut | The lock's axis 22 mm in from the opening's right edge, 52.5 mm above the box's inside floor |
| Hopper door, flag | The box's own; shut and down | |
| Key | One of the box's two keys | Flat on the mat, its blade pointing away from the robot, its bow's centre at (215, -255) |
| Key mat | Black silicone, 100 x 100 x 3 | Centred at (227, -255) |
| Parcel | ULINE S-425 White Indestructo Mailer (4 x 3 x 2 in inside; outside 4-3/4 x 3-3/8 x 2-3/8 in, 121 x 86 x 60), white corrugated, with a 200 g calibration weight taped inside: 0.24 kg | Inside on the floor, lying on one of its 121 x 60 sides (60 across, 86 tall), its length front to back, its near end 12 behind the inside of the front panel, centred at y 50 |
| Parcel's place | An outline of 24 mm blue painter's tape on the bench, 160 (x) by 100 (y) inside | Inside centred at (235, 50) |
| Cameras | Two phones on tripods, for scoring | One looking at the mailbox's front from the robot's left, one from above the bench |

The start is the real resting state, nothing pre-started: access door shut and locked (key out, plug at locked),
hopper door shut, flag down, key on its mat, parcel inside. The end is the complete outcome: the parcel on its marked
place, the access door shut and locked, the key back on its mat, both hands clear and still.

The hopper door, its outgoing-mail clip and the flag are part of the product and are modelled, but this task does not
use them.

## Score

| Score | When |
|---|---|
| 1.0 | The 0.75 state, the key lying on its mat, everything still and both hands clear, held for 1 s |
| 0.75 | The parcel on its place; the access door shut and locked |
| 0.5 | The parcel on its place |
| 0.25 | The access door was opened by the key |
| 0 | Anything else; and for the rest of the trial once the key or the parcel has left the bench |

Every rung above 0 needs the door to have been opened by the key at some point. The rungs, as the scorer checks them:
- **Opened by the key:** the access door open more than 30 degrees while the plug was turned more than 45 degrees
  toward unlocked. (A parcel got out any other way scores 0.)
- **The parcel on its place:** seen from above, the rectangle that bounds the parcel, with its sides parallel to the
  bench's edges, inside the tape's inside edges (160 by 100), and its lowest point within 6 mm of the bench. It may
  stand or lie on any face.
- **Shut:** the access door within 1.5 degrees of shut. **Locked:** the plug within 5 degrees of locked (the key's
  slot horizontal; with the key out, look into the keyway).
- **The key on its mat:** seen from above, the rectangle that bounds the key inside the mat's edges or no more than
  5 mm past them, and its lowest point within 6 mm of the mat's top.
- **Still:** every moving part (both doors, the plug, the key, the mail clip, the flag, the parcel) under 10 mm/s and
  0.5 rad/s. **Hands clear:** every point of both grippers' finger pads, and the point midway between them, at least
  30 mm from the box (sides parallel to the bench's edges) that bounds each of those parts.
- **Left the bench:** its centre off the bench top, or its lowest point more than 50 mm below the bench top.
- **Guards:** a trial scores 0 if any part was not built, and from the step on which a part's state can't be read
  or the key's or the parcel's centre is not finite. If the arms' state can't be read, the hands can't be checked clear, so 1.0 can't be reached.
- **Real trial:** the trial video shows the door opened and the plug turned (the key's bow upright) and the 1 s
  hold; a photo from above with the tape in view shows the parcel inside it and the key on its mat; a protractor
  app against the door on a paused frame shows its angle; with the key out, the keyway's slot is horizontal at
  locked; a ruler gives 30 mm from the hands. Locked is also checked after the trial with a pull: a luggage scale
  hooked on a loop of tape stuck to the door's top edge, pulled outward to 5 N; locked, the door does not open past
  5 degrees (in the simulation a pull of 0.4 N m, about 4 N at the top edge, stopped on the cam at 3.0-3.2 degrees;
  unlocked, 1.5 N there overcomes the magnet).

## How the file works, and its fidelity

Every part is a rigid PhysX body at its real size and mass; the doors, the plug and cam, the key, the mail clip and
the flag hang on revolute or D6 joints. The file changes only drive set points, stiffness, damping and force limits
at run time, and only these, each gated on a real state:
- **The plug** is held by its wafers (a stiff drive at its end) unless the key is fully home: its shoulder within
  1 mm of the plug's face, on the lock's axis within 1 mm, square within 4 degrees. Then the drive lets go (a light
  snap within 6 degrees of either end) and only the blade's contact with the keyway's four walls turns the plug.
- **Key retention:** while the plug is turned 3 degrees or more off locked, a drive along the keyway (3,000 N/m,
  capped at 40 N) holds the key in; home at locked, a light grip (300 N/m, 0.3 N) keeps it from sliding out under
  its own weight.
- **The cam** is a 39 x 10 x 2.5 mm steel collider behind the front panel. Locked, it overlaps the right jamb by
  12 mm: the door can't open, and an open door can't shut.
- **Magnets:** within 3 degrees of shut, each door's drive pulls it shut at 0.05 N m per degree (so at most 0.15 N m;
  the hopper door's cap of 0.30 N m is never reached), off beyond.
- **The flag's catch:** past 45 degrees up, its drive's target moves to its up stop (the flag is not used here).
- **The keyway** is a 2.6 x 8.5 mm slot for a 2.0 x 7.5 mm blade. A real mailbox lock fits its key to a fraction
  of a millimetre, which PhysX at this time step cannot resolve, so inserting the key is easier in the simulation
  than on the real box; the real trial keeps the real fit.
- **Not simulated:** the wafers and the blade's bitting (looks), the weather strip, the anti-pry baffle behind the
  hopper, the box's inner partition.

## Sources and estimates

| Value | Used | Source / status |
|---|---|---|
| Mailbox outside | 15.2 x 12.0 x 18.1 in, 22.44 lb, 14/16 ga galvanized, powder coat, cam lock, 2 keys, magnetic closure, front access | Maker's product page (Architectural Mailboxes, Oasis Classic 6200), read 2026-11-21 |
| Layout | Hopper door hinged at its bottom with the outgoing-mail clip inside; flag on the right near the front top; access door at the bottom with its lock at its right end | Maker's mounting instructions (2010 Rev G, 2020 Rev E), read 2026-11-21 |
| Mount | 4x4 adapter plate, item 7530 | Maker's 2010 instructions |
| Lock | Quarter-turn cam, counter-clockwise unlocks, key out only when locked | CompX National catalogue (2020) for USPS-approved mailbox locks: typical, not this box's lock |
| Parcel | 4-3/4 x 3-3/8 x 2-3/8 in outside, 0.09 lb | ULINE S-425 listing, read 2026-11-22 |
| Openings and doors | Access opening 245 x 95; access door 265 x 105 x 6, 0.40 kg; hopper door 265 x 210 x 6, 1.0 kg; lock 22 mm in from the opening's right edge | **Estimates** from the maker's drawings, to measure |
| Lock parts | 19 mm bezel, 13 mm plug, 5/8 in cylinder, 39 x 10 x 2.5 mm cam, 2.6 x 8.5 mm keyway | **Estimates**, to measure |
| Key | Bow 26 x 24 x 2 mm with a 5 mm hole, blade 15.5 x 7.5 x 2.0 mm, 6 g | **Estimate** |
| Stops, magnets, drags | Access door 95 degrees, hopper 70; magnets 0.05 N m per degree within 3 degrees of shut; hinge drags | **Estimates**, to measure |
| Mounting height | 60 mm off the bench (the real box is 41-45 in up) | A bench stand-in for the post: the door needs about 45 mm of drop to fall open past level with the key in its lock |
| Friction | Steel 0.40/0.35, card 0.50/0.45, key 0.35/0.30, silicone 1.0/0.9 (static/dynamic) | **Estimates** |

## Forces against the robot

- Key turn and insertion: no published figures; cam locks are light (estimate 0.05-0.3 N m and 1-5 N). The turn is a
  face pinch on the bow's flat faces (form closure), so it does not rest on grip force.
- Door: the access door is 0.40 kg on a 105 mm leaf (about 0.2 N m open); lifting it at its free edge takes about
  2 N. The magnet's 0.15 N m needs about 3 N at the key.
- Parcel: 0.24 kg, pinched across its 60 mm width (the simulated gripper squeezes at its drive limit; holding 0.24 kg
  on card at a friction of 0.475 takes about 2.5 N per pad).
- Every load is well under the YAM fingers' roughly 40 N each.

## The scripted demo

It reads the simulator's state, as a scripted demo may, and only changes where the arms are sent; it never moves a
part or writes to the simulation.
- **Key in and turned (right arm).** It takes the key off its mat with an edge pinch (pitched 30 degrees so the
  forearm clears the hopper door), carries it level to the lock and lines it up 3 mm in front of the keyway by
  watching its pose; feeds it in at 4 mm/s and, if its tip stops on the face, circles the tip 0.5 mm until it drops
  into the slot; lets go at home, takes the bow again on its flat faces and rolls the wrist a quarter turn,
  never more than 10 degrees ahead of the plug.
- **Door open.** It takes the upright bow from above and draws the door open round its hinge until it passes its
  magnet, then slides its pads off; the door falls open to its stop with the key riding in its lock.
- **Parcel out (left arm).** Reaching in through the opening, pitched 8 degrees down, it pinches the parcel across its 60 mm width over
  its middle, lifts it 5 mm (the opening leaves 9 mm over it), draws it out 165 mm, lifts it 130 mm and holds it
  aside while the door is shut.
- **Door shut (right arm).** Pointing down and forward at 45 degrees, its fingers closed, it slips its fingertips
  under the open door's free edge, so that the edge rests on the fingers 45 mm back from their tips, and follows the
  edge up its arc, never more than 20 degrees ahead of the door (the soft arm presses only as hard as it is held off
  its target), until the magnet takes the door; near shut only the fingertips touch it.
- **Parcel down, lock, key back.** The left hand sets the parcel inside the tape and lets go. The right hand takes
  the turned key on its faces again, rolls it back a quarter turn to locked, takes it in the edge pinch, draws it
  out (it comes out only at locked) and lays it on its mat. Both arms park clear.

## Results

From real runs of this task file on the official stack. Only one GPU type was available to this build (NVIDIA RTX
PRO 6000); the demo has not been run on another GPU type.

| Run | GPU | Trials | Score |
|---|---|---|---|
| Scripted demo | NVIDIA RTX PRO 6000 | 3 | 1.0 on 3 of 3 |
| Dream-1 (`dream`) | NVIDIA RTX PRO 6000 | 1 | 0.0 |

- Each demo trial ended on success after 1,981 control steps (99.05 s of simulated time; the plan is 99.9 s, the run
  limit 4,000 steps). The three trials, run back to back in one process, took 3,731 s of wall time.
- Dream-1 ran all 4,000 steps (200 s of simulated time; 2,790 s of wall time). In six frames sampled evenly through
  its video, the access door stays shut and the taped rectangle stays empty; both arms stay near the front of the
  table, and nothing is seen picked up or moved. It scored 0.

Before the evidence runs, the demo was traced step by step on a copy of this file that differs only in comments, one
docstring, two warning strings and one local variable's name (3 trials in one process, from the start state above):
1.0 on 3 of 3.

| Trial | Key fully in | Door open past 30 deg | Door shut again | Parcel on its place | Locked | Key on its mat | 1.0 from |
|---|---|---|---|---|---|---|---|
| 1 | 16.60 s (shoulder 0.69 mm from the face, 0.48 mm off the axis) | 41.90 s | 68.40 s | 74.20 s | 79.85 s | 95.35 s | 98.10 s |
| 2 | 16.55 s (shoulder 0.85 mm from the face, 0.36 mm off the axis) | 41.90 s | 68.40 s | 74.20 s | 80.30 s | 95.35 s | 98.10 s |
| 3 | 16.60 s (shoulder 0.54 mm from the face, 0.46 mm off the axis) | 41.90 s | 68.40 s | 74.20 s | 79.85 s | 95.35 s | 98.10 s |

- At the end of each trial: the door 0.04 degrees open (shut), the plug 0.01 degrees from locked, the parcel standing
  inside its tape at (233, 50-52), the key on its mat at (213-215, -255), everything still and both hands clear.
- **Negative tests** (full runs from the start state, one trial each; each changes the demo's own plan and switches
  off the servo that would repair its mistake):

  | Negative | Score | What the trace shows |
  |---|---|---|
  | Do nothing | 0.0 | Nothing moved |
  | Key in, not turned; the door pulled by the key as in the demo | 0.0 | The plug stayed locked; the door moved at most 0.09 degrees (the cam on the frame) |
  | Parcel left inside (the left arm stays parked) | 0.25 | Door opened, shut and locked, key on its mat; the parcel still at its start |
  | Door left unlocked (the right arm parks once the door is shut) | 0.5 | Parcel placed, door shut, plug at -90 degrees, key in the lock |
  | Key left in the lock (turned back, not taken out) | 0.75 | Parcel placed, door shut and locked, key in the lock |

  A door pushed shut on a thrown cam is not an arm test: in this routine the key can be turned back only with the
  door near shut. With test drives on the same lock, a door pushed shut with 0.4 N m and the cam thrown stopped at
  10.9-11.1 degrees; with the cam clear it shut to 0.03 degrees.

## Real trial: safety and reset

- **People and the rig:** nothing sharp, hot or wet. The pads touch the key, the door's folded edge and the
  corrugated parcel. Check the door's free edge and the cam lock's bezel for burrs on the real box and break them
  with a file before the first trial; keep fingers of people clear of the doors' hinges.
- **Reset** (about 2 minutes; an estimate, not timed): if the key is not on its mat, turn it to locked and take it
  out; put the parcel back inside on its side at its marks (pencil lines on the floor of the box: its near end 12 mm
  behind the front panel, its centre 50 mm left of the box's centre line); shut the access door and check it is locked (it does not open by hand); shut the hopper door,
  flag down; lay the key on its mat at its marks, blade pointing away from the robot. Nothing is consumed.
- **Kit:** the mailbox (about $130 at a home-improvement retailer), a 4x4 post offcut, a plywood offcut, two clamps,
  a silicone mat, a 200 g calibration weight, a ULINE S-425 mailer, painter's tape.

## Assets and licences

- **This task** adds no third-party assets: every shape is built in the task file from primitives and closed
  triangle meshes written in code. No logo or trademark is drawn.
- **The robot** is not in this folder; it comes with the official package under Requirements. That package's
  README (at the commit above) says its YAM robot model (`yam_arena/assets/yam/`) "comes from i2rt under the MIT
  license"; the file beside it, `LICENSE.i2rt`, reads "MIT License, Copyright (c) I2RT Robotics". The same README
  says the package's own code has no licence yet.
