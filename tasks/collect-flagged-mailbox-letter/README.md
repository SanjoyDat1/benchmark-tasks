# Collect the letter from the flagged mailbox in a row of three, then latch its drop-down door and lower the flag

One task file, `collect_flagged_mailbox_letter.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.

Three standard curbside mailboxes stand side by side on a three-box spreader bar, their drop-down doors toward the
robot. The middle one has its red carrier signal flag up: the customer has left a letter in it. The robot is the mail
carrier: it opens that mailbox only, takes the letter out and puts it in the mail tray, shuts the door until its latch
catches and lowers the flag. The other two stay shut.

**The scene is fixed:** it is always the middle mailbox that is flagged. The two side mailboxes look the same, with
their flags down; they are there to be left alone. The instruction gives the flag rule, not the box, so the robot has
to find the raised flag in its cameras; opening a mailbox whose flag is down scores 0 for the trial.

The skill is **getting a thin, flat envelope out of a deep, narrow box with flat parallel fingers**, then two short
contact jobs against stops. The envelope (a 5 x 7 in card in its A7 envelope, about 2 mm thick) lies flat on the
mailbox's floor: the fingers cannot get under it where it lies. As people do with a card on a table, the robot must
press on it with a fingertip and draw it to the sill (enough press to carry it, not so much that the fingertip
skids or the envelope stays put), until part of it overhangs; then pinch the overhang, where the open door slopes
away below it and leaves only a narrow wedge for the lower finger; then draw it out of the 6.6 in (168 mm) opening without
catching the sill, the door or the box. The drop-down door then has to be lifted from under its free edge through
115 degrees and pushed past its latch, and the flag pushed past its detent, beside a neighbouring mailbox and its
flag. The instruction says which mailbox and what to do with the flag: the choice is not what is tested.

## Requirements

- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`
  (Isaac Lab-Arena `release/0.3.1`; `inspect-robots` and `inspect-robots-yam` as it installs them);
  `source "$ARENA_DIR/yam.env"` in the shell.
- Nothing to download: the task file builds its geometry from boxes, cylinders and closed triangle meshes.
- `-E control_hz=20`: the score's 1 s hold is counted as 20 control steps, so run at 20 Hz (the demo reads the
  control rate from the embodiment).

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/collect-flagged-mailbox-letter/collect_flagged_mailbox_letter.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The mailbox with its red flag up has a letter to go out. Take the letter out and put it in the mail tray, shut that mailbox until its latch catches, put its flag down, and move your hands away. Leave the other mailboxes shut with their flags down." \
  --scorer task_score --max-steps 3000 --store-frames --log-dir ~/yam-eval/mailbox-dream
```

To run the scripted demo, use the same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/mailbox-demo`.

## Why it matters, and what is real

- **Recognition.** A carrier emptying a flagged curbside mailbox is a scene nearly everyone in the US has watched
  from close up. About 40 percent of US delivery points are curbside (USPS Office of Inspector General, report
  RISC-WP-26-002, FY2025, p. 29). The flag rule is the Postal Service's own: "The customer should raise the flag to
  indicate that outgoing mail has been deposited" (Postal Operations Manual 321.2); city carriers "Collect prepaid
  mail from curb boxes when flag is raised even if there is no mail for delivery to these boxes" (Handbook M-41,
  June 2019, 371); "Letter carriers can pick up mail as they make their deliveries if the flag is up" (USPS FAQ,
  Outgoing Mail Pickup). The mailbox standard requires the flag to "remain in position until retracted by the
  carrier" (USPS-STD-7C 3.6). No USPS sentence found says in so many words that the carrier lowers it; the standard's
  "retracted by the carrier" is the closest.
- **Not Impact on jobs.** Postal Service Mail Carriers (SOC 43-5052) number 328,820 (BLS OEWS, May 2025), under the
  500,000 bar, so the task is entered for its recognisability.
- **No lock and no key.** These are standard, non-locking boxes: "The Postal Service does not allow carriers to open
  locked boxes and does not accept keys for this purpose" (Postal Operations Manual 632.527).
- **Closest known tasks.** BEHAVIOR-1K `bringing_in_mail` (mail inside a mailbox, goal: the mail on a table and the
  mailbox not open) has the same goal symbolically; its sibling activities `collecting_mail_from_the_letterbox`,
  `mailing_letters` and `sending_packages` put mail in or take it out. None has a flag, a latch, a row of boxes or
  contact physics in its definition: there the hard part, getting a flat envelope off a floor with a parallel
  gripper inside a box, is abstracted away. RoboTwin 2.0 `move_playingcard_away` picks a card from an open table.
  The ability tested here is drawing a flat envelope out of a deep, narrow box with flat parallel fingers, working a
  drop-down door's latch and a flag's detent, and leaving two look-alike boxes untouched.

## Setup and objects

Millimetres; x away from the robot, y to its left, z up from the table top.

| Object | Make, size, material | Place |
|---|---|---|
| Mailboxes (3) | Architectural Mailboxes (formerly Gibraltar) Elite post-mount, black, E1100BAM: 225.6 tall x 175.8 wide x 509.3 deep (8.88 x 6.92 x 20.05 in), 4.6 lb, galvanized steel, non-locking, USPS approved; arched top; front door hinged along its bottom edge with a latch tab at its top; red signal flag on the right side near the front | Fronts at x 380; centres y +288, 0, -288; bottoms 180 above the table |
| Spreader bar | Salsbury Industries 4883 three-wide mailbox spreader, 34 x 7.25 x 1 in, steel | Under the three boxes, centred x 635, y 0 |
| Post stand-in | A 4x4 post offcut, 136 tall, on a 3/4 in plywood base 300 x 300 clamped to the table | Under the spreader's middle |
| Letter | A 5 x 7 in greeting card in a white A7 envelope, 184 x 133, about 2 thick and 25 g (estimates), face up | On the middle mailbox's floor, long side along the box, centred, its near edge 25 behind the door's inner face |
| Mail tray | Officemate 21040 stackable letter tray (one of the two-pack), 248 x 318 x 56, black recycled PET, low front toward the robot | Centre x -50, y 0, long side along y (between the arms' bases) |

The bench stand-in: real boxes stand 41-45 in above the road (USPS-STD-7C 3.11.1); here their bottoms are 180 mm
above the table, enough for an open door to rest clear of it with room for fingers under its free edge. Nothing of
the mailboxes is moved, enlarged or turned for the robot.

## Score

| Score | When (at the end of the trial) |
|---|---|
| 1.0 | The letter in the tray; the flagged mailbox's door latched; its flag down; the other two shut with flags down; everything still and every finger at least 30 mm from the letter and from the flagged mailbox's door and flag (each one's bounding box; the door's includes its pull tab); held for 1 s |
| 0.75 | The letter in the tray and the flagged door latched, but not 1.0 (its flag still up, or the end not yet still, clear and held) |
| 0.5 | The letter in the tray, the door not latched |
| 0.25 | The letter was taken all the way out of its mailbox at some time and is not back inside it |
| 0 | Anything else, including the letter left in the mailbox and doing nothing |
| 0 for the whole trial | Only for a real failure: a mailbox whose flag is down opened (its door past 10 degrees at any time), or the letter fallen off the table (any of it 50 mm below the table top) |

- **In the tray:** seen from above, every corner of the envelope inside the tray's walls, and its lowest corner within
  30 mm of the tray's floor. **Latched:** the door within 1.5 degrees of shut (the latch catches within 3).
  **Down:** the flag within 10 degrees of lying along the side. **Out of its mailbox:** seen from above, the envelope's
  whole outline in front of the shut doors' faces. **Back inside:** any corner of it behind the door's face, within
  10 mm of the box's sides.
  **Still:** every moving part (the doors, flags, letter and tray) under 10 mm/s and 0.5 rad/s. **Held:** 1 s
  (20 control steps at 20 Hz).
- Every rung needs the other two mailboxes shut with their flags down at the time; a neighbour's flag knocked up
  past 10 degrees scores 0 only while it is up.
- **Nothing harmless zeroes a trial:** the flagged door opened and shut again, the tray pushed, a neighbour's flag
  nudged, the letter laid on the table or carried past its edge cost nothing for good.
- **Void (the trial scores 0):** the scene's parts were not all built, the arms' state couldn't be read at any step,
  a reading was not a finite number, or the parts couldn't be read.
- **Real trial, by hand:** a phone level on each door and flag, or a photo against a square (a latched door sits flush
  and stays shut when tapped; 10 degrees of a door is about 40 mm at its tab); a photo from above for the tray (the
  envelope inside its walls, lying in it); the trial video for the envelope leaving its mailbox, a flag-down mailbox
  opened, the letter off the table, the 30 mm between fingers and the work, and 1 s with nothing moving.

## How the file works, and its fidelity

Every part is a rigid PhysX body at its real size, built in the task file; the doors, flags, letter and tray have
their real (estimated) masses, and the mailboxes and their stand are fixed in place. The door's 6 mm plate and 2 mm rim
make each box about 515 x 180 x 228 mm overall, plus the tab. The doors and flags hang on
revolute joints. Two small rules, run every control step, switch joint drives by the real angles; they move nothing
by themselves otherwise, and no rule moves the letter.
- **The latch** (STD-7C 3.3: a "positive mechanical" latch, "not spring-loaded", opening with 1 to 5 lb): within 3
  degrees of shut a drive capped at 1.0 N m pulls the door shut and holds it (about 1 lb pulled at the tab); past 3
  degrees the door is free and falls open under its own weight to its stop at 115 degrees (STD-7C: at least 100,
  preferably 120 or less), where it stays until pushed shut.
- **The flag's detent** (STD-7C 3.6: stays up until retracted, 2 lb or less to lower): past 45 degrees a drive
  capped at 0.35 N m carries the flag on to its rest, up or down.
- **The letter** is a rigid 2 mm plate (a real card flexes a little). Paper on the galvanized floor: friction 0.30
  static, 0.25 dynamic, the lower of a pair (an **estimate**); the envelope's own 0.60 static, 0.55 dynamic against
  anything else (the fingers' material is the robot's own).
- **The hinge pin** sits 3 mm under the box's bottom edge (the door's rolled edge wraps it), so an open door's face
  lies just under the floor's edge and the envelope slides out over it.
- **The arches** (the box's roof and the door's top) collide as stacks of thin slabs inside the drawn curved shells;
  their inner corners show a little inside the roof.
- **Looks only:** the arch's outer shell, the galvanized inside, the door's flange and embossed line, the hinge
  knuckles, the flag's bracket, the envelope's stamp and address lines.
- **Simplified:** the latch's hook riding over the bead as it shuts (the sim's latch only pulls the door in within 3
  degrees; a real one also resists a little just before), the door's sheet steel (a 6 mm rigid plate), the spreader's
  bolts (the boxes are fixed in place).

## Sources and estimates

| Value | Used | Source / status |
|---|---|---|
| Mailbox | 8.88 x 6.92 x 20.05 in, 4.6 lb, galvanized steel, non-locking; latch tab at the door's top; flag on the right near the front, down pointing to the rear | Maker's page and photos (architecturalmailboxes.com, read 2026-11-22) |
| Door, flag rules | Opens 1-5 lb, positive latch, not sprung, stays open; >= 100 degrees (<= 120 preferred); flag on the right, within 2 in of the front, pivot 3.75-8.50 in up, >= 4 sq in of signal above the box, <= 2 lb to lower | USPS-STD-7C 3.3, 3.6, 4.10, 4.11, Fig. 1A |
| Spreader | 34 x 7.25 x 1 in | Salsbury 4883 retail listings; **spacing 11.33 in (34 / 3) is derived, to measure** |
| Door | 115 degree rest, 0.28 kg, latch 1.0 N m, tab 15 x 30 x 3 mm | **Estimates, to measure** (a luggage scale on the tab; a phone level) |
| Flag | Pivot 30 mm behind the front, 95 mm (about 3.75 in) up; 190 mm arm, 16 x 3 mm; 57 x 51 mm signal plate; 0.06 kg; detent 0.35 N m | **Estimates, to measure** |
| Letter | A7 envelope 7-1/4 x 5-1/4 in; 2 mm, 25 g | Size from retailers; thickness and mass **estimates**; within USPS letter limits (DMM 101 1.1) |
| Tray | 12.5 x 9.75 x 2.2 in | Officemate 21040 spec sheet. The carrier's own 1-ft letter tray (DMM 203 5.2: 10-1/4 x 10 x 4-5/8 in inside) is postal property and not sold |

## The scripted demo

The right arm does the whole routine; the left arm watches from its park. It reads the simulator's state every
control step, as a scripted demo may, and only changes where the arm is sent; it never moves a part or writes to the
simulation.
1. **Door:** a level pinch on the pull tab; the hand draws the tab round the hinge, at most 6 degrees ahead of the
   door, until the door is 8 degrees open (past the latch's 3); then it slides back and up as its fingers open, and the
   door falls open to its stop.
2. **Envelope to the sill:** shut fingertips pitched 35 degrees down press 6 mm into the envelope's top (on the soft
   arm, about 2 N) and draw it toward the robot, never more than 8 mm ahead of it, in two strokes (pressing 50 mm, then
   100 mm, behind its near edge, so the press always stays inside the box), until it overhangs the sill by 80 mm.
3. **Out:** a level pinch on the overhang, the lower finger in the wedge between the envelope and the open door; the
   near end lifted 30 degrees (the envelope pivots on its far end on the floor), then drawn out along its length.
4. **Tray:** turned a quarter turn about the vertical and tilted 40 degrees, carried over the tray and let go with
   its lowest corner 12 mm above the tray's floor and its middle 40 mm beyond the tray's centre (toward the mailboxes).
5. **Door shut:** the hand comes low in front of the open door, then in under it, the shut fingers' upper face
   carrying the door 40 mm in from its free edge; the hand leads the door up its arc by at most 20 degrees, then
   pushes its face 4 degrees past shut; the latch takes it within 3 degrees.
6. **Flag:** the fingertips on the flag's arm 160 mm from its pivot, the hand yawed 25 degrees toward the box and
   pitched down as the flag goes (10 degrees more than the flag has dropped, at most 30) so the push stays near square
   to the arm; it leads the flag down by at most 20 degrees to 36 degrees (past the detent's
   middle), and the detent carries it to its rest. Then the arm parks.

## Results

From real runs of this task file on the official stack. Only one GPU type was available (NVIDIA RTX PRO 6000); the
demo has not been run on another GPU type.

| Run | GPU | Trials | Score |
|---|---|---|---|
| Scripted demo | NVIDIA RTX PRO 6000 | 3 | 1.0 on 3 of 3 |
| Dream-1 (`dream`) | NVIDIA RTX PRO 6000 | 1 | 0.0 |

- Each demo trial ended on success after 1,416 control steps (70.8 s of simulated time; the plan is 78.6 s, the run
  limit 3,000 steps). The three trials, run back to back in one process, took 946 s of wall time (about 315 s a trial).
- Dream-1 ran all 3,000 steps (150 s of simulated time; 844 s of wall time). In six frames sampled evenly through its
  video, by 30 s the left arm has opened the left mailbox, whose flag is down, and that door hangs open in the later
  samples; the middle mailbox's flag stays up and its door shut, and the tray stays empty. It scored 0.
- Traced step by step (a debugging wrapper; an earlier copy of this file with the same routine, before the scene was
  fixed and the void rules were added), the demo took the envelope out at 37.65 s, had it in the tray at 42.3 s, the
  door latched at 58.2 s and 1.0 from 69.85 s, the same in each of 3 trials; at the end the envelope lay flat in the
  tray, 14 mm from its far wall, the door and flag within 0.02 degrees of shut and down.

- **Negative tests** (this task file, one trial each from the start state on the same GPU type; each changes the
  demo's own plan, so the servo that would repair its mistake is not in it):

  | What the robot does wrong | Expected | Score | What the trace shows |
  |---|---|---|---|
  | Nothing (both arms parked) | 0 | 0.0 | Nothing moved |
  | Opens the right mailbox (flag down) first, then does everything else right | 0 | 0.0 | That door passed 10 degrees at 8.3 s; the rest of the routine then put the letter in the tray, latched the door and lowered the flag: still 0 |
  | Opens the flagged mailbox, never touches the letter, shuts it and lowers the flag | 0 | 0.0 | The envelope still on the mailbox's floor |
  | Leaves the door open | 0.5 | 0.5 | Letter in the tray at 42.25 s; the door at 115 degrees, the flag down |
  | Leaves the flag up | 0.75 | 0.75 | Door latched at 58.15 s; the flag at 89.8 degrees |
  | Lets the letter go on the table beside the tray | 0.25 | 0.25 | Out at 37.6 s, then on the table; never in the tray (coming from that drop point the hand did not get under the door, which stayed open) |

## Real trial: safety and reset

- **Nothing sharp or wet:** the latch tab is a folded edge and the door's edges are rolled; check the tab for burrs
  and file them. No liquids, chips or heat.
- **Reset** (about 2 minutes; an estimate): open the middle mailbox, lay the envelope flat, face up, centred, its
  near edge on a pencil mark 25 mm in from the door's inner face; shut its door until the latch clicks and raise its
  flag until it stays up; check the left and right doors are latched and their flags down; set the tray on its tape
  outline.

## Assets and licences

- **This task** adds no third-party assets: every shape is built in the task file from primitives and meshes it
  computes. No logos, trademarks or fonts are drawn.
- **The robot** is not in this folder; it comes with the official package under Requirements. That package's
  README (at the commit above) says its YAM robot model (`yam_arena/assets/yam/`) "comes from i2rt under the MIT
  license"; the file beside it, `LICENSE.i2rt`, reads "MIT License, Copyright (c) I2RT Robotics". The same README
  says the package's own code has no licence yet.
