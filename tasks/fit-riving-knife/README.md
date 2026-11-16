# Refit and lock a table saw's riving knife after dado work

One task file, `fit_riving_knife.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.
A coworker has just finished cutting dadoes on a 10 inch cabinet table saw: the dado set is off and the blade is back
on, but the riving knife, which can't be used with a dado set, is still off. The robot fits it back and locks it: the
knife goes down onto two locating pins, flat against the mount's base plate, and the clamp's handle, which stays up
while open, is pushed down through its resistance until it clicks over centre and the pressure plate locks the knife.

**Why it matters.** Ripping boards on a table saw is core carpentry (O*NET 47-2031.00 Carpenters: "shape or cut
materials to specified measurements, using hand tools, machines, or power saws"), and putting blades and attachments
on saws is a listed task of saw setters (51-7041.00: "mount and bolt sawing blades or attachments to machine shafts").
The riving knife is what stops kickback. SawStop's manual says it may not be used with a dado set (p. 33), so every
time a dado set comes off and the blade goes back on, the knife has to be refitted before the next rip (OSHA 29 CFR
1910.213(c)(2) requires a spreader on hand-fed ripsaws). It is required setup between jobs, not an aside. A knife that
isn't down on both pins, or isn't locked, rides up or swings into the blade's path.

**What makes it hard.** The difficulty is physical, not knowing what to do. A 2.3 mm steel plate has to go straight
down onto two 6 mm pins with 0.5 mm of play, in a 14 cm opening a few centimetres from the teeth (touching them caps
the score), and stay seated and flat while an over-centre clamp handle is pushed through its resistance (up to about
8 N at its pad) to the click. An arm that holds the knife sags and lags a few millimetres under its weight, more than
the slots' play, so the knife is fitted as a person fits one: lowered down the gap beside the base plate, put against
the plate, and slid down it until the round pins steer its slots on. Unheld, the knife does stand on its pins (its
centre of mass is 6 mm behind the front pin), but tipped 2 degrees forward it would reach the teeth.

**No prescribed arms.** The rules say nothing about which arm does what, or how many. The scripted demo keeps hold of
the knife, seated on its pins, with one arm while the other pushes the handle, because of that snap; a policy may do
it any way that ends with the knife seated and locked.

## Requirements

- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`
  (Isaac Lab-Arena `release/0.3.1` with its object library pointed at NVIDIA's production bucket, `inspect-robots`
  1.0.0, `inspect-robots-yam` 0.38.0 and `inspect-robots-dream`). Built against Isaac Lab 3.0 / Isaac Sim 6.1 as
  recorded there.
- `source "$ARENA_DIR/yam.env"` in the shell.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/fit-riving-knife/fit_riving_knife.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The dado set has just come off the saw and its blade is back on. Fit the riving knife back on behind the blade and lock it in place." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/riving-knife-dream
```

The scripted demo (full score): the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/riving-knife-demo`.

Videos: `for log in ~/yam-eval/riving-knife-*/*.json; do inspect-robots video "$log"; done`.

## Score

Scored at the end of the trial (`SCORE_OVER = "end"`); a full score must hold for 1 s. The knife's position is that
of its own frame (the centre of its slotted foot plate), measured from where it is when home: both slots' closed ends
on the pins and its face flat on the base plate.

| Score | What must be true at the end | How to check it on a real saw |
|---|---|---|
| 0 | The knife is off the bench, or the arms' state can't be read, or (voiding the trial) the knife, the lever, the blade or a pin isn't the shape this file builds, or the simulator reported a non-finite number or a non-unit orientation, or the knife's or the lever's state is missing or unreadable, or the knife or a gripper moved more than 40 cm in one control step | |
| 0.25 | Both pins in the slots, the knife not down: within 1 mm of home along the blade, at most 8 mm up, over the pins' 2 mm length, turned less than 5 degrees | The pins visible inside both slots |
| 0.5 | Seated: both slots' ends on their pins (within 1 mm of home along and up), turned less than 3 degrees, the clamp not locked | The slots' ends resting on the pins; a steel rule across them |
| 0.75 | Locked: seated, flat on the base plate (within 0.4 mm), turned less than 1 degree, the handle down past its dead centre and the pressure plate on the knife (no more than 0.4 mm off it; it may press a little into it, as the simulated contacts give under the clamp) | A 0.4 mm feeler gauge goes neither between the knife and the base plate nor between the plate and the knife; the handle is fully down, below its click point |
| 1.0 | Locked, every gripper's tool point at least 3 cm, and a 3 x 3 grid of points over each finger pad at least 1 cm, from the knife's and the lever's outlines, both still (each moved less than 0.5 mm and 0.025 rad since the last control step: 1 cm/s and 0.5 rad/s), all held for 1 s (20 control steps) | Hands off; the knife doesn't move when tugged |

Capped at 0.5 if, at any one control step or anywhere on the way since the last one (every point moving in a
straight line), a gripper's tool point or any point of its two finger pads' faces (30 x 27 mm each) came within 5 mm
of the blade's tooth band, or any point of the knife (every one of its boxes, faces and edges included) within 2 mm of
it: carbide tips are brittle and chip when struck by hard objects, even with the saw unplugged. The band is the
blade's outer 8 mm (the carbide tips and gullets) out to its tips, 3 mm thick (the tips' width). The distances are
computed from those shapes at each step, not sampled at points, to within 0.1 mm and never as further than they are;
on the way between two steps the shapes are checked every 2 mm of travel, so to within about 1 mm. One touch is
enough, and a hand passing through between two steps counts. Only the teeth count: reaching past the blade's flat body
is not penalised.

"Locked" is physical: the lever is past the spring's dead centre (so the spring holds it down) and the plate rests on
the knife (so the knife is clamped). With no knife the lever goes 0.25 rad further, past the plate's high point, to its
stop; the plate then leaves 1.9 mm, too little for the knife, and that is not "locked".

"Still" is judged from where the knife and the lever are at each control step, not from the velocities the simulator
reports. In a GPU run, clamped and let go, neither changed at all (to 0.01 mm and 0.0001 rad) over the last 6 s, yet
the simulator kept reporting the knife at 1.0 cm/s and the lever at 0.89 rad/s (1.7 cm/s and 2 rad/s while the
handle was still held): velocities from the solver at the clamped contacts, not motion the scorer can see. The first
step of a trial has nothing to compare with, so it is never "still".

Why each rule: a knife on one pin, or not bottomed, rides up into the blade's path; an unlocked knife can be pushed
out of line by the workpiece; a knife off the base plate's plane steers the board into the blade; held by a gripper
it isn't fitted. Doing nothing scores 0 (the knife stays in its holder). Locking the clamp before the knife is in
fails: the closed plate leaves 1.9 mm for the 2.3 mm knife, so its slots can't reach the pins (0). Seating the knife and
never locking it scores 0.5. A knife turned round can't seat (its slots are not symmetric about its foot). Resting on
the pins' tops or on the closed plate isn't "in the slots" (0).

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes). The scripted demo plans 28.3 s: the right arm lifts the knife
  out of its holder by its back, carries it over the mount 11.5 mm clear of the top, lets it settle, lowers it down
  the middle of the 3.5 mm gap between the base plate and the open clamp until its foot is just above the pins, puts
  it against the base plate and slides it down the plate onto the pins (the round pins steer the slots on), and keeps
  hold of it there; meanwhile the left arm waits over the clamp's handle and, once the knife is seated, pushes the
  handle down through its 29 mm travel: it clicks over 19.5 mm down and locks the knife. The right arm lets go, and
  both lift clear. In the simulator the full score holds from step 499 and the trial ends as a success at step 519
  (25.95 s); three trials run back to back each took the same 519 steps. Wall clock on one GPU shared with the
  batch's other runs: about 3.4 minutes for the first trial (about 2 of them startup) and about 3.4 minutes for each
  further trial in the same process.
- The demo is open-loop: it reads nothing from the scene. Its right-arm targets allow for that arm's lag carrying the
  knife (up to 2 mm), as measured once in a GPU run of the official controller; the numbers are fixed in the file
  (`right_lag`).
- No randomness: the layout is fixed (`-E object_jitter=0`), so no seed is needed. With jitter the clamp lever, which
  hangs on its hinge, would be moved off its hinge axis; the file then sees a lever that isn't where its hinge holds
  it and scores 0.

## The saw and the mount

The sizes are SawStop's 10 in Professional Cabinet Saw's (owner's manual, specifications, p. 91): a 10 in blade with a
3 mm (0.118 in) kerf on a 2 mm plate, 3-1/8 in maximum depth of cut at 0 degrees, a 2.3 mm (0.090 in) riving knife,
and a 20 x 27 in cast-iron top; the saw is 34 in tall. Its riving knife clamp (pp. 32-33): lift the clamping handle
fully up and the clamping plate moves away from the knife; the knife goes into the clamp flat against the base plate,
where the positioning pins align it; lower the handle completely to lock it. With a dado set neither the blade guard
nor the riving knife may be used (p. 33); a dado set also needs its own brake cartridge and table insert (p. 29).
SawStop's support pages describe the handle as turning a cam that pushes the pressure plate across onto the knife.

What is modelled, and how:

- **The saw.** Only the top's middle 485 x 485 mm, its 2 cm cast-iron plate and the cabinet's sheet-steel walls below
  it, the blade (254 mm at its tips, raised fully: 79.4 mm above the top; a 2 mm plate with 40 carbide tips, 3 mm wide,
  turned to face out within its rim; the tips are looks only) and the knife mount. The table insert is out, as it is
  for a knife change. The opening around the blade (140 x 340 mm) is modelled, not measured: the manual
  doesn't give the insert's size, and the real one is likely narrower; this one leaves room for a gripper to reach the
  handle.
- **Why the cutout.** The robot's bench is 75 cm high, and its arms reach straight down from no higher than about
  95 cm. A cabinet saw's top is about 86 cm up and the knife stands 8 cm above it, so the saw's top is set 9.5 cm above
  the bench. For a real trial: stand the saw on the floor in a cutout in the robot's bench and raise the bench 2 cm (to
  about 77 cm), so the saw's top is 9.5 cm above it as here.
- **The mount.** A steel base plate behind the blade with two positioning pins (6 mm across, 2 mm long) and a clamp.
  The real clamp is a handle turning a cam that pushes a separate pressure plate across. Here the plate is part of
  the lever: its 2 mm face sits just below the hinge axis, 15 mm in front of it, so near the knife it moves like that
  cam close to its high point, a few tenths of a millimetre for the lever's last tenths of a radian, which turns a
  modest spring torque into a firm clamp. Past the high point it backs off again; with no knife the lever goes on to
  its stop. The handle sits on a shaft that runs forward under the opening (modelled: the real handle is at the
  clamp), so it is in front of the blade, on the robot's side.
- **Over-centre lock.** The manual has the handle lifted fully up, the knife put in, then the handle lowered
  completely, so the open handle stays up on its own; how the real handle feels on the way down isn't documented. The
  dead centre at 70% of the travel is this model's choice, so that locking is a deliberate push through resistance and
  not a touch. The over-centre action is a spring between a seat on the casting and an arm on the lever, acting along
  a fixed line (a PhysX D6 joint with one linear drive, 60.5 N/mm). With the handle up it holds it there (0.1 N m); pushed down, the handle meets rising
  resistance, up to about 8 N at its pad; 70% of the way down (19.5 mm) the spring's line crosses the hinge and the
  handle clicks over, snaps down and presses the plate on the knife. Reopening takes about 7 N up at the pad. A light
  rotary damper (0.05 N m s/rad) on the hinge slows the snap to about 60 ms.
- **The clamp force, and how it was chosen.** SawStop doesn't publish its clamp force; the manual says it is
  adjustable. At least 100 N was chosen so that friction alone, steel on steel (about 0.2) on both faces, holds the
  knife against a 40 N pull along its slots: 2 x 0.2 x 100 N = 40 N, 32 times the knife's weight. The 40 N is a design
  assumption, not a published load. Depending on where across the plate's 2 mm face the contact settles, the clamp is
  100 to 250 N. In the simulation the parts carry the framework's rubber-like friction (1.0), so the hold is larger.
- **The knife.** 2.3 mm steel plate (0.133 kg, 140 x 140 mm): its front edge 5 mm from the blade and its top 3 mm
  below the teeth (Bosch and Grizzly manuals give 3 to 8 mm), its foot with two open slots (7 mm for the 6 mm pins,
  14 mm deep). The slots are placed so that, unheld, it stands on its pins: its centre of mass is 6 mm behind the
  front pin and 17 mm ahead of the back one (the real part's outline isn't published). It is built from boxes, so its
  front edge is stepped every 5 mm; the gap is 5 mm at the steps' corners and a little more between them.
- **The knife's holder.** On the bench to the right of the saw (y -42.4 to -34.7 cm; the knife in it spans y -42 to
  -28 cm): two wooden blocks 1.7 x 6.2 x 2.5 cm each side of a 3 mm slot, and a 1.2 x 0.8 x 6.4 cm post under the
  knife's lower front edge. The knife stands in it upright, as it goes on the saw.
- **The table insert stays out.** On SawStop's saws the insert slides in at an angle around the riving knife and under
  two screw heads at the back of the opening, and is held by a front lock-down screw (a 3 mm hex key). That is tool
  work outside straight-down grips, and lifting a plate over a blade standing 8 cm proud is beyond the arms' reach, so
  it is left out: the task ends with the knife locked, ready for the insert.

Every object uses the framework's rubber-like friction (1.2 static, 1.0 dynamic), and so do the knife's and the
lever's added parts, except the knife's slotted foot: steel on the steel pins and base plate, lightly oiled (0.35
static, 0.30 dynamic, the lower of the pair's two). With rubber-like friction the round pins couldn't steer the slots
on.

## How the file works

- **Scene hook.** The task-file format has boxes and upright cylinders only. Right after the framework imports its
  scene builder, the file wraps `yam_arena.scene.task_object`. It recognises its own objects by value (they are frozen
  dataclasses), so an object of the same name from anywhere else is left alone. The blade and the pins are spawned
  with their axes along x; the knife's spawner adds the rest of the knife (31 boxes) as colliders of its rigid body;
  the lever's adds, first, the hinge (a `UsdPhysics.RevoluteJoint` with limits and the damper) and the spring (a
  generic `UsdPhysics.Joint`, every motion free, with one linear drive), then its plate, handle and spring arm; the
  blade's adds its teeth (looks only). Where Isaac Lab isn't installed nothing is installed, and the file warns.
- **It checks the scene was built.** Every step, the knife's and the lever's reported outlines must match the shapes
  the file builds, turned as reported, within 0.5 mm, the lever centred where its hinge holds it, and the blade and
  pins where they belong, lying along x (the blade's outline takes in its teeth, 3 mm across the 2 mm plate and inside
  its rim, since Arena's outline covers every shape under an object). If anything fails to build, what is left is a
  bare shape that fails this check. A mismatch, a non-finite number, a non-unit orientation, a knife or lever
  missing from the step's states or unreadable, or the knife or a gripper moving more than 40 cm in one control step
  (8 m/s, far beyond a normal motion of these arms: most likely a reset the file didn't see, or a broken reading), voids
  the trial.
- **The knife's pose.** The framework reports an object's world-aligned bounding box (the box of its own-frame
  bounding box turned into the world) and its orientation; the file recovers the knife frame's origin from them.
- **The lever's angle** comes from its orientation (a turn about +y).
- **The arms.** Task files get only the objects' states, so the file finds the calling embodiment (the caller with
  an episode clock, a task and a `reset`) and uses it once its reset is hooked; a new trial then starts a fresh
  record (also if the episode clock goes back). Every step it reads both arms' joint positions and turns them into
  tool points and finger pads with the framework's forward kinematics. If the arms can't be read on a step, it warns
  (once per trial) and that step scores 0.
- **The demo** is open-loop: `demo()` returns waypoints computed from the layout's constants only.

## Sources

- SawStop 10 in Professional Cabinet Saw owner's manual: table insert, p. 28; dado sets, p. 29; riving knife and
  blade guard installation, and dado sets, pp. 32-33,
  https://www.manualsdir.com/manuals/463780/sawstop-30hp.html?page=30 (p. 28), ?page=31 (p. 29), ?page=34 and
  ?page=35 (pp. 32-33); specifications, p. 91, https://www.manualsdir.com/manuals/463780/sawstop-30hp.html?page=93
- SawStop support, "Adjusting Guard Clamp" (the handle turns a cam that pushes the pressure plate):
  https://www.sawstop.com/?p=706 (read through search results; the page refused a direct fetch)
- SawStop quick-start manual, changing the table insert (rear screw heads, front lock-down screw):
  https://www.manualsdir.com/manuals/463767/sawstop-sawstop.html?page=8
- Riving knife 3 to 8 mm from the blade: Bosch's online table-saw manuals ("Adjusting the riving knife") and the
  Grizzly G0690 manual, https://www.manualslib.com/manual/1044096/Grizzly-G0690.html?page=41 (read through search
  results)
- Carbide tips are brittle and shouldn't take sharp blows from hard objects: Sheffield Industrial Saws, technical
  advice, https://sheffsaws.squarespace.com/technical
- OSHA 29 CFR 1910.213(c)(2): hand-fed circular ripsaws shall be furnished with a spreader.
  https://www.ecfr.gov/current/title-29/subtitle-B/chapter-XVII/part-1910/subpart-O/section-1910.213
- O*NET 47-2031.00 Carpenters, https://www.onetonline.org/link/summary/47-2031.00; 51-7041.00 Sawing Machine Setters,
  Operators, and Tenders, Wood, https://www.onetonline.org/link/summary/51-7041.00

## Assets and licence

No third-party assets. Every shape in the scene (the saw's top and cabinet, the blade and its teeth, the base plate and
pins, the clamp lever and its spring seat, the knife and its holder) is built in `fit_riving_knife.py` from the
framework's boxes and cylinders and USD cubes; nothing is downloaded or loaded from an asset library.
