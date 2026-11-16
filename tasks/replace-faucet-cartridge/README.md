# Replace a faucet's cartridge so hot stays on the left

One task file, `replace_faucet_cartridge.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.
A single-handle bathroom faucet with its handle off stands on a piece of vanity top, its hot supply on the left
and cold on the right. Its handle has gone stiff and won't open fully. The robot checks the fault, swaps the worn
1225-type cartridge for a new one the way the manufacturer's sheet says, and tests its work:
1. **Test before.** Lift the old stem (it binds about 2 mm up, short of its 6 mm full-on) and try a turn (it
   binds too). Push it back down.
2. Slide the copper retainer clip out.
3. Pull the old cartridge out by its stem.
4. Push the new one fully home with its tabs in the valve body's cut-outs.
5. Slide the clip back in. It only goes when the cartridge is home, to within about half a millimetre.
6. Set the stem so that, once the handle is refitted, swinging it to the left opens the hot supply.
7. **Test after,** with the clip home. Lift the new stem to full-on, swing it to the full-hot end and to the
   full-cold end, return it to the centre and push it down (off). The cartridge must stay put under its clip.
8. Put the old cartridge in the parts tray (any time after step 3).

## Requirements

- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`:
  - Isaac Lab-Arena `release/0.3.1`, here at `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`;
  - `inspect-robots` 1.0.0 and `inspect-robots-yam` 0.38.0;
  - `inspect-robots-dream` at `b5e936a201cec66c54c297e522c920d98445e339`;
  - Arena's assets read from the production bucket, as in Robocurve's November 5 launchable.

  Written for Isaac Lab 3.0 / Isaac Sim 6.1 as pinned there. Results on that stack are in "Runtime and seeds".
- `source "$ARENA_DIR/yam.env"` in the shell.
- No downloaded or third-party assets: all geometry is built by the task file.
- `-E control_hz=20`, as in the commands below. The file reads the control rate from the embodiment when it can,
  and otherwise counts its 1 s hold as 20 steps.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/replace-faucet-cartridge/replace_faucet_cartridge.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "The faucet handle is stiff and won't open fully. Check it, then replace the cartridge with the new one, set so that once the handle is refitted, hot is on the left. Make sure the faucet works before you leave it off, and put the old one in the parts tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/faucet-dream
```

To run the scripted demo, use the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/faucet-demo`.

To make the videos: `for log in ~/yam-eval/faucet-*/*.json; do inspect-robots video "$log"; done`.

## How hot and cold work here

This follows the real 1225:
- **The cartridge.** A plastic shell holds a brass piston on the end of the stem. Pulling the stem up opens the
  water and turning it sets the temperature.
- **The notch.** Moen's support articles say the notch on the stem sets where hot and cold are
  ([1225 Cartridge: Kitchen Faucet](https://solutions.moen.com/Article_Library/1225_Cartridge:_Kitchen_Faucet),
  [1225 Cartridge: Moentrol with Integrated Transfer Valve](https://solutions.moen.com/Article_Library/1225_Cartridge:_Moentrol_with_Integrated_Transfer_Valve)).
  The stem turns a full circle, which is how an installer swaps hot and cold when the supplies are piped backwards,
  so a stem set wrong gives hot on the right.
- **Rotation stops.** The cartridge has none. Where a trim limits the handle's swing, the stops are in the handle
  or its stop tube; that is reported for Moen's shower valves and was not checked for this faucet.
- **Not checked:** the piston's port layout and the quarter-turn swing are modelled, not taken from a drawing,
  since Moen publishes none.

The model has the same parts. The piston, its port and the handle's swing are a model of the internals, not
taken from a drawing:
- **Valve body:** an inlet port through its wall low down on each side: hot on the left as a user facing the
  faucet sees it, cold on the right. Each takes a brass supply shank with a red or a blue nut.
- **Cartridge shell:** an inlet window on each side, perpendicular to its tabs. The tabs key the shell into the
  body front-back, so its windows always face the inlets.
- **Stem:** it turns in the shell against its O-rings' drag (a damped joint) and slides up toward on. A small
  spring detent ball in the shell's top presses on it, so it stays where it is left (on or off). Inside, it ends
  in a brass piston with one port, on the notch's side. At full-on (6 mm up) the port stands level with the inlets.
- **The fault:** the old stem is stiff. Its joint stops it 2 mm up and 15 degrees either way, so the faucet won't
  open fully. The new stem lifts its full 6 mm and turns freely.
- **Setting:** with the stem's notch to the front, the port faces the front, between the inlets. A left swing of
  the handle (modelled as a quarter turn, clockwise seen from above) brings the port onto the hot inlet. Set with
  the notch to the back, the same swing opens the cold one.

So the score's rule is Moen's: the new stem's notch faces the front, within 20 degrees. The code computes it from
the stem's turn as the port's angle after the modelled swing, against the hot inlet's direction. Because the port
is on the notch's side, that is the same rule. No water flows: the ports, the piston and the windows have no
effect in the simulation beyond their geometry, and the tests are mechanical: how far a stem lifts and turns.

## Score

`--scorer task_score` gives 1.0 when the task succeeds (a full score held for 1 s); otherwise it gives the task
file's `score()` at the end of the trial. Every step also reports `info["score"]`.

The score is 0 in either case:
- the new cartridge, its stem or the clip has at any point left the table: its outline's lowest point is 5 cm
  below the table top, or its centre is off the table;
- while the old cartridge is still in the valve body: its lowest point is no more than 2 mm above the rim, and its
  centre is within 2.5 cm of the bore's axis. A cartridge set down against the column stands 3.2 cm off, so it
  counts as out.

Otherwise:

| Score | What must be true at the end |
|---|---|
| 0.25 | The old cartridge is out of the valve body, after the clip was taken out of its way (below) |
| 0.5 | The new cartridge is home, having gone home with the clip out of its way (below), however its stem is set: its body's bottom within 1 mm of where its tabs on the cut-outs' floor put it, its centre within 1.5 mm of the bore's axis, tilted under 3 degrees, its tabs within 8 degrees of front-back |
| 0.75 | Home, clipped (the clip put back into its way once it was home) and set for hot on the left, and not risen past the slots' play under the clip (2 mm, below) at any point since it was first clipped. **Clipped:** the clip lies flat (under 5 degrees), its legs within 5 degrees of front-back, its outline reaching at least 11.5 mm past the bore's axis both ways, its lowest point resting on the slots' floor (0.5 mm below to 0.2 mm above it), centred within 2 mm, over a cartridge whose bottom is within 0.5 mm of its seat. **Hot on the left:** the new stem's notch faces the front within 20 degrees, where the model's port uncovers more than half of the hot inlet on the handle's left swing |
| 1.0 | All of the above, and: the faucet was **tested before** and **tested after** (below); it is **left off**, the new stem within 1 mm of its lower stop; the old cartridge lies in the parts tray (centre inside its walls, lowest point under 1.5 cm above its floor); every gripper's tool point and finger pads are at least 1 cm from the new cartridge, its stem and the clip; and those three are still (under 1 cm/s and 0.5 rad/s). All this held for 1 s (until then it reads 0.75) |

**The tests.** Each is read from the stems' poses on their cartridges:
- **Tested before:** the old stem lifted at least 1.5 mm up its cartridge while the cartridge is still in the
  valve body, before the clip has first moved 2 mm from where it was put. (It binds at 2 mm. An old stem resting
  lifted in the tray later doesn't count.)
- **Tested after:** once the new cartridge has been home and clipped, with the clip retaining it, its stem lifted
  at least 5.5 mm (full-on is 6) and, so lifted, turned so its port faces within 10 degrees of the hot inlet, and
  also of the cold inlet, in either order. A stem swung while down doesn't count: the check is made with the faucet
  open. **Retaining:** as clipped, but the clip's legs anywhere in their slots' play, over a cartridge still keyed
  in the bore (on its axis, upright, tabs within 8 degrees) and no higher than the legs let it (its shoulder at
  most 0.5 mm into their underside, and at most 2 mm above its seat). A lifted stem can
  draw a clipped cartridge up the 0.5 mm gap under the legs and then the legs up their 1 mm of play to the slots'
  ceiling: 1.5 mm. The limit allows 0.5 mm more; a cartridge without its clip rose 4.5 mm in the simulator.

A job done right but without either test scores 0.75, as does one that leaves the faucet on. A cartridge pulled
more than 2 mm up under its clip (the clip didn't hold it) is capped at 0.5 for the rest of the trial, and counts
as home again only once reseated with the clip out of its way (below); one lifted with the clip out of its way, to
redo the seating, is not capped. A cartridge drawn up within the play must be home and clipped again at rest for
0.75 or 1.0.

**In order.** A cartridge can only be changed one way. The clip comes out of the cartridge's way before the old
cartridge can come out, the new cartridge goes home with the clip out of its way, and the clip goes back once it is
home. So each rung counts only when reached that way:
- the old cartridge counts as out only after the clip has been seen out of its way while the old one was still in
  the valve body, and never if it was forced past the clip (below);
- the new one counts as home only if it was seen home while the clip was out of its way, and wasn't forced past
  the clip on its way there. Once forced past it, it counts again only after it has stood clear above the slots
  (or off the bore's axis) and gone home again with the clip out of its way. It also needs to be seen home with
  the clip out of its way again after being forced more than 2 mm up its bore with the clip in its way;
- it counts as clipped only if the clip's latest return into its way found it home.

**Forced past the clip** means one of two things. Either the cartridge is seen with the clip over its shoulder,
its shoulder more than 4 mm above its seat and its bottom still below the slots' top (68 mm up). Or it goes from
one side of that span to the other in one step with the clip over its shoulder at either end: a jump whose order
the readings can't show. Why 4 mm: with its shoulder 3 mm up, a cartridge is at the slots' ceiling, past wherever
the legs can be in their slots, and the other 1 mm is for the solver's give and the readings. The legs let a
cartridge rise only 1.5 mm (the 0.5 mm gap under them and their 1 mm of play). With the clip home, the old
cartridge rose at most 1.55 mm in the demo's before-test in the simulator, and 1.72 mm when an earlier build pulled
on it hard: 2.3 mm or more short of the limit. A cartridge forced through the clip crosses every depth from 4 to 68
mm, so a lower limit would catch nothing more.

**Over its shoulder** means some part of the clip within 11 mm of the bore's axis in plan, measured as for "out of
its way" (below). That is the cartridge's 12 mm radius less its 0.5 mm side play in the bore and 0.5 mm for a
contact's give, so a clip that only touches a cartridge's side doesn't count as forcing it. In the simulator, leg
tips pushed back against a cartridge standing 16.8 mm proud, 0.5 mm off the axis, stopped 11.5 to 11.6 mm from it.
Pushing the clip early like that, backing off and doing it properly loses nothing. Credit still needs the clip out
of its way.

**Out of its way** is read from the clip's own shapes (bight, legs and tab), turned and tipped as it lies. It is not
read from its outline, a box square to the world, which round a clip set down turned takes in up to 21 mm of empty
space:
- for a cartridge going in, the clip is out of its way only when no part of it comes within the cartridge's radius
  (12 mm) of the bore's axis in plan, however high it is. With the legs along the slots, that is where their inner
  edges meet the shoulder's edge: 9.4 mm either side of the axis with the clip centred in its slots, and 9.75 mm
  with it at either end of its 0.5 mm side play;
- for the old cartridge coming out, the clip is also out of its way when it lies wholly on or above the rim (its
  lowest point at most 0.5 mm into it). The valve body doesn't hold it there, so the rising cartridge would lift
  it off. The same goes for a clip wholly below the slots' floor: over the bore, only the old cartridge and the
  valve body are there.

A clip slid only part way out still covers the shoulder, so it isn't out of the way. The demo slides it 31 mm, to
2.0 mm past that point (1.7 mm at either end of its side play). A real clip in a cartridge's way blocks it both
ways, so these rules only refuse credit to a state that a solver let a cartridge through.

**One cap for every mistake left in the faucet:** a home cartridge that is unclipped, or set anywhere but hot on
the left, stays at 0.5. This covers a stem left as it came, a reversed stem, and anything in between. There is one
tolerance on the setting (20 degrees) and no cliff between "wrong" and "very wrong". An unclipped cartridge can blow
out of the body under supply pressure, and a reversed one is a scald risk and a code violation; either way the
plumber comes back. So neither outranks the other.

**The old cartridge is scrap:** if it falls off the table, only the tray condition fails.

**Either way round:** the cartridge's tabs are symmetric, so it may go in with either tab at the front; its shell
windows face the inlets either way. The clip may go back in from the front or from the back, as the manufacturer
allows.

**In a real trial:** use an old cartridge that is stiff but not seized, and pull the clip with pliers if your
gripper can't. Push the clip back home rather than pinching it in. With its legs in both walls' slots it can turn
only about 1.6 degrees, so a clip held at the wrist's angle can wedge. To score:
- read the tests from the video: the old stem lifted before the clip moved; the new one lifted fully and swung
  both ways, then pushed down;
- read "home" off the tabs, which must be flush with the rim;
- check "clipped" by eye: the legs pass right across through both slots;
- after the trial, refit the handle, turn the water on at the stops, open the handle fully, swing it left and feel
  for hot water;
- read hands clear and still from the video.

## Why this task

**Core plumbing repair.** Replacing the cartridge is the standard fix for a dripping single-handle faucet (see
the repair guides from [Lowe's](https://www.lowes.com/n/how-to/repair-leaky-single-handle-faucet) and
[This Old House](https://www.thisoldhouse.com/plumbing/how-to-repair-a-dripping-single-handled-faucet)), and for a
stiff one: Moen's FAQ
[Handle is difficult to operate](https://solutions.moen.com/FAQ's/FAQ's-Troubleshooting/General/Handle_is_difficult_to_operate)
(read 2026-11-12) puts it down to debris, mineral buildup or a damaged seal in the cartridge, which "will need to
be serviced or replaced". For a 1225 it first suggests greasing the cartridge; this task is the replacement.
Plumbers (O*NET 47-2152.00) and maintenance and repair workers (49-9071.00) do it as routine work, and the 1225 is
one of Moen's common cartridges.

**Test before and after.** A tradesperson confirms the fault before taking the faucet apart and checks the work
before leaving. Moen's procedure ends with a hot and cold check: "If the hot and cold are reversed", turn the stem
180 degrees (the article below). Without water in the simulation, the robot makes the check mechanically: the stem
must lift to full-on and swing to both ends, and the cartridge must stay put under its clip.

**The difficulty is in the hands.** Each step is contact-rich, millimetre-scale manipulation:
- pinch a 10 x 8 mm tab and slide a 1.5 mm copper clip out of two slots;
- pull the old cartridge out by its 8 mm stem against about 10 N of O-ring drag;
- insert the new one keyed by its tabs into 12 mm cut-outs, and push it fully home against the same drag;
- thread the clip back through both slots, which it passes only with the cartridge home to within about 0.5 mm;
- turn the stem to within 20 degrees of its set alignment;
- work the stems: lift the stiff old one against its bind, and the new one to full-on and both ends, without
  pulling the cartridge up, then push it down to off.

**The stem's alignment.** The setting is a physical alignment the robot must achieve: notch to the front, as the
old stem stood when it came out. In the model that alignment puts the piston's port where the handle's left
swing opens the hot inlet. Plumbing codes require hot on the left, for example IPC 607.4 as adopted locally
([City of Chicago 18-29-607.4](https://codelibrary.amlegal.com/codes/chicago/latest/chicago_il/0-0-0-2691725))
and the UPC's equivalent. The exact model-code wording wasn't checked against the ICC book.

**No artificial constraints.** The instruction states the goal only: any arm or both may be used, in any order
that works. All geometry is built by the task file; there are no third-party assets.

**Moen's own procedure.** The task follows it step by step ([1225 Cartridge: Single Handle Bathroom Faucets
(Prior to 2009)](https://solutions.moen.com/Article_Library/1225_Cartridge:_Single_Handle_Bathroom_Faucets_(Prior_to_2009)),
read 2026-11-12):
1. Slide the copper retainer clip out.
2. Raise the stem and pull the cartridge out by it.
3. Push the new cartridge completely in, its plastic tabs flush with the valve body and centred on the cut-outs at
   12 and 6 o'clock.
4. Slide the clip back in on top of the cartridge. Don't force it: if it resists, the cartridge "isn't aligned".
5. Turn the stem so the small notch faces forward. If hot and cold are reversed, turn the stem 180 degrees.

**Not a variant of the board's sealant-gun reload.** That task restores a gun to working order. Here the work is
drag, keying, a retainer that proves the cartridge is home, a precise rotation, and a function test of the valve.

## What the robot sees

The instruction is a work order: the handle is stiff and won't open fully; check it, replace the cartridge with
hot on the left, make sure the faucet works, leave it off, and put the old one in the parts tray. The scene shows
the rest: the clip's
tab, the tabs in the cut-outs, the supplies' red and blue nuts, and the stems' dark notches. The old stem's notch
faces the front, which is the alignment the new stem must be given. The order follows from the parts:
- the clip has to come out before the cartridge can;
- the new cartridge has to be pushed fully home before the clip goes back;
- the fault is checked before the clip comes out, and the work after it goes back.

At 320 x 176:
- **From a wrist camera** 6 to 10 cm above the stem, the notch is about 4 to 5 pixels long, and each supply's red
  or blue nut is about 8 to 10 pixels across.
- **From the top camera,** the notch is under a pixel and each nut about 2 pixels. Aligning the stem needs a close
  view.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes). The scripted demo parks at 104.3 s and its plan is 106.8 s
  long in all.
  - In the simulator, three trials run back to back each scored 1.0 and ended as a success at step 2,069 (103.45 s),
    the full score having held for 1 s.
  - The Dream-1 trial scored 0.0 and ran all 2,400 steps.
  - Wall clock on one GPU shared with three other runs: about 38 minutes for the first trial (startup included) and
    about 35 to 38 minutes for each further trial in the same process.
- No randomness: the layout is fixed (`-E object_jitter=0`), so no seed is needed.

## The scripted demo

The faucet stands in front of the right arm. That is where its straight-down grips reach high enough to lift a
10 cm cartridge clear of the rim; the left arm's straight-down grips can't reach that far across. So the demo does
the job with the right arm, as a plumber can, and parks the left. That is the demo's choice, not a rule: a policy
may use either arm or both. In order, the right arm:
1. Tests the old stem: grips it, lifts it aiming 12 mm up (it binds at 2), tries a 30 degree turn (it binds at
   15), turns it back, pushes it down past its lower stop, and lets go.
2. Pinches the clip's tab and slides the clip 31 mm out. The legs' tips clear the cartridge's top, still in their
   slots, and the clip rests level on the spout, which joins the valve body just under the slots.
3. Grips the old stem, raises it to its stop, pulls the cartridge out, and drops it in the parts tray.
4. Lifts the new cartridge out of its holder by its body, the pads closing across the body's top 14 mm, below the
   tabs, so its tabs keep their bearing; the stem turns freely in it and couldn't hold them. It draws the cartridge up
   out of the holder slowly, lowers it until its bottom hangs 3 mm over the rim, lets it hang still, and eases it
   into the bore at about 8 mm/s, then lowers it on until its bottom is 39 mm in, just above the ball plungers,
   and lets go. There the bore's 0.5 mm play holds it within about 1.5 degrees of upright, and it settles on the
   balls. The arm then closes its fingers over the stem and pushes the cartridge home by the stem's top, straight
   down its axis, aiming 32 mm past home for the arm's give against the plungers' drag. Earlier versions carried
   it by the 5 mm boss round its stem and pushed it home by that grip. In the simulator it then hung anywhere from
   0.5 to 4.1 degrees off vertical from one trial to the next, against the 1.7 degrees that the bore's 1.5 mm
   lead-in allows, and once it jammed in the bore's mouth, went in turned 6 degrees and stayed 0.7 mm proud, so
   the clip couldn't go home.
5. Grips the stem, which the push left down on its lower stop (off), and turns it a quarter turn anticlockwise,
   from the left to the front.
6. Pushes the clip back home with its closed fingers on the flat front face of the tab, 3 mm above the clip's
   bar. It slows for the last 8 mm and stops with the legs' tips 16 mm behind the bore's axis, inside the back
   wall's outer face: pushed across that face, the legs' ends caught on it and the clip hopped in the simulator.
   The legs still reach 4.5 mm past what "clipped" needs. Nothing holds the clip, so its slots set its angle. (A
   pinch would lock it to the wrist's angle, and with its legs in both walls' slots the clip can turn only about
   1.6 degrees.) The fingers back off 4 mm before rising, so nothing pulls the clip back.
7. Tests the new stem: grips it slowly and lets the grip settle, then lifts it to full-on, aiming 1 mm past, and
   holds there. The aim comes from the lags measured in the simulator: the arm settled 0.5 mm short of its aim,
   and the stem slipped up to 0.5 mm in the grip as a swing began, so it still ends at its stop. Pressing past the
   stop draws the clipped cartridge up, which lifts the clip's legs off their floors and lets the clip creep;
   aiming 1 mm past presses under half as hard as the aim that did that. It swings the stem a quarter turn to the
   full-hot end, holds, and swings it back to the centre, then pushes it down past its lower stop. The arm's
   straight-down grips turn only 90 degrees, so it lets go, turns its open fingers back slowly, and grips again
   for the full-cold end. Then it lets go and parks. The two swings cancel, so the stem ends set as it was.

Every waypoint comes from the layout constants; nothing is read from the simulator.

## How the file works

- **Scene.** The task-file API has boxes, upright cylinders and spheres. The file wraps the framework's
  `yam_arena.scene.task_object` once that module loads. It matches this file's own object specs by value, so other
  tasks' objects are untouched. That lets it add what the API lacks:
  - **The vanity's spawner** also builds:
    - the valve body, as static triangle meshes: the bore's wall with the two inlet ports and the pockets for the
      ball plungers; the clip slots and the cut-outs; and lead-ins at the rim and at the cut-outs' floor;
    - its floor;
    - the holder cup;
    - two ball plungers: 6 mm steel balls on PhysX prismatic joints with spring drives, anchored to the vanity.
  - **Each cartridge's spawner** adds its two tabs and the boss round its stem, as more colliders of its body,
    plus its O-ring bands and inlet windows (looks only).
  - **Each stem's spawner** adds:
    - its notch, piston and port (looks only);
    - a joint to its cartridge. Its frames put the stem's spin on the joint's twist axis, with a damping drive;
      the stem slides up to a stop, and every other axis is locked. The new stem's twist is free and its slide
      6 mm. The old stem's slide stops at 2 mm and its twist at 15 degrees either way; its joint frame is turned
      so its twist reads 0 where it starts (it stands turned 180 degrees on its cartridge, and PhysX twist limits
      must lie within 180 degrees).
  - **Each cartridge's detent ball** (`old_detent_ball`, `new_detent_ball`): a 3 mm steel-grey sphere that is a
    task object of its own, so the framework resets it with the other parts. It starts touching its stem in the
    cartridge's top; the hook adds only its prismatic joint to the cartridge, whose spring drive presses it about
    1.2 N onto the stem. It is set in the side of the boss and stands 1 mm proud of it toward the back tab (2 mm short
    of the tab), inside the boss's height, so the fingers on the body and the clip's legs pass clear of it.
  - **The clip's spawner** adds its legs and grab tab.
- **Never fails open.**
  - **Build record:** the hook records which objects it built. The score is 0 unless every one was built.
  - **First-step check:** on each trial's first step, every part's reported outline must match the shapes built,
    within 0.5 mm, and so must the vanity's, which takes in the valve body and the holder built under it. Each part
    must start where it was put, and each stem and detent ball where its joint holds it on its cartridge. A
    mismatch scores 0 for that trial.
  - **Valid readings:** any non-finite position, orientation, velocity, outline or joint reading, or an orientation
    that isn't a rotation (a quaternion far from unit length), also scores 0 for the trial.
- **Keyed by geometry.**
  - **Seating:** the tabs pass only through the cut-outs, within about 8 degrees of front-back. A cartridge is home
    when its tabs rest on the cut-outs' floor.
  - **Clip:** its legs rest on the slots' floor, 0.5 mm above a seated cartridge's shoulder. A cartridge 0.6 mm or
    more higher blocks them (checked offline by point sampling). "Clipped" needs both the legs on the floors
    (lowest point at most 0.2 mm above) and the cartridge's bottom within 0.5 mm of its seat, so a clip tipped
    over a cartridge that stands proud doesn't count.
  - **What the score reads:** every state comes from PhysX poses and outlines. The rules are fixed thresholds on
    them, plus one fixed modelled number: the handle's quarter-turn swing in the hot-left rule.
- **O-ring drag.** Two balls stand 1 mm proud of the bore, 28 mm above its floor. Each presses about 4.5 N on a
  cartridge passing; with friction about 1.1 between ball and shell, pushing one home or pulling it out takes about
  10 N. The stem's turning drag is a damping of 0.06 N m s/rad on its joint: about 30 mN m at the demo's turn, and
  more against a quick knock. Each cartridge and stem body also has angular damping (10 per second), standing in
  for the same O-ring friction, so a cartridge carried by its stem doesn't keep spinning on it. PhysX applies it to
  each body's whole spin, about every axis, so it also slows a dropped part's rolling.
- **Episode rules.** `score()` runs after every step and reads the end state. Only a new part leaving the table,
  the trial's checks, the tests (each latched once made), the clip's first move, a cartridge rising under its
  clip, and the run of steps at full score carry between steps.
- **New trials.** Task files get no reset call, so the file hooks the embodiment's `reset` (once, with a callback
  per loaded copy of the file). It also starts a new trial whenever the embodiment's episode clock runs backwards.
  Either way each trial starts fresh.
- **Hands.** The grippers' tool points and finger pads come from the embodiment's joint positions and the
  framework's own forward kinematics.

## Checking it without a GPU

The file loads without Isaac Sim, so `task_file.load()` checks its objects, hooks and demo on any machine with
`yam-isaac-lab-arena` installed:

```bash
python -B -I -c "from yam_arena import task_file; t = task_file.load('tasks/replace-faucet-cartridge/replace_faucet_cartridge.py'); print(t.names)"
```

Without Isaac Lab the file builds nothing extra (so it scores 0 there): the valve body, the ball plungers and the
joints are added only when Isaac Lab can be imported. Before submitting, we checked these offline as well:
- the demo, played through the framework's own IK and forward kinematics, with the gripper's and fingers'
  collision meshes tested against every solid;
- that the clip passes only with the cartridge home, that only front-back seats, and that the piston's port meets
  the hot inlet only when set;
- the clip's push: the pushing fingers press the tab's flat face, clear of the spout and in front of the column;
  the push can't tip the clip; it stops short of where the clip's legs began to jump in the simulator; and the
  clip's whole sweep clears the ball plungers;
- the clearances, the drag against the grip and the arm's compliance, and how large the notch and the supplies'
  nuts appear in the cameras;
- the tests: the detent holds a stem lifted; the lifts reach full-on and the pushes reach off despite the arm's
  lag; the after-test's lift ends at the stem's stop yet presses on it under half as hard as the aim that drew
  the cartridge up (both by the lags measured in the simulator); only the free stem reaches the after-test's mark;
  retention's 2 mm limit clears the slots' play and an unclipped cartridge's rise; and a keyed cartridge can turn
  at most 6.7 degrees in its cut-outs, short of the 8 that "keyed" allows.

## Fidelity

All parts are rigid bodies:
- the valve body is a static triangle mesh;
- the cartridges, stems and clip are compounds of primitives at the 1225's approximate size (Moen publishes no
  drawing).

There is no water, so the tests are mechanical: how far a stem lifts and turns, and whether the cartridge stays put
under its clip. Hot and cold follow from where the piston's port faces the inlets.

The stiff old stem is modelled as hard joint limits (2 mm and 15 degrees). A real stiff cartridge is scale or a
worn seal, a drag that gives here and there rather than a clean stop. The detent that holds a stem on or off is a
spring ball; the real 1225 holds its stem by its O-rings' friction. The ball weighs 1 g, more than a real 3 mm steel
ball, to keep the solver steady against the 20 g stem.

The stems' turning drag on their cartridges is modelled two ways: a damping drive on the joint, and angular damping
(10 per second) on each cartridge and stem body. The second is a stand-in: PhysX applies it to a body's whole spin,
about every axis, so it also slows the rolling of a part that is dropped or set down, which real O-rings wouldn't.

O-ring drag is the friction of two spring-loaded balls (about 1.1 between ball and shell), about 10 N in all.
That is the order of a lubricated O-ring fit, not a model of grease. Parker's O-ring handbook works an example of
about 12 N of seal friction for a single dry ring of this size; that is a scaling estimate, not a measurement of
the 1225. A seized cartridge, which needs a puller, is out of scope: the old one is
taken to be a recent install. The drag stays within what the arm's compliant position control can push and pull.

**Solver settings for millimetre parts.** The clip is solved with 32 position and 4 velocity iterations (PhysX's
default for a body is 4 and 1). The valve body's colliders and the clip's legs and grab tab start contacts 0.2 mm out
and rest at zero separation. A pair of them then looks 0.4 mm out, less than the valve body's thinnest wall (the
0.5 mm ledge under the slots) and the clip's 0.5 mm side play in its slots. Without these settings, the 7 g clip,
caught in an overlap while it was pushed home, was flung out of its slots in the simulator at up to 0.65 m/s. Every
part keeps PhysX's unbounded speed for pushing it out of an overlap. Capped at 5 cm/s, a part let whatever pressed
it hard go through it: the fingers closed through a gripped stem and the boss, and a cartridge pulled up hard went
through the clip's legs. The cartridges, their stems and the detent balls keep all of PhysX's defaults. These are
solver settings only: no part's size, mass or friction changes, and no scoring threshold.
