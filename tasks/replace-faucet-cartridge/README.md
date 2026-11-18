# Replace a faucet's cartridge so hot stays on the left

One task file, `replace_faucet_cartridge.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.
A single-handle bathroom faucet stands on a piece of vanity top with its handle off, its hot supply on the left
and cold on the right. Its handle has gone stiff and won't open fully. The robot checks the fault, swaps the worn
1225-type cartridge for a new one the way the manufacturer's sheet says, and tests its work:
1. **Test before.** Lift the old stem: it binds about 2 mm up, short of its 6 mm full-on. Try a turn: it binds too.
   Push it back down.
2. Slide the copper retainer clip out.
3. Pull the old cartridge out by its stem, and put it in the parts tray.
4. Push the new one fully home with its tabs in the valve body's cut-outs.
5. Slide the clip back in. It only goes when the cartridge is home, to within about half a millimetre.
6. Set the stem so that, once the handle is refitted, swinging it to the left opens the hot supply.
7. **Test after,** with the clip home. Lift the new stem to full-on, swing it to the full-hot end and to the
   full-cold end, return it to the centre and push it down (off). The cartridge must stay put under its clip.

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
  The stem turns a full circle, so a stem set wrong gives hot on the right.
- **Rotation stops.** The cartridge has none. Where a trim limits the handle's swing, the stops are in the handle
  or its stop tube (reported for Moen's shower valves, not checked for this faucet).
- **Not checked:** the piston's port layout and the quarter-turn swing are modelled; Moen publishes no drawing. The
  swing is consistent with Moen's bathroom-cartridge guide, which gives a 1225 faucet's top-mount handle 180 degrees
  of rotation in all ([Faucet Cartridges: Bathroom](https://solutions.moen.com/Article_Library/Faucet_Cartridges:_Bathroom),
  read 2026-11-16).

The model has the same parts; the piston, its port and the swing are a model of the internals:
- **Valve body:** an inlet port low down on each side, hot on the left as a user facing the faucet sees it, cold on
  the right, each with a brass supply shank and a red or a blue nut.
- **Cartridge shell:** an inlet window on each side, perpendicular to its tabs. The tabs key it front-back, so its
  windows always face the inlets.
- **Stem:** it turns against its O-rings' drag (a damped joint) and slides up toward on; a spring detent ball holds
  it where it is left. It ends in a brass piston with one port, on the notch's side, level with the inlets at
  full-on (6 mm up).
- **The fault:** the old stem's joint stops it 2 mm up and 15 degrees either way, so the faucet won't open fully, as
  a stem bound by scale would. The new stem lifts its full 6 mm and turns freely.
- **Setting:** with the notch to the front, the port faces the front, between the inlets, and a left swing of the
  handle (a quarter turn, clockwise seen from above) brings it onto the hot inlet. Set with the notch to the back,
  the same swing opens the cold one.

So the score's rule is Moen's: the new stem's notch faces the front within 20 degrees. The code computes it as the
port's angle after the modelled swing against the hot inlet's direction, which is the same rule. No water flows:
the tests are mechanical, how far a stem lifts and turns.

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

**In a real trial** the same rules are read dry with simple tools ("Real trial"). Push the clip back home rather than
pinching it in: in both slots it can turn only about 1.6 degrees, so a clip held at the wrist's angle can wedge.

## Real trial

No water reaches the robot rig: the score is read dry, hot on the left from the notch as in the simulation, and
water is used only at a separate test station.

**The valve.** No Moen faucet made today takes the 1225: the current Chateau L4621 lavatory faucet takes the
nut-held 1255 (Moen parts sheet, Rev. 4/16).

Moen's M-PACT Moentrol tub/shower valve 3520 (1/2 in sweat), sold new, does take the 1225:
- Its parts sheet (Rev. 6/16) lists "Replacement Cartridge 1200 or 1225" and "Retainer Clip 883". The Chateau 4621
  lavatory faucet's sheet (Rev. 1/09) lists the same clip.
- Its spec sheet says "1225 cartridge design".

So the kit uses that valve body:
- It lies on its back on the board, cartridge up, with its shower outlet away from the robot and its tub outlet
  toward it; its hot inlet is then on the left.
- Moen's Moentrol rule, "Rotate the stem of the cartridge (#6) so the small notch is facing down"
  ([1225 Cartridge: Single Handle Shower-Moentrol Valve](https://solutions.moen.com/Article_Library/1225_Cartridge:_Single_Handle_Shower-Moentrol_Valve),
  read 2026-11-16), then means the notch toward the robot: the scene's rule.
- The cartridge, its bore, the cut-outs at 12 and 6 o'clock and the copper clip are the real parts. The valve has a
  tub outlet where the scene has a spout.
- Clip side: Moen's Moentrol article says to "Lift the tab at the top of the U-shaped" clip; with the valve on its
  back that tab points away from the robot. If the 3520's clip won't go in from the robot's side, start with its tab
  at the back: the score accepts the clip from either side ("Either way round", above).

A pre-2009 Moen lavatory faucet that takes the 1225 (Chateau 4621 or 4601, parts sheets Rev. 1/09 and 2/03) may be
used instead, raised on blocks for its shanks.

**Kit.** Prices were read on 2026-11-16; "listing" means a retailer's search listing. The total is about $450,
estimates included, with no item over $130 (a plumber sweating the stubs saves the torch kit):
- **Valve:** Moen 3520 ($126.39, Home Depot listing), with:
  - two 3 in stubs of 1/2 in copper tube, sweated into the inlets once, by a plumber or with a torch kit (about $45,
    estimate);
  - two 1/2 in pipe straps holding the stubs to the board;
  - the board: the 22 x 20 cm, 12 mm white solid-surface or plywood vanity top, stuck to the rig's table with
    heavy-duty double-sided tape (about $10, estimate). A 50 N pull up on the valve must not lift it.
- **Parts:**
  - three genuine Moen 1225 ($26.49 each, Zoro listing): new, made stiff, spare;
  - two spare 96365 clips ($9.80 each, SupplyHouse listing);
  - Moen 99915 silicone grease;
  - a cup about 32 mm across and 20 mm deep;
  - a tray 11 cm square inside with 3 cm walls;
  - a lever handle about 80 mm long, as the scene's reference.
- **Tools:** a 50 kg luggage scale ($9.99, Walmart listing), a 6 in digital caliper (Harbor Freight 63586,
  $11.99), an 8 in mill file and 320 grit emery cloth ($5.99 each, listings), cloth gaffer tape ($28.50, AndyMark
  listing), a fine paint marker ($2.53, Staples listing), and a phone on a tripod (about $15, estimate).
- **Test station** (about $70, estimate): a 12 qt dish pan ($8.18, Walmart listing), a bucket, two funnels, two
  2 ft lengths of 5/8 in ID vinyl tube with hose clamps, an instant-read thermometer (ThermoPro TP03BW, $13.99,
  Target listing), a kettle ($14.84, Walmart listing), cups and towels.

**The stiff old cartridge** is made once, in about 15 minutes, and reused. For a 1225 that is hard to operate,
Moen's FAQs give two remedies:
- "lubricate it both inside and out with a silicone-based grease (Moen part number 99915)"
  ([Handle is difficult to operate](https://solutions.moen.com/FAQ's/FAQ's-Troubleshooting/General/Handle_is_difficult_to_operate));
- "For one-handle faucets with a 1225 cartridge, replace the cartridge"
  ([My kitchen faucet handle is hard to operate](https://solutions.moen.com/FAQ's/FAQ's-Kitchen_And_Bar/Maintenance/My_kitchen_faucet_handle_is_hard_to_operate.__What_can_I_do_to_correct_this%3F)).

Both were read 2026-11-16. This task is the replacement.

The fault is made as the reverse of the first remedy:
1. Wash a new 1225 in warm soapy water, working the stem up to its stop and down, and a full turn each way, 20
   times. Rinse it the same way and dry it for 10 minutes.
2. Grease its body's outer O-rings again, so it slides like a new one.
3. Dot its body below the tabs with paint.

It reproduces the symptom, a stem that takes more force to move. The scene's cause, scale, can't be prepared
repeatably.

**Check it at every reset** (about 3 minutes, with the cartridge home). Hook the luggage scale by a cord under a
small hose clamp tightened on the stem over a wrap of tape:
- **Pull-out:** before the clip goes in, raise the scale slowly, straight up, past the stem's stop until the
  cartridge starts to rise. Note the reading, push the cartridge home, and slide the clip in.
- **Lift:** with the clip in, note the highest reading before the stem moves.
- **Turn:** tape the cord's end to the stem, wrap it twice round, and pull level and square to the stem. Note the
  highest reading before it turns; times the 4 mm radius, that is the torque.
- **Reference:** measure the new cartridge once, the same way.
- **Accept** the old one when its lift and turn readings are at least twice the new one's, its lift and pull-out
  readings are at most 30 N each, its turn reading is at most 30 N (our estimate of what the gripper can apply on the
  8 mm stem, not a measurement), and its stem still lifts to its stop and turns fully by hand. If it fails, wash it
  again or prepare the spare.

**Clip and pads.** File every edge of the clip's legs, tips and tab at about 45 degrees until slightly rounded, and
finish with emery cloth. A cotton swab drawn along each edge must not catch; repeat the check every 10 trials. Cover
each finger pad with one layer of gaffer tape from the first trial. Replace a bent clip: Moen says the water can't
be turned back on until a damaged clip is replaced.

**Scoring with simple tools.** With each stem down, paint a thin ring on it level with its cartridge's top, and
measure each stem's full lift once with the calipers. Film each trial with the phone at rim height, about 30 cm
away, framing about 15 cm. After the trial, read the score:
- **From the video:**
  - a new part leaving the table;
  - the clip out of the way before the old cartridge rose;
  - hands open, clear by 1 cm and still for 1 s;
  - tested before: the old ring rose before the clip moved;
  - tested after: with the clip home, the new ring rose to its full lift and, so lifted, the notch swung a quarter
    turn each way, each within 10 degrees, read from video taken above the valve.
- **Home:** the calipers' depth rod, from the rim to each tab, reads under 1 mm (Moen: tabs "flush with the valve
  body").
- **Clipped:** both legs run through both slots and lie flat, and the tabs stay flush.
- **Hot on the left:** in a photo from above, against a printed protractor, the notch faces the robot within 20
  degrees.
- **Left off, and the old cartridge in the tray:** the new ring back level; the photo.

**The water check, at a separate test station, never at the rig.** It checks Moen's notch rule with water. Run it
before the first trial, and after any trial whose reading is in doubt.
1. Lay the board in the dish pan, away from the rig. Clamp a tube onto each inlet stub and hold a funnel in each,
   about 40 cm up. The left inlet, seen from the robot's side, is hot.
2. At the station, after the trial, refit the handle, turn the water on (pliers on the stem do if the handle
   doesn't fit):
   - pour warm water (40 to 45 degrees C, kettle and tap water, checked with the thermometer) into the left funnel;
   - pour cold tap water into the right one;
   - keep both funnels full, because a pressure-balancing valve can shut one side when the other runs dry.
3. Pull the stem fully open. Catch about 100 ml at full left and at full right: it passes when the left cup is at
   least 10 degrees C warmer.
4. Drain everything, pull the cartridge, and towel the bore, the cartridge and the board dry before the board goes
   back.

**Reset:** the old cartridge home with its notch toward the robot, stem down and checked; the clip in; the new
cartridge in its holder with its notch to the left, reused after its O-rings are checked.

## Why this task

**Core plumbing repair.** Replacing the cartridge is the standard fix for a dripping single-handle faucet (see the
repair guides from [Lowe's](https://www.lowes.com/n/how-to/repair-leaky-single-handle-faucet) and
[This Old House](https://www.thisoldhouse.com/plumbing/how-to-repair-a-dripping-single-handled-faucet)). It is also
the fix for a stiff one. Moen's FAQ
[Handle is difficult to operate](https://solutions.moen.com/FAQ's/FAQ's-Troubleshooting/General/Handle_is_difficult_to_operate)
gives the cause as "sediment or debris in the cartridge, mineral buildup or a damaged seal within the cartridge":
"The cartridge will need to be serviced or replaced". For a 1225, that page suggests grease,
and Moen's kitchen FAQ says "replace the cartridge" (both read 2026-11-16; see "Real trial"). The workers who do it
(read 2026-11-16 unless noted):
- **General maintenance and repair workers** hold 1,529,700 US jobs (BLS OEWS, May 2025).
  - O*NET 49-9071.00 gives them the Core task (importance 69) "Assemble, install, or repair wiring, electrical or
    electronic components, pipe systems, plumbing, machinery, or equipment".
  - The BLS handbook says they "work on plumbing" and "begin by doing simple tasks, such as fixing leaky faucets"
    ([General Maintenance and Repair Workers](https://www.bls.gov/ooh/installation-maintenance-and-repair/general-maintenance-and-repair-workers.htm),
    modified 2026-10-29).
- **Plumbers, pipefitters and steamfitters** (465,840): O*NET 47-2152.00's Core task "Maintain or repair plumbing
  by replacing defective washers, replacing or mending broken pipes, or opening clogged drains".
- **Their helpers** (44,330): O*NET 47-3015.00's Core task "Perform rough-ins, repair and replace fixtures and water
  heaters, and locate, repair, or remove leaking or broken pipes".

49-9071 alone passes 500,000. No official source says how often each worker does this job.

**Test before and after.** A tradesperson confirms the fault before taking the faucet apart, and checks the work
before leaving; Moen's last step covers hot and cold being "reversed". Without water in the simulation, the robot
checks mechanically: the stem lifts to full-on and swings to both ends, and the cartridge stays put under its
clip.

**The difficulty is in the hands,** in a fixed order, as the task.json context lists step by step.

**The stem's alignment.** The setting is a physical alignment the robot must achieve: notch to the front, as the
old stem stood when it came out. In the model, that alignment puts the piston's port where the handle's left swing
opens the hot inlet. Hot on the left is code. Chicago's plumbing code, which adopts the IPC's 607.4, requires it
([18-29-607.4](https://codelibrary.amlegal.com/codes/chicago/latest/chicago_il/0-0-0-2691727)); the ICC's own text
wasn't checked.

**No artificial constraints.** The instruction states the goal only: any arm or both may be used, in any order
that works. All geometry is built by the task file; there are no third-party assets.

**Moen's own procedure**
([1225 Cartridge: Single Handle Bathroom Faucets (Prior to 2009)](https://solutions.moen.com/Article_Library/1225_Cartridge:_Single_Handle_Bathroom_Faucets_(Prior_to_2009)),
read 2026-11-16):
1. Slide the copper clip out.
2. Turn the cartridge a quarter turn with the plastic tool from the 1225 kit, "to help loosen" it.
3. Raise the stem until it stops and pull the cartridge out by it with pliers ("This may require some effort").
4. Push the new one completely in, tabs flush and centred on the cut-outs at 12 and 6 o'clock.
5. Slide the clip back on top. If it resists, the cartridge "may not be properly aligned".
6. Turn the stem so the small notch faces forward, or 180 degrees round if hot and cold are reversed.

The task leaves out the loosening turn, which frees a cartridge stuck in its body. This one is stiff in its stem,
and its body's O-rings drag like a new one's (about 10 N). In the model, too, a keyed cartridge turns at most 6.7
degrees in its cut-outs. The robot pulls it out by its stem, as Moen does with pliers.

**Not a variant of the board's sealant-gun reload.** That task restores a gun to working order. Here the work is
drag, keying, a retainer that proves the cartridge is home, a precise rotation, and a function test of the valve.

## Fidelity

All parts are rigid bodies. The valve body is a static triangle mesh. The cartridges, stems and clip are
compounds of primitives at the 1225's approximate size; Moen publishes no drawing. There is no water, so the tests
are mechanical, and hot and cold follow from where the piston's port faces the inlets.

**The valve body is a model.** It is a column with side inlets, red and blue nuts and a spout. The real trial uses
a Moen Moentrol valve body lying on the board ("Real trial"). Its cartridge, bore, cut-outs and clip are the real
parts. Its inlets, tub outlet and pressure-balancing spool are not modelled.

**The stiff old stem** is modelled as hard joint limits (2 mm and 15 degrees), a stem bound by scale, so the faucet
won't open fully. A real stiff cartridge is a drag that gives here and there rather than a clean stop. The real
trial's old cartridge reproduces that drag: a genuine 1225 with the grease washed out of its stem, needing at least
twice the new one's force. It drags where the model stops, so a real robot may lift it further than 2 mm; the
before-test (a lift of at least 1.5 mm) reads the same either way. In the real trial the old stem does reach its
stop under enough force, so "won't open fully" holds strictly only in the simulation; there it means the stem won't
open fully under the force that opens a new cartridge.

**The detent and the drag.** The detent that holds a stem on or off is a spring ball; the real 1225 holds its stem
by its O-rings' friction. The ball weighs 1 g, more than a real 3 mm steel ball, to keep the solver steady against
the 20 g stem. The stems' turning drag is a damping drive on the joint, plus angular damping (10 per second) on
each cartridge and stem body. The second is a stand-in: PhysX applies it to a body's whole spin, about every axis,
so it also slows the rolling of a dropped part.

**O-ring drag** is the friction of two spring-loaded balls (about 1.1 between ball and shell), about 10 N in all:
the order of a lubricated O-ring fit. Parker's O-ring handbook works an example of about 12 N for a single dry ring
of this size, a scaling estimate, not a measurement of the 1225. A seized cartridge, which needs a puller, is out of
scope.

**Solver settings for millimetre parts.**
- The clip is solved with 32 position and 4 velocity iterations (PhysX's default is 4 and 1).
- The valve body's colliders and the clip's legs and tab start contacts 0.2 mm out and rest at zero separation. A
  pair of them then looks 0.4 mm out, less than the 0.5 mm ledge under the slots and the clip's 0.5 mm side play.
  Without this, the 7 g clip, caught in an overlap while it was pushed home, was flung out of its slots at up to
  0.65 m/s.
- Every part keeps PhysX's unbounded speed for pushing it out of an overlap. Capped at 5 cm/s, a part let whatever
  pressed it hard go through it: the fingers closed through a gripped stem and the boss, and a cartridge pulled up
  hard went through the clip's legs.
- The cartridges, their stems and the detent balls keep all of PhysX's defaults.

These are solver settings only: no size, mass, friction or scoring threshold changes.

## What the robot sees

The instruction is a work order. The scene shows the rest: the clip's tab, the tabs in the cut-outs, the supplies'
red and blue nuts, and the stems' dark notches. The old stem's notch faces the front, the alignment the new stem
must be given. The order follows from the parts:
- the clip comes out before the cartridge can;
- the new cartridge goes fully home before the clip goes back;
- the fault is checked before the clip comes out, and the work after it goes back.

At 320 x 176, from a wrist camera 6 to 10 cm above the stem, the notch is 4 to 5 pixels long and each nut 8 to 10
pixels across. From the top camera the notch is under a pixel, so aligning the stem needs a close view.

## Runtime and seeds

- **Run limit:** 2,400 control steps at 20 Hz (2 minutes). The scripted demo parks at 104.3 s and its plan is
  106.8 s long in all.
- **Results:** three trials run back to back in the simulator each scored 1.0 and ended as a success at step 2,069
  (103.45 s). The Dream-1 trial scored 0.0 and ran all 2,400 steps.
- **Wall clock:** about 35 to 38 minutes a trial on one shared GPU.
- **Seeds:** none needed; the layout is fixed (`-E object_jitter=0`).

## The scripted demo

The faucet stands in front of the right arm, whose straight-down grips reach high enough to lift a 10 cm
cartridge clear of the rim. So the demo uses the right arm and parks the left; a policy may use either arm or both.
In order, the right arm:
1. Tests the old stem: it lifts it, aiming 12 mm up (it binds at 2), tries a 30 degree turn (it binds at 15), turns
   it back, pushes it down past its lower stop, and lets go.
2. Pinches the clip's tab and slides the clip 31 mm out, onto the spout. The legs' tips clear the cartridge, still
   in their slots.
3. Grips the old stem, raises it to its stop, pulls the cartridge out and drops it in the parts tray.
4. Lifts the new cartridge out of its holder by its body, below the tabs, so the tabs keep their bearing; the free
   stem couldn't hold them. It lowers it until it hangs 3 mm over the rim, lets it hang still, and eases it into
   the bore at about 8 mm/s, then on until its bottom is 39 mm in, and lets go. There the bore's 0.5 mm play holds
   it within about 1.5 degrees of upright. The closed fingers then push it home by the stem's top, straight down,
   aiming 32 mm past home for the arm's give against the drag. Earlier versions gripped the 5 mm boss, and the
   cartridge hung up to 4.1 degrees off vertical and once jammed.
5. Turns the stem a quarter turn anticlockwise, from the left to the front.
6. Pushes the clip back home with its closed fingers on the tab's front face, 3 mm above the clip's bar. It slows
   for the last 8 mm and stops with the legs' tips 16 mm behind the bore's axis, inside the back wall's outer face,
   where the legs' ends caught and the clip hopped. The legs still reach 4.5 mm past what "clipped" needs. Nothing
   holds the clip, so its slots set its angle (in both slots it can turn only about 1.6 degrees). The fingers back
   off 4 mm before rising.
7. Tests the new stem: it grips it, lifts it to full-on, aiming 1 mm past, and holds there. Measured lags set that
   aim: the arm settled 0.5 mm short of its aim, and the stem slipped up to 0.5 mm in the grip as a swing began.
   Pressing harder past the stop drew the clipped cartridge up. It swings the stem a quarter turn to the full-hot
   end and back, pushes it down past its lower stop, regrips (its straight-down grips turn only 90 degrees) for the
   full-cold end, then lets go and parks. The two swings cancel, so the stem ends set as it was.

Every waypoint comes from the layout constants; nothing is read from the simulator.

## How the file works

- **Scene.** The task-file API has boxes, upright cylinders and spheres. The file wraps the framework's
  `yam_arena.scene.task_object`, matching its own object specs by value, to add what the API lacks:
  - the valve body as static triangle meshes (bore, inlet ports, clip slots, cut-outs, lead-ins), its floor and the
    holder cup;
  - two ball plungers on spring-driven prismatic joints for the O-ring drag;
  - the cartridges' tabs and bosses;
  - the stems' joints: twist with a damping drive, and a slide to a stop. The old stem stops at 2 mm and
    15 degrees; the new one slides 6 mm and turns freely;
  - each cartridge's 3 mm detent ball, its own task object, sprung about 1.2 N onto the stem;
  - the clip's legs and tab.
- **Never fails open.** The score is 0 unless every hook-built object was built, every part's outline matches its
  shapes within 0.5 mm and starts where it was put on the first step, and every reading is finite and a valid
  rotation.
- **What the score reads:** PhysX poses, outlines and joint states. The rules are fixed thresholds on them, plus
  one fixed modelled number: the handle's quarter-turn swing in the hot-left rule.
- **Episode rules.** `score()` runs after every step. Only a part leaving the table, the checks, the latched tests,
  the clip's first move, a cartridge rising under its clip, and the run at full score carry between steps.
- **New trials.** Task files get no reset call, so the file hooks the embodiment's `reset`. It also starts a new
  trial whenever the embodiment's episode clock runs backwards.
- **Hands.** The grippers' tool points and pads come from the embodiment's joint positions and the framework's
  forward kinematics.

## Checking it without a GPU

The file loads without Isaac Sim:

```bash
python -B -I -c "from yam_arena import task_file; t = task_file.load('tasks/replace-faucet-cartridge/replace_faucet_cartridge.py'); print(t.names)"
```

Without Isaac Lab it builds nothing extra, so it scores 0 there. These were checked offline before submitting:
- the demo through the framework's IK and forward kinematics, with collision checks;
- the keying, the clip's pass only at home, and the hot-left geometry;
- the clip's push and the clearances;
- the tests' margins against the arm's measured lag.
