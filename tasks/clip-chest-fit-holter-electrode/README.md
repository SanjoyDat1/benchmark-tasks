# Clip the hair off a Holter electrode site on a manikin's chest with a surgical clipper, then stick the electrode on

One task file, `clip_chest_fit_holter_electrode.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.

A CPR training manikin lies on its back on the bench, its head to the robot's left. A square of dark faux fur over
its breastbone stands in for chest hair. Four ECG electrodes are already on bare skin round it, each with a lead of a
5-lead snap cable snapped on. The last electrode, with the cable's fifth lead already snapped onto it (as the
monitor makers say to do before it goes on), lies on its liner card on the bench; a powered surgical clipper,
switched off, lies on a tray. The robot has to clip the hair off the site in the middle of the fur, put the clipper
back switched off, peel the electrode off its card, put it on the bare patch and press it on.

**The physical ability.** Keeping a powered clipper's blade seated on a curved surface while pulling it teeth first
through the pile: the blade cuts only when its edge is down on the skin, the sole close to flat, the motor running and
the stroke going teeth first at 2-60 mm/s, and a hand that lifts, rakes or skids leaves hair; covering the site and not much
more; then placing an adhesive part that holds only on the cleared patch, by a small grip on its lead's snap
connector, and pressing it on. That is **precise control of force and position with a powered tool on a curved body,
then of a light part into place**. It is not one of the covered abilities: nothing is held still while the other
hand turns or tightens, nothing is pushed into a snap-fit or detent until it clicks (the snap connector is on from
the start; the clipper's push button is only pressed), and
nothing is picked and stood or stacked. The electrode placement alone would be a plain pick-and-place; here it can
only succeed on the patch the clipping made.

## Requirements

- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`
  (Isaac Lab-Arena `release/0.3.1`; `inspect-robots` and `inspect-robots-yam` as it installs them);
  `source "$ARENA_DIR/yam.env"` in the shell.
- Nothing to download: the task file builds its geometry from boxes, cylinders, a sphere, convex slices and
  triangle meshes written in code.
- `-E control_hz=20`: the scorer counts its 1 s hold as 20 control steps, and its cutting and pressing rules use
  the 20 Hz step.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/clip-chest-fit-holter-electrode/clip_chest_fit_holter_electrode.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clip the hair off the electrode site in the middle of the fur patch on the training manikin's chest with the clipper, no more than needed, then stick the electrode from the liner card onto the bare patch there. Leave the clipper switched off on its tray and the liner card off the manikin." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/holter-dream
```

To run the scripted demo, use the same command with `--policy yam_scripted` and `--log-dir ~/yam-eval/holter-demo`.

## The job, and who does it

- **The procedure.** Holter and electrode makers put hair removal first. Welch Allyn H3+ Holter manual
  (9515-165-60-ENG Rev C, p. 22): "Remove any hair from the electrode sites using a razor or shaver.", then
  "Attach the patient cable lead wires to the electrodes before applying them to the patient." and "Secure each
  electrode by exerting slight pressure around the outer edge and inner ring". 3M Red Dot application instructions
  (70-2010-7348-6): "Excessive hair at the placement site should be removed by clipping." and "connect the snap
  style leadwire to the electrode prior to placing on the patient", then "Firmly rub your finger around the outside
  of the front label". A hospital standard (London Health Sciences Centre, continuous ECG monitoring, updated
  2020-02-01): "Trim hair." "Do not shave the skin". So this task clips (3M's and the hospital's method) and follows
  the H3+ order: lead on first, then the electrode pressed on near its edges, not on the stud. The site is the H3+ diagram's mid-sternum
  position. (All read 2026-11-22.)
- **Occupations** (O*NET task statements, pages marked "Updated 2026"; BLS OEWS May 2025 national employment, read
  2026-11-22 through the BLS public API):

  | SOC | Occupation | O*NET task | OEWS May 2025 |
  |---|---|---|---|
  | 31-9092 | Medical assistants | "Operate x-ray, electrocardiogram (EKG), or other equipment to administer routine diagnostic tests." | 817,870 |
  | 29-2031 | Cardiovascular technologists and technicians | "Set up 24-hour Holter and event monitors, scan and interpret tapes, and report results to physicians." and "Attach electrodes to the patients' chests, arms, and legs, connect electrodes to leads from the electrocardiogram (EKG) machine, and operate the EKG machine to obtain a reading." | 62,960 |
  | 31-1131 | Nursing assistants | "Wash, groom, shave, or drape patients to prepare them for surgery, treatment, or examination." | 1,448,910 |

  **Who does which part, plainly.** Putting electrodes on a patient's chest is part of running an EKG: the BLS
  Occupational Outlook Handbook says "EKG machines monitor the heart's performance through electrodes attached to a
  patient's chest, arms, and legs." Medical assistants' task names operating the EKG, not the electrodes; the
  cardiovascular technicians' tasks name both the Holter set-up and the electrodes. Clipping the site is the
  makers' first step for an electrode on a hairy chest, so whoever applies the electrode does it; nursing assistants'
  task names shaving patients to prepare them for an examination. No source was found that says how often any of
  these occupations clips a chest before an electrode goes on, and none is claimed. Medical assistants alone
  (817,870) are over 500,000; with cardiovascular technicians, 880,830.
- **How often.** No published national count of Holter or ambulatory ECG tests was found. Medicare fee-for-service
  alone paid for about 916,000 ambulatory ECG services in 2024 (our sum of the national rows for codes 93224,
  93227, 93241, 93244, 93245 and 93248 in CMS's "Medicare Physician & Other Practitioners - by Geography and
  Service"; a floor: Medicare Advantage, private insurance and Medicaid are not in it).

## The closest tasks on the board, and what differs

- **#26 "Place ECG leads on the correct electrodes"** (accepted): snapping leads onto electrodes already on a
  manikin. The same setting, a different ability: here the lead is already on (the makers' order), and the work is
  clearing the skin with a powered clipper and placing the electrode so it holds.
- **#3417 "Place six EEG cup electrodes on a mannequin scalp"** (pending, category Other): placing electrodes on a
  manikin, with no hair removal and no powered tool.
- **#2275 "Shave a care manikin's left cheek and jaw with an electric shaver"**
  (not accepted, final): powered hair removal on a manikin. Its final reasons were "Real-world trials would be too
  difficult to reproduce consistently as specified" and the importance bar; its earlier reviews asked for drawings
  of a custom sprung cheek and an instrumented shaver, and objected that its UV-gel coverage check could not tie
  each square to the force and the cheek's stretch when it was wiped. What this task does instead:
  - **Nothing custom or instrumented.** The clipper, manikin, electrodes, cable and fur are bought as they are; the
    fur is taped on. There is no force sensor and no tracer: the real trial's only force check is a 1 N lift on the
    snap connector with a luggage scale after the trial.
  - **The outcome is visible after the trial.** Clipped pile stays clipped, so coverage is a photo of the patch with
    a template and a ruler; the electrode either holds a 1 N lift on its snap or it doesn't; peeled off, its gel
    shows trapped pile or not. Apart from the hazard (seen on the video), no check has to be matched to the instant
    it happened.
  - **Importance** rests on electrode placement (medical assistants' EKG task, over 500,000 on its own), with the
    clipping as that work's own first step on a hairy chest, not on grooming.
- **#2069 "Taper the back of a training head with a clipper over a comb"** (not accepted, revisable): its review
  said "Twelve 6 mm locks separated by substantial gaps do not reproduce cutting and blending a continuously haired
  head" and that scoring "only the twelve locks can award full credit while hair between them remains untapered".
  Here the hair is continuous: 4,800 cells of 2 mm cover the whole 160 x 120 square, a cell is cut only under the
  seated, moving edge, and the site's every cell is scored (95% of it must be cut).

## Setup and objects

Millimetres; x away from the robot (from the line through the arms' bases), y to its left, z up from the bench.
"Along the body" is y (toward the head, +y); "across" is x.

| Object | Make, size, material | Place |
|---|---|---|
| Manikin | Prestan Professional Adult CPR-AED training manikin, PP-AM-100M-MS (torso with head; 61 x 35.6 x 17.8 cm, 7 lb: listing values, unverified) | On its back, head to the robot's left, body along y; the electrode site (mid-sternum) at (310, -50), the skin there 150 above the bench (**estimate**: measure the real unit) |
| Hair | Shannon Fabrics Luxe Cuddle, 10 mm pile, a dark solid colour: a 160 (along the body) x 120 square cut from the bolt, pile brushed toward the hips | Double-sided tape over the sternum, its centre on the site |
| Placed electrodes | Four 3M Red Dot 2560 (foam, 40 x 35), each with a lead of a 5-lead snap cable (4 mm wire) snapped on | On bare skin outside the fur, centred at (230, 50), (390, 50), (215, -220), (405, -220); each lead runs to the nearer side and onto the bench. Positions are illustrative (not from a diagram) |
| Clipper | 3M Surgical Clipper Professional 9681 with a 9680 blade: about 180 x 60 in plan, blade 36.4 wide, fixed head; switched off | Lying flat, button up, teeth toward the robot, on the tray; the middle of its cutting edge at (180, -470) |
| Tray | Flat plastic instrument tray, 240 x 120 x 4 | Centred at (270, -470), beyond the hips |
| Electrode to place | One more 3M Red Dot 2560, the cable's fifth lead's snap connector already on its stud, on a 70 x 48 card of its liner (cut round one electrode; how the 2560 comes on its liner was not found) | The card at (100, -20), long side along y; the electrode 10 toward +y from the card's centre; the card's bare tab at its -y end |
| Cameras | Two phones on tripods, for scoring: one above the chest, one from the robot's side | |

The start is the real starting state: the clipper off on its tray, the fur uncut, the electrode on its card with its
lead on. The end is the complete outcome: the site clear, the electrode stuck on it, the clipper switched off on its
tray, the card on the bench, both hands clear.

## Score

| Score | When |
|---|---|
| 1.0 | The 0.75 state with a tidy finish (the clipper switched off and lying on its tray, the liner card off the manikin, at most 70 cm2 clipped), everything still and both hands clear, held for 1 s |
| 0.75 | The electrode stuck on bare skin at the site |
| 0.5 | The site clipped, with a tidy finish |
| 0.25 | The site clipped |
| 0 | Anything else; and the whole trial if the running clipper touches one of the four electrodes already on the chest, or its snap or lead |

An untidy finish costs one step, not the trial: a stuck electrode with the clipper left running, left off its tray,
the card left on the manikin or more than 70 cm2 clipped scores 0.75, and a clipped site with no electrode scores
0.25 instead of 0.5. Running the clipper in the air costs nothing. A trial's score is its last step's score (the
1.0 state ends the trial once it has held).

- **The site:** an outline the electrode's size (40 x 35 with 6 mm corners, the long side along the body) centred
  on the site's mark, the centre of the fur square. **Clipped:** at least 95% of that outline over cut pile.
- **Stuck on bare skin at the site:** the electrode, off its liner and gel side down (its back within 30 degrees of
  the skin's normal), was pressed onto the chest with at least 3 N for 1 s in a row (anywhere on it), and has not
  been pulled off since (moved 2 mm from where it stuck); when it stuck, at least 95% of its own outline lay over cut
  pile or over skin outside the fur; and its centre (the stud) is within 15 mm of the site's mark.
- **Clipped area:** the cut pile's area, at most 70 cm2: about five times the electrode's outline (13.7 cm2), a patch of
  about 8 x 9 cm. An estimate of "the site and not much more": no source gives an area. (The demo clips about 45 cm2.)
- **Switched off:** the motor off. **On its tray:** seen from above, every corner of the clipper's body and of its
  blade's sole inside the tray's edges, and the lowest of those corners within 10 mm of the tray's top. **The card
  off the manikin:** no corner of the liner card more than 6 mm above the bench top (on the bench, or below it; on
  the manikin it is not).
- **Still:** the clipper, its button, the electrode and the card each under 10 mm/s and 0.5 rad/s. **Hands clear:**
  points over both grippers' finger pads (at their tips, middles and backs, on both edges and the middle), and the
  tool point between the fingers, all at least 30 mm from the box that bounds each of the clipper, the electrode
  and the card in its own frame.
- **The hazard:** the clipper touching one of the four electrodes already on the chest, or its snap or lead (a
  contact force of 0.05 N or more), while its motor runs. A running blade can cut a lead wire or lift a placed
  electrode. The rest of the trial scores 0. (The electrode being placed is not part of this rule.)
- **Guards:** a trial is void (0) if any part was not built, if the parts' state cannot be read after the first
  three steps or a part's pose or velocity is not finite, if a contact reading fails or is not finite or a write to
  the simulation fails (the hazard could then not be checked), if a new trial's parts could not be set back to the start (or are not within 5 mm of where the first trial's
  were, at its third step), or if at any step the arms' state
  cannot be read (the hands' clearance could then not be checked); unread state is never treated as safe.
- **Real trial:**
  - *Site clipped:* a clear 40 x 35 template with a 1 mm grid, centred by ruler on the fur square's centre (80 mm
    from its ends, 60 mm from its sides), photographed from above: uncut pile inside the outline covers no more
    than 5% of it (about 70 mm2, a tuft of about 8 x 9 mm).
  - *Clipped area:* a 1 cm grid transparency laid on the patch, photographed from above; count the clipped squares.
  - *Stuck:* lift the electrode's snap connector straight up with a luggage scale to 1 N (or hang a 100 g weight
    from it) for 1 s: it stays on (the simulated hold on bare skin is 2.5 N, and at least 1.8 N with 95% of it bare).
    *At the site:* a ruler from the fur square's edges to the stud. *Bare under it:* afterwards peel it off and
    photograph its gel side against the template: trapped pile under no more than 5% of it.
  - *Off and on its tray, the card on the bench, still, hands clear:* the overhead photo and the trial video (the
    motor is audible; press the button once after the trial: if it starts, it was off). A ruler gives the hands'
    30 mm.
  - *The hazard:* the trial video shows the running clipper touching a placed electrode or a lead.

## How the file works, and its fidelity

Every part is a rigid PhysX body at its real size (the manikin's chest shape is estimated); the manikin, the
placed electrodes and their leads are fixed
(kinematic) bodies. Hair and adhesion cannot be simulated in PhysX, so the fur is a field of cells and the
electrode's stick is a joint, each changed only by rules that need the real motion:
- **The fur:** 80 x 60 cells of 2 mm over the 160 x 120 square, drawn as 14,400 strands 10 mm tall leaning toward
  the hips. A cell is cut (to 0.4 mm stubble, never back) only in a control step in which: the motor is on; the
  clipper presses on the chest with at least 0.3 N (a PhysX contact reading); the cutting edge is within 1.5 mm of
  the skin at this step and the last (only the part of the edge that is within 1.5 mm cuts); the blade's sole is
  within 15 degrees of the skin (steeper is raking: "Do not rake"); the edge moves teeth first (within 60 degrees of
  the teeth's direction) at no more than 60 mm/s and at least 0.1 mm in the step; and the cell's centre lies in the
  area the seated edge swept. A pass held 5 mm up, pushed backward or sideways, run too fast or with the motor off
  cuts nothing (each checked in the simulation with test drives before the demo was written). The nap is drawn but not used: 3M's guide allows pushing
  against the growth or pulling toward you.
- **Feed drag:** while it cuts, a force on the clipper opposite its horizontal motion, 0.04 N per cm2/s of hair cut, at most
  1 N (about 0.4 N for a full-width pass at 27 mm/s): the motor cuts; the hand only guides.
- **The button:** a 3 g cap on a slide (1.5 mm travel, 1 N preload, 4 N to bottom it). Pushed past 70% of its
  travel it switches the motor on or off; it re-arms below 30%.
- **Sticking:** the electrode is held to its liner card by a joint whose three linear springs are capped at 0.4 N
  (and its three angular ones at 0.4 N x 10 mm, a peel from one edge); it lets
  go once the electrode has moved 2 mm from the card, so a free card comes up with it and a held one stays. On the
  chest, after 3 N for 1 s, a joint at the electrode's place holds it with springs capped at 2.5 N times the bare
  share of its outline to the sixth power (2.5 N bare, 0.66 N at 80%, 0.04 N at 50%: pile under the gel stops it
  sticking); it breaks once the electrode has moved 2 mm from where it stuck.
- **Gel side down:** the electrode sticks only with its back within 30 degrees of the skin's normal.
- **The manikin's skin** collides through 42 solid convex slices along the body (a thin shell let a pressed
  clipper sink through); the smooth skin is drawn over them.
- **Not simulated:** hair as strands, the clipped pile's debris, the gel, the fifth lead's wire (the four placed
  leads are fixed rigid 4 mm rods; the fifth lead ends at its snap connector), the manikin's soft skin and its chest
  compression, the clipper's vibration and sound.
- **Untested in the real world:** that a surgical clipper cuts this faux fur down to its backing, and that an
  electrode sticks to the clipped knit backing. A fursuit guide reports trimming faux fur with pet clippers
  (unverified); no source shows a surgical clipper on faux fur. If the backing does not hold an electrode, the
  real-trial check "stuck" fails for every trial: test both before running trials.

## Sources and estimates

| Value | Used | Source / status |
|---|---|---|
| Clipper | 3M 9681 with blade 9680: about 18 x 6 cm in plan, blade 36.4 mm, fixed head, $120.16 | CME Corp listing, read 2026-11-22; 3M how-to guide (©2014): "like a pencil", "blade flat or at a slight angle", "Do not rake" |
| Clipper thickness, mass, button | 35 mm, 200 g, one push-on/push-off button on the top face, 115 mm behind the edge, 1.5 mm travel, 4 N | **Estimates** (not found; the guide's "button is in OFF position" suggests a latching button) |
| Blade head | Sole 12 mm, set 25 degrees to the body | **Estimate** (the head is fixed; a pencil hold with the blade flat needs it angled) |
| Stubble | 0.4 mm | **Estimate** (0.2-0.5 mm range from distributor listings, unverified; 3M's value not found) |
| Cut rule | 0.3 N seat, 1.5 mm, 15 degrees, 60 mm/s | **Estimates** (no clipping speed or force is published; "Do not use excessive pressure") |
| Electrode | 3M Red Dot 2560, foam, 4 x 3.5 cm | Solventum product page, read 2026-11-22 |
| Electrode thickness, stud, snap connector, liner card | 1.2 mm; 4 mm stud; 16 x 11 x 8 mm connector; 70 x 48 x 0.6 mm card, 2 g | **Estimates** (not found) |
| Adhesion | Liner release 0.4 N; hold 2.5 N on bare skin; 3 N for 1 s to stick | **Estimates** (published peel values: 0.1-1 N/cm on skin, about 0.3 N/cm on glass; none for this electrode) |
| Manikin | Prestan PP-AM-100M-MS, $215.00 | Prestan's listing on ProTrainings, read 2026-11-22; size and weight unverified |
| Chest shape | Sternum 150 mm over the bench, half-width 165 mm, a 0.6 m radius along the body | **Estimates**, to measure |
| Fur | Luxe Cuddle, 10 mm pile | Retail listings (snippets, unverified) |
| Lead wire | 4 mm | A 5-lead snap cable listing ("Lead Wire Diameter 4 mm"), read 2026-11-22 |
| Clipped-area cap | 70 cm2 (about five times the electrode's outline) | **Estimate** (Zio's manual: the prepared area "will need to extend larger than" the patch; no number found) |

## Forces against the robot

- The button: about 4 N to bottom it, pressed with closed fingertips.
- The clipper: 200 g (estimate), pinched across its 60 mm body; on the chest it presses with about 1-3.5 N (the
  GPU trace) and pulls against at most 1 N of drag.
- The electrode: 4.5 g with its connector; the peel off the card takes 0.4 N; the press takes 3 N for 1 s.
- Every load is far under the roughly 40 N each YAM finger manages (the figure the review instructions use).

## The scripted demo

It reads the simulator's state, as a scripted demo may, and only changes where the arms are sent; it never moves a
part or writes to the simulation.
- **Switch on, take the clipper (right arm).** Closed fingertips press the button on (sent 15 mm past its top: the
  arm's give sets the push); the hand opens round the clipper and pinches it across its body 90 mm behind the edge,
  lifts it, raises it to 130 mm over the tray and carries it to 60 mm over the patch, over the near end of the
  first pass (a lower, straight carry skimmed the abdomen).
- **Two pulls toward the robot.** Each starts 12 mm over the skin at the far side of the site, lowers onto the skin
  with the edge sent 8 mm into it along the normal (the arm's give seats the blade), and pulls 68 mm at 27 mm/s,
  teeth first, the body 35 degrees above the skin. The two passes are 24 mm apart along the body and clipped 44-45 cm2
  on the GPU. A servo watches the clipper: it moves the hand along the skin's normal (up to 4 mm back or 12 mm
  further than planned) until the edge's mean gap is 0.3 mm, during the pulls turns the
  clipper's pitch so the sole sits 5 degrees heel-up, and before each pull moves the hand until the edge is where
  the plan puts it (the clipper sits a little differently in the pinch each time); during the pull it holds the
  edge on its line.
- **Back and off.** It carries the clipper back high, sets it on the tray, lets go, and presses the button off,
  aimed at the button where it lies.
- **The electrode (left arm, the right hand pinning the card).** The right hand goes round the hips' end and pins
  the card's bare tab with its fingertips. The left hand, tilted 30 degrees, pinches the snap connector (aimed at
  it where it lies: pinning the card can shift it), peels the electrode straight up 40 mm (the card stays), and
  holds it up while the right hand leaves. It measures how the electrode hangs in its grip, carries it over the
  site, pushes it down onto the skin (3 mm into the skin along its tilted approach) and lets go. Then its
  fingertips, opened either side of the snap connector, press the electrode's head half and then its hip half
  (the tips' middles over its centre, then 20 mm toward the hips), each sent 22 mm straight down for 1.4 s. Both arms return to where they started.

## Results

From real runs of this task file on the official stack. Only one GPU type was available to this build (NVIDIA RTX
PRO 6000); the demo has not been run on another GPU type.

| Run | GPU | Trials | Score |
|---|---|---|---|
| Scripted demo | NVIDIA RTX PRO 6000 | 3 | **1.0 in 3 of 3** (each trial ended on success at 1,456 control steps; 838 s wall for the three) |
| Dream-1 (`dream`) | NVIDIA RTX PRO 6000 | 1 | **0.0** over all 2,400 steps (548 s wall; mean inference latency 0.58 s): it moved both arms over the manikin and back; the clipper stayed off on its tray, the fur uncut and the electrode on its card |

Before the evidence runs, this task file (the same file) was traced step by step on the official stack: 3 trials in
one process from the start state above. **All three scored 1.0.**

| Trial | Motor on | Site clipped | Clipper off on its tray | Electrode off its liner | Stuck (bare share) | From the mark | Scores 1.0 from (after its 1 s hold) | Steps |
|---|---|---|---|---|---|---|---|---|
| 1 | 3.65 s | 27.0 s (1,118 cells, 44.7 cm2) | 40.6 s | 51.75 s | 65.85 s (100%) | 3.3 mm | 71.8 s | 1,456 |
| 2 | 3.55 s | 26.95 s (1,115 cells, 44.6 cm2) | 40.6 s | 51.75 s | 65.85 s (100%) | 3.3 mm | 71.8 s | 1,456 |
| 3 | 3.55 s | 27.0 s (1,108 cells, 44.3 cm2) | 40.6 s | 51.75 s | 65.75 s (100%) | 2.5 mm | 71.8 s | 1,456 |

- Each trial began from the start state (the fur uncut, the electrode on its card, the clipper off on its tray) and
  ended at success; the clipper never touched a placed electrode or a lead (0 N all run).
- Runtime: the score reaches 1.0 at 71.8 s (step 1,436), once the 1.0 state has held 1 s; the official runner then
  ends the trial when success() has held 1 s more, at step 1,456 (72.75 s of simulated time). The run limit is 2,400 steps, about 1.6 times that.
- **Negative tests** (full runs from the start state, one trial each, on this file; each changes the demo's own
  routine in one way; the demo's servos only keep the blade seated and aim at the electrode where it lies, so none
  corrects the mistake tested):

  | Negative | Score | What the trace shows |
  |---|---|---|
  | The electrode stuck on without clipping (the clipper never used) | 0.0 | Pressed on the uncut pile 7.3 mm from the mark: its outline 0% bare, so it does not hold (0 N); the site 0% clipped |
  | Clipping with the motor off (the button never pressed) | 0.0 | Both passes seated on the chest (100 of 101 pull steps at 0.3 N or more): no cell cut; the electrode on the uncut pile |
  | The electrode set on the bare patch but not pressed | 0.5 | The site 100% clipped and the clipper off on its tray; the electrode 1.2 mm from the mark, the electrode's contact force on the chest never above 0.04 N (its own weight): not stuck |
  | The clipper put back on its tray running (the switch-off press skipped) | 0.75 | Site clipped, electrode stuck 2.6 mm from the mark on 100% bare skin, hands clear and still; the motor still on: an untidy finish, one step lower |
  | Clipping well over the cap: the demo's two passes and two more toward the hips, all out to the fur's near edge, with a re-grip on the tray between (in one grip the clipper turned in the pinch by the fourth pass and raked) | 0.75 | 2,041 cells, 81.6 cm2 clipped (over 70); the site 100% clipped, the electrode stuck 2.7 mm from the mark on bare skin, the clipper off on its tray: an untidy finish, one step lower |
  | Positive control: the running clipper held still in the air for 1.5 s before the first pass | 1.0 | The motor ran in the air (30 control steps, nothing cut there); then the routine as in the demo: 1.0 from 73.3 s, the electrode stuck 3.4 mm from the mark over 99.6% bare skin |
  | Do nothing | 0.0 | Nothing moved; the clipper off on its tray, the card on the bench, the fur uncut |

## Real trial: kit, safety and reset

- **Kit** (prices read 2026-11-22; "est." where the price was not verified): the clipper $120.16; its 9682 charger
  stand, about $63-154 (est.; the clipper is cordless and must be charged); the manikin $215.00; a bag of 50 Red Dot
  2560 electrodes (price not found; the sister 2570 sells for about $0.26 each, est.); a 5-lead snap cable, about
  $20-40 (est.); one yard of Luxe Cuddle, about $19-30 (est.; about 130 squares of 10 x 10 cm, so about $0.30-0.45 worth
  per trial at 160 x 120); a plastic tray and double-sided tape (a few dollars, est.). **About $470 at the lowest
  listed prices**, of which $335.16 is verified. The clipper is said to ship with one blade (unverified); on a
  manikin one blade serves many trials (a box of 50 blades is about $346, est., and is not needed).
- **Safety:** the clipper's blade is made to stand away from the skin ("Away-from-the-skin blade positioning
  minimizes the potential for nicks or cuts", 3M) and runs only on the fur; keep people's fingers off the chest
  while it runs. Clipped pile falls on the manikin: brush it into a bin after each trial; nothing wet, hot or
  sharp-edged reaches the robot. The gripper pads touch the clipper's plastic body, its button, the card and the
  electrode's connector.
- **Reset** (about 3 minutes, an estimate, not timed): peel the placed electrode off and discard it; lift the fur
  square off and discard it; tape a fresh 160 x 120 square, cut from the same bolt with its pile brushed toward
  the hips, over the sternum with its centre on the site (pencil ticks on the manikin's skin at the square's
  edges). Put a new electrode on a liner card, snap the fifth lead onto its stud, and lay the card at its marks, tab
  toward the hips. Switch the clipper off if it is on, brush the pile off its blade, and lay it on the tray at its
  marks, teeth toward the robot. Check that the four placed electrodes are still stuck and their leads lie as at the
  start. Consumed per trial: one electrode and one fur square.

## Assets and licences

- **This task** adds no third-party assets: every shape is built in the task file from primitives, convex slices
  and triangle meshes written in code. No logo or trademark is drawn.
- **The robot** is not in this folder; it comes with the official package under Requirements. That package's
  README (at the commit above) says its YAM robot model (`yam_arena/assets/yam/`) "comes from i2rt under the MIT
  license"; the file beside it, `LICENSE.i2rt`, reads "MIT License, Copyright (c) I2RT Robotics". The same README
  says the package's own code has no licence yet.
