# Trim a sheet on a two-speed sailboat winch: grind in fast gear, reverse into slow gear to full trim, stow the handle

One task file, `trim_sheet_two_speed_winch.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.

A plain-top two-speed sheet winch is bolted to a plywood deck board on the bench. The sheet runs straight across the
bench past the drum: from the sail's pull (springs below the bench's right-hand edge) over an edge roller, through a
fairlead on the deck board, three clockwise turns round the drum, and on as the tail over the left-hand edge to a
hanging 1 kg weight that keeps it taut. A red mark on the sheet reaches the edge of the drum at full trim. The locking
winch handle (a grey arm with a red grip, and a red lock lever on top of its hub) stands in its holder; a horn cleat
stands on a vertical board beside the winch.

The robot takes the handle from its holder, locks it into the winch (with both hands: the lever must be held turned
for it to go in), and grinds. Turning the handle clockwise drives
the drum in first gear (direct, fast); as the sheet comes in the springs pull harder until first gear is too hard for
the arm, and the robot reverses: anticlockwise is second gear, 2.76 times stronger, and the drum still turns the same
way. It grinds until the red mark reaches the drum's edge (there a stopper on the sheet meets the fairlead and the
sheet comes in no further), then takes the handle out with both hands, one holding the lock lever turned while the
other lifts, and puts it back in its holder. The pawls stop the drum running back, so there is no undoing an overtrim, and nothing tells the
robot when to shift except its own hands.

The skill is **precise control of force and position under a rising load, with a gear change made by reversing the
crank's direction**, followed by a two-handed release: one hand holds the handle's lock lever turned while the other
lifts the handle out.
On a boat one person grinds while another tails the sheet; here the robot is the grinder and the hanging weight is the
tailer.

## Requirements

- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`
  (Isaac Lab-Arena `release/0.3.1`; `inspect-robots` and `inspect-robots-yam` as it installs them);
  `source "$ARENA_DIR/yam.env"` in the shell.
- Nothing to download: the task file builds its geometry from boxes, cylinders, spheres and tube meshes.
- `-E control_hz=20`: the 1 s hold is 20 control steps.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/trim-sheet-two-speed-winch/trim_sheet_two_speed_winch.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Trim the sheet in on the winch until the red mark on the sheet reaches the drum's edge, then put the winch handle back in its holder." \
  --scorer task_score --max-steps 6000 --store-frames --log-dir ~/yam-eval/winch-dream
```

The scripted demo: the same command with `--policy yam_scripted`.

## Why it matters (Societal relatability)

- Boating is widespread in the US: **11,674,073** recreational vessels were registered in 2024 (US Coast Guard,
  Recreational Boating Statistics 2024), and "85M Americans Go Boating Each Year" (the trade group NMMA, citing the
  Coast Guard's 2018 National Recreational Boating Participation Survey). Grinding a sheet in on a winch is the job a
  short clip of a sailboat under way shows; this task is that job, done the way the winch's maker describes.
- The technique is the maker's own: "As the load increases, activate second speed by turning the handle in the
  opposite direction." (Harken's Rewind winch manual). The finish is a sailor's mark: "One way is to mark the sheet at the edge
  of the winch drum when maximum trimmed." (Sailing World, "On Your Marks, Get Set, Go", 2020).
- No jobs claim is made: the US water-transport occupations are well under the 500,000-worker bar.

## What is new, and what earlier reviews said

- **Plain crank turning is covered** (#616; #2216: "already tested by dexhand: Grinder Crank"; #2719: "ordinary crank
  turning"). What is new here: the load rises as the sheet comes in; partway through, first gear becomes too hard for
  the arm, and the only way on is to reverse the crank (the drum keeps turning the same way); the stop has no undo;
  and the locking handle takes both hands to take out (one holds its lever turned while the other lifts).
  A fast-only grind stalls short of the mark; a slow-only grind gets there but takes 4.83 handle turns
  against 2.99 (both are scored runs, below).
- **#2889** (a two-speed landing leg) was closed with "Real-world trials would be too difficult to reproduce
  consistently as specified". This task specifies the load exactly: two extension springs with their rates, preloads
  and anchor points, a stopper, the marks, and luggage-scale readings to check the rig at three marks before each
  session (Real trial, below).
- **#3699** (seating a trailer-winch handle) was closed as "a niche operation that most people will not recognize on
  sight". This task is the whole trim on a sailboat's sheet winch (fit the handle, grind in both gears against the
  sail's pull, stow the handle), not seating a handle.

## Setup and objects (millimetres; x away from the robot, y to its left, z up from the bench)

- **Deck board:** 18 mm plywood, 480 x 740, centred at (380, 230), clamped to the bench.
- **Winch:** Harken Performa 20.2PTP (plain top, two speed; gear ratios 1:1 and 2.76:1, power ratios 6.95:1 and
  19.20:1; base 5-3/8 in, drum 2-7/8 in, height 5-1/16 in, 4.4 lb, maker's table via a retailer), bolted to the deck
  board with its axis at (400, 100). First gear is clockwise seen from above (Harken's convention as summarised from
  its manual; to confirm on the unit before a trial by counting drum turns per handle turn: 1 clockwise, 1/2.76
  anticlockwise).
- **Handle:** Lewmar 250 mm (10 in) grey alloy locking winch handle, part 29141111 (two retailer listings: "250mm/10
  Grey Alloy Locking Winch Handle, forged", "Colour: Gray"). Its weight is not published: modelled at 600 g, the
  listed figure for a comparable 254 mm aluminium lock-in handle (Harken B10ASG, via a retailer). Modelled 254 mm (10 in) from the
  socket's axis to the grip's, the length the winch maker's power ratios are for; the listing's 250 mm is 1.6% shorter.
  Its grip and lock lever are drawn red, as in the photo the task was modelled on; the retailer listings do not state
  those colours. Owners' forum posts (no maker's page) say these makers' winches and handles share the 11/16 in
  eight-point ("double-square") drive; this kit mixes brands, so check the fit before a trial.
- **The lock lever** (shape, travel and force not published: estimates): a lever on top of the hub, turning about the
  drive's axis, 72 degrees anticlockwise from the arm (seen from above), with a thumb tab 40 mm from the axis. Turned
  anticlockwise against its spring (2 N at rest, 5 N at its 30 degree stop, at the tab) it moves the latch: free from 20
  degrees, locked again below 10. An owners' forum describes the lock as "a small piece of steel that rotates with the
  thumb lever that slides under the socket". The handle stands in its holder at (250, 500), an open socket on a post
  (no lock) whose top is 100 mm above the deck board, its arm pointing away from the robot and 52 degrees to its right
  (across the line from the left arm's base to the holder).
- **Sheet:** 10 mm double-braid polyester. It runs along x = 358.5 (the drum's side nearer the robot): from an edge
  roller clamped to the bench's right-hand edge at y = -640, through a fairlead on the deck board's edge at y = -120,
  onto the drum (three clockwise turns), and off the top turn as the tail along +y to an edge roller at y = +640, then
  down to a 1.0 kg weight hanging 120 mm below the bench top. A red tape mark on the sheet is 406.9 mm from the drum's
  edge at the start (it reaches the edge at full trim). A 24 mm stopper ball on the sheet touches the fairlead's rubber
  snubber at full trim (its centre then 26 mm beyond the fairlead's centreline) and stops the sheet there.
- **The sail's pull:** below the right-hand edge roller the sheet's end is held by two extension springs to a floor
  anchor: spring A (always on) 200 N/m with 30 N of preload; spring B 934 N/m, on a slack link that takes up after
  150 mm of sheet has come in. So the sheet's load is 30 N at the start, 60 N at 150 mm, and 351 N at full trim
  (406.9 mm); past it the stopper's snubber (about 20,000 N/m) holds the sheet.
- **Horn cleat:** a 6 in horn cleat on a vertical board (18 x 200 x 120) at (569, 480). It is not used in this task:
  the weight tails the sheet, and the trim is held by the winch's pawls and the turns on the drum.

## Score

Measured: **how far the red mark ends short of the drum's edge** along the sheet (a ruler on the sheet; past the edge
counts as at it), the handle turns it took to get there (counted on the trial video), where the handle is, and
whether the hands are clear.

- **1.0:** the mark within 5 mm of the drum's edge (or past it), reached within 4 turns of the handle (both
  directions, counting the turns that moved the sheet: first gear while it is easy, then second, took
  2.99 in the demo; second gear alone took 4.83, and takes that many however fast the hand turns); the handle out of
  the winch and standing in its holder; both hands at least 30 mm from the handle; the handle, its knob and lock lever
  and the winch's spindle still (each under 10 mm/s); all held for 1 s.
- **0.75:** the mark within 5 mm without the full 1.0 finish (more than 4 turns, the handle not in its holder, a hand
  within 30 mm of it, a part moving, or not yet held 1 s).
- **0.5:** the mark 5 to 30 mm short, with the handle in its holder, the hands clear and the parts still.
- **0.25:** 30 to 80 mm short, with the same finish.
- An untidy finish (the handle not in its holder, a hand within 30 mm of it, or a part moving) steps 0.5 and 0.25 down
  one rung.
- **0** otherwise, and for the whole trial if the handle is dropped: falling at 1 m/s or faster (a 51 mm free fall)
  with no gripper on it and not in the winch or the holder. A dropped winch handle can go overboard or land on a foot.
- **Void** (0) if the parts' or arms' state can't be read, a write fails, the simulation goes non-finite, or a new
  trial's parts aren't within 5 mm of their start (checked at its third control step).
- The score is read at the trial's end (the finish must hold through its last 1 s).

Turns: the handle's turns about the winch's axis while they drive the drum (both directions take the sheet in),
summed; the 3 degrees of free play after each reversal does not count, nor does jiggling of under 0.5 degrees. By
hand: count the knob's revolutions on the video while the mark moves toward the drum. Why turns: first gear is there to
bring the sheet in quickly while the load is light, and the turn count stands in for that without timing the arm; so
second gear alone, which trims fully but slowly, loses one step (0.75), not the trim.

Definitions. In the winch: counted from when the spigot is 6 mm into the socket (axis within 2 mm, tilt within 8
degrees) until it is lifted 23 mm with the lock free. In its holder: counted from when it seats in the
holder's socket (axis within 4 mm, tilt within 8 degrees, within 4 mm of seated; its bore is 2 mm a side clear) until
it is lifted 23 mm, and out of the winch. The lock: freed once the lever is turned 20 degrees, and across again only
when the lever drops below 10 degrees, going in as well as out; across with the handle fully in (within 3 mm of
seated), it locks; across part way in or out, the latch catches it there (a handle caught part way still counts as in
the winch and still drives it). Still: the handle, its knob and lock lever and the winch's spindle each moving under
10 mm/s. Hands clear: every finger-pad point 30 mm or more from the handle's shapes (the lever counted as the square
it can sweep). Gripper on it: pads closer than 60 mm with a point between them within 5 mm of the handle.

Real-trial checks: a ruler on the sheet from the mark to the drum's edge, the knob's turns counted on the video while
the sheet comes in, a photo of the handle in its holder, and the video for hands and drops.

## The force window (the arm's motors)

The two gears differ by 2.76 times, close to the YAM's ratio of peak to rated joint torque (Damiao DM4340, joints 1-3:
9 N m rated, 27 N m peak; DM4310, joints 4-6: 3 N m rated, 7 N m peak). So no load lets the slow gear finish within
the rated torque while making the fast gear impossible for any push the arm can make:

- Offline (the official joint model), the arm's tangential force at the knob round the circle is about 12 N at rated
  torque (10.4 to 17.8 N on the near two-thirds) and 38.4 N at peak torque at the circle's weakest point.
- Full trim needs 18 N at the knob in second gear; first gear there would need over 49 N. First gear at 38.4 N stalls
  67.3 mm short (the rig without arms on the GPU: 339.6 mm of sheet in, 406.9 mm needed).
- **The slow phase therefore runs above the motors' rated torque.** Measured on the GPU over three trials of the
  demo (the push at the knob is the handle's torque over its 254 mm length): second gear held a median 19.6-19.7 N,
  95th percentile 22.5-22.6 N, peak 23.5-23.6 N, and more than 18 N for about 18 s of its 30 s; the whole routine's
  peak was 23.8 N (first gear's 95th percentile 22.9-23.0 N). That is under the arm's peak-limited 38.4 N and the
  gripper's roughly 40 N a finger, but about twice the 12 N the arm gives at rated torque: **on real hardware the slow
  phase would run the shoulder above its continuous rating for about 18 s.**

## How the file works, and its fidelity

Simulated as rigid PhysX bodies at real size (shapes not in the maker's figures are estimates):

- **The winch's body and socket** (a fixed body with the socket's bore), **the spindle** (the input shaft the handle
  drives), **the handle** (spigot, hub, arm; grey), **its knob** (red; spins freely on the handle) and **its lock
  lever** (red; a bar on the hub's top with a thumb tab, turning about the drive's axis through 30 degrees against a
  spring, 2 N rising to 5 N at the tab; estimates, as above). The deck board, the winch's base and bore, the
  holder, the rollers and their brackets, the fairlead and the horn cleat also collide; the sheet, its turns, the tail,
  the weight, the mark, the stopper, the snubber and the drum's outer barrel are drawn only.
- **The socket's fit and the lock** are a joint, not contact between the drive's eight points and the socket. Once the
  spigot is 6 mm into the socket, on its axis within 2 mm and upright within 8 degrees, a joint ties the handle to the
  spindle: across and in tilt it is held; along the axis it slides freely once the lever has been turned 20 degrees,
  until it drops below 10. With the latch across: fully in (within 3 mm of seated) it locks, a stiff catch (200 kN/m,
  up to 300 N); part way in or out, the same catch holds the handle where it first counted as in (6 mm in, or deeper
  if it arrived fast). Lifted 23 mm with the lever held, the handle is free. That the lever must be held to put the handle in, as well as to take it out, is modelled, not
  sourced: no maker's text was found; an unverified search snippet for another make's handle sells it as going in
  "without the need to depress or rotate a knob", which suggests ordinary locking handles do need it.
  Round the axis the joint is a stiff spring (50 N m/rad), whose twist is the push the handle passes to the gears, so
  it is known at rest too.
- **The gears** are a per-step coupling, not modelled teeth (the real gears are inside a closed housing too). Each
  control step, the spindle's turn since the last step drives the drum: a clockwise turn 1:1, an anticlockwise turn
  1:2.76, the drum always the same way, after 3 degrees of backlash at each reversal; the drum never turns back (the
  pawls). The spindle drives the drum only once it goes past the furthest point it has reached since its last
  reversal, and a reversal counts only after it has gone back 0.5 degrees (so solver jitter takes no sheet in). The
  sheet taken in is the drum's turn times its 36.5 mm radius (the radius the maker's power ratios imply).
- **The load at the handle** is a brake on the spindle: a stiff position hold whose set point gives way each step only
  by the twist above the limit for the pushed direction, the sheet's load times the drum's radius over that gear's
  ratio (the tail weight's pull helps), so under the limit nothing moves and the handle feels exactly the limit at rest.
  The limit includes 0.05 N m of gear-train drag (an estimate). Which gear's limit applies is read from the sign of the
  twist (a twist under 0.3 N m either way counts as no push and gets the larger, first-gear limit), so a stalled
  first-gear push never creeps.
- **The sheet is not simulated rope.** Its three turns on the drum, the lead to the fairlead and the roller, the tail
  and the weight are drawn; the red mark and the stopper ball move along the drawn lead by the sheet taken in; the drum
  turns by the coupling. The sheet's load is the springs' law of the sheet taken in (30 N + 200 N/m, then 1,134 N/m
  after 150 mm, then the snubber at 20,000 N/m past full trim). The tail and its weight are drawn and nothing can reach
  them, so the file treats the tail as always held unless the sheet's load passes what three turns hold against a
  9.81 N tail: 620 N at a friction coefficient of 0.22 (an estimate, polyester on aluminium). Past that the turns slip for good: the drum turns under the sheet, takes nothing more in, and the handle
  turns freely. The demo's 22 N cap stops at about 428 N on the stopper; a second-gear push over about 32 N would reach
  620 N (13 mm onto the stopper), with the mark already past the drum's edge.
- **The holder** is an open socket on a post, with no lock (its bore 2 mm a side clear of the spigot). While the spigot
  is seated in it, a joint keeps the handle upright there (a snug pocket: up to 28 mm of lift along the axis, and turning round it against
  0.5 N m s/rad of drag); lifted 23 mm, the handle is free.
- **Physics written each step:** the brake's set point, the socket and holder joints' state, and the drawn drum, mark
  and stopper.

## The scripted demo

Both arms. It reads the knob's, the handle's and the lock lever's places (the simulator's state, which the rules allow
for a scripted demo; a real robot would take the knob's place from its own joint encoders while it holds it) and the
push at the knob (the handle's twist, while the socket holds the handle; a real robot would read it from its motor
currents). The left arm cranks and carries the handle; the right arm works the lock lever.

1. The left hand takes the handle from its holder by its hub (pads level, the approach tilted 60 degrees from straight
   down), 40 mm up the hub, carries it over the winch, lines the spigot up over the socket and sets it in: with the
   lever let go the latch stops it part way in (8 mm above seated in the demo, where it first counted as in), and the
   socket now holds it upright.
   The left hand loosens its pinch and stays round the hub.
2. The right hand sets off only at this point in the plan, once the handle is set: the knob stands between the right
   arm's base and the hub, so the hand rises, passes out beyond the winch and comes over the hub from there. It puts its closed
   fingertips behind the lever's tab, then pushes the tab along its arc: its target is the tab's place plus a push
   along its travel that grows until the lever reads 27 degrees and eases off past 29 (the arm gives under load, so a
   fixed target would not press the spring's 5 N). With the lever
   held turned, the left hand pinches the hub again and pushes the handle home; the right hand lets go and the lever
   springs back: locked. Both hands let go.
3. The left hand grips the knob from the robot's side (pads level, the approach tilted 60 degrees from straight down),
   where the knob is when it sets off.
4. Grinds: each control step the hand's target is the knob's place pushed ahead along the circle's tangent; the push
   grows while the knob turns slower than one turn in 12 s, shrinks when it turns faster, and is cut back fast when the
   push it reads passes the gear's cap (22 N) or the knob runs away. In first gear (clockwise) it grinds until the knob
   has turned less than 3 degrees in 2 s with the push at 85% of the cap or more (checked from 3 s into the gear): too
   hard. Then it reverses into second gear (anticlockwise, cap 22 N) until the same happens again: the mark is at the
   drum's edge and the stopper holds the sheet.
5. Takes the handle out with both hands: the left hand lets go of the knob and pinches the hub; the right hand turns
   the lever and holds it; the left hand lifts the handle out; the right hand lets go once it is out. The left hand
   carries it to the holder, puts it in, lets go, and both arms go back to their ready poses.

Limits of the scripted demo (not of the task): its hub pinch holds the handle's off-centre weight against the pads'
faces only when the handle points across the line from the left arm's base; spring B is sized so the demo's grind nets
one whole turn, which brings the handle out pointing that way, with the lever on the side the right hand reaches
without crossing the left hand. A grind that ends at another heading leaves the handle leaning in the pinch and the
lever elsewhere. The left arm's ready pose, where it parks, is within 30 mm of the stowed knob for some stow headings
other than the demo's.

## Results

All on the official stack above, NVIDIA RTX PRO 6000 Blackwell, task file SHA-256
`89c467681a4aacf69919ddbbf180d5485992f678557e79289ea385a351b8a307`.

- **Scripted demo, 3 trials in one process, the task file itself:** 1.0 on 3 of 3 (per-trial scores 1.0, 1.0, 1.0;
  2,277 control steps, about 114 s, a trial).
- **The same demo traced (3 trials; the file loaded unchanged inside a logging wrapper), for the figures below:**
  1.0 on 3 of 3. Full trim after 2.985, 2.989 and 2.993 handle turns (at 75.75, 75.25 and 75.05 s); the mark ended
  3.8, 4.0 and 3.9 mm past the drum's edge (on the stopper); the handle back in its holder each time (seated at 108.0
  to 108.05 s), scoring 1.0 from 112.85 s.
- **The handle, per trial:** going in, the latch stopped it 7.1 to 8.2 mm above seated; with the lever turned 30.1 to
  30.2 degrees it went home, and let go it latched (24.3 s). Coming out, the right hand turned the lever 30.2 degrees,
  the left lifted the handle out (out at 101.35 to 101.4 s, lifted 68 to 71 mm); the two hands' tool points stayed at
  least 50 mm apart while the lever was worked. Over the holder the left hand's grip turned the handle's lean in the
  pinch from 3.3 to 4.4 degrees down to 0.5 to 0.6 before lowering it. (On an earlier build without that turn, one
  trial in three leaned 9 to 10 degrees, jammed in the holder's mouth 5.7 mm above seated and scored 0.75.)
- **Grind, per trial:** first gear 1.07 turns in 18.2 to 18.9 s, stalled at 240.3 to 240.8 mm of sheet (the file's own
  stall point at 22 N: 239.4); second gear 2.08 to 2.09 turns in 29.8 to 30.5 s, stalled at 410.7 to 410.9 mm.
- **Negatives, one trial each** (the demo's routine with one wrong choice; no servo term corrects it):
  - nothing: 0 (the mark 406.9 mm short);
  - first gear only, at the demo's 22 N cap: 0 (it stalled with the mark 166.1 mm short; the handle taken out with
    both hands and put back);
  - second gear only: 0.75 (full trim, 4.1 mm past the edge, but after 4.83 handle turns; the handle taken out with
    both hands and put back);
  - handle left in the winch: 0.75 (full trim; the handle still locked in the winch);
  - pulled up with the lever never turned (the left hand pinches the hub and lifts; the right hand stays back):
    0.75 (the lock held: the handle rose 0.12 mm and stayed latched in the winch);
  - the lever turned (30.2 degrees) and let go before the lift, then pulled up: 0.75 (it relocked; the handle rose
    0.13 mm and stayed in the winch);
  - handle let go 150 mm up after it comes out: 0 (it fell at 1.53 m/s with no gripper on it).
- **Dream-1** (the instruction above, one trial, the task's 6,000-step limit): 0. By 100 s it had taken the handle
  out of its holder and left it lying on the deck board between the holder and the winch, where it stayed to the end;
  the handle never reached the winch and the red mark did not move (top-camera frames at 0, 100, 200 and 300 s).
- **The rig without arms** (stiff jacks turn the knob with a set push and the lever with 8 N at its tab; same spring
  and mechanics as the file): a 40 N pull with the lever at rest lifts the latched handle 0.15 mm; with the lever
  turned it comes out; lowered back with the lever at rest, the latch stops it 17.8 mm up; with the lever turned it
  goes home, and let go it latches. First gear at 22 N stalls at 238.0 mm of sheet (the file's own stall point 239.4);
  at 38.4 N at 339.6 mm (339.9), 67.3 mm short; second gear at 10 N, under its load, creeps 0.014 mm; at 22 N it
  reaches 410.6 mm, 3.7 mm onto the stopper; let go, the drum moves 0.02 deg; with the lever turned the handle comes
  out under load and the sheet moves 0.004 mm.

## Cost and parts

- Harken Performa 20.2PTP $656.95 (retailer listing); Lewmar 29141111 handle £96.89 to £107.61, or €128.95 (two
  retailer listings); sheet, two springs, a 1 kg weight, two clamp-on rollers, plywood and clamps: **about $850 in
  all** (an estimate). That is more than a few
  hundred dollars. The cheaper route is a Barton 21201 composite two-speed winch, $308.23; its per-speed ratios are not
  published, so a real trial with it would first count handle turns per drum turn in each direction and size spring B
  from that.

## Real trial: rig, safety and reset

- **Rig check before a session** (a luggage scale hooked into the sheet at the fairlead, pulled by hand along the
  lead): 30 N with the mark at its start (406.9 mm from the drum's edge), 60 N with it 256.9 mm from the edge (150 mm
  in), and 351 N with it at the edge (spring B's slack link must take up within 5 mm of the 150 mm mark; adjust its
  length). With the mark at the drum's edge, the stopper ball (24 mm) just touches the fairlead's snubber.
- **The springs** are specified by rate, preload and travel (catalogue parts to be sourced to these numbers): spring A
  200 N/m, 30 N preload, at least 420 mm of further travel; spring B 934 N/m, at least 270 mm of travel. Both hang
  below the bench's right-hand edge from the sheet's end to a floor anchor (an eye bolt in a 20 kg kettlebell, or a
  floor ring), clear of the robot's rig.
- **The tail weight** (1.0 kg) hangs 120 mm below the bench top at the start and drops about 410 mm by full trim: it
  ends about 170 mm above a 750 mm bench's floor (with its own 50 mm height). A clamp-on roller (any 25 mm smooth
  roller or a rope fairlead) at each bench edge carries the lead and the tail. On a real winch the tail must stay taut
  while the drum turns: if the weight is lifted or the tail goes slack, the turns slip on the drum and the sheet comes
  in no further, so a robot that disturbs the tail loses the trim.
- **Safety:** at full trim the sheet carries about 350 N (36 kgf). Keep hands and faces off the lead and the springs'
  line while the robot grinds; the springs hang below the bench, guarded by its edge. The winch's pawls hold the load
  when the handle is out; the robot's grippers touch only the handle. Both arms work within a few centimetres of each
  other over the winch while the handle goes in and comes out: run that part slowly first, with the sheet slack.
- **Reset** (about 3 minutes; an estimate, not timed): free the sheet from the drum, let the springs take it back out
  until the mark is 406.9 mm from the drum's edge (the stopper ball against nothing), put three clockwise turns back on
  the drum, hang the tail weight 120 mm below the bench top, and stand the handle in its holder, arm pointing as at
  the start.

## Assets and licences

- **This task** adds no third-party assets: every shape is built in the task file from primitives. No logo or
  trademark is drawn.
- **The robot** is not in this folder; it comes with the official package under Requirements. That package's README
  (at the commit above) says its YAM robot model (`yam_arena/assets/yam/`) "comes from i2rt under the MIT license";
  the file beside it, `LICENSE.i2rt`, reads "MIT License, Copyright (c) I2RT Robotics". The same README says the
  package's own code has no licence yet.
