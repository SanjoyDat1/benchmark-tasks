# Pick the right tool from a pile and hand it over handle first

One task file, `tool_from_pile.py`, for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena) on Dream Labs' station scene.
A coworker setting concrete pavers asks for the tool to tap them down without chipping them, and holds out an open
gloved hand. The robot has to work out that the tool for that job is the rubber mallet (not the look-alike steel
cross-peen hammer beside it), lift the two tools lying across the mallet's handle off it, and hand the mallet over
handle first: the handle's upper part across the palm, the head just past the thumb, pointing away from the
coworker, then let go.

The core is choosing the tool from the job and getting it out of a pile; the safe, one-handed hand-over is the
finish.

## Requirements

- `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh`
  (Isaac Lab-Arena `release/0.3.1` with its object library pointed at NVIDIA's production bucket, `inspect-robots`
  1.0.0, `inspect-robots-yam` 0.38.0 and `inspect-robots-dream`). Tested on Isaac Lab 3.0 / Isaac Sim 6.1 as recorded
  there.
- `source $ARENA_DIR/yam.env` in the shell.
- Network access to NVIDIA's public asset bucket: the four tools are NVIDIA SimReady assets loaded from it (see
  "The tools").

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/tool-from-pile/tool_from_pile.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "I'm setting concrete pavers and need to tap them down without chipping them. Find the tool for that in the pile and hand it to me handle first: lay its handle across my open palm with the head out past my thumb, pointing away from me, then let go. Leave the other tools on the bench." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/handover-dream
```

The scripted demo (full score): the same command with `--policy yam_scripted` and
`--log-dir ~/yam-eval/handover-demo`.

Videos: `for log in ~/yam-eval/handover-*/*.json; do inspect-robots video "$log"; done`.

## Score

`--scorer task_score` gives 1.0 when the task succeeds and otherwise the task file's `score()` at the end of the
trial. Every step also reports `info["score"]`.

Terms: a tool's **outline** is its world-aligned bounding box (the box of its turned bounding box, so it holds the
whole tool; for the tilted mallet it reaches up to 21 mm below the real tool, which is why heights use the outline's
middle or the mallet's own frame). A gripper's **tool point** is the point midway between its fingertips (the
framework's TCP); its **finger pads** end 13 mm past it. A gripper **holds** a tool when its tool point is within
3 cm of the tool's outline, its fingers are closed to under 8.5 cm (fully open reads 9.4 cm; closed on the mallet's
head, about 7.1 cm), and the line between its pads crosses the outline; a held tool never counts as let go. A tool has **fallen** when its outline's centre is
off the table or 5 cm below its top; it is **up** when its outline's middle is 7 cm or more above the table (lying,
the tools are 1 to 5 cm up). **Still** means under 1 cm/s and 0.5 rad/s.

**Handed over properly**, from the mallet's pose (its frame comes from the reported orientation and the outline's
centre, which is its bounding box's centre):

- the handle lies across the palm: the handle's axis crosses over the palm, and its underside over the palm's
  centre is -4 to +8 mm from the palm's top;
- the head points toward the thumb, away from the coworker's body, within 25 degrees (so the handle runs across
  the palm, not along the fingers);
- the handle is within 10 degrees of level;
- the point of the handle over the palm's centre is 1 to 7 cm from where the handle meets the head: the upper part
  of the handle is in the palm, and the head lies just past it.

Why this pose: the mallet's mass is mostly in its head, and its centre of mass is on the handle 0.8 cm from the head.
A static open hand can't close, so the mallet has to balance on it: with the handle's upper part across the palm,
the centre of mass is over the palm and the head lies just past the palm's thumb-side edge, on the thumb (laid
flat, its top 2.2 cm below the palm's, just above the bottom of the head). That is also how a person takes a
hammer: the handle across the palm and the head beyond the thumb and forefinger, so closing the hand grips it ready
to use. Handed the other way round, the head would point at the coworker's body, and with the grip end in the palm
the mallet would tip off.

Nothing scores (0) if, at any point, the mallet was **dropped** (once no gripper holds it, its outline's centre falls 2 cm
below where a gripper last held it, or it moves down faster than 0.63 m/s, the speed of a 2 cm free fall), it was
**dragged** out of the pile (its outline centre 2 cm or more from where it lay, in plan, while a cover still within 5 cm
of where it lay overlaps its outline in plan and in height, so lifting the mallet clear of a cover lying beside it is
fine), or another tool was **knocked** about (moving over 5 cm/s with no gripper's tool point
within 5 cm of it, for 2 control steps in a row, after the first second while the heap settles: lifting the mallet with the tools still on it does this); or if, at the end, any tool has fallen, the hammer's outline centre
is more than 5 cm from where it lay, or another tool is up, still held by a gripper, or lies on the coworker's hand, cuff or sleeve (its outline over one
of them and within 3 cm above it). Lying, the tools' outline middles are at most 4.6 cm up (the pry bar tilted on the
mallet), so "up" at 7 cm can't be reached lying down, and a cover still in a gripper counts whatever its height. If
the arms' state can't be read, the trial scores 0: nothing that ends credit could be checked. Otherwise:

| Score | What must be true at the end |
|---|---|
| 0.25 | The mallet's head rests on the palm (head first), or a gripper holds it up or within 7 cm of the palm's centre |
| 0.5 | Its handle rests on the palm, but it isn't handed over properly |
| 0.75 | Handed over properly (a gripper may still hold it) |
| 1.0 | Handed over properly, every gripper within 10 cm of the mallet open at least 6 cm with its tool point at least 3 cm from the mallet's outline, and every tool still, all held for 1 s (20 control steps) |

Caps, once they happen: 0.25 if the head was aimed at the coworker (within 30 degrees of the direction to their body)
while a gripper held it with its outline within 15 cm of the palm's centre, 2 steps in a row; 0.5 if any point of the mallet moved faster than
0.25 m/s (the reduced speed for robots working near people) while a gripper carried it (held, not yet resting on the
palm) with its outline within 15 cm of the palm's centre, 2 steps in a row; 0.5 if a gripper's tool point or finger pads came within 3 mm of the coworker's hand, cuff or sleeve (the wrist links
are not checked).

The embodiment senses no finger contact, so holding, letting go and touching are judged from geometry. Because the
outline holds the whole mallet, a tool point 3 cm from it is at least 3 cm from every part of the mallet. A closed
gripper parked more than 10 cm away doesn't count. `success()` holds from the first step everything 1.0 needs is
true and nothing has capped the score; the embodiment ends the trial once that has held for 1 s, which is the step
`score()` reaches 1.0, so a trial cut off sooner ends at 0.75.

No pattern scores without choosing the mallet and handing it over: handing over the hammer moves it (0), a sweep or
a hedge moves it too, any other tool on the coworker or still held up ends credit, a naive grab can't close on the handle under the pry bar or knocks the covers about (0), dragging the mallet out
from under them scores 0, and a head-first hand-over scores at most 0.25, and only the handle-first pose scores above 0.5. The exact rules are in `task.json` and in the file's
`state()` and `grade()`.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes). The scripted demo plans 33 s of simulated time: both arms
  clear the two tools off the mallet's handle at once (about 10 s), then the left arm grips the mallet near its
  head, lifts it, carries it to the coworker (slowly, about 0.1 m/s, over the last 38 cm), lays it on the palm, lets go and parks.
  Everything the full score needs holds from about 26 s, and the trial ends as a success 1 s later. Wall clock on one
  RTX PRO 6000: about 3 minutes for one trial, about 2 of them start-up; each further trial in the same
  process adds about 50 s. A trial that runs to the step limit (Dream-1's did) takes about 12 minutes.
- No randomness: the heap is a fixed layout (`-E object_jitter=0`), so no seed is needed. With jitter, each loose
  object would be shifted on its own, and the tools lying across the mallet could be shifted into it.

## The tools

NVIDIA SimReady hand-tool assets (Creative Commons Attribution 4.0, CC BY 4.0; each asset folder carries its
`LICENSE.txt`), loaded at real size from
`https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/SimReady/Industrial/Tools/Hand/`.
The scene hook accepts only these four files. ETags recorded 2026-11-09 (top-level USD; geometry payload):

| Tool | Asset folder | Size, mass | ETag (top USD) | ETag (`payloads/geometries.usd`) |
|---|---|---|---|---|
| Rubber mallet (the answer) | `Rubber_Mallet_A01` | 39.4 cm, head 7.1 x 13.1 cm; 585 g | `899e2af3993a2728eb7524b29ad36ad1` | `2f50b768347b75a2c44d20fae810d14d` |
| Fork pry bar | `Metal_Fork_Pry_Bar_A01` | 20.7 cm; 131 g | `6daae16db141b972c5e927a34d7408db` | `64f32e06a3fc5d1e78efd981c342de4e` |
| Phillips No. 1 screwdriver | `Screwdriver_Phillips_Ph1_A01` | 14.7 cm; 49 g | `3dabfa28f9ba9686b6933959b0d1a8e2` | `fa0074bcd130fbfc4eec87d49000d8aa` |
| Cross-peen hammer (the look-alike) | `Cross_Peen_Hammer_A01` | 39.4 cm, head 13 cm; 1.35 kg | `8d6d93032c43b8169eeb53d1d50bb1d4` | `b4036a66adec0ff126aa1f2f8cd3557c` |

Each is one rigid body with the asset's own mass, physics materials (friction) and collision (signed-distance-field
meshes; convex decomposition for the Phillips screwdriver), spawned with the asset's `Physics` variant set to
`physics`, as Arena's own SimReady objects are. Two changes, made right after each tool is spawned:

- **Mass properties of the mallet and the hammer.** The assets put the mallet's centre of mass 7.7 cm out along
  its handle and the hammer's at its middle (PhysX uses them as authored), although nearly all of each tool's mass
  is in its head. The file replaces their centre of mass and principal moments with a two-part model at the assets'
  own total masses: a head (the mallet's 71 x 131 mm rubber cylinder, 450 g; the hammer's steel head, 1.15 kg) and
  a 32 cm handle (135 g; 200 g). The mallet's centre of mass is then on its handle 0.8 cm from the head, the
  hammer's 1.1 cm inside its head.
- **Spin damping** of 2 per second on every tool: PhysX has no rolling resistance, so a round screwdriver handle
  dropped on the bench would otherwise rock or roll for a long time.

If a tool doesn't come out as exactly one rigid body, the hook raises instead of building it with the asset's
values. Left out on purpose: the dead-blow hammer (also non-marring, so the job would have two right answers), tools
with joints (pliers, adjustable wrenches: several rigid bodies each), and a Torx screwdriver (its centre of mass lies
on its round handle's axis, so it kept rocking on the bench).

## The coworker's hand

Fixed boxes on the robot's left: a palm 10 x 9 cm and 4 cm high lying palm
up on the table (x 31 to 41, y 35.5 to 44.5 cm), four fingers toward the robot, a glove cuff and a navy sleeve to the
table's far edge, where the coworker stands. Beside the palm, a low thumb web and
then the thumb lie flat, the thumb's top 2.2 cm below the palm's, 4 mm above the bottom of the mallet's head when its
handle lies flat on the palm (the head reaches 2.6 cm below the handle). The head then rests on the thumb and the
handle on the palm, tilted about 2 degrees. On official-stack runs, lying on its round handle alone the mallet rolled
about the handle without ever settling, and with the thumb level with the head it chattered between the two supports. The hand can't close
or move, so the mallet has to be left lying on it.

## How the file works

- **Loading.** The task-file API has no external models, so the four tools are `Tool` specs (a subclass of the
  API's `Asset`). Right after `yam_arena.scene` is imported, the file wraps `yam_arena.scene.task_object`: a spec
  carrying the `built_by_hook` marker builds itself from its USD, and every other object goes to the original.
  Dispatch is by marker, not class, because the official loader imports the task file afresh on every load. On a
  machine without the simulator nothing imports `yam_arena.scene`, so the hook does nothing there.
- **The heap.** Each tool's resting pose was solved from its measured mesh: the mallet lies on its head and the end
  of its grip (tipped 2.9 degrees), the pry bar and the Phillips screwdriver rest with one end on the table and the
  other across the mallet's handle (the pry bar right over the best place to grip it), and the hammer lies flat. Each
  is spawned 1.5 mm above that pose and settles under gravity during the reset (the pry bar settles at about 17.2
  degrees, the screwdriver at 16.7).
- **Episode rules.** `score()` is called after every control step. The score is read from the state at the end,
  plus what carries between steps: how many steps in a row everything 1.0 needs has held, where a gripper last held
  the mallet, and whether a drop, a knock, an aim, a speeding or a touch has happened. A new trial (the embodiment's
  reset, which the file wraps) clears them. `success()` and `score()` share each step's state dict, so a step counts
  once.
- **The robot's grippers.** Task files get only the objects' states, so the file finds the calling embodiment (the
  caller with an episode clock, a task and a `reset`) and uses it once its reset is hooked. Every step it reads both
  arms' joint positions from it and turns them into tool points, finger directions and pad positions with the
  framework's own forward kinematics (`yam_arena.ik.tcp_pose`). If the arms can't be read on a step, it warns (once
  per trial), that step scores 0, and it tries again on the next step.
- **The scripted demo** (`demo()`): the left arm lifts the pry bar off the mallet's handle and lays it down behind
  the heap while the right arm does the same with the Phillips screwdriver at the front right. The left arm then
  grips the mallet's handle 2 cm from the head (a shallow grip: the tool point 8 mm above the handle's axis, so the
  pads stay about 7 mm above the palm), lifts it, carries it toward the coworker, slows to about 0.1 m/s near them,
  lowers the mallet onto the palm with the head 3 mm past its thumb-side edge, opens, rises and parks.

## Sources

- Rubber mallet for pavers, steel hammers chip them: Angi, "Tools and Materials Needed to Install Concrete Pavers"
  (https://angi.com/articles/what-tools-do-you-need-lay-pavers.htm); EngineerFix, "Why You Need a Rubber Mallet for
  Pavers" (https://engineerfix.com/why-you-need-a-rubber-mallet-for-pavers/).
- High spots tapped down with a rubber mallet: Angi, "How to Lay Pavers"
  (https://angi.com/articles/10-steps-install-pavers-amidst-your-landscape.htm).
- The trade: O*NET 47-4091.00, Segmental Pavers (https://www.onetonline.org/link/summary/47-4091.00).
- Tools: NVIDIA SimReady assets, CC BY 4.0.
