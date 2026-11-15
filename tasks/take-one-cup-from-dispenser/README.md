# Pull one paper cup out of a wall cup dispenser, rim through the gasket, and stand it upright on the mat

A stainless-steel pull-type cup dispenser hangs on the wall above a counter, holding a short stack of nested 12 oz
paper hot cups. The bottom cup hangs out of the dispenser, held by its rolled rim on a soft silicone gasket. A black
rubber bar mat lies on the counter to its right.

The instruction is: "Pull one paper cup out of the wall dispenser and stand it upright on the black mat."

Taking a paper cup from the dispenser by an office water cooler or coffee station is an everyday task. For a robot it
asks for three things at once. The cup hangs under the dispenser, so it has to be reached from the side, under an
overhang: a straight-down grip would run into the dispenser. A paper cup is light and thin, so it has to be pinched
gently: the gripper's full squeeze would crush it. And only the bottom cup hangs out, held by its rolled rim on the
gasket's flaps with the next cup nested inside it, so it has to be drawn straight down, hard enough for its rim to
push the flaps aside but without catching the rim on the gasket ring; the flaps then spring back and catch the next
cup's rim, so that one stays in. Once out, the cup is carried clear of the dispenser, stood upright on the mat and let
go.

## Scene

Positions in cm from the midpoint between the arm bases (x forward, y left); heights above the counter (the table top).
The layout is fixed.

| Object | Description |
|---|---|
| Wall (fixed) | White painted panel, 2 thick, 80 wide (y -25 to 55), 72 tall, its face at x 44.75 |
| Cup dispenser (fixed) | Stainless-steel pull-type dispenser for 8-24 oz cups (for example a San Jamar C3450SS): a vertical tube 11 across outside and 41 long, capped at the top, held 1.2 off the wall by two steel brackets. Its axis is at x 38, y 14, and its bottom edge is 24.5 above the counter (a cup height and more, so a cup pulled down can slide off the next one above the counter). Its bottom end has a 3 mm lip inside |
| Gasket | Black silicone, in a ring 10.6 across and 1.6 tall resting on the lip inside the tube's bottom end. Its eight flaps reach in to 8.5 across, 2.5 mm inside a cup's rim, and give outward and downward when a rim is pulled down through them, which takes a pull of about 2 N (measured in the sim) |
| Paper cups (4, 11 g each) | Brown kraft 12 oz (355 ml) single-wall paper hot cups: 11 tall, 9 across the rolled rim (a 3.2 mm roll), 6 across the bottom. Nested 1.8 apart, open end up, inside the dispenser. The bottom one hangs by its rim on the gasket's flaps with its bottom 15 above the counter, so about 9 of it sticks out below the dispenser |
| Bar mat (fixed) | Black rubber, 16 (x) by 12 (y), 4 mm thick, centred at x 34, y -3 (right of the dispenser) |

In the sim the gasket is its own part, resting on the lip: a ring with eight sprung flaps, each a rounded tip on a
slider that gives outward and 45 degrees downward (60 N/m). The cups are rigid shells built from flat facets, with a
hexagonal rolled rim and four small stacking bumps inside near the bottom, on which the next cup's bottom edge rests.

## Scoring

The task file follows the trial after every control step:

- **Out of the dispenser:** all of a cup, rim included, is below the dispenser's bottom edge, and it has come off the
  next cup: their axes more than a rim's radius apart, or no overlap in height (a cup pulled only partway is still round
  the next cup's bottom).
- **Still in the dispenser:** a cup hangs upright (within 10 degrees) no more than 5 mm lower than the bottom cup hung
  at the start (its bottom 15 cm above the counter). The next cup drops onto the gasket at exactly that height; a cup
  dragged partly through the gasket hangs lower. On a real rig this is measured at the outlet with a tape measure.
- **Crushed:** a gripper squeezed a cup hard enough to dent or crease it. On a real rig, from the video: the wall
  visibly pushed in. The sim's cups are rigid, so it reads the grip force instead: each gripper's squeeze is its
  fingers' position error times the gripper's stiffness (5,000 N/m, capped at the 50 N effort limit), the smaller of
  the two fingers; over 20 N while the gripper's TCP is within 1 cm of a cup counts as crushing it. For scale, an
  ordinary paper cup's wall gives about 6 mm under 5 N at two-thirds of its height (it is stiffer near the bottom,
  where the demo pinches it), and the YAM's gripper squeezes 50 N when told to close fully.
- **Tipped:** any cup ever tipped more than 60 degrees from upright (dropped or knocked over), judged from the video
  on a real rig. **Fell:** a cup ended more than 5 cm below the counter (on the floor). A crushed, tipped or dropped
  cup would be thrown away.

`score()` gives:

- 0 if no cup is out, if two or more cups are below the dispenser, or if a cup was crushed, tipped or fell;
- 0.25 when exactly one cup is out;
- 0.5 when, also, every other cup is still in the dispenser and the cup that is out stands upright (within 10
  degrees) with its bottom edge on the counter or the mat (within 6 mm), still (moving under 1 cm/s and turning under
  0.5 rad/s; on a real rig, no visible movement for 1 s), and let go (each arm's TCP, the point between its finger
  pads, at least 4 cm from a cylinder as wide as the rim from the cup's bottom to its rim; the finger surfaces reach
  about 2.4 cm from the TCP, so no part of a gripper is within about 1.5 cm of the cup);
- 1.0 when, also, its whole bottom is on the mat (its centre at least 3 cm inside every edge of the mat).

`success()` ends the episode once 1.0 has held for 1 s. On a real rig the end state is checked with a tape measure and
a protractor (or a photo square to the counter), and crushing and tipping from the trial video.

## Run a trial

Following the task board's simulation walkthrough (`source ~/IsaacLab-Arena/yam.env`), from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Pull one paper cup out of the wall dispenser and stand it upright on the black mat." \
  --scorer task_score --no-prompt --max-steps 1200 --store-frames --log-dir ~/yam-eval/cup-dispenser-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The layout is
fixed (`object_jitter=0`), so no seed is needed. A full 1,200-step trial takes about TBD minutes of wall time with
frames stored; the scripted demo ends on success at step TBD.

## Scripted demo

Run it with `--policy yam_scripted` instead of `--policy dream`. The left arm does everything; the right arm parks.

1. With its approach tipped 62 degrees forward (straight down, the gripper would be inside the dispenser), the left
   arm comes in at the hanging cup's height with its fingers either side of it, and pinches the cup 1.9 cm above its
   bottom, near the pad tips (the fingers' roots join across the gap 4.4 cm behind the TCP, so a deeper pinch would
   put the cup's side into them). Both fingers close, centring the cup between them, but only to just inside its
   sides: a squeeze of about 6 N, not fully (50 N would crush it).
2. It pulls the cup 11.6 cm straight down (aiming 0.6 cm further, because the arm gives against the flaps): after
   about 9 mm the rim pushes the flaps aside and snaps through (about 2 N), the next cup drops 1.8 cm onto the flaps
   and is caught by its rim, and the pulled cup slides on down off it until its rim is below the next cup's bottom.
3. It moves the cup out from under the dispenser, over the mat, sets it down and lets go, backs away along its
   approach and parks.

The grip (pitch, pinch height and how far behind the cup's axis the TCP sits) was chosen with `check_clearance.py`,
which poses the YAM's meshes along the demo with the robot's inverse kinematics: the closest approaches are 5.8 mm to
the dispenser, 4.8 mm to the cup on the way in and 11.6 mm to the counter.

Set `CUP_DEBUG=1` to print every cup's position, tilt and rim height, the flaps' slider positions and both TCPs every
5 steps, with each gripper's squeeze force and the largest squeeze on a cup so far. `CUP_SAG="dx,dy,dz"` shifts
every pinch pose to make up for the loaded arm's sag; `CUP_PITCH_DEG`, `CUP_GRIP_UP` and `CUP_GRIP_FORWARD` change
the grip and `CUP_SQUEEZE_N` the pinch's squeeze.

Shortcuts, with `DEMO_VARIANT` set in the environment:

| `DEMO_VARIANT` | What the left arm does | Score |
|---|---|---|
| `full` | as above | TBD (expected 1.0) |
| `nothing` | waits | TBD (expected 0.0) |
| `pull_only` | pulls a cup out and holds it in the air | TBD (expected 0.25) |
| `counter` | stands the cup on the counter left of the dispenser, off the mat | TBD (expected 0.5) |
| `keep_holding` | stands it on the mat but never lets go | TBD (expected 0.25) |

## Offline checks

`test_task.py` checks the geometry (the gasket holds a rim but lets a pulled one through, the flaps stay inside the
tube, nested cups clear each other, the pull brings the whole cup out above the counter), the scoring rules on
hand-built states (including the squeeze force: the demo's 6 N pinch stays under 20 N even if the cup is half a
millimetre wider than modelled, and a full close counts as crushing), the demo's reach with the robot's inverse kinematics, and runs `check_clearance.py`, without Isaac
Sim:

```bash
PYTHONPATH=<path to yam-isaac-lab-arena> python -m pytest test_task.py -q
PYTHONPATH=<path to yam-isaac-lab-arena> python check_clearance.py full
```

## Real-world build

Mount a stainless-steel pull-type cup dispenser for 8-24 oz cups (16 in tube, with its silicone gasket; for example a
San Jamar C3450SS) on a wall
panel or a board clamped upright at the back of the table, its bottom edge 24.5 cm above the table and its axis 38 cm
out and 14 cm left of the arms' midpoint. Load it from the top with a nested stack of four 12 oz single-wall paper
hot cups (90 mm rim, 60 mm base, 110 mm tall), so the bottom one hangs by its rim on the gasket. Lay a 16 x 12 cm
rubber bar mat on the table, centred 34 cm out and 3 cm right of the arms' midpoint. Film the trial. Score the end
state with a tape measure and a protractor, and crushing (a dent or crease in the cup's wall) and tipping from the
video.
It is safe: the arm pulls an 11 g paper cup out of a fixed dispenser.

## Licence

This task is released under the MIT licence. `partkit.py` comes from `tasks/din-rail-mount` in this repository,
extended with sphere and convex-prism colliders, sliders along any direction, parts without a body (fixed scenery)
and a reset that restores the springs' targets; it is released under the same licence. No third-party models or
assets are used. Every object is built from primitive shapes and convex prisms.
