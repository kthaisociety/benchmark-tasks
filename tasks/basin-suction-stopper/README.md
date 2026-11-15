# Pull a suction stopper out of a basin's drain from under the faucet's spout and put it in the soap dish

A round white vessel basin stands on the counter in front of the arms. A flat grey suction stopper is pressed down
over the grid drain in the middle of its floor. A chrome vessel faucet stands behind the basin, and its square spout
reaches over the back of the stopper. A soap dish sits on the counter to the right of the basin.

The instruction is: "Take the stopper out of the basin's drain and put it in the soap dish."

Taking the plug out of a basin, to let the water out or to clean the drain, is something almost everyone does. A basin
with a grid drain (common on vessel basins) has no plug of its own, so it is closed with a flat suction stopper.
Pressed down, the stopper sticks to the glaze, and it takes a firm pull, about 10 N, to come out. People with weak
hands often find that hard. The robot has to:

- get its fingers onto a 2.4 cm knob at the bottom of a 12.5 cm-deep bowl, under a spout whose end hangs 18 cm above
  the knob, over the back edge of the stopper: a hand coming straight down onto the knob runs into it;
- grip the knob hard enough and pull hard enough to break the seal (a grip that only carries the stopper's 45 g
  slides off), and keep hold when the seal lets go suddenly;
- lift the stopper out over the rim, carry it to the soap dish and let go.

## Scene

Positions in cm from the midpoint between the arm bases, on the counter (x forward, y left). The layout is fixed.

| Object | Description |
|---|---|
| Basin (fixed) | Round white glazed-ceramic vessel basin, 40.6 across (16 in), 14 tall (5.5 in), 1.2 walls, straight sides, a flat inner floor 1.5 above the counter; centred at x 25, y 0. A 4.4 (1 3/4 in) chrome grid drain, which has no plug, in the middle of the floor; no overflow |
| Stopper (45 g) | Grey silicone flat suction stopper: a disc 7.5 across and 0.6 thick with a shallow suction cup underneath, and a knob 2.4 across and 2.2 tall in its middle. Pressed down, centred over the drain |
| Faucet (fixed) | Chrome single-lever vessel faucet (a tall basin mixer) behind the basin: a 5.5 x 5.5 x 1 base plate and a 3.5 x 3.5 square column 30 tall at x 49, y 0 (the plate 1 cm clear of the basin), a lever on top. A 2.2 x 2.2 square spout runs from the column toward the arms, reaching 20 from the column's axis; its outlet is 22 above the counter at x 29 (4 behind the drain, 8 above the rim), so the spout's end is over the back edge of the stopper |
| Soap dish (fixed) | Blue-grey ceramic, 14 x 11 outside and 2 tall, walls 0.6, floor 0.5; centred at x 22, y -30, long side along x; fixed with putty |

## Scoring

The task file follows the trial after every control step and `score()` gives the state at the end:

- **0:** the stopper is still sealed on the drain.
- **0.25:** the seal has been broken, but the stopper is still in the basin (or has fallen off the counter). The seal
  breaks once some point of the stopper's rim comes 3 mm or more off the basin floor (lifted or tipped), or its
  centre moves 15.5 mm or more off the drain's (slid off, so its rim reaches the drain opening). A broken seal stays
  broken: setting the stopper back on the drain doesn't reseal it.
- **0.5:** the stopper is out of the basin and on the counter or held, but not in the soap dish.
- **0.75:** the stopper lies in the soap dish, but a gripper's fingertip centre is within 6 cm of its middle, or it
  is still moving.
- **1.0:** the stopper lies in the soap dish, both grippers' fingertips are 6 cm or more from its middle, and it has
  moved under 2 mm in the last 0.5 s. `success()` ends the episode once that has held for 2 s.

"In the basin" means the stopper's middle is within 19.6 cm of the basin's centre (inside its wall) and under 16 cm
above the counter. "In the soap dish" means its middle is inside the dish's inner walls and under 4.5 cm above the
counter, in any orientation. Anywhere else after the seal is broken (on the counter, on the rim or the faucet, held
in the air) is 0.5. In a real trial, a stopper still on the drain at the end counts as broken if a luggage scale
lifts it with under 2 N, or if the trial video shows its rim 3 mm up or the stopper 15.5 mm off the drain at any
point. A real stopper pressed back down can reseal, but that doesn't take the credit away. The rest is a look and a
tape measure.

## Run a trial

Following the task board's simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Take the stopper out of the basin's drain and put it in the soap dish." \
  --scorer task_score --no-prompt --max-steps 2000 --store-frames --log-dir ~/yam-eval/basin-stopper-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The layout is
fixed (`object_jitter=0`), so no seed is needed. A full 2,000-step Dream-1 trial took about 11 minutes of wall time
on an L40S with frames stored (including the model server's round trips); the scripted demo ends on success after
about 23 s of simulated time (454 control steps) and took about 3 minutes.

## Scripted demo

Run it with `--policy yam_scripted` instead of `--policy dream`. Only the right arm moves:

- Its grip is leaned 35 degrees back toward the arms (the fingertips ahead of the wrist), so the wrist stays clear of
  the spout. It comes in over the front of the basin, slides down the leaned axis until the open fingers straddle the
  knob, 2 mm below its top, and closes.
- It draws back 4.5 cm up the leaned axis over 3.5 s. The knob is held fast, so the arm's pull builds (the arm is
  springy, about 760 N/m along that axis) until the seal lets go. In the sim it holds for the first 0.9 s of the
  draw, then the pull (up and toward the arms) slides the stopper toward the arms with its far edge rising, and the
  seal breaks 1.7 s into the draw, when the stopper's centre is 15.9 mm off the drain's, with its rim 2 mm up. The
  stopper then pops about 2 cm up in the grip.
- It lifts the stopper over the rim, carries it to the soap dish 20 cm above the counter, lowers it to 14 mm above
  the dish floor, lets go, rises and goes back to its start pose.

Set `SINK_DEBUG=1` to print the stopper's pose, its rim lift and offset from the drain, and the score terms every 4
control steps. `F_SEAL` (N, default 10) sets the seal's hold. Demo-only knobs: `PULL_TILT` (degrees, 35) the grip's
lean, `PULL_MM` (45) how far it draws back, `GRASP_DEPTH` (mm, 2) how far below the knob's top the TCP sits.

Shortcuts, with `DEMO_VARIANT` set in the environment:

| `DEMO_VARIANT` | What the arm does | Score |
|---|---|---|
| `full` | as above | 1.0 (the seal breaks at step 164, 8.2 s; score 1.0 from step 376; the episode ends on success at step 454) |
| `nothing` | waits | 0.0 (the stopper doesn't move: rim and centre within 0.1 mm all run) |
| `weak_pull` | grips the knob and draws back only 1 cm (about 8 N along the leaned axis, 6 N of it upward), then lets go | 0.0 (the rim rose 0.7 mm and the centre moved 3.3 mm; the seal held) |
| `drop_back` | breaks the seal, lifts the stopper 6 cm and drops it back in the basin | 0.25 |
| `counter` | puts the stopper on the counter beside the soap dish | 0.5 |
| `keep_holding` | lowers the stopper into the soap dish but never lets go | 0.75 |
| `straight` | as `full` with a straight-down grip (a probe of the spout's hindrance, not a scoring case) | 0.0: the wrist runs into the spout and the fingertips stop about 16 cm above the knob |

## How the suction is modelled

PhysX has no suction, and the stopper is a rigid body. While it is sealed, the task file applies a constant 10 N force
pulling its centre straight down onto the basin floor (from the first physics step). So it comes up only when it is
pulled with more than 10 N plus its weight. Tipping it needs a turning moment of 10.4 N times its 3.75 cm radius, and
pushing it sideways slides it once the push beats its friction on the glaze (about 8 N). After each control step, the
task file checks the stopper's pose. Once any point of its rim is 3 mm or more off the floor, or its centre 15.5 mm or
more off the drain's (where the rim reaches the drain opening and air gets in), the seal is broken for good. The force
is then off and the stopper is an ordinary loose body. The real silicone is flexible, so a person can also peel its
edge up by a tab or a fingernail with less force. A rigid stopper can't be peeled, so that cheaper route is missing
here.

The basin is empty, so a real trial stays dry (no water on the wrist camera or gripper). The water a full basin
holds would add about 1 N to the hold (6 cm of water over the 4.4 cm drain), which is inside the calibration band
below.

The 10 N is an estimate, not a measurement: over an open drain the suction holds only across the cup's rim, and we
expect a wet, freshly pressed universal stopper to take somewhere around 5 to 15 N to pull straight off. The real
build pins it at 8 to 12 N with the reset check below, and `F_SEAL` changes it in the sim.

## Offline checks

`test_task.py` runs without Isaac Sim. It checks the scene geometry, the seal's break conditions and force balance,
the scoring rule and the stopper's position findings. It checks the demo plans' IK reach (under 3 mm error on every
variant) and the arms' collision hulls against the basin, faucet, soap dish and counter along every demo path (1 cm or
more of clearance). It checks that a straight-down grasp runs into the spout (the hulls overlap it) while the leaned
one clears it by 3 cm or more. It also checks the pull budget: the arm's joint stiffness gives well over 10.4 N at
the full draw (about 35 N), and under it at 1 cm, with joint torques under half the motors' limits.

```bash
PYTHONPATH=<path to yam-isaac-lab-arena> python -m pytest test_task.py -q
```

## Real-world build

Use a 16 in round straight-sided vessel basin with a 1 3/4 in grid drain (no overflow), set on the table over a 44 mm
hole drilled through it (or cut in a table insert). The drain body goes down through the hole to a standard tailpiece
and a P-trap below the table, kept filled with water: what is under the drain changes the stopper's hold. Fit a
single-lever vessel faucet (a tall basin mixer) with a 20 cm spout reach, so its outlet lands about 4 cm behind the
drain and 22 cm above the counter; a spacer block under its base trims the height. Use a universal flat suction
stopper about 7.5 cm across with a centre knob, and a 14 x 11 cm soap dish stuck down with putty. Before each trial,
wipe the drain area and the stopper's underside with a damp cloth and press the stopper down centred over the drain
for 2 s. Check that a luggage scale on a cord loop round the knob's base reads 8 to 12 N when it lets go. If it reads
lower, wipe it wetter and press harder; if higher, press more lightly; if it stays outside the band, replace the
stopper. Then press it down again. Score by eye and with a tape measure.

The trial is dry and safe to run: there is no water in the basin, the pull is about 10 N, and the faucet is off.

## Licence

This task is released under the MIT licence. `partkit.py` is this team's part library (rigid parts built from
several boxes and cylinders), released under the same licence. No third-party models or assets are used: every
object is built from primitive shapes.
