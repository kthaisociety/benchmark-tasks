# Size a blood-pressure cuff to a child's arm

A child-size training arm (a fixed 5 x 5 cm beam, 20 cm round) lies across the table: a lighter upper arm with its
shoulder end on the robot's left, a darker forearm and hand stub on its right. Three blue cuffs stand in a row in
front of it. A cuff is a rigid arch: two legs, a bridge over the arm and a square grip knob on top. The instruction is
just "Put a blood-pressure cuff on the training arm." It first said "the child's arm", which Dream-1's safety filter
refused as force applied to a child, so the child framing is in `context`.

All three cuffs fit over the arm. They differ only in width along the arm: 4, 8 and 12 cm. A blood-pressure cuff's
width should be about 40% of the arm's circumference (here 8 cm for a 20 cm arm); a narrower cuff reads falsely high
and a wider one falsely low. Nothing in the scene says so, so the choice cannot be found by trying cuffs on: each one
goes on. The 8 cm cuff is in the middle of the row, so taking the nearest, the farthest, the widest or the narrowest
never scores.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `score()` and the scripted `demo()` |
| `cuff_scoring.py` | Geometry, scoring and the demo plan in plain Python, so they are tested without the simulator |
| `test_task.py` | Offline tests (no Isaac Sim, yam_arena or GPU) |
| `partkit.py` | Rigid parts built from boxes for yam_arena task files, copied unchanged from `tasks/din-rail-mount` |

## Setup

World frame in metres: x forward from the arm bases, y left; heights are above the table. The layout is fixed.

| Object | Where | Size |
|---|---|---|
| Upper arm (fixed) | x 0.34, y -0.10 to +0.08 | 5 x 18 x 5 cm, light skin tone |
| Elbow crease | y -0.10 | 6 mm dark band |
| Forearm and hand stub (fixed) | x 0.34, y -0.20 to -0.10 | 5 x 10 x 5 cm, darker |
| `cuff_12` | x 0.17, y +0.10 | 12 cm wide, 100 g |
| `cuff_8` | x 0.17, y -0.03 | 8 cm wide, 70 g |
| `cuff_4` | x 0.17, y -0.15 | 4 cm wide, 40 g |

Each cuff is one rigid body: two 6 mm legs 6 cm tall with 10 cm between them (2.5 cm clear of the arm's sides), a 6 mm
bridge (1 cm above the arm's top), and a 2.5 x 2.5 x 5 cm grip knob on the bridge. It stands on the table on its
legs, its width running along y like the arm's. Nothing starts where the parked arms hide it from the top camera
(x < 0.30 with |y| > 0.20), and the full-score spot is in reach of either arm.

## Scoring

Evaluated every control step; the score at the end counts (`SCORE_OVER = "end"`). The right size is computed, not
named: the cuff whose width is 37 to 50% of the arm's circumference (`CIRCUMFERENCE` = 4 x 5 cm), which is only the
8 cm cuff (40%; the others are 20% and 60%).

| Final state | Score |
|---|---|
| The 8 cm cuff is on the upper arm: standing on the table, legs either side of the arm, wholly along the arm, lower edge at least 2 cm above the crease, centre within 2.5 cm of the arm's axis (its legs clear of the arm), tilted at most 8 degrees, turned at most 12 degrees from square; and no other cuff over the arm | 1.0 |
| It is over the arm elsewhere (forearm, crease) or askew, or another cuff is over the arm too | 0.5 |
| It is over the arm but not standing on the table: the arm's axis between its legs (centre within 5 cm of it) and between the arm's ends, tilted at most 30 degrees (held over it, perched on it, a leg on the arm) | 0.25 |
| Otherwise, including any other cuff over the arm on its own | 0 |

There is no `success()`: the run always goes to the step limit and the final state counts, so a cuff knocked off after
it was placed loses the credit. Credit follows the final state only, so a wrong cuff tried and taken off costs
nothing, and a wrong cuff left on the arm earns nothing. The cuffs start 17 cm in front of the arm's axis, so doing
nothing or setting a cuff down in front of the arm scores 0. Every limit is a tape-measure reading on the final photo;
the ones that depend on simulation (tilt, turn) have a margin of several degrees.

## Checks

From the repository root:

```bash
python3 -m unittest discover -v -s tasks/bp-cuff-size -p test_task.py
```

The tests cover the credit scheme (including a scan showing only the 8 cm cuff can ever reach 1.0), the geometry
limits (reach, clearances, spacing, the parked arms' blind spot) and the demo plan.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/bp-cuff-size/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put a blood-pressure cuff on the training arm." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/bp-cuff-dream
```

## Scripted demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/bp-cuff-size/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put a blood-pressure cuff on the training arm." \
  --scorer task_score --max-steps 440 --store-frames --log-dir ~/yam-eval/bp-cuff-demo
```

The right arm grips the 8 cm cuff by its knob, lifts it, carries it over the middle of the upper arm, lowers it until
its legs are just above the arm, steadies, sets it down astride the arm, lets go, withdraws and parks clear of the
top camera (21 s, 420 steps; moving slowly keeps the hanging cuff from swinging).

`CUFF_DEMO` selects a shortcut instead, each of which must score what the tests expect:

| `CUFF_DEMO` | What the arm does | Score |
|---|---|---|
| `correct` (default) | 8 cm cuff onto the middle of the upper arm | 1.0 |
| `narrow` / `wide` | the 4 cm / 12 cm cuff onto the same spot | 0 |
| `elbow` | the 8 cm cuff onto the forearm | 0.5 |
| `beside` | the 8 cm cuff set down in front of the arm, not over it | 0 |
| `hold` | the 8 cm cuff carried over the arm and held there | 0.25 |
| `nothing` | waits | 0 |

## Runtime and seeds

Fixed layout (`-E object_jitter=0`), no seeds. About 1 minute per trial for the full 1,200 steps on an L40S plus Isaac
Sim start-up; the demo needs about 300 steps.

## Fidelity

Rigid bodies built from boxes; the arm is fixed, the cuffs are loose with friction 1.0. A real cuff is a fabric wrap
with an inflatable bladder, which the simulator cannot do, so each cuff is a rigid arch that stands over the arm.
Only the cuff's width is modelled, not its length (a real bladder should wrap 80 to 100% of the circumference), and
the cuffs carry no printed size range or index line. The arm is a square beam, 20 cm round by tape. Nothing is
inflated and no reading is taken. No contact-force cap is enforced in simulation.

## Real-world build

A 5 x 5 cm wooden or PVC beam, 28 cm long, marked 18 cm / 10 cm with tape at the crease. Three arches cut from 6 mm
PLA or plywood to the sizes above (a 12, 8 and 4 cm wide strip folded or glued into a U 10 cm inside, 6 cm tall), each
with a 2.5 x 2.5 x 5 cm block glued on top. Tape marks for the starting positions. Safe to run: nothing moves except
light plastic parts.

## Licences

This task is MIT licensed. `partkit.py` is copied unchanged from `tasks/din-rail-mount/partkit.py`. No third-party
assets are used; every object is a primitive box.
