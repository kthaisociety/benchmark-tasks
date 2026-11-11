# Set a power rack's safety arms for a lifter's bench press

A scaled power rack stands on the table with a lifter figure lying on its bench, head toward the robot, under a bar
resting in the J-hooks. Each upright carries a red safety arm that slides up and down and clicks into one of eight
notches, 1.5 cm apart. For a bench press, both arms belong at the notch just below where the bar touches the lifter's
chest: a failed rep then lands on the arms, not on the lifter, and a full rep is never stopped. The left arm starts at
the top notch (too high) and the right arm at the bottom one (too low), so one has to come down and the other up.

The instruction doesn't say where the arms go. The task tests fitting a safety stop to a person's body from what the
robot sees: the chest's height under the bar, against the notches on the uprights.

**Instruction:** Set the rack's safety arms for this bench press.

## Files

- `task.py`: the scene (`OBJECTS`: one partkit part, the rack with the bar and the figure, with the two safety arms on
  vertical sliders), the per-step notch detent, `score()` and `demo()`.
- `safety.py`: the notch detent, the trial record and scoring, and the demo plans. It is pure Python, and includes a
  one-dimensional model of the arms and the gripper for offline checks.
- `test_task.py`: 19 offline tests (`python3 -m unittest test_task` in this folder).
- `partkit.py`: rigid parts with sliders, from `tasks/din-rail-mount`.

## Run a trial

```bash
source /data/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_arena -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 600 --policy <policy> \
    --instruction "Set the rack's safety arms for this bench press."
```

## The safety arms

Heights are the top surface of an arm, above the table. The notches are at 3.0, 4.5, 6.0, 7.5, 9.0, 10.5, 12.0 and
13.5 cm, marked in white on the uprights' inner faces. The bar's underside touches the lifter's chest at 8.0 cm.

Every control step, `task.py` reads each arm's slider and runs a pop pin (`Detent` in `safety.py`). A locked arm is
held at its notch by a 150 N/m drive (plus a 3.3 mm sag offset for its 50 g), so a brush or a light drag moves it less
than 6 mm and it springs back. Pulled more than 6 mm (about 0.9 N), it unlocks and follows wherever it is moved; once it
has been still for 0.5 s, it locks into the nearest notch.

## Scripted demo

Run the command above with `--policy yam_scripted`. Each arm, the left one 0.5 s ahead of the right:

1. comes over its safety arm's handle (at x 18.25 cm, y +/-13.5 cm, on the arm's near end, 13 cm from the bar), with the
   fingers open 50 mm and turned to close along y;
2. lowers so the pad tips sit 7 mm above the arm's top, and closes to 8 mm narrower than the 2 cm handle;
3. moves the safety arm straight down (left, from 13.5 cm) or up (right, from 3.0 cm) to 7.5 cm, at 5 cm/s;
4. holds 0.6 s, opens, lifts clear and parks out of the top camera.

The plan takes about 11 s; the trial runs to the 600-step limit and is scored at the end.

Shortcut variants: set `SAFETY_DEMO=<mode>` with the scripted policy. The expected scores come from the offline model.

| Mode | Left arm | Right arm | Expected score (offline and in the simulator) |
|---|---|---|---|
| `correct` (default) | 7.5 | 7.5 | 1.0 |
| `nothing` | 13.5 (left at the top) | 3.0 (left at the bottom) | 0 |
| `right_only` | 13.5 | 7.5 | 0.5 |
| `both_top` | 13.5 | 13.5 | 0.25 |
| `both_low` | 3.0 | 3.0 | 0 |
| `one_low` | 6.0 | 6.0 | 0 |
| `one_high` | 9.0 | 9.0 | 0.25 |
| `uneven` | 7.5 | 9.0 | 0.5 |
| `held_between` | let go at 8.0 | let go at 8.0 | 1.0 (each locks into 7.5) |

## Scoring (`score()` at the end)

- **0 if:**
  - the rack moved more than 1 cm or turned more than 3° (latched for the trial); or
  - either arm ends below 7.0 cm: a failed rep would come down on the chest before the arm caught it.
- **1.0** if both arms are between 7.0 and 8.0 cm: the 7.5 cm notch, just under the chest contact.
- **0.5** if one arm is there and the other at 8.0 cm or higher.
- **0.25** if both are at 8.0 cm or higher: safe, but the lifter can't complete a rep.

`test_every_pair_of_notches` checks all 64 end states: only one of them scores 1.0, and every one is covered.

## Runtime and seeds

- Run limit: 600 steps (30 s at 20 Hz). The demo needs about 11 s.
- The layout is fixed (`object_jitter=0`); there are no seeds.

## Fidelity

- The rack, bar and figure are one rigid body, at about one fifth scale.
- The safety arms are slides with a notch detent instead of pop pins.
- The lifter doesn't move and the bar isn't lifted: the score checks where the arms would catch it.

## Real-world build

- **Rack:** any power rack with a bench, or a scaled model with two arms on vertical rails and spring detents.
- **Scoring:** a tape measure from the floor (or table) to each arm's top and to the bar's underside resting on a
  lying person's chest, or the notch number against a photo.
- **Safety:** nothing is lifted; the bar stays in the J-hooks, and a mannequin or a person lies under it.

## Checks

Offline:
- the novelty gate: no existing task sets a safety stop to fit a person's body; the closest are other gym tasks
  (stripping plates, collars, spotting);
- feasibility on paper: grip a 2 cm handle on a slide and move it straight up or down, both arms on separate objects,
  no tilt, pour, handover, tight fit or precise force;
- 19 tests.

Probes in the simulator (scripted, about 260 steps):
1. With the handles 7 cm from the bar, the left gripper's housing ran into the rack, and after letting go the fingers
   dragged a free-following arm back up. Moving the handles 13 cm from the bar and adding the pop pin (locked until a
   6 mm pull) fixed both: each arm moved where the gripper took it, locked into the nearest notch, and stayed there as
   the fingers left.
2. The fingers close on the 2 cm handle at 3.0 and 13.5 cm and carry the arms 4.5 and 6 cm without slipping.
3. In the 320 x 176 top camera the red arms stand out against the figure, and their heights show in perspective
   (setup.png).

Build in the simulator: every variant scores as expected over the full 600 steps, and every arm ends seated on a
notch with the rack unmoved (`held_between`, let go 5 mm above the notch, locks into 7.5 cm on both sides). The demo
scores 1.0 on two trials in a row, so the reset works.

Dream-1: the 40-step smoke test ran with the instruction unchanged, and the one full run (600 steps) scored 0.0. Both
arms moved over the rack but never took a handle, so the safety arms stayed at their start notches (13.5 and 3.0 cm).

## Licence

- The task (`task.py`, `safety.py`, `test_task.py`, `task.json`, `README.md`) is released under the MIT licence.
- `partkit.py` comes from `tasks/din-rail-mount` in this repository.
- No third-party models or assets are used.
