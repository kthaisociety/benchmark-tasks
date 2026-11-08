# Close a valve by hand after its actuator loses power

> The valve actuator has lost power. Close the valve by hand: hold the orange lever down and turn the handwheel until
> the lever stays down by itself, then turn the handwheel clockwise until the indicator's pointer reaches the red
> CLOSED end.

Electric valve actuators drive a valve from a motor. When power is lost, an operator closes the valve with the
handwheel, but in motor mode the handwheel is uncoupled and spins without moving the valve. The operator has to hold
the hand/auto (declutch) lever and turn the wheel until the clutch dogs drop in. The lever then stays in HAND, and
the wheel closes the valve. This task tests that mode change: one arm holds the lever while the other turns the wheel
until the clutch engages, then the turning continues stroke by stroke until the valve is closed.

The scene, the clutch rule and the scoring are in `task.py`. `task.json` lists every object with its size and material.

## Run and score

From this directory, with the program's Isaac Lab-Arena environment active:

```bash
I=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$I" --scorer task_score --max-steps 2400 --no-prompt --store-frames --log-dir logs/demo

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$I" --scorer task_score --max-steps 2400 --no-prompt --store-frames --log-dir logs/dream
```

- **Runtime per trial:** at most 120 s of simulated time (2,400 steps at 20 Hz). The scripted demo ends in 595 steps (29.8 s),
  3 s after the valve closes.
- **Seeds:** none needed. Everything is one fixed assembly, so `object_jitter` has no effect.
- Set `AMO_TRACE=trace.jsonl` to log the wheel angle, paddle position, clutch state, closed fraction and score after
  every step.

Offline checks (no simulator, need `yam_arena` importable):

```bash
python -m pytest -q test_scoring.py   # clutch dogs and every grade boundary
python reach_check.py                 # IK reach of every scripted variant
```

## Scoring as implemented

`score()` reads the clutch state and the wheel's hinge angle:

- **1.0:** clutch engaged and the wheel within 5 degrees of its closed stop (one turn past the engaging position).
- **0.5:** engaged and at least half closed (half a turn).
- **0.25:** engaged.
- **0:** otherwise, including any amount of turning without the clutch engaged.

The trial ends at 1.0 once the closed condition has held for 3 s. Otherwise it scores the state at the last step.

How a person scores a real trial: the paddle stays down with nothing on it (engaged), and the pointer's position on
the 60 mm strip gives the closed fraction.

## Mechanism as implemented

`parts.py` builds the assembly as one articulation: the handwheel is a revolute joint, and the paddle and pointer are
spring sliders. An `on_step` callback runs the actuator's logic after every control step:

- **Motor mode:** the wheel turns freely (0.05 N m s/rad damping) and nothing else moves.
- **Engaging:** if the paddle is at least 90% of its 12 mm travel down when the wheel crosses a dog angle
  (50 + 120·n degrees, either direction), the clutch engages. The paddle's spring target moves below its bottom stop,
  so it stays latched.
- **Hand mode:** valve travel is the wheel's clockwise rotation since engaging, over one turn. Beyond either end the
  wheel's drive switches to a 30 N m/rad spring holding it at the stop. Within the range it is free.
- **Indicator:** the pointer's spring target follows the valve travel.

## Shortcuts checked (scripted, `AMO_VARIANT=<name>`)

| Variant | What it does | Expected | Measured |
|---|---|---|---|
| `demo` | hold paddle through stroke 1, then 6 strokes clockwise | 1.0 | 1.0 (engaged at 50.6° in stroke 1; two reset trials both 1.0) |
| `no_lever` | turn the wheel 5 strokes without touching the paddle | 0 | 0 (wheel turned 449°, never engaged) |
| `tap_lever` | press and release the paddle, then turn | 0 | 0 (paddle sprang back; wheel turned 449°, never engaged) |
| `engage_only` | hold paddle, one stroke | 0.25 | 0.25 (engaged at 50.6°, 4.6% closed) |
| `half` | hold paddle, three strokes | 0.5 | 0.5 (60.6% closed) |
| `anticlockwise` | hold paddle, turn anticlockwise (engages at −70°, then the open stop) | 0.25 | 0.25 (engaged at −71.4°, held at the open stop, 0% closed) |

## Scripted demo

The left arm presses the paddle with closed fingers and holds it for 5 s, then lifts off and parks. The right arm turns
the wheel the way #958 turns its star handle. Closed fingers go down 45 mm from the axis between two spokes and sweep
clockwise from 255 to 110 degrees in 14 short moves, pushing the spoke ahead. Then they lift clear and go back for the
next spoke. Fixed waypoints only.

## Licences and credits

- The task (`task.py`, `task.json`, `README.md`, `test_scoring.py`, `reach_check.py`) is under the MIT licence.
- `parts.py` (rigid multi-shape parts with hinges and sliders) is copied unchanged from the star-handle gate-valve
  task submitted in robocurve/benchmark-tasks#958, whose docstring says to copy it next to the task file.
- No third-party models or textures are used. Every shape is a box or cylinder defined in `task.py`.
