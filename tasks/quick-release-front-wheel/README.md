# Refit a quick-release front wheel so the bike is ready to ride

Instruction: "Put the front wheel back in the bike's fork so that the bike is ready to ride."

A bicycle's front fork stands upside down in a workshop stand, the way a mechanic flips a bike to refit a wheel, so
its dropout slots open upward. Next to it, a 20-inch front wheel with a quick-release (QR) skewer stands in a wheel
stand with its lever fully open. Scale 1:4. The wheel is only ready to ride when both axle ends are fully down in
their dropout slots and the lever is swung fully closed, lying along the fork. A lever left part-way open, or closed
while an axle end rests on the dropout's edge, lets the wheel come out. A lever closed pointing forward isn't how
manuals show it. The instruction doesn't say any of this: knowing what "ready to ride" means is part of the task.

What the sources say (read 2026-11-11):

- Dahon bicycle owner's manual, front wheel, p. 48: the axle "seats firmly at the top of the slots"; "The lever should
  now be parallel to the fork blade and curved toward the wheel"; with the right force "the lever should make a clear
  embossed mark on the surface of the fork".
- Park Tool, *Wheel Removal & Installation*: "Make sure the hub is fully seated in the frame or fork"; "Verify that
  wheel is centered in frame or fork"; "Place front lever just in front of fork blade when possible".
- Sheldon Brown, *Quick Releases*: the front lever is "Usually ... on the left side"; closing needs "enough force to
  leave an imprint on the palm of your hand".

The sources differ on which side the lever goes, so its side isn't scored: the wheel may go in either way round.

## Files

| File | What it holds |
| --- | --- |
| `task.py` | The task file: objects, instruction, `score()`, the lever's mechanism (applied every step) and the scripted demo with its shortcut variants |
| `layout.py` | Every size and position |
| `bike.py` | The wheel (a jointed body: axle, free-spinning rim, skewer, lever) and the fork, built from boxes and cylinders |
| `mechanism.py` | The lever's detents and friction, as spring targets for its two joints |
| `scoring.py` | The scorer, pure NumPy |
| `parts.py` | Jointed multi-shape parts for yam_arena (see Licences) |
| `run.py` | Runs Inspect Robots with a per-step trace in the log directory |
| `test_scoring.py` | Scorer and mechanism checks with no simulator |
| `reach_check.py` | Replays every variant's waypoints through the arm's IK with no simulator |
| `export_media.py` | Turns a recorded trial into the review video (H.264) and the setup screenshot |

## Run a trial

From this folder, with the program's Isaac Lab-Arena environment active:

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
python run.py run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --scorer task_score --max-steps 1200 --seed 0 --no-prompt \
  --store-frames --log-dir runs/demo
```

Use `--policy dream` for Dream-1. The score is printed as `task_score`. `run.py` is optional; plain
`inspect-robots run ...` works the same, just without `trace-<n>.jsonl`.

- **Runtime per trial:** about 3 minutes on one GPU for the scripted demo (24 s of simulated time, including
  Isaac Sim start-up); a trial that runs to the 1,200-step limit takes longer.
- **Seeds:** none. The start is fixed and `object_jitter=0`.
- **Checks with no simulator:** `python -m pytest -q test_scoring.py` and `python reach_check.py`.

## Scoring

Read at the end of the run (`SCORE_OVER = "end"`). Where the axle's centre line crosses each dropout, an axle end is
**seated** when it is within 3 mm of fully down in the slot (against the slot's end) and within 6 mm of the slot's
centre line. The slot gives it 4 mm of play either way. The lever is **closed** at 80° or more of its 90° swing.
Past 60° it snaps closed. Where a closed lever points is measured in the fork's plane, from straight down the
blade.

- **0 for good** if the axle moves 5 mm or more while the lever stays closed. Example: closing the lever first and
  then fitting the wheel, which a real closed QR wouldn't let through the dropouts. Pulling a closed wheel out of
  the fork counts too. This fault is latched for the trial; nothing afterwards clears it.
- **1.0**: both ends seated, lever closed, pointing within 30° of the blade, or turned toward the bike's rear.
- **0.75**: both ends seated, lever closed but pointing more than 30° toward the bike's front (the white reflector
  on the crown faces the front).
- **0.5**: both ends seated, lever not closed (open or part-way).
- **0.2**: lever not closed, and each axle end either seated or in its dropout's mouth (centre within 1.6 cm of the
  slot line and at most 1.3 cm above the mouth's top). The wheel is in the fork but not fully down.
- **0**: the lever closed while an axle end isn't seated, or the wheel not in both dropouts.

There is no early success stop: every run goes to its step limit and is scored at the end, so a run can finish by
moving the arm clear. Which side the lever is on is never scored. On a real bench this needs a ruler at each dropout and a look at the
lever, or a photo.

## The lever's mechanism

A real QR clamps with a cam. Here (`mechanism.py`, applied after every control step) the lever springs back fully
open below 15° of closure, snaps closed past 60° (over centre), and in between stays where it is left. That
in-between hold is friction: the joint spring's rest point trails the lever by 6°. The skewer turns about the axle
the same way while the lever is open, and is clamped where it was once the lever is closed. The closed lever stays
closed when let go.

## Scripted demo

Open loop. Its waypoints come from where the wheel stand, the fork's dropouts and the lever are; it reads nothing a
policy couldn't see. The right arm does everything and the left arm parks. Two arms working this close together
collide.

The right arm grips the tyre's top, fingers closing across it, and lifts the wheel out of its stand. It carries the
wheel 17 cm along its own plane with the axle 12 mm above the dropouts' mouths and lowers it until the axle is 6 mm
above its seat. Then it lets go: the axle drops down the flared mouths to the slot ends. With closed fingers it
presses the lever's paddle down and round its arc. In the measured run the lever passed 47° and snapped closed
(90°, pointing straight down the blade) at about 17 s. The axle ends sat at 84.0 mm, their seated height, within 3
mm of the slot centre lines. The arm then backs off and parks, and the run ends at 24.2 s with score 1.0.

## Shortcut checks

Select a variant with `QR_VARIANT=<name>` before the run command (scripted policy). Measured on the simulator:

| Variant | What it does | Measured |
| --- | --- | --- |
| `demo` | as above | 1.0 |
| `nothing` | both arms park | 0 (wheel untouched in its stand) |
| `open` | fits the wheel, leaves the lever open | 0.5 (both axle ends seated at 84.0 mm, lever at 2°) |
| `half_open` | fits the wheel, presses the lever only part-way and lets go | 0.5 (seated, lever left part-way) |
| `one_side` | fits the wheel turned 17° and 2 cm off, lever open | 0 (the far axle end misses its dropout and drops beside it; the near end hangs in its mouth 9 mm above its seat) |

Some cases can't be set up cleanly by a scripted arm, so `test_scoring.py` checks them on states from the wheel's
own kinematics instead:

- **Lever closed, then the wheel fitted:** latched 0 (`test_closing_then_fitting_is_a_latched_fault`). Pressing the
  lever while the wheel stands in its stand tips the wheel, since only the tyre holds it there.
- **Lever closed on an axle that isn't seated:** 0 (`test_not_fully_down_is_not_seated`,
  `test_cocked_one_side_seated_is_point_two_closed_is_zero`). A scripted press on the cocked wheel missed the lever.
- **Lever closed pointing forward:** 0.75 (`test_lever_pointing_forward_is_three_quarters_rearward_is_full`). A
  scripted sideways sweep turned the skewer only 6-12°, and the lever closed within 30° of the blade, scoring 1.0.

## Scores

- **Dream-1:** 0.0 (1,200 steps). The right arm went to the wheel in its stand and hovered over and beside it for
  most of the run without lifting it; the left arm stayed back. The wheel never moved.
- **Scripted demo:** 1.0 (ends at 24.2 s; also 1.0 in both trials of a two-trial reset check)

## Licences and credits

- The task (`task.py`, `layout.py`, `bike.py`, `mechanism.py`, `scoring.py`, `run.py`, `test_scoring.py`,
  `reach_check.py`, `export_media.py`, `README.md`, `task.json`) is under the MIT licence.
- `parts.py` (rigid multi-shape parts with hinges and sliders) is copied from the star-handle gate-valve task
  submitted in robocurve/benchmark-tasks#958, whose docstring says to copy it next to the task file. Three small
  additions are marked "(added)": turned boxes, a hinge hanging on another hinge, and per-part friction combine mode
  and self-collision.
- No third-party models or textures. Every shape is a box or cylinder defined in `bike.py` and `task.py`.
