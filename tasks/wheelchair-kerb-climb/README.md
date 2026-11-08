# Tip a wheelchair back to get it up a kerb

Instruction: "Get the wheelchair up onto the grey kerb so that all four of its wheels rest on top, without tipping it over or letting the dummy fall off the seat. Then let go of the chair."

A 1:4 attendant wheelchair, loaded with a weighted dummy, faces a 15 mm kerb (6 cm at full scale). The front casters are too small to roll up it; pushed straight at the kerb they jam and the chair pitches forward. The way up is the one attendants are taught: press the tipping lever to tip the chair back on its rear wheels, roll forward until the casters are over the kerb, lower them onto it, then push the rear wheels up.

## Files

| File | What it holds |
| --- | --- |
| `task.py` | The task file: scene objects, the instruction, `score()`, `success()` and the scripted demo (`demo()`), plus the shortcut variants used in the checks below |
| `layout.py` | Every position on the table |
| `chair.py` | The wheelchair's geometry and its build as one jointed body (USD/PhysX) |
| `scoring.py` | The scorer, pure NumPy |
| `run.py` | Runs Inspect Robots with the wheelchair built from `chair.py` in place of a placeholder box, and the scorer reset at each trial |
| `test_scoring.py` | Scorer checks with no simulator |
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

Use `--policy dream` for Dream-1. The score is printed as `task_score`. With `--log-dir`, `run.py` also writes a per-step trace (`trace-1.jsonl`: score, fault, chair pose, pitch, wheel centres, dummy and both tool centre points).

- **Runtime per trial:** about 3 minutes on one GPU for the demo, which ends on success after 27 s of simulated time; up to about 6 minutes for a trial that runs to the limit.
- **Seeds:** none. The start is fixed and `object_jitter=0`.
- **Scoring checks, no simulator:** `python -m pytest -q test_scoring.py`

## Scoring

Read at the end of the run. A wheel is **up** when its centre is 1 cm or more past the kerb face and it rests on the kerb top (its lowest point within 3 mm of it).

- **0, for good,** if at any time the chair tips back more than 25°, forward more than 15° or sideways more than 12°; if all four wheels are more than 5 mm clear of the road, the kerb top and the kerb edge at once (the chair was lifted); or if the dummy leaves the seat: its centre off the seat's area or more than 4 cm above the seat top, or the dummy tilted 20° on the seat. These faults are latched for the trial; nothing afterwards clears them.
- Otherwise **0** unless both casters are up; **0.25** with both casters up; **0.6** with all four wheels up; **1.0** with all four wheels up, the chair level within 3°, at rest (under 1 cm/s and 0.2 rad/s) and both grippers' tool centre points 5 cm or more from the chair's outline, all held for 1 s. The episode ends 1 s after that.

On a real table this needs a ruler, a protractor and a video: where each wheel stands relative to the kerb face, how far the chair tipped, whether the dummy stayed seated, and whether the hands were clear at the end.

## Scripted demo

One arm does it all; the left arm parks. The right arm presses the orange tipping bar 22 mm down (the chair tips back about 12°), moves forward 7 cm while pressing, which rolls the chair until the casters are over the kerb, and lets the bar up so the casters settle on the kerb. Then it pushes the black push bar forward 15 cm, which rolls the rear wheels up, backs off and moves clear. The demo is open loop: its waypoints come from the chair's known start, and it reads nothing a policy couldn't see.

Both arms working within about 11 cm of the table's centre line at once collide with each other, which is why the demo uses one arm. A policy may use both.

## Scores

- **Dream-1:** 0.0 (1,200 steps). It moved the right arm to the chair and pulled it about 2 cm back, but never tipped it; no fault.
- **Scripted demo:** 1.0, ending on success at 26.8 s.

## Shortcut checks

Each variant is selected with `CHAIR_VARIANT=<name>` before `python run.py ...` (scripted policy):

| Variant | What it does | Score |
| --- | --- | --- |
| `demo` | as above | 1.0 |
| `nothing` | both arms park | 0 |
| `shove` | pushes the push bar straight at the kerb without tipping | 0: the casters jam and the chair pitches forward past 15° (fault at 7.4 s) |
| `casters` | tips and gets the casters up, then stops | 0.25 |
| `overtip` | presses the tipping bar 50 mm instead of 22 | 1.0: the bar bottoms on the road at 13°, so pressing can't tip the chair further; only lifting reaches the 25° limit |

Lifting the chair clear is a latched fault (checked in `test_scoring.py`), as is any dummy fall.

## Licence and assets

The task is released under the MIT licence. There are no third-party assets: the chair, dummy, kerb and road are built from boxes, spheres and cylinders in this folder's code.
