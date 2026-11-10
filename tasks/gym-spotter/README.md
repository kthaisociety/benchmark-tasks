# Spot a lifter through a set of bench presses

A scaled smith machine stands on the table with a lifter figure lying on its bench, head toward the robot. The bar
slides vertically on the machine's uprights over the lifter's chest, with the lifter's hands and forearms on it. A
scripted lifter presses a set: three easy reps, a slower fourth, a grinding fifth that crawls through a sticking point,
and a sixth that stalls 3.5 cm off the chest, shakes for 2.5 s and then sinks back down onto the chest.

The robot is the spotter. It should keep its hands off the bar while the lifter is moving it, and take hold when the
sixth rep stalls. Then it helps the bar up to lockout, where the lifter racks it. The instruction says only "spot". Nothing
says which rep will fail, but the slowdown is visible. The task tests timing an intervention on a moving object
from what it sees: too early and the robot fights the lifter, too late and the bar is back on the chest.

**Instruction:** Spot the bar through this set of bench presses.

## Files

- `task.py`: the scene (`OBJECTS`: one partkit part, the machine and the figure, with the bar on a vertical
  slider), the per-step lifter drive, `score()`, `success()` and `demo()`.
- `spotter.py`: the scripted lifter, the trial record and scoring, and the demo plans. It is pure Python, and includes a
  one-dimensional model of the bar and gripper for offline checks.
- `test_task.py`: 19 offline tests (`python3 -m unittest test_task` in this folder).
- `partkit.py`: rigid parts with sliders, from `tasks/din-rail-mount`.

## Run a trial

```bash
source /data/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_arena -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 700 --policy <policy> \
    --instruction "Spot the bar through this set of bench presses."
```

## The lifter

Every control step, `task.py` reads the bar's slider position and sets the slider's drive target to where the lifter
wants the bar. The drive is 500 N/m with 30 N·s/m damping and a 4.9 mm sag offset for the bar's 0.25 kg. The set is
fixed:

| Time (s) | Phase |
|---|---|
| 0–1.0 | holds the bar at lockout |
| 1.0–8.35 | reps 1–3: down 1.0 s, touch 0.15 s, up 0.9 s, 0.6 s at the top |
| 8.95–11.4 | rep 4: up in 1.3 s, then 1.5 s at the top |
| 12.9–17.05 | rep 5: up in 3.0 s, crawling from 40% to 55% of the travel for 1.8 s; 1.5 s at the top |
| 18.55–19.7 | rep 6 down |
| 19.7–21.3 | rep 6 up to 3.5 cm, slowly |
| 21.3–23.8 | stalled at 3.5 cm, shaking ±2 mm |
| 23.8–25.8 | sinks back to the chest; pinned from then on |

Two rules let the spotter help:

- **Help is followed.** In reps 5 and 6, a bar lifted more than 5 mm above where the lifter has it counts as helped.
  From then on in that rep the lifter's target stays 1 mm under the bar, so the spotter carries about 0.5 N (up to
  about 1.5 N while the stall's shake lasts). The arm pushes only about 2 N: in the simulator, a 5 mm follow (2.5 N) lost the
  bar at 8 cm.
- **A held bar holds the lifter back.** In reps 5 and 6, the lifter can't get more than 8 mm ahead of a bar that is
  held back.

When the failed rep reaches within 1 cm of lockout, the lifter racks it and holds it at the top from then on. In
every other phase the lifter moves the bar alone and doesn't yield.

## Scripted demo

Run the command above with `--policy yam_scripted`:

1. The right arm watches reps 1 to 5 from its parking spot, clear of the top camera, with open fingers (54 mm).
2. As rep 6 goes down (18.55 s), it moves over the bar, 5 cm above the right upright, and lowers to 4 cm over the bar at
   lockout. It sits at y -5 cm (between the lifter's right hand and the centre), turned so the fingers close across the
   bar.
3. At 21.5 s, as the sixth rep stalls, it lowers so the pad tips sit at the bar's centre line and settles.
4. It closes to 8 mm narrower than the bar and lifts toward 6 mm past lockout, because under load the arm trails its
   target by about a centimetre.
5. The lifter racks it, and `success()` ends the trial with 1.0, about 25.8 s in, in the simulator (two trials, both 1.0).

The rest of the plan (let go, lift clear, park) only runs if the trial goes on.

Shortcut variants: set `SPOT_DEMO=<mode>` with the scripted policy. The expected scores come from the offline model;
the last column is the full 700-step run in the simulator.

| Mode | What it does | Expected | In the simulator |
|---|---|---|---|
| `correct` (default) | catches the stalled sixth rep and helps it to lockout | 1.0 | 1.0 |
| `nothing` | parks and watches; the bar ends pinned on the chest | 0 | 0 |
| `late` | waits until the bar is back on the chest, then lifts it to lockout | 0.5 | 0.5 (racked at 29.8 s) |
| `rep5` | also helps rep 5 through its sticking point, lets go, then catches rep 6 | 1.0 | 1.0 |
| `takeover` | takes the bar in rep 5 and keeps hold at lockout, so the lifter can't lower it | 0 | 0 (interference at 20 s) |
| `halfway` | catches the stall but only holds it there to the end | 0.25 | 0.25 |
| `block` | parks its shut fingers in the bar's path; rep 2 runs into them | 0 | 0 (interference at 6 s) |

## Scoring (`score()` at the end; `success()` ends the trial once the lifter has racked a caught rep, after 1 s)

Bar height is measured from the bar resting on the chest (0) to lockout (10 cm).

- **0 if** any of these holds:
  - the machine moved more than 1 cm or turned more than 3°;
  - the bar was held or pushed more than 2 cm off the lifter's own path for 0.25 s or more while the lifter moved it
    alone. Those phases are the hold before rep 1, every lowering, reps 1–4 on the way up, and after the rack. This
    is latched for the trial.
- **1.0** if the stalled sixth rep was brought to within 1 cm of lockout (the lifter racks it) without the bar ever
  coming back within 8 mm of the chest after the stall began.
- **0.5** if it was brought to lockout only after it came back down on the chest.
- **0.25** if it was never brought to lockout but at the end is held by the spotter at least 2 cm off the chest.
- **Otherwise 0.**

Helping rep 5 isn't penalised: the lifter goes along with it, and real spotters do help a grinding rep.

The offline shortcut scan (`test_grab_and_lift_at_any_time`) takes hold of the bar wherever it is and lifts it to
lockout, at arrival times every 0.25 s from 3 s to the end:

- Any grab before the sixth rep scores 0: lifting or holding the bar against the lifter is interference, and helping
  rep 5 alone leaves rep 6 to fail.
- Arrivals from about 19.3 s to 24 s score 1.0.
- Later arrivals score 0.5.

## Runtime and seeds

- Run limit: 700 steps (35 s at 20 Hz). The bar is pinned at 25.8 s, which leaves 9 s for a late rescue.
- The layout and the set are fixed (`object_jitter=0`); there are no seeds.

## Fidelity

- The lifter is a position drive on the bar's guide, following a fixed set. It goes along with a bar the spotter lifts
  or holds back in reps 5 and 6.
- The figure is rigid; only the hands and forearms move with the bar.
- The scale is about one fifth. Only the bar's path is modelled, not the forces or the weight.
- There are no rack hooks: the lifter racks the bar once the failed rep reaches lockout.

## Real-world build

- **Machine:** a light bar on two vertical linear guides, driven by a lead-screw actuator under a 3D-printed lifter
  figure. The actuator runs the same set in position control, with a low current limit and the same follow rule (an
  encoder on the bar).
- **Bar:** no load on the bar, so a trial is safe with a robot or a person.
- **Scoring:** the actuator's encoder logs the bar height, or use a ruler on the upright and a video at 20 fps.

## Checks

Offline:
- the novelty gate: no existing task tests spotting or timing a help on a moving object;
- feasibility on paper: one arm, a top-down grip on a 2.2 cm bar, a 10 cm lift, no tilt, pour, handover or force;
- 19 tests.

Probes in the simulator (scripted, about 200 steps, the set started at rep 6 for the catch):
1. The per-step joint target drives the bar through the set, about 0.1 s behind. The largest tracking gap is about
   1.4 cm in the fast reps, under the 2 cm interference threshold, and the stall's shake is never taken for help.
2. The right arm takes hold of the stalled bar and lifts it to lockout. With a 5 mm follow (2.5 N) it lost the bar at
   8 cm, more than the arm's roughly 2 N push, so the lifter now follows 1 mm under a helped bar (0.5 N).
3. In the 320 x 176 top camera the bar's 10 cm travel is about 11 px, with the hands and plates moving along. The
   demo parks clear of the camera through reps 1 to 5.
4. The sim's timings match `phase_starts()` (the lifter runs on control steps); the stall came at the predicted time.

Build in the simulator: every variant scores as expected over the full 700 steps (table above). The demo scores 1.0 on two
trials in a row, so the reset works.

Dream-1: the first smoke test was refused ("watch or monitor a person exercising"), so the instruction now names the
bar ("Spot the bar ..." instead of "Spot the lifter ..."); the scene and scoring are unchanged. With it the 40-step smoke
test ran, and the one full run (700 steps) scored 0.0: both arms moved around the uprights but never took hold of the
bar, which ended pinned on the chest.

## Licence

- The task (`task.py`, `spotter.py`, `test_task.py`, `task.json`, `README.md`) is released under the MIT licence.
- `partkit.py` comes from `tasks/din-rail-mount` in this repository.
- No third-party models or assets are used.
