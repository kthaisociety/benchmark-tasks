# Check the gauge with a reference block before quarantining a failed part

A part has just failed a go/no-go gauge. Before rejecting it, the robot tests the gauge's known-good reference block
on the same test pad. If the reference passes, the gauge is working and the part goes to quarantine. If the
reference fails too, the gauge is at fault: the robot stands the out-of-service tag on the gauge and sends the part
for retest. In both cases the reference block goes back to its holder.

The instruction names the reference check and what each container and the tag mean, but not which result leads to
which disposition: that is the inspection knowledge being tested. Each run has two trials with the same layout, one
for each outcome, so a policy that always quarantines (or always tags out) scores at most half.

## Files

| File | What it is |
| --- | --- |
| `task.py` | The yam_arena task file: scene, gauge model, scorer and scripted demo |
| `partkit.py` | Compound parts and spring sliders, copied unchanged from `tasks/din-rail-mount` |
| `test_task.py` | Offline tests of the scorer and demo plan (no simulator) |

## Scene and reset

All positions are in cm from the midpoint between the arm bases, x forward and y left, on the station table.
`task.py` has the exact geometry.

| Object | Description and start |
| --- | --- |
| Gauge (fixed, 10 kg) | Dark base plate 17 x 34 cm at x 40, y 0. Steel test pad 8 x 8 x 1.2 cm at x 36, y 0. Black TEST button cap 3.5 x 3.5 cm, top 4.3 cm high, 8 mm travel, at x 38, y -14. Grey lamp housing 4 x 12 x 6 cm at x 46, y -1, with a green PASS lamp (y -4) and a red FAIL lamp (y +2), both dark at the start. Yellow tag plate 8 x 8 cm at x 38, y 13 |
| Failed part | Plain aluminium block 4 x 4 x 5 cm, 216 g, on an orange 8 x 8 cm mat at x 24, y 13 |
| Reference block | Same block with a 1.2 cm green band round its top, in a green open holder at x 24, y -12 |
| Out-of-service tag | Yellow polymer stand 5 x 3.5 x 6 cm, 60 g, with a 1 cm red top band, on a yellow mat at x 50, y 12 |
| Quarantine bin | Fixed red open frame, 10 x 10 cm pocket, walls 6 mm thick and 2 cm tall, at x 33, y 29 |
| Retest tray | Fixed blue open frame, same size, at x 47, y 27 |

The reference holder has an 8 x 8 cm pocket with the same walls. Use the first top-camera frame as the setup image.

**Gauge model.** Pressing TEST past 4 mm tests whatever stands on the pad and lights one lamp until the next test.
An empty pad lights nothing. In trial 1 the gauge is in calibration: the reference alone passes (green); anything
else fails (red). In trial 2 the gauge has drifted: every test fails (red). Nothing visible differs between trials
before a test. On a real rig, the evaluator sets a hidden switch for trial 2.

**Reset.** Between trials, put every block and the tag back at its start, darken both lamps and set the gauge state
for the next trial. In simulation the reset hook does this, and trials alternate in calibration / drifted.

## Scoring

Each trial is scored at its end from photos and a ruler, with the objects still (under 1 cm/s). The run's score is
the mean of its two trials.

- **Check:** the reference block alone was tested: TEST pressed while the reference stood on the pad (centre within
  2 cm of the pad centre, bottom within 6 mm of the pad top) and the part didn't. Without it the trial scores **0**.
- **0.25** for the check.
- **+0.25** if the reference block is in its holder at the end.
- **+0.5** for the correct disposition. Trial 1: the part in the quarantine bin. Trial 2: the tag standing on the tag
  plate (0.25) and the part in the retest tray (0.25).
- **Cap at 0.25** if, at any point in the trial: the part or tag was placed before the check; the part went to the
  wrong bin or tray; the tag was placed in trial 1; or the reference was put in a bin or tray. The cap stays for the
  trial even if the object is moved back.

In a bin, tray or holder, or on the tag plate, means centre inside its walls and bottom under 2 cm above the table
(an object carried over a bin doesn't count). Possible trial scores: 0, 0.25, 0.5, 0.75, 1.0.

## Run and score on Isaac Lab-Arena

From this folder, with the program's Isaac Lab-Arena environment active:

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --policy yam_scripted --scorer task_score --no-prompt --instruction "$instruction" \
  --max-steps 1200 --epochs 2 --store-frames --log-dir runs/demo
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --policy dream --scorer task_score --no-prompt --instruction "$instruction" \
  --max-steps 1200 --epochs 2 --store-frames --log-dir runs/dream
python -m pytest -q test_task.py
```

Run both trials in one process (`--epochs 2`): the trial order (in calibration, then drifted) comes from the reset
count. Each trial allows 1,200 actions (60 s at 20 Hz). The scripted demo takes 615 steps (31 s) in trial 1 and 808 steps (40 s)
in trial 2. It plays open loop, so it is written for each outcome and played in trial order, as an operator would act
on the lamp.

## Validation

All runs on the L40S instance, object jitter 0, 20 Hz, two trials per run (`--epochs 2`).

| Run | Trial 1 (in calibration) | Trial 2 (drifted) | Score |
| --- | --- | --- | --- |
| Scripted YAM demo | **1.0** in 615 steps: reference tested, PASS, part in quarantine, reference home | **1.0** in 808 steps: reference tested, FAIL, tag on the plate, part in retest, reference home | **1.0** |
| Dream-1 | 0.0 at 1,200 steps: no object moved | 0.0 at 1,200 steps: no object moved | **0.0** |

In the demo the reference block landed within 1.1 cm of the pad centre (2 cm allowed). Both lamps are dark in the
first frame of each trial; the PASS lamp lit in trial 1 and the FAIL lamp in trial 2. A 40-step Dream-1 smoke test
ran first with no refusal.

The 19 offline tests (`test_task.py`) cover the full procedure in both trials, testing the part instead of the
reference, guessing without a check, both dispositions at once, a disposition made before the check and then undone,
the reference put in a bin, carrying over a bin, and the demo plan's reach and timing. These shortcut cases are
checked against the scorer offline, not in simulation. The task has not been run on hardware.

## Licences

Task files are under the repository's MIT licence. `partkit.py` comes from `tasks/din-rail-mount`. The scene uses
only primitive shapes; no third-party assets.
