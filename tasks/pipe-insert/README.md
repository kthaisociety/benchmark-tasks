# Push-fit an elbow onto a copper and a plastic pipe, with an insert in the plastic one

Two pipe tails stand up from a wall block: copper in front of the left arm, white plastic (PEX) in front of the right
arm. Two grey push-fit elbows lie in a tray, and a bright brass pipe insert stands on a peg stand. The instruction is
only "Fit an elbow onto each pipe." A plumber knows the rule: a push-fit fitting's grab ring squeezes soft plastic pipe,
so a metal insert goes into the plastic pipe's end first; copper needs none, and an insert in it stops the fitting
going home.

The task tests that rule with three push-down insertions with a few millimetres of clearance: the insert into the plastic pipe's
bore, then an elbow's socket over each pipe end.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=pipe_insert.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit an elbow onto each pipe." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pipe-insert-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=pipe_insert.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Fit an elbow onto each pipe." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/pipe-insert-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/pipe-insert-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end; `TASK_DEBUG=1` also prints the parts' poses every 2 s. Offline checks of the
scoring, the latch and every demo plan (no simulator, needs `yam_arena`): `python -m pytest -q test_pipe_insert.py`.

## Scoring
A part is on a pipe when its centre is within 3 mm of the pipe's axis and it tilts at most 5 degrees. The insert is
seated when its collar rests on the pipe end; an elbow is fully on when its cap rests on the pipe end, or on the
insert's collar if one is seated there (each within 3 mm). Judged at the end of the trial:

| Joint | 0.5 when |
|---|---|
| Copper | an elbow is fully on and there is no insert in the copper pipe |
| Plastic | the insert is seated, an elbow is fully on over it, and no elbow was ever fully on the plastic pipe without the insert |

The plastic joint's violation is latched for the trial (a real grab ring has already crushed the pipe end; taking the
elbow off doesn't undo that) and cleared on reset. The trial runs to the end and the final state is scored. On a real
rig: a ruler from each fitting's top to the wall, and a look into the plastic pipe's end.

## The demo
The right arm takes the insert from its stand by the collar, lowers it over the white pipe, pauses with the shank just
above the bore and pushes it slowly home; then it takes the right elbow by its tab and pushes it over the plastic pipe
and the insert. Then the left arm pushes the left elbow onto the copper pipe. One move at a time, the idle arm waiting
clear; both arms end clear of the camera.

## Shortcut checks
Scripted, with another `DEMO_PLAN` (`run_checks.sh variants`), 2,400 steps:

| Variant | End state | Score |
|---|---|---|
| Both elbows pushed on, no insert | copper joint made, plastic joint without its insert | 0.5 |

`test_pipe_insert.py` checks this and the other plans against the scorer offline: do nothing 0; the insert in the
copper pipe, then both elbows 0 (the copper elbow sits proud and the plastic joint has no insert); an elbow pushed onto
the plastic pipe first and fixed later 0 for that joint (latched for the trial).

## Runtime and seeds
The demo finishes its moves at about 62 s (about 1,240 steps) of the 2,400-step limit; the trial runs to its limit and
scores the final state. One fixed layout: `-E object_jitter=0`.

## Licence
The task is released under the MIT licence. `partkit.py` is copied unchanged from `tasks/din-rail-mount`.
