# Find a DC earth fault by pulling fuses one at a time

A substation's DC distribution panel stands in front of the right arm: four feeders (F1-F4), each running through a
fuse carrier that stands in its own socket, and an insulation monitor whose needle sits in the red zone. The
instruction is only "Locate the fault on the DC system." An operator knows the procedure: pull one fuse, watch the
needle, and if it stays red, put that fuse back before trying the next, so protection is never lost on two circuits at
once; finish with only the faulty feeder's fuse out. Pulling every fuse clears the meter too, but leaves three healthy
circuits unpowered, and scores 0.25.

The task tests an instrument-guided search with a constraint, done with the hands: grasp and pull a carrier, watch the
meter, reseat it in a 2.5 mm-clearance socket, move on, and stop in the right state.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=dc_earth_fault.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Locate the fault on the DC system." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/dc-earth-fault-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=dc_earth_fault.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Locate the fault on the DC system." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/dc-earth-fault-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/dc-earth-fault-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end; `TASK_DEBUG=1` also prints every socket change and the needle once a second.
Offline checks of the model and scoring (no simulator, needs `yam_arena`): `python -m pytest -q test_dc_earth_fault.py`.

## Scoring
A socket is closed while a carrier stands in it with its bottom within 10 mm of the socket floor, whichever carrier it
is; otherwise it is open. Judged at the end of the trial, from the sockets and the record of how many were ever open
at once:

| End state | Never two open at once | Two or more open together at some point |
|---|---|---|
| F3's socket the only open one | 1.0 | 0.5 |
| F3's socket open with another | 0.25 | 0.25 |
| F3's socket closed | 0 | 0 |

The trial always runs its full 1,200 steps, so putting F3's fuse back after finding it is seen and scores 0. On a real rig: a ruler on each carrier's cap height, and a video of
the sockets for the "two open at once" record.

## DC system model
Whenever F3's socket is open the needle moves towards the green mark with a 0.4 s lag; otherwise towards red. The
model runs in a partkit `on_step` hook, which reads the carriers' positions and turns the needle.

## The demo
The right arm pulls F1's carrier 5 cm up, holds it 1.5 s while the meter stays red, and lowers it back in; does the
same with F2; pulls F3's, sees the needle swing to green, sets that fuse down beside the panel and moves clear. Each reinsertion pauses 5 mm above the socket before a slow last descent; a fuse taken back from the table is aimed 5 mm further out (`INSERT_AIM_X`), where it otherwise lands short.

## Shortcut checks
Each is the task with another `DEMO_PLAN` (`run_checks.sh variants`), scripted, 1,200 steps:

| Variant | Score | Why |
|---|---|---|
| Do nothing | 0 | F3's socket closed |
| Pull all four and set them down | 0.25 | F3 open with three others |
| Pull F1, F2, F3 without putting any back | 0.25 | F3 open with two others |
| F1 out, F2 pulled and put back, F1 back, F3 out | 0.5 | right end state, but F1 and F2 were out together |
| Check F1, F2 and F3, putting each back | 0 | F3 found, then reinserted |
| Pull F3 directly | 1.0 | legitimate: pulling a fuse is the test, and the meter confirms it |

## Runtime and seeds
The demo pulls F3 at 26.6 s and the trial runs to its 1,200-step limit (60 s at 20 Hz); the longest variant ends its
plan at about 50 s. One fixed layout: `-E object_jitter=0`; the DC model is deterministic.

## Licence
The task is released under the MIT licence. `partkit.py` is copied unchanged from `tasks/din-rail-mount`.
