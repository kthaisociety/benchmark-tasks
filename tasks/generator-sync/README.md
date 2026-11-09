# Synchronise a generator before closing it onto the grid

A generator control panel stands in front of the right arm: a synchroscope, a red CLOSE button, and RAISE and LOWER
speed buttons. The instruction is only "Bring the generator online." An operator knows the unwritten part: the
generator must first be brought to the grid's speed (tap RAISE until the synchroscope pointer turns slowly), then the
breaker is closed as the pointer passes 12 o'clock. At the start the pointer spins once every 2 s, passing 12 o'clock
often, so pressing CLOSE straight away is tempting and scores 0.

The task tests closed-loop control from a moving visual signal (tap, watch the pointer's speed, tap again) and timing
an action to it, with the rule coming from the job rather than the instruction.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=generator_sync.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Bring the generator online." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/generator-sync-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=generator_sync.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Bring the generator online." \
  --scorer task_score --max-steps 400 --store-frames --log-dir ~/yam-eval/generator-sync-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/generator-sync-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end. Offline checks of the generator model and scoring (no simulator, needs
`yam_arena`): `python -m pytest -q test_generator_sync.py`.

## Scoring
Only the first CLOSE press counts (pushed 4 mm or more of its 8 mm travel). At that moment:

| Pointer speed | Pointer position | Score |
|---|---|---|
| faster than one turn per 6.7 s (> 0.15 Hz), either direction | anywhere | 0 |
| one turn per 6.7 s or slower | within 15 degrees of 12 o'clock | 1.0 |
| one turn per 6.7 s or slower | within 30 degrees | 0.5 |
| one turn per 6.7 s or slower | beyond 30 degrees | 0 |

CLOSE never pressed: 0.25 if the speed is matched at the end (one turn per 6.7 s or slower), else 0. A 1.0 close ends
the trial after 1 s. Later presses change nothing, so repeated pressing can't find the window.

On a real rig, film the synchroscope from above: the frame where the CLOSE button goes down gives the pointer's angle
(protractor on the frame) and the frames before it its turn time (stopwatch).

## Generator model
The pointer's angle is the generator's phase relative to the grid; it turns at the speed difference (slip), clockwise
when the generator is fast. The slip starts at -0.5 Hz; each RAISE press adds 0.125 Hz, each LOWER press subtracts it
(a press counts once, on the way down). The model runs in a partkit `on_step` hook, which also places the pointer.

## The demo
The right arm taps RAISE five times (slip -0.5 to +0.125 Hz: one slow clockwise turn per 8 s), moves over CLOSE and
waits, then presses as the pointer passes 12 o'clock at t = 11.86 s. The press registers about 0.2 s after the stroke
starts; scripted run: CLOSE at -2.2 degrees, score 1.0.

## Shortcut checks
Scripted runs of the submitted task file with the demo replaced by a shortcut (zero jitter):

| Shortcut | Score |
|---|---|
| Do nothing | 0 |
| Press CLOSE at once (pointer still turning once per 2 s) | 0 |
| Press CLOSE eight times in a row, hoping one lands at 12 o'clock | 0 |
| Press LOWER five times (generator even slower), then CLOSE | 0 |
| Match the speed, then press CLOSE without waiting for 12 o'clock | 0 |
| Match the speed, never press CLOSE | 0.25 |

## Runtime and seeds
- The demo presses CLOSE 11.9 s into the trial, which then ends 1 s later with success. On the shared L40S a demo
  run takes about 4 minutes including Isaac Sim start-up.
- One fixed layout and start: `-E object_jitter=0`; the generator model is deterministic.

## Licence
The task is released under the MIT licence. `partkit.py` is copied unchanged from `tasks/din-rail-mount`.
