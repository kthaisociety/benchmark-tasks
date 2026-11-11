# Put a pressure-relief cushion in a wheelchair the right way round

A wheelchair stands right of the midline; a contoured pressure-relief cushion lies on the table beside it, backwards.
The cushion has a sunken well (dark) at one end and a raised ridge (light grey) at the other. The instruction is only
"Put the cushion in the wheelchair." A carer knows the well goes at the back, against the backrest, under the sit
bones; fitted backwards, the cushion loads the sit bones and causes pressure injuries.

The task tests a care rule the instruction doesn't state, done with the hands: read the cushion's orientation from
above, turn it 180 degrees as two quarter turns with a regrasp in between, and lower a wide part between the side
guards (2 cm clear each side) flat onto the seat.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=wheelchair_cushion.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the cushion in the wheelchair." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/wheelchair-cushion-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=wheelchair_cushion.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the cushion in the wheelchair." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/wheelchair-cushion-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/wheelchair-cushion-*/*.json; do inspect-robots video "$log"; done
```

`TASK_DEBUG=1` prints the cushion's pose and running score every 2 s. Offline checks of the scoring (no simulator,
needs `yam_arena`): `python -m pytest -q test_wheelchair_cushion.py`.

## Scoring
From the cushion's pose at the end of the trial; the trial runs its full 2,400 steps.

| End state | Score |
|---|---|
| Well toward the backrest (within 20 deg), flat on the seat pan (bottom within 5 mm, tilt <= 5 deg, centre within 4 cm of the pan's centre front to back), at rest | 1.0 |
| Well toward the backrest, centre over the seat pan, not seated (on a guard, tilted) | 0.5 |
| Well toward the backrest, flat on the table | 0.25 |
| Backwards or sideways anywhere, or still moving | 0 |

On a real rig: the cushion's orientation and position from a photo, its height with a ruler.

## The demo
The right arm grips the handle, lifts the cushion 6 cm, turns it a quarter turn and sets it down; regrasps and does
it again, so the well now faces the backrest; then lifts it over the guards, pauses above the seat and lowers it
slowly between them. It lets go, retreats upwards and moves clear of the camera.

## Shortcut checks
Scripted, with another `DEMO_PLAN` (`run_checks.sh variants`), 2,400 steps:

| Variant | End state | Score |
|---|---|---|
| Do nothing | backwards on the table | 0 |
| Put it in as found | flat in the seat, backwards (yaw 180 deg) | 0 |
| One quarter turn, then put it in | flat in the seat, sideways (yaw -90 deg) | 0 |

`test_wheelchair_cushion.py` also checks: turned round but left on the table 0.25; resting on a guard or tilted in the
seat 0.5; 15 degrees off still counts; pushed 6 cm toward the backrest 0.5; still moving 0.

## Runtime and seeds
The demo finishes its moves at about 50 s (about 1,000 steps) of the 2,400-step limit; the trial runs to its limit and
scores the final state. One fixed layout: `-E object_jitter=0`.

## Licence
The task is released under the MIT licence. `partkit.py` is copied unchanged from `tasks/din-rail-mount`.
