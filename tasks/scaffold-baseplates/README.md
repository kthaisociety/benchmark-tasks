# Stand two scaffold tubes on a lawn: sole board, base plates, then tubes

A patch of lawn with two white marks lies in front of the robot. The instruction is only "Stand the two scaffold tubes
up on the lawn at the two marks." A scaffolder knows a standard never goes straight on soft ground: it punches in and
the scaffold leans. The footing is built in order: the timber sole board laid across the marks to spread the load, a
steel base plate on the board at each mark, then each tube stood in its base plate's collar. Anything set straight on
the grass sinks and leans, and stays that way for the trial.

The task tests a site rule the instruction doesn't state, with two arms working at once: a long board carried and laid
across both marks, two plates set on it at the marks, two tall tubes carried upright and lowered into 3.4 cm collars
(2 mm of play round a 3 cm tube).

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=scaffold_baseplates.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the two scaffold tubes up on the lawn at the two marks." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/scaffold-baseplates-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=scaffold_baseplates.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stand the two scaffold tubes up on the lawn at the two marks." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/scaffold-baseplates-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/scaffold-baseplates-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end; `TASK_DEBUG=1` also prints every part's pose and the running score every 2 s,
and each part that sinks. Offline checks of the ground model and scoring (no simulator, needs `yam_arena`):
`python -m pytest -q test_scaffold_baseplates.py`.

## Scoring
Judged on the final state:

| Item | Points |
|---|---|
| Sole board on the lawn across both marks (centre within 3 cm of the marks' midpoint, turned at most 10 degrees) | 0.2 |
| Each base plate on the board (bottom within 5 mm of the board's top, over the board), within 3 cm of its mark | 0.15 each |
| Each tube standing in a counted base plate's collar (axis within 6 mm, foot within 5 mm of the plate, tilt at most 5 degrees) | 0.25 each |

A plate or tube that comes to rest straight on the grass (its bottom within 4 mm of the grass, over the lawn and not
over the board, still for 0.5 s) sinks 8 mm and leans (6 degrees for a plate, 10 for a tube) and earns nothing for the
rest of the trial, even if it is lifted again, as a punched-in footing has to be re-bedded. On a real rig: a tape
measure on the board, plates and tubes against the marks, and the video for anything set on the grass.

## The demo
The right arm lifts the sole board across its width and lays it on the lawn across the marks. Then both arms work at
once, each on its own side: its base plate, gripped by the collar, onto the board at its mark; then its tube, gripped
4 cm below the top, carried upright from behind the lawn and lowered slowly into the collar. Each arm retreats above
the tube tops before moving clear.

## Shortcut checks
Scripted, with another `DEMO_PLAN` (`run_checks.sh variants`), 2,400 steps:

| Variant | End state | Score |
|---|---|---|
| Tubes stood straight on the grass at the marks | the tubes sink and lean | 0 |

`test_scaffold_baseplates.py` checks this and partial states offline: do nothing 0; board only 0.2; board and plates
0.5; board with the tubes straight on the grass 0.2; base plates on the grass without the board, tubes in them 0;
anything that has sunk stays sunk; carrying a part over the grass doesn't sink it; a tube off axis or leaning doesn't
count.

## Runtime and seeds
The demo finishes its moves at about 62 s (about 1,240 steps) of the 2,400-step limit; the trial runs to its limit and
scores the final state. One fixed layout: `-E object_jitter=0`.

## Licence
The task is released under the MIT licence. `partkit.py` is copied unchanged from `tasks/din-rail-mount`.
