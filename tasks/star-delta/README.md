# Relink a 230/400 V motor for a 400 V supply: star, not delta

The open terminal box of a three-phase motor stands in front of the right arm: six brass studs in the IEC layout, the
line row U1 V1 W1 at the back (where the supply leads come in) and the winding-end row W2 U2 V2 at the front. Three
brass links stand on them in delta, one down each column. The instruction is only "Link this 230/400 V motor for its
new 400 V supply." An electrician knows the rule: on a 230/400 V nameplate, 400 V means star. The delta links come off,
two of them bridge the front row (W2-U2 and U2-V2, the second stacked on the first at U2), and the third goes in the
box lid. Left in delta, each winding gets root 3 times its rated voltage and burns out.

The task tests a rating rule that the instruction doesn't state, done with the hands: lift each link clear of two
studs, turn two of them a quarter turn and drop each over two studs at once with 2 mm of clearance.

## Run a trial
On a machine with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s `setup.sh`), with
a Dream API key in `~/.dreamlabs/credentials`, from this folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=star_delta.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Link this 230/400 V motor for its new 400 V supply." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/star-delta-dream

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=star_delta.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Link this 230/400 V motor for its new 400 V supply." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/star-delta-demo

# One MP4 per camera, next to each run's frames
for log in ~/yam-eval/star-delta-*/*.json; do inspect-robots video "$log"; done
```

The run prints `task_score` at the end; `TASK_DEBUG=1` also prints each link's place and pose every 2 s. Offline checks
of the scoring and of every shortcut plan (no simulator, needs `yam_arena`): `python -m pytest -q test_star_delta.py`.

## Scoring
A link is on a stud pair when both its hole centres are within 4 mm of the two studs, its plate's bottom is within
9 mm of the board and it tilts at most 8 degrees. It is in the lid when its centre is inside the rim, within 2 cm of
the lid's floor. Judged at the end of the trial, from the links' poses:

| End state | Score |
|---|---|
| W2-U2 and U2-V2 bridged, the third link in the lid | 1.0 |
| One star pair bridged, the other links in the lid | 0.6 |
| All three links in the lid (safe, but the motor is not connected) | 0.3 |
| Any link anywhere else: left in delta, across the line row, loose in the box, on the table, still held | 0 |

The trial runs its full 2,400 steps and the final state is scored. On a real rig: the links' positions from a photo of the box.

## The demo
The right arm lifts W1-V2's link and sets it in the box lid; lifts V1-U2's, turns it a quarter turn and drops it over
U2-V2; then does the same with U1-W2's over W2-U2, where it stacks on the first at U2. Each move settles before it
grips and before it lets go, and pauses with the plate just above the stud tops before a slow last descent.

## Shortcut checks
Scripted, with another `DEMO_PLAN` (`run_checks.sh variants`), 2,400 steps:

| Variant | Score |
|---|---|
| Do nothing (delta left on) | 0 |
| One star link added at U2-V2, two delta links still on | 0 |
| One link left on the table, star made with the other two | 0 |

`test_star_delta.py` checks these and three more plans against the scorer offline: all three links in the lid 0.3; one
star bridge and two links in the lid 0.6; the star made across the line row U1-V1-W1 0.

## Runtime and seeds
The demo reaches 1.0 at about 48 s (about 960 steps) of the 2,400-step limit. One fixed layout: `-E object_jitter=0`.

## Licence
The task is released under the MIT licence. `partkit.py` is copied unchanged from `tasks/din-rail-mount`.
