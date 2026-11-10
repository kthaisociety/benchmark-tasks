# Hold a rail end stopper down and slide the carriage off past it

Unlike the spring-plunger panel task (#223) and the spring-return drawer task (#805), where one arm works against a spring part that resists, here one arm must hold a spring stopper down for the whole slide while the other arm moves a different part past it, and only then release it. One gripper cannot do both.

Linear-guide rails on gantries and wafer handlers carry an end stopper that keeps the carriage from running off. To service the carriage a technician holds the stopper down with one hand and slides the carriage off with the other, because forcing it past the stop face damages the balls and the stopper. One gripper cannot press the stopper and pull the carriage at once, so the task needs two coordinated arms: hold a spring part down for the whole slide, release it only after the carriage is clear, and never lift the carriage over the stop. The score is latched, so a lift over the stopper cannot be undone.

A grey T-rail lies along x on a low plinth, its open end toward the arms and a far-end block at the other end. A blue
carriage with a grip post is seated near the far end. An orange spring stopper near the open end has a vertical face
that blocks the carriage; its 2 cm pad sits off to the right of the rail. A light blue mat on the left is for the
removed carriage. The scene, scorer and scripted demo are in `task.py`.

## Run a trial

In the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`), from the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/rail-carriage-remove-past-stop/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the orange stopper down and slide the blue carriage along the grey rail off its open end, keeping it within 1.5 cm of the rail line and no more than 3 cm above it until it is past the stopper. Set it upright with its centre on the blue mat, then let go of both and keep both fingers at least 3 cm from the post." \
  --scorer task_score --store-frames --max-steps 600 --log-dir runs/dream
```

Scripted demo: the same flags with `--policy yam_scripted`. Wrong attempts for the shortcut checks: set `RCRP_VARIANT`
to `nothing`, `left_only`, `tip_off`, `right_only`, `lift_over`, `lift_restore`, `elsewhere` or `hold` before the scripted run.
Wall time per trial: about 3.5 minutes (600 steps, 30 s simulated) including simulator start. No randomness, no seeds.

## Scoring

Computed per step from object poses and the arms' finger joints and pad tips. See `scoring` in `task.json`.

- 1.0: carriage off the rail (past the open end, or more than 4 cm to the side of the rail axis), upright (3 degrees), centre within the 10 x 10 cm mat, and for the last 1 s still (under 1 mm), released
  (both arms' fingers at least 90% open and 3 cm from the post) and stopper back up.
- 0.5: off the rail but not all of that.
- 0: still on the rail, or, at any time before it was wholly past the stopper (its far end beyond the stopper's rear
  edge, 7.6 cm from the rail centre toward the open end), lifted more than 3 cm above its seat or its centre more than
  1.5 cm sideways from the rail axis. Latched for the trial, so putting it back does not help.

## Two arms and reach

One gripper cannot press the stopper pad and pull the carriage at once. Offline inverse kinematics: the right arm
reaches the pad (worst error 0.2 mm) and the left arm reaches the post, the slide to the open end and the mat (worst
error 0.16 mm). The mat is on the left arm's side; the pad is on the right arm's side, 15 cm from the rail so the two
hands do not touch.

## Shortcut and robustness checks (scripted, 600 steps)

| Variant | What it does | Score |
|---|---|---|
| Demo | Right arm holds the pad down, left arm slides the carriage off and sets it on the mat, both let go | 1.0 |
| `nothing` | Both arms park | 0 |
| `left_only` | Pulls the carriage without holding the stopper: it stops against the stopper face (carriage centre at -48 mm, face at -66 mm, stopper not pressed), then the scripted carry tips it off the rail head sideways, short of the mat; the sideways latch trips as well | 0 |
| `right_only` | Presses the stopper, carriage untouched | 0 |
| `lift_over` | Lifts the carriage over the stopper | 0 (latched) |
| `lift_restore` | Lifts it 5 cm, puts it back, then does the job properly | 0 (latch holds) |
| `elsewhere` | Sets it down beside the rail, not on the mat | 0 |
| `tip_off` | Jams on the stopper face, tips the carriage sideways off the rail and carries it to the mat (the sideways offset passes 1.5 cm, so the latch trips) | 0 (latched) |
| `hold` | Sets it on the mat but keeps gripping it to the end | 0.5 |
| Dream-1 | One run | 0.0 |

## Results

Dream-1: 0.0 on the final instruction and scorer (600 steps; the arm does not grip the post). Demo: 1.0.

## Fidelity

See `fidelity` in `task.json`. The stopper's travel is 7 mm (face 6 mm) so its top clears the carriage's side wall.

## Licence

The task is released under the MIT licence. It uses no library or third-party assets: the rail, plinth, stopper,
carriage, mat and slab are plain boxes and a cylinder, released under the task licence. `partkit.py` is reused from
the accepted `din-rail-mount` task in this repository (same licence as the repository).
