# Turn two hurdles round so their feet face the runners' approach

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [hurdles_facing_the_runners.py](hurdles_facing_the_runners.py).

Two lanes of track run away from the arms, from a start line with starting blocks to a chequered finish line, with a hurdle on its marks in each lane. Both hurdles were put out the wrong way round, their L-shaped feet pointing toward the finish. The instruction is only to set the hurdles for the race. What goes unsaid is the rule every track official checks: a hurdle's feet must point toward the runners' approach, so that a clipped hurdle tips away from the runner instead of bracing against them. Each hurdle must be turned round on its marks.

[partkit.py](partkit.py) builds each hurdle (bar, uprights and feet) as one rigid part.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=hurdles_facing_the_runners.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the hurdles for the race." \
  --scorer task_score --max-steps 2000 --store-frames --log-dir ~/yam-eval/hurdles-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 1100`. The left arm grips the left lane's hurdle by its bar, lifts it a few centimetres, turns it a quarter turn and sets it down, then regrips the bar and turns it another quarter turn, so it ends on its marks facing the start; then the right arm does the same in the right lane.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on how the hurdles stand when it ends.

## Checks

Scripted variants of the same scene, set with the `HURDLE_MODE` and `HURDLE_SHIFT` environment variables:

| Variant | Score |
|---|---|
| Demo: both hurdles turned round on their marks | 1.0 |
| Nothing done (`HURDLE_MODE=none`) | 0.0 |
| Only the left hurdle turned (`HURDLE_MODE=one`) | 0.5 |
| Both turned only a quarter turn, standing side-on (`HURDLE_MODE=half`) | 0.0 |
| Both turned round but set down 6 cm past their marks (`HURDLE_SHIFT=0.06`) | 0.0 |

Setting `HURDLE_DEBUG=1` prints each hurdle's position, heading, tilt and whether it counts as set every 40 steps.

## Runtime and seeds

- Demo: about 960 steps (48 s simulated), about three minutes on an L40S.
- Dream-1: 2,000 steps (100 s simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes and cylinders in the task file.
