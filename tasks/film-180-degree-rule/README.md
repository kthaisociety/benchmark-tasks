# Place two reverse-shot cameras without crossing the line between the actors

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [film_180_degree_rule.py](film_180_degree_rule.py).

Two pairs of actors stand on their marks, each filmed by a dark first camera. A yellow camera stands on the far side of each pair. The instruction asks only for the second angle on the set. What goes unsaid is the 180-degree rule: every camera covering a scene stays on one side of the line through its actors, or the actors appear to swap places on screen when the shots are cut together. The two lines run at different angles and nothing on the floor marks them.

[partkit.py](partkit.py) builds each camera (foot, mast, body, lens and handle) as one rigid part.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=film_180_degree_rule.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the two yellow model cameras for the second angle on this tabletop film set." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/film-180-degree-rule-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 660`. Each arm grips its yellow camera by the top handle, carries it across to the first camera's side of the line through its actors, turns it towards them and sets it down.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the cameras stand when it ends.

## Checks

Scripted variants of the same scene, set with the `FILM_MODE` environment variable:

| Variant | Score |
|---|---|
| Demo: both cameras on the correct side and aimed | 1.0 |
| Nothing done (`FILM_MODE=none`) | 0.0 |
| Only the left set done (`FILM_MODE=one`) | 0.5 |
| Both on the correct side, lenses not aimed (`FILM_MODE=no_aim`) | 0.5 |
| Both moved but left on the wrong side of the line (`FILM_MODE=wrong`) | 0.0 |

Setting `FILM_DEBUG=1` prints each camera's position, signed distance from the line, aiming error and tilt every 40 steps.

## Runtime and seeds

- Demo: about 640 steps (32 s simulated), about two minutes on an L40S.
- Dream-1: 1,500 steps (75 s simulated), about 4 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes, cylinders and spheres in the task file.
