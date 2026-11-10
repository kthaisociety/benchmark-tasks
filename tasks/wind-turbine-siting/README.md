# Site wind turbines on two plots so none stands in another's wake

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [wind_turbine_siting.py](wind_turbine_siting.py).

Two plots lie under one prevailing wind, shown by a wind rose and a windsock. Plot A is long in the direction of the wind and plot B is long across it. The instruction is only to site the six turbines. What goes unsaid is the wake rule: turbines need about 7 rotor diameters between them along the wind and 3 across it. Lining the turbines up along each plot's long side is right for plot B and puts plot A's turbines in each other's wake.

[partkit.py](partkit.py) builds each turbine (foot, tower, nacelle and rotor) as one rigid part.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=wind_turbine_siting.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Site the six wind turbines on the two plots." \
  --scorer task_score --max-steps 2000 --store-frames --log-dir ~/yam-eval/wind-turbine-siting-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 1500`. The left arm sites three turbines in a row across the wind on plot A, gripping each by its tower, then the right arm does the same on plot B, where that row runs along the plot's long side.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the turbines stand when it ends.

## Checks

Calling `score()` directly on exact turbine positions:

| Layout | Score |
|---|---|
| Demo: a row across the wind on each plot, 10.5 cm apart | 1.0 |
| Plot A as a triangle, two across the wind and one 22 cm downwind | 1.0 |
| A row along each plot's long side | 0.5 |
| A row along the wind on both plots | 0.0 |
| Rows across the wind but only 6 cm apart | 0.5 |
| One of plot A's turbines outside the plot | 0.75 |
| Plot A only | 0.5 |
| Nothing done | 0.0 |

The task file also has scripted versions of the first layouts, set with the `WIND_MODE` environment variable (`none`, `long`, `along`, `close`, `one`). Setting `WIND_DEBUG=1` prints each plot's count and each turbine's position and tilt every 40 steps.

## Runtime and seeds

- Demo: about 1,450 steps (73 s simulated), about ten minutes on an L40S.
- Dream-1: 2,000 steps (100 s simulated), about 7 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes and cylinders in the task file.
