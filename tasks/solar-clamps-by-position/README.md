# Clamp two solar panels with the right clamp at each position: end clamps at the row ends, mid clamps between

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [solar_clamps_by_position.py](solar_clamps_by_position.py).

Two solar panels lie side by side on two mounting rails, and a parts tray beside each arm holds two end clamps and one mid clamp. The instruction is only to clamp the panels to the rails. What goes unsaid is how installers choose clamps: an end clamp, with one lip, goes only at a row end over one panel's outer edge; a mid clamp, with a lip each side, goes in the gap between the panels and holds both. Each of the six positions, three on each rail, needs the right kind.

[partkit.py](partkit.py) builds each clamp (body, lip and bolt head) as one rigid part.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=solar_clamps_by_position.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Clamp the panels to the rails." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/solar-clamps-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 1400`. The left arm puts its two end clamps at the left row end on both rails and its mid clamp in the gap on the near rail; then the right arm does the same on the right, with its mid clamp in the gap on the far rail.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the clamps are when it ends.

## Checks

Scripted variants of the same scene, set with the `SOLAR_MODE` environment variable:

| Variant | Score |
|---|---|
| Demo: end clamps at the row ends, mid clamps in the gap | 1.0 |
| Nothing done (`SOLAR_MODE=none`) | 0.0 |
| Only the left arm's three clamps (`SOLAR_MODE=left_only`) | 0.5 |
| The left arm swaps its mid clamp and one end clamp (`SOLAR_MODE=swapped`) | 0.667 |

Setting `SOLAR_DEBUG=1` prints every clamp's position and tilt every 40 steps.

## Runtime and seeds

- Demo: about 1,380 steps (69 s simulated), about three minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes and cylinders in the task file.
