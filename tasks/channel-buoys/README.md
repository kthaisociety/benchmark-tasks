# Moor red and green channel buoys on the correct side for ships entering harbour

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [channel_buoys.py](channel_buoys.py).

On a chart model, two channels run in from open sea to harbours at different ends, and one of them bends. Three gates each have two mooring pads, one either side of the channel. The instruction is only to lay the buoys. What goes unsaid is the IALA Region A convention: a ship entering from sea keeps red marks to port and green marks to starboard, so the right colour for a pad depends on the direction of travel at that gate.

[partkit.py](partkit.py) builds each buoy (foot and body) as one rigid part.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=channel_buoys.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the six buoys for the two harbour channels." \
  --scorer task_score --max-steps 1500 --store-frames --log-dir ~/yam-eval/channel-buoys-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 780`. The left arm moors the two buoys of the straight channel and the right arm the four of the channel with the bend, each lifted from the workboat by its body.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the buoys stand when it ends.

## Checks

Scripted variants of the same scene, set with the `BUOY_MODE` environment variable:

| Variant | Score |
|---|---|
| Demo: all six on the correct pads | 1.0 |
| Nothing done (`BUOY_MODE=none`) | 0.0 |
| Only the straight channel done (`BUOY_MODE=one`) | 0.33 |
| All six on the opposite pads (`BUOY_MODE=reversed`) | 0.0 |
| Straight channel reversed, other two gates correct (`BUOY_MODE=habit`) | 0.67 |

Setting `BUOY_DEBUG=1` prints each pad's result and each buoy's position and tilt every 40 steps.

## Runtime and seeds

- Demo: about 760 steps (38 s simulated), about three minutes on an L40S.
- Dream-1: 1,500 steps (75 s simulated), about 4 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes and cylinders in the task file.
