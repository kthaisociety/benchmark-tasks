# Site a rain gauge clear of the wall and turn the instrument screen's door to face north

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [weather_station_siting.py](weather_station_siting.py).

A small weather observing site stands on a lawn beside a brick wall, with a compass rose on the grass showing which way is north. A copper rain gauge stands beside the wall and a white louvred instrument screen stands beside it with its door facing south. The instruction is only to set up the weather station. What goes unsaid are two rules from standard observing practice: a rain gauge must stand clear of obstacles, at least twice and ideally up to four times as far away as they are tall, or the wall shelters it; and an instrument screen's door must face north in the northern hemisphere, so the sun never falls on the thermometers when it is opened. The gauge therefore has to be carried out from the wall, and the screen turned round to face north.

[partkit.py](partkit.py) builds the rain gauge and the screen (body, louvres, door, handle frame) as rigid parts.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=weather_station_siting.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set up the weather station." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/weather-station-siting-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 760`. The left arm grips the rain gauge and carries it out from the wall to 22 cm clear of it; then the right arm grips the screen by its handle, turns it a quarter turn, sets it down, regrips the handle and turns it another quarter turn, so its door faces north.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the instruments stand when it ends.

## Checks

Scripted variants of the same scene, set with the `WX_MODE` environment variable:

| Variant | Score |
|---|---|
| Demo: gauge carried out, screen turned to face north | 1.0 |
| Nothing done (`WX_MODE=none`) | 0.0 |
| Only the gauge moved (`WX_MODE=gauge_only`) | 0.5 |
| Only the screen turned (`WX_MODE=screen_only`) | 0.5 |
| Gauge moved, screen turned only a quarter turn so its door faces east (`WX_MODE=wrong_way`) | 0.5 |

Setting `WX_DEBUG=1` prints the gauge's clearance and the screen's door direction every 40 steps.

## Runtime and seeds

- Demo: about 760 steps (38 s simulated), about three minutes on an L40S.
- Dream-1: 1,800 steps (90 s simulated), about 5 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes and cylinders in the task file.
