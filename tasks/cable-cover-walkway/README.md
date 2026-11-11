# Lay a cable cover over the cable where the walkway crosses it

A task file for the `yam_arena` embodiment (Isaac Lab-Arena) on the station scene: [cable_cover_walkway.py](cable_cover_walkway.py).

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/cable-cover-walkway/cable_cover_walkway.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Lay the cable cover." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/cable-cover-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 400`.

The run prints `task_score` at the end. `score()` returns 0 unless the cover rests flat along the cable and over it; then 1.0 if it spans the whole walkway width, 0.5 if it covers the cable elsewhere.

## Runtime and seeds

- Demo: 194 steps, about 2 minutes on an L40S including start-up.
- Dream-1: 600 steps (30 seconds simulated).
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence

Task code under MIT. No third-party assets.
