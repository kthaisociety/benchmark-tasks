# Take every remove-before-flight item off an aircraft and stow it in the kit bag

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [remove_before_flight.py](remove_before_flight.py).

A model light aircraft stands chocked on the table. Three items that protect it on the ground are still fitted, each flagged with a red streamer: a plug in the engine air intake on the nose, a cover over the pitot tube on the wing, and a lock pin in the main landing gear. The instruction is only to get the aircraft ready for flight. What goes unsaid is the walkround every pilot and ground crew does: every remove-before-flight item comes off and goes into the kit bag, where it can be counted, rather than being left on the apron as debris.

[partkit.py](partkit.py) builds each item (a block in a socket, a grip knob and its streamer) as one rigid part.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=remove_before_flight.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Get the aircraft ready for flight." \
  --scorer task_score --max-steps 1800 --store-frames --log-dir ~/yam-eval/remove-before-flight-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 800`. The left arm lifts the intake plug off the nose and the cover off the pitot tube into the kit bag; then the right arm pulls the gear lock pin and drops it in too.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the three items lie when it ends.

## Checks

Scripted variants of the same scene, set with the `RBF_TAKE` (items to remove) and `RBF_DROP` environment variables:

| Variant | Score |
|---|---|
| Demo: all three items into the kit bag | 1.0 |
| Intake plug and pitot cover only; gear pin left in | 0.667 |
| All three removed but set down on the table beside the bag (`RBF_DROP=table`) | 0.0 |
| Nothing done | 0.0 |

Setting `RBF_DEBUG=1` prints each item's position and whether it is in the bag every 40 steps.

## Runtime and seeds

- Demo: about 650 steps (33 s simulated), about two minutes on an L40S.
- Dream-1: 1,800 steps (90 s simulated), about 4 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes and cylinders in the task file.
