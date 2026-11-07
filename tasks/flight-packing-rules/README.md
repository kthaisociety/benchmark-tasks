# Pack for a flight: the power bank in the cabin bag, the large shampoo in the hold case

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [flight_packing_rules.py](flight_packing_rules.py).

An open suitcase with a checked-baggage tag and a small cabin backpack stand on a bench, with a power bank, a large bottle of shampoo and a folded sweater between them. The instruction is to pack the bags for the flight and says the suitcase goes in the hold. What goes unsaid are two airline rules that point opposite ways: spare lithium batteries and power banks must travel in the cabin, because a battery fire in the hold cannot be reached, and liquids over 100 ml are not allowed through security in the cabin, so the shampoo must go in the hold. Putting everything in either bag breaks one of them.

[partkit.py](partkit.py) builds the power bank, the shampoo bottle and the sweater as rigid parts.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=flight_packing_rules.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pack the bags for the flight; the suitcase goes in the hold." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir ~/yam-eval/flight-packing-rules-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 800`. The left arm lowers the power bank into the backpack and lays the sweater on the shirts in the suitcase; then the right arm lowers the shampoo into the free half of the suitcase.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the items lie when it ends.

## Checks

Scripted variants of the same scene, set with the `PACK_PB`, `PACK_SH` and `PACK_SWEATER` environment variables (`cabin` or `hold` for each item):

| Variant | Score |
|---|---|
| Demo: power bank in the backpack, shampoo and sweater in the suitcase | 1.0 |
| Everything in the suitcase | 0.0 |
| Power bank and shampoo in the backpack | 0.5 |
| Swapped: power bank in the suitcase, shampoo in the backpack | 0.0 |
| Nothing done (`PACK_NONE=1`) | 0.0 |

The demo was also run for two trials in one process; both scored 1.0. Setting `PACK_DEBUG=1` prints the items' positions and the score every 40 steps.

## Runtime and seeds

- Demo: about 760 steps (38 s simulated), about two minutes on an L40S.
- Dream-1: 1,600 steps (80 s simulated), about 4 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes and cylinders in the task file.
