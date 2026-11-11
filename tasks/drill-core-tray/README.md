# Box a run of drill core in reading order, with a core-loss block where core was lost

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [drill_core_tray.py](drill_core_tray.py).

A core shed bench: five pieces of drill core lie on a V-rack in the order they came out of the hole, top of hole at the rack's red-tagged left end, with a 6 cm gap after the second piece where core was lost. The instruction is only to box the run in the core tray. Core is boxed like reading a book: rows from the tray's labelled top, each filled from the left, every piece with its red uphole cap to the left, and a core-loss block of the lost length placed where the core is missing. Three loss blocks (9, 3 and 6 cm) lie beside the tray. Closing up the gap, snaking a row back or using a loss block of the wrong length puts every later piece at the wrong depth.

[partkit.py](partkit.py) builds each core piece, with its red cap, as one rigid part.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=drill_core_tray.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Box the core run in the core tray." \
  --scorer task_score --max-steps 2000 --store-frames --log-dir ~/yam-eval/drill-core-tray-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted`. The left arm boxes core 1 at the start of the top row, the 6 cm loss block at the start of the middle row and core 4 at the start of the bottom row; then the right arm boxes core 2, core 3 and core 5 after them.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the pieces lie when it ends.

## Checks

Calling `score()` directly on exact positions:

| Layout | Score |
|---|---|
| Demo layout | 1.0 |
| Nothing done | 0.0 |
| Gap closed up: core 3 at the start of the middle row | 0.333 |
| Middle row snaked: core 3 first, loss block after it | 0.667 |
| Loss block of the wrong length (9 or 3 cm) | 0.667 |
| One piece turned round, red cap to the right | 0.833 |
| Gap left open in the right place, no loss block | 0.833 |
| Demo layout plus a wrong-length loss block in the tray | 0.833 |
| Rows filled from the bottom of the tray | 0.333 |
| Every row read right to left | 0.0 |

The task file also has scripted versions of some of these layouts, set with the `CORE_MODE` environment variable (`none`, `closed`, `snake`, `wrong_block`, `one`). Setting `CORE_DEBUG=1` prints each item's result, position and tilt every 40 steps.

## Runtime and seeds

- Demo: about 1,360 steps (68 s simulated).
- Dream-1: 2,000 steps (100 s simulated).
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes in the task file.
