# Impose an eight-page saddle-stitched booklet so the pages read in order once folded

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1, inspect-robots 1.0) on the station scene: [booklet_imposition.py](booklet_imposition.py).

Eight numbered page blocks start scrambled in front of four boards, each board one printed face of a sheet. The two boards on one side of the table are the two faces of one sheet. The instruction is only to impose the pages for a saddle-stitched booklet. What goes unsaid is the layout that folding forces: 8 | 1 and 2 | 7 on one sheet, 6 | 3 and 4 | 5 on the other.

[partkit.py](partkit.py) builds each page block (body and numeral) as one rigid part.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=booklet_imposition.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Impose the eight pages on the four boards for a saddle-stitched booklet." \
  --scorer task_score --max-steps 2000 --store-frames --log-dir ~/yam-eval/booklet-imposition-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 1700`. The left arm lays pages 8, 1, 2 and 7 on its two boards, reaching across the table for page 8, then the right arm lays 6, 3, 4 and 5 on its two, reaching across for page 5.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the pages stand when it ends.

## Checks

Scripted variants of the same scene, set with the `IMPOSE_MODE` environment variable:

| Variant | Score |
|---|---|
| Demo: 8 \| 1, 2 \| 7, 6 \| 3, 4 \| 5 | 1.0 |
| Nothing done (`IMPOSE_MODE=none`) | 0.0 |
| Reading order: 1 \| 2, 7 \| 8, 3 \| 4, 5 \| 6 (`IMPOSE_MODE=naive`) | 0.0 |
| Left and right swapped on every board (`IMPOSE_MODE=flipped`) | 0.5 |
| Only the left arm's sheet done (`IMPOSE_MODE=one`) | 0.5 |

Mixing the sheets (pairs that add up to nine, but 8 | 1 and 4 | 5 on one sheet, for example) is beyond the scripted arms' reach, so it was checked by calling `score()` directly on exact page positions: it scores 0.0. The same check gives 1.0 with the two sheets swapped between the sides of the table, or with the two boards of a sheet swapped, both of which are valid layouts.

Setting `IMPOSE_DEBUG=1` prints each board's pages and each block's position and tilt every 40 steps.

## Runtime and seeds

- Demo: about 1,540 steps (77 s simulated), about ten minutes on an L40S.
- Dream-1: 2,000 steps (100 s simulated), about 6 minutes on an L40S.
- `object_jitter=0` keeps the layout fixed, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence and third-party material

This task is released under the MIT licence.

- [partkit.py](partkit.py) is copied unchanged from [tasks/din-rail-mount](../din-rail-mount/partkit.py) in this repository, by that task's authors, who have not stated a licence for it.
- No third-party models or textures are used: every object is built from boxes and cylinders in the task file.
