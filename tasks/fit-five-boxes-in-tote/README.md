# Fit five boxes into a tote they only fit one way

Instruction given to the policy: `Put all five boxes in the tote, each lying flat on the tote floor.`

Files: `task.json`, `fit_five_boxes.py` (scene, scoring, success and the scripted demo), `README.md`.

## Environment

Built on the program's Brev launchable (Isaac Lab-Arena, Isaac Sim, L40S). Before every shell:
`source ~/IsaacLab-Arena/yam.env`.

## Run and score one Dream-1 trial

From the root of this repository:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/fit-five-boxes-in-tote/fit_five_boxes.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put all five boxes in the tote, each lying flat on the tote floor." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/fit-five-boxes-dream
```

`task_score` is 1.0 when `success()` (all five boxes packed) holds for 1 s, which ends the trial; otherwise
it is `score()` at the end: 0.25 for exactly four packed with the fifth on the table outside the tote, else 0.
The CLI prints `task_score: <value>`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/fit-five-boxes-in-tote/fit_five_boxes.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Put all five boxes in the tote, each lying flat on the tote floor." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/fit-five-boxes-demo
```

The demo is open loop. The left arm packs the red box into the far row, the yellow box into the middle row
and the blue box into the near row, each turned a quarter turn so it lies side to side; the right arm turns
the green and purple boxes a quarter turn into the column beside the right wall, the purple one only after
the blue one is in (both go by the near wall). Each box is gripped across its 6 or 7 cm side, 12 mm
narrower than the box, with the fingers opened only 1 cm past the box on the way down and 6 mm past it on
release, so they fit the 2 cm gaps. The red box is gripped 4 cm off centre to keep its place inside the
left arm's reach. The demo reaches `task_score: 1.0` in 1,209 steps, ended by `success`.

Shortcut variants, selected with the environment variable `FIT_BOXES_VARIANT` (default `full`), each
checked in simulation:

| Variant | What it does | Score |
|---|---|---|
| `full` | the demo above | 1.0 |
| `four` | the purple box left on the table | 0.25 |
| `stack` | the purple box set on top of the blue box | 0.0 |
| `nothing` | no box moved | 0.0 |

`FIT_BOXES_DEBUG=1` prints every box's pose and packed flag every 40 steps.

## Why there is one arrangement

The tote and box sizes were chosen by an exhaustive search on a 1 cm grid, boxes lying flat and turned
0 or 90 degrees, with at least 2 cm between boxes and walls (the room the fingers need). For this tote, 5 of
the 32 orientation patterns can be packed, all with the three long boxes side to side, and a red, blue,
yellow or purple box placed in the middle of the tote leaves no room for the rest.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (`--max-steps 2400`).
- Runtime per trial on an L40S: the demo about 4 minutes; add 4 to 6 minutes for the first run after boot.
- Seeds: `--seed 0`; `-E object_jitter=0` fixes the layout, and the open-loop demo needs it.

## Submitted videos and scores

Versions: inspect-robots 1.0.0, yam-arena at commit 0059d02a1a91, Dream checkpoint `dreamlabs/dream-1-beta`;
task file SHA-256 (first 12 hex) 4ae70638186d (`sha256sum fit_five_boxes.py | cut -c1-12`).

- Dream-1 run: 2,400 steps, `task_score = 0.0`, ended by the step limit. The left arm drew back towards the
  table edge and the right arm hovered over the green and purple boxes; no box was picked up.
- Demo: `task_score = 1.0`, ended by `success` after 1,209 steps; two further reset-check trials also
  scored 1.0.
- Setup screenshot: frame 0 of the top camera at 640 x 360. Both videos are the 320 x 176 `top_cam` stream
  rendered with `inspect-robots video <log>` (H.264).

## Licence

MIT. All objects are primitive boxes; no third-party assets.
