# Find the 10 mg pack among look-alike cartons whose labels face away, and dispense only it

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [check_label.py](check_label.py).

Four identical white medicine cartons stand in a tote with their labels facing away from the arms. One label has the red band of the 10 mg strength, the other three the green band of 5 mg. Which slot holds the 10 mg carton is drawn from the trial's seed at every reset, so its position says nothing. No camera can see a label at the start (checked below): the robot has to turn cartons until a label shows before it chooses.

The label is a coloured band, not text, because the simulation renders no readable text.

## Scoring

`score()` runs every control step:

- **Shown:** the red band counts as shown once the overhead camera image (640 x 360) holds at least 15 red pixels for 10 steps in a row (0.5 s) while the 10 mg carton is outside the basket. Only the 10 mg label is red, and the image is the one the camera records, so a photo of the trial settles it.
- **In the basket:** the 10 mg carton counts only when it is released and at rest on the basket floor: centre inside the walls, base within 1 cm of the floor, no closed gripper within 8 cm of it.
- **Score:** 0 if any 5 mg carton is inside the basket walls (held or not). Otherwise 1.0 for the 10 mg carton in the basket after its band was shown; 0.25 for the band shown without the carton ending in the basket; 0 for anything else, including the 10 mg carton dispensed without its band ever being shown.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=check_label.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the 10 mg box in the basket. The 10 mg packs have a red band on their label." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/check-label-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 1600`. Each arm turns its two cartons half a turn, in two quarter turns with a fresh grip between, so their labels face the arms. The demo then looks for the red band in the overhead camera image it receives, maps the band's image column to a slot with the camera's published pose and lens, and the arm on that side lifts that carton into the basket. It decides from the camera image only, not from the simulator's state.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on what ends in the basket and on whether the red band was shown first.

`LABEL_SLOT=0..3` fixes the 10 mg carton's slot (0 is the leftmost) and `LABEL_DEBUG=1` prints the red pixel count and the 10 mg carton's position every step.

## Checks

At the start, with the arms at rest, the overhead camera shows 0 red pixels in each of the four layouts.

The demo with the 10 mg carton in each slot, and with slots drawn from the seed:

| Run | Score |
|---|---|
| Slot 0 (left arm) | 1.0 |
| Slot 1 (left arm) | 1.0 |
| Slot 2 (right arm) | 1.0 |
| Slot 3 (right arm) | 1.0 |
| Three trials, slots drawn from the seed (slots 1, 2 and 3) | 1.0, 1.0, 1.0 |

How much of a turned label the overhead camera sees depends on how square the carton ends up after its turn, so the red pixel count when shown varies between runs (from just over the threshold of 15 to over 100 at 640 x 360 in our runs); a label facing away gives 0.

Wrong strategies, scripted with `LABEL_VARIANT` (10 mg carton in slot 1):

| Variant | Score |
|---|---|
| Dispense the 10 mg carton without turning any (a lucky guess) | 0 |
| Dispense a 5 mg carton without turning any | 0 |
| Turn all four, dispense nothing | 0.25 |
| Turn all four, then dispense a 5 mg carton | 0 |
| Nothing done | 0 |

## Runtime and seeds

- Demo: about 1,440 steps (72 s simulated), about four minutes on an L40S.
- Dream-1: 2,400 steps (2 minutes simulated), about 7 minutes on an L40S.
- Each trial draws the 10 mg slot from its seed (`inspect-robots run --seed N` repeats a sequence of layouts); `object_jitter=0` keeps every other position fixed. The scripted demo is deterministic for a given slot.

## Licence and third-party material

This task is released under the MIT licence, including [trialkit.py](trialkit.py), a helper written for this task: it draws the per-trial layout from the trial's seed, fixes the label panels to the cartons, and runs the demo in stages from the policy's observations.

No third-party models, textures or code are used: every object is built from boxes and cylinders in the task file.
