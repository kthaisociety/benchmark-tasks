# Put the vaccine cool box in the shade and the solar charger in the sun

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena),
Isaac Lab-Arena) on the station scene: [task.py](task.py). No third-party models: every object is a primitive box.

The station's lamp hangs high on the robot's right and stands in for the sun. A tall fixed screen (a windbreak)
stands upright along x, and the lamp throws its shadow across the table to the robot's left, slanted away from the
robot. A blue cool box starts in full sun; a flat black solar charger starts lying in the shadow. The robot must
swap their situations: the cool box entirely in the shade (direct sun warms vaccines), lid included, and the charger
entirely in the sun. The ability tested is knowing where a shadow lies, from the light's direction and height and
from an object's own height, then putting one object inside it and the other outside it.

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/shade-and-sun/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 --store-frames \
  --log-dir ~/yam-eval/shade-and-sun-dream \
  --instruction "Move the blue cool box so it sits entirely in the shade, and the black solar charger so it sits entirely in the sun."
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 360`, which plays this file's
own `demo()` with the left arm: the charger out into the sun first (it lies where the box will go), then the box into
the shade. It finishes by about step 340.

## Scoring (`score()`, at the end of the episode)

- **Shade** is computed from the lamp (x 0.04, y -0.80, z 2.6875 m, the station's `LAMP_POS`) and the screen: on the
  plane at height *h*, it is the quadrilateral between the screen's shaded face and the screen's top edge projected
  from the lamp onto that plane.
- **Cool box, 0.5:** resting upright on the table (lowest point within 1 cm of it, tilt under 20 degrees) with all
  four corners of its lid in the shade at lid height.
- **Charger, 0.5:** lying flat on the table with none of its top corners in the shade.
- Score = the sum: 0, 0.5 or 1.0. An object still in the air earns nothing.

Shortcut and partial variants (`SHADE_DEMO=<mode>`), all run on Brev with the score shown:

| Mode | What it does | Score |
|---|---|---|
| `correct` | charger to sun, box to shade | 1.0 (two trials in a row: 1.0, 1.0) |
| `nothing` | no motion | 0 |
| `screen_end` | box set right beside the screen, but off its near end, in the sun | 0 |
| `edge` | box set where its base is in the ground shadow but its lid is in the sun | 0 |
| `charger_only` | only the charger moved | 0.5 |
| `box_only` | only the box moved (to a shaded spot beside the charger) | 0.5 |
| `hold` | both moved, but the box is held 3 cm above the shade at the end | 0.5 (charger only) |

## Runtime and seeds

- Run limit 1,200 steps (1 minute at 20 Hz); the scripted demo uses about 340.
- Layout fixed with `object_jitter=0`, so no seed is needed. The scripted demo is open loop and deterministic.

## Licence

Task under MIT. Built only from `yam_arena.task_file` primitives; no library models or third-party assets.
