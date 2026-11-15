# Open a crown-capped bottle on a mounted bottle opener

A full 275 ml long-neck glass bottle with a red crown cap stands in front of the right arm. Further forward a cast steel
wall-mount bottle opener is screwed to a dark post, as on a bar counter. Its jaw sticks out toward the robot. A cork
coaster lies near the middle of the table.

The instruction is: "Open the bottle on the mounted bottle opener, then stand the open bottle on the coaster and let go."

Bartenders and servers open crown-capped bottles on fixed openers many times a shift. The opener can't move, so the
bottle has to be brought to it. The robot lifts the bottle and slides the back of the cap into the jaw's slot, so that
the catch lip at the bottom of the slot is under the lower edge of the cap's skirt. Then it pulls the bottle down. The
lip holds the back of the skirt while the bottle drops, so the skirt is bent up off the glass bead and the cap pops
off. Then the robot stands the open bottle on the coaster. A crown cap can't be pulled off by hand. Away from the catch
lip, the cap stays crimped on however it is pulled.

## The bottle, the cap and the opener

All sizes are in the model (`task.py`) and in `task.json`.

- **Bottle:** a 275 ml long-neck glass bottle with a standard 26 mm crown finish, 450 g full of beer (or still water
  for lab trials). It is 18.5 cm tall to the top of the finish. The body is 5.8 cm across and 11 cm tall. The shoulder narrows to a lower neck 3.2 cm across
  (13 to 16.2 cm up), then to 2.35 cm under the finish. The finish's bead is 2.64 cm across and 4.3 mm tall.
- **Crown cap:** crimped on, 2.88 cm across the skirt and 6 mm tall. Its skirt reaches 0.7 mm below the bead, and its
  lower edge stands 1.2 mm out from the bead all round. That edge is what a catch lip gets under.
- **Opener:** a back plate 6 cm wide, 7 cm tall and 5 mm thick, on a dark-stained hardwood post 4.5 cm wide and
  2.5 cm deep that stands on the table behind it, clamped to the table, and runs up to the plate's top, 23.5 cm. A jaw 1.8 cm wide sticks out of the plate's front face
  toward the robot. From the bottom up it has:
  - **The catch lip:** 4 mm thick, its top 21.4 cm above the table, reaching 3.2 cm out from the plate (to x 41.35 cm).
  - **The slot:** 8.5 mm tall, its back wall 3.1 cm out from the plate (x 41.45 cm). The lip sticks 1 mm out of the
    back wall. When the cap's skirt touches the back wall, the lip is 1 mm under the skirt's edge and 0.2 mm clear of
    the bead.
  - **The upper prong:** 3 mm thick, reaching 3.9 cm out from the plate (to x 40.65 cm), 8 mm over the cap's top. It
    is the fulcrum the cap tips against.
  - **Placement:** the plate's front face is at x 44.55 cm (`FACE_X` in `task.py`), the jaw is centred on y -20 cm,
    and the plate spans 16.5 to 23.5 cm above the table. The post stands behind the plate, from x 45.05 to 47.55 cm.

## How it differs from nearby tasks

- **Paint-tin lids pried off with a hand tool** (a lid lifter's claw hooked under tabs, or a screwdriver levered under
  the lid). There the container stays put and a hand-held tool does the work. Here the tool is fixed and the robot
  moves the workpiece, a full bottle carried in mid-air, into it. The cap comes off from the bottle's own downward
  stroke, and nothing is levered by hand.
- **Uncorking a wine bottle with a corkscrew:** a hand-held tool is screwed in and pulled while the bottle is held still.
- **Twist-off, flip-top, child-resistant and latched bottle caps:** these are opened by hand, with no tool.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, the crimp rule, the let-go check, `score()`, `success()` and the scripted `demo()` |
| `partkit.py` | Rigid parts built from boxes and cylinders, with pose writes and step/reset hooks, for yam_arena task files |
| `check_release.py` | Checks the let-go test without the simulator (yam_arena and NumPy only): see [Let go](#let-go) |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Open the bottle on the mounted bottle opener, then stand the open bottle on the coaster and let go." \
  --scorer task_score --no-prompt --max-steps 1200 --store-frames --log-dir ~/yam-eval/bottle-opener-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The layout is
fixed (`object_jitter=0`), so no seed is needed. A trial takes about 3.5 minutes of wall time on an L40S at 1,200 steps.

## Scripted demo

Run it with `--policy yam_scripted` instead of `--policy dream`, and `--max-steps 500`. It reaches full score: the cap
pops at step 263, and the episode ends on success at step 413 (20.6 s of simulated time). Set `BOTTLE_DEBUG=1` to print
the bottle, the cap and the trial record every 5 steps.

1. The right arm grips the bottle by its lower neck (its TCP 15.4 cm above the bottle's base) and lifts it 3.5 cm.
2. It carries the bottle in front of the opener at slot height. Then it moves it straight back until the back of the
   cap's skirt reaches the back of the slot, so the catch lip is under the skirt's edge.
3. It pulls the bottle down 6 mm while pressing it 4 mm further back into the jaw, as a person does. The lip holds the
   back of the skirt, which is bent up off the bead, and the cap springs off toward the robot and falls on the table.
4. It lowers the bottle 12 mm clear of the jaw, backs it out, stands it on the coaster, lets go and moves the arm clear of the top camera.

Loaded with the full bottle, the arm misses its commanded pose by a pose-dependent offset: in front of the opener it
lands 16.5 mm further forward and 11.6 mm low. The demo aims that much the other way (`SAG` at the opener, `SAG_DOWN`
over the coaster).

Shortcuts, with `DEMO_VARIANT` set in the environment:

| `DEMO_VARIANT` | What the arm does | Score |
|---|---|---|
| `nothing` | waits | 0.0 |
| `skip_opener` | carries the closed bottle straight to the coaster and lets go | 0.0 |
| `pull_cap` | pinches the cap and pulls it straight up | 0.0 |
| `under_prong` | stops the cap 6 mm short of the slot, under the prong's tip, and pulls down | 0.0 |
| `no_place` | opens the bottle and keeps holding it | 0.5 |
| `no_release` | opens it and stands it on the coaster without letting go | 0.75 |

## Scoring

The task file follows the trial after every control step and keeps a record that is cleared on reset:

- **Caught:** the back of the cap's skirt is on the catch lip. The lip's tip is 12.8 to 14.2 mm in from the cap's axis
  (past the bead and inside the skirt), the cap's axis is within 6 mm of the jaw's centre line sideways, and the
  skirt's lower edge is 0 to 2.5 mm above the lip (-0.5 mm allows for contact), with the bottle within 20 degrees of
  upright.
- **Popped:** while caught, the skirt's edge on the opener side has been bent at least 1.5 mm up off the bead. The cap
  then springs 10 mm forward and 5.5 mm up within one control step, clear of the bead and the prong, flips and falls.
  Until then, the cap is put back on the finish after every step unless it is caught. Pulling, pushing or knocking it
  anywhere else does nothing.
- **Tipped:** the bottle tilted more than 30 degrees at any time, or fell off the table. This holds before the pop
  too: a full bottle of beer tipped that far and then opened foams over, and the opener is used with the bottle upright.
- **Let go:** once the cap has popped, after every step: no part of either arm is within 1 cm of any part of the
  bottle. See [Let go](#let-go).

`score()` gives 0 if the bottle tipped. Otherwise it gives:

- **1.0** when the cap has popped and the bottle stands on the coaster (axis within 3 cm of its centre, base on it,
  within 10 degrees of upright), still (moving under 1 cm/s, turning under 0.5 rad/s), and let go: no part of either arm (fingers, gripper, wrist or arm) within 1 cm
  of any part of the bottle, body included.
- **0.75** when popped and on the coaster, but still held, touched or moving.
- **0.5** when popped.
- **0.25** when caught.
- **0** otherwise.

`success()` ends the episode once 1.0 has held for 1 s.

### Let go

Full score needs both arms clear of the whole bottle, not just of the place where the demo grips it. A hand that
holds the bottle by its body, leans on it or keeps a finger on it gets 0.75.

- **The arms:** every moving link of both arms (`link1` to `link5`, `gripper`, `tip_left`, `tip_right`), as the
  convex collision hulls the simulator collides with. These are the YAM URDF's CoACD hulls, which the arm's USD is
  converted from unchanged. The hulls' surfaces are sampled once, at reset, every 3 mm on the gripper, fingers and
  wrist and every 6 mm on the upper and lower arm. Points inside another hull of the same link are dropped.
- **The bottle:** the exact glass the model is built from (`BOTTLE_PROFILE`): body, label band, shoulder, necks and
  finish, as stacked cylinders in the bottle's own frame.
- **The test:** after every step, the arms' sample points are moved to the links' current poses and their signed
  distance to the bottle is computed exactly. Every hull point lies within 0.6 × the spacing of a sample, so each
  link's distance is reduced by that much first. The bottle counts as let go when the result is at least 1 cm, so an
  arm part 1 cm or closer never passes.

`python check_release.py` poses the arms with the URDF's kinematics around the bottle standing on the coaster and runs
the same test, with the old test (each TCP more than 6 cm from the grip point on the neck) for comparison. A negative
gap means a hull overlaps the glass: in these kinematic poses closed fingers sit at their closed target, where in the
simulator they stop on the glass.

| Case | Nearest part | Gap | Let go | Old test |
|---|---|---|---|---|
| Right gripper closed on the lower neck | right `tip_right` | -17.5 mm | no | no |
| Right gripper half open round the neck | right `tip_left` | +5.7 mm | no | no |
| Right gripper fully open round the neck (the demo's let-go) | right `tip_right` | +28.7 mm | yes | no |
| Left fingertips resting on the body 7 cm up, TCP 10.1 cm from the grip point | left `tip_right` | -1.7 mm | no | yes |
| Left fingertips 6 mm off the body | left `tip_right` | +4.9 mm | no | yes |
| Left fingertips 2 cm off the body | left `tip_right` | +19.8 mm | yes | yes |

## Real-world build

Use a full 275 ml long-neck crown-capped glass bottle with a 26 mm crown finish. Use a cast or stainless jaw-type
wall-mount bottle opener (a back plate and a jaw with a catch lip under an upper prong). Screw it to a post or block
that stands on the table, with the catch lip's top at the height in `task.json` and the jaw facing the robot. If no
cast opener matches the jaw's dimensions, bend and weld one from 3 mm steel. With the bottle held upright and pressed
back into the jaw, a pull down pops the cap. To judge the 0.25 level, film the jaw from the side (lip under the skirt) and from above (cap on the jaw's centre
line) as well. Score
from a video and a ruler. Opening the bottle is safe: the arm only pulls the bottle down a few millimetres with the
cap in the jaw. Use a crown-capped PET bottle if glass on the bench isn't allowed.

## Licence

This task is released under the MIT licence. `partkit.py` comes from `tasks/din-rail-mount` in this repository. It is
unchanged and released under the same licence. No third-party models or assets are used. Every object is built from
primitive shapes.
