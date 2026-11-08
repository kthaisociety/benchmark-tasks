# Open a crown-capped bottle on a mounted bottle opener

A short green glass soda bottle with a red crown cap stands in front of the right arm. Further forward a steel bottle
opener is fixed to the front of a dark post, as on a bar counter, with a flat tooth sticking out toward the robot. A cork
coaster lies near the middle of the table.

The instruction is: "Open the bottle on the mounted bottle opener, then stand the open bottle on the coaster and let go."

Bartenders and servers open crown-capped bottles on fixed openers many times a shift. The opener can't move, so the
bottle has to be brought to it. The robot lifts the bottle and slips the cap's edge over the tooth so the tooth sits
under it, then pulls the bottle down. The tooth holds the cap's edge and the cap pops off. Then the open bottle is stood
on the coaster. A crown cap can't be pulled off by hand: away from the tooth the cap stays crimped on, however it is
pulled.

## How it differs from nearby tasks

- **Paint-tin lids pried off with a hand tool** (a lid lifter's claw hooked under tabs, or a screwdriver levered under
  the lid). There the container stays put and a hand-held tool does the work. Here the tool is fixed and the robot
  moves the workpiece, a bottle carried in mid-air, onto it. The cap comes off from the bottle's own downward stroke, and
  nothing is levered by hand.
- **Uncorking a wine bottle with a corkscrew**: a hand-held tool screwed in and pulled, with the bottle held still.
- **Twist-off, flip-top, child-resistant and latched bottle caps**: opened by hand, with no tool.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, the crimp rule, `score()`, `success()` and the scripted `demo()` |
| `partkit.py` | Rigid parts built from boxes and cylinders, with pose writes and step/reset hooks, for yam_arena task files |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the bottle on the mounted bottle opener, then stand the open bottle on the coaster and let go." \
  --scorer task_score --no-prompt --max-steps 1200 --store-frames --log-dir ~/yam-eval/bottle-opener-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The layout is
fixed (`object_jitter=0`) and no seed is needed. A trial takes about 4 minutes of wall time on an L40S at 1,200 steps.

## Scripted demo

Run it with `--policy yam_scripted` instead of `--policy dream` and `--max-steps 450`. In the reset check, two trials in a row
both reach full score, and the episode ends on success at step 366 (18 s of simulated time). Set `BOTTLE_DEBUG=1` to print the bottle, the cap and the trial record
every 10 steps.

1. The right arm grips the bottle near the top of its body and lifts it 4.5 cm.
2. It carries the bottle in front of the opener, then forward until the cap's edge is over the tooth, 12 mm above it.
3. It lowers the bottle 2.5 cm. The tooth catches the cap's edge and, after 1 mm, the cap pops off and falls on the table.
4. It backs the bottle out, stands it on the coaster, lets go and moves the arm clear of the top camera.

The loaded arm sags about 1.6 cm sideways and 9 mm down from its commanded pose, so the demo aims that much high and to
the left (`SAG`).

Shortcuts, with `DEMO_VARIANT` set in the environment:

| `DEMO_VARIANT` | What the arm does | Score |
|---|---|---|
| `nothing` | waits | 0.0 |
| `skip_opener` | carries the closed bottle straight to the coaster and lets go | 0.0 |
| `pull_cap` | pinches the cap and pulls it straight up | 0.0 |
| `no_place` | opens the bottle and keeps holding it | 0.5 |
| `no_release` | opens it and stands it on the coaster without letting go | 0.75 |

## Scoring

The task file follows the trial after every control step and keeps a record that is cleared on reset:

- **Hooked:** the cap is upright within 20 degrees, with the tooth under its edge. The tooth's tip is 8 to 14 mm in from
  the cap's axis, past the neck and inside the cap's edge, within 1.6 cm of the tooth's centre sideways, with the cap's
  underside 0 to 6 mm above the tooth.
- **Popped:** while hooked, the bottle has come down at least 1 mm under the cap. Until then, the cap is put back on the
  neck after every step unless it is hooked. Pulling, pushing or knocking it anywhere else does nothing.
- **Tipped:** the bottle tilted more than 30 degrees at any time, or fell off the table.

`score()` gives 0 if the bottle tipped. Otherwise it gives 1.0 when the cap has popped and the bottle stands on the
coaster (axis within 3 cm of its centre, base on it, within 10 degrees of upright), still, with both grippers' TCPs at
least 6 cm from its grip point. It gives 0.75 when popped and on the coaster, 0.5 when popped, 0.25 when hooked, and
0 otherwise. `success()` ends the episode once 1.0 has held for 1 s.

## Real-world build

Use a 200-275 ml crown-capped glass soda bottle and a standard cast or stainless wall-mount bottle opener screwed to a
post or block at the height in `task.json`. Score from a video and a ruler. Opening the bottle is safe: the arm only
pulls the bottle down by a few centimetres. Use a full plastic crown-capped bottle if glass on the bench isn't allowed.

## Licence

This task is released under the MIT licence. `partkit.py` comes from `tasks/din-rail-mount` in this repository. It is
unchanged and released under the same licence. No third-party models or assets are used. Every object is built from
primitive shapes.
