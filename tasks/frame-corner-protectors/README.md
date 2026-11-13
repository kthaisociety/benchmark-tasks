# Push foam corner protectors onto all four corners of a framed picture

A framed picture lies face down on two carpet-covered blocks, ready to be packed for a move. The blocks run under its
two 30 cm sides, so all four corners stick out past them. Beside the frame lie four white foam corner protectors, two
on each side, each one beside a corner with its pocket facing it.

The instruction is: "Fit the four foam corner protectors onto the corners of the picture frame, then let go of them."

Movers and packers fit corner protectors to framed pictures and mirrors before wrapping them, so that a knock on a
corner doesn't split the mitre or crack the glass. A foam protector is a block with an L-shaped pocket cut into two
adjacent sides. The pocket is as tall as the frame is thick, and its foam squeezes the frame. So the protector has to
be held level with the frame, lined up with its edge within a few millimetres, and pushed on until the corner reaches
the back of the pocket. The push moves the frame unless something stops it. Once on, the protector stays on by itself.

## Scene

Positions in cm from the midpoint between the arm bases (x forward, y left). The layout is fixed.

| Object | Description |
|---|---|
| Padded blocks (2, fixed) | Pine 2x4 offcuts covered in grey carpet: 16 long (x), 8.9 wide, 3.8 tall, centred at x 30, y ±18.5 |
| Picture frame (1.1 kg) | Face down on the blocks. Oak moulding 3 wide and 2.5 thick; 30 (x) by 40 (y), centred at x 30, y 0. Glass, mount and hardboard back fill the opening. Corners at x 15 and 45, y ±20, each sticking out 7 past the blocks' ends |
| Corner protectors (4, 20 g each) | White cross-linked polyethylene foam: a block 7 x 7 and 4.5 tall, with an L-shaped pocket in two adjacent sides, 4.5 deep each way and 2.55 tall, 1 cm of foam above and below. The open edges are bevelled 6 mm over 10 mm. The pocket's floor stands 2 mm proud until pressed (about 6 N at 1.5 mm). Each lies on the table beside its corner, pocket facing it, the back of the pocket at the corner's x and at y ±30 |

## Scoring

The task file follows the trial after every control step:

- **Dropped:** the frame tilted more than 15 degrees at any time, or its front went 1.5 cm below the blocks' top (it
  slid off a block).
- **A corner counts** when a protector is fully on it and let go:
  - the corner within 4 mm of the back of the pocket in both directions;
  - the frame's front within 3 mm of the pocket floor's line, and the floor pressed down at least 0.8 mm, so the
    floor is under the frame and the roof over it and the foam grips;
  - the protector square and level with the frame (its rotation within 6 degrees of the frame's, turned for that
    corner);
  - each gripper's TCP (fingertip centre) at least 4 cm from the protector's middle.

`score()` gives 0 if the frame was dropped or doesn't end flat on both blocks (tilted more than 3 degrees).
Otherwise it gives 0.25 for each corner that counts. With all four counting it gives 1.0 once the frame and all four
protectors are still, and 0.75 before that. `success()` ends the episode once 1.0 has held for 1 s.

Any protector can go on any corner, as long as its pocket faces that corner.

## Run a trial

Following the task board's simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=352 \
  --instruction "Fit the four foam corner protectors onto the corners of the picture frame, then let go of them." \
  --scorer task_score --no-prompt --max-steps 2400 --store-frames --log-dir ~/yam-eval/corner-protectors-dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end of the episode. The layout is
fixed (`object_jitter=0`), so no seed is needed. A trial takes about TBD minutes of wall time on an L40S at 2,400 steps.

## Scripted demo

Run it with `--policy yam_scripted` instead of `--policy dream`. Each arm fits the two protectors on its own side
while the other arm holds the frame down, pressing on its back beside the middle. For each protector the arm:

1. pinches it across its closed wall (the fingers close along y on solid foam, clear of the pocket);
2. lifts it and brings it beside its corner at the frame's height, 5 cm out along y and 1.5 mm out along x so the
   closed wall clears the frame's end;
3. pushes it straight on along y until the corner reaches the back of the pocket, then presses it 2 mm into the
   corner along both edges;
4. lets go and lifts away.

Set `CORNERS_DEBUG=1` to print the frame and, for each protector, where its corner sits in the pocket and how far the
floor is pressed, every 5 steps. `CORNERS_SAG="dx,dy,dz"` shifts every fitting pose to make up for the loaded arm's
sag.

Shortcuts, with `DEMO_VARIANT` set in the environment:

| `DEMO_VARIANT` | What the arms do | Score |
|---|---|---|
| `full` | as above | TBD (expected 1.0) |
| `nothing` | wait | TBD (expected 0.0) |
| `two_only` | fit the left side's two protectors only | TBD (expected 0.5) |
| `keep_holding` | fit all four, but keep hold of the last | TBD (expected 0.75) |
| `no_hold` | fit all four with nobody holding the frame | TBD |

## Offline checks

`test_task.py` checks the geometry (the protector fits the open gripper, the squeeze, the bevels' height window, the
clearances to the blocks), the scoring rules on hand-built states, and the demo's reach with the robot's inverse
kinematics, without Isaac Sim:

```bash
PYTHONPATH=<path to yam-isaac-lab-arena> python -m pytest test_task.py -q
```

## Real-world build

Use a 30 x 40 cm oak picture frame (moulding 3 cm wide and 2.5 cm thick) with its glass, mount and back, and two pine
2x4 offcuts covered in carpet, 16 cm long, screwed or clamped to the table. Use four foam corner protectors made for
frames up to 1 inch (2.5 cm) thick: a 7 cm square block with a 4.5 cm deep pocket, which grips the frame when pushed
on (or cut them from 4.5 cm cross-linked polyethylene foam). Mark where each protector lies. Score with a tape measure
at the end. Fitting them is safe: the arms push a 20 g foam block onto a frame lying on blocks.

## Licence

This task is released under the MIT licence. `partkit.py` comes from `tasks/din-rail-mount` in this repository. It is
unchanged and released under the same licence. No third-party models or assets are used. Every object is built from
primitive shapes.
