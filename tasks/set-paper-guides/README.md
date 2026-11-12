# Set a paper cassette's two guides snug against the stack

A grey A4 printer cassette lies on the table with a stack of paper in its far-left corner, the corner the printer
feeds from. Its two orange paper guides are wide open: a side guide on the right and a rear guide at the near end.
Each guide is locked unless its release tab is squeezed from both sides; while squeezed it slides freely, and it locks
where it is let go. The robot must close both guides onto the stack and let go, without moving the stack.

Closing the guides is the last step of loading any printer, copier or envelope feeder: a gap lets sheets skew and jam,
and pushing a guide too hard drives the stack out of the feed corner. Unlike registering a part against fixed stops,
the stops here move to the part, and only while their latch is held open: the robot has to find that the guides are
locked, squeeze each tab across its slide direction, slide it until it meets the stack, and release it there. The two
guides sit at opposite corners of the tray and slide at right angles, so the demo works both at once, one per arm.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, the release-tab lock (an `on_step` hook), `score()` and the scripted `demo()` |
| `partkit.py` | Rigid parts built from boxes, with sliders along any axis, for yam_arena task files (see Licence) |
| `build_runs.sh` | Scripted checks: the full demo over two trials and every shortcut variant, with contact sheets |

`partkit.py` hooks `yam_arena.scene` and the embodiment's reset without modifying yam_arena, so the task runs with the
standard `yam_arena` embodiment and `task_score` scorer.

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Set the paper tray's two guides snug against the paper stack, then let go of them." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/paper-guides-dream
```

`task_score` prints the trial's score: `score()` at the end of the episode (`SCORE_OVER = "end"`).

## Scripted demo and shortcut checks

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \
  -E control_hz=20 --instruction "Set the paper tray's two guides snug against the paper stack, then let go of them." \
  --scorer task_score --max-steps 360 --epochs 2
bash build_runs.sh        # all variants below, with a contact sheet each
```

The demo (`demo()` in `task.py`) reads nothing at run time; the layout is fixed. The right arm works the side guide and
the left arm the rear guide, at the same time: each goes down around its tab with the fingers open, squeezes it, slides
the guide 3 mm (side) or 8 mm (rear) past where it meets the stack (the stack is against the walls, so the guide stops
and the pads slip), lets go, lifts and moves clear of the camera. It takes about 270 steps (13.5 s).

`DEMO_VARIANT` selects a variant; each was run and its end state checked:

| Variant | What it does | Score | Why |
|---|---|---|---|
| `full` | the demo above, two trials in a row | 1.0, 1.0 | both gaps 0.0 mm, stack moved at most 1.5 mm (into its corner), both arms clear |
| `nothing` | no arm moves | 0 | guides still open (35.5 and 48.5 mm) |
| `side_only` | only the side guide | 0.5 | side gap 0.0 mm, rear still 48.5 mm |
| `push` | closed fingers push each tab without squeezing it | 0 | locked guides do not move (35.5 and 48.5 mm) |
| `hold` | the full demo but never lets go | 0 | both gaps 0.0 mm, but the fingers are still on the tabs |
| `shove_stack` | presses on the stack's top and drags it toward the side guide | 0 | the stack cannot be dragged from above (0.0 mm moved); guides still open |

## Scoring

Evaluated every control step; the score at the end of the episode counts.

- 0 if at any time the stack moved more than 5 mm or turned more than 3 degrees in the tray, or was lifted more than
  10 mm, or the tray moved more than 5 mm (latched for the trial, cleared on reset).
- 0 while any finger pad is within 2 cm of either tab: both arms must have let go at the end.
- Otherwise, per guide: 0.5 if it is locked with at most 1 mm between its face and the stack's edge, 0.25 if at most
  5 mm, else 0. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

In the real world: a feeler gauge or ruler between each guide and the stack, tape marks for the stack's corner and the
tray's outline, and the top-camera video for lifting and release.

## Runtime and seeds

About 1 minute per trial for the full 1,200 steps, plus about 1 minute for Isaac Sim to start. The layout is fixed
(`-E object_jitter=0`); there are no seeds to choose.

## Fidelity

Rigid bodies built from boxes. The paper stack is one rigid block, so sheets cannot bow or fan out. Each guide is a
prismatic joint whose drive is stiff (20 kN/m, about 0.5 mm of give under the arm's push) while locked and free while
both finger pads of one gripper straddle its tab at most 6 cm apart; this stands in for the squeeze latch of a real
cassette. The tray is weighted (5 kg) so it stays put, as if seated in a printer. No contact-force cap.

## Real-world build

Any A4 printer cassette with squeeze-to-release side and rear guides, removed from its printer and clamped or weighted
on the table at the stated position, and a 100-sheet stack of A4 80 g/m2 paper. Mark the tray's outline and the
stack's far-left corner with tape.

## Licence

MIT. `partkit.py` is the version published with `tasks/dual-latch-lid` in this repository, with `finger_points()` added
from `tasks/bayonet-mate`'s copy; no third-party models or assets are used.
