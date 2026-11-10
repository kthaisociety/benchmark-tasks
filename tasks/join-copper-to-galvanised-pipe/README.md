# Join a new copper section between galvanised and copper pipe

A pipe run lies on a bench in front of the robot: an existing galvanised steel pipe on the left, a new copper section in
the middle and an existing copper pipe on the right, with an open 12 cm gap at each joint. A tray nearer the robot
holds three fittings of the same size: a galvanised steel coupling, a dielectric union (a brass half and a galvanised
half separated by a black insulating ring, with a steel nut) and a copper coupling. The instruction only says to seat a
fitting in each gap.

A plumber tying new copper into old galvanised pipe must keep the two metals apart: a plain coupling across them
corrodes the joint (galvanic corrosion), so the steel-to-copper joint takes the dielectric union, with its brass end
soldered to the copper. The copper-to-copper joint takes the plain copper coupling; a steel fitting there corrodes too,
and the union would put its steel half against copper. With one union for two joints, "always use the special part"
fails, and the fitting nearest each gap is the wrong one. The robot has to tell the metals apart by sight, match a
fitting to each metal pair, and turn the union a quarter turn in the right direction (its brass end starts pointing at
the robot; turning it the other way puts it in backwards).

## Layout

Positions in cm, forward of the line through the arm bases (61 cm apart) and left or right of their midpoint (robot's
view); heights above the table top. Exact geometry is in `task.py` and `scoring.py`.

- **Pipe bench** (fixed; one 20 kg body), centred 38 cm forward: a wooden board 10 x 64 x 1 cm. Pipe run along it,
  axis 5.72 cm high, pipes 4 cm across, on 3 x 3 cm steel posts at 26 and 4.5 cm either side of centre:
  galvanised steel pipe 31 to 21 cm left; gap A 21 to 9 cm left; new copper pipe 9 cm left to 9 cm right; gap B 9 to
  21 cm right; existing copper pipe 21 to 31 cm right.
- **Saddles**, one under each gap: a grey steel plate 7 cm (front to back) x 12 cm, top 3 cm high, with two dark rails
  1 cm wide and 0.8 cm tall along the pipe line, inner edges 1.6 cm either side of it. A 5 cm fitting laid on them sits
  with its axis at the pipes' axis height (the union, on its 5.4 cm nut, 2.5 mm higher).
- **Tray** (fixed, dark plastic), 13 x 32 x 1.2 cm, centred 20 cm forward; three slots with the same rails, running
  front to back, at 10 cm left, centre and 10 cm right.
- **Fittings**, lying in the slots with their axes pointing at the robot, each 9 cm long and 5 cm across:
  - left slot: galvanised steel coupling, 180 g, with 5.2 cm bands at its ends;
  - centre slot: dielectric union, 300 g: a 3.3 cm brass half (nearest the robot), a black insulating ring (6 mm), a
    darker steel nut 5.4 cm across (1.8 cm long) and a 3.3 cm galvanised half;
  - right slot: copper coupling, 150 g, with a slightly darker stop ring in the middle.

## Files

| File | What |
|---|---|
| `task.py` | The yam_arena task file: objects, `score()`, `success()` and the scripted demo with its shortcut variants |
| `scoring.py` | The scoring rules as pure Python (NumPy only) |
| `test_scoring.py` | Scoring tests with synthetic poses (full score, each partial level, each shortcut) |
| `test_task.py` | Simulator-free checks of `task.py`: loading, slot clearances, arm reach of every plan, plan end states |
| `partkit.py` | Rigid parts built from boxes and cylinders, copied from `tasks/dual-latch-lid` |

## Run a trial

In the Isaac Lab-Arena environment from the simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this
folder:

```bash
# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Join the new copper pipe into the line: seat a fitting from the tray in each of the two gaps, then let go." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir ~/yam-eval/copper-galv-dream

# The scripted demo (full score); DU_VARIANT plays a shortcut instead
DU_VARIANT=demo inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Join the new copper pipe into the line: seat a fitting from the tray in each of the two gaps, then let go." \
  --scorer task_score --max-steps 600 --store-frames --log-dir ~/yam-eval/copper-galv-demo

for log in ~/yam-eval/copper-galv-demo/*.json; do inspect-robots video "$log"; done
```

`task_score` prints the trial's score: 1.0 on success (both joints right, held for 1 s), otherwise `score()` at the
last step.

Scripted variants (`DU_VARIANT`) and the scores they should reach:

| Variant | What the arms do | Expected |
|---|---|---|
| `demo` | union into the left gap, brass end to the copper; copper coupling into the right gap | 1.0 |
| `reversed` | as `demo`, but the union turned the other way (brass end to the steel pipe) | 0 |
| `nearest` | each arm takes the fitting nearest its gap: steel coupling left, copper coupling right | 0 |
| `union_b` | the union into the right (copper-to-copper) gap | 0 |
| `only_a` | the union only, correctly | 0.75 |
| `only_b` | the copper coupling only | 0.25 |
| `held` | both fittings placed correctly but never let go | 0 |
| `nothing` | both arms park | 0 |

`DU_TRACE=<file>` appends the fittings' poses, the grippers' tool points and the score every 10 steps.

Local checks, without the simulator:

```bash
python -m pytest -q test_scoring.py   # NumPy and pytest only
python -m pytest -q test_task.py      # also needs yam_arena importable
```

## Scoring

Judged at the last step (`SCORE_OVER = "end"`). A fitting is **seated** in a gap when it rests still in the gap's
saddle: centre within 1 cm of the pipe line (front to back) and within 1.5 cm of the gap's centre along it, axis height
within 6 mm of resting on the saddle rails, axis within 15 degrees of the pipe line, moving under 1 cm/s, and **let
go**: no point of either gripper (wrist to pad tips) within 5 cm of the fitting, and the fitting's centre moved at most
2 mm over the last second (20 control steps). The grippers' positions come from the arms' joint angles through
`yam_arena.ik.tcp_pose`; if the arms can't be read, nothing counts as let go.

- **0** if the bench was moved 1 cm or turned 2 degrees at any time in the trial (latched; cleared on reset).
- **0** if at the end a fitting that does not belong is **in** a gap (centre within 5 cm front to back and 7 cm along
  the pipe of the gap's centre, at most 4 cm above the pipe axis: set down, propped on a pipe end or lying across):
  the steel coupling in either gap, the copper coupling in the left gap, the union in the right gap, or the union in
  the left gap with its brass end within 60 degrees of pointing at the steel pipe.
- Otherwise **0.75** for the union seated in the left gap with its brass end toward the copper section, plus **0.25**
  for the copper coupling seated in the right gap. Both: **1.0**.

On a real bench: a tape measure for the positions, and a photo from above for the union's direction.

## Runtime and seeds

The scripted demo seats both fittings in about 20 s of simulated time (about 400 steps; the right arm waits for the
left to clear the tray), then the trial ends with success. Wall time on the instance: about 45 s for the scripted demo
and about 5 min for a full 1,200-step Dream-1 trial. One fixed layout (`-E object_jitter=0`); there are no seeds to choose.

## Results

All runs in the simulator, fixed layout, 320 x 176 cameras at 20 Hz.

- **Scripted demo:** 1.0 in both reset-check trials and in the recorded trial (union in gap A with its brass end
  toward the copper, copper coupling in gap B, both released and still).
- **Shortcut variants:** `reversed` 0 (union in gap A backwards), `nearest` 0 (steel coupling in gap A, the fitting
  nearest each gap), `union_b` 0 (union in the copper-to-copper gap), `only_a` 0.75, `only_b` 0.25, `held` 0 (both
  right but never released), `nothing` 0.
- **Dream-1:** 0.0 in 1,200 steps. It reached over the tray several times but never seated a fitting in either gap.

## Fidelity

Pipes and fittings are rigid cylinders. A fitting counts as fitted when it lies in the saddle between the pipe ends,
with 1.5 cm of play at each end: there are no threads, no solder and no push-on, so the pipe ends do not enter the
fittings. Pipe size is scaled up (4 cm pipe, 5 cm fittings) so the grippers can hold the fittings, and the fittings are
lighter than real ones. The bench is one heavy body (20 kg) rather than bolted down.

## Real-world build

Pipe stubs and fittings can be real copper and galvanised pipe and fittings of about 1-1/4" (or painted 3D-printed
stand-ins of the dimensions in `task.py`), with a real dielectric union. The saddles and tray are a plate with two
8 mm strips glued on per slot; screw the bench board to the table.

## Licences

The task (code and scene) is released under the MIT licence. `partkit.py` is copied from `tasks/dual-latch-lid`
(itself derived from the partkit in `tasks/din-rail-mount`), under the same licence. No third-party models or assets
are used: every object is built from primitive shapes in `task.py`.
