# Uncover a buried pipe with a spade without touching it

Dig the soil off the top of the yellow pipe between the white marks until the top of the pipe is bare along at least 8 cm. Never let the spade touch the pipe.

Before a trench is dug near a marked gas, water or power line, the crew digs by hand at the locate marks and
uncovers the line ("potholing"), so they know exactly where it runs and how deep it is before a machine
comes near. The digging has to stop short of the line: a spade that gouges a plastic gas pipe or nicks a
cable sheath causes a leak or a fault, often found only later. This trainer tests that skill at tabletop
scale. The robot works a blade through fill toward an obstacle it can only partly see, taking a shallower
cut each time it gets closer, and stops a few millimetres short. The scorer ignores how much soil moves or
where it goes. It looks at two things: how much of the pipe's top is bare, and whether the blade ever
reached the pipe.

Closest existing work: scooping or sweeping loose material from one container to another (pull request
#666 scoops beads into a bowl; the previous cohort's "fill the bag with rice using the scoop"; EFE's Scoop
Pour and the RoboDojo and BEHAVIOR sweeping tasks). Pull request #887 moves spoil piles back from a
trench edge, and #425 lowers a cable into a trench. None of them digs down toward a hidden object, or
penalises the tool for reaching it.

## Fixed setup and reset

Bimanual YAM station, worktop 747.5 mm above the floor. Positions are millimetres from the midpoint between
the two arm bases: X forward (away from the arms), Y toward the left arm, Z up from the worktop. Nothing is
rotated. `task.py` holds every dimension and position below.

| Object | Construction and start |
| --- | --- |
| Soil box | Fixed, open-topped, grey: an inner channel 180 mm long (X 210 to 390) and 54 mm wide (Y −237 to −183), standing on the worktop, which is its floor. 6 mm walls; the two side walls and the far wall are 60 mm high, the near wall (X 204 to 210, nearest the arms) only 24 mm high. White locate marks, 4 mm wide, run across the top of both side walls at X 250 and X 350, bounding the marked stretch. |
| Gas pipe | Fixed, yellow, 25 mm across, lying on the worktop along the middle of the channel (axis Y −210), from the near wall's inside face (X 210) out through the far wall to X 420. Its top is at Z 25, 1 mm above the near wall. |
| Soil | 156 cubes, 12 mm on a side and 1.2 g each (wooden craft cubes in a physical copy), brown, in 13 rows along the channel at X 216.5 to 368.9 (12.7 mm pitch), so the last 15 mm before the far wall is empty. Each row has two outer columns (Y −229.5 and −190.5), four cubes high from the worktop (centres at Z 6.1, 18.3, 30.5, 42.7), beside the pipe. It also has two inner columns (Y −216.5 and −203.5), two cubes high, standing on the pipe (centres at Z 31.6 and 43.8). The soil is about 25 mm deep over the pipe's top. |
| Spoil board | Fixed, grey, 100 × 120 × 4 mm, centred at X 150, Y −210, on the worktop just past the near wall. |
| Spade | Loose, 70 g: a green 16 × 24 × 90 mm handle on a steel blade 3 mm thick, 42 mm wide and 35 mm tall. The blade's faces point toward and away from the arms. It stands in its holder with the blade's bottom edge on the slot floor, the handle's centre at X 300, Y −330, the handle's top 127 mm above the worktop. A physical copy can be 3D printed. |
| Spade holder | Fixed, dark grey: two 19 × 60 × 20 mm jaws either side of a 5 mm slot, centred at X 300, Y −330. |

The left arm isn't needed, and there are no consumables. To reset, stand the spade back in its holder,
rebuild the cubes in the lattice above (a printed 12.7 mm grid card at the near wall helps), and wipe and
re-chalk the pipe. The simulated trial always starts from that lattice (`object_jitter=0`). The setup image
is the trial's first top-camera frame.

## Scoring

- **Strike:** any contact between the spade and the pipe scores the whole trial **0**, whenever it happens.
  On a physical copy, rub chalk on the pipe before the trial; any chalk on the spade or scuff on the pipe is
  a strike. In simulation, a strike is any point on the spade's faces within 1 mm of the pipe's surface. The
  scorer checks this at every control step and at three poses interpolated between steps, so a fast swipe
  through the pipe is still caught. The grippers may touch the pipe: a hand on a pipe does no harm.
- **Bare length:** the pipe's top is the 10 mm wide strip along its crown. On a physical copy, draw two
  lines on the pipe 5 mm either side of the top. The marked stretch is judged in twenty 5 mm segments. A
  segment is bare when, seen straight down, no soil cube lies over the strip there.
- **Credit**, the best level reached during the trial, counted only while every cube in the box has been
  still for 1 s:
  - **1.0**: at least 80 mm (16 of 20 segments) bare. The trial ends there.
  - **0.5**: at least 40 mm bare.
  - **0.25**: at least 10 mm bare.
  - **0**: less, or any strike.

How the soil is moved doesn't matter: with the spade or by hand, onto the board, the table or anywhere.
Cubes beside the pipe, below its top strip, don't count against it. To check a real trial, take a photo from
straight above with a ruler along the wall, and count the bare segments.

## Run and score on Isaac Lab-Arena

From this folder, with the program's Isaac Lab-Arena environment active:

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
python run.py run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "$instruction" --scorer task_score --max-steps 2400 --seed 0 --no-prompt \
  --store-frames --log-dir runs/demo
python run.py run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --scorer task_score --max-steps 2400 --seed 0 --no-prompt \
  --store-frames --log-dir runs/dream
python -m pytest -q test_scoring.py
```

`run.py` wraps the `inspect-robots` CLI. It authors the compound soil box, pipe, spoil board, holder and
spade and the 156 loose cubes. It also lets a demo waypoint pitch the grip (this demo keeps the blade
upright) and scores each control step with `scoring.Judge`. Arena records the final `task_score`. Each trial
also writes `dig-1.jsonl`, with these fields for every control step:
- the score, and whether the pipe has been struck
- the spade's current and closest gap to the pipe
- the bare length
- the TCPs and the spade's pose
- cube counts: in the box, dragged out, and off the table

`remote_run.sh` runs one trial on the program's GPU VM. `trace_summary.py` and `contact_sheet.py` summarise
a trial's trace and frames.

Each trial allows 2,400 actions (120 simulated seconds at 20 Hz). The scripted demo finishes in about 1,170 actions (58.6 simulated seconds). On the shared L40S that took 265 s of wall time at 320 × 176 camera frames.

## Fidelity

The soil is 12 mm rigid wooden cubes, which hold a cut face the way damp soil does but are much coarser than
real soil. The pipe and box are fixed rigid colliders. A strike is judged geometrically, not by contact
force, so even a feather touch counts. The spade is a rigid compound with rubber-like friction (1.2 static)
on the handle. The YAM's wrist is compliant: under drag the blade swings back by up to 10° and rides up, and
with the spade held the arm settles about 1.5 mm below its target. A policy has to leave margin for both,
as a person does with a real spade.

## Validation

The scripted YAM demo (`yam_scripted`, seed 0, `object_jitter=0`, 320 × 176 frames) scored **1.0** at
58.6 simulated seconds. It made seven straight drags toward the arms with the blade upright: one 12 mm over
the pipe, one at 6 mm and five at 5 mm. They moved 102 of the 156 cubes out over the near wall and left the
whole 100 mm stretch bare. No strike: the blade's closest approach to the pipe was 3.3 mm. The score rose to
0.25 at 24 s and 0.5 at 42 s on the way.

Earlier commissioning runs show what the strike rule catches. Aiming the blade 4 mm over the pipe, before
allowing for the 3 mm grip offset and the arm settling 1.5 mm low, put the blade on the pipe as it went down.
That trial scored 0, even though it went on to bare the whole stretch.

The 16 offline tests (`test_scoring.py`) cover:
- every credit level
- the starting layout scoring 0
- a 2 mm skim not counting as a strike, and a 0.5 mm touch counting
- a fast swipe through the pipe between control steps being caught
- a strike erasing credit already earned
- moving soil not latching credit
- the demo staying within the arm's reach and at least 4 mm over the pipe

The demo video comes from a repeat of that demo at 640 × 360. It also scored 1.0 at 58.6 s, with the same
3.3 mm closest approach.

Dream-1 (`dream`, seed 0, 320 × 176 frames) scored **0.0** in 2,400 actions. It never picked up the spade or
moved any soil, and it never touched the pipe.

The physical copy hasn't been built.

## Licence

Released under the MIT licence. The scene uses only primitive shapes authored in `task.py` and `run.py`, no
third-party assets. It runs on Isaac Lab-Arena and the `yam_arena` embodiment, under their own licences.
