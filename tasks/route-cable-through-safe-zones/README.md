# Run conduit for a concealed cable from a socket to a new back box

> Run conduit for a new cable from the socket to the empty back box on this wall panel, using the grey conduit
> lengths. The panel is for a house in the UK and will be boarded over. Leave any length you don't need on the bench.

A wall panel lies flat on the bench with a socket low on the left and an empty back box higher up and to the
right. The shortest way to join them is a single conduit length laid diagonally across the panel, and one of the
three lengths fits that gap. Once the panel is boarded over, nobody can see where the cable is. In the UK, a cable
concealed less than 50 mm inside a wall without earthed metal sheathing, earthed metal conduit or other protection
against nails and drills has to run in a zone where people expect cables: horizontally or vertically in line with
the accessory it serves, or within 150 mm of the ceiling or of a corner (BS 7671:2018, Regulation 522.6.202). A
diagonal run is where someone later drills to hang a shelf. The task checks whether the robot knows this trade rule
and follows it without being told. It has to choose a longer L-shaped route, either along level with the socket
and then up in line with the back box, or up in line with the socket and then along level with the back box.

Existing cable tasks give the route as a printed path, a taped lane, clips or pins (#15, #157, #173, #210), or say
what to avoid (#5). In this task the route is not drawn, and the instruction doesn't say which part of the panel is
off limits. The nearest wall task (#923) uses a detector to find a hidden pipe. In this task the robot lays the cable
run so that someone drilling later will avoid it.

## Fixed setup

Coordinates are in metres in the station's world frame: x points forward, away from the arm bases, y points
left (toward the left arm) and z points up. The table top is at z = 0.7475. All parts start unrotated, except where
noted, and there is no object jitter. `task.py` and `scoring.py` define the full geometry.

| Object | Type, size, material | Position |
| --- | --- | --- |
| Wall panel | Section of a factory-built timber-frame wall panel, lying flat. Its OSB face is 360 × 400 × 18 mm. The panel runs 360 mm up the wall (along x) and 400 mm along it (along y). Fixed. | x 0.11–0.47, y −0.20–0.20 |
| Sole plate | Timber, 45 × 400 × 45 mm, along the panel's floor edge (nearest the robot). Fixed. | x 0.065–0.11 |
| Socket | Existing single socket on an 80 × 80 mm box, 20 mm proud of the face. White faceplate with three pin slots and a rocker. Fixed. | centre (0.19, 0.12) |
| Back box | Empty 80 × 80 mm back box, 20 mm proud of the face. Grey, open, with a dark 64 mm mouth. Fixed. | centre (0.38, −0.1175): 190 mm higher than the socket and 237.5 mm to its right |
| Conduit lengths (3) | Rigid grey PVC conduit, 170 mm long, 25 × 25 mm square section, 60 g each. Free. | lying on the bench along x, centres (0.385, 0.26), (0.385, 0.32) left of the panel and (0.385, −0.26) right of it |

Reset by returning the three lengths to their bench positions. Nothing is consumed, and the panel and boxes are fixed.
The setup image is the first overhead camera frame.

## Scoring

Scoring uses the object poses at the end of the trial and checks the panel in top view.

- **Laid length:** any part of the length's outline lies over the panel face and its lowest point is within 30 mm
  of the face. Lengths on the bench or still held above the panel are ignored.
- **Zones:** each box has two strips. Its column runs up and down the wall directly in line with the box, and its row
  runs along the wall level with the box. Each strip is the 80 mm box width plus 10 mm on either side. A laid
  length is in a zone if its whole outline is inside one strip and its axis is within 10° of that strip's direction.
- **Joint:** two outlines (length to length, or length to box) are within 20 mm of each other.

| Score | Condition |
| --- | --- |
| **0** | Any laid length is outside every zone. This includes a diagonal, a run along the wall at a height level with neither box and a spare length left on the panel. A length is also at fault if its axis tilts more than 10°, its lowest point is more than 6 mm above the face (for example, it rests on a box or another length), or it is still moving at the end. |
| **1.0** | Otherwise, a chain of laid lengths joins the socket to the back box, with every joint within 20 mm. |
| **0.5** | Otherwise, a chain of laid lengths joins the socket to the back box, with every joint within 50 mm. |
| **0.25** | Otherwise, at least one laid length is within 20 mm of a box. |
| **0** | Otherwise, for example when nothing has been laid. |

A diagonal length joins the two boxes with joints of about 11 mm, but it is outside every zone and scores 0. On a
real bench, mark the strips on the panel with a pencil and square, then check each joint with a tape measure. A
length's end must be within 20 mm of the box it serves or of the length it meets.

## Run and score (Isaac Lab-Arena, inspect-robots)

From this folder, with the program's Isaac Lab-Arena environment active:

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Scripted demo (full score)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --max-steps 800 --no-prompt --store-frames --log-dir runs/demo

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream -E task=task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
  --scorer task_score --max-steps 2400 --no-prompt --store-frames --log-dir runs/dream

# Shortcut variants of the scripted demo: route_a, diagonal, stray, one_leg, open_joint, nothing
CSZ_VARIANT=diagonal inspect-robots run ... --log-dir runs/diagonal

# Offline checks (no simulator)
python -m pytest -q test_scoring.py
python reach_check.py
```

`--scorer task_score` reports the score at the last step (`SCORE_OVER = "end"`). There is no early success because
a later move could still spoil the run. Set `CSZ_DEBUG=1` to print every length's pose, zone and faults every 40
steps.

| Variant | What it does | Expected |
| --- | --- | --- |
| `full` | Right arm lays the back box's column. Left arm lays the socket's row. The spare length stays on the bench. | 1.0 |
| `route_a` | Left arm lays the socket's column, then the back box's row. | 1.0 |
| `diagonal` | One length goes straight across from box to box. | 0 |
| `stray` | The full route, plus the spare length left on the panel out of zone. | 0 |
| `one_leg` | Only the back box's column, butted against the box. | 0.25 |
| `open_joint` | The full route, with the column 37.5 mm short of the back box. | 0.5 |
| `nothing` | Both arms park. | 0 |
| `probe` | Left arm lays only the socket's row (one grip, a quarter turn, one placement). | 0.25 |

## Runtime and seeds

- The run limit is 2,400 control steps, 2 minutes at 20 Hz.
- The scripted demo finishes in 27 s of simulated time (540 steps). On an L40S, a 600-step demo run takes about
  2 minutes of wall time including Isaac Sim start-up. A full 2,400-step Dream-1 trial took 6.3 minutes.
- One fixed layout: `-E object_jitter=0`, so the seed doesn't change the start (`--seed 0` used).

## Validation

All runs used the station scene on an L40S with `object_jitter=0`.

- **Scripted demo: 1.0.** The right arm lays the back box's column and the left arm lays the socket's row. Both
  lengths end within 1 mm and 1° of the plan, with about 7 mm joints. Two reset-check trials and the recorded
  trial each scored 1.0.
- **Variants:** `route_a` 1.0, `diagonal` 0 (the length lies at −50°, joined to both boxes but out of zone), `stray` 0
  (the spare length lies on the panel out of zone), `one_leg` 0.25, `open_joint` 0.5, `nothing` 0 and `probe` 0.25.
  Each result matches the table above.
- **Dream-1: 0.0 in 2,400 steps.** It picked up and moved lengths. At the end, one length lay skewed across the open
  wall between the two boxes, in line with neither, so the trial scores 0.
- Offline, `test_scoring.py` (28 tests) and `reach_check.py` (worst IK error under 0.2 mm in every variant) pass.

## Fidelity

The scene uses rigid bodies only and has no cable. The conduit lengths stand in for the cable run. Real 20 mm round
conduit is modelled as a 25 mm square bar so that the gripper can hold it and it can't roll. Only a straight section
of the panel is modelled, and the battens of its service void are left out. The section has no ceiling or corner
zones because neither the top of the wall nor a corner is on it. The two boxes are fixed, and their 20 mm
projection makes the end joints visible to the camera. No electricity is involved, so a real trial is safe.

## Licences and credits

- This task (`task.py`, `scoring.py`, `test_scoring.py`, `reach_check.py`, `README.md`, `task.json`) is released
  under the MIT licence.
- It uses only `Box` primitives from `yam_arena.task_file`. It has no third-party models, textures or partkit code.
