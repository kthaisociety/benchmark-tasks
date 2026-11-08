# Route a train round a passing loop with one points lever

The blue ball is a train, and the red wagon is stopped on the main line. Send the ball round the loop and back onto the main line: pull the black points lever toward you before the ball reaches the first junction, then push it back once the ball is past that junction and before it reaches the end of the loop. Never move the lever while the ball is on a junction, and do not touch the track or the ball.

Yard crews still line hand-thrown switches ahead of cars being shoved, and operating rules forbid throwing a switch while a wheel is on the points. The same skill runs manual diverter gates on conveyors and baggage lines. In each case the worker never touches the moving load: they predict when it will arrive and act in the gap before and after it. Here the ball is the moving load and one lever works both ends of a passing loop through a rod. Pulled, the entry points turn into the loop and the loop's exit is set into a sand drag, a short dead end. Normal, the entry runs straight and the exit leads back onto the main line. So the first throw must land before the ball reaches the entry, and the second inside the window while the ball is in the loop. The nearest existing tasks either push a track switch once for a coasting cart, or time a single release into a moving target. None needs two timed throws that must bracket the moving object's position.

## Fixed setup and reset

Use the calibrated bimanual YAM station with its 747.5 mm worktop. Coordinates are millimetres from the midpoint between the arm bases. X points forward, Y points toward the left arm, and heights are above the table. `layout.py` gives every dimension and `task.py` builds the scene from it.

| Item | Construction and start |
| --- | --- |
| Track board | Grey board, 214 × 660 × 12 mm, centred at X/Y 490/0. Its top is the running surface. |
| Channels | White polymer walls, 8 mm thick and 22 mm high, with 36 mm clear between them. The main line runs along X 440 from Y 300 to the buffer stop at Y −300. The loop runs along X 540. The entry diagonal leaves the main line at 45° from Y 170 on its centreline. The exit diagonal leaves the loop at 45° from Y −80 and joins the main line 100 mm further on. Past the exit junction, the loop runs straight on into the sand drag (yellow floor), which ends at a wall at Y −250. A white line across the main line at Y −249 is the exit line. Dark sleeper marks are decoration only. |
| Points blades | Two steel flappers, 6 × 64 × 20 mm, each hinged on a vertical pin at the frog, the point where its two routes divide. Each blade lies upstream of its pin. Entry blade A is hinged at X/Y 462/117. When normal, it lies in the gap in the main line's far wall. Pulled, it turns 38.5° across the main line and deflects the ball into the loop. Exit blade B is hinged at X/Y 518/−133. When normal, it is turned 38.5° across the loop and deflects the ball onto the exit diagonal. Pulled, it lies in the gap in the loop's near wall and the ball runs straight on into the sand drag. |
| Points lever | Black steel lever, 100 mm from pivot to handle centre, 150 g, with a white polymer handle 22 × 50 × 26 mm. The pivot is at X/Y 275/−130, height 40 mm, on a dark 80 × 60 × 34 mm frame. It swings fore and aft between stops 22° either side of vertical. It starts normal, leaning away from the arms, and is pulled toward them. It has no spring: past vertical, its own weight carries it onto the other stop. A rod beneath the board links it to both blades. |
| Train | Blue hard-rubber ball, 28 mm diameter, 30 g. It starts at X/Y 440/270 on the main line, behind the yellow gate. |
| Gate | Yellow polymer bar, 32 × 8 × 28 mm, across the main line at Y 245. It lifts 45 mm 5 s after the start and releases the ball. |
| Wagon | Red polymer block, 30 × 60 × 32 mm, fixed on the main line centred at Y 0, between the two junctions. |

Reset by returning the lever to normal (leaning away from the arms), which also sets both blades. Then put the ball back at Y 270 behind the lowered gate. Nothing is consumed. There is no object jitter. Use the initial top camera frame as the setup image.

## Scoring

The ball is **on a junction** while its centre is within 19 mm (its radius plus 5 mm) of the area that junction's blade sweeps. The lever is **over** when it is within 10% of its throw from either stop.

- **1.0** if the ball goes round the loop and back onto the main line, with its centre crossing the white exit line (Y −249, within 20 mm of the main line's centreline).
- **0.5** if the ball leaves the main line through junction A into the loop but does not reach the exit line. For example, the lever was never restored and the ball ran into the sand drag.
- **0** otherwise. This includes the ball running into the wagon. Any of these faults also scores 0, whatever happens afterwards:
  - the lever is partway over (between 10% and 90% of its throw), or moves more than 10% of its throw, while the ball is on a junction;
  - either gripper's tool centre point comes over the track (X 360–640 mm, Y −340 to 340 mm) lower than 120 mm above the table;
  - the ball's centre leaves the running height by more than 8 mm downward or 12 mm upward (lifted, knocked out or derailed).

The trial ends at a full score, at a fault, or at the run limit of 400 actions (20 s at 20 Hz). Scoring a physical trial needs only video and a ruler. Check the lever's position in the frames where the ball is on a junction, and check which side of the exit line the ball finishes.

## Run and score on Isaac Lab-Arena

From the task directory with the program's Isaac Lab-Arena environment active:

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
for policy in yam_scripted dream; do
  width=320; height=176
  if [ "$policy" = yam_scripted ]; then width=640; height=360; fi
  python run.py run --embodiment yam_arena --policy "$policy" \
    -E task=task.py -E object_jitter=0 -E control_hz=20 \
    -E cam_width="$width" -E cam_height="$height" \
    --instruction "$instruction" --scorer task_score \
    --max-steps 400 --seed 0 --no-prompt --store-frames --log-dir "runs/$policy"
done
python -m pytest -q test_scoring.py
python export_media.py --demo runs/yam_scripted --dream runs/dream --out media
```

A trial takes about 1.5–2 minutes of wall time on an L40S. Use new log directories when repeating a trial. Arena records the final `task_score`. `run.py` also writes `trace-1.jsonl` with the ball, lever, blade and gate states, the gripper positions, the stage reached and any fault, one row per action. To run a validation control, set `PASSING_LOOP_DEMO` to `idle`, `pull_only`, `early_restore` or `late_pull` for the `yam_scripted` policy.

## Fidelity

The track, blades, lever, gate, wagon and ball are rigid PhysX bodies with real contact. The ball rolls and is turned only by the walls and blades. The blades and lever turn on revolute joints, and the lever's stops and over-centre action come from its weight. Three things are simplified:

- **The rod.** A stiff angular spring drives each blade toward the angle that the lever's position gives.
- **The ball's drive.** The ball is driven toward the exit by a constant 7 mN pull with 0.1 N·s/m of drag. On the straight it runs at a steady 7 cm/s, standing in for a train under power. It runs slower on the diagonals, where the walls take part of the pull.
- **The gate.** It is lifted by a fixed force at the release time.

For a physical reproduction, tilt the board slightly toward the buffer stop and line the channels with felt. Adjust the tilt until the ball takes 8 ± 1 s to cover 60 cm of straight. The physical procedure has not been tested on hardware.

## Validation

All runs use the submitted files, seed 0 and no jitter, on an L40S.

- **Scripted YAM demo: 1.0 in 333 actions** (16.65 simulated seconds). The right arm pulls the lever at 3 s, before the gate lifts at 5 s. The ball enters the loop through blade A. The ball clears junction A at about 8.2 s. The arm pushes the lever back at 9.3–10.0 s, which is 1.8 s before the ball reaches junction B at about 11.9 s. Blade B turns the ball onto the exit diagonal, and it crosses the exit line at 16.65 s.
- **Dream-1: 0.0 in 149 actions.** It did not touch the lever. At 7.45 s it brought its left gripper down over the track (TCP 867 mm high, 425 mm forward), which is a fault. The ball was still on the main line, heading for the wagon.
- **Commissioning controls**, the same scripted arm with one change each (`PASSING_LOOP_DEMO`):
  - `idle`, lever untouched: 0.0. The ball stops against the wagon.
  - `pull_only`, lever never restored: 0.5. The ball runs into the sand drag.
  - `early_restore`, pushed back while the ball is on junction A: 0.0. The fault is raised at 7.5 s.
  - `late_pull`, pulled only after the ball has passed junction A: 0.0. The ball stops against the wagon.
- **Offline tests:** 11 tests in `test_scoring.py` pass. They cover the full run, the sand drag, the wagon, throws on either junction, a half-thrown lever, a gripper over the track, the ball lifted off the track, the ball carried to the exit without going round the loop, and the result staying final once reached.
