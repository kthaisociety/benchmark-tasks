# Time a gravity-chute release into an empty moving tray

> Use the two gates to send the yellow ball into the empty moving tray. Let the ball stop at the second gate before releasing it.

A parcel sorter releases items upstream so they reach a receiving tray as it passes the induction point. Here the teal gate feeds a ball down a covered chute to the orange gate. Both gates lift vertically. The ball has a second downhill run after the orange gate, so the robot has to judge where the empty tray will be when the ball arrives. A red dummy load marks the occupied tray.

[Honeywell sortation guide](https://automation.honeywell.com/content/dam/honeywell-edam/sps/igs/en-us/campaign/igs-sps-sorting-out-your-sortation-options.pdf) · [Automatic loading system](https://patents.google.com/patent/US3817368A/en)

## Setup and reset

The standard station has a table height of 0.7475 m. Coordinates below are metres relative to the table surface, with x forward and y left. All shapes are primitives. The measured model is the geometry in [task.py](task.py).

| Object | Material and geometry | Starting location |
| --- | --- | --- |
| Gravity chute | Steel. Sloped deck 667 mm in plan, 124 mm wide and 16 mm thick. Three 106 × 34 mm legs, two 7 mm outer rails and a slotted 6 mm guard over the channel. Inner lane 56 mm wide. The guard has a 22 mm sight slit and two 32 mm gate openings. | x = 0.225, y = +0.252 to −0.415, outlet z = 0.042, pitch 5.5°. |
| Teal and orange gates | Rigid polymer, 95 g each. Blades 92 × 14 × 44 mm with 29 × 22 × 30 mm top handles. Guide ledges and cheeks support the blade tips outside the lane. | x = 0.225. Teal at y = +0.190, orange at y = −0.090. Blade bottoms 3 mm above the ramp at the respective centres. |
| Yellow ball | Rigid polymer sphere, diameter 40 mm and mass 30 g. | Spawned at x = 0.225, y = +0.221. It settles against the uphill face of the teal gate. |
| Rail bed and shuttle | Steel rail bed 556 × 155 × 4 mm with two 5 × 4 mm side rails. One fixed drive anchor and a 240 × 22 × 8 mm moving crossbar. | Rail centre (0.235, −0.490). Shuttle spans x = 0.080 to 0.390, with a one-second dwell before each reversal. |
| Two blue trays | Rigid polymer. 96 × 122 mm internal opening, 8 mm floor, 5 mm walls. Inlet lip rises 10 mm from the floor, remaining walls rise 30 mm. A dark rubber liner has crossed 4 mm wide, 3 mm high ribs at 24 mm spacing. | On the shuttle at x offsets ±0.067, at y = −0.490. Floor z = 0.016. |
| Red dummy load | Rigid polymer block, 66 × 66 × 44 mm, fixed to its tray. | Centred on the occupied tray liner. |

Return both gates to their guides, put the yellow ball upstream of the teal gate, and reset the shuttle to the listed home. The shuttle stays at home during scene settling. Each layout is fixed with zero pose jitter. The setup image shows layout A at the first control step from the top camera (left) and the right wrist camera (right), which shows the red load in the +x tray and the empty −x tray.

| Layout | Occupied pocket | Home x | Initial direction | Speed | Start delay |
| --- | --- | --- | --- | --- | --- |
| A, submission layout | +x | 0.390 m | −x | 10 cm/s | 0.00 s |
| B, occupancy control | −x | 0.390 m | −x | 10 cm/s | 0.00 s |
| C, direction and speed control | +x | 0.080 m | +x | 8 cm/s | 0.45 s |
| D, direction and speed control | −x | 0.080 m | +x | 12 cm/s | 0.80 s |

## Run a trial

Use the benchmark's Isaac Lab-Arena environment with PhysX and the `yam_arena` Inspect Robots plugin. The task loader and waypoint API were checked against YAM Arena revision `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`. Run from the task folder. The instruction comes from the same metadata used for submission.

```bash
source /data/IsaacLab-Arena/yam.env
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
SORTER_TRIAL=A inspect-robots run --embodiment yam_arena --policy yam_scripted \
  --scorer task_score --instruction "$instruction" --no-prompt \
  -E task="$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  --max-steps 600 --store-frames --log-dir /tmp/gravity-tray-demo-A
```

Read `task_score` from the Inspect Robots log. The action budget is 600 steps, or 30 seconds at 20 Hz. The scripted plan estimates a 1.08 s roll and drop after the gate clears. The policy plays a waypoint plan computed from the documented geometry and shuttle schedule. Each repeat uses the same starting layout and trajectory. Use the default layout A and replace `--policy yam_scripted` with `--policy dream` for the model recording.

On layout A the timed script reaches 1.0 and the run ends at step 267 (13.4 s), well inside the budget. Dream-1 scored 0.0 on layout A over the full 600 steps. The table lists every validation run on the submitted code, all with zero jitter. Each scored exactly as expected, and layout A gave 1.0 on both of two repeats. Use `SORTER_TRIAL=B`, `C` or `D` in separate runs to check the layout controls. `SORTER_DEMO` selects the comparison plan. Use a fresh log directory for each run.

| Script | Expected and measured score |
| --- | --- |
| `timed` (default), layouts A–D | 1.0 |
| `idle` | 0 |
| `early`, opening the second gate before the ball stops | 0 |
| `aligned`, starting the lift only when the empty tray is under the outlet | 0 |
| `occupied`, aiming the release at the occupied tray | 0 |
| `fixed` on B, reusing the timing for A after swapping the loaded tray | 0 |
| `staged`, releasing only the first gate | 0.25 |

Run `python -m unittest discover -s . -v` for the CPU score and timing tests.

## Scoring

Score the last control step. At each step the scorer reads the two gate poses, ball centre and actual shuttle joint position.

- A seated gate has its base within 13 mm along x and 19 mm along y of its starting slot, and from 7 mm below to 9 mm above its seated height. The vertical axis and blade width axis have dot products of at least 0.97 and 0.96 with their starting axes. A gate counts as opened once its base rises 44 mm while aligned within those slot and orientation tolerances. Moving a gate out of alignment before that point invalidates the trial.
- Stage the ball with gate 2 seated, the ball centre 19 to 45 mm uphill of gate 2 and its speed at most 25 mm/s for ten consecutive samples. This earns 0.25 while it remains parked there. Moving gate 2 out of its seated envelope before the dwell invalidates the trial.
- Through the chute, the ball centre stays within 25 mm of the centreline and from 20 mm below to 16 mm above the rolling centre height. Each gate must open before the ball moves more than 8 mm past its plane. The ball must reach the outlet 0.50 to 1.40 s after gate 2 clears, then land within 0.35 s. In the drop corridor the ball centre is within 40 mm of the chute centreline, from y = −0.551 to −0.397, and at z ≤ 0.074 m.
- The landing is the first sample with the ball centre at most 8 mm above its expected tray-floor height. At that step the shuttle must move at least 40 mm/s and the ball must be in the empty tray. The accepted centre region has half-width 33 mm and half-depth 46 mm. Entering the occupied tray or losing containment after capture invalidates the trial.
- Full credit requires the ball on the empty tray floor within 8 mm and moving at most 50 mm/s relative to the tray for the final 20 consecutive samples. Other outcomes score 0. Invalidated trials stay at 0 until reset.

A real trial can be assessed with a fixed video, marked gate positions, a tape measure and frame timestamps. For the simulated trial, the visible positions supply those same measurements.

## Licensing

The task code and custom primitive geometry are released under MIT. The compound-object helper is adapted from `tasks/din-rail-mount/partkit.py` in this repository. This folder includes no external meshes or textures. Isaac Lab-Arena and the installed YAM station supply the runtime and robot assets under their own licenses.
