# Mark a boat's dead-reckoning position on a nautical chart

A paper nautical chart lies taped to the table: 1:50,000 Mercator, open sea between 49°57'N and 50°04'N. A pencilled
course line runs from a red buoy on 045° true. The boat has logged 6 nautical miles since it passed the buoy. The robot
has to put a position marker on the chart where the boat is now.

> The boat passed the red buoy at the start of the pencilled course line and has since run 6 nautical miles along that
> line. Put the position marker on the chart where the boat is now, then let go of it.

The instruction doesn't say how to measure the distance. A navigator knows: on a Mercator chart distance is taken off
the latitude scale at the side of the chart, level with the leg, because one minute of latitude is one nautical mile.
The longitude scale along the top and bottom is the classic wrong answer. A minute of longitude is shorter by
cos(latitude), here about two thirds of a mile, so six minutes taken off it put the boat 2.1 nm short (sources:
[savvy navvy, "How to calculate nautical miles on a chart"](https://www.savvy-navvy.com/blog/how-to-calculate-nautical-miles-on-a-chart);
[Casual Navigation, "How to measure distances on a nautical chart"](https://casualnavigation.com/how-to-measure-distances-on-a-nautical-chart/);
[USCG Master 100 study guide, 4.1 Nautical charts](https://open-exam-prep.com/study-guides/uscg-master-100/nav-charts-aton/nautical-charts)).
A pair of navigator's dividers lies on the chart, as it would on a chart table. Using them is not required: only where
the marker ends up is scored.

The task tests whether a policy can read a measuring scale off a printed chart, knows which of the chart's two scales
measures distance, and transfers a length along a drawn line to a precise spot.

How it differs from the nearest submissions (scanned 2026-11-11: every pull request to #1643, the task board including
the previous cohort's planned tasks, and the Embodied First Exam catalogue; nothing else uses a nautical chart):

- PR #1557, scribe a circle with spring dividers: uses dividers as a compass around a fixed centre. Here the dividers
  (optional) measure a length on a scale, and the score is a position on a chart.
- PR #781, measure and mark 300 mm with a tape: a length read off a tape in millimetres. Here the unit must be found on
  the chart itself, and the wrong scale is printed right beside the right one.
- Board task "mark a grid square and a six-figure grid reference on a topographic map": reads a coordinate off
  labelled grid lines. Here no coordinate is given: the robot has to measure a distance run along a drawn line, and
  the grid lines are deliberately not where the answer is.
- PR #1595, site a ship's navigation lights where COLREG Annex I requires: placement by a stated rule on a ship model.
  Here it is a navigation measurement, and the rule is not stated.

## Fixed setup

Coordinates are from the midpoint between the arm bases (61 cm apart): x forward, y toward the left arm, z up from the
table top. The station's top camera sees the whole chart.

| Object | Type, size, material | Where |
|---|---|---|
| Chart | paper Mercator chart, 1:50,000, 295 x 417 mm (1 mm thick as simulated), taped down (fixed) | centred at (28.5, 0) cm, north away from the robot |
| Printing | latitude scales on the left and right edges (alternate whole minutes in black bands, 0.1' ticks), longitude scales on the near and far edges, grid lines at 50°00'N, 2°55'W, 2°50'W and 2°45'W, pale blue sea | on the chart |
| Buoy | red buoy symbol (7 mm square with a dark position dot) | 49°58.3'N 2°58.5'W: (20.3, 15.5) cm |
| Course line | dark pencil line 1.5 mm wide, 045° true, 7 nm long | from the buoy |
| Position marker | yellow disc 25 mm across, 6 mm thick, with a black knob 12 mm across and 16 mm tall; 30 g | standing on the table at (11.5, 0) cm |
| Dividers | brass, two 8 x 7 mm legs 150 mm from hinge to steel point, 20 mm hinge head; 80 g; a free-turning hinge (no spring, very light damping) | on the chart, hinge at (17.5, -3.5) cm, legs toward the right, opened 30 degrees |

On the chart a minute of latitude (a nautical mile) is 37.0 mm and a minute of longitude 23.8 mm. The exact
projection is in `chart.py` (spherical Mercator, true to scale at 50°N); the scene is built from it in `task.py`.

Reset: stand the marker back at (11.5, 0) cm; lay the dividers back as above. Nothing is drawn or consumed. Run with
`-E object_jitter=0`; there is one fixed layout, so the seed does not change the start.

## Scoring

The scorer measures at the centre of the marker's base, the point it marks on the chart. A navigator checks it with
dividers against the latitude scale; for a real trial, a ruler from the dead-reckoning position works as well (0.2 nm
is 7.4 mm, 0.5 nm is 18.5 mm).

| Score | Condition |
|---|---|
| 1.0 | Within 0.2 nm (7.4 mm) of the dead-reckoning position, 50°02.5'N 2°51.9'W, which is 6 nm (222 mm) from the buoy along the line at (36.1, -0.2) cm. |
| 0.5 | Within 0.5 nm (18.5 mm) of it. |
| 0 | Farther: the longitude-scale answer (2.1 nm short), the buoy, the line's end (1 nm past), anywhere else. |

Credit is given only if the marker lies flat on the charted area (its base within 3 mm of the paper, tilted at most 10
degrees), has been let go (no point of either gripper, wrist mount to pad tips, within 5 cm of it) and has moved at
most 2 mm over the last second. Holding a full score for 1 s ends the trial. The marker's turn doesn't matter.

0.2 nm is the precision of a careful plot at this scale; 0.5 nm credits a measurement on the right scale that was
transferred loosely. Nothing between the right scale and the wrong one earns credit: the longitude-scale answer is
2.1 nm out. The answer is at least 1.7 nm from every point where the course line crosses a grid line, so aiming at a
printed crossing scores nothing.

## Run a trial

On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), from this directory:

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
common=(--embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320
        -E cam_height=176 --instruction "$instruction" --scorer task_score --max-steps 2400)

# Dream-1
CDR_TRACE=runs/dream inspect-robots run "${common[@]}" --policy dream --store-frames --log-dir runs/dream

# The scripted demo
CDR_TRACE=runs/demo inspect-robots run "${common[@]}" --policy yam_scripted --store-frames --log-dir runs/demo

# Shortcut variants of the scripted demo (expected score after the colon)
for v in dividers:1 close:1 half:0.5 lon_scale:0 at_buoy:0 line_end:0 held:0 nothing:0; do
  CDR_VARIANT=${v%%:*} CDR_TRACE=runs/variant-${v%%:*} inspect-robots run "${common[@]}" --policy yam_scripted \
    --max-steps 600 --log-dir runs/variant-${v%%:*}
done

# Review videos and the setup screenshot
python export_media.py runs/demo demo.mp4 --setup setup.png
# (the submitted setup screenshot is the top camera's first frame rendered at -E cam_width=1280 -E cam_height=704)
python export_media.py runs/dream model_run.mp4

# Offline checks of the chart, the scorer, the variant targets and the demo's reach (IK)
python -m pytest -q test_scoring.py
```

The run prints `task_score` at the end. The trial ends with 1.0 once `success()` has held for 1 s. Otherwise it scores
`score()` at the last step. `CDR_TRACE=<dir>` writes one JSON line per step (the marked point, its distance in nm, the
let-go check, the dividers' opening and leg A's heading) to `<dir>/trace-<trial>.jsonl`; `export_media.py` captions the video from it.
`CDR_TICKS=0.5` prints only half-minute ticks, if the full chart (about 500 printed shapes) is slow to load.

| Variant | Marker set down at | Expected |
|---|---|---|
| `demo` | the dead-reckoning position | 1.0 |
| `dividers` | the same, after the right arm opens the dividers to the 6 nm span (95.6 degrees) | 1.0 |
| `close` | 0.15 nm past it along the line | 1.0 |
| `half` | 0.35 nm short of it along the line | 0.5 |
| `lon_scale` | 6 minutes of longitude (142.9 mm) along the line: 2.1 nm short | 0 |
| `at_buoy` | on the buoy (left arm) | 0 |
| `line_end` | the end of the pencilled line, 7 nm | 0 |
| `held` | the right spot, but the gripper keeps hold of it | 0 |
| `nothing` | not moved | 0 |

## Runtime and seeds

- The scripted demo moves for about 11 s of simulated time (about 220 steps) and then waits out the 1 s success hold;
  the `dividers` variant moves for about 21 s. Wall time on one NVIDIA L40S: about 45 s to load the scene, about
  3 minutes for the demo with frames stored, about 17 minutes for a full 2,400-step Dream-1 run.
- One fixed layout (`-E object_jitter=0`). The seed doesn't change the start.

## Results

From the official simulator (`yam_arena`, `-E object_jitter=0`, 20 Hz, 320 x 176 cameras):

| Run | Score | Where the marker ended |
|---|---|---|
| Dream-1, 2,400 steps | 0 | not moved: both grippers closed in the first seconds and the arms drifted back beside the chart, touching neither the marker nor the dividers |
| Scripted demo | 1.0 (at 9.4 s) | 0.05 nm (1.7 mm) from the dead-reckoning position |
| `dividers` | 1.0 | dividers opened to 95.3 degrees (6 nm is 95.6), leg A still; marker 0.02 nm off |
| `close` | 1.0 | 0.14 nm off |
| `half` | 0.5 | 0.37 nm off |
| `lon_scale` | 0 | 2.18 nm off |
| `at_buoy` | 0 | 6.03 nm off |
| `line_end` | 0 | 0.98 nm off |
| `held` | 0 | 0.01 nm off, still in the gripper |
| `nothing` | 0 | still on the table |

The dividers opening checks that the tool works in the simulator: a gripper can take leg B and swing it to the 6 nm
span while leg A stays where it lies.

## Licences

The task (scene, scoring and demo code) is released under the MIT licence. `partkit.py` is copied from
`tasks/din-rail-mount` in this repository, with two additions: hinged bodies (`Hinge`, for the dividers) and a fixed
sheet with visual-only printing (`Sheet`, for the chart). The chart is drawn from a spherical Mercator projection in
`chart.py`; no real chart, texture or third-party model is used.
