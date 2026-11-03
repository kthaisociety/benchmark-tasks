# Safe glass bottle sorting

Sort four colored glass bottles onto matching red and blue shelves while preserving
all bottles throughout. This is a single fixed arrangement, with two misplaced
bottles, two correctly placed neighbors, and one staging pad for a swap. No scenario
variants or random layouts are included.

## Runtime and dependencies

Use the Robocurve YAM Isaac Lab-Arena environment: Isaac Sim 6.1, Isaac Lab 3.0,
Arena release/0.3.1, `yam-arena` (tested VM revision
`0059d02a1a91bc6834c1d0846022891df0c78730`), Inspect Robots 0.60.0 and
`inspect-robots-dream` 0.1.0. NVIDIA GPU required for trials. NumPy and pytest suffice
for offline scoring tests, with `yam-arena` importable. The station includes two
6-DoF YAM arms, parallel-jaw grippers, a table, lights, a camera stand, overhead RGB
camera and two wrist cameras.

`run.py` is mandatory: it adds compound bottle geometry and per-bottle PhysX contact
sensors, samples safety at every 240 Hz physics step, resets episode safety state,
and verifies gripper opening and withdrawal. It patches only the current Python
process; installed packages are untouched. Running `task.py` with the stock CLI
alone deliberately cannot earn credit because the safety monitor is not armed.

## Run and score

On the GPU VM, from this repository root:

```bash
source ~/IsaacLab-Arena/yam.env
python tasks/safe-glass-bottle-sorting/run.py run \
  --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/safe-glass-bottle-sorting/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=640 -E cam_height=360 \
  --instruction "Sort all four glass bottles onto the shelf matching their color. Keep every bottle upright and intact; finish with grippers open and withdrawn." \
  --scorer task_score --seed 0 --max-steps 2400 --store-frames --no-prompt \
  --log-dir runs/safe-glass-bottle-sorting/demo
```

Use `--policy dream` and a fresh log directory for the Dream-1 evaluation. Dream
credentials belong in the environment or the existing credentials file, never in
this task folder. The horizon is 2,400 control steps, or 120 simulated seconds at
20 Hz. The scripted plan takes approximately 60 seconds of simulated time. Measured results for the final task revision (seed 0, no jitter):

| Policy | Score | Steps | Simulated time | Wall time | Termination |
| --- | ---: | ---: | ---: | ---: | --- |
| Scripted demo | 1.0 | 1,146 | 57.3 s | 200.8 s | success |
| Dream-1 | 0.0 | 1,723 | 86.15 s | 289.7 s | unsafe |

Both evaluations completed without trial errors. The first visible demo also
reached 1.0; its 414.9 s wall time included the initial shader compilation. The
final recorded demo has no safety violations, maximum tilt 2.684 degrees and
maximum net normal force 10.769 N. All four bottles are sorted and both grippers
clear at completion. `validation.json` stores exact measured values. The Dream
trial predates the added JSONL trace writer; its trial log records the unsafe
termination and zero score, without a separate bottle safety trace.

Add `-E headless=false` for the Isaac Sim window. Forward the VM desktop's port
6080 through SSH and open the local noVNC viewer. Keep the existing desktop
password and network binding. For a persistent viewer after evaluation, run:

```bash
python tasks/safe-glass-bottle-sorting/view.py --demo
```

This replays the demonstration on a loop in the live Isaac Sim window. Omit
`--demo` to hold the starting layout. Stop with Ctrl+C. Preview episodes are not
submission trial logs.

Each saved trial contains the reported score and termination reason. The supplied runner
also writes `bottle-safety.jsonl` beside the trial, including per-step
`safety_violations`, `peak_force_n`, `max_tilt_deg`, `sorted_count`, and
`grippers_clear`, bottle poses and velocities. Use the **final** episode score; do not use
best-so-far scoring because safety violations are irreversible.

## Objects and layout

World axes are x forward, y left, z up; units are metres and kilograms. Station
table top is z=0.7475. All fixtures are fixed wooden platforms. A shallow red shelf
is centered at (0.225, 0.13, 0.7535), size (0.27, 0.16, 0.012). The blue shelf is at
(0.225, -0.13, 0.7595), size (0.27, 0.16, 0.024). The gray staging pad is at
(0.205, 0, 0.7535), size (0.22, 0.065, 0.012). These are two open, shallow shelves
side by side at different heights, accessible from above; enclosed cabinets are
outside this task's scope.

All four bottle surrogates are 0.10 m tall, 0.036 m maximum diameter and 0.08 kg.
A single rigid body carries three cylinder colliders: body diameter 0.036, height
0.070, local center z=-0.015; shoulder diameter 0.030, height 0.012, local center
z=0.026; neck diameter 0.018, height 0.030, local center z=0.035. The cylinders
join/overlap; the bottle's rigid origin and bounding-box center coincide. Bodies
have friction 0.65 static/0.50 dynamic and zero restitution. Red and blue are
opaque visual surrogates for colored glass.

| Object | Color | Initial center (x, y, z) |
| --- | --- | --- |
| blue_swap | Blue | (0.16, 0.13, 0.8095) |
| red_neighbor | Red | (0.29, 0.13, 0.8095) |
| red_swap | Red | (0.16, -0.13, 0.8215) |
| blue_neighbor | Blue | (0.29, -0.13, 0.8215) |

The demo first stages blue_swap at (0.27, 0), then stages red_swap at (0.14, 0),
then returns red_swap to the red shelf and blue_swap to the blue shelf. Only one
arm moves a bottle at a time; the other parks clear. Every bottle is manipulated
by contact/gravity/friction, with no attachment, teleportation or goal snapping.

## Safety and scoring

Every physics step irreversibly invalidates the trial if **any** bottle:

- tilts more than 20 degrees from upright;
- exceeds 40 N magnitude of net normal contact force (virtual damage);
- has its lowest point below z=0.7535 (a fall off a shelf/pad);
- has center outside x=[0.07,0.38], y=[-0.23,0.23], or top above z=1.0675;
- contains a nonfinite state or force measurement.

Invalid episodes terminate with score 0. Later restoration does not erase a
failure. All four bottles, including untouched neighbors, are monitored.

A bottle counts as correctly sorted only when its entire world-aligned bounding
box fits on its matching shelf with at least 3 mm margin, its bottom is within
4 mm of the shelf top, tilt is at most 5 degrees, linear speed is below 0.01 m/s
and angular speed below 0.10 rad/s. Shelf surface geometry determines support;
hovering, stacking, leaning, or holding a bottle cannot produce full credit.
Both grippers must be at least 75% open, with each TCP at least 85 mm from every
bottle center. With safety and release checks passing, score is
`max(0, correctly_sorted_count - 2) / 2`: 0, 0.5, or 1. The initial state scores 0.
All four valid placements and release checks must hold continuously for 2 seconds
to terminate successfully. Partial credit uses the final state.

These are declared simulation thresholds, not measured glass fracture limits.
PhysX reports net normal forces; opposed forces can cancel and tangential forces
are omitted. Actual fracture, wall thickness, transparency and liquid contents
are not simulated. For physical reproduction use colored plastic surrogate
bottles, a ruler/protractor and video for placement/tilt, plus force sensing for the
virtual-damage threshold. No real glass breaking is needed.

## Reset and offline validation

Reset places every bottle at the listed upright pose with zero velocity, resets
arms to READY, and clears all latched safety events. Use seed 0 and zero object
jitter for all submitted trials.

```bash
# With yam-arena importable, no GPU required:
python -m pytest -q tasks/safe-glass-bottle-sorting/test_task.py
# In the development workspace only:
python tools/robot_env.py check tasks/safe-glass-bottle-sorting/task.py
```

Tests cover unchanged-state zero score, partial/full credit, release gating,
hovering, shelf edges, movement, tilt, force/fall/workspace damage latching,
nonfinite states, reset, and refusal to score an uninstrumented stock runner.

## Submission media

Export each trial's camera streams with `inspect-robots video <trial.json>`.
Choose the unobstructed overhead recording, encode H.264/yuv420p with faststart,
and use its initial fixed-layout frame for setup.png. Videos must be <=50 MB.
Keep media, logs and credentials outside this task folder. After recording both
trials, set measured scores in task.json and upload using the repository's
`tools/upload_media.py`; it writes the real media ID. No fabricated ID is used.
The task board needs authenticated access to check pending/accepted duplicates.


From the development repository root, upload the prepared media after setting
`ROBOCURVE_TEAM` and `ROBOCURVE_TOKEN` in your shell:

```bash
python tools/upload_media.py tasks/safe-glass-bottle-sorting \
  --video .local/media/safe-glass-bottle-sorting/model_run.mp4 \
  --demo .local/media/safe-glass-bottle-sorting/demo.mp4 \
  --screenshot .local/media/safe-glass-bottle-sorting/setup.png
```

This writes the real `media` field. Commit that metadata update before opening
the PR; use only this task folder in the PR. The current preparation deliberately
leaves `media` absent until upload succeeds. Perform the authenticated duplicate
check before submitting. The local prepared branch is
`codex/safe-glass-bottle-sorting`.
