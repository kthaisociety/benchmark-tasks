## Task

Sort four colored fragile bottles onto matching shallow shelves in one fixed
bimanual YAM station layout. Two misplaced bottles must be exchanged while the
two correctly placed neighboring bottles remain upright and intact. A shared
staging pad allows a safe swap.

A physics-step monitor latches tipping, falls, workspace violations and virtual
contact damage. Full credit requires all bottles resting on the correct shelves,
both grippers released and withdrawn, and two seconds of stability. The scripted
demo uses only simulated contact manipulation; no object snapping or attachment.

## Reproduction

Use `run.py` with the commands, fixed seed, dependency versions and numerical
criteria in README.md. The wrapper installs task-specific geometry and sensing
in memory without changing simulator packages. `task.json` records fidelity
limits; virtual damage is a net normal force surrogate, not real glass fracture.

## Validation

- 14 offline scoring/safety regression tests pass.
- Both scripted arm plans pass the reachability check (worst position error
  2.171 mm, worst orientation error 0.562 degrees).
- Final scripted GPU demo: 1.0, seed 0, 1,146 steps (57.3 simulated seconds),
  200.8 s wall time; no trial errors or safety violations.
- Dream-1: 0.0, seed 0, 1,723 steps, 289.7 s wall time; unsafe termination.
- Demo peak normal contact force 10.769 N, maximum bottle tilt 2.684 degrees.
- Both recordings are H.264/yuv420p, 640x360, 20 fps; setup image matches the
  fixed initial layout. Media upload and authenticated duplicate check remain.

This description is a prepared draft. Complete the media upload and media ID
before opening the submission pull request. Authenticated task-board duplicate
checking is also required before submission.
