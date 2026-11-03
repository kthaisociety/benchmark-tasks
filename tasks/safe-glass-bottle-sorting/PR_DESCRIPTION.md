## Task

Sort six fragile colored bottles between two shelves stacked vertically in a
narrow rack: red below, blue above. Swap the misplaced center bottles through a
staging pedestal while protecting four correctly placed neighbors. The upper
shelf, roof and panels require horizontal entry from the front.

Each bottle starts 6 mm from the open front edge; outer bottles have 7 mm side
clearance. Adjacent bottle surfaces have a 24 mm gap, so fully opening the jaws
would interfere with the neighbors. All constraints use contact geometry.

A physics-step safety latch zeros the score after excessive tilt, virtual damage,
uncontrolled descent or a workspace violation. Full credit requires all six
bottles resting on their matching shelf and both grippers clear for two seconds.
The unchanged arrangement scores zero. No object teleportation or attachment.

## Reproduction and validation

Use run.py, which installs task-specific geometry, sensing and horizontal
quaternion motion in memory. README.md lists exact dependencies, seed, inventory,
commands, scoring and fidelity limits. PREVIEW.md covers the viewer controls.

- 18 offline regression tests pass.
- Horizontal demo IK errors below 0.1 mm and 0.02 degrees for both arms.
- Scripted GPU demo: 1.0, seed 0, 1,238 steps (61.9 simulated seconds),
  266.9 s wall time; no errors or safety violations.
- Demo maximum tilt 8.873 degrees and peak net normal force 9.151 N.
- Earlier flat-layout results are archived and do not describe this task.
- Dream-1: 0.0, seed 0, 788 steps, 186.5 s wall time; red_swap tipped.
- Separate physical edge-risk probe: an ungrasped upper bottle tips to 36.11
  degrees after a frontward velocity perturbation; safety violation is latched.
- Both submitted recordings are H.264/yuv420p at 640x360 and 20 fps.

Before opening the PR, upload the revised videos and setup image, commit the
resulting real media ID, and perform the authenticated task-board duplicate check.
