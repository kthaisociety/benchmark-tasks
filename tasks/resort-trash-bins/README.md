# Correct mixed recycling bins

**Draft; not ready for submission.** This task models an inspection step at a workplace recycling station. Three shallow bins already contain a mixture of correctly placed and misplaced glass, cardboard and metal items. The robot moves each misplaced item to its assigned bin and sets aside an item outside those categories.

The scene, scoring and scripted demo have not been run in the simulator. The demo's reach, grasps, placement, timing and score are unverified. Dream-1 has not been run. `task.json` contains pending scores and an empty media ID; these must be replaced with real results and uploaded media before submission.

The task is implemented in [resort_trash_bins.py](resort_trash_bins.py) for the `yam_arena` station embodiment. The item shapes are visual proxies: green cylinders represent glass bottles, gray cylinders represent metal cans, brown boxes represent cardboard and a red box represents an item outside the three categories. The simulation does not model actual glass, metal, paper or an apple. The instruction states the category and destination for each proxy.

## Starting layout and scoring

The complete fixed scene, dimensions, colors, starting slots and poses are specified in `resort_trash_bins.py` and summarized in `task.json`. Each fixed tray has a 14 cm square opening and 3 cm walls; the fourth, purple tray is the set-aside area. There are three glass, three cardboard, three metal and one out-of-category item. Each recycling bin starts with one correct item and two items from other categories; the glass bin additionally contains the out-of-category item. The purple tray starts empty.

Each destination contributes 0.25 when every item assigned to that destination counts as inside it, and no item assigned elsewhere counts as inside it. An item counts when its axis is within 15 degrees of upright, it is resting, its world-axis-aligned bounding box fits inside the opening, and its lowest point is between 2 mm below and 8 mm above the tray floor. Otherwise that destination contributes 0. The score is their sum. This uses the simulator's `resting()` and object bounds; cylinder bounds can be conservative under rotation. Validate the conditions against real simulator frames and object poses before submission.

## Candidate scripted trial

On the shared instance, from the repository root and after the team's required approvals and resource checks:

```bash
source /data/IsaacLab-Arena/yam.env
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/resort-trash-bins/resort_trash_bins.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Correct the recycling bins: put the green glass bottles in the blue bin, the brown cardboard boxes in the yellow bin, and the silver metal cans in the gray bin. Place the red out-of-category item in the purple tray." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/resort-trash-demo
```

The candidate `demo()` contains seven transfers; it is not yet evidence that a score of 1.0 is achievable. Inspect `task_score`, object poses and a contact sheet or video, then run multiple episodes to check reset and repeatability. Check failure variants, including doing nothing, a misplaced object, a contaminated destination, an object outside all trays, and an item stacked on another. Investigate whether the evaluator reports the final or best score and ensure later failures cannot retain credit.

## Dream-1

A Dream-1 run is **not** part of the scripted command above. Follow the team workflow, including explicit approval before a 40-step smoke test and before a full run. Freeze the scene and instruction before the full run. Record the actual scores and produce the required video and screenshot before submission.

## Runtime, reset and seeds

The run limit is provisionally 2,400 steps at 20 Hz, or two minutes of simulated control time. Wall-clock runtime is unknown until measured. `object_jitter=0` requests a fixed layout; no seed has been established as necessary. Verify the complete initial state and score after each reset.

## Offline logic checks

From the repository root, run:

```bash
python3 -m unittest discover -s tasks/resort-trash-bins -p 'test_*.py' -v
```

These tests use stand-ins for `yam_arena`. They check scoring examples and the planned slot sequence; they do not check the actual simulator API, physics, camera visibility, reachability or motion.

## Asset and code provenance

The draft defines its scene using primitive `Box` and `Cylinder` objects from `yam_arena`; it does not include third-party mesh assets. The project's licensing and attribution must be verified before submission.
