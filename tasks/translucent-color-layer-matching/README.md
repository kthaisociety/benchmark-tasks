# Translucent lighting-filter matching — local draft

Select plaques whose combined tint matches a separately lit reference, without moving it. This is an **unrun, provisional UPGRADE**, not a validated submission. The board was inaccessible; reservation status is unknown. See [REVIEW.md](REVIEW.md).

## Setup

The fixed `station` scene uses metres: x is forward, y is left and z is up. Heights below are relative to the tabletop (`TABLE_TOP["station"]` in code). There is no object jitter.

| Plaque | Centre (x, y) | Nominal shader tint RGB |
| --- | --- | --- |
| Amber | (0.150, 0.280) | (1.00, 0.60, 0.25) |
| Cyan | (0.330, 0.280) | (0.25, 0.90, 1.00) |
| Violet | (0.150, -0.330) | (0.80, 0.25, 0.85) |
| Lime | (0.330, -0.330) | (0.55, 1.00, 0.32) |

Each loose plaque is one 50 g, 60 × 60 × 20 mm rigid body: a nominal 4 mm tinted acrylic sheet on four dark PLA rails, 8 mm wide and 16 mm high, leaving a 44 × 44 mm opening. The sheet faces up. Gripping the framed part is unverified.

Two fixed 120 × 120 × 10 mm white acrylic lightboxes are centred at (0.300, 0.055) and (0.300, -0.145). The first has a dark square outline, 60 × 60 mm on its outer edges, made of 2 mm wide, 0.2 mm high marks. The second carries a separate loose 50 g reference with the same geometry and nominal tint (0.25, 0.54, 0.25). All five plaques start 1 mm above their supports. Identical 44 × 44 mm white USD rectangle lights, intensity 1000 in renderer units, are positioned 0.3 mm above the box tops and face upward. The appearance of the light and materials is unverified.

## Draft scoring and reset

The **0.45/0.45 split is an unconfirmed draft assumption**. Add 0.10 for at least one required plaque, 0.45 for both amber and cyan, and 0.45 if at least one is present and no extra is connected. Either order scores equally. One required plaque without extras scores 0.55; both score 1.0; both with an extra score 0.55. With no required plaque, the score is 0.

The scorer builds a geometric chain from the lightbox. Each plaque must be sheet-up within 3° of horizontal, centred within 5 mm per axis, and within 2 mm and 2° of its final pose in each of the last 20 samples. Its lowest corner must be within ±0.5 mm of the support's highest corner. The gap allows for the 0.2 mm raised outline and the plaque collider's 0.5 mm contact offset; its rest offset is zero. Actual resting gaps, penetration and the robot's ability to meet the 5 mm tolerance need simulator verification.

The outline marks the ideal footprint, not a hard boundary: the centre tolerance permits 5 mm of overhang per axis at zero yaw, and more at rotated corners. For level plaques at any yaw, the 44 mm openings still share a central 18 × 18 mm patch of the nominal light window. That geometric margin does not verify illumination, the camera view, shadows or tilted plaques. An uncounted plaque is connected if its projected footprint overlaps a connected plaque by at least 1 mm along every separating axis and their vertical bounds are within 3 mm; this repeats transitively. This conservative rule can penalize an extra that does not touch.

**Release is not verified.** The task uses poses and pose history, with no verified gripper or contact signal. It rejects a 1.9 mm gap, but a plaque held still 0.4 mm above its support can still earn full credit. Isaac Lab has contact sensors and other repository tasks use robot-link poses and a PhysX contact view; neither is wired or verified here. Support and release need a tested signal or scoring redesign.

After the underlying reset, the hook clears the latch and pose histories and records the reference pose. Settling and restoration of the initial layout are unverified. A displacement over 3 mm or rotation over 2° from that pose latches score 0, even after return. There is no early success termination. Sampling can miss movement between callbacks; callback frequency and reset timing need verification. Nothing is consumed.

## Color check

The sheet uses `GlassMdlCfg` with `glass_color`, `thin_walled=True`, `glass_ior=1.0` and zero frosting; its default MDL is a Nucleus resource. The rectangle lights are rotated upward from their default local −Z direction. The [material](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/api/lab/isaaclab.sim.spawners.html) and [light](https://openusd.org/24.05/api/class_usd_lux_rect_light.html) APIs do not establish their actual rendered appearance.

`optics.py` multiplies nominal RGB values as an offline hypothesis: amber and cyan produce (0.25, 0.54, 0.25), and the closest of the other 15 subsets differs by 0.17 in its largest channel. This is not a render or a transmission measurement. Runtime scoring checks identity and position, not color.

After the applicable gates and an authorized probe, start with five settled, unedited frames: the empty light, amber alone, cyan alone, and both orders of amber and cyan. Use the same camera, exposure and lighting. Select equal-size interior regions for stack and reference, avoiding rails, glare, shadows and clipping. For each frame run:

```bash
python3 tasks/translucent-color-layer-matching/optics.py --image /path/to/frame.png \
  --stack-roi X,Y,WIDTH,HEIGHT --reference-roi X,Y,WIDTH,HEIGHT
```

The Pillow-based tool assumes sRGB, compares mean decoded RGB with a **preliminary** maximum-channel threshold of 0.05, and flags over 5% clipped pixels as inconclusive. It cannot assess ROI suitability, undo tone mapping, detect near-clipping or establish perceptual equivalence. Inspect the frames and reference visibility too. If the target looks plausible, screen one order of each remaining 12 subsets. If the intended orders differ, or an alternative is ambiguous or matches, check the other orders of the affected sets. These 17 frames cannot rule out matches in untested orders: before treating a full score as valid, establish that both intended orders match and no other plausible order matches, expanding the checks as needed or agreeing on scoring equivalent combinations. No simulator frames or real filters have been tested.

## Checks and later simulator run

From the repository root:

```bash
python3 -m unittest discover -v -s tasks/translucent-color-layer-matching -p test_task.py
python3 tasks/translucent-color-layer-matching/optics.py
```

Tests use synthetic poses, images and simulator stubs. Isaac Lab 2.2 documents `wxyz` root poses; 3.0 uses `xyzw`. Other repository tasks assume 3.0, but do not establish this launchable's version. The adapter infers the convention from API methods; record the installed version and checkout/tag and verify it before an authorized run. See [2.2 root poses](https://isaac-sim.github.io/IsaacLab/v2.2.0/source/api/lab/isaaclab.assets.html) and [3.0 release notes](https://isaac-sim.github.io/IsaacLab/v3.0.0/source/refs/release_notes.html).

The proposed left-arm demo picks amber then cyan; its waypoints total about 33.3 s excluding overhead. The provisional limit is 2,400 steps at 20 Hz (120 s), seed 0. **Do not run on the instance until the team's novelty, paper-feasibility, coordination and user-go-ahead gates are met.** Probe the risky grasp in a scratch copy first. After the applicable probe/build gates, the full scripted command is:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/translucent-color-layer-matching/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stack the translucent plaques in the outlined area to match the color of the reference plaque. Do not move the reference plaque." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames \
  --log-dir /path/to/scratch-output
```

Check both orders, held and floating variants, extras, reference movement and return, two resets and video frames. Real-world safety, gripping, optics, support/release, reset, runtime and scores remain unverified. No model run, media or media ID exists. `partkit.py` is adapted from `tasks/din-rail-mount/partkit.py`; verify code and asset licences before submission.
