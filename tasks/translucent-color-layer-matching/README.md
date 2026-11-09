# Combine lighting gels to match a reference colour

Four colour-correction gels sit in frames beside two lightboxes. One lightbox carries a fixed reference gel
assembly; the other is empty and outlined. The robot has to work out which gels, combined, transmit the
reference colour and stack those in the outlined area without disturbing the reference.

Two gels together transmit the product of their colours, so no single gel matches the target and the choice
cannot be made by comparing one gel at a time. With this set the answer is amber plus cyan, in either order.

## Setup

The fixed `station` scene uses metres: x is forward, y is left and z is up. Heights are relative to the
tabletop (`TABLE_TOP["station"]`). There is no object jitter.

| Gel | Centre (x, y) | Frame colour (RGB) |
| --- | --- | --- |
| Amber | (0.150, 0.280) | (1.00, 0.60, 0.25) |
| Cyan | (0.330, 0.280) | (0.25, 0.90, 1.00) |
| Violet | (0.150, -0.330) | (0.80, 0.25, 0.85) |
| Lime | (0.330, -0.330) | (0.55, 1.00, 0.32) |

Each loose gel is one 50 g, 60 x 60 x 20 mm rigid body: a 4 mm clear acrylic glazing on four PLA rails,
8 mm wide and 16 mm high, leaving a 44 x 44 mm aperture. The rails carry the gel's colour, which is how each
gel is identified on the table. The 16 mm rails are the gripping faces.

Two fixed 120 x 120 x 10 mm white lightboxes are centred at (0.300, 0.055) and (0.300, -0.145), each with a
44 x 44 mm lit window in its top face. The first has a dark 60 x 60 mm outline, 2 mm wide and 0.2 mm high.
The second carries the reference: two clear, neutral-framed 50 g frames stacked in a fixed assembly, so the
reference is seen through the same two glazings as a correct two-gel stack. All gels start 1 mm above their
supports to settle.

## How the colour is produced

**The mixed colour is computed, not ray traced.** The renderer does not composite stacked transmissive
sheets — with tinted glazing the topmost sheet wins and stacking order changes the result, which was measured
on the instance before this design was chosen. So the glazing is clear and each lightbox window is lit with
the colours of the gels above it multiplied componentwise (`optics.nominal_tint`), updated every control step
from the gels' poses. The reference window is lit with the same product for amber and cyan.

This is the one place the simulation departs from the physical story, and it is why the reference is a
two-glazing assembly: a correct stack and the reference are then seen through the same optical path, so equal
colours render equally. Measured at 320x176 with the gels in place, the two correct orders differ by 1-2 of
255, the correct stack sits 11-13 from the reference, and the nearest wrong combination sits 33 away.

## Scoring

A gel counts as stacked when it is glazing-up within 10 degrees of horizontal, centred within 1 cm per axis of
the outlined square, still for 20 control steps, and its lowest corner is within 3 mm of the top of its
support. Credit follows the transmitted colour rather than specific gel names, so any combination that
reproduces the reference earns full marks:

| Outcome | Score |
| --- | --- |
| Stacked gels transmit the reference colour, nothing else touching | 1.0 |
| They do, but an extra gel is on or against the stack | 0.5 |
| The stack is part of a correct combination, no extra touching it | 0.25 |
| Anything else, including an empty outline | 0.0 |

Moving the reference more than 5 mm or 5 degrees from its pose at reset latches 0 for the trial, even if it is
put back. The latch and the stillness history are cleared on reset. There is no `success()`: a later action
must still be able to spoil the result.

Every tolerance is checkable from a photograph or with a tape measure. Scoring reads pose, not contact, so a
gel held steady within the 3 mm support tolerance counts as placed; the embodiment exposes no gripper or
contact signal.

## Checks

From the repository root:

```bash
python3 -m unittest discover -v -s tasks/translucent-color-layer-matching -p test_task.py
python3 tasks/translucent-color-layer-matching/optics.py
```

The scripted demo, seed 0, 2,400 steps at 20 Hz:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/translucent-color-layer-matching/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stack gels in the outlined area until its light matches the colour of the reference light. Do not move the reference." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames
```

## Licences

This task is MIT licensed. `partkit.py` is adapted from `tasks/din-rail-mount/partkit.py` and **modified**: it
binds a glass MDL to a part's first box when the part carries a `filter_tint` in its `meta`, which is how the
clear glazing is spawned. No third-party assets are used; every object is a primitive box.
