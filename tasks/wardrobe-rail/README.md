# Refit a wardrobe rail under the hat shelf, between a closed and an open end support

A wardrobe section stands on the table: a base board, two white side panels 60 cm apart and, as in a real wardrobe,
a hat shelf across the top. The shelf hangs 5.5 cm above the rail, so the rail and its supports can be seen and
reached only from below and in front, never from above. The left panel carries a closed end support (a round cup),
the right panel an open U support. The rail, a round chrome wardrobe rod 25 mm across and 59.2 cm long, lies in two
grooved foam blocks in front of the arms. The robot must fit the rail into both supports and let go. The scene,
scorer and scripted demo are in `wardrobe_rail.py`.

The rail is a few millimetres shorter than the gap between the panels, as real rails are cut, but longer than the gap
minus the closed cup's depth. The shelf leaves no room to tilt it steeply (raised 13 degrees, the right end hits the
shelf), so it goes in the way people fit a rail under a shelf: nearly level and swung in plan. The left end is fed
into the closed cup with the right end swung about 18 degrees toward the front. The right end is then swung back,
lifted just enough (about 3 cm) to clear the U's front cheek, and lowered into the U while the left end stays in its
cup. Both arms hold the rail from the front and below the shelf, so the grippers are tipped forward instead of
pointing straight down.

## Revision (TK-F585ZS)

Review: "Rails should be mounted such that only a view from below it is available." Changes:

- A hat shelf (1.8 cm melamine) now spans the section 5.5 cm above the rail, covering the rail and both supports when
  seen from above, as in real wardrobes.
- The rail is fitted under it: nearly level and swung in plan, with the right end lifted only over the U's cheek. The
  old 13 degree tilt-in is blocked by the shelf (variant `tilt_in`).
- The rail is a round 25 mm wardrobe rod (the oval tube is also common, but a round rod is the standard closet rod);
  the supports are sized for it (29 mm bore, 1.0 cm deep).
- The scripted demo holds the rail with the grippers tipped 45 degrees forward, reaching under the shelf.

## Files

| File | What |
|---|---|
| `wardrobe_rail.py` | The yam_arena task file: objects, `score()` and the scripted demo |
| `partkit.py` | Rigid parts built from boxes and cylinders (from the accepted `din-rail-mount` task) |
| `task.json` | The submission details |

## Run a trial

In the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`), from the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/wardrobe-rail/wardrobe_rail.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Fit the wardrobe rail into its two end supports." \
  --scorer task_score --store-frames --max-steps 2400 --log-dir runs/dream
```

Scripted demo: the same flags with `--policy yam_scripted --max-steps 640`. The score is printed as `task_score`.
Wrong attempts for the shortcut checks: set `RAIL_VARIANT` to `tilt_in`, `level`, `hold` or `nothing` before the
scripted run. `RAIL_DEBUG=1` prints the rail end positions every 20 steps.

## Scoring

Computed from the final state. Credit is given only if both grippers are open (fingers at least 90% open) with both
pad tips at least 3 cm from the rail's axis, and the rail has moved less than 1 mm, for the last 1 s.

- 0.4: the left end is in the closed support: its centre at least 0.8 cm into the bore and inside the bore's outline.
- 0.4: the right end rests in the U support: its centre at least 0.5 cm over the floor and within 0.3 cm of the
  support axis height (so it lies on the floor, not on the cheeks).
- 0.2: both ends in and the rail level within 1 degree.

A real trial is scored by eye and with a ruler: the rail's left end is visibly inside the cup, the right end lies in
the U, and a spirit level on the rail reads level.

## Shortcut checks (scripted)

| Variant | What it does | Score |
|---|---|---|
| Demo | Swings the rail in plan under the shelf, feeds the left end, lifts the right end over the cheek into the U, lets go, backs out, parks | 1.0 |
| `tilt_in` | The old way: right end raised 13 degrees, left end fed, swung down | 0 |
| `level` | Carries the rail in level and square and pushes it toward the closed cup | 0 |
| `hold` | Fits it correctly but never lets go | 0 |
| `nothing` | Both arms park | 0 |

## Results

Dream-1: 0.0 over the full 2,400 steps (2 minutes). Both arms move toward the rail and the wardrobe, but the rail is never lifted out of the foam blocks. Demo: 1.0 (rail level, left end 9.7 mm into the cup, right end on the U floor).

## Runtime and seeds

- Scripted demo: about 560 steps (28 s simulated).
- Dream-1: 2,400 steps (2 minutes simulated).
- No randomness: run with `-E object_jitter=0`. No seeds.

## Fidelity

Rigid bodies built from boxes and cylinders. The rod is a 25 mm cylinder; the closed cup is a square 29 mm bore 1.0 cm
deep, the U is 29 mm wide with 2 cm cheeks. The wardrobe section with its hat shelf is one fixed 25 kg part; the foam
blocks are one 3 kg part with grooves that keep the rod from rolling. Friction 1.2 everywhere. No screws or fixings.
The rod is built along the part's vertical axis and laid along y at each reset (partkit cylinders are vertical). The
scripted demo lets a waypoint carry a full gripper rotation (a small wrapper on `yam_arena.demo.grip_rotation`, in
the task file) and aims each grip a few millimetres off to allow for how the rod sits in the tipped grip.

## Real-world build

A wardrobe section from two 16 cm offcuts of 18 mm melamine board screwed to a base board, 60.0 cm apart inside, a
hat shelf of the same board 5.5 cm above the rail, a standard closed round end support and an open U support for
25 mm wardrobe rod at the same height, and a 25 mm chrome rod cut to 59.2 cm. Two foam blocks with a groove.

## Licence

The task is released under the MIT licence. It uses no library or third-party assets: every object is built from
plain boxes and cylinders and released under the task licence. `partkit.py` is reused from the accepted
`din-rail-mount` task in this repository (same licence as the repository).
