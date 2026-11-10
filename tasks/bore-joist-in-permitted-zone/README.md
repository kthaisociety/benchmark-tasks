# Set up a drill guide for a pipe hole through a ceiling joist

A full-size 47 x 97 mm softwood ceiling joist, 800 mm clear span, lies flat and clamped on the bench to be bored
before it goes in. The plumber has chalked a cross at mid-span, close to one edge, where he would like his 15 mm pipe to
pass. The electrician's 22 mm hole is already there, on the centre line in the left-hand third of the joist. The robot
has to stand a drill guide block on the joist where the 22 mm hole should actually be bored.

> The plumber has chalked a cross where he wants his 15 mm pipe to pass through this ceiling joist. Stand the drill
> guide on the joist where the 22 mm hole for it should be bored, then let go of it.

The instruction doesn't give the rule. A carpenter knows it: UK building-control guidance allows holes in a solid
timber joist only on its centre line (the neutral axis), between 0.25 and 0.4 of the clear span from a support, no wider
than 0.25 of the depth and at least three diameters apart
([LABC Hertfordshire Technical Guidance Note 22, "Notching of joists", May 2011](https://www.hertsmere.gov.uk/Documents/09-Planning--Building-Control/Building-Control/22-Guidance-Note---Technical---Notching-of-joists-May-2011.pdf),
"Safe areas for drilling"; the same zones are in NHBC Standards 6.4.18 "Notching and drilling"). On this joist that
means 200 to 320 mm from either end, on the centre line. The electrician's hole sits 260 mm from the left end, so any
new hole in the left-hand zone would be within 66 mm (three 22 mm diameters) of it. The only place left is the
right-hand zone, on the centre line. The chalk cross is wrong on both counts: it is at mid-span, outside both zones, and
near an edge, off the centre line.

The task tests whether a policy knows where a timber beam may be weakened, reads the existing work on it, and overrides
a colleague's mark instead of following it.

How it differs from the nearest submissions:

- PR #593, decking bundles near the joist bearing: an OSHA rule on where a load may be landed along steel joists,
  stated in its instruction. Here the rule isn't stated. It is a two-dimensional zone for weakening a timber member
  (along the span and across the depth), plus spacing from existing work.
- PR #1372, nail plates over bored studs: the holes are already bored and the robot judges which ones need protection.
  Here the robot chooses where a hole may go.
- PR #923, scanning for a hidden pipe before drilling: detecting a hidden hazard with a tool. No structural rule.

## Fixed setup

Coordinates are from the midpoint between the arm bases (61 cm apart): x forward, y toward the left arm, z up from the
table top. The station's top camera sees the whole joist.

| Object | Type, size, material | Where |
|---|---|---|
| Joist | C16 softwood, 47 x 97 x 800 mm, lying flat, 97 mm face up; fixed (clamped) | centre (36, 0) cm, long axis along y, ends at y = +/-40 cm |
| Chalk cross | white chalk, two strokes 7 mm wide: 50 mm along the joist, 26 mm across | mid-span (y = 0), centred 15 mm in from the joist's near edge |
| Existing hole | 22 mm hole, shown as a dark disc | on the centre line, 260 mm from the left end (y = +14 cm) |
| Drill guide | beech block, 60 mm cube, 150 g; steel bush (32 mm ring, 22 mm bore) centred in its top face | standing on the table at (21, 0) cm, square to the table |

The joist is cut to fit between joist hangers 800 mm apart, so its clear span is its length and its ends are the
supports. The exact geometry is in `scoring.py` (joist, zone and hole numbers) and `task.py` (scene).

Reset: stand the guide back on the table at (21, 0) cm, square to the table edges. Nothing is bored or consumed. Run
with `-E object_jitter=0`; there is one fixed layout, so the seed does not change the start.

## Scoring

The scorer measures at the centre of the guide's bore, where it meets the joist face (the centre of the hole that would
be bored). It reads the final state with a tape measure and a square.

| Score | Condition |
|---|---|
| 1.0 | The guide stands on the joist face (base within 3 mm of it, tilted at most 5 degrees). The bore is 200 to 320 mm from the nearer end of the joist, at least 66 mm from the existing hole's centre and within 10 mm of the centre line. The guide has been let go (no point of either gripper, wrist mount to pad tips, within 5 cm of it) and its bore has moved at most 2 mm over the last second. Holding this for 1 s ends the trial. |
| 0 | Everything else: more than 10 mm off the centre line; less than 200 mm or more than 320 mm from the nearer end (near a support, or around mid-span, including the chalk cross); within 66 mm of the existing hole (the whole left-hand zone); tilted, on the table, still held, or never moved. |

There is no partial credit. Every spot other than a permitted one breaks the guidance, and a hole that breaks it is
simply wrong, so giving it credit would reward a defect. There is no intermediate result that keeps to the rule.

The guide may be turned any way about the vertical. The 10 mm centre-line tolerance keeps the edge of the 22 mm hole
at least 26.5 mm from both faces of the joist. Real boring is done on the drawn centre line. The let-go check reads
both grippers' positions from the arms' joint angles through the framework's kinematics (`yam_arena.ik.tcp_pose`). If
the arms can't be read, the trial scores 0. The 66 mm spacing and the 200 to 320 mm zone are
the guidance's own figures for this joist (3 x 22 mm; 0.25 and 0.4 x 800 mm).

For a real trial, measure from the bore's centre to the nearer end of the joist, to the existing hole's centre and to
the joist's near edge (48.5 mm = on the centre line).

## Run a trial

On a machine set up with the official simulator (the Robocurve Brev launchable, or `yam-isaac-lab-arena`'s
`setup.sh`), from this directory:

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --scorer task_score --max-steps 600 --store-frames --log-dir runs/dream

# The scripted demo
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --scorer task_score --max-steps 600 --store-frames --log-dir runs/demo

# Shortcut variants of the scripted demo (expected score in brackets)
for v in offcentre:0 held:0 edge:0 cross:0 midspan:0 bearing:0 on_hole:0 near_hole:0 nothing:0; do
  JHZ_VARIANT=${v%%:*} inspect-robots run --embodiment yam_arena --policy yam_scripted \
    -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$instruction" --scorer task_score --max-steps 600 --log-dir runs/variant-${v%%:*}
done

for log in runs/dream/*.json; do inspect-robots video "$log"; done

# Offline checks of the scorer, the variant targets and the demo's reach (IK)
python -m pytest -q test_scoring.py
```

The run prints `task_score` at the end. The trial ends with 1.0 once `success()` has held for 1 s. Otherwise it scores
`score()` at the last step.

| Variant | Guide set down at | Expected |
|---|---|---|
| `demo` | right-hand zone, 260 mm from the right end, on the centre line | 1.0 |
| `offcentre` | the same, 15 mm off the centre line | 0 |
| `held` | the right spot, but the gripper keeps hold of it | 0 |
| `edge` | the same, 30 mm off the centre line | 0 |
| `cross` | on the chalk cross | 0 |
| `midspan` | on the centre line, 370 mm from the right end | 0 |
| `bearing` | on the centre line, 70 mm from the right end | 0 |
| `on_hole` | over the existing hole (left arm) | 0 |
| `near_hole` | left-hand zone, 310 mm from the left end, 50 mm from the existing hole (left arm) | 0 |
| `nothing` | not moved | 0 |

## Runtime and seeds

- The scripted demo moves for about 11 s of simulated time (about 220 steps) and then waits out the 1 s success
  hold. Wall time per trial on the instance: about 45 s for the scripted demo and about 2.5 min for a full
  600-step Dream-1 trial.
- One fixed layout (`-E object_jitter=0`). The seed doesn't change the start.

## Results

All runs in the simulator, fixed layout, 320 x 176 cameras at 20 Hz.

- **Scripted demo:** 1.0 in both reset-check trials and in the recorded trial (bore 1.6 mm off the centre line,
  253 mm from the right end, guide flat and released).
- **Shortcut variants:** every one scored as in the table above. `offcentre` ended 13 mm off the centre line, `edge`
  28 mm off, `midspan` 351 mm from the end, `bearing` 66 mm from the end, `on_hole` and `near_hole` in the left-hand
  zone, `held` in the right spot but never released, and `cross` tipped off the joist edge (the cross is 15 mm from
  the edge, so a 60 mm guide centred on it overhangs and falls).
- **Dream-1:** 0.0 in 600 steps. It picked up the guide and carried it toward the chalk cross, but never stood it on
  the joist in the permitted zone.

## Licences

The task (scene, scoring and demo code) is released under the MIT licence. `partkit.py` is copied unchanged from
`tasks/din-rail-mount` in this repository. No third-party models or textures are used: every object is a plain box or
cylinder.
