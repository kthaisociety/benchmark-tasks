# Open a shelf-ready tray of cans by tearing its front panel off along the perforation

A brown corrugated shelf-ready tray of twelve food cans stands on the table, front toward the robot. Its front panel
comes off along a printed perforation, and the opened tray goes straight onto the shelf as the display.

The instruction is: "Open this shelf-ready tray so it can go straight onto the shelf: take its front panel off along the
perforation and leave every can standing in the tray."

Stockers and order fillers open shelf-ready trays all shift when they restock. The panel comes off by hand, with no
knife: the perforation tears cleanly when the panel is peeled out and down from it, and holds when it is just pulled
straight up or sideways. A panel torn off the line, a tray dragged off its spot or a can knocked over leaves a tray that
can't be shelved as it is.

## Scene

Positions in cm from the midpoint between the arm bases (x forward, y left); heights above the table. The layout is
fixed (no jitter); both arms start in their ready pose.

| Object | Description |
|---|---|
| Tray | Brown B-flute corrugated board, 3 mm, 260 g, shrink film already removed. Outside 31.0 wide (y) x 24.1 deep (x) x 11.5 high, open top. Its front face at x 20, centred on y 0, square to the table edge |
| Removable part | The front wall above a 4.0 cm lip, with the front part of each side wall above a diagonal perforation (die-line below) |
| Perforation | 6 mm cuts with 1 mm ties (an easy-open perforation), printed as dashed lines: across the front at the top of the lip, and down each diagonal |
| Cans | Twelve 15 oz steel food cans, 7.5 cm across x 11.1 cm, 425 g, paper labels, standing upright on the tray floor 4 across x 3 deep, touching, the front row 8 mm behind the front panel |

### Die-line of the removable part

```
   side wall (left), seen from outside            front wall, seen from the robot             side wall (right)
   back                         front                                                    front                 back
    |       top edge 11.5    |   |<-------------------- 31.0 -------------------->|   |      top edge       |
    |      9.0    ____.......|   +------------------------------------------------+   |.......____    9.0   |
    |        ____/ .  panel  |   |                removable panel                 |   | panel . \____       |
    |   ____/  . diagonal    |   |                (7.5 high)                      |   |  diagonal  . \____  |
    |  /  . perforation      |   |- - - - - - - - - perforation - - - - - - - - - |   |    perforation  .  \|
    | . lip top at the front |   |                lip (4.0 high, stays)           |   |                     |
    +------------------------+   +------------------------------------------------+   +---------------------+
```

Each diagonal runs straight from the side wall's top edge, 9.0 cm back from the tray's front face, down to the lip's top
(4.0 cm up) at the front corner. The front wall's perforation runs the full width at 4.0 cm. The front corners are plain
creases (folds), not perforated.

## Model

- **Board:** the tray is one rigid body. The removable part is three rigid pieces of board: the front panel and the two
  triangular side flaps, joined at the front corners by the board's creases (hinges about the vertical, sprung straight,
  0.002 N m per degree).
- **Perforation:** 15 ties. Nine sit along the lip, and three sit down each diagonal at a fifth, half and four fifths
  of its length. Each is a ball joint: it carries force but no moment, like a narrow tie of board. It breaks for good past
  its strength: 0.6 N on the diagonals, 1.2 N on the lip.
  - Peeling the panel out and down about the lip loads the ties farthest from the lip most. So the tops of the
    diagonals tear first (with about 3 N at the panel's top edge), and the tear then runs down them, each next tie
    needing less.
  - Once both diagonals are torn, the panel hangs on the lip ties, and pulling it on out, square to the lip, tears
    them.
  - A straight lift or a sideways pull spreads over every tie and holds (about 14 N).
  - These strengths are set for an easy-open perforation; they were not measured on real board.
- **Torn:** a tie counts as torn once its two ends are more than 2 mm apart.
- **Not modelled:** tears off the perforation line. The real trial checks for them, and any such tear scores 0.

## Scoring

`score()` judges the end of the trial:

- **0, whatever else:** the tray ever tipped more than 10 degrees (and, on a real rig, the board torn more than 5 mm off
  the perforation line anywhere).
- **1.0:** the panel off (every tie torn) and not in or on the tray (its pieces' middles outside the tray's outline, or
  more than 5 cm above its rim); the tray moved no more than 2 cm at any corner; all 12 cans upright (within 10 degrees)
  in the tray; both grippers' TCPs 3 cm or more from the tray, the panel and the flaps; and everything still (under 1
  cm/s) for 1 s.
- **0.5:** the panel off, but not all of that.
- **0.25:** the tear started (at least one tie torn), the panel still on.
- **0:** nothing torn.

On a real rig: the video, tape marks at the tray's corners, and a look at the torn edges.

## Run a trial

From this folder, with Inspect Robots and the `yam_arena` embodiment installed:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')" \
  --scorer task_score --policy <policy> --max-steps 2400
```

`--policy yam_scripted` runs the scripted demo (`demo()`), and `DEMO_VARIANT` picks a variant (below). The layout is
fixed, so there is no seed. Runtime per trial (one 48 GB NVIDIA GPU, including about 30 s of start-up): about 6
minutes for the full 2,400 steps, under 2 minutes when the trial ends early on success.

Offline checks (no simulator): `PYTHONPATH=<yam-isaac-lab-arena> python -m pytest -q test_task.py`. They cover:
- the geometry and the finger's gap;
- the tear's force balance: a peel tears with about 3 N, the lip holds meanwhile, the tear runs on once started, and a
  straight lift needs about twice the peel;
- scoring;
- that every demo waypoint is within reach.

## Scripted demo

1. The left hand turns its fingers to close along x.
2. It lowers them over the middle of the front panel's top edge, 6 mm either side of the board: the back finger goes
   down behind the panel into the gap between the two middle front cans (3 cm wide there), the front finger in front.
3. It pinches the top 2 cm of the panel and peels it out and down about the lip to about 55 degrees, turning the
   wrist with it. The diagonals tear, top ties first.
4. It pulls the panel straight out (aimed 4 cm past: the arm gives sideways), and the lip tears.
5. It carries the panel off to the left of the tray, puts it down on the table, lets go and parks.

The right hand stays parked.

| Variant (`DEMO_VARIANT`) | What it does | Expected | Result |
|---|---|---|---|
| `full` | peel the panel off, carry it aside, put it down | 1.0 | 1.0 (diagonals torn top to bottom at steps 119, 122 and 142; the lip at step 262; success held at about step 410) |
| `nothing` | arms stay parked | 0 | 0.0 |
| `lift` | pinch the panel and pull it straight up (in its own plane), aimed 8 mm up | 0 (holds) | 0.0 (no tie torn) |
| `start_only` | pinch and peel 3 degrees, then hold | 0.25 | 0.25 (the diagonals' top and middle ties torn) |
| `keep_holding` | tear it off but keep holding the panel | 0.5 | 0.5 |
| `drop_in_tray` | tear it off and put it down on top of the cans | 0.5 | 0.5 |

## Files

- `task.py`: the scene, the perforation, scoring and the scripted demo.
- `bodykit.py`: multi-shape rigid bodies and joints for yam_arena task files (this team's part library). This copy adds
  convex-prism shapes and break forces on its ball and fixed joints.
- `test_task.py`: offline checks.

No third-party assets: every part is built from boxes, cylinders and prisms in `task.py`.
