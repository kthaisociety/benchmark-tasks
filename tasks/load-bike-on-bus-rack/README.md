# Load a bike onto a bus's front bike rack

Instruction: "Load the bike onto the bus's bike rack, ready for the bus to drive off."

A 1:6 bus front with a two-position bike rack, already lowered, and a bicycle standing in a wheel rail on the
pavement. Transit agencies publish how to load these racks:

- King County Metro ([How to load](https://kingcounty.gov/en/dept/metro/rider-tools/bikes-and-transit/how-to-load)):
  "When the rack is empty, lift your bike into the rack space farthest from bus." "Face the front wheel toward the
  hook (yellow or black)." "Squeeze the black button and pull the support arm straight out and over the top of the
  front wheel." "Place the hook as close to the top of the wheel frame as possible."
- Metro Transit ([Bike-n-Ride](https://www.metrotransit.org/bike-n-ride-bus)): "Do not place the arm on top of a
  fender or brake — it should be in direct contact with the tire."
- TriMet ([Bikes on buses](https://trimet.org/bikes/bikesonbuses.htm)): "place the bar over your front wheel (as
  close to the frame as possible)."

The agencies disagree on fenders: King County says the hook can rest on one. The bike here has none. It does have
a front brake on the fork crown, just above the tyre.

So the robot has to lift the bike into the position farther from the bus with the front wheel at the hook's end.
It holds the bike upright there, since the channels are wide enough for it to lean about 45°. Then it swings that
position's support arm out and down so the hook rests on the front tyre near the fork, and lets go. A bike left in
the channels without the arm scores 0.25 at most. In the simulator a bike set down perfectly upright happens to
stay standing on its flat tyre segments; a real one would tip over, and if it leans past 25° the trial scores 0.

## Files

| File | What it holds |
| --- | --- |
| `task.py` | The task file: scene objects, the instruction, `score()`, `success()`, the scripted demo (`demo()`) and the shortcut variants used in the checks below |
| `layout.py` | Every position on the table |
| `bike.py` | The bicycle's geometry: one rigid body of boxes, built with partkit |
| `rack.py` | The rack's geometry, the support arms' hinge geometry (hook point at any hinge angle), and the rack's build as one jointed body (USD/PhysX) |
| `scoring.py` | The scorer, pure NumPy |
| `run.py` | Runs Inspect Robots with the rack built from `rack.py` in place of a placeholder box. It also passes the rack's pose, both hinge angles and both tool centre points to `score()`, and resets the scorer at each trial |
| `partkit.py` | Compound rigid parts (the bike), from `tasks/din-rail-mount` |
| `test_scoring.py` | Scorer and geometry checks with no simulator |
| `export_media.py` | Turns a recorded trial into the review video (H.264) and the setup screenshot |

## Run a trial

From this folder, with the program's Isaac Lab-Arena environment active:

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
python run.py run --embodiment yam_arena --policy yam_scripted \
  -E task=$PWD/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --scorer task_score --max-steps 1600 --seed 0 --no-prompt \
  --store-frames --log-dir runs/demo
```

Use `--policy dream` for Dream-1. The score is printed as `task_score`. With `--log-dir`, `run.py` also writes a
per-step trace (`trace-1.jsonl`). Each row holds the score and fault, which position the bike is in, the bike pose
and lean, both wheel centres, both hinge angles and hook points, why the outer hook is or isn't on the front tyre,
the rack pose and both tool centre points.

- **Runtime per trial:** about 5 minutes on one GPU, including simulator start-up. The demo ends on success at
  26.3 s of simulated time (526 steps). A trial run to the 1,600-step limit takes longer.
- **Seeds:** none. The start is fixed and `object_jitter=0`.
- **Scoring checks, no simulator:** `python -m pytest -q test_scoring.py`

## Scoring

Read at the end of the run.

- **0, for good,** if at any time the bike leans over more than 25° (it fell over), or the rack moves more than
  1 cm or turns more than 3°. These faults are latched for the trial; nothing afterwards clears them.
- A wheel is **in** a position when its centre is within 2.4 cm of that channel's centre line, no more than 1 cm
  above resting on the table, and inside the channel's end stops.
- With both wheels in neither position: **0**.
- In the position **nearer the bus**: **0.25**, or **0.5** if secured and let go.
- In the position **farther from the bus**: **0.25**; **0.6** if secured; **1.0** if secured, let go and at rest
  (under 1 cm/s and 0.2 rad/s), all held for 1 s. The episode ends 1 s after that.
- **Secured** means three things:
  - The bike leans 12° or less.
  - The front wheel is at the end of the position where that position's hook is.
  - That position's hook rests on the front tyre near the fork. The hook pad's bottom centre is within 1.2 cm of
    the tyre's mid-plane. Its distance from the front axle is between 0.4 cm less and 0.8 cm more than the wheel's
    radius (5.7 cm), so it is touching the tyre and not resting on the brake or frame. It is above the axle and no
    more than 4 cm along the bike from it (the fork).
- **Let go** means both grippers' tool centre points are 4 cm or more from the bike's outline and from the hook.

On a real rack this needs a tape measure and a photo: which channel each wheel is in, which way the bike faces,
where the hook sits on the tyre, and whether the bike stands.

## Scripted demo

Open loop: every waypoint comes from the scene's known start, nothing a policy couldn't see.

1. The right arm turns its gripper 90°, takes the top tube over the bike's centre of mass, and lifts the bike
   3.2 cm out of the rail. It carries the bike 10 cm forward and sets the wheels in the channels of the position
   farther from the bus. It then holds the bike upright.
2. The left arm comes down with its fingers part-open (about 5.6 cm) either side of the outer support arm's black
   knob, fingertips 1.8 cm below it. It follows the knob's arc: the far finger pulls the arm out past vertical, and
   the near finger lowers it onto the front tyre. It stops 3° short of rest and lets the arm's weight settle it.
3. The left arm lifts off. The right arm opens, lifts and moves clear.

## Scores

- **Dream-1:** 0.0 (1,600 steps). It took the bike in the pavement rail with the right arm and pulled it about
  6 cm toward itself; the bike tipped past 25° at 31 s (latched fault) and never reached the rack.
- **Scripted demo:** 1.0, success at 25.3 s, ending at 26.3 s. Run twice in a row with a reset between: [1.0, 1.0].

## Shortcut checks

Each variant is selected with `BIKE_VARIANT=<name>` before `python run.py ...` (scripted policy, `--max-steps 700`;
the scripted policy ends 3 s after its plan, so every end state below is settled). Measured on 2026-11-11:

| Variant | What it does | Score | End state |
| --- | --- | --- | --- |
| `demo` | as above | 1.0 | success at 25.3 s; outer hinge -70.95° (hook on the front tyre), lean 0.7° |
| `nothing` | both arms park | 0 | bike in the rail, arms stowed |
| `unsecured` | bike in the right channels, let go with the support arm still stowed | 0.25 | in the outer position, not secured. The rigid bike, set down upright, happens to stay standing on its flat tyre segments (lean 0.0°); a real one would tip |
| `held` | bike in the right channels, still held, support arm stowed | 0.25 | in the outer position, not secured |
| `inner` | bike in the position nearer the bus; the outer arm swung over it; let go | 0.25 | in the inner position; the outer arm stopped at -56° on that bike, hook off its tyre |
| `brake` | bike slid 2.1 cm toward the hook's end; the arm comes down on the head tube and brake | 0.25 | the arm stopped at -60.7° on the head tube and crown, hook not on the tyre |

Not run as variants but covered in `test_scoring.py`: the bike reversed in the outer position with the arm on the
rear wheel (0.25); the inner position used correctly, front wheel toward its hook at the other end (0.5); hook on
the tyre more than 4 cm from the fork (0.25); a gripper left on the hook (0.6); moving the rack (latched 0).

## Licence and assets

The task is released under the MIT licence. `partkit.py` comes from `tasks/din-rail-mount` (MIT). There are no
third-party assets. The bike, rack, bus front and rail are built from boxes in this folder's code.
