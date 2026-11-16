# Lock a spring E-track fitting into a van floor track

A spring E-track fitting with a steel O-ring tie-off stands in a wooden stand beside a section of floor E-track on a
plywood van-floor panel. The fitting's head spans 70 mm across its two lips with the latch out, so it only goes down a
62.2 mm slot with the latch squeezed in 10 mm (1.1 mm spare each end). Once the lips are under the 2.7 mm steel face
it has to slide toward its fixed lip so that lip hooks under the steel, and only then be let go so the latch lip
springs out under the other end. Let go too early and the latch lip rests on top of the face: a false lock that pulls
straight out under load. Truck drivers lock these fittings on every load: every strap or load bar in a van anchors to one.

Instruction: "Lock the tie-off ring into the track."

## Run and score a trial

```
inspect-robots run --embodiment yam_arena -E task=$PWD/lock_e_track_fitting.py -E object_jitter=0 -E control_hz=20 \
    --scorer task_score --policy <policy> --instruction "Lock the tie-off ring into the track." --max-steps 1600
```

`--policy yam_scripted` plays the scripted demo (about 34 s, 680 steps). One trial runs in about 2 minutes on an L40S.
No random seeds: the layout is fixed (`object_jitter=0`). `ETRACK_DEBUG=1` prints the fitting's state every step.

The score (`score()` in the task file) reads the fitting and latch poses: the nearest slot, how far each lip reaches
under the face past its slot end, the lips' height relative to the face's underside, the body's height above the face,
and whether the gripper still holds it.

| Score | Condition |
|---|---|
| 0 | not in a slot, latch lip resting on the face (false lock), or lips not under the face |
| 0.25 | in a slot, neither lip 2 mm under the face |
| 0.5 | one lip at least 2 mm under, or both but the body more than 2 mm above the face |
| 0.75 | both lips at least 2 mm under, body on the face, still held |
| 1.0 | the same, gripper let go |

## Demo

The scripted demo grips the heel tab and the trigger (squeezing the latch in), lifts the fitting out of its stand,
turns it a quarter turn so the head lies along a slot, and carries it over the slot. It then reads the fitting's real
pose and corrects the hand three times (to about 0.5 mm), and once more with the tongue just above the face, the way a
person lines a fitting up by eye. It goes down the slot, presses the body flat, slides it 5 mm toward the fixed lip,
lets go, moves the finger off the trigger, presses the body down and pushes it toward the fixed lip.

## Checks

Each run was made with this task file; `ETRACK_VARIANT` picks a shortcut instead of the demo.

| Run | What it does | Score |
|---|---|---|
| `demo` | full demo | 1.0 |
| `release_high` | lines the head up but lets go above the face | 0 |
| `no_slide` | goes down the slot and lets go without sliding the fixed lip under: the latch kicks it up onto the face | 0 |
| `off_centre` | open loop, 3 mm off along the slot: the head lands on the face | 0 |
| `leave` | reaches the fitting and stops; it stays in the stand | 0 |
| `dream` | Dream-1, 1,200 steps: never takes the fitting out of the stand | 0 |

## Real trials

A section of standard floor E-track (12-gauge steel, 62.2 x 14.7 mm slots on 50.8 mm centres) screwed to a plywood
offcut and a spring E fitting with an O-ring tie-off. Squeezing the trigger needs about 12 N; nothing is sharp or
heavy.

## Licence

All code (the task file and trialkit.py) is our own, MIT licensed. No third-party meshes, textures or code are used.
