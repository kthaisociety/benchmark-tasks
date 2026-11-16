# Close a dozen-egg carton with its locking flap tucked inside the lid

A pulp dozen-egg carton stands open on the counter after eggs were taken out. The lid stands back past upright, held
open by the springy pulp hinge, and the front locking flap hangs outward. The robot must close the carton so it stays
shut. The scene, scorer and scripted demo are in `egg_carton.py`.

Closing an egg carton is something almost every adult does several times a week. The pulp carton only stays shut
when the front flap is folded up inside the lid's front edge: a bead along the top of the flap then sits above a bead
inside the lid's skirt, and the flap's own spring keeps it pressed outward under the skirt. Shut the lid over a flap
that is still hanging out and the lid springs back open. A tucked flap that is let go creeps back out (soft pulp, about
5 degrees a second), so the flap has to be held in until the lid is most of the way down, then the lid brought the
rest of the way down over it. People do this with two hands: one folds and holds the flap, the other brings the lid
over. The scripted demo does the same with the two arms.

## Files

| File | What |
|---|---|
| `egg_carton.py` | The yam_arena task file: objects, `score()` and the scripted demo |
| `partkit.py` | Rigid parts built from boxes and cylinders (from the accepted `din-rail-mount` task) |
| `hingekit.py` | Sprung revolute hinges on partkit parts (written for this task family, MIT) |
| `gripkit.py` | Gripper orientation helpers for the scripted demo |
| `task.json` | The submission details |

## Run a trial

In the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`), from the repository root:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/egg-carton-close/egg_carton.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Close the egg carton." \
  --scorer task_score --store-frames --max-steps 2400 --log-dir runs/dream
```

Scripted demo: the same flags with `--policy yam_scripted --max-steps 380`. The score is printed as `task_score`.
Wrong attempts for the shortcut checks: set `EGG_VARIANT` to `lid_only`, `slow` or `nothing` before the scripted run.
`EGG_DEBUG=1` prints the lid and flap angles, carton movement and finger positions every 10 steps.

## Scoring

Final state only. The lid angle is 0 when shut and 100 degrees as found; the flap angle is 0 when upright, positive
when tucked in toward the eggs and -60 degrees as found.

- **1.0**: lid within 5 degrees of shut, flap within 6 degrees of upright or further in (so it is inside the skirt,
  where its bead holds the lid), both halves of the carton within 1 cm of where they started, no finger within 3 cm of
  the carton's closed outline, and the lid still (moving under 0.5 degrees) for the last 1 s.
- **0.5**: the same, but the carton was pushed more than 1 cm.
- **0** otherwise.

Every other end state is open: without the flap inside, nothing holds the lid down, and the pulp hinge opens it again
as soon as it is let go. The scorer needs nothing but the two hinge angles, the carton's position and the finger
positions.

### Shortcut checks (scripted, same scene)

| Attempt | What happens | Score |
|---|---|---|
| `EGG_VARIANT=lid_only`: shut the lid with the flap left hanging out | the lid springs back open (100 degrees) when released | 0 |
| `EGG_VARIANT=slow`: tuck the flap, let go, close the lid 2.5 s later | the flap has crept out; the lid springs back open | 0 |
| Hold the lid shut at the end | fingers within 3 cm of the carton | 0 |
| `EGG_VARIANT=nothing` | the carton stays open | 0 |

## Results

- Scripted demo: **1.0**, twice (lid locked at 2.2 degrees, flap 10 degrees in, carton moved 2 mm).
- Dream-1: see `task.json` (`scores`).

## Simulation notes

- Rigid bodies built from boxes and cylinders. The carton is modelled as two rigid halves joined along the middle of
  the cup tray (front and back rows of six eggs, 0.375 kg each), because the lid and the flap must be able to touch
  each other: the lid's hinge belongs to the back half and the flap's hinge to the front half.
- Hinges are revolute joints with torsion springs (`hingekit.py`): lid 0.02 N m/rad (about 0.35 N at the lid's front
  edge when shut), flap 0.0086 N m/rad (about 0.3 N at its top when upright) with 0.1 N m s/rad damping (soft pulp:
  a tucked flap creeps back out at about 5 degrees a second). The lock is two 3 to 4 mm beads: one inside the lid's
  front skirt, one along the top of the flap with a 45 degree lead-in above it.
- The carton is placed once at reset; nothing is moved or edited during a trial.
- Eggs are 4.2 cm cylinders (a large egg is 4.3 cm across).

## Licence

The task is released under the MIT licence. It uses no library or third-party assets: every object is built from
plain boxes and cylinders and released under the task licence. `partkit.py` is reused from the accepted
`din-rail-mount` task in this repository (same licence as the repository); `hingekit.py` and `gripkit.py`
were written for this task family and are MIT.
