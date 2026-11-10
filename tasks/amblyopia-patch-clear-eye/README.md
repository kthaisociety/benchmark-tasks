# Patch the correct eye for amblyopia occlusion therapy

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena),
Isaac Lab-Arena) on the station scene: [task.py](task.py). `partkit.py` (copied from `tasks/din-rail-mount`) is only
used here for its `env()` accessor, to check that both grippers have released a patch at the end.

A tan head board stands in for a child mannequin's face, seen from above, with two identical, unmarked eye markers,
each ringed by a low raised guide (an alignment guide, like a training mannequin often has). A cream chart card lies
between the robot and the head with two coloured dots in the same two y-columns as the head's eyes: a grey dot over
this week's flagged (weaker) eye's column, a green dot over the clear (stronger) eye's column. Two identical foam
occlusion patches sit on a tray. Amblyopia ("lazy eye") occlusion therapy patches the **stronger** eye, forcing the
brain to rely on the weaker one - the opposite of the intuitive move to patch whichever eye looks "wrong". The
instruction names the convention ("amblyopia occlusion therapy", "standard patch practice") but never says which eye
that is; the robot has to read the chart's two dots, match each to its eye by column, and apply the occlusion rule
itself. The two eye markers on the head carry no colour hint of their own, so the chart is the only place the
information lives.

## Requirements

- `yam-isaac-lab-arena` with Isaac Lab-Arena installed (`source IsaacLab-Arena/yam.env`).
- `partkit.py` is in this folder (copied from `tasks/din-rail-mount`, used only for its `env()` accessor).

## Run a trial

Dream-1:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/amblyopia-patch-clear-eye/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --scorer task_score --max-steps 1200 --store-frames \
  --log-dir ~/yam-eval/amblyopia-patch-dream \
  --instruction "Read the eye chart beside the child mannequin, then apply one adhesive eye patch over the correct eye for today's amblyopia occlusion therapy, following standard patch practice, and leave the other eye uncovered. Then let go."
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted --max-steps 500`, which runs this file's
own `demo()` directly (no separate demo script needed, unlike the cable tasks). Probed on Brev: the demo reaches
`task_score` 1.0 and the harness reports `outcome: succeeded` by about step 220 (its own early-success stop).

Shortcut variants (`PATCH_DEMO=<mode>`, default `correct`), all probed and confirmed to score exactly 0:

- `nothing`: no motion, both patches left on the tray.
- `wrong`: patches the flagged (grey-column) eye instead of the clear one.
- `both`: patches both eyes. The flagged-eye placement runs *first* in this demo, deliberately - placing the clear
  eye first would trigger the harness's own early-success stop before the flagged-eye placement ever ran, hiding the
  violation; with the flag covered first, `score()`'s hard-fail check fires as soon as the clear eye would otherwise
  register.
- `hold`: correct placement, but the arm keeps gripping the patch at the target instead of letting go.

## Scoring (`score()`, at the end)

- A patch counts as resting on an eye when its centre (top view) is within 3.5 cm of that eye marker's centre and it
  rests on the head's top surface (within 1 cm), with both grippers at least 4.5 cm away (released).
- 0 if: the head or chart moved from its mark; a patch rests on the flagged (grey-column) eye; neither patch rests
  within 3.5 cm of the clear (green-column) eye; a gripper still holds a patch.
- Otherwise: 1.0 within 1.5 cm of the clear eye marker's centre, 0.5 within 1.5-3.5 cm.
- `distance()`: the nearer patch's distance from the clear eye marker, +0.10 if a patch sits on the flagged eye.

## Runtime and seeds

- Run limit 1,200 steps (1 minute at 20 Hz); the scripted demo itself finishes by ~step 220.
- Layout fixed with `object_jitter=0`, so no seed is needed. The scripted demo is open loop and deterministic.

## Probing notes (what it took to get here)

The grasp-and-place mechanic took nine probe iterations on Brev before it worked cleanly, in order:

1. A 1.6 cm tall flat disc was never gripped, only grazed - `ONBOARDING.md` 5's "objects under about 4.5 cm tall
   slip out of the grip" rule, for a disc squeezed from above (unlike a flat bar gripped along its length).
2. A 3.6 cm diameter cup got shoved sideways instead of lifted - likely too wide for the gripper's opening.
3. Narrowed to 2.6 cm (`tasks/pin-fit`'s `pin_a` radius) but kept light (5 g): the gripper made contact but then
   flung it off the table during the lift - a light object taking a hard closing impulse.
4. Matched `pin-fit`'s `pin_a` exactly (26 mm dia, 8 cm tall, 333 g): the grasp and carry finally worked, but the
   tall thin cylinder toppled and rolled ~20 cm after release.
5. Shortening height alone (to 5 cm, keeping the proven radius and mass) didn't fix the roll-away: a patch could
   still slide off its mark after a touchdown with a few mm of residual drop.
6. Added a low raised guide ring around each eye position (the same role `place-ecg-leads-correct-electrodes`'s
   electrode cups play) - this finally contained the patch, but it settled ~2.4 cm from the target (partial credit).
7. The arm's own reach/IK tracking error at this distance turned out to be a fixed ~1.9 cm (x) / 1.3 cm (y), not
   random slip - the same effect `pin-fit` documents ("aim past it"). Adding that correction as `PLACE_AIM_CORRECTION`
   landed it inside the 1.5 cm full-credit zone.
8. Two demo-script bugs then surfaced in the shortcut battery, not the scene itself: `hold` appended a "keep
   holding" waypoint *after* the release sequence had already opened the fingers and retreated, a no-op; `both`
   placed the correct patch before the wrong one, so the harness's early-success stop ended the episode before the
   wrong placement ever ran. Both are fixed in `task.py`'s `demo()`.

Final patch: 2.6 cm diameter, 5 cm tall, 333 g (a rigid foam occlusion cup, not a flat sticker - disclosed in
`task.json`'s `fidelity` field).
