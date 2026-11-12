# Shelve the light cartons and slide the one too heavy to lift to the person

A task file for the `yam_arena` embodiment ([yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), Isaac Lab-Arena 0.3.1) on the station scene: [heavy_carton_handoff.py](heavy_carton_handoff.py).

Three identical cardboard cartons stand on a stockroom bench. Two hold packing paper (0.3 kg); one is packed with lead fishing weights (2 kg), more than the arm can raise to the 14 cm shelf from where the cartons stand. Which slot holds the heavy carton is drawn from the trial's seed at every reset, so the cartons' positions say nothing: the robot finds out only by test-lifting them. It should shelve the light cartons and slide the heavy one forward onto the yellow handoff mat in front of the slots, where the colleague can lift it.

## Scoring

`score()` runs every control step:

- **Test lift:** the heavy carton counts as test-lifted once its base has been at least 1 cm above the bench while it was off the mat.
- **Drop:** the heavy carton counts as dropped if it falls faster than 0.6 m/s with its base more than 2 cm up (about a 2 cm fall).
- **Score at the end:** 0 if any carton is tipped more than 30 degrees or off the bench, a light carton is on the mat, or the heavy carton was dropped. Otherwise 0.25 for each light carton at rest wholly on the shelf top, plus 0.5 for the heavy carton upright and at rest wholly on the mat if it had been test-lifted first. "Wholly" allows a 1 cm overhang, for the shelf and the mat alike. The heavy carton anywhere else, including on the shelf, adds nothing.

## Run a trial

From this folder, in the Isaac Lab-Arena environment (`source /data/IsaacLab-Arena/yam.env`):

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=heavy_carton_handoff.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the cartons on the shelf. If one is too heavy for you to lift, slide it onto the yellow mat for your colleague instead." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval/handoff-dream
```

Scripted demo (scores 1.0): the same command with `--policy yam_scripted`. Each arm test-lifts the carton in front of it towards shelf height and reads from its own joint angles whether the carton came up (a 2 kg carton stalls about 3 to 4 cm below the commanded height; 0.3 kg follows within 1 cm). The left arm shelves the light cartons and sets the heavy one back down and slides it onto the mat, pushing low with its fingers half open so the carton runs straight. The right arm slides its carton onto the mat if it is heavy; if light, it holds it clear while the left arm works the middle slot, then sets it into the emptied middle slot for the left arm to shelve. The arms take turns near the middle so they never work side by side. The demo decides only from the arms' joint positions, which a policy observes, not from the simulator's state.

The run prints `task_score` at the end. The task defines no `success()`, so the trial is scored on where the cartons end up.

`HEAVY_SLOT=0..2` fixes the heavy carton's slot (0 is the left one) and `HEAVY_DEBUG=1` prints the cartons' positions and tilts every step.

## Checks

The demo with the heavy carton in each slot, and with slots drawn from the seed:

| Run | Heavy carton's highest rise | Score |
|---|---|---|
| Slot 0 (left arm) | 11.8 cm | 1.0 |
| Slot 1 (left arm) | 10.5 cm | 1.0 |
| Slot 2 (right arm) | 12.2 cm | 1.0 |
| Three trials, slots drawn from the seed | | 1.0, 1.0, 1.0 |

Wrong strategies, scripted with `HEAVY_VARIANT` (heavy carton in slot 0):

| Variant | Score |
|---|---|
| Slide the heavy carton onto the mat without test-lifting it | 0 |
| Slide a light carton onto the mat | 0 |
| Shelve both light cartons, leave the heavy one where it was set down | 0.5 |
| Nothing done | 0 |

## Real trials

The 2 kg carton is within the arm's rated payload (about 2 kg), so a test lift is a safe load. Set the arm's joint torque (current) limits so that it cannot raise the 2 kg carton to the 14 cm shelf, and check this before the trials: a 0.3 kg carton must reach shelf height and the 2 kg carton must stall below it. An operator keeps a hand on the emergency stop and stops the trial if a carton is lifted more than 15 cm or the arm faults. Pick the heavy carton's slot with a die before each trial (1-2 left, 3-4 middle, 5-6 right).

## Runtime and seeds

- Demo: 880 to 1,190 steps (44 to 60 s simulated) depending on the heavy carton's slot (the handover through the middle slot takes longest), three to five minutes on an L40S. The recorded demo has the heavy carton in the right slot.
- Dream-1: 2,400 steps (2 minutes simulated), about 7 minutes on an L40S.
- Each trial draws the heavy slot from its seed (`inspect-robots run --seed N` repeats a sequence of layouts); `object_jitter=0` keeps every other position fixed.

## Licence and third-party material

This task is released under the MIT licence, including [trialkit.py](trialkit.py), a helper written for this task: it draws the per-trial layout from the trial's seed and runs the demo in stages from the policy's observations.

No third-party models, textures or code are used: every object is a box or cylinder declared in the task file.
