# Shuffle a sorted deck and deal it round the table

Six card tiles, two red, two green and two blue, start in one deck sorted by colour. The robot shuffles the deck so
that no two cards of the same colour touch, then deals it from the top, one card at a time, clockwise to three
seats until each seat has two cards.

A dealer does two things here that a pick-and-place policy doesn't: it changes the order of a stack without
spreading the cards out, and it hands out cards strictly in deck order and seat rotation. Sorting by colour is not
enough, and neither is putting two cards on every seat. Each card has to come off the top of the deck that was
shuffled and go to the seat whose turn it is. Restacking with one free hand forces the robot to plan with three
piles, because two piles can only cut or reverse the deck.

## Setup

One fixed layout on the calibrated bimanual YAM station (`SCENE = "station"`, table top 747.5 mm). Coordinates are in
metres from the midpoint between the arm bases: x forward, y toward the left arm. Every value is in `task.py`.

| Object | Type, size, material | Centre (x, y) |
| --- | --- | --- |
| 6 card tiles | Rigid plastic tiles, 90 × 56 × 16 mm, 40 g each; 2 red, 2 green, 2 blue | stacked on the dark mat |
| Deck mat | Dark grey fixed plastic mat, 110 × 80 × 2 mm | 0.22, 0.13 |
| Shuffle mats | 2 light grey fixed plastic mats, 110 × 80 × 2 mm | 0.34, 0.13 and 0.22, 0.00 |
| Seat mats | 3 yellow fixed plastic mats, 130 × 100 × 2 mm | left 0.36, 0.25; middle 0.40, 0.00; right 0.36, −0.25 |

The deck starts red, red, green, green, blue, blue from the bottom, with each card's long side along x. Use the
first top-camera frame as the setup image.

## Scoring

The scorer reads every card's position after each control step and keeps a trial record that is cleared only on
reset.

- **Deck:** six cards stand in one settled stack on the dark mat or one of the grey mats, each card flat (tilted under 15°),
  within 25 mm of the bottom card's centre in x and y and within 5 mm of its level's height.
- **Shuffled:** the last deck formed before any card reaches a seat has no two cards of the same colour next to each
  other.
- **Dealt:** a card is dealt when it first comes to rest (under 1 cm/s), flat, with its centre within 40 mm (x) and
  35 mm (y) of a seat mat's centre.
- **Misdeal:** a card comes to rest off every mat on the table, comes to rest tilted more than 15°, or falls off the
  table. A misdeal scores 0 for the rest of the trial.

**0** unless the deck was shuffled when dealing began. Then **0.25**, plus **0.125** for each deal in an unbroken run
from the first deal in which the card is the next one from the top of the shuffled deck, it went to the seat whose
turn it was (left, middle, right, left, middle, right), and it is still resting flat on that seat. **1.0** when all
six are dealt correctly. The trial ends once the score has held at 1.0 for 3 s; otherwise it scores the last step.

Shortcuts score 0: dealing the sorted deck, cutting the deck (which leaves same-colour pairs), carrying cards to the
seats without first forming a mixed six-card deck, dropping a card, and dealing counter-clockwise from the right
(0.25 only for the shuffle). Taking a card from below the top or skipping a seat ends the credit at that deal.

To score a physical trial, film from above. Check the colour order of the six-card stack before the first deal and
the order in which cards land on the seats, and measure any doubtful card with a ruler against the seat mat.

## Run and score

On the Robocurve Brev launchable (Isaac Lab-Arena with `yam_arena`), from this folder:

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')

# The scripted demo, which reaches full score
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --scorer task_score --no-prompt --max-steps 3000 \
  --store-frames --log-dir runs/demo

# Dream-1
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --scorer task_score --no-prompt --max-steps 3000 \
  --store-frames --log-dir runs/dream

# Offline scorer tests (NumPy and pytest only)
python -m pytest -q test_scoring.py
```

Set `CARD_DEBUG=1` to print each card's mat and height, the recorded deck and the deals every 40 steps.

- **Run limit:** 3,000 control steps (150 s at 20 Hz).
- **Seeds:** one fixed layout with `-E object_jitter=0`, so the seed doesn't change the start.
- **Runtime:** the scripted demo reaches 1.0 after 2,346 steps and the trial ends at 2,406 steps (120 s simulated).

## Validation

The scripted demo scored **1.0** in three separate runs, including two consecutive trials in one session that check the reset. The left arm restacked the deck in eight single-card moves, using both grey mats,
as blue, green, red, green, blue, red from the bottom on the middle mat. It then dealt to the left and middle seats,
and the right arm dealt to the right seat. The 18 offline tests in `test_scoring.py` cover the full task, partial
deals, every shortcut above, reset, and a card held still in the air, which is not a misdeal.

Dream-1 on the same layout scored **0.0** at the 3,000-step limit. It moved the left gripper toward the deck several times but never grasped a card, so the deck stayed sorted on the dark mat and no card reached a seat.

## Fidelity

The cards are 16 mm rigid tiles, not paper cards. The station's parallel grippers only point straight down and can't
pick up anything as thin as a playing card, so each card is thickened to a height the grippers hold reliably. A
shuffle is a sequence of single-card moves between three piles. A riffle or overhand shuffle needs hands. The cards
are coloured on every face, so they are dealt face up. PhysX rigid contact with the station's default friction.

## Licence

Task code is released under the MIT licence. `partkit.py` is copied unchanged from `tasks/din-rail-mount` in this
repository and is used only for its reset hook. The scene uses only primitive boxes, with no third-party assets.
