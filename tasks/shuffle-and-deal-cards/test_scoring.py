"""Offline checks of the scorer with hand-built card poses: `python -m pytest -q test_scoring.py`."""

import numpy as np
import pytest

import scoring

TOP = 0.7475
T = scoring.CARD_THICKNESS
COLOURS = dict(zip([f"c{i}" for i in range(6)], ["red", "red", "green", "green", "blue", "blue"]))
PILES = (
    scoring.Zone("deck_mat", (0.32, 0.08), (0.03, 0.025)),
    scoring.Zone("grey_right", (0.32, -0.08), (0.03, 0.025)),
    scoring.Zone("grey_near", (0.22, 0.0), (0.03, 0.025)),
)
SEATS = (
    scoring.Zone("seat_left", (0.38, 0.27), (0.04, 0.035)),
    scoring.Zone("seat_far", (0.47, 0.0), (0.04, 0.035)),
    scoring.Zone("seat_right", (0.38, -0.27), (0.04, 0.035)),
)
CENTRE = {z.name: z.center for z in (*PILES, *SEATS)}
MIXED = ["c4", "c2", "c0", "c3", "c5", "c1"]  # blue green red green blue red, bottom to top


def tracker():
    return scoring.Tracker(colors=COLOURS, piles=PILES, seats=SEATS, table_top=TOP)


def layout(piles, held=None):
    """Cards at rest in ``piles`` (zone -> bottom-to-top names); ``held`` maps a name to a moving mid-air pose."""
    cards = {}
    for zone, names in piles.items():
        for level, name in enumerate(names):
            pos = np.array([*CENTRE[zone], TOP + 0.002 + T / 2 + level * T])
            cards[name] = scoring.Card(pos=pos, up=1.0, speed=0.0)
    for name, pos in (held or {}).items():
        cards[name] = scoring.Card(pos=np.array(pos), up=1.0, speed=0.2)
    return cards


def deal_states(deck, count, seats=("seat_left", "seat_far", "seat_right")):
    """The table after ``count`` cards are dealt from the top of ``deck`` on the right grey mat."""
    remaining = list(deck)
    piles = {"grey_right": remaining, **{s: [] for s in seats}}
    states = []
    for index in range(count):
        card = remaining.pop()
        piles[seats[index % 3]].append(card)
        states.append(layout({k: list(v) for k, v in piles.items()}))
    return states


def run(states):
    t = tracker()
    value = 0.0
    for state in states:
        value = t.update(state)
    return t, value


def test_start_scores_zero():
    _, value = run([layout({"deck_mat": [f"c{i}" for i in range(6)]})])
    assert value == 0.0


def test_full_task_scores_one():
    states = [layout({"deck_mat": [f"c{i}" for i in range(6)]}), layout({"grey_right": MIXED})]
    states += deal_states(MIXED, 6)
    t, value = run(states)
    assert value == 1.0
    assert t.shuffled and len(t.deals) == 6


@pytest.mark.parametrize("count, expected", [(1, 0.375), (3, 0.625), (5, 0.875)])
def test_partial_deals(count, expected):
    states = [layout({"grey_right": MIXED})] + deal_states(MIXED, count)
    assert run(states)[1] == expected


def test_shuffle_alone_needs_a_deal_to_count():
    # Credit is given when dealing starts from a mixed deck; restacking alone earns nothing yet.
    assert run([layout({"grey_right": MIXED})])[1] == 0.0
    assert run([layout({"grey_right": MIXED})] + deal_states(MIXED, 1))[1] == 0.375


def test_dealing_the_sorted_deck_scores_zero():
    sorted_deck = [f"c{i}" for i in range(6)]
    states = [layout({"grey_right": sorted_deck})] + deal_states(sorted_deck, 6)
    assert run(states)[1] == 0.0


def test_a_cut_deck_with_pairs_scores_zero():
    cut = ["c2", "c3", "c4", "c5", "c0", "c1"]
    assert run([layout({"grey_right": cut})] + deal_states(cut, 6))[1] == 0.0


def test_dealing_without_ever_forming_a_deck_scores_zero():
    # Cards carried straight from two partial piles to the seats never formed a mixed six-card deck.
    states = [
        layout({"deck_mat": ["c0", "c1", "c2"], "grey_near": ["c3", "c4", "c5"]}),
        layout({"deck_mat": ["c0", "c1"], "grey_near": ["c3", "c4", "c5"], "seat_left": ["c2"]}),
    ]
    assert run(states)[1] == 0.0


def test_wrong_rotation_stops_credit():
    states = [layout({"grey_right": MIXED})]
    states += deal_states(MIXED, 6, seats=("seat_left", "seat_right", "seat_far"))
    assert run(states)[1] == 0.375  # the first card went to the right seat, the second did not


def test_counter_clockwise_from_the_right_scores_only_the_shuffle():
    states = [layout({"grey_right": MIXED})]
    states += deal_states(MIXED, 6, seats=("seat_right", "seat_far", "seat_left"))
    assert run(states)[1] == 0.25


def test_card_taken_from_the_middle_breaks_the_deal():
    remaining = list(MIXED)
    second = remaining.pop(-2)  # the second card from the top, not the top card
    states = [layout({"grey_right": MIXED}), layout({"grey_right": remaining, "seat_left": [second]})]
    assert run(states)[1] == 0.25


def test_card_moved_off_its_seat_loses_its_credit():
    states = [layout({"grey_right": MIXED})] + deal_states(MIXED, 6)
    last = states[-1]
    moved = dict(last)
    moved["c3"] = scoring.Card(pos=np.array([*CENTRE["seat_far"], TOP + 0.042]), up=1.0, speed=0.0)
    # c3 was the third card from the top and was dealt to the right seat; it now rests on the far seat instead.
    assert run(states + [moved])[1] == 0.5


def test_dropped_card_latches_a_misdeal():
    dropped = layout({"grey_right": MIXED[:-1]})
    dropped["c1"] = scoring.Card(pos=np.array([0.40, 0.15, TOP + 0.008]), up=1.0, speed=0.0)
    states = [layout({"grey_right": MIXED}), dropped, layout({"grey_right": MIXED})] + deal_states(MIXED, 6)
    t, value = run(states)
    assert value == 0.0 and "off the mats" in t.misdeal


def test_tilted_or_fallen_card_latches_a_misdeal():
    tilted = layout({"grey_right": MIXED})
    tilted["c1"] = scoring.Card(pos=tilted["c1"].pos, up=0.8, speed=0.0)
    assert run([tilted] + deal_states(MIXED, 6))[1] == 0.0
    fallen = layout({"grey_right": MIXED[:-1]})
    fallen["c1"] = scoring.Card(pos=np.array([0.5, 0.0, TOP - 0.3]), up=1.0, speed=0.5)
    assert run([layout({"grey_right": MIXED}), fallen] + deal_states(MIXED, 6))[1] == 0.0


def test_a_card_held_still_in_the_air_is_not_a_misdeal():
    hover = layout({"grey_right": MIXED[:-1]})
    hover["c1"] = scoring.Card(pos=np.array([0.40, 0.15, TOP + 0.12]), up=1.0, speed=0.0)
    assert run([layout({"grey_right": MIXED}), hover] + deal_states(MIXED, 6))[1] == 1.0


def test_misaligned_stack_is_not_a_deck():
    cards = layout({"grey_right": MIXED})
    cards["c0"] = scoring.Card(pos=cards["c0"].pos + [0.0, 0.02, 0.0], up=1.0, speed=0.0)
    cards["c0"] = scoring.Card(pos=cards["c0"].pos + [0.0, 0.01, 0.0], up=1.0, speed=0.0)
    assert tracker()._stack(cards) is None


def test_reset_clears_the_record():
    t, _ = run([layout({"grey_right": MIXED})] + deal_states(MIXED, 6))
    t.reset()
    assert t.deals == [] and t.deck is None and t.misdeal is None
    assert t.update(layout({"deck_mat": [f"c{i}" for i in range(6)]})) == 0.0
