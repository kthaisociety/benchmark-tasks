"""Trial record and score for the shuffle-and-deal task. NumPy only, so it runs without the simulator.

The tracker sees every card's pose after every control step. It remembers the last time all six cards stood in
one settled stack before the first card reached a seat (the deck that was dealt), the order in which cards came
to rest on the seats, and any misdeal. A misdeal latches for the rest of the trial and is cleared only on reset.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

CARD_THICKNESS = 0.016
REST_SPEED = 0.01  # m/s
FLAT_COS = float(np.cos(np.radians(15.0)))
STACK_XY = 0.025  # a card is in a stack when its centre is within this of the bottom card's
STACK_Z = 0.005  # and within this of its level's height
TABLE_LEVEL = 0.03  # a resting card centred lower than this above the table lies on the table
OFF_TABLE = 0.05  # a card centred this far below the table top has fallen off
SHUFFLE_CREDIT = 0.25
DEAL_CREDIT = 0.125


@dataclass(frozen=True)
class Zone:
    name: str
    center: tuple[float, float]
    half: tuple[float, float]  # how far the card centre may sit from the mat centre in x and y

    def holds(self, xy) -> bool:
        return abs(xy[0] - self.center[0]) <= self.half[0] and abs(xy[1] - self.center[1]) <= self.half[1]


@dataclass(frozen=True)
class Card:
    pos: np.ndarray  # centre, world frame (m)
    up: float  # z component of the card's up axis: 1 when flat
    speed: float

    @property
    def resting(self) -> bool:
        return self.speed < REST_SPEED

    @property
    def flat(self) -> bool:
        return self.up >= FLAT_COS


def up_component(quat_xyzw) -> float:
    x, y, _, _ = (float(v) for v in quat_xyzw)
    return 1.0 - 2.0 * (x * x + y * y)


def no_matching_neighbours(order, colors) -> bool:
    return all(colors[a] != colors[b] for a, b in zip(order, order[1:]))


@dataclass
class Tracker:
    colors: dict  # card name -> colour name
    piles: tuple  # Zone for each mat a deck may stand on
    seats: tuple  # Zone for each seat, in dealing order
    table_top: float
    deck: list | None = None  # last settled six-card stack, bottom to top, before the first deal
    dealt_deck: list | None = None  # the deck as it was when the first card reached a seat
    deals: list = field(default_factory=list)  # (card, seat) in the order the cards came to rest
    misdeal: str | None = None

    def reset(self) -> None:
        self.deck, self.dealt_deck, self.misdeal = None, None, None
        self.deals = []

    def zone_of(self, card: Card):
        for zone in (*self.piles, *self.seats):
            if zone.holds(card.pos):
                return zone.name
        return None

    def _stack(self, cards: dict):
        """The six cards bottom to top if they stand settled and flat in one stack on one mat, else None."""
        if not all(c.resting and c.flat for c in cards.values()):
            return None
        zones = {self.zone_of(c) for c in cards.values()}
        if len(zones) != 1 or zones.pop() not in {p.name for p in self.piles}:
            return None
        order = sorted(cards, key=lambda n: cards[n].pos[2])
        base = cards[order[0]].pos
        bottom = self.table_top + CARD_THICKNESS / 2
        for level, name in enumerate(order):
            pos = cards[name].pos
            if np.hypot(*(pos[:2] - base[:2])) > STACK_XY:
                return None
            # The lowest card may sit on a thin mat, so measure each level from the card below it.
            expected = bottom if level == 0 else cards[order[level - 1]].pos[2] + CARD_THICKNESS
            if level == 0 and not 0 <= pos[2] - bottom <= STACK_Z:
                return None
            if level and abs(pos[2] - expected) > STACK_Z:
                return None
        return order

    def update(self, cards: dict) -> float:
        for name, card in cards.items():
            if card.pos[2] < self.table_top - OFF_TABLE:
                self.misdeal = self.misdeal or f"{name} fell off the table"
            elif card.resting and not card.flat:
                self.misdeal = self.misdeal or f"{name} came to rest tilted"
            elif card.resting and self.zone_of(card) is None and card.pos[2] < self.table_top + TABLE_LEVEL:
                self.misdeal = self.misdeal or f"{name} came to rest off the mats"

        dealt = {name for name, _ in self.deals}
        if not self.deals:
            stack = self._stack(cards)
            if stack is not None:
                self.deck = stack
        for name, card in cards.items():
            if name in dealt or not (card.resting and card.flat):
                continue
            seat = next((s.name for s in self.seats if s.holds(card.pos)), None)
            if seat is None:
                continue
            if not self.deals:
                self.dealt_deck = list(self.deck) if self.deck is not None else None
            self.deals.append((name, seat))
            dealt.add(name)
        return self.score(cards)

    @property
    def shuffled(self) -> bool:
        return self.dealt_deck is not None and no_matching_neighbours(self.dealt_deck, self.colors)

    def score(self, cards: dict) -> float:
        if self.misdeal or not self.shuffled:
            return 0.0
        top_first = self.dealt_deck[::-1]
        seat_names = [s.name for s in self.seats]
        value = SHUFFLE_CREDIT
        for index, (name, seat) in enumerate(self.deals):
            card = cards[name]
            still_there = card.resting and card.flat and self.zone_of(card) == seat
            if name != top_first[index] or seat != seat_names[index % len(seat_names)] or not still_there:
                break
            value += DEAL_CREDIT
        return round(value, 6)
