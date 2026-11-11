"""On the station table, an exhibition hall floor before the doors open: booths divide it into a grid of aisles, some
aisle segments are closed off by booths built across them, and there are two fire exits on different walls. Stewards
tape a floor arrow at each marked aisle junction pointing along the aisles toward the nearest exit by walking distance,
which for several junctions is not the exit nearest in a straight line.
"""

import heapq
import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
FLOOR_T, FLAT = 0.002, 0.0006
BASE = TOP + FLOOR_T

HALL, AISLE, CARPET, STAND, WALL, EXIT, SPOT, MAT, HEAD = (0.70, 0.70, 0.68), (0.55, 0.56, 0.58), \
    (0.30, 0.38, 0.62), (0.85, 0.85, 0.82), (0.25, 0.25, 0.27), (0.10, 0.62, 0.25), (0.95, 0.80, 0.10), \
    (0.96, 0.96, 0.94), (0.10, 0.62, 0.25)

XS, YS = (0.28, 0.38, 0.48), (0.20, 0.07, -0.07, -0.20)
X0, X1, Y0, Y1 = 0.235, 0.525, -0.265, 0.265
AISLE_W = 0.04
NODES = [(i, j) for i in range(3) for j in range(4)]
POS = {n: (XS[n[0]], YS[n[1]]) for n in NODES}
EDGES = [((i, j), (i, j + 1)) for i in range(3) for j in range(3)] + [((i, j), (i + 1, j)) for i in range(2) for j in range(4)]
CLOSED = {((1, 2), (1, 3)), ((2, 1), (2, 2)), ((0, 1), (1, 1))}
EXITS = {"far_exit": ((2, 1), (1, 0)), "side_exit": ((1, 3), (0, -1))}
SIGNED = ((0, 0), (0, 1), (1, 2), (2, 2))
MAT_L, MAT_W, MAT_H, HEAD_L = 0.06, 0.022, 0.016, 0.016
STARTS = {"arrow_a": (0.20, 0.13), "arrow_b": (0.20, 0.05), "arrow_c": (0.20, -0.05), "arrow_d": (0.20, -0.13)}
ARROW_AT = dict(zip(STARTS, SIGNED))


def _exit_xy(name):
    n, d = EXITS[name]
    wall = X1 if d[0] else (Y0 if d[1] < 0 else Y1)
    return (wall, POS[n][1]) if d[0] else (POS[n][0], wall)


def _walking(start):
    adj = {n: [] for n in NODES}
    for a, b in EDGES:
        if (a, b) not in CLOSED:
            w = math.dist(POS[a], POS[b])
            adj[a].append((b, w))
            adj[b].append((a, w))
    best = {}
    for name, (n0, _) in EXITS.items():
        dist, pq = {n0: math.dist(POS[n0], _exit_xy(name))}, [(math.dist(POS[n0], _exit_xy(name)), n0)]
        while pq:
            d, n = heapq.heappop(pq)
            if d > dist[n]:
                continue
            for m, w in adj[n]:
                if d + w < dist.get(m, 9.0):
                    dist[m] = d + w
                    heapq.heappush(pq, (d + w, m))
        best[name] = dist
    name = min(best, key=lambda e: best[e].get(start, 9.0))
    dist = best[name]
    n0, d0 = EXITS[name]
    if start == n0:
        return d0
    m = next(m for m, w in adj[start] if abs(dist.get(m, 9.0) + w - dist[start]) < 1e-9)
    return int(np.sign(POS[m][0] - POS[start][0])), int(np.sign(POS[m][1] - POS[start][1]))


ANSWER = {n: _walking(n) for n in SIGNED}


def _hall():
    cx, cy = (X0 + X1) / 2, (Y0 + Y1) / 2
    out = [Box("hall_floor", pos=(cx, cy, TOP + FLOOR_T / 2), size=(X1 - X0, Y1 - Y0, FLOOR_T), color=HALL)]
    for i, x in enumerate(XS):
        out.append(Box(f"aisle_x_{i}", pos=(x, cy, BASE + FLAT / 2), size=(AISLE_W, Y1 - Y0, FLAT), color=AISLE))
    for j, y in enumerate(YS):
        out.append(Box(f"aisle_y_{j}", pos=(cx, y, BASE + FLAT / 2), size=(X1 - X0, AISLE_W, FLAT), color=AISLE))
    xs_edges, ys_edges = (X0, *XS, X1), (Y1, *YS, Y0)
    k = 0
    for i in range(len(xs_edges) - 1):
        for j in range(len(ys_edges) - 1):
            a, b, c, d = xs_edges[i], xs_edges[i + 1], ys_edges[j], ys_edges[j + 1]
            ax, bx = a + (AISLE_W / 2 if a in XS else 0.0), b - (AISLE_W / 2 if b in XS else 0.0)
            cy_, dy = c - (AISLE_W / 2 if c in YS else 0.0), d + (AISLE_W / 2 if d in YS else 0.0)
            out.append(Box(f"booth_{k}", pos=((ax + bx) / 2, (cy_ + dy) / 2, BASE + FLAT * 1.5),
                           size=(bx - ax - 0.008, cy_ - dy - 0.008, FLAT), color=CARPET))
            k += 1
    for i, (a, b) in enumerate(sorted(CLOSED)):
        (xa, ya), (xb, yb) = POS[a], POS[b]
        along_x = abs(xb - xa) > 1e-9
        size = (abs(xb - xa) - AISLE_W - 0.008, AISLE_W + 0.004, FLAT) if along_x else \
            (AISLE_W + 0.004, abs(yb - ya) - AISLE_W - 0.008, FLAT)
        out += [Box(f"closed_carpet_{i}", pos=((xa + xb) / 2, (ya + yb) / 2, BASE + FLAT * 2), size=size, color=CARPET),
                Box(f"closed_stand_{i}", pos=((xa + xb) / 2, (ya + yb) / 2, BASE + 0.0125), size=(0.03, 0.03, 0.025),
                    color=STAND)]
    gaps = {name: _exit_xy(name) for name in EXITS}
    for name, (x, y), size in (("wall_near", ((X0, 0.0)), (0.006, Y1 - Y0 + 0.006, 0.012)),):
        out.append(Box(name, pos=(x, y, BASE + size[2] / 2), size=size, color=WALL))
    far_gap, side_gap = gaps["far_exit"][1], gaps["side_exit"][0]
    for name, y_from, y_to in (("wall_far_a", Y1, far_gap + 0.03), ("wall_far_b", far_gap - 0.03, Y0)):
        out.append(Box(name, pos=(X1, (y_from + y_to) / 2, BASE + 0.006), size=(0.006, y_from - y_to, 0.012), color=WALL))
    out.append(Box("wall_left", pos=((X0 + X1) / 2, Y1, BASE + 0.006), size=(X1 - X0, 0.006, 0.012), color=WALL))
    for name, x_from, x_to in (("wall_right_a", X0, side_gap - 0.03), ("wall_right_b", side_gap + 0.03, X1)):
        out.append(Box(name, pos=((x_from + x_to) / 2, Y0, BASE + 0.006), size=(x_to - x_from, 0.006, 0.012), color=WALL))
    out += [Box("far_exit_door", pos=(X1 + 0.012, far_gap, BASE + 0.02), size=(0.012, 0.05, 0.04), color=EXIT),
            Box("side_exit_door", pos=(side_gap, Y0 - 0.012, BASE + 0.02), size=(0.05, 0.012, 0.04), color=EXIT)]
    for i, n in enumerate(SIGNED):
        out.append(Box(f"junction_spot_{i}", pos=(*POS[n], BASE + FLAT * 2), size=(0.03, 0.03, FLAT), color=SPOT))
    out.append(Box("steward_table", pos=(0.20, 0.0, TOP + 0.001), size=(0.05, 0.34, 0.002), color=(0.45, 0.32, 0.20)))
    return out


def _mat(name):
    body = MAT_L - HEAD_L
    return part(name, pos=(*STARTS[name], TOP + 0.003), mass=0.04, color=MAT, friction=1.0, boxes=[
        ((-HEAD_L / 2, 0, MAT_H / 2), (body, MAT_W, MAT_H), MAT),
        ((body / 2, 0, MAT_H / 2), (HEAD_L, MAT_W + 0.008, MAT_H), HEAD)])


ARROWS = {n: _mat(n) for n in STARTS}
OBJECTS = [*_hall(), *ARROWS.values()]

DEBUG = os.environ.get("EXIT_DEBUG") == "1"
MODE = os.environ.get("EXIT_MODE", "demo")
SEAT, MAX_ANGLE, MAX_TILT, MAX_SPEED = 0.03, 30.0, 15.0, 0.02
_K = {"k": 0}


def _tilt_deg(s):
    return float(np.degrees(np.arccos(np.clip(s.rotation()[2, 2], -1.0, 1.0))))


def _junction_ok(states, n):
    want = np.array(ANSWER[n], dtype=float)
    for s in states:
        if math.dist(s.pos[:2], POS[n]) > SEAT or _tilt_deg(s) > MAX_TILT or np.linalg.norm(s.lin_vel) > MAX_SPEED:
            continue
        heading = s.rotation()[:2, 0]
        heading = heading / max(np.linalg.norm(heading), 1e-9)
        if np.degrees(np.arccos(np.clip(heading @ want, -1.0, 1.0))) <= MAX_ANGLE:
            return True
    return False


def score(objects):
    """0.25 for each marked junction with an arrow mat at rest within 3 cm of its centre, pointing within 30 degrees
    of the aisle that starts the shortest walking route to an exit."""
    states = [objects[n] for n in ARROWS]
    results = {n: _junction_ok(states, n) for n in SIGNED}
    total = 0.25 * sum(results.values())
    _K["k"] += 1
    if DEBUG and _K["k"] % 40 == 0:
        print(f"EXIT step {_K['k']} score {total} " + " ".join(f"{n}:{int(v)}" for n, v in results.items()) + " " + " ".join(
            f"{n[-1]}:{np.round(objects[n].pos[:2], 3).tolist()}/{math.degrees(math.atan2(objects[n].rotation()[1, 0], objects[n].rotation()[0, 0])):.0f}"
            for n in ARROWS), flush=True)
    return float(total)


TRAVEL = TOP + 0.22
AIM = {"arrow_a": (0.02, 0.0), "arrow_d": (0.01, -0.01)}
PICK_AIM = {"arrow_a": (0.02, 0.0), "arrow_d": (0.02, 0.0)}


def _plan():
    answer = dict(ANSWER)
    if MODE == "straight_line":
        for n in SIGNED:
            ex = min(EXITS, key=lambda e: math.dist(POS[n], _exit_xy(e)))
            dx, dy = np.subtract(_exit_xy(ex), POS[n])
            answer[n] = (int(np.sign(dx)), 0) if abs(dx) >= abs(dy) else (0, int(np.sign(dy)))
    if MODE == "all_far":
        answer = {n: (1, 0) for n in SIGNED}
    return {name: (POS[n], math.atan2(answer[n][1], answer[n][0])) for name, n in ARROW_AT.items()}


def _move(name, xy, yaw1):
    px, py = PICK_AIM.get(name, (0.0, 0.0))
    x0, y0 = STARTS[name][0] + px, STARTS[name][1] + py
    ax, ay = AIM.get(name, (0.0, 0.0))
    tx, ty = xy[0] + ax, xy[1] + ay
    grip = TOP + 0.003 + MAT_H / 2 + 0.008
    rest = BASE + FLAT * 2 + MAT_H / 2 + 0.008 + 0.003
    return [(1.3, (x0, y0, TRAVEL), FINGER_OPEN, "above the arrow", 0.0), (1.1, (x0, y0, grip), FINGER_OPEN, "descend", 0.0),
            (0.4, (x0, y0, grip), FINGER_OPEN, "settle", 0.0), (0.6, (x0, y0, grip), FINGER_CLOSED, "grip", 0.0),
            (1.0, (x0, y0, TRAVEL), FINGER_CLOSED, "lift", 0.0), (2.0, (tx, ty, TRAVEL), FINGER_CLOSED, "carry", yaw1),
            (1.2, (tx, ty, rest + 0.02), FINGER_CLOSED, "lower to the junction", yaw1),
            (0.6, (tx, ty, rest), FINGER_CLOSED, "set down", yaw1), (0.5, (tx, ty, rest), FINGER_OPEN, "let go", yaw1),
            (0.9, (tx, ty, TRAVEL), FINGER_OPEN, "up", yaw1)]


def demo():
    """The left arm sets arrows a and b at the two left junctions near the steward table; the right arm then sets c and
    d at the two right junctions further in."""
    if MODE == "none":
        return {"left": [Waypoint(1.0, (0.12, 0.42, TRAVEL), FINGER_OPEN, "idle", 0.0)]}
    plan = _plan()
    left = [s for n in ("arrow_a", "arrow_b") for s in _move(n, *plan[n])]
    left += [(1.5, (0.12, 0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0)]
    right = [(sum(w[0] for w in left), (0.12, -0.42, TRAVEL), FINGER_OPEN, "wait", 0.0)]
    right += [s for n in ("arrow_c", "arrow_d") for s in _move(n, *plan[n])]
    right += [(1.5, (0.12, -0.42, TRAVEL), FINGER_OPEN, "move clear", 0.0), (2.0, (0.12, -0.42, TRAVEL), FINGER_OPEN, "settle", 0.0)]
    return {"left": [Waypoint(*s) for s in left], "right": [Waypoint(*s) for s in right]}
