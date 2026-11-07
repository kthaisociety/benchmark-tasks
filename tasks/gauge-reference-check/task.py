"""A go/no-go test station on the table. The plain grey block on the orange mat is a part that has just failed the
gauge. The grey block with a green top band, in the green holder, is the gauge's known-good reference block. The
gauge has a raised steel test pad, a black TEST button on a pedestal to the right of the pad, and two lamps on a
housing behind the pad: green for PASS and red for FAIL. Pressing TEST tests whatever stands on the pad and lights
one lamp until the next test. Left of the gauge is a yellow tag plate; the yellow out-of-service tag (red top band)
stands on a yellow mat at the back. The red open bin is quarantine; the blue open tray is retest.

Each run has two trials with the same layout. In trial 1 the gauge is in calibration; in trial 2 its limit has
drifted and it fails every block. Nothing visible tells them apart until a block is tested.
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from partkit import Slider, env, on_reset, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- layout (world x forward, y left, metres; heights above the table top) -----------------------------------------
GAUGE_XY = (0.38, 0.0)
PAD_XY = (0.36, 0.0)           # test pad centre
PAD_TOP = 0.013                # pad top above the table (gauge spawned 1 mm up)
BUTTON_XY = (0.38, -0.14)
BUTTON_TOP = 0.043             # button cap top above the table at rest
TAG_PLATE_XY = (0.38, 0.13)
HOLDER_XY = (0.24, -0.12)      # reference block's home
SUSPECT_XY = (0.24, 0.13)      # failed part starts here
TAG_HOME_XY = (0.50, 0.12)
QUARANTINE_XY = (0.33, 0.29)
RETEST_XY = (0.47, 0.27)

BLOCK = (0.04, 0.04, 0.05)     # part and reference block, aluminium
TAG = (0.05, 0.035, 0.06)      # out-of-service tag stand, polymer

GREY = (0.62, 0.63, 0.65)
STEEL = (0.45, 0.48, 0.52)
DARK = (0.18, 0.18, 0.20)


def frame(name, xy, pocket, color, wall=0.006, height=0.02):
    """A fixed open frame (four walls on the table) round a ``pocket`` (x, y) opening."""
    x, y = xy
    px, py = pocket
    z = TOP + height / 2
    long_x = (px + 2 * wall, wall, height)
    long_y = (wall, py, height)
    return [Box(f"{name}_wall_near", pos=(x - (px + wall) / 2, y, z), size=long_y, color=color),
            Box(f"{name}_wall_far", pos=(x + (px + wall) / 2, y, z), size=long_y, color=color),
            Box(f"{name}_wall_left", pos=(x, y + (py + wall) / 2, z), size=long_x, color=color),
            Box(f"{name}_wall_right", pos=(x, y - (py + wall) / 2, z), size=long_x, color=color)]


def mat(name, xy, size, color):
    return Box(name, pos=(xy[0], xy[1], TOP + 0.001), size=(size[0], size[1], 0.002), color=color)


# The gauge is one heavy compound part (10 kg; the arms push about 2 N). Local frame origin at GAUGE_XY, table level.
gx, gy = GAUGE_XY
BUTTON = Slider("button", rest=(0.0, -0.14, 0.03), travel=0.008, rate=200.0, damping=5.0,
                boxes=(((0.0, -0.14, 0.036), (0.035, 0.035, 0.012), DARK),))
# Each lamp is a fixed coloured lens under a dark shutter. At rest the shutter covers the lens (lamp off from the
# first frame); the task lowers it 13 mm into the housing to show the lamp.
HOUSING = (0.30, 0.30, 0.33)
LENSES = {"pass": (-0.04, (0.10, 0.85, 0.25)), "fail": (0.02, (0.90, 0.12, 0.10))}
SHUTTERS = {k: Slider(f"{k}_shutter", rest=(0.08, ly, 0.06), travel=0.013, rate=3000.0, damping=20.0,
                      boxes=(((0.08, ly, 0.0635), (0.028, 0.028, 0.006), HOUSING),))
            for k, (ly, _) in LENSES.items()}
GAUGE = part("gauge", pos=(gx, gy, TOP + 0.001), mass=10.0, color=DARK,
             sliders=(BUTTON, SHUTTERS["pass"], SHUTTERS["fail"]),
             boxes=[((0.02, 0.0, 0.002), (0.17, 0.34, 0.004), DARK),                 # base plate
                    ((PAD_XY[0] - gx, 0.0, 0.006), (0.08, 0.08, 0.012), STEEL),      # test pad
                    ((0.08, -0.01, 0.03), (0.04, 0.12, 0.06), HOUSING),              # lamp housing
                    *(((0.08, ly, 0.0575), (0.025, 0.025, 0.006), c) for ly, c in LENSES.values()),  # lenses
                    ((0.0, -0.14, 0.01), (0.05, 0.05, 0.02), DARK),                  # button pedestal
                    ((0.0, 0.13, 0.003), (0.08, 0.08, 0.006), (0.95, 0.80, 0.10))])  # tag plate

PART = Box("part", pos=(SUSPECT_XY[0], SUSPECT_XY[1], TOP + BLOCK[2] / 2 + 0.003), size=BLOCK, mass=0.216,
           color=GREY)
# partkit reports a part's bounds around its body origin, so both loose parts are built centred on it.
REFERENCE = part("reference", pos=(HOLDER_XY[0], HOLDER_XY[1], TOP + BLOCK[2] / 2 + 0.003), mass=0.216,
                 color=GREY,
                 boxes=[((0, 0, -0.006), (0.04, 0.04, 0.038), GREY),
                        ((0, 0, 0.019), (0.04, 0.04, 0.012), (0.10, 0.70, 0.25))])     # green band on top
TAG_STAND = part("tag", pos=(TAG_HOME_XY[0], TAG_HOME_XY[1], TOP + TAG[2] / 2 + 0.003), mass=0.06,
                 color=(0.95, 0.80, 0.10),
                 boxes=[((0, 0, -0.005), (TAG[0], TAG[1], 0.05), (0.95, 0.80, 0.10)),
                        ((0, 0, 0.025), (TAG[0], TAG[1], 0.01), (0.85, 0.10, 0.08))])  # red band on top

POCKET = (0.08, 0.08)
BIN_POCKET = (0.10, 0.10)
OBJECTS = [
    GAUGE,
    mat("suspect_mat", SUSPECT_XY, (0.08, 0.08), (0.95, 0.50, 0.10)),
    mat("tag_mat", TAG_HOME_XY, (0.08, 0.07), (0.95, 0.80, 0.10)),
    *frame("holder", HOLDER_XY, POCKET, (0.10, 0.60, 0.25)),
    *frame("quarantine", QUARANTINE_XY, BIN_POCKET, (0.80, 0.10, 0.08)),
    *frame("retest", RETEST_XY, BIN_POCKET, (0.10, 0.30, 0.80)),
    PART, REFERENCE, TAG_STAND,
]

# -- trial record ---------------------------------------------------------------------------------------------------
# The task module lives across trials: everything below is cleared in the reset hook, never "back at start".
_S = {"trial": 0, "demo_trial": 0}


def gauge_ok(trial):
    """Trial 1, 3, ... the gauge is in calibration; trial 2, 4, ... it has drifted and fails everything."""
    return trial % 2 == 1


def new_record(trial):
    # "shown" starts unset (not None) so the first step darkens both lamps.
    return {"trial": trial, "pressed": False, "lamp": None, "shown": "unset", "checked": False, "wrong": False,
            "decided_early": False, "tests": []}


_R = new_record(1)


@on_reset
def _reset(environment=None):
    global _R
    _S["trial"] += 1
    _R = new_record(_S["trial"])


# -- geometry helpers (pure: unit-tested without the simulator) ------------------------------------------------------
def inside(state, xy, pocket, max_bottom=0.02):
    """Centre within the pocket's walls (x, y) and the bottom less than ``max_bottom`` above the table."""
    cx, cy = state.pos[0], state.pos[1]
    return (abs(cx - xy[0]) < pocket[0] / 2 and abs(cy - xy[1]) < pocket[1] / 2
            and state.low[2] - TOP < max_bottom)


def on_pad(state):
    """Standing on the test pad: centre within 2 cm of the pad centre, bottom within 6 mm of the pad top."""
    return (abs(state.pos[0] - PAD_XY[0]) < 0.02 and abs(state.pos[1] - PAD_XY[1]) < 0.02
            and abs(state.low[2] - TOP - PAD_TOP) < 0.006)


def judge(record, objects, pressed_now):
    """Update the trial record from one step's object states and return the trial score."""
    part_s, ref_s, tag_s = objects["part"], objects["reference"], objects["tag"]
    ok = gauge_ok(record["trial"])

    # A test happens on the press edge, on whatever stands on the pad.
    if pressed_now and not record["pressed"]:
        ref_on, part_on = on_pad(ref_s), on_pad(part_s)
        if ref_on or part_on:
            passed = ok and ref_on and not part_on
            record["lamp"] = "pass" if passed else "fail"
            record["tests"].append({"reference": ref_on, "part": part_on, "lamp": record["lamp"]})
            if ref_on and not part_on:
                record["checked"] = True
    record["pressed"] = pressed_now

    part_q = inside(part_s, QUARANTINE_XY, BIN_POCKET)
    part_r = inside(part_s, RETEST_XY, BIN_POCKET)
    tag_on = inside(tag_s, TAG_PLATE_XY, (0.08, 0.08), max_bottom=0.02)
    ref_binned = inside(ref_s, QUARANTINE_XY, BIN_POCKET) or inside(ref_s, RETEST_XY, BIN_POCKET)

    # Latched for the whole trial: a disposition before the reference check, or a wrong one.
    if (part_q or part_r or tag_on) and not record["checked"]:
        record["decided_early"] = True
    if ref_binned or (ok and (part_r or tag_on)) or (not ok and part_q):
        record["wrong"] = True

    if not record["checked"]:
        return 0.0
    score = 0.25
    if inside(ref_s, HOLDER_XY, POCKET) and ref_s.resting():
        score += 0.25
    if ok:
        score += 0.5 if part_q and part_s.resting() else 0.0
    else:
        score += 0.25 if tag_on and tag_s.resting() else 0.0
        score += 0.25 if part_r and part_s.resting() else 0.0
    if record["wrong"] or record["decided_early"]:
        score = min(score, 0.25)
    return score


def _set_lamps(environment, lamp):
    g = environment.unwrapped.scene[GAUGE.name]
    target = g.data.joint_pos_target.clone()
    names = list(g.joint_names)
    for k in LENSES:   # shutter at rest = lamp dark; shutter lowered 13 mm = lamp showing
        target[:, names.index(f"{k}_shutter_slide")] = -0.013 if lamp == k else 0.0
    g.set_joint_position_target(target)


def _show(lamp):
    if _R["shown"] == lamp:
        return
    _R["shown"] = lamp
    _set_lamps(env(), lamp)


def _darken_before_reset(module):
    """Wrap the embodiment's reset so both lamps are dark before the reset settles: the first frame of a trial is
    taken before partkit's on_reset hooks run, and the shutter targets survive a reset."""
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_gauge_lamps_patched", False):
        return
    base_reset = cls.reset

    def reset(self, *a, **k):
        if getattr(self, "_env", None) is not None:
            _set_lamps(self._env, None)
        return base_reset(self, *a, **k)

    cls.reset = reset
    cls._gauge_lamps_patched = True


def _install_lamp_reset():
    import partkit
    name = "yam_arena.inspect_embodiment"
    if name in sys.modules:
        _darken_before_reset(sys.modules[name])
    else:
        previous = partkit._Finder.targets[name]
        partkit._Finder.targets[name] = lambda m: (previous(m), _darken_before_reset(m))


_install_lamp_reset()


def score(objects):
    pressed = objects[GAUGE.name].sliders["button"] < -0.004     # 0 at rest, negative when pressed
    result = judge(_R, objects, pressed)
    _show(_R["lamp"])
    return result


# -- scripted demo ---------------------------------------------------------------------------------------------------
GRIP = 0.030          # TCP above a block's bottom for a mid-height grasp
TRAVEL = TOP + 0.16
PARK = {"right": (0.12, -0.42, TOP + 0.20), "left": (0.12, 0.42, TOP + 0.20)}


def _xyz(xy, bottom):
    return (xy[0], xy[1], TOP + bottom + GRIP)


def demo_plan(ok):
    """Both outcomes share the reference check (right arm); the left arm acts on what the lamp showed."""
    press = TOP + BUTTON_TOP
    bx, by = BUTTON_XY
    right = pick_place_waypoints(_xyz(HOLDER_XY, 0.0), _xyz(PAD_XY, PAD_TOP + 0.003), travel_z=TRAVEL)
    right += [Waypoint(1.5, (bx, by, press + 0.06), FINGER_CLOSED, "above TEST"),
              Waypoint(1.2, (bx, by, press + 0.01), FINGER_CLOSED, "touch TEST"),
              Waypoint(1.2, (bx, by, press - 0.012), FINGER_CLOSED, "press TEST"),
              Waypoint(0.5, (bx, by, press - 0.012), FINGER_CLOSED, "hold"),
              Waypoint(1.0, (bx, by, press + 0.06), FINGER_CLOSED, "release TEST"),
              Waypoint(1.5, (bx, by, press + 0.06), FINGER_CLOSED, "read lamp")]
    right += pick_place_waypoints(_xyz(PAD_XY, PAD_TOP), _xyz(HOLDER_XY, 0.003), travel_z=TRAVEL)
    right += [Waypoint(1.5, PARK["right"], FINGER_OPEN, "clear")]

    wait = sum(w.duration for w in right[:8]) + 7.0        # after the press and the lamp read
    left = [Waypoint(wait, PARK["left"], FINGER_OPEN, "wait for the reference check")]
    if ok:
        left += pick_place_waypoints(_xyz(SUSPECT_XY, 0.0), _xyz(QUARANTINE_XY, 0.003), travel_z=TRAVEL)
    else:
        left += pick_place_waypoints(_xyz(TAG_HOME_XY, 0.0), _xyz(TAG_PLATE_XY, 0.007), travel_z=TRAVEL)
        left += pick_place_waypoints(_xyz(SUSPECT_XY, 0.0), _xyz(RETEST_XY, 0.003), travel_z=TRAVEL)
    left += [Waypoint(1.5, PARK["left"], FINGER_OPEN, "clear")]
    return {"right": right, "left": left}


def demo():
    # A scripted stand-in for an operator who reads the lamp: yam_scripted plays open loop, so the demo is written
    # for each outcome and plays them in trial order (trial 1 in calibration, trial 2 drifted).
    _S["demo_trial"] += 1
    return demo_plan(gauge_ok(_S["demo_trial"]))
