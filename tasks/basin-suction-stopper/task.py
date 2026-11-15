"""Take a suction stopper out of a vessel basin's drain, under the faucet's spout, and put it in the soap dish.

A round white ceramic vessel basin (40.6 cm across, 14 cm tall) stands on the counter in front of the arms, its
centre 25 cm out on the midline. It has a grid drain, which never closes, in the middle of its flat floor. A grey
silicone suction stopper (a 7.5 cm disc with a 2.4 cm knob) is pressed down over the drain. A chrome vessel faucet
stands on the counter behind the basin; its square spout reaches 20 cm toward the arms, so its outlet hangs 22 cm
above the counter, 4 cm behind the drain, and its end is over the back edge of the stopper. A ceramic soap dish is
fixed on the counter to the right of the basin, in front of the right arm.

Seal model
- While the stopper is sealed, a constant force F_SEAL (10 N) pulls its centre straight down onto the basin floor.
  Pulled straight up, it lets go once the pull exceeds F_SEAL plus its weight; tilted, it lets go once the turning
  moment exceeds F_SEAL times its radius; pushed sideways, it slides once the push exceeds its friction on the glaze
  (about 0.75 x 10.4 N).
- The seal breaks for good when any point of the disc's rim is more than RIM_BREAK (3 mm) above the floor (lifted or
  tipped), or its centre is more than SLIDE_BREAK (15.5 mm, where the rim reaches the drain opening) from the drain's
  centre (slid off). The force is then off and the stopper is an ordinary loose body. It does not reseal.
- Checked every control step (20 Hz). The 10 N is an estimate; the real build sets the seal so that a luggage scale on
  the knob reads 8 to 12 N when it lets go (see the README).

Scoring (score(), judged at the end; success() ends the episode once 1.0 has held for SUCCESS_HOLD_S)
  0     the stopper is still sealed on the drain
  0.25  the seal is broken but the stopper is still in the basin (or has fallen off the counter)
  0.5   the stopper is out of the basin (on the counter, held in the air, anywhere but the basin and the soap dish)
  0.75  the stopper is in the soap dish, but a gripper is still within CLEAR of it or it is still moving
  1.0   the stopper is in the soap dish, both grippers' fingertips at least CLEAR (6 cm) from its middle, and it is
        still (its middle moved under 2 mm over the last 0.5 s)

Env knobs (defaults are the task): F_SEAL (N, 10); demo only: PULL_TILT (deg, 35), PULL_MM (45), GRASP_DEPTH (mm, 2),
DEMO_VARIANT (full, nothing, weak_pull, drop_back, counter, keep_holding, straight); SINK_DEBUG=1 prints a trace every
4 control steps.
"""
from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import partkit  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from partkit import _np, _t, on_env_reset, on_reset, on_step, part  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, READY, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0
TOP = TABLE_TOP[SCENE]
DEBUG = os.environ.get("SINK_DEBUG", "") not in ("", "0")
F_SEAL = float(os.environ.get("F_SEAL", "10"))          # the suction's hold (N)

# --- geometry (m; world frame: x forward from the arms, y left, z up) ----------------------------------------------
XC, YC = 0.25, 0.0                                      # basin centre on the counter
R_OUT, WALL, H_BASIN, FLOOR_T = 0.203, 0.012, 0.14, 0.015
R_IN = R_OUT - WALL
FLOOR_Z = TOP + FLOOR_T                                 # the basin's flat inner floor
N_WALL = 32
DRAIN_R, GRID_R = 0.022, 0.019                          # 44 mm (1 3/4 in) drain opening; the grid inside its flange
DISC_R, DISC_T, KNOB_R, KNOB_H = 0.0375, 0.006, 0.012, 0.022
STOPPER_H = DISC_T + KNOB_H
STOPPER_MASS = 0.045
MID = np.array([0.0, 0.0, 0.014])                       # the stopper's middle in its own frame (origin: underside)
RIM_BREAK = 0.003
SLIDE_BREAK = DISC_R - DRAIN_R
BASIN_MU, STOPPER_MU = 0.5, 1.0

FAUCET_X = XC + R_OUT + 0.0275 + 0.01                   # column axis: base plate 1 cm clear of the basin
REACH, OUTLET_Z, SPOUT_W = 0.20, 0.22, 0.022            # outlet centre 20 cm from the column axis, 22 cm up
COL_W, COL_H, PLATE_W = 0.035, 0.30, 0.055
OUTLET_X = FAUCET_X - REACH

DISH_XY = (0.22, -0.30)                                 # soap dish centre; its long side along x
DISH_OUT, DISH_H, DISH_WALL, DISH_FLOOR = (0.14, 0.11), 0.02, 0.006, 0.005
DISH_IN = (DISH_OUT[0] - 2 * DISH_WALL, DISH_OUT[1] - 2 * DISH_WALL)

CLEAR = 0.06                                            # fingertips this far from the stopper's middle: let go
STILL_S, STILL_MM = 0.5, 2.0

WHITE, CHROME, DARK, GREYS, DISHC = (0.94, 0.94, 0.93), (0.78, 0.79, 0.81), (0.12, 0.12, 0.13), (0.30, 0.32, 0.35), \
    (0.62, 0.74, 0.80)


def rel(p):
    return tuple(float(v) for v in (np.asarray(p) - np.array([0, 0, TOP])))


def _quat_z(a):
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


def basin_boxes():
    """The basin's wall: N_WALL upright panels round a circle, each a little wider than its facet so they overlap."""
    out, rm = [], R_OUT - WALL / 2
    width = 2 * R_OUT * math.tan(math.pi / N_WALL) + 0.002
    for i in range(N_WALL):
        a = 2 * math.pi * i / N_WALL
        out.append(((XC + rm * math.cos(a), YC + rm * math.sin(a), H_BASIN / 2), (WALL, width, H_BASIN), WHITE,
                    _quat_z(a)))
    return out


BASIN = part("basin", pos=(0, 0, TOP), mass=None, color=WHITE, friction=BASIN_MU, boxes=basin_boxes(), cylinders=[
    ((XC, YC, FLOOR_T / 2), R_IN + 0.001, FLOOR_T, WHITE),                     # the floor
    ((XC, YC, FLOOR_T + 0.0002), DRAIN_R, 0.0004, CHROME, True),               # drain flange (drawn only)
    ((XC, YC, FLOOR_T + 0.0005), GRID_R, 0.0004, DARK, True),                  # its grid (drawn only)
])

_sx0, _sx1 = OUTLET_X - SPOUT_W / 2, FAUCET_X - COL_W / 2                       # the spout runs back to the column
FAUCET = part("faucet", pos=(0, 0, TOP), mass=None, color=CHROME, friction=0.5, boxes=[
    ((FAUCET_X, YC, 0.005), (PLATE_W, PLATE_W, 0.01), CHROME),                  # base plate
    ((FAUCET_X, YC, COL_H / 2), (COL_W, COL_W, COL_H), CHROME),                 # square column
    (((_sx0 + _sx1) / 2, YC, OUTLET_Z + SPOUT_W / 2), (_sx1 - _sx0, SPOUT_W, SPOUT_W), CHROME),   # square spout
    ((OUTLET_X, YC, OUTLET_Z - 0.0003), (0.016, 0.016, 0.0006), DARK, None, True),                  # outlet (drawn)
    ((FAUCET_X + 0.025, YC, COL_H + 0.006), (0.06, 0.012, 0.012), CHROME),      # lever, pointing back
])

_dx, _dy = DISH_XY
DISH = part("soap_dish", pos=(0, 0, TOP), mass=None, color=DISHC, friction=0.6, boxes=[
    ((_dx, _dy, DISH_FLOOR / 2), (DISH_OUT[0], DISH_OUT[1], DISH_FLOOR), DISHC),
    ((_dx, _dy + DISH_OUT[1] / 2 - DISH_WALL / 2, DISH_H / 2), (DISH_OUT[0], DISH_WALL, DISH_H), DISHC),
    ((_dx, _dy - DISH_OUT[1] / 2 + DISH_WALL / 2, DISH_H / 2), (DISH_OUT[0], DISH_WALL, DISH_H), DISHC),
    ((_dx + DISH_OUT[0] / 2 - DISH_WALL / 2, _dy, DISH_H / 2), (DISH_WALL, DISH_IN[1], DISH_H), DISHC),
    ((_dx - DISH_OUT[0] / 2 + DISH_WALL / 2, _dy, DISH_H / 2), (DISH_WALL, DISH_IN[1], DISH_H), DISHC),
])

STOPPER = part("stopper", pos=(XC, YC, FLOOR_Z), mass=STOPPER_MASS, color=GREYS, friction=STOPPER_MU, boxes=[
    ((0, 0, DISC_T + KNOB_H - 0.0005), (0.006, 0.006, 0.001), DARK, None, True),   # a dot on the knob (drawn only)
], cylinders=[
    ((0, 0, DISC_T / 2), DISC_R, DISC_T, GREYS),
    ((0, 0, DISC_T + KNOB_H / 2), KNOB_R, KNOB_H, GREYS),
])

OBJECTS = [BASIN, FAUCET, DISH, STOPPER]

# --- grips for the demo: yaw codes >= 5000 name a stored rotation ----------------------------------------------------
_orig_grip = _demo.grip_rotation
GRIPS: dict[int, np.ndarray] = {}


def grip_code(r):
    code = 5000 + len(GRIPS)
    GRIPS[code] = np.asarray(r, float)
    return float(code)


def _grip(yaw: float = 0.0):
    if yaw >= 4999.5:
        return GRIPS[int(round(yaw))]
    return _orig_grip(yaw)


_demo.grip_rotation = _grip
PULL_TILT = math.radians(float(os.environ.get("PULL_TILT", "35")))
PULL = float(os.environ.get("PULL_MM", "45")) / 1000
GRASP_DEPTH = float(os.environ.get("GRASP_DEPTH", "2")) / 1000


def tilted(tilt=PULL_TILT, yaw=0.0):
    """A downward grip leaned back toward the arms by ``tilt`` (the wrist nearer the arms than the fingertips), its
    fingers closing along world y."""
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), -tilt) @ _orig_grip(yaw)


TILT = grip_code(tilted())
START = grip_code(ik.tcp_pose(np.array(READY, float), np.zeros(3))[1])

# --- state ---------------------------------------------------------------------------------------------------------
_S: dict = {}


@on_reset
def _reset(environment=None):
    _S.clear()
    _S.update(step=0, sealed=True, broke=None, how="", hist=[], score=0.0, why="start", ep_len=-1, hold=0,
              peak_lift=0.0, peak_off=0.0)


@on_env_reset
def _pre_settle(environment):
    """The stopper is put back centred on the drain (undoing any object_jitter: the layout is fixed) and sealed from
    the first physics step, before the embodiment settles the scene."""
    sc = environment.unwrapped.scene
    try:
        _write(sc, "stopper", STOPPER.pos, (0.0, 0.0, 0.0, 1.0))
    except Exception as e:  # noqa: BLE001  (with object_jitter=0 the stopper is already there)
        print(f"[sink] could not re-centre the stopper on reset: {e!r}", flush=True)
    _force(sc, "stopper", (0.0, 0.0, -F_SEAL))


def _write(sc, name, pos, quat_xyzw, lin=(0, 0, 0), ang=(0, 0, 0)):
    import torch
    asset = sc[name]
    origin = _t(sc.env_origins[0])
    pose = torch.tensor([[*map(float, pos), *map(float, quat_xyzw)]], dtype=torch.float32, device=origin.device)
    pose[:, :3] += origin.to(torch.float32)
    asset.write_root_pose_to_sim_index(root_pose=pose)
    v = torch.tensor([[*map(float, lin), *map(float, ang)]], dtype=torch.float32, device=origin.device)
    asset.write_root_velocity_to_sim_index(root_velocity=v)


def _rot(q_xyzw):
    x, y, z, w = q_xyzw
    return partkit._quat_matrix((w, x, y, z))


def _pose(sc, name):
    p = _np(sc[name].data.root_link_pose_w)[0].copy()
    p[:3] -= _np(sc.env_origins[0])
    return p


def _force(sc, name, f):
    """A constant world force on a part's body at its centre of mass (None clears it)."""
    import torch
    comp = sc[name].permanent_wrench_composer
    comp.reset()
    if f is None:
        return
    dev = _t(sc.env_origins[0]).device
    ft = torch.tensor([[[float(v) for v in f]]], dtype=torch.float32, device=dev)
    comp.add_forces_and_torques_index(forces=ft, torques=torch.zeros_like(ft), body_ids=[0], is_global=True)


def _tips(sc):
    """Pad-tip point of each gripper (the TCP moved on to the fingertips)."""
    out = {}
    for key in sc.articulations:
        art = sc.articulations[key]
        bn = list(art.body_names)
        if "gripper" in bn:
            p = _np(art.data.body_link_pose_w)[0, bn.index("gripper")]
            p[:3] -= _np(sc.env_origins[0])
            r = _rot(p[3:7])
            out[key] = p[:3] + r @ np.array(TCP_OFFSET) + r[:, 2] * PAD_TIP_PAST_TCP
    return out


RIM_ANGLES = np.linspace(0.0, 2 * math.pi, 24, endpoint=False)


def seal_findings(pos, r):
    """(highest rim point above the floor, centre's distance from the drain's centre) for a stopper pose."""
    rim = np.stack([np.cos(RIM_ANGLES) * DISC_R, np.sin(RIM_ANGLES) * DISC_R, np.zeros_like(RIM_ANGLES)], axis=1)
    pts = np.asarray(pos, float) + rim @ np.asarray(r, float).T
    lift = float(pts[:, 2].max() - FLOOR_Z)
    off = math.hypot(float(pos[0]) - XC, float(pos[1]) - YC)
    return lift, off


def seal_breaks(lift, off):
    return lift > RIM_BREAK or off > SLIDE_BREAK


def where(mid):
    """(in the basin, in the soap dish, off the counter) for the stopper's middle point."""
    in_basin = math.hypot(float(mid[0]) - XC, float(mid[1]) - YC) < R_IN + 0.005 and mid[2] < TOP + H_BASIN + 0.02
    in_dish = (abs(float(mid[0]) - DISH_XY[0]) < DISH_IN[0] / 2 + 0.005
               and abs(float(mid[1]) - DISH_XY[1]) < DISH_IN[1] / 2 + 0.005 and mid[2] < TOP + 0.045)
    off_counter = mid[2] < TOP - 0.02
    return in_basin, in_dish, off_counter


def rule(sealed, in_basin, in_dish, off_counter, clear, still):
    """The score from the findings (see the module docstring)."""
    if sealed:
        return 0.0
    if in_dish:
        return 1.0 if clear and still else 0.75
    if in_basin or off_counter:
        return 0.25
    return 0.5


def _update(environment):
    base = environment.unwrapped
    sc = base.scene
    ep = getattr(base, "episode_length_buf", None)
    if ep is not None:
        ep = int(_np(ep)[0])
        if ep < _S.get("ep_len", -1):                      # a reset happened: start over
            _reset()
        _S["ep_len"] = ep
    _S["step"] += 1
    n = _S["step"]
    p = _pose(sc, "stopper")
    r = _rot(p[3:7])
    pos = p[:3]
    mid = pos + r @ MID
    lift, off = seal_findings(pos, r)
    if _S["sealed"]:
        _S["peak_lift"] = max(_S["peak_lift"], lift)
        _S["peak_off"] = max(_S["peak_off"], off)
        if n > 4 and seal_breaks(lift, off):              # the first steps settle the scene
            _S["sealed"] = False
            _S["broke"] = n
            _S["how"] = "lifted" if lift > RIM_BREAK else "slid"
            print(f"[sink] step {n}: seal broken ({_S['how']}: rim {lift * 1000:.1f} mm up, centre {off * 1000:.1f} mm "
                  f"off the drain, {n / 20:.1f} s)", flush=True)
    _force(sc, "stopper", (0.0, 0.0, -F_SEAL) if _S["sealed"] else None)
    tips = _tips(sc)
    near = min((float(np.linalg.norm(t - mid)) for t in tips.values()), default=1.0)
    hist = _S["hist"]
    hist.append(mid.copy())
    span = round(STILL_S * 20)
    del hist[:-(span + 1)]
    still = len(hist) > span and all(np.linalg.norm(h - mid) < STILL_MM / 1000 for h in hist[:-1])
    in_basin, in_dish, off_counter = where(mid)
    clear = near >= CLEAR
    score = rule(_S["sealed"], in_basin, in_dish, off_counter, clear, still)
    _S["hold"] = _S["hold"] + 1 if score >= 1.0 else 0
    why = (f"sealed={int(_S['sealed'])} basin={int(in_basin)} dish={int(in_dish)} off={int(off_counter)} "
           f"near={near * 100:.1f}cm still={int(still)}")
    if score != _S["score"]:
        print(f"[sink] step {n}: score {score} ({why})", flush=True)
    if not np.all(np.isfinite(p)) and not _S.get("nan"):
        _S["nan"] = 1
        print(f"[sink] first NaN stopper pose at step {n}", flush=True)
    _S["score"], _S["why"] = score, why
    return pos, mid, lift, off, tips, why


@on_step
def _log(environment):
    if "sealed" not in _S:
        _reset()
    try:
        pos, mid, lift, off, tips, why = _update(environment)
    except Exception as e:  # noqa: BLE001
        import traceback
        print(f"[sink] update error {e!r}\n{traceback.format_exc()}", flush=True)
        raise
    n = _S["step"]
    if not DEBUG or (n % 4 and n > 4):
        return
    tc = " ".join(f"{k[:5]}=({v[0]:.3f},{v[1]:.3f},{v[2]:.3f})" for k, v in tips.items())
    print(f"[sink] step {n} stopper=({pos[0]:.4f},{pos[1]:.4f},{(pos[2] - FLOOR_Z) * 1000:+.1f}mm) "
          f"mid_z={(mid[2] - TOP) * 100:.1f}cm rim={lift * 1000:.1f}mm off={off * 1000:.1f}mm | {why} "
          f"score={_S['score']} {tc}", flush=True)


def score(objects):
    return float(_S.get("score", 0.0))


def success(objects):
    return _S.get("hold", 0) >= round(SUCCESS_HOLD_S * 20)


# --- scripted demo --------------------------------------------------------------------------------------------------
KNOB_TOP = np.array([XC, YC, FLOOR_Z + STOPPER_H])
HI = TOP + 0.20                                         # carry height: the stopper's underside 17 cm up, over the rim
COUNTER_XY = (0.36, -0.30)                              # 'counter' variant: on the counter beside the soap dish
DROP_GAP = 0.014                                        # let go with the underside 14 mm above the dish floor


def _plan(steps, start_t=0.0):
    """[(t_end, pos, finger, label, grip)] -> waypoints (durations)."""
    out, t = [], start_t
    for t_end, p, f, label, g in steps:
        assert t_end > t + 1e-9, (label, t_end, t)
        out.append(Waypoint(float(t_end - t), tuple(float(v) for v in p), float(f), label, float(g)))
        t = t_end
    return out


def plans(variant="full"):
    """Timed plans for both arms: {arm: [(t_end, pos, finger, label, grip)]}, the start TCPs and the key points.

    Right arm, the grip leaned PULL_TILT back toward the arms so the wrist stays clear of the spout: come in over the
    front of the basin, slide down the leaned axis onto the knob, close, draw back up that axis PULL past the grasp
    over 3.5 s (the arm's pull builds until the seal lets go), lift clear of the rim, carry to the soap dish, lower,
    let go and go back. Left arm: waits.
    Variants: nothing; weak_pull (draws back only 1 cm, about 4 N, then lets go); drop_back (unseals, lifts 6 cm and
    drops it back in the basin); counter (puts it on the counter beside the soap dish); keep_holding (lowers it into
    the dish but never lets go); straight (the same, with a straight-down grip: a probe of the spout's hindrance)."""
    from yam_arena import station
    start = {a: ik.tcp_pose(np.array(READY, float), np.asarray(station.BASES[a], float))[0] for a in ("left", "right")}
    rot = tilted(0.0 if variant == "straight" else PULL_TILT)
    g = TILT if variant != "straight" else 0.0
    a = rot[:, 2]                                       # the approach axis, wrist -> fingertips
    G = KNOB_TOP - np.array([0, 0, GRASP_DEPTH])        # TCP on the knob's axis, GRASP_DEPTH below its top
    pull = 0.01 if variant == "weak_pull" else PULL
    P = G - a * pull
    lift_pt = np.array([G[0] - 0.05, G[1], HI])
    drop_xy = COUNTER_XY if variant == "counter" else DISH_XY
    lo = TOP + DROP_GAP + STOPPER_H - GRASP_DEPTH + (0.0 if variant == "counter" else DISH_FLOOR)
    R_ = [(2.5, G - a * 0.16, FINGER_OPEN, "over the knob", g),
          (5.0, G, FINGER_OPEN, "down onto the knob", g),
          (6.5, G, FINGER_CLOSED, "close on the knob", g),
          (10.0, P, FINGER_CLOSED, "draw back (pull)", g)]
    if variant == "nothing":
        R_ = [(1.0, start["right"], FINGER_OPEN, "wait", START)]
    elif variant == "weak_pull":
        R_ += [(11.0, P, FINGER_OPEN, "let go", g), (13.0, P - a * 0.08, FINGER_OPEN, "back off", g),
               (16.0, start["right"], FINGER_OPEN, "back to start", START)]
    elif variant == "drop_back":
        R_ += [(12.0, G + np.array([0, 0, 0.06]), FINGER_CLOSED, "lift 6 cm", g),
               (13.0, G + np.array([0, 0, 0.06]), FINGER_OPEN, "drop", g),
               (15.0, lift_pt, FINGER_OPEN, "up", g),
               (18.0, start["right"], FINGER_OPEN, "back to start", START)]
    else:
        R_ += [(12.0, lift_pt, FINGER_CLOSED, "lift over the rim", g),
               (15.0, (*drop_xy, HI), FINGER_CLOSED, "carry", g),
               (17.0, (*drop_xy, lo), FINGER_CLOSED, "lower", g)]
        if variant == "keep_holding":
            R_ += [(23.0, (*drop_xy, lo), FINGER_CLOSED, "keep holding", g)]
        else:
            R_ += [(18.0, (*drop_xy, lo), FINGER_OPEN, "let go", g),
                   (19.5, (*drop_xy, TOP + 0.12), FINGER_OPEN, "up", g),
                   (22.0, start["right"], FINGER_OPEN, "back to start", START)]
    L_ = [(1.0, start["left"], FINGER_OPEN, "wait", START)]
    end = R_[-1][0] + STILL_S + SUCCESS_HOLD_S + 3.0
    L_.append((end, L_[-1][1], L_[-1][2], "rest", L_[-1][4]))
    R_.append((end, R_[-1][1], R_[-1][2], "rest", R_[-1][4]))
    return {"left": L_, "right": R_}, start, {"G": G, "P": P, "lift": lift_pt, "lo": lo, "end": end, "rot": rot}


def demo():
    variant = os.environ.get("DEMO_VARIANT", "full")
    timed, _, _ = plans(variant)
    return {arm: _plan(steps) for arm, steps in timed.items()}
