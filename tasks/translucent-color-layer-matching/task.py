"""Draft yam_arena lighting-filter task; see README for the unverified mechanics.

The demo and scorer use fixed world positions. No success() is supplied: a
subsequent movement of the reference must still be able to spoil the trial.
"""

from dataclasses import replace
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from filter_scoring import (FILTER_NAMES, LIGHTBOX_H, PLAQUE_SIZE, REFERENCE,
                            STACK_XY, Trial)  # noqa: E402
from optics import FILTER_TINTS, REFERENCE_TINT  # noqa: E402
from partkit import on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
INSTRUCTION = ("Stack the translucent plaques in the outlined area to match the color of the reference plaque. "
               "Do not move the reference plaque.")

REF_XY = (0.300, -0.145)
STARTS = {"amber": (0.150, 0.280), "cyan": (0.330, 0.280),
          "violet": (0.150, -0.330), "lime": (0.330, -0.330)}
FRAME = (0.20, 0.20, 0.22)
WHITE = (0.94, 0.94, 0.94)
OUTLINE = (0.08, 0.08, 0.08)
SHEET_THICKNESS = 0.004
RAIL_WIDTH = 0.008
FRAME_H = PLAQUE_SIZE[2] - SHEET_THICKNESS
MASS = 0.050


def _plaque(name, xy, tint, bottom):
    # Sheet covers the full top surface; four rails support it underneath and
    # provide 16-mm-high gripping faces. All five boxes form one rigid body.
    sheet_z = PLAQUE_SIZE[2] / 2 - SHEET_THICKNESS / 2
    rail_z = -PLAQUE_SIZE[2] / 2 + FRAME_H / 2
    offset = (PLAQUE_SIZE[0] - RAIL_WIDTH) / 2
    boxes = [((0.0, 0.0, sheet_z), (0.060, 0.060, SHEET_THICKNESS), tint),
             ((0.0, offset, rail_z), (0.060, RAIL_WIDTH, FRAME_H), FRAME),
             ((0.0, -offset, rail_z), (0.060, RAIL_WIDTH, FRAME_H), FRAME),
             ((offset, 0.0, rail_z), (RAIL_WIDTH, 0.044, FRAME_H), FRAME),
             ((-offset, 0.0, rail_z), (RAIL_WIDTH, 0.044, FRAME_H), FRAME)]
    p = part(name, pos=(xy[0], xy[1], bottom + PLAQUE_SIZE[2] / 2 + 0.001),
             boxes=boxes, mass=MASS, color=FRAME, friction=1.0)
    # The local copy of partkit binds OmniGlass only to box 0.
    return replace(p, meta={"filter_tint": tint})


def _outline():
    # A 60 mm outer square: the exact nominal footprint of one plaque. The
    # 0.2 mm raised marks can support the rails; the scorer allows that height.
    x, y = STACK_XY
    z = TOP + LIGHTBOX_H + 0.0001
    return [Box("outline_n", pos=(x, y + 0.029, z), size=(0.060, 0.002, 0.0002), color=OUTLINE),
            Box("outline_s", pos=(x, y - 0.029, z), size=(0.060, 0.002, 0.0002), color=OUTLINE),
            Box("outline_e", pos=(x + 0.029, y, z), size=(0.002, 0.056, 0.0002), color=OUTLINE),
            Box("outline_w", pos=(x - 0.029, y, z), size=(0.002, 0.056, 0.0002), color=OUTLINE)]


OBJECTS = [
    Box("stack_lightbox", pos=(*STACK_XY, TOP + LIGHTBOX_H / 2), size=(0.120, 0.120, LIGHTBOX_H), color=WHITE),
    Box("reference_lightbox", pos=(*REF_XY, TOP + LIGHTBOX_H / 2), size=(0.120, 0.120, LIGHTBOX_H), color=WHITE),
    *_outline(),
    *[_plaque(name, STARTS[name], FILTER_TINTS[name], TOP) for name in FILTER_NAMES],
    _plaque(REFERENCE, REF_XY, REFERENCE_TINT, TOP + LIGHTBOX_H),
]

_TRIAL = Trial(TOP)


def _states(environment):
    from yam_arena.scene import object_states
    return object_states(environment, [*FILTER_NAMES, REFERENCE])


def _install_white_lights(environment):
    """Author identical, upward-facing USD rectangle lights at the two box tops.

    The plane is 0.3 mm above the white support so it is not hidden by its
    opaque collider. The renderer and MDL behaviour have not been verified.
    """
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdGeom, UsdLux

    stage = get_current_stage()
    origin = environment.unwrapped.scene.env_origins[0]
    origin = getattr(origin, "torch", origin)
    if hasattr(origin, "detach"):
        origin = origin.detach().cpu().numpy()
    root = "/World/TranslucentFilterLights"
    UsdGeom.Xform.Define(stage, root)
    for name, (x, y) in (("stack", STACK_XY), ("reference", REF_XY)):
        path = f"{root}/{name}"
        if stage.GetPrimAtPath(path):
            continue
        light = UsdLux.RectLight.Define(stage, path)
        light.CreateWidthAttr(0.044)
        light.CreateHeightAttr(0.044)
        api = UsdLux.LightAPI(light.GetPrim())
        api.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        api.CreateIntensityAttr(1000.0)
        xf = UsdGeom.Xformable(light.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(float(origin[0]) + x, float(origin[1]) + y,
                                         float(origin[2]) + TOP + LIGHTBOX_H + 0.0003))
        xf.AddRotateXOp().Set(180.0)  # USD area lights point along local -Z.


@on_reset
def _reset(environment=None):
    if environment is None:
        _TRIAL.reset(None)
        return
    _install_white_lights(environment)
    _TRIAL.reset(_states(environment)[REFERENCE])


@on_step
def _step(environment=None):
    if environment is not None:
        _TRIAL.observe(_states(environment))


def score(objects):
    return _TRIAL.score(objects)


def _pick_place(name, level):
    """Open-loop, top-down pick at the frame and place at the given level."""
    x, y = STARTS[name]
    sx, sy = STACK_XY
    # At this TCP height the pad tips are approximately 8 mm above the bottom.
    pick_z = TOP + 0.016
    place_z = TOP + LIGHTBOX_H + level * PLAQUE_SIZE[2] + 0.016
    travel_z = TOP + 0.160
    grip = -(PLAQUE_SIZE[1] - 0.008) / 2
    return [Waypoint(1.5, (x, y, travel_z), FINGER_OPEN, "above plaque", 0.0),
            Waypoint(1.5, (x, y, pick_z), FINGER_OPEN, "descend", 0.0),
            Waypoint(0.6, (x, y, pick_z), FINGER_OPEN, "settle", 0.0),
            Waypoint(1.5, (x, y, pick_z), grip, "grip frame", 0.0),
            Waypoint(1.5, (x, y, travel_z), grip, "lift", 0.0),
            Waypoint(2.0, (sx, sy, travel_z), grip, "carry", 0.0),
            Waypoint(1.0, (sx, sy, place_z + 0.025), grip, "above stack", 0.0),
            Waypoint(2.5, (sx, sy, place_z), grip, "lower onto support", 0.0),
            Waypoint(0.8, (sx, sy, place_z), grip, "settle", 0.0),
            Waypoint(1.5, (sx, sy, place_z), FINGER_OPEN, "release", 0.0),
            Waypoint(1.5, (sx, sy, travel_z), FINGER_OPEN, "withdraw", 0.0)]


def demo():
    # One arbitrary order. This is a proposed trajectory, not a verified demo.
    left = _pick_place("amber", 0) + _pick_place("cyan", 1)
    left += [Waypoint(1.5, (0.12, 0.42, TOP + 0.20), FINGER_OPEN, "park", 0.0)]
    return {"left": left}
