"""Theatre lighting-gel matching for yam_arena.

Four colour-correction gels sit in frames beside two lightboxes. One lightbox
carries a sealed reference gel; the other is empty and outlined. The robot has
to work out which gels combined transmit the reference colour and stack those,
without disturbing the reference.

The renderer does not composite stacked transmissive sheets (the topmost one
wins), so the glazing is clear and each lightbox window is lit with the colour
computed from the gels above it. See README.md.
"""

from dataclasses import replace
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from filter_scoring import (FILTER_NAMES, LIGHTBOX_H, PLAQUE_SIZE, REFERENCE,  # noqa: E402
                            REFERENCE_PARTS, STACK_XY, Trial, display_stack)
from optics import FILTER_TINTS, REFERENCE_TINT, nominal_tint  # noqa: E402
from partkit import on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
# No success(): a later action must still be able to spoil the result.
INSTRUCTION = ("Stack gels in the outlined area until its light matches the colour of the reference light. "
               "Do not move the reference.")

REF_XY = (0.300, -0.145)
STARTS = {"amber": (0.150, 0.280), "cyan": (0.330, 0.280),
          "violet": (0.150, -0.330), "lime": (0.330, -0.330)}
WHITE = (0.94, 0.94, 0.94)
OUTLINE = (0.08, 0.08, 0.08)
REF_FRAME = (0.22, 0.22, 0.24)
CLEAR = (1.0, 1.0, 1.0)
SHEET_THICKNESS = 0.004
RAIL_WIDTH = 0.008
FRAME_H = PLAQUE_SIZE[2] - SHEET_THICKNESS
MASS = 0.050
EMISSIVE = 1.2  # window brightness; chosen so no channel clips at 320x176


def _plaque(name, xy, frame_colour, bottom):
    """Clear glazing on four rails; the rails carry the gel's identifying colour.

    All five boxes form one rigid body. The 16 mm rails are the gripping faces.
    """
    sheet_z = PLAQUE_SIZE[2] / 2 - SHEET_THICKNESS / 2
    rail_z = -PLAQUE_SIZE[2] / 2 + FRAME_H / 2
    offset = (PLAQUE_SIZE[0] - RAIL_WIDTH) / 2
    boxes = [((0.0, 0.0, sheet_z), (0.060, 0.060, SHEET_THICKNESS), CLEAR),
             ((0.0, offset, rail_z), (0.060, RAIL_WIDTH, FRAME_H), frame_colour),
             ((0.0, -offset, rail_z), (0.060, RAIL_WIDTH, FRAME_H), frame_colour),
             ((offset, 0.0, rail_z), (RAIL_WIDTH, 0.044, FRAME_H), frame_colour),
             ((-offset, 0.0, rail_z), (RAIL_WIDTH, 0.044, FRAME_H), frame_colour)]
    p = part(name, pos=(xy[0], xy[1], bottom + PLAQUE_SIZE[2] / 2 + 0.001),
             boxes=boxes, mass=MASS, color=frame_colour, friction=1.0)
    return replace(p, meta={"filter_tint": CLEAR})


def _outline():
    """A 60 mm outer square marking the nominal footprint of one gel frame."""
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
    # Two clear plaques in a neutral holder: the reference is seen through the
    # same two glazings as a correct two-gel stack, so equal colours render equal.
    _plaque(REFERENCE_PARTS[0], REF_XY, REF_FRAME, TOP + LIGHTBOX_H),
    _plaque(REFERENCE_PARTS[1], REF_XY, REF_FRAME, TOP + LIGHTBOX_H + PLAQUE_SIZE[2]),
]

_TRIAL = Trial(TOP)
_SHADERS = {}


def _states(environment):
    from yam_arena.scene import object_states
    return object_states(environment, [*FILTER_NAMES, *REFERENCE_PARTS])


def _window(stage, path, xy, z):
    """An emissive quad filling the lightbox window.

    Reuses the prims if they are already on the stage: reset runs again for
    every trial, and re-authoring a transform op raises.
    """
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    existing = stage.GetPrimAtPath(path + "/Mat/Surface")
    if existing:
        return UsdShade.Shader(existing)

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(Gf.Vec3d(xy[0], xy[1], z))
    cube.AddScaleOp().Set(Gf.Vec3f(0.044, 0.044, 0.0004))
    material = UsdShade.Material.Define(stage, path + "/Mat")
    shader = UsdShade.Shader.Define(stage, path + "/Mat/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 1.0, 1.0))
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.0, 0.0))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material)
    return shader


def _set_window(key, tint):
    shader = _SHADERS.get(key)
    if shader is None:
        return
    from pxr import Gf

    shader.GetInput("emissiveColor").Set(Gf.Vec3f(*(min(1.0, c) * EMISSIVE for c in tint)))


def _install_windows(environment):
    """Author the two emissive window quads; kept separate so it can be stubbed."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import UsdGeom

    stage = get_current_stage()
    origin = environment.unwrapped.scene.env_origins[0]
    if hasattr(origin, "detach"):
        origin = origin.detach().cpu().numpy()
    root = "/World/GelWindows"
    UsdGeom.Xform.Define(stage, root)
    _SHADERS.clear()
    for key, (x, y) in (("stack", STACK_XY), ("reference", REF_XY)):
        _SHADERS[key] = _window(stage, f"{root}/{key}",
                                (float(origin[0]) + x, float(origin[1]) + y),
                                float(origin[2]) + TOP + LIGHTBOX_H + 0.0003)


@on_reset
def _reset(environment=None):
    if environment is None:
        _TRIAL.reset(None)
        return
    _install_windows(environment)
    states = _states(environment)
    _set_window("reference", REFERENCE_TINT)
    # Unlit (white) for the starting layout, which never has a gel on the
    # stack box. Reading the poses here can still see the previous trial's
    # stack, which left a stale colour in that trial's first frame.
    _set_window("stack", nominal_tint(()))
    _TRIAL.reset({name: states[name] for name in REFERENCE_PARTS})


@on_step
def _step(environment=None):
    if environment is None:
        return
    states = _states(environment)
    _TRIAL.observe(states)
    _set_window("stack", nominal_tint(display_stack(states)))
    _set_window("reference", REFERENCE_TINT)


def score(objects):
    return _TRIAL.score(objects)


def _pick_place(name, level):
    """Open-loop, top-down pick at the frame rails and place at the given level."""
    x, y = STARTS[name]
    sx, sy = STACK_XY
    # At this TCP height the pad tips sit about 8 mm above the plaque bottom,
    # gripping the upper half of the 16 mm rails.
    pick_z = TOP + 0.016
    place_z = TOP + LIGHTBOX_H + level * PLAQUE_SIZE[2] + 0.016
    travel_z = TOP + 0.160
    grip = -(PLAQUE_SIZE[1] - 0.008) / 2
    return [Waypoint(1.5, (x, y, travel_z), FINGER_OPEN, "above gel", 0.0),
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
    left = _pick_place("amber", 0) + _pick_place("cyan", 1)
    left += [Waypoint(1.5, (0.12, 0.42, TOP + 0.20), FINGER_OPEN, "park clear of the camera", 0.0)]
    return {"left": left}
