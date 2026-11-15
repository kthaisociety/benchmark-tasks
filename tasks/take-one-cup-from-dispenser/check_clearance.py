"""Offline clearance check for the scripted demo: plays its waypoints through yam_arena's ArmPlan (the same IK
yam_scripted uses), poses the YAM's visual meshes (link4, link5, gripper, both fingers) with URDF forward kinematics,
and reports the closest approach of any mesh vertex to the dispenser (tube and lip, as one cylinder), the wall, the
counter and mat, and the hanging cup. No Isaac Sim. CUP_PITCH_DEG, CUP_GRIP_UP and CUP_GRIP_FORWARD try other grips.

    PYTHONPATH=<yam-isaac-lab-arena> python check_clearance.py [variant]
"""
import os, sys, struct, math
from pathlib import Path
import numpy as np

task_dir = Path(__file__).resolve().parent
variant = sys.argv[1] if len(sys.argv) > 1 else "full"
os.environ["DEMO_VARIANT"] = variant
sys.path.insert(0, str(task_dir))
import task as T  # noqa: E402
from yam_arena import ASSET_DIR, station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.urdf import forward_kinematics, rpy_matrix  # noqa: E402
from yam_arena.constants import FINGER_OPEN, ARM_JOINTS  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402

URDF = ASSET_DIR / "yam.urdf"
root = ET.parse(URDF).getroot()
LINKS = ("link4", "link5", "gripper", "tip_left", "tip_right")


def read_stl(path):
    data = Path(path).read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        pts = [list(map(float, l.split()[1:4])) for l in data.decode(errors="ignore").splitlines() if l.strip().startswith("vertex")]
        return np.array(pts)
    n = struct.unpack("<I", data[80:84])[0]
    arr = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return arr["v"].reshape(-1, 3).astype(float)


meshes = {}
for link in root.iter("link"):
    name = link.get("name")
    if name not in LINKS:
        continue
    vis = link.find("visual")
    org = vis.find("origin")
    xyz = np.array([float(v) for v in org.get("xyz").split()])
    rpy = np.array([float(v) for v in org.get("rpy").split()])
    fn = vis.find("geometry/mesh").get("filename")
    v = np.unique(np.round(read_stl(URDF.parent / fn), 5), axis=0)
    meshes[name] = (rpy_matrix(rpy) @ v.T).T + xyz

base = np.array(station.BASES["left"])
plan = T.demo()["left"]
arm = ArmPlan(plan, base)
cup = T.CUPS[0]
ax, ay = T.AXIS_XY


def dispenser_clearance(p):
    """Distance from points to the dispenser's solid (tube + lip as one cylinder from its bottom edge up), negative inside."""
    r = np.hypot(p[:, 0] - ax, p[:, 1] - ay)
    dr = r - T.TUBE_OUT
    dz = T.DISPENSER_BOTTOM - p[:, 2]
    outside = np.where((dr > 0) & (dz > 0), np.hypot(dr, dz), np.maximum(dr, dz))
    return outside


def cup_clearance(p, base_z):
    """Distance from points to the hanging cup's outside (a frustum), negative inside (rough: radial at each height)."""
    z = p[:, 2] - base_z
    zc = np.clip(z, 0, T.CUP_H)
    r = np.hypot(p[:, 0] - ax, p[:, 1] - ay)
    rad = T.R0 + T.SLOPE * zc
    dr = r - rad
    dz = np.where(z < 0, -z, np.where(z > T.CUP_H, z - T.CUP_H, 0.0))
    return np.where(dz > 0, np.hypot(np.maximum(dr, 0), dz), dr)


HANGING = ("in front of the cup, tilted", "down to the pinch height", "fingers either side of the cup")
# Both fingers close (seen on Brev): open, each pad sits 47 mm from the TCP; closed on the cup, each stops on its side,
# R_GRIP from the TCP.
OPEN = FINGER_OPEN
CLOSED_ON_CUP = -T.R_CONTACT  # where the tilted pads' upper edge meets the cup
worst = {}


def note(key, d, label, link):
    if key not in worst or d < worst[key][0]:
        worst[key] = (d, label, link)


t, dt = 0.0, 0.05
while t < arm.duration:
    a7, label = arm.action7(t)
    q = {n: float(v) for n, v in zip(ARM_JOINTS, a7[:6])}
    # the commanded opening, but a pad stops on the cup's side once it is between them (after the pinch)
    q["joint7"] = q["joint8"] = min(float(a7[6]) * FINGER_OPEN, CLOSED_ON_CUP)
    fk = forward_kinematics(URDF, q)
    for link, v in meshes.items():
        T4 = fk[link]
        P = (T4[:3, :3] @ v.T).T + T4[:3, 3] + base
        note("dispenser", dispenser_clearance(P).min(), label, link)
        note("wall", (T.WALL_X - P[:, 0]).min(), label, link)
        over_mat = (np.abs(P[:, 0] - T.MAT_XY[0]) <= T.MAT_SIZE[0] / 2) & (np.abs(P[:, 1] - T.MAT_XY[1]) <= T.MAT_SIZE[1] / 2)
        note("counter", (P[:, 2] - np.where(over_mat, T.MAT_TOP, T.TOP)).min(), label, link)
        if label in HANGING:
            note("cup (approach)", cup_clearance(P, T.CUP1_HANG_Z).min(), label, link)
        if label == "pinch":
            # both pads press on the cup: up to 1.5 mm of mesh overlap at a finger is the intended contact
            key = "pads on the cup (contact)" if link in ("tip_left", "tip_right") else "cup (pinched, other links)"
            note(key, cup_clearance(P, T.CUP1_HANG_Z).min(), label, link)
    t += dt
print(f"variant {variant}, pitch {math.degrees(T.PITCH):.0f} deg, grip_forward {T.GRIP_FORWARD * 1000:.0f} mm, "
      f"grip_up {T.GRIP_UP * 1000:.0f} mm: "
      f"{arm.duration:.1f} s; worst IK {arm.worst_ik_mm:.2f} mm {arm.worst_ik_deg:.2f} deg")
for k, (d, label, link) in worst.items():
    if "(contact)" in k:  # a pad pressing on the cup: up to 1.5 mm of mesh overlap is the intended contact
        flag = "OK " if d > -0.0015 else "HIT"
    else:
        flag = "OK " if d > 0.003 else "LOW" if d > 0 else "HIT"
    print(f"  {flag} {k:24s} {d * 1000:7.1f} mm  ({link}, during: {label})")
