"""Every size and position, in metres. Pure constants (no simulator).

World frame (the station's): x away from the arms, y toward the left arm, z up. The arm bases stand at x 0,
y +-0.305. The bike's front is +y, marked by the white reflector on the fork crown.

Wheel frame: origin at the axle's middle, the axle along x, the wheel's plane y-z. The quick-release lever is on the
-x end (facing the robot), and its adjusting nut on the +x end.
Fork frame: origin on the table top under the middle between the two dropouts, axes as the world's.
"""

import math

TOP = 0.7475  # station table top (yam_arena.task_file.TABLE_TOP["station"])
TIP = 0.013  # yam_arena PAD_TIP_PAST_TCP: the closed pad tips are this far below the TCP
FX_HALF = 0.008  # a closed finger's half-thickness along x with the default grip yaw (UNVERIFIED: a guess; tune on Brev)

# -- the wheel: a 20-inch folding-bike wheel at 1:4 -----------------------------------------------------------------
R_OUT = 0.064  # tyre outside radius (a 20 x 1.75 tyre is about 0.51 m across)
R_RIM = 0.053  # tyre inside / rim outside
R_IN = 0.047  # rim inside
TYRE_W, RIM_W = 0.014, 0.010
SEGMENTS = 24  # the ring is made of flat segments; one is centred on top, one at the bottom
HUB_R = 0.010
LOCKNUT_X = 0.015  # the hub's locknut faces, either side of the middle (25 mm at 1:4 of a 100 mm hub)
AXLE_HALF = 0.058  # the axle ends
AXLE_SIDE = 0.010  # square axle section (a real axle has flats where it sits in the dropouts)
CAP_R = 0.009  # cam housing (lever end) and adjusting nut (far end), on the axle beyond its ends
CAP_X = (0.058, 0.066)
PIVOT_X = -0.062  # the lever's cam pivot, on the axle's line
LEVER = dict(arm=(0.006, 0.035, 0.008), paddle=(0.035, 0.058, 0.022), thick=0.006)  # from/to radius, width (m)
PADDLE_R = 0.0465  # middle of the paddle, from the pivot
MASS = dict(base=0.05, spin=0.22, aim=0.006, cam=0.006)  # kg: axle and nut, rim+tyre+spokes+hub shell, skewer, lever

# -- the quick-release lever's mechanism ------------------------------------------------------------------------------
# Closure c (deg): 0 = fully open (the lever points straight out along the axle), 90 = closed (it lies along the
# fork). Aim a (deg): the skewer's turn about the axle; 0 = the closed lever points down the blade toward the crown,
# + toward the bike's front. The cam joint's angle is -c (radians) and the aim joint's is +a.
OPEN_DETENT = 15.0  # below this the lever springs back fully open
CLOSE_DETENT = 60.0  # past this the cam goes over centre and the lever snaps closed
SLIP = 6.0  # in between, the lever stays where it is left (friction): its spring's rest point trails it by this much
AIM_LIMIT = 90.0
CAM = dict(spring=0.08, damping=0.0015)  # N m/rad, N m s/rad
AIM = dict(spring=0.15, damping=0.002)

# -- the fork: fixed upside down in a workshop stand, dropouts up -----------------------------------------------------
FORK_XY = (0.34, -0.12)
H_SEAT = R_OUT + 0.020  # axle centre when fully seated, above the table (the tyre clears the crown by 6 mm)
SLOT_END = H_SEAT - AXLE_SIDE / 2  # the slot's closed end (the "top of the dropout" on an upright bike)
SLOT_HALF = 0.009  # slot half-width along y (1.8 cm slot, 1.0 cm axle)
SLOT_STRAIGHT = 0.006
FLARE_H, FLARE_DEG = 0.012, 30.0  # the mouth's walls lean out 30 deg from vertical
MOUTH_HALF = SLOT_HALF + FLARE_H * math.tan(math.radians(FLARE_DEG))
MOUTH_TOP = SLOT_END + SLOT_STRAIGHT + FLARE_H
PLATE_X = 0.034  # dropout plate centres either side of the middle (inner faces at 0.030: 15 mm play per side)
PLATE_T = 0.008
JAW_OUT = 0.020  # each jaw reaches this far either side of the slot centre
PLATE_DEPTH = 0.016  # plate below the slot end
CROWN = dict(half_x=0.040, half_y=0.010, z0=0.006, z1=0.014)
BASE = dict(x0=-0.045, x1=0.060, half_y=0.080, h=0.006)
FORK_FRICTION = 0.3  # painted steel; combine "min", so the axle slides down the mouth

# -- the wheel stand: two rubber blocks gripping the tyre's bottom ---------------------------------------------------
STAND_XY = (FORK_XY[0], FORK_XY[1] - 0.17)
STAND_BLOCK = (0.012, 0.050, 0.020)
STAND_GAP = 0.003  # each side of the tyre
WHEEL_START = (STAND_XY[0], STAND_XY[1], TOP + R_OUT + 0.001)  # axle centre at the start
