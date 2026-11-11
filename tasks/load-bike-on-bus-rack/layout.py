"""Where everything stands, in table coordinates (metres): x away from the arms, y toward the left arm, z above the
station's worktop. The arm bases stand at x 0, y +-0.305. The scene is at 1:6 scale. Pure constants."""

TOP = 0.7475  # the station's worktop (world z)

# The bike starts upright in a pavement wheel rail, front wheel toward +y. BIKE0 is the bike frame's origin: on the
# ground, midway between the axles.
BIKE0 = (0.14, -0.02)
RAIL = dict(x=0.14, y0=-0.185, y1=0.145, gap=0.0025, wall=0.004, h=0.015)  # gap: free space each side of the tyre

# The rack, lowered, in front of the bus. Its body origin (rack frame) sits here.
RACK_ORIGIN = (0.30, 0.0, 0.0005)
TRAYS = {"outer": 0.24, "inner": 0.31}  # each position's tray centre line (x); "outer" is farthest from the bus
TRAY = dict(y0=-0.19, y1=0.15, half_in=0.021, wall=0.004, h=0.015)  # U-channels standing on the table
PIVOT_X, PIVOT_Z = 0.38, 0.035  # both support arms' hinges, at the rack's bus end
ARM_Y = {"outer": 0.0895, "inner": -0.1295}  # each support arm's swing plane
HOOK_END = {"outer": +1, "inner": -1}  # which end of each position its hook is at (+1: +y); front wheel goes there
FRONT_OFFSET = 0.022  # the hook rests this far beyond the front axle (toward the wheel's leading edge) when the bike is centred
SPINE = dict(x=0.38, half_x=0.006, y0=-0.20, y1=0.17, h=0.022)

# The bus front: a black bumper, then the white front panel with a dark windscreen band.
BUMPER = dict(x0=0.42, x1=0.55, half_y=0.27, h=0.05)
BUS = dict(x0=0.55, x1=0.57, half_y=0.29, h=0.32)
