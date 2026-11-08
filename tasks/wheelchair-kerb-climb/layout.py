"""Where everything stands, in the station's world frame (metres): x away from the arms, y toward the left arm,
z up; the arm bases stand at x 0, y +-0.305. Pure constants."""

TOP = 0.7475  # the station's worktop
AXLE0 = (0.23, 0.0)  # the chair's rear-axle middle at the start; the chair faces +x, toward the kerb
KERB_X0, KERB_H = 0.37, 0.015  # the kerb's face and height
KERB_TOP = TOP + KERB_H
KERB = dict(x1=0.60, half_y=0.20)
ROAD = dict(x0=0.08, half_y=0.20)  # dark road surface from here to the kerb face, a paint layer
DUMMY = dict(x=0.045, w=0.06, d=0.08, h=0.06, mass=0.12)  # on the seat: centre x in the chair frame, size, kg
