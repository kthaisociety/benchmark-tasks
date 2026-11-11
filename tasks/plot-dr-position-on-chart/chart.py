"""The chart: a 1:50,000 Mercator chart of open sea between 49°57'N and 50°04'N, 3°00'W and 2°44'W, printed on an
A3-sized sheet that lies taped flat on the table, north away from the robot. Pure Python (math and numpy), so the
scorer and its tests can use it without the simulator; task.py turns DECALS into the printed marks.

World frame: metres from the midpoint between the arm bases, x forward (north on the chart), y toward the left arm
(west on the chart), z up.

The projection is a spherical Mercator, true to scale along 50°N, where a minute of longitude is 1852 cos 50° m and a
minute of latitude is one nautical mile (1852 m): on the chart a minute of longitude is 23.81 mm and a minute of
latitude is 37.04 mm at 50°N (36.97 mm at 49°57'N, 37.08 mm at 50°04'N).
"""

from __future__ import annotations

import math

NM = 1852.0  # metres; one minute of latitude
SCALE = 50_000
LAT_TRUE = 50.0  # degrees: the latitude where the chart is true to scale
LON_MIN = NM * math.cos(math.radians(LAT_TRUE)) / SCALE  # chart length of one minute of longitude (m)
_A = LON_MIN * 60 * 180 / math.pi  # chart length of one radian of longitude (m)

LAT_S, LAT_N = 49.0 + 57 / 60, 50.0 + 4 / 60  # the inner neatline (degrees, north positive)
LON_W, LON_E = -3.0, -3.0 + 16 / 60  # degrees, east positive: 3°00'W to 2°44'W

CHART_X = 0.285  # the world x of the middle of the charted area
PAPER_THICK = 0.001  # the sheet as simulated (a real chart is about 0.1 mm)
BORDER = 0.006  # the scale border between the inner and outer neatlines
MARGIN = 0.012  # plain paper outside the outer neatline


def merc(lat_deg: float) -> float:
    """Mercator northing on the chart (m) of a latitude."""
    return _A * math.log(math.tan(math.pi / 4 + math.radians(lat_deg) / 2))


_LON_MID = (LON_W + LON_E) / 2


def to_world(lat_deg: float, lon_deg: float) -> tuple[float, float]:
    """(x, y) on the table of a chart position."""
    x = CHART_X + merc(lat_deg) - (merc(LAT_S) + merc(LAT_N)) / 2
    y = -(lon_deg - _LON_MID) * 60 * LON_MIN  # east is -y
    return x, y


def to_chart(x: float, y: float) -> tuple[float, float]:
    """(lat, lon) in degrees of a point on the table."""
    northing = x - CHART_X + (merc(LAT_S) + merc(LAT_N)) / 2
    lat = math.degrees(2 * math.atan(math.exp(northing / _A)) - math.pi / 2)
    lon = _LON_MID - y / (60 * LON_MIN)
    return lat, lon


def lat_minute(lat_deg: float) -> float:
    """Chart length (m) of one minute of latitude, i.e. one nautical mile, at a latitude."""
    return LON_MIN / math.cos(math.radians(lat_deg))


# -- the charted area and the sheet --------------------------------------------------------------------------------
INNER_X = (to_world(LAT_S, 0)[0], to_world(LAT_N, 0)[0])
INNER_Y = (to_world(0, LON_E)[1], to_world(0, LON_W)[1])  # (east edge, west edge)
OUTER_X = (INNER_X[0] - BORDER, INNER_X[1] + BORDER)
OUTER_Y = (INNER_Y[0] - BORDER, INNER_Y[1] + BORDER)
PAPER_X = (OUTER_X[0] - MARGIN, OUTER_X[1] + MARGIN)
PAPER_Y = (OUTER_Y[0] - MARGIN, OUTER_Y[1] + MARGIN)
PAPER_SIZE = (PAPER_X[1] - PAPER_X[0], PAPER_Y[1] - PAPER_Y[0], PAPER_THICK)
PAPER_CENTRE = ((PAPER_X[0] + PAPER_X[1]) / 2, (PAPER_Y[0] + PAPER_Y[1]) / 2)

# -- the passage -----------------------------------------------------------------------------------------------------
# WHY: the answer must lie well away from every printed grid line, or "where the course crosses the grid" would score:
# from here the DR position (50°02.5'N) is 2.5' of latitude (3.5 nm along the line) from the 50°00'N parallel.
BUOY = (49 + 58.3 / 60, -(2 + 58.5 / 60))  # red can buoy: 49°58.3'N, 2°58.5'W
COURSE_DEG = 45.0  # true; a rhumb line is straight on a Mercator chart
RUN_NM = 6.0  # the distance run since the buoy, from the log
LINE_NM = 7.0  # the pencilled course line runs on 1 nm past the answer


def along_course(nm: float, start=BUOY, course_deg: float = COURSE_DEG) -> tuple[float, float]:
    """(lat, lon) after sailing ``nm`` nautical miles on a rhumb line (mid-latitude sailing, exact enough over 8 nm)."""
    lat0, lon0 = start
    dlat = nm * math.cos(math.radians(course_deg)) / 60
    dep = nm * math.sin(math.radians(course_deg))  # departure (nm east)
    mid = lat0 + dlat / 2
    return lat0 + dlat, lon0 + dep / (60 * math.cos(math.radians(mid)))


BUOY_XY = to_world(*BUOY)
DR = along_course(RUN_NM)
DR_XY = to_world(*DR)  # the answer
LINE_END_XY = to_world(*along_course(LINE_NM))


def course_point(chart_m: float) -> tuple[float, float]:
    """The point ``chart_m`` metres from the buoy along the pencilled line (as a ruler would measure on the paper)."""
    ux, uy = math.cos(math.radians(COURSE_DEG)), -math.sin(math.radians(COURSE_DEG))
    return BUOY_XY[0] + chart_m * ux, BUOY_XY[1] + chart_m * uy


# The classic mistake: six minutes taken off the longitude scale (top or bottom border) instead of the latitude scale.
LON_SCALE_XY = course_point(RUN_NM * LON_MIN)

# -- printing (decals in the sheet's frame) ------------------------------------------------------------------------
INK = (0.08, 0.08, 0.10)
PAPER = (0.95, 0.95, 0.91)
WATER_TINT = (0.84, 0.91, 0.95)  # the pale blue of the charted sea
PENCIL = (0.30, 0.30, 0.34)
BUOY_RED = (0.85, 0.10, 0.12)
INK_H = 0.0002  # printing stands this proud of the paper so it renders above it
TICK_W = 0.0004  # 0.1' ticks: as printed; too fine for the 320 x 176 cameras, the minute bands are not
LINE_W = 0.0015  # neatlines and the course line (a soft pencil line, drawn broad so the top camera shows it)


def _decal(x0, x1, y0, y1, color, layer=1, yaw=0.0):
    """A printed rectangle between world x0..x1, y0..y1 (or centred there when yawed), in the sheet frame."""
    cx, cy = (x0 + x1) / 2 - PAPER_CENTRE[0], (y0 + y1) / 2 - PAPER_CENTRE[1]
    z = PAPER_THICK / 2 + layer * INK_H - INK_H / 2
    return ((cx, cy, z), (abs(x1 - x0), abs(y1 - y0), INK_H), color, yaw)


def _frame(x0, x1, y0, y1, w, color, layer):
    return [_decal(x0, x1, y0, y0 + w, color, layer), _decal(x0, x1, y1 - w, y1, color, layer),
            _decal(x0, x0 + w, y0, y1, color, layer), _decal(x1 - w, x1, y0, y1, color, layer)]


def _grid(lo_deg, hi_deg):
    """Every whole 5 minutes strictly inside lo..hi (degrees)."""
    return [k / 12 for k in range(math.floor(lo_deg * 12), math.ceil(hi_deg * 12) + 1) if lo_deg + 1e-9 < k / 12 < hi_deg - 1e-9]


GRID_PARALLELS = _grid(LAT_S, LAT_N)  # 50°00'N
GRID_MERIDIANS = _grid(LON_W, LON_E)  # 2°55'W, 2°50'W, 2°45'W


def decals(tick_step_min: float = 0.1) -> list:
    """Everything printed on the sheet. ``tick_step_min`` sets the finest scale ticks (0.1' as on a real chart)."""
    out = [_decal(*INNER_X, *INNER_Y, WATER_TINT, layer=1)]
    out += _frame(*OUTER_X, *OUTER_Y, LINE_W, INK, 3)  # outer neatline
    out += _frame(*INNER_X, *INNER_Y, LINE_W / 2, INK, 3)  # inner neatline
    half = BORDER / 2  # outer half of the border: alternate whole minutes in black; inner half: the ticks
    for k in range(7):  # latitude scales, west (left) and east (right) borders
        if k % 2 == 0:
            xa, xb = to_world(LAT_S + k / 60, 0)[0], to_world(LAT_S + (k + 1) / 60, 0)[0]
            out.append(_decal(xa, xb, OUTER_Y[1] - half, OUTER_Y[1], INK, 2))
            out.append(_decal(xa, xb, OUTER_Y[0], OUTER_Y[0] + half, INK, 2))
    for k in range(16):  # longitude scales, south (near) and north (far) borders
        if k % 2 == 0:
            ya, yb = to_world(0, LON_W + k / 60)[1], to_world(0, LON_W + (k + 1) / 60)[1]
            out.append(_decal(OUTER_X[0], OUTER_X[0] + half, yb, ya, INK, 2))
            out.append(_decal(OUTER_X[1] - half, OUTER_X[1], yb, ya, INK, 2))
    n = round(1 / tick_step_min)
    for i in range(1, 7 * n):  # latitude ticks
        x = to_world(LAT_S + i / (60 * n), 0)[0]
        length = half if i % n == 0 else (0.7 * half if (2 * i) % n == 0 else 0.4 * half)
        out.append(_decal(x - TICK_W / 2, x + TICK_W / 2, INNER_Y[1], INNER_Y[1] + length, INK, 2))
        out.append(_decal(x - TICK_W / 2, x + TICK_W / 2, INNER_Y[0] - length, INNER_Y[0], INK, 2))
    for i in range(1, 16 * n):  # longitude ticks
        y = to_world(0, LON_W + i / (60 * n))[1]
        length = half if i % n == 0 else (0.7 * half if (2 * i) % n == 0 else 0.4 * half)
        out.append(_decal(INNER_X[0] - length, INNER_X[0], y - TICK_W / 2, y + TICK_W / 2, INK, 2))
        out.append(_decal(INNER_X[1], INNER_X[1] + length, y - TICK_W / 2, y + TICK_W / 2, INK, 2))
    for lat in GRID_PARALLELS:  # graticule every 5 minutes
        x = to_world(lat, 0)[0]
        out.append(_decal(x - TICK_W, x + TICK_W, *INNER_Y, INK, 2))
    for lon in GRID_MERIDIANS:
        y = to_world(0, lon)[1]
        out.append(_decal(*INNER_X, y - TICK_W, y + TICK_W, INK, 2))
    # the pencilled course line, from the buoy on 045° for LINE_NM
    (bx, by), (ex, ey) = BUOY_XY, LINE_END_XY
    length = math.hypot(ex - bx, ey - by)
    yaw = math.atan2(ey - by, ex - bx)
    cx, cy = (bx + ex) / 2, (by + ey) / 2
    out.append(((cx - PAPER_CENTRE[0], cy - PAPER_CENTRE[1], PAPER_THICK / 2 + 3.5 * INK_H),
                (length, LINE_W, INK_H), PENCIL, yaw))
    # the buoy: a red can symbol with its small position circle (dark dot) on the buoy's position
    out.append(_decal(bx - 0.0035, bx + 0.0035, by - 0.0035, by + 0.0035, BUOY_RED, 4))
    out.append(_decal(bx - 0.0010, bx + 0.0010, by - 0.0010, by + 0.0010, INK, 5))
    return out


def fmt(lat_deg: float, lon_deg: float) -> str:
    def part(v, pos, neg):
        h = pos if v >= 0 else neg
        v = abs(v)
        d = int(v)
        return f"{d}°{(v - d) * 60:04.1f}'{h}"
    return f"{part(lat_deg, 'N', 'S')} {part(lon_deg, 'E', 'W')}"


if __name__ == "__main__":
    print("paper", [round(v, 4) for v in PAPER_X], [round(v, 4) for v in PAPER_Y], "size", PAPER_SIZE)
    print("lat minute", round(lat_minute(LAT_S) * 1000, 2), "-", round(lat_minute(LAT_N) * 1000, 2), "mm;",
          "lon minute", round(LON_MIN * 1000, 2), "mm")
    print("buoy", fmt(*BUOY), [round(v, 4) for v in BUOY_XY])
    print("DR", fmt(*DR), [round(v, 4) for v in DR_XY])
    print("lon-scale error", [round(v, 4) for v in LON_SCALE_XY],
          "short by", round(1000 * math.dist(DR_XY, LON_SCALE_XY), 1), "mm")
    print("line end", [round(v, 4) for v in LINE_END_XY])
    print("decals", len(decals()))
