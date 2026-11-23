# coding=utf-8
"""Where the head goes for close-up photos. Plain Python.

Every head position is target - model.centre_offset(z, h), rounded to 0.1 mm. The touchscreen
is sent X, Y, Z to one decimal (headcam.request_url) and must find the head already there, or it
moves the head itself.
"""
from __future__ import absolute_import

# Stage A (printed ChArUco board): 3x3 head offsets at each close-up height, wider higher up,
# then a few higher photos so the fit spans the overview height too.
BOARD_LEVELS = ((40.0, 15.0), (45.0, 15.0), (50.0, 20.0), (60.0, 20.0), (80.0, 25.0))
BOARD_HIGH = ((120.0, ((0, 0), (-40, -40), (40, -40), (-40, 40), (40, 40))),
              (200.0, ((0, 0), (-60, 0), (60, 0))),
              (290.0, ((0, 0), (0, -60), (0, 60))))
HOLDOUT_Z = 45.0                       # the whole level, plus one corner of each other level
# Stage B (burned crosses): each cross centred, then moved across the photo, and one higher.
CROSS_POSES = ((50.0, 0.0, 0.0), (50.0, 15.0, 0.0), (50.0, 0.0, 15.0), (60.0, 0.0, 0.0))


def _r(value):
    return round(value + 0.0, 1)


def _pose(target, dx, dy, z, model, **extra):
    ox, oy = model.centre_offset(z, 0.0)
    pose = dict(x=_r(target[0] + dx - ox), y=_r(target[1] + dy - oy), z=_r(z), dx=float(dx), dy=float(dy))
    pose.update(extra)
    return pose


BED_W, BED_H = 320.0, 350.0
MIN_BOX = 2.0
TRAVEL_FLOOR = 200.0          # head Z for any XY move outside the module sweep (rotary clearance)
XY_FEED_MM_S = 3000 / 60.0
Z_FEED_MM_S = 1500 / 60.0
LATER_TILE = 0.6                # a later tile's photo, as a share of capture_s
COMPOSE_S = 20.0              # the test machine, 2026-11-21: 15-30 s for 1-3 tiles


# The head's shadow on a close-up photo (room light, 2026-11-21, Z50): its left edge was at photo
# column 660 with the head at X144.4 and 846 at X242.6, moving with the head.
SHADOW_COL = (660.0, 144.4, (846.0 - 660.0) / (242.6 - 144.4))
SHADOW_SPARE = 0.05
# The 10W module's outline round its beam, in mm (-X, -Y, +X, +Y), from Snapmaker's
# Snap-LASER-10W-ASSY.STEP: the body is 114 x 67 mm with the beam 17 mm right of its middle; the
# camera side (-X) reaches 74 mm. Front and back are taken as the larger, 37 mm, both ways.
MODULE_REACH = (74.0, 37.0, 40.0, 37.0)


def shadow_frac(head_x, z=50.0, width=1024):
    """The share of a photo's right side the head's shadow may cover with the head at head_x."""
    col = SHADOW_COL[0] + SHADOW_COL[2] * (head_x - SHADOW_COL[1])
    return min(0.6, max(0.15, (width - col) / float(width) + SHADOW_SPARE))


def _shadow_columns(x0, x1, head_z, h, d_bed, margin, ox, step, model):
    """(head X, view x0, view x1) per column, left to right: each column's view is as wide as the
    head's shadow at that head X allows, and neighbours overlap by (1 - step) of the narrower."""
    left, top, _, bottom = margin if isinstance(margin, (tuple, list)) else (margin,) * 4

    def view(hx):
        frac = shadow_frac(hx, head_z, model.width)
        fx0, _, fx1, _ = model.footprint(head_z, h, (left, top, left + int(frac * model.width), bottom), d_bed)
        return hx - ox + fx0, hx - ox + fx1

    fx0 = model.footprint(head_z, h, (left, top, left, bottom), d_bed)[0]
    columns = []
    edge = x0
    for _ in range(60):
        hx = _r(edge + ox - fx0 - 0.15)          # 0.15 mm spare: the 0.1 mm rounding
        v0, v1 = view(hx)
        columns.append((hx, v0, v1))
        if v1 - 0.1 >= x1:
            break
        edge = v1 - (1.0 - step) * (v1 - v0)
    if len(columns) == 1:                    # one tile: centre it on the box
        v0, v1 = columns[0][1:]
        shift = (x0 + x1) / 2.0 - (v0 + v1) / 2.0
        if v0 + shift <= x0 + 1e-6 and v1 + shift >= x1 - 1e-6:
            hx = _r(columns[0][0] + shift)
            columns = [(hx,) + view(hx)]
    return columns


def _spread(lo, hi, size, step):
    """Tile centres along one axis: one tile if the box fits, else evenly spread so the outer
    tiles meet the box edges, no further apart than step."""
    import math
    width = hi - lo
    if width <= size:
        return [(lo + hi) / 2.0]
    n = int(math.ceil((width - size) / step - 1e-9)) + 1
    first, last = lo + size / 2.0, hi - size / 2.0
    return [first + (last - first) * i / (n - 1) for i in range(n)]


def plan(box_lb, h, standoff, ctx, model):
    """The tile plan for a close-up of box_lb = (x0, y0, x1, y1) in LightBurn mm, of an object
    whose top (the tallest thing under the module sweep) is h above the bare bed, with the head
    standoff above it. ctx: o (bridge offset), limits ((minx, miny), (maxx, maxy)) of the head,
    current (x, y, z), hood_touch_z (head Z where the 10W hood meets the bare bed), d_bed, and
    the settings (overlap, min_clearance, max_height, hood_radius (margin), max_tiles, capture_s,
    margin_px, standoff_range). ValueError says why a close-up cannot be done."""
    x0, y0, x1, y1 = [float(v) for v in box_lb]
    if x0 > x1:
        x0, x1 = x1, x0
    if y0 > y1:
        y0, y1 = y1, y0
    if x0 < 0 or y0 < 0 or x1 > BED_W or y1 > BED_H:
        raise ValueError("the box must be inside the bed (LightBurn 0..%g x 0..%g mm)" % (BED_W, BED_H))
    if x1 - x0 < MIN_BOX or y1 - y0 < MIN_BOX:
        raise ValueError("the box must be at least 2 mm each way")
    if h is None:
        raise ValueError("say the object's height (the tallest thing under the dashed area, 0 = flat)")
    h = float(h)
    if not 0.0 <= h <= ctx["max_height"]:
        raise ValueError("the object height %.1f mm is outside 0..%.0f mm" % (h, ctx["max_height"]))
    lo, hi = ctx["standoff_range"]
    if not lo <= standoff <= hi:
        raise ValueError("the stand-off %.1f mm is outside %.0f..%.0f mm" % (standoff, lo, hi))
    d_bed = ctx.get("d_bed", 0.0)
    head_z = _r(standoff + h + d_bed)
    if model.z_range and not model.z_range[0] - 0.05 <= head_z <= model.z_range[1] + 0.05:
        raise ValueError("head Z %.1f is outside the heights the camera was calibrated at (%.0f..%.0f)"
                         % (head_z, model.z_range[0], model.z_range[1]))
    clearance = head_z - (ctx["hood_touch_z"] + h)
    if clearance < ctx["min_clearance"]:
        raise ValueError("the 10W's hood would be %.1f mm above the object (minimum clearance %.0f mm): "
                         "raise the stand-off" % (clearance, ctx["min_clearance"]))
    margin = ctx.get("margin_px", 40)
    fx0, fy0, fx1, fy1 = model.footprint(head_z, h, margin, d_bed)
    fw, fh = fx1 - fx0, fy1 - fy0
    cx_off, cy_off = (fx0 + fx1) / 2.0, (fy0 + fy1) / 2.0
    ox, oy = ctx["o"]
    step = 1.0 - ctx["overlap"]
    # spread with the view 0.2 mm smaller, so rounding the head to 0.1 mm never leaves the
    # outer tiles short of the box edge
    rows = _spread(y0, y1, fh - 0.2, (fh - 0.2) * step)
    if ctx.get("shadow"):
        # columns, left to right, each as wide as its own head's shadow allows (wider to the right)
        columns = _shadow_columns(x0, x1, head_z, h, d_bed, margin, ox, step, model)
    else:
        heads = [_r(tx + ox - cx_off) for tx in _spread(x0, x1, fw - 0.2, (fw - 0.2) * step)]
        columns = [(hx, hx + cx_off - ox - fw / 2.0, hx + cx_off - ox + fw / 2.0) for hx in heads]
    cols = columns
    if len(cols) * len(rows) > ctx["max_tiles"]:
        raise ValueError("the box needs %d tiles at this stand-off (maximum %d): make it smaller or "
                         "raise the stand-off" % (len(cols) * len(rows), ctx["max_tiles"]))
    (minx, miny), (maxx, maxy) = ctx["limits"]
    cur = ctx["current"]
    rows.sort(key=lambda ty: abs(ty + oy - cy_off - cur[1]))
    rows = [rows[0]] + sorted(rows[1:], key=lambda ty: abs(ty - rows[0]))
    tiles = []
    # serpentine: the first row from its end nearer the head, then alternate
    ascending = abs(cols[0][0] - cur[0]) <= abs(cols[-1][0] - cur[0])
    for ty in rows:
        line = sorted(cols, reverse=not ascending)
        ascending = not ascending
        for hx, vx0, vx1 in line:
            hy = _r(ty + oy - cy_off)
            if not minx <= hx <= maxx:
                reach = (maxx + cx_off - ox + fw / 2.0, minx + cx_off - ox - fw / 2.0)
                raise ValueError("a tile needs head X %.1f, beyond the machine's X limit (%.0f..%.0f): the "
                                 "camera sees LightBurn X up to %.0f (and from %.0f) at this height"
                                 % (hx, minx, maxx, reach[0], max(reach[1], 0.0)))
            if not miny <= hy <= maxy:
                raise ValueError("a tile needs head Y %.1f, beyond the machine's Y limit (%.0f..%.0f)"
                                 % (hy, miny, maxy))
            vy = hy + cy_off - oy                                  # the view centre in LB
            tiles.append(dict(i=len(tiles), x=hx, y=hy, z=head_z, target_lb=((vx0 + vx1) / 2.0, ty),
                              view_lb=(vx0, vy - fh / 2.0, vx1, vy + fh / 2.0)))
    m = ctx["hood_radius"]                     # the margin round the module's outline
    rx0, ry0, rx1, ry1 = MODULE_REACH
    beams = [(t["x"] - ox, t["y"] - oy) for t in tiles]
    sweep = (min(b[0] for b in beams) - rx0 - m, min(b[1] for b in beams) - ry0 - m,
             max(b[0] for b in beams) + rx1 + m, max(b[1] for b in beams) + ry1 + m)
    travel_z = max(float(cur[2]), TRAVEL_FLOOR)
    path = [(cur[0], cur[1])] + [(t["x"], t["y"]) for t in tiles]
    xy = sum(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5 for a, b in zip(path, path[1:]))
    # later photos are quicker than the first (the head is already low; 2026-11-21: 15 tiles took
    # 314-326 s, one took 87-105 s)
    seconds = (ctx["capture_s"] * (1 + LATER_TILE * (len(tiles) - 1)) + xy / XY_FEED_MM_S
               + 2 * (travel_z - head_z) / Z_FEED_MM_S + COMPOSE_S + ctx.get("extra_s", 0.0))
    return dict(box_lb=(x0, y0, x1, y1), h=h, standoff=float(standoff), head_z=head_z, travel_z=travel_z,
                nx=len(cols), ny=len(rows), view_mm=(cols[0][2] - cols[0][1], fh), tiles=tiles, sweep_lb=sweep,
                seconds=seconds, o=(ox, oy), clearance=clearance)


def calib_poses(stage, target, model):
    """Head positions for calibration photos. stage "board": target is the board's middle in the
    B frame. stage "cross": target is the list of burned crosses in the B frame (LB + o)."""
    if stage == "board":
        poses = []
        for z, d in BOARD_LEVELS:
            for j, dy in enumerate((-d, 0.0, d)):
                for i, dx in enumerate((-d, 0.0, d)):
                    corner = i == 2 and j == 2
                    poses.append(_pose(target, dx, dy, z, model, holdout=(z == HOLDOUT_Z or corner)))
        for z, offsets in BOARD_HIGH:
            for dx, dy in offsets:
                poses.append(_pose(target, dx, dy, z, model, holdout=False))
        return poses
    if stage == "cross":
        return [_pose(cross, dx, dy, z, model, cross=n, holdout=False)
                for n, cross in enumerate(target) for z, dx, dy in CROSS_POSES]
    raise ValueError("unknown calibration stage %r (board or cross)" % (stage,))
