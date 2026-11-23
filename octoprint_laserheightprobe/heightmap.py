# coding=utf-8
"""A probed height map, turned into the Snapmaker's own bed mesh.

The idea: load the object's heights into the firmware's bilinear bed-levelling mesh and turn
levelling on. The firmware then adds the height under the head to the Z of every move, so a
job from LightBurn follows the object without LightBurn knowing anything about it.

What the firmware allows (Snapmaker2-Controller source, A350):

- one bilinear mesh, at most 11 x 11 points, over a FIXED area: machine X 52 to 332, Y 43 to
  328 whatever the size. The spacing is whole millimetres, so 11 points are 28 mm apart. Small
  objects get few nodes; this module says how well the grid can follow the surface.
- the mesh is indexed by MACHINE X, Y. A G92 or work origin (LightBurn's start G-code sets
  one) moves G-code X, Y but not the mesh, so samples carry the shift measured when they were
  probed (machine minus G-code, from the M114 step counters).
- the same mesh holds the 3D printing mesh, in absolute heights. meshload.py keeps a copy and
  puts it back.

The map is based at the LOWEST probed point (Chris's choice): that point is 0, everything else
is how far above it the surface is, so the mesh only ever lifts the head. LightBurn is focused
on the lowest point. If the mesh were ever lost, the head would stay at the lowest focus height
over the taller parts, which is why the summary compares the height range with the laser's
focus gap when that is known.

The fit: each node of the cells that hold samples is a free value, chosen by least squares so
the firmware's bilinear interpolation matches the samples, plus a small pull against curvature
(so a node the samples leave open continues the surface's slope instead of swinging), and a
far weaker pull towards equal neighbours. The nodes no
sample touches copy their neighbours outwards, so the head stays level away from the object.

A fitted node may sit a little below 0 or above the range: the object's edge rarely falls on a
grid line, so the node just outside it carries the surface's slope on (a plane is only exact
that way). The firmware's interpolation over the object itself stays within the probed heights.
So nodes are only kept within one range of the probed heights, and a dip below the lowest
point (just outside the probed area, over the bed) is reported. The worst miss at the samples
is reported, not hidden.

No OctoPrint in here, so the tests feed it numbers.
"""
from __future__ import absolute_import, division

import math

GRID = 11
GRID_START = (52.0, 43.0)
GRID_SPAN = (280.0, 285.0)
# Pull against curvature (second differences along X and Y), relative to one sample's weight.
# A plane costs nothing, so an incline fits exactly; it only decides what the samples leave open.
SMOOTH = 0.02
# A much weaker pull towards equal neighbours, so a node that no triple reaches is still pinned.
LEVEL = 1e-4
# M420 V prints three decimals and M421 is sent with three.
READBACK_TOLERANCE = 0.0015


class MapError(ValueError):
    """The samples cannot make a safe mesh."""


def firmware_geometry(grid=GRID):
    """The start and spacing the firmware uses for a grid of this size (whole millimetres)."""
    return GRID_START, (float(int(GRID_SPAN[0] / (grid - 1))), float(int(GRID_SPAN[1] / (grid - 1))))


def samples_from(points, shift):
    """[(machine_x, machine_y, z, point)] from a job's points and its machine shift.

    shift: (sx, sy) with machine = G-code + shift, measured while the points were probed.
    """
    if shift is None:
        raise MapError("these results have no machine X, Y shift. Probe them again with this "
                       "version of the plugin")
    out = []
    for p in points or []:
        if p.get("z") is None:
            continue
        out.append((round(p["x"] + shift[0], 3), round(p["y"] + shift[1], 3), float(p["z"]), p))
    return out


# -- the firmware's own evaluation ---------------------------------------------------------
#
# The A350 does not interpolate the mesh bilinearly between its points. Its Marlin build has
# ABL_BILINEAR_SUBDIVISION with BILINEAR_SUBDIVISIONS 3 (Configuration.h) and
# EXTRAPOLATE_BEYOND_GRID: M421 rebuilds a grid three times finer by Catmull-Rom splines through
# the points (abl.cpp bed_level_virt_interpolate, edges by linear extrapolation), and the head's
# lift is bilinear on that finer grid. Its spacing is the whole-mm grid spacing divided by 3 in
# integer arithmetic: 27 -> 9, but 29 -> 9, so in Y the A350 lays the mesh over 27 mm per point,
# not 29. Modelled exactly, the lift the printer applied at 24 spots on 2026-11-21 matched to
# 0.002 mm; plain bilinear had been up to 0.12 mm out (the "0.16 mm mismatch").
# All of it is linear in the node heights, so the map is fitted through this model too: what the
# printer then applies is the probed surface.

SUBDIVISIONS = 3
MODEL = "a350-catmull-rom-3"            # maps fitted through this model carry this tag
_VIRT_CACHE = {}


def effective_end(start, spacing, grid):
    """Where the A350's subdivided grid really ends (its spacing is whole-mm // 3 * 3)."""
    return tuple(start[k] + (int(spacing[k]) // SUBDIVISIONS) * SUBDIVISIONS * (grid - 1) for k in (0, 1))


def _cr(t):
    """Catmull-Rom weights on p[i-1], p[i], p[i+1], p[i+2] at t (abl.cpp bed_level_virt_cmr)."""
    return (-t * (1 - t) ** 2 * 0.5, (2 - 5 * t * t + 3 * t ** 3) * 0.5,
            t * (1 + 4 * t - 3 * t * t) * 0.5, -t * t * (1 - t) * 0.5)


def _add(acc, weights, k):
    for node, w in weights.items():
        acc[node] = acc.get(node, 0.0) + k * w


def _temp_coord(x, y, n):
    """bed_level_virt_coord(x, y) on the firmware's (n + 2) padded grid, as node weights."""
    if x > n + 1 or y > n + 1:
        return {}                              # read past the array, always weighted by t = 0
    if x == 0 or x == n + 1:
        ep, ip = (n - 1, n - 2) if x else (0, 1)
        if 1 <= y <= n:
            return {(ep, y - 1): 2.0, (ip, y - 1): -1.0}
        out = {}
        _add(out, _temp_coord(ep + 1, y, n), 2.0)
        _add(out, _temp_coord(ip + 1, y, n), -1.0)
        return out
    if y == 0 or y == n + 1:
        ep, ip = (n - 1, n - 2) if y else (0, 1)
        if 1 <= x <= n:
            return {(x - 1, ep): 2.0, (x - 1, ip): -1.0}
        out = {}
        _add(out, _temp_coord(x, ep + 1, n), 2.0)
        _add(out, _temp_coord(x, ip + 1, n), -1.0)
        return out
    return {(x - 1, y - 1): 1.0}


def _virt_weights(n):
    """{(vx, vy): {(i, j): w}} for the firmware's subdivided grid of an n x n mesh (cached)."""
    if n not in _VIRT_CACHE:
        sub, table = SUBDIVISIONS, {}
        nv = (n - 1) * sub + 1
        for vx in range(nv):
            for vy in range(nv):
                x, tx = min(vx // sub, n - 1), vx - min(vx // sub, n - 1) * sub
                y, ty = min(vy // sub, n - 1), vy - min(vy // sub, n - 1) * sub
                cx, cy = _cr(tx / float(sub)), _cr(ty / float(sub))
                acc = {}
                for i in range(4):
                    for j in range(4):
                        k = cx[i] * cy[j]
                        if k:
                            _add(acc, _temp_coord(x + i, y + j, n), k)
                table[(vx, vy)] = dict((node, w) for node, w in acc.items() if abs(w) > 1e-12)
        _VIRT_CACHE[n] = table
    return _VIRT_CACHE[n]


def _virt_axis(v, start, spacing, n):
    """(cell index, fraction) on the subdivided grid along one axis, as abl.cpp
    bilinear_z_offset does it: int start, int spacing // 3, last cell carried on past the edge."""
    sv = int(spacing) // SUBDIVISIONS
    nv = (n - 1) * SUBDIVISIONS + 1
    r = (v - int(start)) / float(sv)
    g = min(max(int(math.floor(r)), 0), nv - 2)
    return g, r - g


def _weights(mx, my, start, spacing, n):
    """Node weights of the lift the firmware applies at machine (mx, my)."""
    gx, u = _virt_axis(mx, start[0], spacing[0], n)
    gy, w = _virt_axis(my, start[1], spacing[1], n)
    table = _virt_weights(n)
    acc = {}
    for (vx, vy), k in (((gx, gy), (1 - u) * (1 - w)), ((gx + 1, gy), u * (1 - w)),
                        ((gx, gy + 1), (1 - u) * w), ((gx + 1, gy + 1), u * w)):
        if k:
            _add(acc, table[(vx, vy)], k)
    return list(acc.items())


def evaluate(nodes, start, spacing, mx, my, extrapolate=False):
    """The lift the A350 applies at machine (mx, my) for these mesh nodes (see above). Clamped
    to the grid's area by default (the conservative figure the map's checks use);
    extrapolate=True carries the edge cell on past the grid, as the A350 does."""
    n = len(nodes)
    if not extrapolate:
        sv = (int(spacing[0]) // SUBDIVISIONS, int(spacing[1]) // SUBDIVISIONS)
        span = ((n - 1) * SUBDIVISIONS * sv[0], (n - 1) * SUBDIVISIONS * sv[1])
        mx = min(max(mx, int(start[0])), int(start[0]) + span[0])
        my = min(max(my, int(start[1])), int(start[1]) + span[1])
    return sum(w * nodes[i][j] for (i, j), w in _weights(mx, my, start, spacing, n))


def virtual_grid(nodes):
    """The firmware's subdivided grid values for these nodes, [vx][vy]."""
    n = len(nodes)
    nv = (n - 1) * SUBDIVISIONS + 1
    table = _virt_weights(n)
    return [[sum(w * nodes[i][j] for (i, j), w in table[(vx, vy)].items()) for vy in range(nv)]
            for vx in range(nv)]


def _solve(a, b):
    """Solve a x = b (a square, dense) by Gaussian elimination with partial pivoting."""
    n = len(b)
    m = [list(a[r]) + [b[r]] for r in range(n)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        if abs(m[pivot][col]) < 1e-12:
            raise MapError("the height map fit is singular")
        m[col], m[pivot] = m[pivot], m[col]
        for r in range(col + 1, n):
            f = m[r][col] / m[col][col]
            if f:
                for k in range(col, n + 1):
                    m[r][k] -= f * m[col][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (m[r][n] - sum(m[r][k] * x[k] for k in range(r + 1, n))) / m[r][r]
    return x


def _real_cell(mx, my, start, spacing, grid):
    """The low node of the mesh cell holding machine (mx, my), as the A350 lays it (eff. spacing)."""
    out = []
    for v, st, sp in ((mx, start[0], spacing[0]), (my, start[1], spacing[1])):
        eff = (int(sp) // SUBDIVISIONS) * SUBDIVISIONS
        out.append(min(max(int(math.floor((v - int(st)) / float(eff))), 0), grid - 2))
    return tuple(out)


def build(points, shift, start=None, spacing=None, grid=GRID, tolerance=0.5, focus_gap=None,
          smooth=SMOOTH, plateau=False):
    """The firmware mesh for a job's points, and how well it fits. Raises MapError.

    Returns dict(nodes=z[i][j] (i along machine X), start, spacing, grid, base (the lowest point),
    range, worst (largest miss at a sample), rms, samples=[...], slope_deg, warnings=[...]).

    plateau=True (a map of one object): every node outside the cells the points cover is tied to
    the nearest covered node, so the mesh holds the object's edge heights flat beyond it. Alone,
    an object's points let the fit carry its slope across the whole bed (-10.9 to +32 mm at the
    edges, 2026-11-21); merged with bed points, the step at the object's edge made the fit swing
    from -33 to +37 mm. The plateau can only put the head higher than the bed needs there.
    """
    if start is None or spacing is None:
        start, spacing = firmware_geometry(grid)
    found = samples_from(points, shift)
    if len(found) < 1:
        raise MapError("there are no measured points to make a height map from")
    end = effective_end(start, spacing, grid)
    for mx, my, _, p in found:
        if not (start[0] - 1e-6 <= mx <= end[0] + 1e-6 and start[1] - 1e-6 <= my <= end[1] + 1e-6):
            raise MapError("point X%.2f Y%.2f is at machine X%.2f Y%.2f, outside the firmware's mesh "
                           "area (X%g to %g, Y%g to %g)" % (p["x"], p["y"], mx, my, start[0], end[0],
                                                           start[1], end[1]))
    low = min(found, key=lambda s: s[2])
    z0 = low[2]
    heights = [(mx, my, z - z0) for mx, my, z, _ in found]
    top = max(h for _, _, h in heights)

    # Free values: every node of a cell that holds a sample.
    rows = [_weights(mx, my, start, spacing, grid) for mx, my, _ in heights]
    tie = None
    if plateau:
        cells = [_real_cell(mx, my, start, spacing, grid) for mx, my, _ in heights]
        lo_i, hi_i = min(c[0] for c in cells), max(c[0] for c in cells) + 1
        lo_j, hi_j = min(c[1] for c in cells), max(c[1] for c in cells) + 1

        def tie(node):
            return (min(max(node[0], lo_i), hi_i), min(max(node[1], lo_j), hi_j))
        merged = []
        for row in rows:
            acc = {}
            for node, w in row:
                t = tie(node)
                acc[t] = acc.get(t, 0.0) + w
            merged.append(list(acc.items()))
        rows = merged
    free = sorted(set(node for row in rows for node, _ in row))
    index = dict((node, k) for k, node in enumerate(free))
    size = len(free)
    a = [[0.0] * size for _ in range(size)]
    b = [0.0] * size
    for row, (_, _, h) in zip(rows, heights):
        for node_p, w_p in row:
            b[index[node_p]] += w_p * h
            for node_q, w_q in row:
                a[index[node_p]][index[node_q]] += w_p * w_q
    def penalise(terms, weight):
        """Add weight * (sum of coefficient * node)^2 to the normal equations."""
        for node_p, c_p in terms:
            for node_q, c_q in terms:
                a[index[node_p]][index[node_q]] += weight * c_p * c_q

    for (i, j) in free:
        for di, dj in ((1, 0), (0, 1)):
            nb = (i + di, j + dj)
            if nb in index:
                penalise([((i, j), 1.0), (nb, -1.0)], LEVEL)
            far = (i + 2 * di, j + 2 * dj)
            if nb in index and far in index:
                penalise([((i, j), 1.0), (nb, -2.0), (far, 1.0)], smooth)
    for k in range(size):
        a[k][k] += 1e-9                      # a node only a zero weight reaches stays solvable
    values = _solve(a, b)

    nodes = [[None] * grid for _ in range(grid)]
    for (i, j), v in zip(free, values):
        nodes[i][j] = min(max(v, -top), 2 * top)
    if tie is not None:
        for i in range(grid):
            for j in range(grid):
                t = tie((i, j))
                if (i, j) != t and nodes[t[0]][t[1]] is not None:
                    nodes[i][j] = nodes[t[0]][t[1]]
    _fill_outwards(nodes)
    dip = -min(v for col in nodes for v in col)

    misses = []
    for (mx, my, h), (_, _, _, p) in zip(heights, found):
        fitted = evaluate(nodes, start, spacing, mx, my)
        misses.append(dict(x=p["x"], y=p["y"], machine_x=mx, machine_y=my, height=round(h, 4),
                           fitted=round(fitted, 4), miss=round(fitted - h, 4)))
    worst = max(misses, key=lambda m: abs(m["miss"]))
    rms = math.sqrt(sum(m["miss"] ** 2 for m in misses) / len(misses))
    slope = 0.0
    for i in range(grid):
        for j in range(grid):
            if i + 1 < grid:
                slope = max(slope, abs(nodes[i + 1][j] - nodes[i][j]) / spacing[0])
            if j + 1 < grid:
                slope = max(slope, abs(nodes[i][j + 1] - nodes[i][j]) / spacing[1])

    warnings = []
    if abs(worst["miss"]) > tolerance:
        warnings.append("The firmware's %g mm grid cannot follow this surface closely: at X%.2f Y%.2f "
                        "the mesh is %+.2f mm off the probed height (tolerance %.2f mm). Expect that "
                        "spot out of focus." % (spacing[0], worst["x"], worst["y"], worst["miss"],
                                                tolerance))
    if focus_gap is None:
        warnings.append("The height range is %.2f mm. Set the laser's focus gap in the settings so "
                        "the plugin can check that a lost mesh could not put the head into the "
                        "object." % top)
    elif top >= focus_gap:
        warnings.append("The height range (%.2f mm) is at least the laser's focus gap (%.2f mm). If "
                        "the mesh were lost, the head would hit the tallest part. Check levelling is "
                        "ON before every job." % (top, focus_gap))
    if dip > 0.05:
        warnings.append("Just outside the probed area the mesh dips up to %.2f mm below the lowest "
                        "point, so the head runs that much lower there. Keep that area clear of "
                        "anything taller than the lowest point, or probe a wider area." % dip)
    if len(free) < 4 or len(found) < 3:
        warnings.append("Only %d point%s: the map is a guess between them."
                        % (len(found), "" if len(found) == 1 else "s"))
    return dict(nodes=[[round(v, 3) for v in col] for col in nodes], start=list(start),
                spacing=list(spacing), grid=grid,
                base=_base(low),
                range=round(top, 4), worst=worst, rms=round(rms, 4), samples=misses,
                slope_deg=round(math.degrees(math.atan(slope)), 1), free_nodes=len(free),
                dip=round(max(dip, 0.0), 3),
                warnings=warnings, tolerance=tolerance, focus_gap=focus_gap, model=MODEL,
                plateau=bool(plateau))


def _base(low):
    """The lowest point, with its focus in G-code Z and, from the step counter, in machine Z.

    The mesh only carries differences. The absolute focus still comes from LightBurn's start
    G-code (G53, G0 Z.., G92 Z0), which is in machine Z, so that is the number to give it.
    """
    mx, my, z, p = low
    focus, machine_z = p.get("focus"), p.get("machine_z")
    machine_focus = None
    if focus is not None and machine_z is not None:
        machine_focus = round(machine_z + (focus - z), 3)
    return dict(x=p["x"], y=p["y"], z=z, machine_x=mx, machine_y=my, focus=focus,
                machine_z=machine_z, machine_focus=machine_focus)


def refit(height_map, start, spacing):
    """The same samples fitted to another grid position (the one the printer reports).

    The firmware places its grid with RAW_X_POSITION at the time of G1029, so a work origin in
    force then can move it. The samples are in machine X, Y, so only the fit changes.
    """
    grid = height_map["grid"]
    end = effective_end(start, spacing, grid)
    inside = [s for s in height_map["samples"]
              if start[0] - 1e-6 <= s["machine_x"] <= end[0] + 1e-6
              and start[1] - 1e-6 <= s["machine_y"] <= end[1] + 1e-6]
    if len(inside) < 3:
        raise MapError("only %d probed point%s inside the printer's mesh area (X%g to %g, Y%g to %g): "
                       "probe inside it" % (len(inside), "" if len(inside) == 1 else "s",
                                            start[0], end[0], start[1], end[1]))
    points = [dict(x=s["machine_x"], y=s["machine_y"], z=s["height"]) for s in inside]
    again = build(points, (0.0, 0.0), start=start, spacing=spacing, grid=grid,
                  tolerance=height_map.get("tolerance", 0.5), focus_gap=height_map.get("focus_gap"),
                  plateau=bool(height_map.get("plateau")))
    for new, old in zip(again["samples"], inside):
        new["x"], new["y"] = old["x"], old["y"]
    left_out = len(height_map["samples"]) - len(inside)
    if left_out:
        again["warnings"] = list(again["warnings"]) + [
            "%d probed point%s outside the printer's mesh area (machine X%g to %g, Y%g to %g) %s left "
            "out: past its edge the printer holds the edge heights" % (
                left_out, "" if left_out == 1 else "s", start[0], end[0], start[1], end[1],
                "was" if left_out == 1 else "were")]
    worst = max(again["samples"], key=lambda m: abs(m["miss"]))
    out = dict(height_map)
    out.update(nodes=again["nodes"], start=again["start"], spacing=again["spacing"],
               samples=again["samples"], worst=worst, rms=again["rms"], dip=again["dip"],
               slope_deg=again["slope_deg"], free_nodes=again["free_nodes"],
               warnings=again["warnings"], model=MODEL, plateau=again["plateau"])
    return out


def _fill_outwards(nodes):
    """Give every empty node the mean of its filled neighbours, ring by ring outwards."""
    n, m = len(nodes), len(nodes[0])
    while any(v is None for col in nodes for v in col):
        updates = {}
        for i in range(n):
            for j in range(m):
                if nodes[i][j] is not None:
                    continue
                near = [nodes[a][b] for a, b in ((i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1),
                                                 (i - 1, j - 1), (i + 1, j + 1), (i - 1, j + 1),
                                                 (i + 1, j - 1))
                        if 0 <= a < n and 0 <= b < m and nodes[a][b] is not None]
                if near:
                    updates[(i, j)] = sum(near) / len(near)
        if not updates:
            raise MapError("no node of the mesh has a value")
        for (i, j), v in updates.items():
            nodes[i][j] = v


def commands(nodes):
    """One M421 per node."""
    return ["M421 I%d J%d Z%.3f" % (i, j, nodes[i][j])
            for i in range(len(nodes)) for j in range(len(nodes[i]))]


def same(a, b, tolerance=READBACK_TOLERANCE):
    """True when two meshes have the same size and every node agrees within `tolerance`."""
    if not a or not b or len(a) != len(b) or any(len(x) != len(y) for x, y in zip(a, b)):
        return False
    return all(abs(p - q) <= tolerance for x, y in zip(a, b) for p, q in zip(x, y))
