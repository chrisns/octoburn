# coding=utf-8
"""Rotary maths: where a round object's axis is, from probed points. Pure, no OctoPrint.

Coordinates are the probe's: X across the rotary axis, Y along it, Z up, and every Z is a
trigger Z (the commanded Z when the deployed pin trips). The rotary axis runs along Y.

- A cross-section (one Y): points (x, z) on the top of the object. With the diameter known
  (fit_circle), two points or more give the axis X and Z; with it unknown (fit_circle_free),
  three or more give the radius too, which checks the diameter.
- Stations: the axis at several Y gives its line (axis_line): how far it runs off square in
  X and how far it climbs in Z per mm of Y.
- Runout: the top of the object probed at one X, Y while B turns. z = a + e cos(B - phase)
  (fit_runout): e is how far the object sits off the chuck's axis.

Probe geometry for clearance (printed head, laser-height-probe.scad): the pin is on the beam
axis; the deployed tip is 24.0 below the plate's bottom edge, the stowed tip 18.9 below
(5.1 above the deployed tip), the probe body's underside 13.6 below (10.4 above the deployed
tip), the body 13.2 x 14 around the pin, and the plate's bottom edge runs 55 mm to -X and 22 mm
to +X of the pin.
"""
from __future__ import absolute_import, division

import math

STOWED_ABOVE_TIP = 5.1
BODY_ABOVE_TIP = 10.4
PLATE_ABOVE_TIP = 24.0
BODY_HALF_X = 6.6
BODY_HALF_Y = 7.0
PLATE_LEFT_X = 55.0
PLATE_RIGHT_X = 22.0


class FitError(ValueError):
    pass


def _solve(a, b):
    """Solve the small dense system a x = b (Gaussian elimination with pivoting)."""
    n = len(b)
    m = [list(map(float, row)) + [float(v)] for row, v in zip(a, b)]
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(m[r][i]))
        if abs(m[p][i]) < 1e-12:
            raise FitError("the points do not pin the fit down (are they all in one place?)")
        m[i], m[p] = m[p], m[i]
        for r in range(i + 1, n):
            f = m[r][i] / m[i][i]
            for c in range(i, n + 1):
                m[r][c] -= f * m[i][c]
    x = [0.0] * n
    for i in reversed(range(n)):
        x[i] = (m[i][n] - sum(m[i][c] * x[c] for c in range(i + 1, n))) / m[i][i]
    return x


def _lsq(rows, rhs):
    """Least squares through the normal equations (small problems only)."""
    k = len(rows[0])
    ata = [[sum(r[i] * r[j] for r in rows) for j in range(k)] for i in range(k)]
    atb = [sum(r[i] * v for r, v in zip(rows, rhs)) for i in range(k)]
    return _solve(ata, atb)


def fit_circle(points, radius, iterations=50):
    """Axis (xc, zc) of a circle of known radius through the tops of points (x, z).

    Each point is on the upper half: z = zc + sqrt(r^2 - (x - xc)^2). Returns dict(xc, zc,
    crown = zc + r, rms, worst) with residuals measured along Z.
    """
    pts = [(float(x), float(z)) for x, z in points]
    if len(pts) < 2:
        raise FitError("need two points or more for a known-radius fit")
    r = float(radius)
    top = max(pts, key=lambda p: p[1])
    xc, zc = top[0], top[1] - r
    for _ in range(iterations):
        rows, res = [], []
        for x, z in pts:
            d = x - xc
            if abs(d) >= r:
                raise FitError("a point at X%.2f is outside a %.1f mm radius from the axis guess"
                               % (x, r))
            s = math.sqrt(r * r - d * d)
            res.append(z - (zc + s))
            rows.append([d / s, 1.0])                         # d/dxc, d/dzc of the model
        step = _lsq(rows, res)
        xc += step[0]
        zc += step[1]
        if abs(step[0]) < 1e-9 and abs(step[1]) < 1e-9:
            break
    resid = [z - (zc + math.sqrt(r * r - (x - xc) ** 2)) for x, z in pts]
    return dict(xc=xc, zc=zc, crown=zc + r, radius=r,
                rms=math.sqrt(sum(v * v for v in resid) / len(resid)),
                worst=max(resid, key=abs), residuals=resid)


def fit_circle_free(points):
    """Axis and radius from three points or more (algebraic fit, then Gauss-Newton)."""
    pts = [(float(x), float(z)) for x, z in points]
    if len(pts) < 3:
        raise FitError("need three points or more to fit the radius too")
    # x^2 + z^2 = 2 a x + 2 b z + c
    a, b, c = _lsq([[2 * x, 2 * z, 1.0] for x, z in pts], [x * x + z * z for x, z in pts])
    r = math.sqrt(max(c + a * a + b * b, 1e-9))
    xc, zc = a, b
    for _ in range(50):
        rows, res = [], []
        for x, z in pts:
            dist = math.hypot(x - xc, z - zc)
            res.append(dist - r)
            rows.append([-(x - xc) / dist, -(z - zc) / dist, -1.0])
        step = _lsq(rows, [-v for v in res])
        xc, zc, r = xc + step[0], zc + step[1], r + step[2]
        if max(abs(s) for s in step) < 1e-10:
            break
    resid = [math.hypot(x - xc, z - zc) - r for x, z in pts]
    return dict(xc=xc, zc=zc, radius=r, diameter=2 * r, crown=zc + r,
                rms=math.sqrt(sum(v * v for v in resid) / len(resid)), residuals=resid)


def axis_line(stations):
    """Line through (y, xc, zc) stations: dict(x0, z0 at y0, dx_dy, dz_dy, y0), y0 = mean Y."""
    st = [(float(y), float(x), float(z)) for y, x, z in stations]
    if len(st) < 2:
        raise FitError("need two stations or more for the axis line")
    y0 = sum(s[0] for s in st) / len(st)
    syy = sum((s[0] - y0) ** 2 for s in st)
    if syy < 1e-9:
        raise FitError("the stations are all at one Y")
    x0 = sum(s[1] for s in st) / len(st)
    z0 = sum(s[2] for s in st) / len(st)
    dx = sum((s[0] - y0) * (s[1] - x0) for s in st) / syy
    dz = sum((s[0] - y0) * (s[2] - z0) for s in st) / syy
    return dict(y0=y0, x0=x0, z0=z0, dx_dy=dx, dz_dy=dz,
                x_at=lambda y: x0 + dx * (y - y0), z_at=lambda y: z0 + dz * (y - y0))


def fit_runout(samples):
    """samples (b_degrees, z) -> dict(mean, e, phase_deg, rms): z = mean + e cos(B - phase)."""
    s = [(math.radians(float(b)), float(z)) for b, z in samples]
    if len(s) < 3:
        raise FitError("need three B angles or more for the runout")
    m, p, q = _lsq([[1.0, math.cos(b), math.sin(b)] for b, _ in s], [z for _, z in s])
    resid = [z - (m + p * math.cos(b) + q * math.sin(b)) for b, z in s]
    return dict(mean=m, e=math.hypot(p, q), phase_deg=math.degrees(math.atan2(q, p)) % 360.0,
                rms=math.sqrt(sum(v * v for v in resid) / len(resid)))


def chord_xs(center, half_width, n):
    """n X positions spread evenly over center +/- half_width."""
    if n < 2:
        return [float(center)]
    return [center - half_width + 2.0 * half_width * i / (n - 1) for i in range(n)]


def max_chord_half_width(radius, max_slope_deg=35.0):
    """How far off the crown a pin can touch before the surface slope passes max_slope_deg."""
    return radius * math.sin(math.radians(max_slope_deg))


def travel_z(crown_trigger_z, clearance=5.0):
    """Commanded Z for moves with the pin stowed: the stowed tip clears the crown by clearance."""
    return crown_trigger_z + clearance - STOWED_ABOVE_TIP


def lowest_safe_trigger(crown_trigger_z, radius, pin_x_from_axis):
    """The lowest commanded Z the deployed pin may reach at pin_x_from_axis without any part of
    the head (body or plate) touching a cylinder of `radius` whose crown trigger Z is given.
    Used as the floor for a probe that may miss the object."""
    def surface(dx):                                          # height below crown at offset dx
        dx = abs(dx)
        return -float("inf") if dx >= radius else -(radius - math.sqrt(radius * radius - dx * dx))
    worst = -float("inf")
    # body: underside 10.4 above the tip, over pin +/- 6.6
    for k in range(-10, 11):
        dx = pin_x_from_axis + BODY_HALF_X * k / 10.0
        worst = max(worst, surface(dx) - BODY_ABOVE_TIP)
    # plate: bottom edge 24 above the tip, from pin - 55 to pin + 22
    for k in range(0, 78):
        dx = pin_x_from_axis - PLATE_LEFT_X + k
        worst = max(worst, surface(dx) - PLATE_ABOVE_TIP)
    return crown_trigger_z + worst
