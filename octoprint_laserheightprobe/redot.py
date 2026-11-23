# coding=utf-8
"""The 10W's red autofocus dot as a height gauge.

The 10W carries a small red light (M2002 L1 / L0) aimed about 31 degrees off vertical, and the
camera beside it. The dot lands where that ray meets the surface, so its place in the photo says
how far the surface is below the head: a triangulation. Snapmaker's touchscreen does this with
two fixed heights and a formula (decompiled fabscreen PreviewViewModel), but its HTTP shortcut
ends with G54, which on this machine holds a stale Luban offset: never use it. This module does
the same with our own photos.

Measured 2026-11-21 (head at X175 Y120 over the paper, photoQuality 1, 1024 x 1280 photos):

- the dot's column follows a pinhole law, x = a + b / (Z + c - h), to 0.16 px over Z100-180
  (a 10.11, b 65708, c 3.52: Snapmaker's own code uses Z + 3.9);
- its row barely moves (642-646), so it lies on a known line: anything off it is not the dot;
- at Z100 one millimetre of height is ~6 px, so the bright-core centroid (repeats to 0.04 px on
  wood) resolves a few hundredths of a millimetre; Chris's 2.7 mm wood read 2.74 (the paper beside
  it 0.09);
- in the bed's frame the dot sits 42 mm to -X of the head at Z100 and 90 mm at Z180, on a ray
  the camera model puts through all five points within 0.04 mm.

Each reading is a light-on / light-off pair from one spot: the dot is the brightest compact rise
in the difference, then the centroid of its bright core in the light-on photo (the core is
saturated white, its red halo lopsided). The head Z the firmware holds is levelled at the head,
but the dot lands tens of mm away, so the height map's difference between the two is applied.
"""
from __future__ import absolute_import, division

import math

from .closeup_libs import CloseupError, cv as _cv, np as _np

BAND_Y = (560, 720)          # the rows the dot can occupy (it sits at 640-650)
LINE_TOL_PX = 8.0            # how far off its known row line a dot may be
MIN_RISE = 20.0              # grey levels the light must add at the dot
MIN_UNIQUE = 1.8             # dot rise over the next best rise 80 px away
AREA = (8, 4000)             # bright-core pixels


def find_dot(on, off, band_y=BAND_Y):
    """The dot in a light-on / light-off pair of BGR photos: dict(x, y, rise, unique, area).
    Raises CloseupError when there is no clear dot."""
    np, cv2 = _np(), _cv()
    if on.shape != off.shape:
        raise CloseupError("the two photos differ in size")
    a = cv2.cvtColor(on, cv2.COLOR_BGR2GRAY).astype(np.float32)
    b = cv2.cvtColor(off, cv2.COLOR_BGR2GRAY).astype(np.float32)
    raw = a - b
    # two views of the rise: lightly smoothed, and a difference of Gaussians that keeps the
    # compact dot and drops the broad glow the light throws round it (strong on wood)
    fine = cv2.GaussianBlur(raw, (0, 0), 2)
    dog = fine - cv2.GaussianBlur(raw, (0, 0), 12)

    def banded(img):
        m = np.full_like(img, -1e9)
        m[band_y[0]:band_y[1]] = img[band_y[0]:band_y[1]]
        return m

    md, mf = banded(dog), banded(fine)
    y, x = np.unravel_index(int(np.argmax(md)), md.shape)
    rise = float(mf[max(0, y - 3):y + 4, max(0, x - 3):x + 4].max())
    if rise < MIN_RISE:
        raise CloseupError("no red dot in the photo (the light added only %.0f grey levels)" % rise)

    def unique(m, peak):
        m2 = m.copy()
        cv2.circle(m2, (int(x), int(y)), 80, -1e9, -1)        # the dot's own glow reaches ~60 px on paper
        return peak / max(float(m2.max()), 1.0)

    uniq = max(unique(mf, rise), unique(md, float(md[y, x])))
    if uniq < MIN_UNIQUE:
        raise CloseupError("the red dot is not clear in the photo (it stands out only %.1f times)" % uniq)
    cx, cy, area = core(a, (x, y))
    if not AREA[0] <= area <= AREA[1]:
        raise CloseupError("the red dot's bright core is %d px, not a dot" % area)
    return dict(x=cx, y=cy, rise=rise, unique=uniq, area=area)


def core(grey, near, r=30, frac=0.85):
    """Centroid of the dot's bright core: pixels within r of `near` brighter than frac of the way
    from the local median to the local maximum, the blob holding the brightest pixel."""
    np, cv2 = _np(), _cv()
    x0, y0 = int(round(near[0])), int(round(near[1]))
    h, w = grey.shape
    xa, xb, ya, yb = max(0, x0 - r), min(w, x0 + r + 1), max(0, y0 - r), min(h, y0 + r + 1)
    win = grey[ya:yb, xa:xb]
    bg, mx = float(np.median(win)), float(win.max())
    mask = (win >= bg + frac * (mx - bg)).astype(np.uint8)
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask)
    yy, xx = np.unravel_index(int(np.argmax(win)), win.shape)
    k = lab[yy, xx]
    wgt = (win - bg) * (lab == k)
    gy, gx = np.mgrid[0:win.shape[0], 0:win.shape[1]]
    s = float(wgt.sum())
    return xa + float((wgt * gx).sum()) / s, ya + float((wgt * gy).sum()) / s, int(stats[k, cv2.CC_STAT_AREA])


class DotModel(object):
    """x = a + b / (Z + lift + c - h): the dot's column for head Z (G-code), the firmware's lift
    under the head minus the bed's height under the dot (lift), and a surface h above the bed.
    Its row lies on y = ly0 + ly1 * x. The light's ray, head-relative in the bed's frame, is
    p0 + s * d (p0 at the head's Z, d pointing down)."""

    def __init__(self, a, b, c, ly0, ly1, p0, d, z_range, residual_px=None, holdout=None, time=None):
        self.a, self.b, self.c = float(a), float(b), float(c)
        self.ly0, self.ly1 = float(ly0), float(ly1)
        self.p0 = [float(v) for v in p0]
        self.d = [float(v) for v in d]
        self.z_range = (float(z_range[0]), float(z_range[1]))
        self.residual_px = residual_px
        self.holdout = holdout
        self.time = time

    def column(self, z, h=0.0, lift=0.0):
        return self.a + self.b / (z + lift + self.c - h)

    def height(self, z, x, lift=0.0):
        if x - self.a <= 1.0:
            raise CloseupError("the red dot is outside the range it can measure")
        return z + lift + self.c - self.b / (x - self.a)

    def mm_per_px(self, z, h=0.0):
        dd = z + self.c - h
        return dd * dd / self.b

    def off_line(self, x, y):
        return abs(y - (self.ly0 + self.ly1 * x))

    def offset(self, z, h=0.0):
        """Where the dot lands, relative to the head (bed frame X, Y), on a surface at height h
        with the head at Z."""
        drop = h - z                                   # the surface, relative to the head's Z
        s = (drop - self.p0[2]) / self.d[2]
        return self.p0[0] + s * self.d[0], self.p0[1] + s * self.d[1]

    def to_dict(self):
        return dict(a=self.a, b=self.b, c=self.c, ly0=self.ly0, ly1=self.ly1, p0=self.p0, d=self.d,
                    z_range=list(self.z_range), residual_px=self.residual_px, holdout=self.holdout,
                    time=self.time)

    @classmethod
    def from_dict(cls, data):
        return cls(data["a"], data["b"], data["c"], data["ly0"], data["ly1"], data["p0"], data["d"],
                   data["z_range"], data.get("residual_px"), data.get("holdout"), data.get("time"))


def fit(samples):
    """A DotModel from photos of a flat surface (h = 0) at several head heights. samples: dicts
    with z (head G-code Z), x, y (dot px), lift (firmware lift at the head minus the bed's height
    under the dot) and rel (the dot relative to the head in the bed frame, x, y, and its drop
    below the head, negative). Needs 4 or more heights over 40 mm or more."""
    np = _np()
    if len(samples) < 4:
        raise CloseupError("the red dot needs photos at 4 or more heights to calibrate")
    z = np.array([s["z"] + s.get("lift", 0.0) for s in samples], float)
    x = np.array([s["x"] for s in samples], float)
    if z.max() - z.min() < 40:
        raise CloseupError("the red dot's calibration heights must span 40 mm or more")
    best = None
    # x = a + b / (z + c): linear in a, b for each c; a fine search over c, then refine
    for lo, hi, step in ((-60.0, 60.0, 0.5), (None, None, 0.01)):
        if lo is None:
            lo, hi = best[1] - 0.6, best[1] + 0.6
        for c in np.arange(lo, hi, step):
            if (z + c <= 1).any():
                continue
            A = np.column_stack([np.ones_like(z), 1.0 / (z + c)])
            sol = np.linalg.lstsq(A, x, rcond=None)[0]
            r = x - A.dot(sol)
            ss = float((r * r).sum())
            if best is None or ss < best[0]:
                best = (ss, float(c), sol, r)
    _, c, (a, b), r = best
    ys = np.array([s["y"] for s in samples], float)
    ly1, ly0 = np.polyfit(x, ys, 1)
    rel = np.array([s["rel"] for s in samples], float)
    centre = rel.mean(axis=0)
    _, _, vt = np.linalg.svd(rel - centre)
    d = vt[0] if vt[0][2] < 0 else -vt[0]
    s0 = (0.0 - centre[2]) / d[2]
    p0 = centre + s0 * d                            # the ray where it is level with the head
    zs = [s["z"] for s in samples]
    return DotModel(a, b, c, ly0, ly1, p0, d, (min(zs), max(zs)),
                    residual_px=float(np.abs(r).max()))
