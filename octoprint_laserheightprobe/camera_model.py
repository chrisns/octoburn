# coding=utf-8
"""The 10W head camera's geometry: where a bed point lands in the photo, and back.

Frames:
- H, the head: machine X, Y, Z from M114.
- B, the beam frame: for a bed point, the head XY that puts the beam on it. LightBurn's
  coordinates are B - o, where o is the bridge's offset (bridge_offset + the module's dx, dy).
  The model is kept camera-to-beam because it is tied to crosses the beam burned.
- h, height above the bare bed.

The camera only translates with the head. Its centre is C = (Hx + tx, Hy + ty, Z + e - d_bed),
where d_bed is how far the bed reference (flat_bed_z) has moved since calibration. It looks down
with a fixed orientation R = Rodrigues(rvec) . diag(1, -1, -1): image right is +X and image up is
+Y (the back of the bed). The lens is a pinhole with focal length f across and f * aspect down
(the photos' pixels are not square: aspect 0.9385, measured 2026-11-20), a principal point
(cx, cy) and radial distortion k1, k2.

Measured on the A350 (2026-11-20):
- px/mm is about 814 / (Z + 5), so f is about 814 and e about 5.
- The view drifts about 7 mm in Y between Z200 and Z40, mostly from the principal point.
- The camera looks about 57 mm to the -X side of the beam.
"""
from __future__ import absolute_import

import time

from .closeup_libs import CloseupError, np as _np

WIDTH, HEIGHT = 1024, 1280
FIELDS = ("width", "height", "f", "aspect", "cx", "cy", "k1", "k2", "rvec", "t", "e", "bed_ref", "o_burn",
          "z_range", "residuals", "created", "surface_ref")


def rodrigues(rvec):
    np = _np()
    r = np.asarray(rvec, float)
    theta = float(np.linalg.norm(r))
    if theta < 1e-15:
        return np.eye(3)
    k = r / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(theta) * kx + (1 - np.cos(theta)) * kx.dot(kx)


class CameraModel(object):
    def __init__(self, f=814.0, aspect=1.0, cx=WIDTH / 2.0, cy=HEIGHT / 2.0, k1=0.0, k2=0.0, rvec=(0.0, 0.0, 0.0),
                 t=(0.0, 0.0), e=5.0, width=WIDTH, height=HEIGHT, bed_ref=None, o_burn=None,
                 z_range=None, residuals=None, created=None, surface_ref=None):
        self.f, self.cx, self.cy, self.k1, self.k2 = float(f), float(cx), float(cy), float(k1), float(k2)
        self.aspect = float(aspect)              # fy / fx: the photos' pixels are not square
        self.rvec = tuple(float(v) for v in rvec)
        self.t = tuple(float(v) for v in t)
        self.e = float(e)
        self.width, self.height = int(width), int(height)
        self.bed_ref = None if bed_ref is None else float(bed_ref)
        self.o_burn = None if o_burn is None else tuple(float(v) for v in o_burn)
        self.z_range = None if z_range is None else tuple(float(v) for v in z_range)
        self.residuals = residuals
        self.created = created if created is not None else time.time()
        # the height map's value where the calibration board lay (calibrated with no map loaded):
        # surface heights for this camera are map - surface_ref
        self.surface_ref = None if surface_ref is None else float(surface_ref)

    # -- geometry ------------------------------------------------------------------------------
    def rotation(self):
        np = _np()
        return rodrigues(self.rvec).dot(np.diag([1.0, -1.0, -1.0]))

    def centre(self, head, d_bed=0.0):
        np = _np()
        return np.array([head[0] + self.t[0], head[1] + self.t[1], head[2] + self.e - d_bed])

    def project(self, head, pts, d_bed=0.0):
        """Pixels (N, 2) of B-frame points (N, 3) = (X, Y, h) seen from head (Hx, Hy, Z)."""
        np = _np()
        p = np.atleast_2d(np.asarray(pts, float))
        return self.project_many(np.broadcast_to(np.asarray(head, float), (len(p), 3)), p, d_bed)

    def project_many(self, heads, pts, d_bed=0.0):
        """project() with a head position per point (N, 3): a whole calibration set at once."""
        np = _np()
        p = np.atleast_2d(np.asarray(pts, float))
        hd = np.asarray(heads, float)
        c = np.column_stack([hd[:, 0] + self.t[0], hd[:, 1] + self.t[1], hd[:, 2] + self.e - d_bed])
        xc = (p - c).dot(self.rotation().T)
        if (xc[:, 2] <= 0).any():
            raise CloseupError("a point is level with or above the camera")
        x, y = xc[:, 0] / xc[:, 2], xc[:, 1] / xc[:, 2]
        r2 = x * x + y * y
        s = 1.0 + self.k1 * r2 + self.k2 * r2 * r2
        return np.column_stack([self.f * s * x + self.cx, self.f * self.aspect * s * y + self.cy])

    def to_plane(self, head, px, h, d_bed=0.0):
        """B-frame X, Y (N, 2) on the plane at height h of pixels px (N, 2)."""
        np = _np()
        q = np.atleast_2d(np.asarray(px, float))
        xd, yd = (q[:, 0] - self.cx) / self.f, (q[:, 1] - self.cy) / (self.f * self.aspect)
        x, y = xd.copy(), yd.copy()
        for _ in range(30):                      # fixed point: converges fast for a mild lens
            r2 = x * x + y * y
            s = 1.0 + self.k1 * r2 + self.k2 * r2 * r2
            nx, ny = xd / s, yd / s
            done = np.abs(nx - x).max() < 1e-13 and np.abs(ny - y).max() < 1e-13
            x, y = nx, ny
            if done:
                break
        rays = np.column_stack([x, y, np.ones_like(x)]).dot(self.rotation())    # R^T applied per row
        c = self.centre(head, d_bed)
        h = np.asarray(h, float)
        if (c[2] - h <= 0).any():
            raise CloseupError("the plane at height %.1f is level with or above the camera" % float(np.max(h)))
        lam = (h - c[2]) / rays[:, 2]
        return c[:2] + rays[:, :2] * lam[:, None]

    def nadir(self, head, h, d_bed=0.0):
        """Where the principal ray meets the plane at height h."""
        p = self.to_plane(head, [[self.cx, self.cy]], h, d_bed)[0]
        return float(p[0]), float(p[1])

    def footprint(self, z, h, margin=0, d_bed=0.0):
        """(x0, y0, x1, y1): the biggest axis-aligned rectangle on the plane at height h, relative
        to the head XY, that the photo from head height z sees with every point at least margin
        pixels inside its edges (one number, or (left, top, right, bottom): the head's shadow
        takes a band on the photo's right)."""
        np = _np()
        head = (0.0, 0.0, z)
        ml, mt, mr, mb = (margin, margin, margin, margin) if np.ndim(margin) == 0 else margin
        n = 257
        us, vs = np.linspace(ml, self.width - mr, n), np.linspace(mt, self.height - mb, n)
        left = self.to_plane(head, np.column_stack([np.full(n, ml), vs]), h, d_bed)
        right = self.to_plane(head, np.column_stack([np.full(n, self.width - mr), vs]), h, d_bed)
        top = self.to_plane(head, np.column_stack([us, np.full(n, mt)]), h, d_bed)
        bottom = self.to_plane(head, np.column_stack([us, np.full(n, self.height - mb)]), h, d_bed)
        x0, x1 = left[:, 0].max(), right[:, 0].min()
        y0, y1 = bottom[:, 1].max(), top[:, 1].min()
        for _ in range(8):                       # only the parts of each edge beside the rectangle count
            x0 = _edge_extreme(left, 1, 0, y0, y1, max)
            x1 = _edge_extreme(right, 1, 0, y0, y1, min)
            y0 = _edge_extreme(bottom, 0, 1, x0, x1, max)
            y1 = _edge_extreme(top, 0, 1, x0, x1, min)
        eps = 1e-6
        return float(x0 + eps), float(y0 + eps), float(x1 - eps), float(y1 - eps)

    def centre_offset(self, z, h, margin=0, d_bed=0.0):
        """The middle of the footprint, relative to the head XY: head = target - this."""
        x0, y0, x1, y1 = self.footprint(z, h, margin, d_bed)
        return (x0 + x1) / 2.0, (y0 + y1) / 2.0

    def pixel_maps(self, head, h, xs, ys, d_bed=0.0):
        """float32 maps (u, v) of the B-frame grid xs, ys (same shape), for cv2.remap. h is one
        height or one per grid point (an uneven bed)."""
        np = _np()
        xs, ys = np.asarray(xs, float), np.asarray(ys, float)
        hs = np.broadcast_to(np.asarray(h, float), xs.shape).ravel()
        px = self.project(head, np.column_stack([xs.ravel(), ys.ravel(), hs]), d_bed)
        return px[:, 0].reshape(xs.shape).astype(np.float32), px[:, 1].reshape(xs.shape).astype(np.float32)

    # -- storage -------------------------------------------------------------------------------
    def to_dict(self):
        out = {}
        for name in FIELDS:
            value = getattr(self, name)
            out[name] = list(value) if isinstance(value, tuple) else value
        return out

    @classmethod
    def from_dict(cls, data):
        return cls(**dict((name, data.get(name)) for name in FIELDS if data.get(name) is not None))


def _edge_extreme(edge, along, across, lo, hi, pick):
    """pick (max or min) of edge[:, across] over the points with edge[:, along] in [lo, hi],
    including the edge's values interpolated exactly at lo and hi."""
    np = _np()
    a, c = edge[:, along], edge[:, across]
    order = np.argsort(a)
    a, c = a[order], c[order]
    inside = c[(a >= lo) & (a <= hi)]
    ends = np.interp([lo, hi], a, c)
    return pick(list(inside) + list(ends))
