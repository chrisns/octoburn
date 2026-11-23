# coding=utf-8
"""The fixed USB bed camera, as the close-up image's base (setting closeup_base = "usb").

The camera hangs above the bed, served by go2rtc as a JPEG snapshot (bedcam_url). The head does
not carry it, and the bed moves in Y under it, so a photo lines up with LightBurn coordinates
only with the bed at the Y the camera was calibrated at (bedcam_park_y, 300 by default, where
LightBurn's own "Bed cam - fixed" camera was aligned). The plugin parks there before every
photo and refuses a photo or a calibration taken at any other Y.

The model:
- a lens: camera matrix K and OpenCV's distortion terms (k1, k2, p1, p2, k3, k4, k5, k6, s1, s2,
  s3, s4; LightBurn stores 12), or none;
- H, a homography from undistorted pixels (undistorted with P = K, so they keep the photo's
  scale) to LightBurn mm on the bed.

A canvas pixel is placed by a forward map: LightBurn mm -> H^-1 -> undistorted pixel -> lens ->
photo pixel, then one interpolation (compositor.bedcam_canvas).

Two calibrations (both kept as a candidate until accepted):
- from LightBurn's prefs (from_lightburn): LensCalibrations and ImageAlignments.ImageProjection
  of its camera. LightBurn does not say which frame its projection works in; undistorted with
  P = K is assumed, and on 2026-11-21 that read the 140 mm between two burned crosses as about
  132 mm, so treat it as a starting guess and check it against the crosses;
- from burned crosses (fit): crosses found in a photo with the bed at the park Y, undistorted
  with the lens, and H fitted by least squares. Six crosses give four spare equations; the
  residuals say how well the result holds.

numpy only (lazily, from closeup_libs); the OpenCV calls are in cvio.py.
"""
from __future__ import absolute_import

import time

from .closeup_libs import CloseupError, np as _np

# The burned crosses on the paper (16 mm, LightBurn mm): the close-up calibration's four and two more.
CROSSES_LB = ((60.0, 100.0), (200.0, 100.0), (60.0, 250.0), (200.0, 250.0), (80.0, 120.0), (180.0, 230.0))
PX_PER_MM = 2.0                 # the bed camera sees about 2 px/mm at 1280 x 720
N_DIST = 12
FIELDS = ("K", "dist", "H", "width", "height", "bed_y", "source", "created", "residuals", "name")


def _mat(m):
    np = _np()
    a = np.asarray(m, float)
    if a.size != 9:
        raise CloseupError("a 3 x 3 matrix needs 9 numbers (got %d)" % a.size)
    return a.reshape(3, 3)


def column_major(values):
    """A 3 x 3 matrix from 9 numbers stored column-major, as LightBurn's prefs hold them."""
    return _mat(values).T


def distort(dist, x, y):
    """OpenCV's lens model on normalised coordinates x, y (arrays): rational radial terms,
    tangential terms and thin prism terms, as many as dist holds (up to 12)."""
    np = _np()
    d = [float(v) for v in (dist or [])]
    if len(d) > N_DIST and any(d[N_DIST:]):
        raise CloseupError("a lens with tilt terms (14 distortion numbers) is not supported")
    d = (d + [0.0] * N_DIST)[:N_DIST]
    k1, k2, p1, p2, k3, k4, k5, k6, s1, s2, s3, s4 = d
    r2 = x * x + y * y
    r4 = r2 * r2
    radial = (1.0 + k1 * r2 + k2 * r4 + k3 * r4 * r2) / (1.0 + k4 * r2 + k5 * r4 + k6 * r4 * r2)
    xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x) + s1 * r2 + s2 * r4
    yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y + s3 * r2 + s4 * r4
    return np.asarray(xd), np.asarray(yd)


def fit_homography(src, dst):
    """The 3 x 3 homography taking points src (N, 2) to dst (N, 2), N >= 4: the normalised
    direct linear transform, least squares over all points."""
    np = _np()
    src, dst = np.asarray(src, float), np.asarray(dst, float)
    if len(src) < 4 or len(src) != len(dst):
        raise CloseupError("a homography needs at least 4 point pairs (got %d)" % len(src))

    def norm(p):
        c = p.mean(0)
        s = np.sqrt(2.0) / max(np.mean(np.hypot(*(p - c).T)), 1e-12)
        return np.array([[s, 0, -s * c[0]], [0, s, -s * c[1]], [0, 0, 1.0]])

    ts, td = norm(src), norm(dst)
    a = []
    for (x, y), (u, v) in zip(_apply(ts, src), _apply(td, dst)):
        a.append([-x, -y, -1, 0, 0, 0, u * x, u * y, u])
        a.append([0, 0, 0, -x, -y, -1, v * x, v * y, v])
    _, _, vt = np.linalg.svd(np.asarray(a))
    h = vt[-1].reshape(3, 3)
    h = np.linalg.inv(td).dot(h).dot(ts)
    return h / h[2, 2]


def _apply(h, pts):
    np = _np()
    p = np.column_stack([np.asarray(pts, float), np.ones(len(pts))]).dot(np.asarray(h).T)
    return p[:, :2] / p[:, 2:3]


class BedcamModel(object):
    def __init__(self, H, width, height, bed_y, K=None, dist=None, source="", created=None, residuals=None,
                 name=""):
        np = _np()
        self.H = _mat(H)
        self.K = None if K is None else _mat(K)
        self.dist = [float(v) for v in (dist or [])]
        self.width, self.height = int(width), int(height)
        self.bed_y = float(bed_y)                   # the bed's Y (head Y, M114) the calibration holds for
        self.source = str(source or "")
        self.created = created if created is not None else time.time()
        self.residuals = residuals
        self.name = str(name or "")
        self._hinv = np.linalg.inv(self.H)
        self._r2_max = None

    # -- geometry ------------------------------------------------------------------------------
    def _undist_to_norm(self, u, v):
        k = self.K
        yn = (v - k[1, 2]) / k[1, 1]
        xn = (u - k[0, 2] - k[0, 1] * yn) / k[0, 0]
        return xn, yn

    def _norm_to_px(self, xn, yn):
        k = self.K
        return k[0, 0] * xn + k[0, 1] * yn + k[0, 2], k[1, 1] * yn + k[1, 2]

    def r2_max(self):
        """The largest normalised radius squared the photo reaches. Past it the lens model is an
        extrapolation that can fold back into the photo (LightBurn's 12-term fit does), so a
        point beyond it is outside the photo whatever the model says."""
        if self._r2_max is None:
            np = _np()
            from . import cvio
            n = 64
            us, vs = np.linspace(0, self.width - 1, n), np.linspace(0, self.height - 1, n)
            edge = np.concatenate([np.column_stack([us, np.zeros(n)]), np.column_stack([us, np.full(n, self.height - 1.0)]),
                                   np.column_stack([np.zeros(n), vs]), np.column_stack([np.full(n, self.width - 1.0), vs])])
            norm = cvio.undistort_points(edge, self.K, self.dist)
            self._r2_max = float(np.max(np.sum(norm * norm, 1)))
        return self._r2_max

    def lb_to_px(self, xs, ys):
        """Photo pixels (u, v) of LightBurn points xs, ys (arrays, same shape) on the bed, and a
        mask of the ones the model can place (in front of the camera, inside the lens's reach)."""
        np = _np()
        xs, ys = np.asarray(xs, float), np.asarray(ys, float)
        hi = self._hinv
        w = hi[2, 0] * xs + hi[2, 1] * ys + hi[2, 2]
        u = (hi[0, 0] * xs + hi[0, 1] * ys + hi[0, 2]) / w
        v = (hi[1, 0] * xs + hi[1, 1] * ys + hi[1, 2]) / w
        # the side of the horizon the photo's own middle is on
        mid = self.H.dot([self.width / 2.0, self.height / 2.0, 1.0])
        sign = np.sign(1.0 / mid[2])
        ok = w * sign > 0
        if self.K is not None and any(self.dist):
            xn, yn = self._undist_to_norm(u, v)
            ok &= xn * xn + yn * yn <= self.r2_max()
            xd, yd = distort(self.dist, xn, yn)
            u, v = self._norm_to_px(xd, yd)
        return u, v, ok

    def px_to_lb(self, px):
        """LightBurn mm (N, 2) of photo pixels (N, 2)."""
        np = _np()
        p = np.atleast_2d(np.asarray(px, float))
        if self.K is not None and any(self.dist):
            from . import cvio
            p = cvio.undistort_points(p, self.K, self.dist, P=self.K)
        return _apply(self.H, p)

    # -- storage -------------------------------------------------------------------------------
    def to_dict(self):
        return dict(H=self.H.tolist(), K=None if self.K is None else self.K.tolist(), dist=list(self.dist),
                    width=self.width, height=self.height, bed_y=self.bed_y, source=self.source,
                    created=self.created, residuals=self.residuals, name=self.name)

    @classmethod
    def from_dict(cls, data):
        return cls(**dict((k, data.get(k)) for k in FIELDS if data.get(k) is not None))


def from_lightburn(camera, bed_y):
    """A model from one of LightBurn's Cameras[] entries (prefs.ini, DeviceList[n].Settings.
    Cameras[]): its selected lens calibration and image alignment. bed_y: where the bed was
    when LightBurn was aligned (300 for "Bed cam - fixed")."""
    if not isinstance(camera, dict):
        raise CloseupError("the LightBurn camera must be one Cameras[] entry (a JSON object)")

    def pick(items, key, name):
        items = list(items or [])
        if not items:
            raise CloseupError("the LightBurn camera has no %s" % key)
        for item in items:
            if item.get(key) == name:
                return item
        return items[0]

    align = pick(camera.get("ImageAlignments"), "AlignmentName", camera.get("SelectedAlignmentName"))
    lens = None
    if camera.get("LensCalibrations"):
        lens = pick(camera.get("LensCalibrations"), "CalibrationName",
                    align.get("CalibrationName") or camera.get("SelectedCalibrationName"))
    if align.get("MaterialHeight"):
        raise CloseupError("the LightBurn alignment is for %s mm material: use one made at 0 mm"
                           % align.get("MaterialHeight"))
    if lens is not None and lens.get("IsFisheye"):
        raise CloseupError("the LightBurn lens calibration is a fisheye one, which is not supported")
    width, height = int(align.get("ImageSizeX") or 0), int(align.get("ImageSizeY") or 0)
    if lens is not None and (int(lens.get("CameraSizeX") or width), int(lens.get("CameraSizeY") or height)) != (width, height):
        raise CloseupError("the LightBurn lens calibration and alignment are for different image sizes")
    if width <= 0 or height <= 0:
        raise CloseupError("the LightBurn alignment has no image size")
    return BedcamModel(H=column_major(align["ImageProjection"]), width=width, height=height, bed_y=bed_y,
                       K=None if lens is None else column_major(lens["CameraMatrix"]),
                       dist=None if lens is None else lens.get("DistortionMatrix"),
                       source="lightburn (unverified: check it against the crosses)",
                       name="%s / %s" % (camera.get("DisplayName", ""), align.get("AlignmentName", "")))


def find_lightburn_camera(prefs, name):
    """The Cameras[] entry called name in LightBurn's prefs (the parsed JSON of prefs.ini)."""
    for device in (prefs or {}).get("DeviceList") or []:
        for camera in ((device or {}).get("Settings") or {}).get("Cameras") or []:
            if camera.get("DisplayName") == name:
                return camera
    raise CloseupError("no camera called %r in the LightBurn prefs" % name)


def check(model, crosses, found):
    """Predicted against found for each cross: [{lb, found_px, found_lb, err_mm}] (None where
    a cross was not found), with the largest and RMS error in mm."""
    np = _np()
    rows, errs = [], []
    for lb, px in zip(crosses, found):
        if px is None:
            rows.append(dict(lb=list(lb), found_px=None))
            continue
        at = model.px_to_lb([px[:2]])[0]
        err = float(np.hypot(at[0] - lb[0], at[1] - lb[1]))
        errs.append(err)
        rows.append(dict(lb=list(lb), found_px=[round(float(px[0]), 2), round(float(px[1]), 2)],
                         found_lb=[round(float(at[0]), 2), round(float(at[1]), 2)], err_mm=round(err, 2),
                         d_mm=[round(float(at[0] - lb[0]), 2), round(float(at[1] - lb[1]), 2)]))
    return dict(crosses=rows, max_mm=round(max(errs), 3) if errs else None,
                rms_mm=round(float(np.sqrt(np.mean(np.square(errs)))), 3) if errs else None, n=len(errs))


def fit(crosses, found, width, height, bed_y, lens=None):
    """A model with H fitted to the found crosses (photo pixels; None for one not found),
    keeping lens = (K, dist) if given. Returns (model, report), the report as check() gives
    it for the new model (with 4 crosses the fit is exact and says nothing)."""
    np = _np()
    pairs = [(lb, px) for lb, px in zip(crosses, found) if px is not None]
    if len(pairs) < 4:
        raise CloseupError("only %d of the crosses were found: at least 4 are needed" % len(pairs))
    lbs = np.array([p[0] for p in pairs], float)
    pxs = np.array([p[1][:2] for p in pairs], float)
    K, dist = (None, None) if lens is None else lens
    if K is not None and dist is not None and any(dist):
        from . import cvio
        und = cvio.undistort_points(pxs, K, dist, P=K)
    else:
        und = pxs
    model = BedcamModel(H=fit_homography(und, lbs), width=width, height=height, bed_y=bed_y, K=K, dist=dist,
                        source="crosses (%d%s)" % (len(pairs), ", lens from the previous calibration" if K is not None else ", no lens"))
    report = check(model, crosses, found)
    report["spare"] = 2 * len(pairs) - 8
    model.residuals = dict(max_mm=report["max_mm"], rms_mm=report["rms_mm"], n=report["n"], spare=report["spare"])
    return model, report
