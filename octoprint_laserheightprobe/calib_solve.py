# coding=utf-8
"""Calibrating the 10W head camera from photos taken at known head positions.

The camera only translates (camera_model.py), so every photo of a flat target is fronto-parallel
and cv2.calibrateCamera cannot separate focal length from distance. Here the head position of
every photo is known from M114:
- moving in Z separates f from e (px/mm = f / (Z + e));
- moving in X/Y fixes f absolutely and the camera's yaw;
- the printed board's scale s stays a free parameter, so printer scaling does no harm.

Stage A uses a printed ChArUco board (OpenCV DICT_4X4_250, 13 x 18 squares of 15 mm). It fits f,
cx, cy, k1, k2, R and e, plus the board's pose on the bed (bx, by, psi) and scale s. t stays 0
because the board's position absorbs it.

Stage B uses crosses the beam burned at known LightBurn positions. Their B-frame position is
LB + o_burn, the bridge offset in force when they were burned. It fits t, the camera centre
relative to the head in the beam frame.

The solver is a small Levenberg-Marquardt with a numeric Jacobian, Huber reweighting and priors.
"""
from __future__ import absolute_import

import copy

from .camera_model import CameraModel
from .closeup_libs import CloseupError, np as _np

# 13 x 18 squares of 15 mm (195 x 270 mm) fits A4 inside printer margins.
BOARD = dict(squares=(13, 18), square=15.0, marker=11.0, dictionary="DICT_4X4_250")
STAGE_A = ("f", "aspect", "cx", "cy", "k1", "k2", "r0", "r1", "r2", "e", "bx", "by", "psi", "s")
STEPS_A = (1e-3, 1e-7, 1e-3, 1e-3, 1e-6, 1e-6, 1e-7, 1e-7, 1e-7, 1e-4, 1e-4, 1e-4, 1e-7, 1e-7)
TILT_PRIOR = 0.0175        # rad (1 degree): a tilt and a principal point shift look alike
HUBER_PX = 1.0
TANGLED = 0.95             # |corr(f, s)| above this: f and the board scale are not separated (a good fit: about 0.2)
GROSS_MM = 0.5             # a held-out corner further out than this (or 5x the median) is a misread


# -- the board -------------------------------------------------------------------------------
def board_corners(squares=BOARD["squares"], square=BOARD["square"]):
    """ChArUco inner-corner ids and their board positions (mm, x right, y up from the top-left),
    in OpenCV's order: id = row * (squares_x - 1) + col."""
    np = _np()
    nx, ny = squares[0] - 1, squares[1] - 1
    ids = list(range(nx * ny))
    xy = np.array([[(i % nx + 1) * square, -(i // nx + 1) * square] for i in ids], float)
    return ids, xy


def board_to_world(bxy, pose):
    np = _np()
    c, s = np.cos(pose["psi"]), np.sin(pose["psi"])
    rot = np.array([[c, -s], [s, c]]) * pose["s"]
    return np.asarray(bxy, float).dot(rot.T) + np.array([pose["bx"], pose["by"]])


def fit_similarity(src, dst):
    """(bx, by, psi, s) with dst = s R(psi) src + (bx, by), least squares (no reflection)."""
    np = _np()
    a, b = np.asarray(src, float), np.asarray(dst, float)
    ma, mb = a.mean(0), b.mean(0)
    a0, b0 = a - ma, b - mb
    cov = b0.T.dot(a0) / len(a)
    u, d, vt = np.linalg.svd(cov)
    sign = np.sign(np.linalg.det(u.dot(vt)))
    dd = np.diag([1.0, sign])
    rot = u.dot(dd).dot(vt)
    s = np.trace(np.diag(d).dot(dd)) / (a0 ** 2).sum(1).mean()
    t = mb - s * rot.dot(ma)
    return dict(bx=float(t[0]), by=float(t[1]), psi=float(np.arctan2(rot[1, 0], rot[0, 0])), s=float(s))


def init_board(obs, model):
    """The board's pose from the highest photo, read through a nominal camera."""
    best = max(obs, key=lambda o: (o["head"][2], len(o["ids"])))
    _, bxy = board_corners()
    plane = model.to_plane(best["head"], best["px"], 0.0)
    return fit_similarity(bxy[best["ids"]], plane)


# -- the solver ------------------------------------------------------------------------------
def lm(fun, x0, steps, priors=(), huber=None, max_iter=100, tol=1e-12):
    """Levenberg-Marquardt on fun(x) -> residual vector. priors: (index, mean, sigma) rows.
    huber: residuals above it are down-weighted (IRLS, weights fixed within an iteration)."""
    np = _np()
    x = np.asarray(x0, float).copy()
    steps = np.asarray(steps, float)

    def weights(r):
        if not huber:
            return np.ones_like(r)
        a = np.abs(r)
        return np.where(a <= huber, 1.0, np.sqrt(huber / np.maximum(a, 1e-30)))

    def full(x, w):
        r = fun(x) * w
        if priors:
            r = np.concatenate([r, [(x[i] - m) / s for i, m, s in priors]])
        return r

    w = weights(fun(x))
    r = full(x, w)
    cost = r.dot(r)
    lam, it = 1e-3, 0
    for it in range(1, max_iter + 1):
        jac = np.empty((len(r), len(x)))
        for k in range(len(x)):
            dx = np.zeros_like(x)
            dx[k] = steps[k]
            jac[:, k] = (full(x + dx, w) - full(x - dx, w)) / (2 * steps[k])
        jtj, jtr = jac.T.dot(jac), jac.T.dot(r)
        improved = False
        for _ in range(12):
            a = jtj + lam * np.diag(np.diag(jtj) + 1e-12)
            try:
                step = -np.linalg.solve(a, jtr)
            except np.linalg.LinAlgError:
                step = -np.linalg.lstsq(a, jtr, rcond=None)[0]
            xn = x + step
            rn = full(xn, w)
            cn = rn.dot(rn)
            if cn < cost:
                improved = True
                small = cost - cn <= tol * max(cost, 1e-30) or np.all(np.abs(step) <= steps * 1e-3)
                x, cost, lam = xn, cn, max(lam / 3.0, 1e-12)
                break
            lam *= 4.0
        w = weights(fun(x))
        r = full(x, w)
        cost = r.dot(r)
        if not improved or small:
            break
    dof = max(len(r) - len(x), 1)
    cov = np.linalg.pinv(jtj) * (cost / dof)
    return dict(x=x, cov=cov, cost=cost, iterations=it, residual=fun(x))


def _corr(cov, i, j):
    np = _np()
    d = np.sqrt(abs(cov[i, i] * cov[j, j]))
    return float(cov[i, j] / d) if d > 0 else 0.0


def _model_a(x, base):
    m = copy.deepcopy(base)
    m.f, m.aspect, m.cx, m.cy, m.k1, m.k2 = x[0], x[1], x[2], x[3], x[4], x[5]
    m.rvec = (x[6], x[7], x[8])
    m.e = x[9]
    m.t = (0.0, 0.0)
    return m


def _stack(obs):
    np = _np()
    _, bxy = board_corners()
    heads, pts, px, which = [], [], [], []
    for n, o in enumerate(obs):
        k = len(o["ids"])
        heads.append(np.tile(np.asarray(o["head"], float), (k, 1)))
        pts.append(bxy[o["ids"]])
        px.append(np.asarray(o["px"], float))
        which.append(np.full(k, n))
    return np.vstack(heads), np.vstack(pts), np.vstack(px), np.concatenate(which)


def _heights(surface, xy, t_hint):
    np = _np()
    if surface is None:
        return np.zeros(len(xy))
    return np.asarray(surface(xy[:, 0] + t_hint[0], xy[:, 1] + t_hint[1]), float)


def _plane_errors(model, pose, obs, surface=None, t_hint=(0.0, 0.0)):
    """Per photo: the mm distances on the bed between where each corner is and where the model
    reads it from the photo (each corner at its own height when the bed's surface is given)."""
    np = _np()
    _, bxy = board_corners()
    out = []
    for o in obs:
        world = board_to_world(bxy[o["ids"]], pose)
        seen = model.to_plane(o["head"], o["px"], _heights(surface, world, t_hint))
        out.append(np.linalg.norm(seen - world, axis=1))
    return out


def solve_board(obs, model0, board0=None, surface=None, t_hint=(0.0, 0.0)):
    """Stage A. obs: [{head: (x, y, z), ids: [...], px: [[u, v], ...], holdout: bool}].
    surface(x, y), B frame: the bed's height under the board (it is ~1.8 mm uneven), with
    t_hint the camera offset guess that turns the fit's camera-centred points into the B frame.
    Returns (model, board pose, report)."""
    np = _np()
    train = [o for o in obs if not o.get("holdout")]
    held = [o for o in obs if o.get("holdout")]
    if len(train) < 6:
        raise CloseupError("only %d calibration photos found the board: need at least 6" % len(train))
    pose0 = board0 or init_board(obs, model0)
    heads, bxy, px, _ = _stack(train)
    zeros = np.zeros(len(bxy))

    def resid(x):
        m = _model_a(x, model0)
        pose = dict(bx=x[10], by=x[11], psi=x[12], s=x[13])
        flat = board_to_world(bxy, pose)
        world = np.column_stack([flat, _heights(surface, flat, t_hint) if surface is not None else zeros])
        return (m.project_many(heads, world) - px).ravel()

    x0 = np.array([model0.f, model0.aspect, model0.cx, model0.cy, model0.k1, model0.k2] + list(model0.rvec) +
                  [model0.e, pose0["bx"], pose0["by"], pose0["psi"], pose0["s"]], float)
    out = lm(resid, x0, STEPS_A, priors=[(6, 0.0, TILT_PRIOR), (7, 0.0, TILT_PRIOR)], huber=HUBER_PX, max_iter=300)
    x = out["x"]
    model = _model_a(x, model0)
    zs = [o["head"][2] for o in obs]
    model.z_range = (float(min(zs)), float(max(zs)))
    pose = dict(bx=float(x[10]), by=float(x[11]), psi=float(x[12]), s=float(x[13]))
    r = out["residual"].reshape(-1, 2)
    fit_rms = float(np.sqrt((r ** 2).sum(1).mean()))
    held_all = np.concatenate(_plane_errors(model, pose, held, surface, t_hint)) if held else np.zeros(0)
    # a wild corner detection (a misread marker) is counted, not averaged in
    cut = max(GROSS_MM, 5.0 * float(np.median(held_all))) if held_all.size else GROSS_MM
    held_err = held_all[held_all <= cut]
    cov = out["cov"]
    sd = dict((name, float(np.sqrt(abs(cov[i, i])))) for i, name in enumerate(STAGE_A))
    corr = dict(f_s=_corr(cov, 0, 13), f_e=_corr(cov, 0, 9), cx_r1=_corr(cov, 2, 7), cy_r0=_corr(cov, 3, 6),
                f_aspect=_corr(cov, 0, 1))
    warnings = []
    if abs(corr["f_s"]) > TANGLED:
        warnings.append("focal length and board scale are tangled (%.3f): the photos need sideways "
                        "head moves" % corr["f_s"])
    report = dict(
        photos=len(train), holdout_photos=len(held), points=int(len(bxy)), iterations=out["iterations"],
        fit_rms_px=fit_rms,
        holdout_rms_mm=float(np.sqrt((held_err ** 2).mean())) if held_err.size else None,
        holdout_max_mm=float(held_err.max()) if held_err.size else None,
        holdout_rejected=int(held_all.size - held_err.size),
        sd=sd, corr=corr, warnings=warnings, board=pose)
    model.residuals = dict(stage_a=dict((k, report[k]) for k in (
        "photos", "holdout_photos", "fit_rms_px", "holdout_rms_mm", "holdout_max_mm")))
    return model, pose, report


def _tie_fit(model, obs, o_burn, surface=None):
    np = _np()
    heads = np.array([o["head"] for o in obs], float)
    px = np.array([o["px"] for o in obs], float)
    world = np.array([[o["lb"][0] + o_burn[0], o["lb"][1] + o_burn[1], 0.0] for o in obs])
    world[:, 2] = _heights(surface, world[:, :2], (0.0, 0.0))

    def resid(t):
        m = copy.deepcopy(model)
        m.t = (t[0], t[1])
        return (m.project_many(heads, world) - px).ravel()

    out = lm(resid, np.array(model.t, float), np.array([1e-4, 1e-4]), huber=HUBER_PX)
    m = copy.deepcopy(model)
    m.t = (float(out["x"][0]), float(out["x"][1]))
    return m


def _cross_errors(model, obs, o_burn, surface=None):
    np = _np()
    out = []
    for o in obs:
        b = np.array([[o["lb"][0] + o_burn[0], o["lb"][1] + o_burn[1]]])
        seen = model.to_plane(o["head"], [o["px"]], _heights(surface, b, (0.0, 0.0)))[0]
        out.append(float(np.hypot(seen[0] - o["lb"][0] - o_burn[0], seen[1] - o["lb"][1] - o_burn[1])))
    return out


def solve_tie(model, obs, o_burn, surface=None):
    """Stage B. obs: [{head, px: [u, v], lb: (X, Y)}] for burned crosses at LightBurn (X, Y),
    burned with the bridge offset o_burn; head Z as the camera really was (with any height map
    lift added). surface(x, y): the bed's height at each cross. Returns (model with t and
    o_burn, report)."""
    np = _np()
    if len(obs) < 3:
        raise CloseupError("need at least 3 cross photos to tie the camera to the beam")
    fit = _tie_fit(model, obs, o_burn, surface)
    fit.o_burn = tuple(float(v) for v in o_burn)
    errs = _cross_errors(fit, obs, o_burn, surface)
    keys = sorted(set(tuple(o["lb"]) for o in obs))
    crosses, loo = [], []
    for key in keys:
        mine = [o for o in obs if tuple(o["lb"]) == key]
        rest = [o for o in obs if tuple(o["lb"]) != key]
        own = _cross_errors(fit, mine, o_burn, surface)
        crosses.append(dict(lb=list(key), photos=len(mine), mean_mm=float(np.mean(own))))
        if len(rest) >= 3:
            loo += _cross_errors(_tie_fit(model, rest, o_burn, surface), mine, o_burn, surface)
    report = dict(t=list(fit.t), rms_mm=float(np.sqrt(np.mean(np.square(errs)))), max_mm=float(max(errs)),
                  loo_max_mm=float(max(loo)) if loo else None, crosses=crosses, photos=len(obs))
    residuals = dict(fit.residuals or {})
    residuals["stage_b"] = dict((k, report[k]) for k in ("rms_mm", "max_mm", "loo_max_mm", "photos"))
    fit.residuals = residuals
    return fit, report
