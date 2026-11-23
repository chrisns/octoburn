"""Calibrating the head camera from photos taken at known head positions.

The camera only translates, so every photo of the flat printed board is fronto-parallel and
cv2.calibrateCamera alone cannot separate focal length from distance. Here the head positions
are known exactly (M114): moving in Z separates f from e, moving in X/Y fixes f and the yaw, and
the board's print scale stays a free parameter. Stage B ties the camera to the beam with crosses
the beam burned at known LightBurn positions.
"""
import numpy as np
import pytest

from lhp_pkg import calib_solve as cs
from lhp_pkg import camera_model as cm
from lhp_pkg import closeup_plan as cp

# aspect: the 10W camera's pixels are not square (fy / fx = 0.9385, measured 2026-11-20)
TRUE = dict(f=812.3, aspect=0.9385, cx=546.0, cy=629.0, k1=-0.05, k2=0.012, rvec=(0.003, -0.002, 0.011), e=4.7)
# The board's top-left origin in the B frame (minus t): its corners then span x 70..235,
# y 302..62, centred near the pose target (160, 175).
BOARD_POSE = dict(bx=67.0, by=310.0, psi=0.013, s=1.004)


def true_model(t=(0.0, 0.0)):
    return cm.CameraModel(t=t, **TRUE)


def board_obs(model, poses, pose=BOARD_POSE, noise=0.2, seed=3, margin=25, outliers=0.0):
    rng = np.random.default_rng(seed)
    ids, bxy = cs.board_corners()
    world = cs.board_to_world(bxy, pose)
    obs = []
    for p in poses:
        head = (p["x"], p["y"], p["z"])
        px = model.project(head, np.column_stack([world, np.zeros(len(world))]))
        keep = ((px[:, 0] > margin) & (px[:, 0] < model.width - margin) &
                (px[:, 1] > margin) & (px[:, 1] < model.height - margin))
        if keep.sum() < 6:
            continue
        seen = px[keep] + rng.normal(0, noise, (keep.sum(), 2))
        if outliers:
            bad = rng.random(len(seen)) < outliers
            seen[bad] += rng.normal(0, 30, (bad.sum(), 2))
        obs.append(dict(head=head, ids=list(np.asarray(ids)[keep]), px=seen.tolist(),
                        holdout=p["holdout"]))
    return obs


def guess():
    return cm.CameraModel(f=814.0, e=5.0, t=(0.0, 0.0))


def poses_for(target=(160.0, 175.0)):
    """The real pose plan, aimed with the nominal camera (as the plugin will)."""
    return cp.calib_poses("board", target, guess())


def test_the_board_has_the_printed_layout():
    ids, xy = cs.board_corners()
    assert len(ids) == 12 * 17 and ids[0] == 0 and ids[-1] == 203          # 13 x 18 squares
    assert xy[1][0] - xy[0][0] == pytest.approx(15.0)                       # 15 mm squares
    assert xy[12][1] - xy[0][1] == pytest.approx(-15.0)                     # next row is lower (y up)


def test_the_board_pose_is_found_from_the_highest_photo():
    model = true_model()
    obs = board_obs(model, poses_for(), noise=0.2)
    pose = cs.init_board(obs, guess())
    # read through the nominal camera (principal point about 34 px out): rough, but close
    # enough for the fit to converge from
    assert pose["bx"] == pytest.approx(BOARD_POSE["bx"], abs=20.0)
    assert pose["by"] == pytest.approx(BOARD_POSE["by"], abs=20.0)
    assert pose["psi"] == pytest.approx(BOARD_POSE["psi"], abs=0.03)
    assert pose["s"] == pytest.approx(BOARD_POSE["s"], abs=0.07)      # the nominal camera has square pixels


def test_stage_a_recovers_the_camera_and_predicts_held_out_photos():
    model = true_model()
    obs = board_obs(model, poses_for())
    fit, pose, report = cs.solve_board(obs, guess())
    assert fit.f == pytest.approx(TRUE["f"], rel=0.002)
    assert fit.aspect == pytest.approx(TRUE["aspect"], abs=0.001)
    assert fit.e == pytest.approx(TRUE["e"], abs=0.2)
    assert report["fit_rms_px"] <= 0.3
    assert report["holdout_rms_mm"] <= 0.05 and report["holdout_max_mm"] <= 0.15
    assert report["photos"] >= 40 and report["holdout_photos"] >= 8
    assert fit.z_range == (40.0, 290.0)
    # the fitted camera maps pixels to the bed like the true one, anywhere in a close-up photo
    head = (200.0, 170.0, 50.0)
    grid = np.array([[u, v] for u in np.linspace(60, 964, 7) for v in np.linspace(60, 1220, 7)])
    a = model.to_plane(head, grid, 0.0)
    b = fit.to_plane(head, grid, 0.0) + np.array([pose["bx"] - BOARD_POSE["bx"], pose["by"] - BOARD_POSE["by"]])
    assert np.abs(a - b).max() < 0.1


def test_without_sideways_moves_the_scale_and_focal_length_are_reported_as_tangled():
    poses = [p for p in poses_for() if p["dx"] == 0 and p["dy"] == 0]
    obs = board_obs(true_model(), poses)
    fit, pose, report = cs.solve_board(obs, guess())
    assert abs(report["corr"]["f_s"]) > 0.95
    assert "tangled" in " ".join(report["warnings"])


def test_a_few_wild_corner_detections_do_not_spoil_the_fit():
    obs = board_obs(true_model(), poses_for(), outliers=0.01)
    fit, pose, report = cs.solve_board(obs, guess())
    assert fit.f == pytest.approx(TRUE["f"], rel=0.003)
    assert report["holdout_rms_mm"] <= 0.08            # wild detections are counted, not averaged in
    assert report["holdout_rejected"] >= 1


def test_stage_b_ties_the_camera_to_the_beam_with_the_burned_crosses():
    t_true = (-56.7, 6.4)
    o_burn = (4.0, -2.5)
    lb = [(40.0, 95.0), (280.0, 95.0), (40.0, 255.0), (280.0, 255.0)]
    crosses_b = [(x + o_burn[0], y + o_burn[1]) for x, y in lb]
    model = true_model(t=t_true)
    poses = cp.calib_poses("cross", crosses_b, cm.CameraModel(f=814.0, e=5.0, t=(-57.0, 6.0)))
    rng = np.random.default_rng(5)
    obs = []
    for p in poses:
        head = (p["x"], p["y"], p["z"])
        bx, by = crosses_b[p["cross"]]
        px = model.project(head, np.array([[bx, by, 0.0]]))[0] + rng.normal(0, 0.3, 2)
        obs.append(dict(head=head, px=px.tolist(), lb=lb[p["cross"]]))
    stage_a = true_model(t=(0.0, 0.0))
    fit, report = cs.solve_tie(stage_a, obs, o_burn)
    assert fit.t == pytest.approx(t_true, abs=0.05)
    assert fit.o_burn == o_burn
    assert report["loo_max_mm"] <= 0.3 and report["rms_mm"] <= 0.15
    assert len(report["crosses"]) == 4


def test_levenberg_marquardt_solves_a_plain_curve_fit_with_a_prior():
    xs = np.linspace(0, 1, 30)
    ys = 2.0 * np.exp(-1.3 * xs) + 0.5

    def resid(p):
        return p[0] * np.exp(-p[1] * xs) + p[2] - ys

    out = cs.lm(resid, np.array([1.0, 1.0, 0.0]), steps=np.array([1e-6, 1e-6, 1e-6]))
    assert out["x"] == pytest.approx([2.0, 1.3, 0.5], abs=1e-6)
    pinned = cs.lm(resid, np.array([1.0, 1.0, 0.0]), steps=np.array([1e-6] * 3),
                   priors=[(2, 0.4, 1e-6)])
    assert pinned["x"][2] == pytest.approx(0.4, abs=1e-5)


def test_an_uneven_bed_under_the_board_is_modelled_when_its_surface_is_given():
    # The A350's bed is ~1.8 mm out across its width: the board's corners are not at one height.
    tilt = lambda x, y: 0.008 * (np.asarray(x, float) - 160.0) + 0.004 * (np.asarray(y, float) - 175.0)
    model = true_model()
    _, bxy = cs.board_corners()
    world = cs.board_to_world(bxy, BOARD_POSE)
    rng = np.random.default_rng(8)
    obs = []
    for p in poses_for():
        head = (p["x"], p["y"], p["z"])
        px = model.project(head, np.column_stack([world, tilt(world[:, 0], world[:, 1])]))
        keep = (px[:, 0] > 25) & (px[:, 0] < 999) & (px[:, 1] > 25) & (px[:, 1] < 1255)
        if keep.sum() >= 6:
            obs.append(dict(head=head, ids=list(np.where(keep)[0]), px=(px[keep] + rng.normal(0, 0.2, (keep.sum(), 2))).tolist(),
                            holdout=p["holdout"]))
    flat_fit, _, flat_rep = cs.solve_board(obs, guess())
    bed_fit, _, bed_rep = cs.solve_board(obs, guess(), surface=tilt)
    assert bed_rep["holdout_rms_mm"] < flat_rep["holdout_rms_mm"]
    assert bed_rep["holdout_rms_mm"] <= 0.05
    assert bed_fit.f == pytest.approx(TRUE["f"], rel=0.002)


def test_the_tie_uses_the_crosses_own_heights():
    t_true, o_burn = (-56.7, 6.4), (4.0, -2.5)
    lb = [(60.0, 100.0), (200.0, 100.0), (60.0, 250.0), (200.0, 250.0)]
    bump = lambda x, y: 0.01 * (np.asarray(x, float) - 100.0)
    model = true_model(t=t_true)
    poses = cp.calib_poses("cross", [(x + o_burn[0], y + o_burn[1]) for x, y in lb],
                           cm.CameraModel(f=814.0, e=5.0, t=(-57.0, 6.0)))
    obs = []
    for p in poses:
        bx, by = lb[p["cross"]][0] + o_burn[0], lb[p["cross"]][1] + o_burn[1]
        px = model.project((p["x"], p["y"], p["z"]), np.array([[bx, by, float(bump(bx, by))]]))[0]
        obs.append(dict(head=(p["x"], p["y"], p["z"]), px=px.tolist(), lb=lb[p["cross"]]))
    fit, rep = cs.solve_tie(true_model(), obs, o_burn, surface=bump)
    assert fit.t == pytest.approx(t_true, abs=0.02) and rep["loo_max_mm"] <= 0.05
