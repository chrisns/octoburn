"""The 10W head camera's geometry: where a bed point lands in the photo, and back.

Frames. Head H = (Hx, Hy, Z) is the machine position (M114). B is the beam frame: for a bed point,
the head XY that puts the beam on it (LightBurn = B - o). h is height above the bare bed. The
camera centre is C = (Hx + tx, Hy + ty, Z + e - d_bed); it only translates with the head (fixed
orientation R). Photos are 1024 x 1280, image right = +X, image up = +Y (the back of the bed).
Measured 2026-11-20: px/mm = 814 / (Z + 5), so f is about 814 and e about 5.
"""
import numpy as np
import pytest

from lhp_pkg import camera_model as cm


def ideal(**kw):
    base = dict(f=814.0, cx=512.0, cy=640.0, k1=0.0, k2=0.0, rvec=(0.0, 0.0, 0.0), t=(-57.0, 6.0), e=5.0)
    base.update(kw)
    return cm.CameraModel(**base)


def real():
    """Something like the real camera: offset principal point, small tilt and yaw, barrel."""
    return ideal(cx=548.0, cy=628.0, k1=-0.06, k2=0.01, rvec=(0.004, -0.003, 0.012), t=(-56.8, 6.3), e=4.6)


@pytest.mark.parametrize("model", [ideal(), real()])
def test_a_point_projected_into_the_photo_comes_back_to_the_same_place(model):
    rng = np.random.default_rng(1)
    for head, h in (((232.0, 178.0, 290.0), 0.0), ((300.0, 120.0, 50.0), 0.0),
                    ((150.0, 200.0, 70.0), 12.5), ((90.0, 60.0, 40.0), 0.0)):
        cxy = model.nadir(head, h)
        pts = np.column_stack([cxy[0] + rng.uniform(-20, 20, 50), cxy[1] + rng.uniform(-20, 20, 50),
                               np.full(50, h)])
        px = model.project(head, pts)
        back = model.to_plane(head, px, h)
        assert np.abs(back - pts[:, :2]).max() < 1e-6


def test_the_ideal_camera_reproduces_the_measured_scale():
    model = ideal()
    for z in (40.0, 60.0, 100.0, 200.0, 290.0):
        head = (200.0, 150.0, z)
        x, y = model.nadir(head, 0.0)
        px = model.project(head, np.array([[x, y, 0.0], [x + 1.0, y, 0.0], [x, y + 1.0, 0.0]]))
        assert px[1, 0] - px[0, 0] == pytest.approx(814.0 / (z + 5.0), rel=1e-9)
        assert px[0, 1] - px[2, 1] == pytest.approx(814.0 / (z + 5.0), rel=1e-9)   # +Y is up the photo


def test_the_image_orientation_matches_the_machine():
    model, head = ideal(), (200.0, 150.0, 100.0)
    x, y = model.nadir(head, 0.0)
    centre, right, back = model.project(head, np.array([[x, y, 0], [x + 5, y, 0], [x, y + 5, 0]], float))
    assert right[0] > centre[0] and abs(right[1] - centre[1]) < 1e-9     # +X: image right
    assert back[1] < centre[1] and abs(back[0] - centre[0]) < 1e-9       # +Y: image up
    assert np.allclose(centre, (512, 640))                               # nadir at the principal point
    assert model.nadir(head, 0.0) == pytest.approx((200 - 57.0, 150 + 6.0))


def test_an_off_centre_principal_point_and_a_tilt_both_make_the_view_drift_with_height():
    # The view centre moved about 7 mm in Y between Z200 and Z40 on the machine. A principal
    # point dy px off centre moves it by dy/f * (D1 - D2); a tilt a by tan(a) * (D1 - D2).
    flat = ideal()
    shifted = ideal(cy=640.0 - 36.0)            # principal point 36 px above the image centre
    tilted = ideal(rvec=(0.02, 0.0, 0.0))       # 0.02 rad about the camera's x axis
    for model in (flat, shifted):                # the middle of the view (image centre)
        lo, hi = model.centre_offset(40.0, 0.0), model.centre_offset(200.0, 0.0)
        drift = hi[1] - lo[1]
        if model is flat:
            assert drift == pytest.approx(0.0, abs=1e-9)
        else:
            assert abs(drift) == pytest.approx(36.0 / 814.0 * 160.0, rel=0.02)
    # a tilt moves the principal ray's foot by tan(a) per mm of height (perspective moves the
    # footprint's middle further still, which is why tiles are planned from the footprint)
    lo, hi = tilted.nadir((0.0, 0.0, 40.0), 0.0), tilted.nadir((0.0, 0.0, 200.0), 0.0)
    assert abs(hi[1] - lo[1]) == pytest.approx(np.tan(0.02) * 160.0, rel=1e-6)


@pytest.mark.parametrize("model", [ideal(), real()])
def test_the_footprint_is_inside_the_photo_and_as_big_as_it_can_be(model):
    for z, h, margin in ((50.0, 0.0, 40), (290.0, 0.0, 0), (70.0, 15.0, 60)):
        head = (200.0, 170.0, z)
        x0, y0, x1, y1 = model.footprint(z, h, margin)
        assert x0 < x1 and y0 < y1
        grid = np.array([[head[0] + x, head[1] + y, h] for x in np.linspace(x0, x1, 25)
                         for y in np.linspace(y0, y1, 25)])
        px = model.project(head, grid)
        assert px[:, 0].min() >= margin - 1e-6 and px[:, 0].max() <= 1024 - margin + 1e-6
        assert px[:, 1].min() >= margin - 1e-6 and px[:, 1].max() <= 1280 - margin + 1e-6
        # 2 % bigger on any side leaves the usable area
        for grow in ((-0.02, 0, 0, 0), (0, -0.02, 0, 0), (0, 0, 0.02, 0), (0, 0, 0, 0.02)):
            w, hh = x1 - x0, y1 - y0
            gx0, gy0, gx1, gy1 = x0 + grow[0] * w, y0 + grow[1] * hh, x1 + grow[2] * w, y1 + grow[3] * hh
            edge = np.array([[head[0] + x, head[1] + y, h] for x in np.linspace(gx0, gx1, 41)
                             for y in np.linspace(gy0, gy1, 41)])
            p = model.project(head, edge)
            inside = ((p[:, 0] >= margin) & (p[:, 0] <= 1024 - margin) &
                      (p[:, 1] >= margin) & (p[:, 1] <= 1280 - margin))
            assert not inside.all()
        cx, cy = model.centre_offset(z, h, margin)
        assert (cx, cy) == pytest.approx(((x0 + x1) / 2, (y0 + y1) / 2))


def test_at_z50_one_photo_covers_about_what_the_machine_showed():
    x0, y0, x1, y1 = ideal().footprint(50.0, 0.0, 0)
    assert x1 - x0 == pytest.approx(1024 * 55 / 814.0, rel=1e-6)       # about 69 x 86 mm
    assert y1 - y0 == pytest.approx(1280 * 55 / 814.0, rel=1e-6)


def test_the_pixel_maps_agree_with_project():
    model, head, h = real(), (180.0, 140.0, 55.0), 0.0
    xs, ys = np.meshgrid(np.linspace(110, 150, 9), np.linspace(120, 170, 7))
    mu, mv = model.pixel_maps(head, h, xs, ys)
    assert mu.dtype == np.float32 and mu.shape == xs.shape
    px = model.project(head, np.column_stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)]))
    assert np.abs(mu.ravel() - px[:, 0]).max() < 1e-3 and np.abs(mv.ravel() - px[:, 1]).max() < 1e-3


def test_non_square_pixels_scale_the_vertical_only():
    # the 10W camera's pixels are about 6 % shorter than wide (fy / fx = 0.9385)
    model, head = ideal(aspect=0.9385), (200.0, 150.0, 60.0)
    x, y = model.nadir(head, 0.0)
    px = model.project(head, np.array([[x, y, 0.0], [x + 1.0, y, 0.0], [x, y + 1.0, 0.0]]))
    assert px[1, 0] - px[0, 0] == pytest.approx(814.0 / 65.0)
    assert px[0, 1] - px[2, 1] == pytest.approx(814.0 * 0.9385 / 65.0)
    back = model.to_plane(head, px, 0.0)
    assert np.abs(back - np.array([[x, y], [x + 1, y], [x, y + 1]])).max() < 1e-9
    assert cm.CameraModel.from_dict(model.to_dict()).aspect == 0.9385


def test_the_bed_reference_moves_the_camera_height():
    # A different flat_bed_z than at calibration (d_bed) is the same as the plane moving.
    model, head = ideal(), (200.0, 150.0, 60.0)
    a = model.project(head, np.array([[150.0, 160.0, 2.0]]))
    b = model.project(head, np.array([[150.0, 160.0, 0.0]]), d_bed=2.0)
    assert np.allclose(a, b)


def test_the_model_survives_a_round_trip_through_json():
    import json
    model = real()
    model.z_range, model.residuals, model.bed_ref, model.o_burn = (40.0, 290.0), {"rms_mm": 0.04}, 13.6, (4.0, -2.5)
    back = cm.CameraModel.from_dict(json.loads(json.dumps(model.to_dict())))
    assert back.to_dict() == model.to_dict()
    head = (210.0, 150.0, 45.0)
    pts = np.array([[150.0, 160.0, 0.0]])
    assert np.allclose(back.project(head, pts), model.project(head, pts))


def test_a_plane_at_or_above_the_camera_is_refused():
    with pytest.raises(cm.CloseupError, match="above"):
        ideal().to_plane((200.0, 150.0, 20.0), np.array([[512.0, 640.0]]), 30.0)


def test_a_height_per_point_follows_an_uneven_surface():
    model, head = real(), (180.0, 140.0, 55.0)
    xs, ys = np.meshgrid(np.linspace(110, 150, 5), np.linspace(120, 170, 4))
    hs = 0.01 * (xs - 110) + 0.005 * (ys - 120)               # a gently tilted bed
    mu, mv = model.pixel_maps(head, hs, xs, ys)
    px = model.project(head, np.column_stack([xs.ravel(), ys.ravel(), hs.ravel()]))
    assert np.abs(mu.ravel() - px[:, 0]).max() < 1e-3 and np.abs(mv.ravel() - px[:, 1]).max() < 1e-3
    back = model.to_plane(head, px, hs.ravel())
    assert np.abs(back - np.column_stack([xs.ravel(), ys.ravel()])).max() < 1e-6
    assert cm.CameraModel.from_dict(dict(model.to_dict(), surface_ref=0.458)).surface_ref == 0.458


def test_a_footprint_can_leave_a_wider_band_on_one_side():
    # the head's shadow falls on the right of the photo: (left, top, right, bottom) margins
    model = ideal()
    x0, y0, x1, y1 = model.footprint(50.0, 0.0, (40, 40, 450, 40))
    a0, b0, a1, b1 = model.footprint(50.0, 0.0, 40)
    assert x0 == pytest.approx(a0) and y0 == pytest.approx(b0) and y1 == pytest.approx(b1)
    assert x1 == pytest.approx(a1 - 410 * 55.0 / 814.0, abs=1e-6)
