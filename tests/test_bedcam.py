"""The fixed USB bed camera as the close-up base: its lens, its homography, the canvas, the
calibration from burned crosses and from LightBurn's prefs.

A synthetic bed camera (a LightBurn-like homography and a mild 12-term lens) photographs a
paper with the six burned crosses; the canvas built from that photo must put every cross
within 0.5 mm of where it is.
"""
import cv2
import numpy as np
import pytest

from lhp_pkg import bedcam as bc
from lhp_pkg import compositor as co
from lhp_pkg import cvio
from lhp_pkg.closeup_libs import CloseupError

# LightBurn's "Bed cam - fixed" projection (2026-11-20): image X runs against LightBurn X
H_LB = [[-0.4596, 0.00143, 443.99], [0.00203, 0.4569, -29.68], [2.14e-5, 1.44e-5, 0.9784]]
K = [[1334.8, 0.0, 666.0], [0.0, 1346.8, 300.0], [0.0, 0.0, 1.0]]
DIST = [0.06, -0.12, 0.001, -0.0007, 0.03, 0.01, -0.02, 0.01, 0.0005, 0.0, -0.0004, 0.0]


def true_model(bed_y=300.0):
    return bc.BedcamModel(H=H_LB, width=1280, height=720, bed_y=bed_y, K=K, dist=DIST, source="test")


def cross_raster(crosses=bc.CROSSES_LB, dpm=10.0, arm=8.0, line=0.6):
    """White paper over the bed with dark crosses (16 mm, line mm wide), LightBurn mm, and its
    geometry: pixel (i, j) centre at LB ((i + 0.5) / dpm, 350 - (j + 0.5) / dpm)."""
    w, h = int(320 * dpm), int(350 * dpm)
    img = np.full((h, w), 230, np.uint8)
    for x, y in crosses:
        u, v = x * dpm - 0.5, (350.0 - y) * dpm - 0.5
        t = max(1, int(round(line * dpm)))
        cv2.line(img, (int(round((u - arm * dpm) * 16)), int(round(v * 16))),
                 (int(round((u + arm * dpm) * 16)), int(round(v * 16))), 60, t, cv2.LINE_AA, 4)
        cv2.line(img, (int(round(u * 16)), int(round((v - arm * dpm) * 16))),
                 (int(round(u * 16)), int(round((v + arm * dpm) * 16))), 60, t, cv2.LINE_AA, 4)
    return np.dstack([img] * 3), dict(x0=0.0, y1=350.0, dpm=dpm)


def render(model, raster, rgeom, bed_shift=0.0):
    """What the bed camera sees of the raster: photo pixel -> lens -> H -> LightBurn mm. With
    the bed bed_shift mm further back than at the calibration (the bed at a lower Y), each
    pixel sees the point bed_shift mm nearer the front."""
    dpm = rgeom["dpm"]
    us, vs = np.meshgrid(np.arange(model.width, dtype=float), np.arange(model.height, dtype=float))
    lb = model.px_to_lb(np.column_stack([us.ravel(), vs.ravel()]))
    x, y = lb[:, 0], lb[:, 1] - bed_shift
    fi = ((x - rgeom["x0"]) * dpm - 0.5).reshape(us.shape).astype(np.float32)
    fj = ((rgeom["y1"] - y) * dpm - 0.5).reshape(us.shape).astype(np.float32)
    src = cvio.blur(raster, 0.5 * dpm / bc.PX_PER_MM)          # coarser than the raster: no aliasing
    out = cv2.remap(src, fi, fj, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=(40, 40, 40))
    return cvio.decode(cvio.encode(out, 90))


def find_crosses(canvas, geom, crosses, window=6.0):
    return co.find_dots(canvas, geom, crosses, window)


def errors(found, crosses):
    return [float(np.hypot(a - x, b - y)) for (a, b), (x, y) in zip(found, crosses)]


def test_the_lens_model_is_opencvs_with_all_twelve_terms():
    pts = np.random.RandomState(1).uniform(-0.45, 0.45, (200, 2))
    ref, _ = cv2.projectPoints(np.column_stack([pts, np.ones(len(pts))]), np.zeros(3), np.zeros(3),
                               np.asarray(K), np.asarray(DIST))
    xd, yd = bc.distort(DIST, pts[:, 0], pts[:, 1])
    k = np.asarray(K)
    mine = np.column_stack([k[0, 0] * xd + k[0, 2], k[1, 1] * yd + k[1, 2]])
    assert np.abs(mine - ref.reshape(-1, 2)).max() < 1e-6


def test_pixels_and_lightburn_mm_round_trip():
    m = true_model()
    lb = np.array(bc.CROSSES_LB)
    u, v, ok = m.lb_to_px(lb[:, 0], lb[:, 1])
    assert ok.all()
    back = m.px_to_lb(np.column_stack([u, v]))
    assert np.abs(back - lb).max() < 1e-4


def test_the_homography_warp_puts_every_cross_within_half_a_millimetre():
    m = true_model()
    raster, rgeom = cross_raster()
    photo = render(m, raster, rgeom)
    geom = co.canvas_geometry(5.0)
    canvas = co.bedcam_canvas(photo, m, geom)
    assert canvas.shape[:2] == (geom["H"], geom["W"])
    err = errors(find_crosses(canvas, geom, bc.CROSSES_LB), bc.CROSSES_LB)
    assert max(err) <= 0.5, err


# LightBurn's own 12-term lens for the bed camera (2026-11-20): big terms that cancel inside the
# photo and fold back outside it
DIST_LB = [0.4470589042972672, -13.082642019388286, -0.07416270028723287, -0.018232817544026475,
           89.96612027037165, 0.30163241044976613, -12.31026896491902, 87.3458561667412,
           0.019577764435822844, -0.011374570774750003, 0.06786227188545639, 0.013881157213885066]
K_LB = [[1334.843784935515, 0, 666.0332751409234], [0, 1346.810080657863, 221.92987379186482], [0, 0, 1]]


@pytest.mark.parametrize("lens", ["mild", "lightburn"])
def test_what_the_camera_cannot_see_stays_black(lens):
    m = true_model() if lens == "mild" else bc.BedcamModel(H=H_LB, width=1280, height=720, bed_y=300.0, K=K_LB,
                                                           dist=DIST_LB)
    geom = co.canvas_geometry(2.0)
    canvas = co.bedcam_canvas(np.full((720, 1280, 3), 200, np.uint8), m, geom)
    u, v, ok = m.lb_to_px(*co.lb_grid(geom, 0, geom["W"], 0, geom["H"]))
    inside = ok & (u > 2) & (u < 1277) & (v > 2) & (v < 717)
    outside = ~ok | (u < -2) | (u > 1281) | (v < -2) | (v > 721)
    assert inside.sum() > 1000 and outside.sum() > 1000
    assert canvas[inside].min() > 150 and canvas[outside].max() == 0


def test_a_photo_of_another_size_is_refused():
    with pytest.raises(CloseupError, match="calibrate"):
        co.bedcam_canvas(np.zeros((480, 640, 3), np.uint8), true_model(), co.canvas_geometry(2.0))


def test_the_crosses_calibration_recovers_the_camera_from_a_rough_guess():
    truth = true_model()
    raster, rgeom = cross_raster()
    photo = render(truth, raster, rgeom)
    # a guess about 4 % out in scale and 5 mm off, as LightBurn's own alignment was
    guess_h = np.diag([1.04, 1.04, 1.0]).dot(np.asarray(H_LB)) + np.array([[0, 0, 5.0], [0, 0, -4.0], [0, 0, 0]])
    guess = bc.BedcamModel(H=guess_h, width=1280, height=720, bed_y=300.0, K=K, dist=DIST)
    lb = np.array(bc.CROSSES_LB)
    u, v, _ = guess.lb_to_px(lb[:, 0], lb[:, 1])
    found = [cvio.find_cross(photo, (a, b), bc.PX_PER_MM) for a, b in zip(u, v)]
    assert all(f is not None for f in found)
    before = bc.check(guess, bc.CROSSES_LB, found)
    assert before["max_mm"] > 3.0
    model, report = bc.fit(bc.CROSSES_LB, found, 1280, 720, 300.0, lens=(guess.K, guess.dist))
    assert report["n"] == 6 and report["spare"] == 4 and report["max_mm"] <= 0.5, report
    geom = co.canvas_geometry(5.0)
    canvas = co.bedcam_canvas(photo, model, geom)
    err = errors(find_crosses(canvas, geom, bc.CROSSES_LB), bc.CROSSES_LB)
    assert max(err) <= 0.5, err
    again = bc.BedcamModel.from_dict(model.to_dict())
    assert np.allclose(again.H, model.H) and again.bed_y == 300.0 and again.residuals["n"] == 6


def test_a_fit_needs_four_crosses():
    with pytest.raises(CloseupError, match="at least 4"):
        bc.fit(bc.CROSSES_LB, [(1.0, 2.0), None, None, (3.0, 4.0), None, (5.0, 6.0)], 1280, 720, 300.0)


def test_the_homography_fit_is_exact_on_exact_points():
    src = np.random.RandomState(2).uniform(0, 1000, (8, 2))
    h = np.array([[0.4, 0.01, 30.0], [-0.02, 0.5, 10.0], [1e-5, 2e-5, 1.0]])
    dst = bc._apply(h, src)
    assert np.abs(bc.fit_homography(src, dst) - h).max() < 1e-8


def _lightburn_camera(**align):
    a = dict(AlignmentName="Overview_0mm", CalibrationName="standard",
             ImageProjection=list(np.asarray(H_LB).T.ravel()), ImageSizeX=1280, ImageSizeY=720, MaterialHeight=0)
    a.update(align)
    return dict(DisplayName="Bed cam - fixed", SelectedAlignmentName="Overview_0mm",
                SelectedCalibrationName="standard", ImageAlignments=[a],
                LensCalibrations=[dict(CalibrationName="standard", CameraMatrix=list(np.asarray(K).T.ravel()),
                                       DistortionMatrix=DIST, CameraSizeX=1280, CameraSizeY=720, IsFisheye=False)])


def test_lightburns_camera_reads_back_column_major():
    prefs = dict(DeviceList=[dict(Settings=dict(Cameras=[dict(DisplayName="other"), _lightburn_camera()]))])
    m = bc.from_lightburn(bc.find_lightburn_camera(prefs, "Bed cam - fixed"), 300.0)
    assert np.allclose(m.H, H_LB) and np.allclose(m.K, K) and m.dist == DIST and m.bed_y == 300.0
    assert "unverified" in m.source
    with pytest.raises(CloseupError, match="no camera"):
        bc.find_lightburn_camera(prefs, "missing")
    with pytest.raises(CloseupError, match="material"):
        bc.from_lightburn(_lightburn_camera(MaterialHeight=3), 300.0)


def test_the_real_lightburn_alignment_puts_its_own_fiducials_back():
    """LightBurn's stored fiducials (pixels, mm) go through its stored projection."""
    h = bc.column_major([-0.45959039304011323, 0.002033644908908219, 2.1406788514823556e-05,
                         0.0014280715817978089, 0.4569335919459753, 1.4424325646987313e-05,
                         443.9852720358741, -29.684146825486163, 0.9784410018339719])
    px = [(880.401, 622.612), (361.71, 618.692), (363.67, 269.197), (879.747, 269.197)]
    mm = [(40, 255), (280, 255), (280, 95), (40, 95)]
    assert np.abs(bc._apply(h, px) - np.array(mm, float)).max() < 0.05
