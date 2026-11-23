"""OpenCV for the close-up camera: the ChArUco board, its detection, and the burned crosses."""
import numpy as np
import pytest

from lhp_pkg import calib_solve as cs
from lhp_pkg import camera_model as cm
from lhp_pkg import compositor as co
from lhp_pkg import cvio

O = (4.0, -2.5)
TOP_LEFT = (65.0, 310.0)          # the board's top-left corner, LightBurn mm


def model():
    return cm.CameraModel(f=812.3, cx=546.0, cy=629.0, k1=-0.05, k2=0.012, rvec=(0.003, -0.002, 0.011),
                          e=4.7, t=(-56.8, 6.3))


def test_the_solver_numbers_the_corners_as_opencv_does():
    import cv2
    board = cvio.charuco_board(cs.BOARD)
    opencv = board.getChessboardCorners()[:, :2] * 1000.0
    ids, xy = cs.board_corners()
    assert len(ids) == len(opencv)
    assert np.allclose(xy[:, 0], opencv[:, 0]) and np.allclose(xy[:, 1], -opencv[:, 1])


@pytest.fixture(scope="module")
def board_raster():
    img = cvio.charuco_board(cs.BOARD).generateImage((195 * 20, 270 * 20), marginSize=0, borderBits=1)
    return np.dstack([img] * 3), dict(x0=TOP_LEFT[0], y1=TOP_LEFT[1], dpm=20.0)


@pytest.mark.parametrize("z", [50.0, 290.0])
def test_the_board_is_found_in_a_photo_where_the_camera_says(board_raster, z):
    m = model()
    raster, rgeom = board_raster
    head = (175.0, 172.0, z)
    photo = cvio.decode(cvio.encode(co.render_photo(raster, rgeom, m, head, O, 0.0), 95))
    ids, px = cvio.detect_charuco(photo, cs.BOARD)
    assert len(ids) >= (8 if z < 100 else 150)
    _, bxy = cs.board_corners()
    world = np.column_stack([TOP_LEFT[0] + bxy[ids, 0] + O[0], TOP_LEFT[1] + bxy[ids, 1] + O[1], np.zeros(len(ids))])
    err = np.linalg.norm(m.project(head, world) - px, axis=1)
    assert np.median(err) < 0.3 and err.max() < 1.0


def test_a_photo_without_the_board_finds_nothing():
    ids, px = cvio.detect_charuco(np.full((1280, 1024, 3), 200, np.uint8), cs.BOARD)
    assert ids == [] and px.shape == (0, 2)


def _cross(centre, px_mm, size=(400, 400), ink=60):
    import cv2
    s = 16
    img = np.full(size, 230, np.uint8)
    cx, cy = centre
    arm, t = 10 * px_mm, max(1, int(round(0.4 * px_mm)))
    def p(x, y):
        return int(round(x * s)), int(round(y * s))
    cv2.line(img, p(cx - arm, cy), p(cx + arm, cy), ink, t, cv2.LINE_AA, shift=4)
    cv2.line(img, p(cx, cy - arm), p(cx, cy + arm), ink, t, cv2.LINE_AA, shift=4)
    cv2.ellipse(img, p(cx, cy), (int(5 * px_mm * s), int(5 * px_mm * s)), 0, 0, 90, ink, t, cv2.LINE_AA, shift=4)
    cv2.ellipse(img, p(cx, cy), (int(5 * px_mm * s), int(5 * px_mm * s)), 0, 180, 270, ink, t, cv2.LINE_AA, shift=4)
    return np.dstack([cv2.GaussianBlur(img, (0, 0), 1.0)] * 3)


@pytest.mark.parametrize("centre", [(200.3, 190.7), (180.85, 221.15)])
def test_a_burned_cross_is_found_to_a_fraction_of_a_pixel_from_a_rough_click(centre):
    img = _cross(centre, 12.5)
    u, v = cvio.refine_cross(img, (centre[0] + 9, centre[1] - 7), 12.5)
    assert abs(u - centre[0]) < 0.25 and abs(v - centre[1]) < 0.25


def test_the_printable_board_is_a4_at_the_right_scale():
    png = cvio.board_page(cs.BOARD, dpi=300)
    page = cvio.decode(png)
    assert page.shape[:2] == (int(round(297 / 25.4 * 300)), int(round(210 / 25.4 * 300)))
    ids, px = cvio.detect_charuco(page, cs.BOARD)
    assert len(ids) == 12 * 17
    step = np.median(np.diff(px[np.argsort(ids)][:12, 0]))
    assert step == pytest.approx(15.0 / 25.4 * 300, rel=0.005)


@pytest.mark.parametrize("centre", [(200.3, 190.7), (180.85, 221.15)])
def test_the_heads_shadow_beside_a_cross_does_not_pull_the_mark(centre):
    # Seen 2026-11-20: at Z50 the head's shadow can fall beside a cross; its dark edge must not
    # be taken for the cross's arm.
    img = _cross(centre, 15.0, ink=170).astype(float)          # a faint burn, as on paper
    img[:, int(centre[0]) + 45:] *= 0.3                        # a broad dark shadow 3 mm to the right
    img = np.clip(img, 0, 255).astype(np.uint8)
    u, v = cvio.refine_cross(img, (centre[0] + 9, centre[1] - 7), 15.0)
    assert abs(u - centre[0]) < 0.3 and abs(v - centre[1]) < 0.3
