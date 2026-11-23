# coding=utf-8
"""Image input and output for the close-up camera: the only module that calls OpenCV."""
from __future__ import absolute_import

from .closeup_libs import CloseupError, cv as _cv, np as _np


def decode(jpeg):
    """BGR uint8 image of JPEG bytes."""
    np, cv2 = _np(), _cv()
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise CloseupError("the photo is not a readable JPEG")
    return img


def encode(img, quality=90):
    cv2 = _cv()
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise CloseupError("could not encode the image as JPEG")
    return buf.tobytes()


def remap(img, map_u, map_v, cubic=True, border=0):
    """img sampled at (map_u, map_v); outside the image gives border."""
    cv2 = _cv()
    return cv2.remap(img, map_u, map_v, cv2.INTER_CUBIC if cubic else cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=(border, border, border))


def gray(img):
    np, cv2 = _np(), _cv()
    if img.ndim == 2:
        return img.astype(np.float32)
    return cv2.cvtColor(np.clip(img, 0, 255).astype(np.uint8), cv2.COLOR_BGR2GRAY).astype(np.float32)


def phase_shift(a, b):
    """(dx, dy) to move b onto a, by phase correlation with a Hanning window, and the response."""
    np, cv2 = _np(), _cv()
    ga, gb = gray(a), gray(b)
    win = cv2.createHanningWindow((ga.shape[1], ga.shape[0]), cv2.CV_32F)
    (dx, dy), response = cv2.phaseCorrelate(gb - gb.mean(), ga - ga.mean(), win)
    return float(dx), float(dy), float(response)


def blur(img, sigma):
    cv2 = _cv()
    return cv2.GaussianBlur(img, (0, 0), sigma) if sigma > 0 else img


def shrink(img, k):
    """img made k times smaller by area averaging (k = 1: unchanged)."""
    cv2 = _cv()
    if k <= 1:
        return img
    return cv2.resize(img, (max(1, img.shape[1] // k), max(1, img.shape[0] // k)), interpolation=cv2.INTER_AREA)


def shrink_to(img, width):
    """img scaled (area averaging) to width pixels wide, keeping its shape."""
    cv2 = _cv()
    h = int(round(width * img.shape[0] / float(img.shape[1])))
    return cv2.resize(img, (int(width), h), interpolation=cv2.INTER_AREA)


def grow(img, size):
    """img resized to size = (width, height), smoothly."""
    cv2 = _cv()
    if (img.shape[1], img.shape[0]) == tuple(size):
        return img
    return cv2.resize(img, tuple(size), interpolation=cv2.INTER_LINEAR)


def erode(img, size):
    """The minimum over a size x size square around each pixel."""
    cv2 = _cv()
    return cv2.erode(img, cv2.getStructuringElement(cv2.MORPH_RECT, (size, size)))


def distance_inside(mask):
    """Pixels from each True pixel to the nearest False one."""
    np, cv2 = _np(), _cv()
    return cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 5)


# -- the printed ChArUco board (calib_solve.BOARD) -----------------------------------------
def charuco_board(spec):
    cv2 = _cv()
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec["dictionary"]))
    return cv2.aruco.CharucoBoard(tuple(spec["squares"]), spec["square"] / 1000.0, spec["marker"] / 1000.0,
                                  dictionary)


def detect_charuco(img, spec):
    """(ids, px (N, 2)) of the board's inner corners found in a photo; ([], empty) if none."""
    np, cv2 = _np(), _cv()
    detector = cv2.aruco.CharucoDetector(charuco_board(spec))
    corners, ids, _, _ = detector.detectBoard(gray(img).astype(np.uint8))
    if ids is None or len(ids) == 0:
        return [], np.zeros((0, 2))
    return [int(i) for i in ids.ravel()], corners.reshape(-1, 2).astype(float)


def board_page(spec, dpi=600, page_mm=(210.0, 297.0)):
    """A printable A4 page (PNG) with the board centred at its true size when printed at dpi,
    100 %, and a caption saying so."""
    np, cv2 = _np(), _cv()
    mm = dpi / 25.4
    bw, bh = spec["squares"][0] * spec["square"], spec["squares"][1] * spec["square"]
    board = charuco_board(spec).generateImage((int(round(bw * mm)), int(round(bh * mm))), marginSize=0, borderBits=1)
    page = np.full((int(round(page_mm[1] * mm)), int(round(page_mm[0] * mm))), 255, np.uint8)
    y0, x0 = (page.shape[0] - board.shape[0]) // 2, (page.shape[1] - board.shape[1]) // 2
    page[y0:y0 + board.shape[0], x0:x0 + board.shape[1]] = board
    caption = ("ChArUco %dx%d, %g mm squares, %s. Print at 100%% (no fit to page). Tape flat."
               % (spec["squares"][0], spec["squares"][1], spec["square"], spec["dictionary"]))
    cv2.putText(page, caption, (x0, y0 + board.shape[0] + int(5 * mm)), cv2.FONT_HERSHEY_SIMPLEX,
                dpi / 600.0, 0, max(1, int(dpi / 300)), cv2.LINE_AA)
    ok, buf = cv2.imencode(".png", page)
    return buf.tobytes()


# -- the burned alignment crosses --------------------------------------------------------
def thin_dark(g, px_per_mm, width_mm=1.5):
    """Dark lines narrower than width_mm (a black-hat): a burned line stands out, while a broad
    dark area or a shadow's edge gives nothing (seen 2026-11-20: the head's shadow at Z50)."""
    np, cv2 = _np(), _cv()
    k = max(3, int(round(width_mm * px_per_mm)) | 1)
    return cv2.morphologyEx(g.astype(np.float32), cv2.MORPH_BLACKHAT,
                            cv2.getStructuringElement(cv2.MORPH_RECT, (k, k)))


def refine_cross(img, uv, px_per_mm, window_mm=7.0):
    """The centre of a burned cross near a rough click uv: the column and row of the window with
    the most thin dark line (the arms carry most of the burn), refined to a fraction of a pixel."""
    np = _np()
    g = gray(img)
    half = max(2, int(round(0.6 * px_per_mm)))          # about the burned line's width

    def peak(p, refine):
        i = int(np.argmax(p))
        if not refine:
            return float(i)
        lo, hi = max(0, i - half), min(len(p), i + half + 1)
        w = np.clip(p[lo:hi] - np.median(p), 0, None)
        return float((w * np.arange(lo, hi)).sum() / w.sum()) if w.sum() > 0 else float(i)

    u, v = float(uv[0]), float(uv[1])
    # pass 1 from the click; pass 2 re-centred, so the window is even about the cross
    for refine in (False, True):
        r = int(round(window_mm * px_per_mm))
        u0, v0 = int(round(u)), int(round(v))
        x0, y0 = max(0, u0 - r), max(0, v0 - r)
        x1, y1 = min(g.shape[1], u0 + r + 1), min(g.shape[0], v0 + r + 1)
        win = g[y0:y1, x0:x1]
        if win.size == 0:
            raise CloseupError("the click is outside the photo")
        dark = thin_dark(win, px_per_mm)
        u, v = x0 + peak(dark.sum(0), refine), y0 + peak(dark.sum(1), refine)
    return u, v


# -- the fixed USB bed camera (bedcam.py) ---------------------------------------------------
def undistort_points(px, K, dist, P=None):
    """Pixels (N, 2) with the lens distortion taken out: normalised camera coordinates, or
    pixels again with camera matrix P (P = K keeps the photo's scale)."""
    np, cv2 = _np(), _cv()
    pts = np.asarray(px, np.float64).reshape(-1, 1, 2)
    out = cv2.undistortPoints(pts, np.asarray(K, np.float64), np.asarray(dist, np.float64),
                              P=None if P is None else np.asarray(P, np.float64))
    return out.reshape(-1, 2)


def _cross_template(px_per_mm, arm_mm, sigma=1.2):
    np, cv2 = _np(), _cv()
    r = int(round((arm_mm + 1.5) * px_per_mm))
    a = int(round(arm_mm * px_per_mm))
    t = np.zeros((2 * r + 1, 2 * r + 1), np.float32)
    t[r, r - a:r + a + 1] = 1.0
    t[r - a:r + a + 1, r] = 1.0
    return cv2.GaussianBlur(t, (0, 0), sigma)


def find_cross(img, uv, px_per_mm, search_mm=15.0, arm_mm=8.0):
    """A burned cross (arms arm_mm from its centre) in a coarse photo (the bed camera, about
    2 px/mm, where a burned line is under a pixel wide): the best match of a cross shape to the
    photo's darkness within search_mm of uv, then the darkness-weighted centre over the arms.
    Returns (u, v, score) with score the match (0 to 1), or None if the window is off the photo.
    On the real bed camera (2026-11-21) it repeats to about 1 px between frames."""
    np, cv2 = _np(), _cv()
    g = gray(img)
    dark = cv2.GaussianBlur(g, (0, 0), 3.0 * px_per_mm) - g        # darker than the paper round it
    t = _cross_template(px_per_mm, arm_mm)
    r = t.shape[0] // 2
    s = int(round(search_mm * px_per_mm))
    u0, v0 = int(round(uv[0])), int(round(uv[1]))
    y0, y1 = max(0, v0 - s - r), min(g.shape[0], v0 + s + r + 1)
    x0, x1 = max(0, u0 - s - r), min(g.shape[1], u0 + s + r + 1)
    win = dark[y0:y1, x0:x1]
    if win.shape[0] < t.shape[0] or win.shape[1] < t.shape[1]:
        return None
    match = cv2.matchTemplate(win, t, cv2.TM_CCOEFF_NORMED)
    _, score, _, loc = cv2.minMaxLoc(match)
    pu, pv = x0 + loc[0] + r, y0 + loc[1] + r
    a = int(round(arm_mm * px_per_mm))
    yy, xx = np.mgrid[-a:a + 1, -a:a + 1]
    u, v = float(pu), float(pv)
    for _ in range(4):
        if pv - a < 0 or pu - a < 0 or pv + a + 1 > g.shape[0] or pu + a + 1 > g.shape[1]:
            break
        c = cv2.GaussianBlur(dark[pv - a:pv + a + 1, pu - a:pu + a + 1], (0, 0), 1.0)
        w = np.clip(c - 0.5 * c.max(), 0, None)
        if w.sum() <= 0:
            break
        du, dv = float((w * xx).sum() / w.sum()), float((w * yy).sum() / w.sum())
        u, v = pu + du, pv + dv
        if abs(du) < 0.5 and abs(dv) < 0.5:
            break
        pu, pv = int(round(u)), int(round(v))
    return u, v, float(score)
