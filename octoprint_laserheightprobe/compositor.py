# coding=utf-8
"""The close-up image: photos placed on the bed in LightBurn coordinates.

The canvas covers the whole bed. Pixel (i, j) has its centre at LB ((i + 0.5) / dpm,
bed_h - (j + 0.5) / dpm), as lightburn_camera.py writes into LightBurn's prefs.

Every photo is placed by a forward remap: canvas pixel -> LB -> B = LB + o -> camera model ->
photo pixel. That handles the lens exactly with one interpolation.
- The base is the latest overview photo, simply resampled. It is not blurred; it only looks soft
  beside the close-ups. With closeup_base = "usb" the overview is the fixed bed camera's photo,
  placed through its own calibration instead (bedcam_canvas, bedcam.py).
- Close-up tiles are blended with weights that fall off near the photo's edge and away from the
  point under the camera, where a height error moves the image least.
- QA reports how far overlapping tiles disagree, in mm: a sign of a wrong height or a moved object.
"""
from __future__ import absolute_import

from . import cvio
from .closeup_libs import CloseupError, cv as _cv, np as _np
from .lightburn_camera import BED_H, BED_W

MARGIN_PX = 40            # a tile's outer band is ignored (vignetting, lens edge)
RAMP_PX = 30              # then its weight ramps up over this many pixels
NADIR_R0 = 0.35           # nadir weight 1 / (1 + (r / (R0 * D))^2)
GAIN_LIMITS = (0.5, 3.0)  # the local brightness correction against the overview
FLAT_MM = 3.0             # its smoothing: shadow and vignetting are broad, detail is fine
SHADOW = (0.45, 0.85)     # a pixel this dark against the overview (ratio) counts from little to fully
COVER_MIN = 0.25          # a pixel is close-up only where some tile's weight reaches this
BAND_WEIGHT = 0.05                     # a tile's weight inside its shadow band, if a neighbour covers it
BAND_RAMP = 0.1                        # ... reached over this share of the photo before the band
QA_MIN_PX = 48            # overlaps smaller than this are not compared
QA_MIN_TEXTURE = 6.0      # grey-level spread an overlap needs to be compared (blank paper has none)
QA_MIN_RESPONSE = 0.15    # phase correlation peak needed to trust a seam measurement
STRIP = 256


def canvas_geometry(dpm, bed_w=BED_W, bed_h=BED_H, max_mp=45.0):
    w, h = int(round(bed_w * dpm)), int(round(bed_h * dpm))
    if dpm <= 0 or w * h > max_mp * 1e6:
        raise CloseupError("a %g px/mm canvas is %.0f megapixels (maximum %.0f)" % (dpm, w * h / 1e6, max_mp))
    return dict(dpm=float(dpm), W=w, H=h, bed_w=float(bed_w), bed_h=float(bed_h))


def px_rect(geom, lb_rect):
    """Canvas pixel ranges (i0, i1, j0, j1), end exclusive, covering lb_rect, clipped."""
    import math
    x0, y0, x1, y1 = lb_rect
    d = geom["dpm"]
    i0 = max(0, int(math.floor(x0 * d)))
    i1 = min(geom["W"], int(math.ceil(x1 * d)))
    j0 = max(0, int(math.floor((geom["bed_h"] - y1) * d)))
    j1 = min(geom["H"], int(math.ceil((geom["bed_h"] - y0) * d)))
    return i0, i1, j0, j1


def lb_grid(geom, i0, i1, j0, j1):
    np = _np()
    d = geom["dpm"]
    xs = (np.arange(i0, i1) + 0.5) / d
    ys = geom["bed_h"] - (np.arange(j0, j1) + 0.5) / d
    return np.meshgrid(xs, ys)


def warp(photo, head, model, o, h, geom, rect, d_bed=0.0, surface=None):
    """The photo resampled onto canvas pixels rect = (i0, i1, j0, j1), and each pixel's distance
    (in photo pixels) inside the photo's edge (negative outside). surface(x, y), B frame, gives
    the height of an uneven bed per point instead of h."""
    np = _np()
    xs, ys = lb_grid(geom, *rect)
    heights = surface(xs + o[0], ys + o[1]) if surface is not None else h
    mu, mv = model.pixel_maps(head, heights, xs + o[0], ys + o[1], d_bed)
    out = cvio.remap(photo, mu, mv, cubic=True)
    edge = np.minimum(np.minimum(mu, model.width - 1 - mu), np.minimum(mv, model.height - 1 - mv))
    return out, edge, xs, ys, mu


def base_canvas(photo, head, model, o, geom, h=0.0, d_bed=0.0, surface=None):
    """The whole canvas from one photo (the overview), in strips to bound memory."""
    np = _np()
    canvas = np.zeros((geom["H"], geom["W"], 3), np.uint8)
    for j0 in range(0, geom["H"], STRIP):
        j1 = min(geom["H"], j0 + STRIP)
        part = warp(photo, head, model, o, h, geom, (0, geom["W"], j0, j1), d_bed, surface)[0]
        canvas[j0:j1] = part
    return canvas


def bedcam_canvas(photo, bedcam, geom):
    """The whole canvas from a fixed bed camera photo (bedcam.py), through its lens and its
    pixel -> LightBurn homography, in strips. Black where the camera does not see."""
    np = _np()
    if photo.shape[1] != bedcam.width or photo.shape[0] != bedcam.height:
        raise CloseupError("the bed camera photo is %d x %d, but its calibration is for %d x %d: calibrate it again"
                           % (photo.shape[1], photo.shape[0], bedcam.width, bedcam.height))
    canvas = np.zeros((geom["H"], geom["W"], 3), np.uint8)
    for j0 in range(0, geom["H"], STRIP):
        j1 = min(geom["H"], j0 + STRIP)
        xs, ys = lb_grid(geom, 0, geom["W"], j0, j1)
        mu, mv, ok = bedcam.lb_to_px(xs, ys)
        mu = np.where(ok, mu, -1e4).astype(np.float32)        # off the photo: the border colour
        mv = np.where(ok, mv, -1e4).astype(np.float32)
        canvas[j0:j1] = cvio.remap(photo, mu, mv, cubic=True)
    return canvas


def _tile_rect(tile, model, o, h, geom, d_bed):
    z = tile["head"][2]
    fx0, fy0, fx1, fy1 = model.footprint(z, h, 0, d_bed)
    hx, hy = tile["head"][0], tile["head"][1]
    return (hx + fx0 - o[0], hy + fy0 - o[1], hx + fx1 - o[0], hy + fy1 - o[1])


def compose(base, tiles, model, o, h, geom, feather_mm=2.0, d_bed=0.0, surface=None, shadow_frac=0.4,
            overview=None):
    """base (H, W, 3) uint8 with the close-up tiles [{img, head}] blended in. Returns
    (canvas, qa). surface(x, y): the bed's height per point (when the object is the bed)."""
    np = _np()
    if not tiles:
        return base.copy(), dict(tiles=0, seam_mm=None, pairs=[])
    rects = [_tile_rect(t, model, o, h, geom, d_bed) for t in tiles]
    union = (min(r[0] for r in rects), min(r[1] for r in rects), max(r[2] for r in rects), max(r[3] for r in rects))
    i0, i1, j0, j1 = px_rect(geom, union)
    if i1 <= i0 or j1 <= j0:
        raise CloseupError("the close-up photos do not cover any of the bed")
    base_part = base[j0:j1, i0:i1].astype(np.float32)
    # what each tile is levelled against: the overview at the tiles' own height (a tall object's
    # top then lines up in both, with no parallax), else the base as it is
    if overview is not None:
        compare = warp(overview["img"], overview["head"], model, o, h, geom, (i0, i1, j0, j1), d_bed,
                       surface)[0].astype(np.float32)
    else:
        compare = base_part
    acc = np.zeros_like(base_part)
    wsum = np.zeros(base_part.shape[:2], np.float32)
    wmax = np.zeros(base_part.shape[:2], np.float32)
    warps = []
    for t in tiles:
        img, edge, xs, ys, mu = warp(t["img"], t["head"], model, o, h, geom, (i0, i1, j0, j1), d_bed, surface)
        img = img.astype(np.float32)
        w_edge = np.clip((edge - MARGIN_PX) / float(RAMP_PX), 0.0, 1.0)
        nx, ny = model.nadir(t["head"], np.mean(h) if np.ndim(h) else h, d_bed)
        r = np.hypot(xs + o[0] - nx, ys + o[1] - ny)
        dist = t["head"][2] + model.e - d_bed - (np.mean(h) if np.ndim(h) else h)
        w = (w_edge / (1.0 + (r / (NADIR_R0 * dist)) ** 2)).astype(np.float32)
        # where the head's shadow can fall in this photo: the default band, widened to where this
        # photo's own shadow starts (it moves with the head)
        start = min((1.0 - shadow_frac) * model.width, _shadow_start(img, compare, mu, w_edge > 0, model.width))
        band = mu > start
        img, shade = _flatten(img, compare, w_edge > 0, geom["dpm"], band)
        # the band (and the soft edge of the shadow just before it, seen 2026-11-21 as 2-3 %
        # grey stripes) is a last resort: where a neighbour saw the spot lit, the neighbour wins;
        # where none did, normalising leaves this tile's own pixels
        last_resort = BAND_WEIGHT + (1.0 - BAND_WEIGHT) * np.clip((start - mu) / (BAND_RAMP * model.width), 0.0, 1.0)
        w *= shade * last_resort.astype(np.float32)
        acc += img * w[..., None]
        wsum += w
        np.maximum(wmax, w_edge * shade, out=wmax)      # how well some tile saw it (lit, not at an edge)
        warps.append((img, w_edge > 0))
    # close-up only where some tile saw the spot well (lit, not at its edge); elsewhere the overview
    covered = wmax > COVER_MIN
    close = np.where(covered[..., None], acc / np.maximum(wsum, 1e-6)[..., None], base_part)
    feather = max(feather_mm * geom["dpm"], 1.0)
    alpha = np.clip(cvio.distance_inside(covered) / feather, 0.0, 1.0)[..., None]
    out = base.copy()
    out[j0:j1, i0:i1] = np.clip(alpha * close + (1 - alpha) * base_part, 0, 255).astype(np.uint8)
    qa = _seams(warps, geom)
    qa.update(tiles=len(tiles), rect_px=(i0, i1, j0, j1))
    return out, qa


def _shadow_start(img, compare, mu, valid, width, bins=32):
    """The photo column where this photo's shadow begins (a band running down its right side),
    from the median tile/overview brightness of each column band; width if none. The median
    down a column is not fooled by an object, which does not span the photo's full height."""
    np = _np()
    if not valid.any():
        return float(width)
    ratio = cvio.gray(img)[valid] / np.maximum(cvio.gray(compare)[valid], 1.0)
    col = np.clip((mu[valid] / width * bins).astype(int), 0, bins - 1)
    med = np.full(bins, np.nan)
    for b in range(bins):
        sel = ratio[col == b]
        if sel.size > 50:
            med[b] = np.median(sel)
    lit = np.nanmedian(med[: bins // 2]) if np.isfinite(med[: bins // 2]).any() else np.nanmedian(med)
    if not np.isfinite(lit) or lit <= 0:
        return float(width)
    dark = med / lit < 0.85
    start = bins
    for b in range(bins - 1, -1, -1):                 # the run of dark bands reaching the right edge
        if not np.isfinite(med[b]):
            continue
        if dark[b]:
            start = b
        else:
            break
    return float(width) if start >= bins else (start - 1) * width / float(bins)


def _flatten(img, base, valid, dpm, band=None):
    """The tile levelled against the overview, and a weight per pixel for the blend.

    A shadow is fixed to the head (the same band of every photo, `band`); content is fixed to
    the bed. So the brightness correction is a smooth quadratic over the tile, fitted robustly
    outside the band: it follows vignetting and exposure but cannot follow an object's shape,
    so a dark object (even one the overview never saw) is kept. Only inside the band is a pixel
    darker than the overview taken for shadow and weighted down, held low within 2 x FLAT_MM of
    it; the tile plan leaves a lit neighbour for that band (closeup_shadow_frac)."""
    np = _np()
    k = max(1, int(dpm // 4))                      # worked out small: everything here is smooth
    small = lambda a: cvio.shrink(a, k)
    gt, gb = small(cvio.gray(img)), small(cvio.gray(base))
    vs = small(valid.astype(np.float32)) > 0.5
    bs = small(band.astype(np.float32)) > 0.5 if band is not None else np.zeros_like(vs)
    hh, ww = gt.shape
    yy, xx = np.mgrid[0:hh, 0:ww].astype(np.float32)
    if vs.any():                                   # coordinates over the tile's own area, not the union
        rows, cols = np.where(vs.any(1))[0], np.where(vs.any(0))[0]
        xx = (xx - (cols[0] + cols[-1]) / 2.0) / max(cols[-1] - cols[0], 1)
        yy = (yy - (rows[0] + rows[-1]) / 2.0) / max(rows[-1] - rows[0], 1)
    terms = np.stack([np.ones_like(xx), xx, yy, xx * xx, xx * yy, yy * yy], -1)
    ratio = gb / np.maximum(gt, 1.0)
    use = vs & ~bs & (gt > 8) & (gb > 8)
    gain = np.full_like(gt, float(np.median(ratio[vs])) if vs.any() else 1.0)
    if use.sum() > 60:
        a, b = terms[use], ratio[use]
        med = np.median(b)
        keep = np.abs(b - med) < 3.0 * 1.4826 * (np.median(np.abs(b - med)) + 1e-6)
        for _ in range(4):                         # robust: content the overview lacks is an outlier
            coef = np.linalg.lstsq(a[keep], b[keep], rcond=None)[0]
            res = b - a.dot(coef)
            mad = np.median(np.abs(res[keep] - np.median(res[keep]))) + 1e-6
            keep = np.abs(res) < 3.0 * 1.4826 * mad
            if keep.sum() < 30:
                break
        gain = terms.dot(coef).astype(np.float32)
    gain = np.clip(gain, *GAIN_LIMITS)
    shade = np.ones_like(gt, dtype=np.float32)
    if bs.any():
        sigma = FLAT_MM * dpm / k
        mask = vs.astype(np.float32)
        norm = np.maximum(cvio.blur(mask, sigma), 1e-3)
        lit = (cvio.blur(gt * gain * mask, sigma) / norm) / np.maximum(cvio.blur(gb * mask, sigma) / norm, 1.0)
        dark = np.clip((lit - SHADOW[0]) / (SHADOW[1] - SHADOW[0]), 0.02, 1.0).astype(np.float32)
        dark = cvio.erode(dark, max(3, int(round(2 * FLAT_MM * dpm / k)) | 1))
        shade = np.where(bs, dark, 1.0).astype(np.float32)
    size = (img.shape[1], img.shape[0])
    return img * cvio.grow(gain, size)[..., None], cvio.grow(shade, size)


def _seams(warps, geom):
    """How far overlapping tiles disagree, in mm (phase correlation over each overlap)."""
    np = _np()
    pairs = []
    for a in range(len(warps)):
        for b in range(a + 1, len(warps)):
            both = warps[a][1] & warps[b][1]
            rows, cols = np.where(both.any(1))[0], np.where(both.any(0))[0]
            if len(rows) < QA_MIN_PX or len(cols) < QA_MIN_PX:
                continue
            sl = (slice(rows[0], rows[-1] + 1), slice(cols[0], cols[-1] + 1))
            texture = float(min(cvio.gray(warps[a][0][sl]).std(), cvio.gray(warps[b][0][sl]).std()))
            if texture < QA_MIN_TEXTURE:
                continue                                  # blank paper: nothing to line up
            dx, dy, resp = cvio.phase_shift(warps[a][0][sl], warps[b][0][sl])
            if resp < QA_MIN_RESPONSE:
                continue
            pairs.append(dict(a=a, b=b, mm=float(np.hypot(dx, dy) / geom["dpm"]), response=resp, texture=texture))
    return dict(seam_mm=max([p["mm"] for p in pairs]) if pairs else None, pairs=pairs)


def surface_from_map(nodes, start, spacing, offset, ref=0.0):
    """surface(x, y): the bed's height at B-frame points from the plugin's height map, as the A350
    applies it (heightmap.evaluate: Catmull-Rom subdivision, whole-mm spacing // 3, carried on
    past the grid), less ref (the map's value where the camera was calibrated). Vectorised."""
    from . import heightmap
    np = _np()
    virt = np.asarray(heightmap.virtual_grid(nodes), float)
    nv = virt.shape[0]
    sub = heightmap.SUBDIVISIONS
    sv = (int(spacing[0]) // sub, int(spacing[1]) // sub)
    s0 = (int(start[0]), int(start[1]))

    def surface(x, y):
        rx = (np.asarray(x, float) + offset[0] - s0[0]) / sv[0]
        ry = (np.asarray(y, float) + offset[1] - s0[1]) / sv[1]
        i = np.clip(np.floor(rx), 0, nv - 2).astype(int)
        j = np.clip(np.floor(ry), 0, nv - 2).astype(int)
        u, w = rx - i, ry - j
        a, b, c, d = virt[i, j], virt[i + 1, j], virt[i, j + 1], virt[i + 1, j + 1]
        return (a * (1 - u) + b * u) * (1 - w) + (c * (1 - u) + d * u) * w - ref

    return surface


# -- synthetic photos, for the self-test and the tests ---------------------------------------
def render_photo(raster, rgeom, model, head, o, h, d_bed=0.0, background=128, surface=None):
    """What the camera at head sees of a raster in LB coordinates, with the raster's pixel
    (i, j) centre at LB (x0 + (i + 0.5) / dpm, y1 - (j + 0.5) / dpm), flat at h or on
    surface(x, y) (B frame)."""
    np, cv2 = _np(), _cv()
    us, vs = np.meshgrid(np.arange(model.width, dtype=float), np.arange(model.height, dtype=float))
    px = np.column_stack([us.ravel(), vs.ravel()])
    plane = model.to_plane(head, px, h, d_bed)
    if surface is not None:
        # the first surface each ray meets from above: start at the highest height and step down
        # (stepping up from the bed would find the bed behind a tall object's edge)
        hs = surface(plane[:, 0], plane[:, 1])
        top = float(np.max(hs))
        hs = np.full(len(px), top)
        for _ in range(8):
            plane = model.to_plane(head, px, hs, d_bed)
            seen = surface(plane[:, 0], plane[:, 1])
            if np.allclose(seen, hs):
                break
            hs = np.minimum(hs, seen)
    lx, ly = plane[:, 0] - o[0], plane[:, 1] - o[1]
    fi = ((lx - rgeom["x0"]) * rgeom["dpm"] - 0.5).reshape(us.shape).astype(np.float32)
    fj = ((rgeom["y1"] - ly) * rgeom["dpm"] - 0.5).reshape(us.shape).astype(np.float32)
    # the photo samples the raster more coarsely: blur first so it does not alias
    px_mm = model.f / (head[2] + model.e - d_bed - h)
    src = cvio.blur(raster, 0.5 * rgeom["dpm"] / px_mm)
    out = cv2.remap(src, fi, fj, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                    borderValue=(background,) * 3)
    return out if out.ndim == 3 else np.dstack([out] * 3)


def dot_raster(rect, dpm, dots, radius=1.0, check=5.0):
    """A white raster over LB rect with a faint check of check mm and black dots at dots."""
    np, cv2 = _np(), _cv()
    x0, y0, x1, y1 = rect
    w, h = int(round((x1 - x0) * dpm)), int(round((y1 - y0) * dpm))
    xs = x0 + (np.arange(w) + 0.5) / dpm
    ys = y1 - (np.arange(h) + 0.5) / dpm
    gx, gy = np.meshgrid(np.floor(xs / check), np.floor(ys / check))
    img = np.where((gx + gy) % 2 == 0, 235, 200).astype(np.uint8)
    for x, y in dots:
        cv2.circle(img, (int(round((x - x0) * dpm * 16 - 8)), int(round((y1 - y) * dpm * 16 - 8))),
                   int(round(radius * dpm * 16)), 0, -1, lineType=cv2.LINE_AA, shift=4)
    return np.dstack([img] * 3), dict(x0=float(x0), y1=float(y1), dpm=float(dpm))


def find_dots(canvas, geom, dots, window=3.0):
    """The darkness-weighted centre (LB mm) of each expected dot, searched within window mm."""
    np = _np()
    found = []
    for x, y in dots:
        i0, i1, j0, j1 = px_rect(geom, (x - window, y - window, x + window, y + window))
        g = cvio.gray(canvas[j0:j1, i0:i1])
        d = np.clip(np.percentile(g, 90) - g, 0, None)
        d[d < 0.5 * d.max()] = 0
        xs, ys = lb_grid(geom, i0, i1, j0, j1)
        found.append((float((d * xs).sum() / d.sum()), float((d * ys).sum() / d.sum())))
    return found


def selftest(model, dpm=15.0, o=(4.0, -2.5)):
    """Synthetic end-to-end on this machine's numpy and OpenCV: a 2 x 2 close-up and an
    overview of a dot pattern, through the given camera model. Returns the errors in mm."""
    import time
    from . import closeup_plan
    np = _np()
    start = time.time()
    geom = canvas_geometry(dpm)
    box = (115.0, 125.0, 205.0, 225.0)                   # 2 x 2 tiles at Z50
    dots = [(x, y) for x in (118.0, 160.0, 202.0) for y in (128.0, 175.0, 222.0)]
    raster, rgeom = dot_raster((100.0, 110.0, 220.0, 240.0), 40.0, dots)
    over_head = (232.0, 178.0, 290.0)
    overview = cvio.decode(cvio.encode(render_photo(raster, rgeom, model, over_head, o, 0.0), 95))
    base = base_canvas(overview, over_head, model, o, geom)
    ctx = dict(o=o, limits=((0.0, 0.0), (330.0, 350.0)), current=over_head, hood_touch_z=11.4, d_bed=0.0,
               overlap=0.25, min_clearance=10.0, max_height=60.0, hood_radius=5.0, max_tiles=25,
               capture_s=6.0, margin_px=MARGIN_PX, standoff_range=(35.0, 150.0))
    p = closeup_plan.plan(box, 0.0, 50.0, ctx, model)
    tiles = []
    for t in p["tiles"]:
        head = (t["x"], t["y"], t["z"])
        tiles.append(dict(head=head, img=cvio.decode(cvio.encode(render_photo(raster, rgeom, model, head, o, 0.0), 95))))
    canvas, qa = compose(base, tiles, model, o, 0.0, geom)
    err_close = [np.hypot(a - x, b - y) for (a, b), (x, y) in zip(find_dots(canvas, geom, dots), dots)]
    err_base = [np.hypot(a - x, b - y) for (a, b), (x, y) in zip(find_dots(base, geom, dots, 4.0), dots)]
    return dict(tiles=len(tiles), close_max_mm=float(max(err_close)), base_max_mm=float(max(err_base)),
                seam_mm=qa["seam_mm"], seconds=round(time.time() - start, 2), canvas=(geom["W"], geom["H"]))
