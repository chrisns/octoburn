"""The close-up image: photos placed on the bed in LightBurn coordinates.

Synthetic end to end: a dot pattern on the bed is photographed through a realistic camera model
(offset principal point, small tilt, barrel distortion, the camera 57 mm beside the beam), each
photo goes through JPEG, and the composite must put every dot where it is on the bed.
"""
import numpy as np
import pytest

from lhp_pkg import camera_model as cm
from lhp_pkg import closeup_plan as cp
from lhp_pkg import compositor as co
from lhp_pkg import cvio
from lhp_pkg import lightburn_camera as lbc

O = (4.0, -2.5)
DOTS = [(x, y) for x in (118.0, 160.0, 202.0) for y in (128.0, 175.0, 222.0)]
BOX = (115.0, 125.0, 205.0, 225.0)        # 90 x 100 mm: 2 x 2 tiles at Z50
OVER = (232.0, 178.0, 290.0)


def model():
    m = cm.CameraModel(f=812.3, cx=546.0, cy=629.0, k1=-0.05, k2=0.012, rvec=(0.003, -0.002, 0.011),
                       e=4.7, t=(-56.8, 6.3))
    m.z_range = (40.0, 290.0)
    return m


def ctx():
    return dict(o=O, limits=((0.0, 0.0), (330.0, 350.0)), current=OVER, hood_touch_z=11.4, d_bed=0.0,
                overlap=0.25, min_clearance=10.0, max_height=60.0, hood_radius=5.0, max_tiles=25,
                capture_s=6.0, margin_px=co.MARGIN_PX, standoff_range=(35.0, 150.0))


@pytest.fixture(scope="module")
def scene():
    m = model()
    raster, rgeom = co.dot_raster((100.0, 110.0, 220.0, 240.0), 40.0, DOTS)
    jpeg = lambda img: cvio.decode(cvio.encode(img, 95))
    overview = jpeg(co.render_photo(raster, rgeom, m, OVER, O, 0.0))
    plan = cp.plan(BOX, 0.0, 50.0, ctx(), m)
    tiles = [dict(head=(t["x"], t["y"], t["z"]),
                  img=jpeg(co.render_photo(raster, rgeom, m, (t["x"], t["y"], t["z"]), O, 0.0)))
             for t in plan["tiles"]]
    raised = [dict(head=t["head"], img=jpeg(co.render_photo(raster, rgeom, m, t["head"], O, 1.0)))
              for t in tiles]                                    # the same, 1 mm higher than stated
    return dict(m=m, raster=raster, rgeom=rgeom, overview=overview, plan=plan, tiles=tiles, raised=raised)


def test_the_canvas_matches_the_lightburn_camera_and_refuses_a_giant():
    g = co.canvas_geometry(15)
    assert (g["W"], g["H"]) == (4800, 5250)
    xs, ys = co.lb_grid(g, 0, 2, 0, 2)
    u, v = lbc.canvas_px(xs[0, 0], ys[0, 0], 15)
    assert (u, v) == pytest.approx((0, 0))
    with pytest.raises(cm.CloseupError, match="megapixels"):
        co.canvas_geometry(25)


def test_the_overview_alone_puts_the_dots_within_half_a_millimetre(scene):
    g = co.canvas_geometry(10)
    base = co.base_canvas(scene["overview"], OVER, scene["m"], O, g)
    found = co.find_dots(base, g, DOTS, window=4.0)
    err = [np.hypot(a - x, b - y) for (a, b), (x, y) in zip(found, DOTS)]
    assert max(err) <= 0.5


def test_the_close_up_puts_every_dot_within_a_tenth_of_a_millimetre(scene):
    g = co.canvas_geometry(15)
    base = co.base_canvas(scene["overview"], OVER, scene["m"], O, g)
    canvas, qa = co.compose(base, scene["tiles"], scene["m"], O, 0.0, g)
    assert qa["tiles"] == len(scene["tiles"]) >= 4
    found = co.find_dots(canvas, g, DOTS)
    err = [np.hypot(a - x, b - y) for (a, b), (x, y) in zip(found, DOTS)]
    assert max(err) <= 0.1
    assert qa["seam_mm"] is not None and qa["seam_mm"] <= 0.1
    # outside the close-up the canvas is still the overview, untouched
    i0, i1, j0, j1 = qa["rect_px"]
    assert np.array_equal(canvas[:j0], base[:j0]) and np.array_equal(canvas[:, :i0], base[:, :i0])


def test_the_close_up_is_sharper_than_the_overview(scene):
    g = co.canvas_geometry(15)
    base = co.base_canvas(scene["overview"], OVER, scene["m"], O, g)
    canvas, _ = co.compose(base, scene["tiles"], scene["m"], O, 0.0, g)
    i0, i1, j0, j1 = co.px_rect(g, (150.0, 165.0, 170.0, 185.0))
    sharp = lambda img: float(np.abs(np.diff(cvio.gray(img[j0:j1, i0:i1]), axis=1)).max())
    assert sharp(canvas) > 2.0 * sharp(base)


def test_a_wrong_height_shows_up_as_a_seam_mismatch(scene):
    g = co.canvas_geometry(15)
    base = co.base_canvas(scene["overview"], OVER, scene["m"], O, g)
    _, qa = co.compose(base, scene["raised"], scene["m"], O, 0.0, g)
    assert qa["seam_mm"] > 0.3


def test_the_right_height_puts_a_raised_pattern_back_in_place(scene):
    g = co.canvas_geometry(15)
    base = co.base_canvas(scene["overview"], OVER, scene["m"], O, g)
    canvas, qa = co.compose(base, scene["raised"], scene["m"], O, 1.0, g)
    found = co.find_dots(canvas, g, DOTS)
    assert max(np.hypot(a - x, b - y) for (a, b), (x, y) in zip(found, DOTS)) <= 0.1
    assert qa["seam_mm"] <= 0.1


def test_the_self_test_runs_end_to_end():
    out = co.selftest(model(), dpm=10)
    assert out["tiles"] >= 4 and out["close_max_mm"] <= 0.1 and out["base_max_mm"] <= 0.5
    assert out["canvas"] == (3200, 3500)


def test_an_uneven_bed_is_followed_when_its_surface_is_given():
    # The A350's bed is about 1.8 mm out across its width (the height map): the camera's true
    # distance to each point changes with it. Given the surface, the dots land in place.
    m = model()
    surface = lambda x, y: 0.012 * (x - 100.0) + 0.006 * (y - 110.0)      # up to ~1.8 mm
    raster, rgeom = co.dot_raster((100.0, 110.0, 220.0, 240.0), 40.0, DOTS)
    jpeg = lambda img: cvio.decode(cvio.encode(img, 95))
    over = jpeg(co.render_photo(raster, rgeom, m, OVER, O, 0.0, surface=surface))
    plan = cp.plan(BOX, 0.0, 50.0, ctx(), m)
    tiles = [dict(head=(t["x"], t["y"], t["z"]),
                  img=jpeg(co.render_photo(raster, rgeom, m, (t["x"], t["y"], t["z"]), O, 0.0, surface=surface)))
             for t in plan["tiles"]]
    g = co.canvas_geometry(15)
    err = lambda canvas, window=3.0: max(np.hypot(a - x, b - y) for (a, b), (x, y)
                                         in zip(co.find_dots(canvas, g, DOTS, window), DOTS))
    flat_base = co.base_canvas(over, OVER, m, O, g)
    true_base = co.base_canvas(over, OVER, m, O, g, surface=surface)
    assert err(true_base, 4.0) <= 0.5 and err(true_base, 4.0) < err(flat_base, 4.0)
    canvas, qa = co.compose(true_base, tiles, m, O, 0.0, g, surface=surface)
    assert err(canvas) <= 0.1 and qa["seam_mm"] <= 0.1
    flat, qa_flat = co.compose(flat_base, tiles, m, O, 0.0, g)
    assert err(flat) > err(canvas)


def test_the_bed_surface_from_the_height_map_matches_the_maps_own_evaluation():
    from lhp_pkg import heightmap
    rng = np.random.default_rng(2)
    nodes = (rng.random((11, 11)) * 1.8).tolist()
    start, spacing, off, ref = (42.0, 34.0), (27.0, 29.0), (19.0, 10.0), 0.458
    surface = co.surface_from_map(nodes, start, spacing, off, ref)
    xs = np.array([10.0, 60.0, 133.0, 250.0, 300.0, 330.0])       # inside and past the grid
    ys = np.array([5.0, 116.0, 174.0, 260.0, 320.0, 340.0])
    got = surface(xs, ys)
    want = [heightmap.evaluate(nodes, start, spacing, x + off[0], y + off[1], extrapolate=True) - ref
            for x, y in zip(xs, ys)]
    assert np.allclose(got, want, atol=1e-9)
    grid_x, grid_y = np.meshgrid(xs, ys)
    assert surface(grid_x, grid_y).shape == grid_x.shape


def test_the_heads_shadow_in_each_tile_is_evened_out_against_the_overview(scene):
    # Seen 2026-11-21: at Z50 the head shades the same band of every photo, which striped the
    # close-up. The overview (Z290) has no shadow, so each tile is levelled against it, and a
    # shadowed pixel counts less where a neighbour's photo also sees that spot.
    g = co.canvas_geometry(15)
    base = co.base_canvas(scene["overview"], OVER, scene["m"], O, g)
    # planned as the plugin does, leaving the photo's right 40 % to a lit neighbour
    plan = cp.plan(BOX, 0.0, 50.0, dict(ctx(), margin_px=(40, 40, 450, 40)), scene["m"])
    shaded = []
    for t in plan["tiles"]:
        head = (t["x"], t["y"], t["z"])
        img = co.render_photo(scene["raster"], scene["rgeom"], scene["m"], head, O, 0.0)
        img = cvio.decode(cvio.encode(img, 95)).astype(float) * 0.7   # darker exposure than the overview
        img[:, 700:] *= 0.4                                       # a dark band down the photo's right side
        shaded.append(dict(head=head, img=np.clip(img, 0, 255).astype(np.uint8)))
    canvas, qa = co.compose(base, shaded, scene["m"], O, 0.0, g)
    found = co.find_dots(canvas, g, DOTS)
    assert max(np.hypot(a - x, b - y) for (a, b), (x, y) in zip(found, DOTS)) <= 0.1
    i0, i1, j0, j1 = qa["rect_px"]
    ours = cvio.gray(canvas[j0:j1, i0:i1])
    ref = cvio.gray(base[j0:j1, i0:i1])
    smooth = lambda a: cvio.blur(a, 60)
    ratio = smooth(ours) / np.maximum(smooth(ref), 1.0)
    assert ratio.min() > 0.93 and ratio.max() < 1.07           # no stripes or fringes left


def test_a_seam_check_over_blank_paper_says_nothing_rather_than_nonsense():
    m = model()
    blank = np.full((1280, 1024, 3), 210, np.uint8)
    noise = np.random.default_rng(4).normal(0, 2, blank.shape)
    tiles = [dict(head=(200.0, 180.0, 50.0), img=np.clip(blank + noise, 0, 255).astype(np.uint8)),
             dict(head=(240.0, 180.0, 50.0), img=np.clip(blank + noise[::-1], 0, 255).astype(np.uint8))]
    g = co.canvas_geometry(10)
    base = np.full((g["H"], g["W"], 3), 210, np.uint8)
    _, qa = co.compose(base, tiles, m, O, 0.0, g)
    assert qa["seam_mm"] is None or qa["seam_mm"] < 0.5


def _dark_square(raster, rgeom, rect, value=30):
    out = raster.copy()
    x0, y0, x1, y1 = rect
    i0, i1 = int((x0 - rgeom["x0"]) * rgeom["dpm"]), int((x1 - rgeom["x0"]) * rgeom["dpm"])
    j0, j1 = int((rgeom["y1"] - y1) * rgeom["dpm"]), int((rgeom["y1"] - y0) * rgeom["dpm"])
    out[j0:j1, i0:i1] = value
    return out


def _mean_in(canvas, g, rect):
    i0, i1, j0, j1 = co.px_rect(g, rect)
    return float(cvio.gray(canvas[j0:j1, i0:i1]).mean())


def test_a_dark_object_the_overview_never_saw_is_kept_not_taken_for_shadow(scene):
    # The overview is older than the object (placed after it was taken): the close-up must show
    # the object, not level it away as "shadow".
    m = scene["m"]
    square = (150.0, 160.0, 175.0, 190.0)                    # 25 x 30 mm, dark, in the box
    raster = _dark_square(scene["raster"], scene["rgeom"], square)
    plan = cp.plan(BOX, 0.0, 50.0, dict(ctx(), margin_px=(40, 40, 450, 40)), m)
    tiles = []
    for t in plan["tiles"]:
        head = (t["x"], t["y"], t["z"])
        img = cvio.decode(cvio.encode(co.render_photo(raster, scene["rgeom"], m, head, O, 0.0), 95)).astype(float)
        img[:, 700:] *= 0.4                                  # and the head's shadow as usual
        tiles.append(dict(head=head, img=np.clip(img, 0, 255).astype(np.uint8)))
    g = co.canvas_geometry(10)
    base = co.base_canvas(scene["overview"], OVER, m, O, g)  # the old overview: no square
    canvas, qa = co.compose(base, tiles, m, O, 0.0, g)
    inner = (155.0, 165.0, 170.0, 185.0)
    assert _mean_in(canvas, g, inner) < 60                   # still dark (value 30), not levelled to paper
    found = co.find_dots(canvas, g, [d for d in DOTS if not (150 <= d[0] <= 175 and 160 <= d[1] <= 190)])
    assert found                                             # and the rest is still in place


def test_a_tall_dark_object_seen_by_parallax_in_the_overview_is_kept(scene):
    # A dark plate 20 mm tall: the overview (Z290, bed plane) sees its top up to ~7 mm off, so
    # the overview disagrees with the close-up around it. The close-up (at height 20) keeps it.
    m = scene["m"]
    plate = (145.0, 155.0, 180.0, 195.0)
    raster = _dark_square(scene["raster"], scene["rgeom"], plate, value=40)
    surface = lambda x, y: np.where((x - O[0] >= plate[0]) & (x - O[0] <= plate[2]) &
                                    (y - O[1] >= plate[1]) & (y - O[1] <= plate[3]), 20.0, 0.0)
    over = cvio.decode(cvio.encode(co.render_photo(raster, scene["rgeom"], m, OVER, O, 0.0, surface=surface), 95))
    plan = cp.plan(plate, 20.0, 50.0, dict(ctx(), margin_px=(40, 40, 450, 40)), m)
    tiles = [dict(head=(t["x"], t["y"], t["z"]),
                  img=cvio.decode(cvio.encode(co.render_photo(raster, scene["rgeom"], m, (t["x"], t["y"], t["z"]),
                                                              O, 0.0, surface=surface), 95)))
             for t in plan["tiles"]]
    g = co.canvas_geometry(10)
    base = co.base_canvas(over, OVER, m, O, g)
    canvas, qa = co.compose(base, tiles, m, O, 20.0, g, overview=dict(img=over, head=OVER))
    assert _mean_in(canvas, g, (150.0, 160.0, 175.0, 190.0)) < 70     # the plate's top stays dark
    assert _mean_in(canvas, g, (182.0, 160.0, 186.0, 190.0)) > 150    # and the paper beside it stays light


def test_a_shadow_wider_than_the_default_band_is_found_in_each_photo(scene):
    # Seen 2026-11-21: the shadow's edge moves with the head (column 660 at head X144, 846 at
    # X242), so near the left of the bed it starts left of the default 40 % band.
    g = co.canvas_geometry(15)
    base = co.base_canvas(scene["overview"], OVER, scene["m"], O, g)
    plan = cp.plan(BOX, 0.0, 50.0, dict(ctx(), margin_px=(40, 40, 600, 40)), scene["m"])
    shaded = []
    for t in plan["tiles"]:
        head = (t["x"], t["y"], t["z"])
        img = cvio.decode(cvio.encode(co.render_photo(scene["raster"], scene["rgeom"], scene["m"], head, O, 0.0), 95))
        img = img.astype(float)
        img[:, 480:] *= 0.4                                   # shadow from column 480 (default band: 614)
        shaded.append(dict(head=head, img=np.clip(img, 0, 255).astype(np.uint8)))
    canvas, qa = co.compose(base, shaded, scene["m"], O, 0.0, g)
    i0, i1, j0, j1 = qa["rect_px"]
    smooth = lambda a: cvio.blur(a, 60)
    ratio = smooth(cvio.gray(canvas[j0:j1, i0:i1])) / np.maximum(smooth(cvio.gray(base[j0:j1, i0:i1])), 1.0)
    assert ratio.min() > 0.93 and ratio.max() < 1.07


def test_the_plan_leaves_more_of_the_photo_to_neighbours_near_the_left_of_the_bed():
    from lhp_pkg import closeup_plan as cplan
    assert cplan.shadow_frac(70.0, 50.0) > cplan.shadow_frac(240.0, 50.0)
    assert 0.45 <= cplan.shadow_frac(70.0, 50.0) <= 0.6
    assert cplan.shadow_frac(330.0, 50.0) >= 0.15


def test_the_soft_edge_of_the_shadow_leaves_no_stripes_where_a_neighbour_saw_it_lit(scene):
    # Seen 2026-11-21 (15 tiles over paper): 2-3 % grey stripes where each tile's shadow began,
    # from the soft edge (about 100 px) before the hard shadow
    g = co.canvas_geometry(15)
    base = co.base_canvas(scene["overview"], OVER, scene["m"], O, g)
    plan = cp.plan(BOX, 0.0, 50.0, dict(ctx(), margin_px=(40, 40, 40 + int(0.45 * 1024), 40)), scene["m"])
    assert plan["nx"] >= 2
    soft = np.ones(1024)
    soft[500:600] = np.linspace(1.0, 0.9, 100)
    soft[600:] = 0.45
    shaded = []
    for t in plan["tiles"]:
        head = (t["x"], t["y"], t["z"])
        img = cvio.decode(cvio.encode(co.render_photo(scene["raster"], scene["rgeom"], scene["m"], head, O, 0.0), 95))
        shaded.append(dict(head=head, img=np.clip(img.astype(float) * soft[None, :, None], 0, 255).astype(np.uint8)))
    canvas, qa = co.compose(base, shaded, scene["m"], O, 0.0, g)
    i0, i1, j0, j1 = qa["rect_px"]
    smooth = lambda a: cvio.blur(a, 60)
    ratio = smooth(cvio.gray(canvas[j0:j1, i0:i1])) / np.maximum(smooth(cvio.gray(base[j0:j1, i0:i1])), 1.0)
    inner = ratio[60:-60, 60:-60]
    assert inner.min() > 0.975 and inner.max() < 1.025
