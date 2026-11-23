"""Where the head goes for close-up photos: calibration poses and (later) the tile plan.

Plain Python against a model that only needs centre_offset(z, h); head = target - centre_offset,
rounded to 0.1 mm because the touchscreen is sent X, Y, Z to one decimal (headcam.request_url)
and must find the head already there.
"""
import pytest

from lhp_pkg import closeup_plan as cp


class Offset(object):
    """A stand-in camera model: the view centre is (-57, +6) from the head at every height."""
    def centre_offset(self, z, h, margin=0, d_bed=0.0):
        return -57.0, 6.0 + 0.02 * z


def test_board_poses_cover_every_height_and_hold_some_out():
    poses = cp.calib_poses("board", (160.0, 175.0), Offset())
    zs = sorted(set(p["z"] for p in poses))
    assert zs == [40.0, 45.0, 50.0, 60.0, 80.0, 120.0, 200.0, 290.0]
    assert len(poses) == 5 * 9 + 5 + 3 + 3
    held = [p for p in poses if p["holdout"]]
    assert all(p["holdout"] for p in poses if p["z"] == 45.0)          # the whole Z45 level
    for z in (40.0, 50.0, 60.0, 80.0):
        assert sum(1 for p in held if p["z"] == z) == 1                # plus one corner per level
    assert len([p for p in poses if not p["holdout"]]) >= 40


def test_a_pose_puts_the_view_centre_on_the_target_plus_its_offset():
    poses = cp.calib_poses("board", (160.0, 175.0), Offset())
    centre = [p for p in poses if p["z"] == 50.0 and p["dx"] == 0 and p["dy"] == 0][0]
    assert (centre["x"], centre["y"]) == pytest.approx((160.0 + 57.0, 175.0 - 6.0 - 1.0))
    wide = [p for p in poses if p["z"] == 80.0]
    assert max(p["dx"] for p in wide) == 25.0 and min(p["dy"] for p in wide) == -25.0
    for p in poses:
        assert round(p["x"], 1) == p["x"] and round(p["y"], 1) == p["y"] and round(p["z"], 1) == p["z"]


def test_cross_poses_photograph_each_cross_from_four_places():
    crosses = [(44.0, 92.5), (284.0, 92.5), (44.0, 252.5), (284.0, 252.5)]   # B frame = LB + o
    poses = cp.calib_poses("cross", crosses, Offset())
    assert len(poses) == 16
    for n in range(4):
        mine = [p for p in poses if p["cross"] == n]
        assert sorted((p["z"], p["dx"], p["dy"]) for p in mine) == [
            (50.0, 0.0, 0.0), (50.0, 0.0, 15.0), (50.0, 15.0, 0.0), (60.0, 0.0, 0.0)]


def cam():
    from lhp_pkg import camera_model as cm
    m = cm.CameraModel(f=814.0, cx=512.0, cy=640.0, e=5.0, t=(-57.0, 6.0))
    m.z_range = (40.0, 290.0)
    return m


def ctx(**kw):
    base = dict(o=(4.0, -2.5), limits=((0.0, 0.0), (330.0, 350.0)), current=(232.0, 178.0, 290.0),
                hood_touch_z=11.4, d_bed=0.0, overlap=0.25, min_clearance=10.0, max_height=60.0,
                hood_radius=5.0, max_tiles=25, capture_s=6.0, margin_px=40, standoff_range=(35.0, 150.0))
    base.update(kw)
    return base


def test_a_coin_sized_box_is_one_tile_centred_on_it():
    p = cp.plan((140.0, 160.0, 165.0, 185.0), 0.0, 50.0, ctx(), cam())
    assert (p["nx"], p["ny"]) == (1, 1) and p["head_z"] == 50.0 and p["travel_z"] == 290.0
    t = p["tiles"][0]
    m = cam()
    ox, oy = m.centre_offset(50.0, 0.0, 40)
    # the tile's view centre is on the box centre in the B frame (LB + o)
    assert (t["x"], t["y"]) == pytest.approx((152.5 + 4.0 - ox, 172.5 - 2.5 - oy), abs=0.051)
    assert round(t["x"], 1) == t["x"] and round(t["y"], 1) == t["y"] and t["z"] == 50.0
    fx0, fy0, fx1, fy1 = t["view_lb"]
    assert fx0 <= 140.0 and fx1 >= 165.0 and fy0 <= 160.0 and fy1 >= 185.0
    assert p["seconds"] > 6.0


def test_a_wide_box_is_tiled_with_overlap_and_every_point_is_seen():
    box = (60.0, 80.0, 200.0, 190.0)
    p = cp.plan(box, 0.0, 50.0, ctx(), cam())
    assert p["nx"] >= 3 and p["ny"] >= 2 and len(p["tiles"]) == p["nx"] * p["ny"]
    for x in (box[0], (box[0] + box[2]) / 2, box[2]):
        for y in (box[1], (box[1] + box[3]) / 2, box[3]):
            assert any(v[0] <= x <= v[2] and v[1] <= y <= v[3] for v in (t["view_lb"] for t in p["tiles"]))
    xs = sorted(set(t["view_lb"][0] for t in p["tiles"]))
    w = p["tiles"][0]["view_lb"][2] - p["tiles"][0]["view_lb"][0]
    assert xs[1] - xs[0] <= 0.75 * w + 0.11                   # at least 25 % overlap


def test_tiles_snake_from_the_row_nearest_the_head():
    p = cp.plan((60.0, 80.0, 200.0, 190.0), 0.0, 50.0, ctx(current=(100.0, 20.0, 290.0)), cam())
    ys = [t["y"] for t in p["tiles"]]
    assert ys[0] == min(ys)                                     # the head is near the front
    first_row = [t for t in p["tiles"] if t["y"] == ys[0]]
    second_row = [t for t in p["tiles"] if t["y"] != ys[0] and t["y"] == min(y for y in ys if y != ys[0])]
    assert [t["x"] for t in first_row] == sorted(t["x"] for t in first_row)
    assert [t["x"] for t in second_row] == sorted((t["x"] for t in second_row), reverse=True)


def test_the_object_height_raises_the_camera_and_moves_the_view():
    flat = cp.plan((140.0, 160.0, 165.0, 185.0), 0.0, 50.0, ctx(), cam())
    tall = cp.plan((140.0, 160.0, 165.0, 185.0), 20.0, 50.0, ctx(), cam())
    assert tall["head_z"] == 70.0
    assert flat["tiles"][0]["x"] == pytest.approx(tall["tiles"][0]["x"], abs=0.11)


def test_the_module_sweep_covers_the_modules_outline_from_its_cad_at_every_beam_position():
    # Snap-LASER-10W-ASSY.STEP: the body reaches 74 mm to -X of the beam (the camera side), 40 to
    # +X and 30/37 front and back; the setting adds a margin all round
    p = cp.plan((60.0, 80.0, 200.0, 190.0), 0.0, 50.0, ctx(), cam())
    x0, y0, x1, y1 = p["sweep_lb"]
    m = 5.0
    for t in p["tiles"]:
        bx, by = t["x"] - 4.0, t["y"] + 2.5                     # the beam under the head, in LB
        assert x0 <= bx - 74.0 - m and bx + 40.0 + m <= x1 and y0 <= by - 37.0 - m and by + 37.0 + m <= y1
    assert cp.MODULE_REACH == (74.0, 37.0, 40.0, 37.0)


@pytest.mark.parametrize("change,args,message", [
    ({}, dict(box=(300.0, 160.0, 320.0, 185.0)), "X limit"),                 # the camera is 57 mm left
    ({}, dict(box=(140.0, 160.0, 140.5, 185.0)), "at least 2 mm"),
    ({}, dict(box=(-5.0, 160.0, 10.0, 185.0)), "inside the bed"),
    ({}, dict(h=None), "height"),
    ({}, dict(h=80.0), "height"),
    ({}, dict(standoff=20.0), "stand-off"),
    (dict(hood_touch_z=45.0), {}, "clearance"),
    (dict(max_tiles=2), dict(box=(60.0, 80.0, 200.0, 190.0)), "tiles"),
    ({}, dict(h=40.0, standoff=150.0), "calibrated"),            # Z190, above a model calibrated to Z120
])
def test_unsafe_or_impossible_close_ups_are_refused(change, args, message):
    box = args.get("box", (140.0, 160.0, 165.0, 185.0))
    h = args.get("h", 0.0)
    standoff = args.get("standoff", 50.0)
    model = cam()
    if message == "calibrated":
        model.z_range = (40.0, 120.0)
    with pytest.raises(ValueError, match=message):
        cp.plan(box, h, standoff, ctx(**change), model)


def test_the_refusal_for_the_right_hand_strip_says_what_can_be_reached():
    with pytest.raises(ValueError) as caught:
        cp.plan((300.0, 160.0, 320.0, 185.0), 0.0, 50.0, ctx(), cam())
    assert "LightBurn X up to" in str(caught.value)


def test_an_unknown_stage_is_refused():
    with pytest.raises(ValueError, match="stage"):
        cp.calib_poses("sky", (0, 0), Offset())


def test_a_shadow_band_on_the_photos_right_is_covered_by_the_next_tile():
    box = (60.0, 80.0, 200.0, 190.0)
    plain = cp.plan(box, 0.0, 50.0, ctx(), cam())
    shaded = cp.plan(box, 0.0, 50.0, ctx(margin_px=(40, 40, 450, 40)), cam())
    assert shaded["nx"] > plain["nx"]
    for x in (box[0], (box[0] + box[2]) / 2, box[2]):          # every point is in some tile's lit part
        for y in (box[1], (box[1] + box[3]) / 2, box[3]):
            assert any(v[0] <= x <= v[2] and v[1] <= y <= v[3] for v in (t["view_lb"] for t in shaded["tiles"]))


def test_shadow_aware_columns_cover_the_box_and_widen_to_the_right():
    c = dict(ctx(), margin_px=(40, 40, 40 + int(0.4 * 1024), 40), shadow=True)
    p = cp.plan((50.0, 90.0, 210.0, 260.0), 0.0, 50.0, c, cam())
    cols = sorted({t["view_lb"][0]: t["view_lb"] for t in p["tiles"]}.values())
    assert cols[0][0] <= 50.0 and cols[-1][2] >= 210.0
    for a, b in zip(cols, cols[1:]):
        assert b[0] < a[2] - 0.2 * (a[2] - a[0]) + 0.5           # neighbours overlap
    widths = [v[2] - v[0] for v in cols]
    assert widths[-1] > widths[0]
    assert len(p["tiles"]) <= 25                                 # the 2026-11-21 18-tile box still fits


@pytest.mark.parametrize("box", [(140.0, 160.0, 165.0, 185.0), (5.0, 55.0, 55.0, 105.0), (60.0, 80.0, 200.0, 190.0)])
def test_every_tile_view_of_a_shadow_aware_plan_lies_where_the_camera_actually_looks(box):
    m = cam()
    c = dict(ctx(), margin_px=(40, 40, 40 + int(0.4 * 1024), 40), shadow=True)
    p = cp.plan(box, 0.0, 50.0, c, m)
    for t in p["tiles"]:
        fx0, fy0, fx1, fy1 = m.footprint(t["z"], 0.0, 40, 0.0)
        vx0, vy0, vx1, vy1 = t["view_lb"]
        assert t["x"] - 4.0 + fx0 - 0.2 <= vx0 and vx1 <= t["x"] - 4.0 + fx1 + 0.2
        assert t["y"] + 2.5 + fy0 - 0.2 <= vy0 and vy1 <= t["y"] + 2.5 + fy1 + 0.2
    xs = sorted({t["view_lb"][0] for t in p["tiles"]})
    assert min(t["view_lb"][0] for t in p["tiles"]) <= box[0] + 1e-6
    assert max(t["view_lb"][2] for t in p["tiles"]) >= box[2] - 1e-6


def test_the_time_estimate_counts_the_fresh_overview_and_the_compose():
    a = cp.plan((140.0, 160.0, 165.0, 185.0), 0.0, 50.0, ctx(capture_s=25.0), cam())
    b = cp.plan((140.0, 160.0, 165.0, 185.0), 0.0, 50.0, ctx(capture_s=25.0, extra_s=25.0), cam())
    assert b["seconds"] - a["seconds"] == pytest.approx(25.0)
    assert 80.0 <= b["seconds"] <= 110.0                  # one tile took 87-105 s on the machine


def test_the_time_estimate_for_many_tiles_matches_the_machine():
    c = ctx(capture_s=25.0, extra_s=25.0, margin_px=(40, 40, 40 + int(0.4 * 1024), 40), shadow=True)
    p = cp.plan((50.0, 90.0, 210.0, 260.0), 0.0, 50.0, c, cam())
    assert 280.0 <= p["seconds"] <= 370.0, (len(p["tiles"]), p["seconds"])   # 15 tiles took 314-326 s
