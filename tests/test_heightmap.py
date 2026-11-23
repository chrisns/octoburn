"""The height map fit: probed points to the firmware's 11 x 11, 28 mm bed mesh."""
import math

import pytest

from lhp_pkg import heightmap


def grid_points(surface, x0=120.0, y0=100.0, size=60.0, n=5):
    pts = []
    for j in range(n):
        for i in range(n):
            x, y = x0 + size * i / (n - 1), y0 + size * j / (n - 1)
            pts.append(dict(x=x, y=y, z=surface(x, y), focus=surface(x, y) + 17.0))
    return pts


def inside_the_probed_heights(m, slack=0.002, check=True):
    """Over the probed rectangle the firmware's interpolation stays within the probed heights."""
    xs = [s["machine_x"] for s in m["samples"]]
    ys = [s["machine_y"] for s in m["samples"]]
    for a in range(11):
        for b in range(11):
            x = min(xs) + (max(xs) - min(xs)) * a / 10.0
            y = min(ys) + (max(ys) - min(ys)) * b / 10.0
            v = heightmap.evaluate(m["nodes"], m["start"], m["spacing"], x, y)
            if check:
                assert -slack <= v <= m["range"] + slack, (x, y, v)


def test_geometry_is_the_firmwares():
    assert heightmap.firmware_geometry(11) == ((52.0, 43.0), (28.0, 28.0))
    assert heightmap.firmware_geometry(5) == ((52.0, 43.0), (70.0, 71.0))


def test_a_plane_fits_exactly_and_is_based_at_the_lowest_point():
    plane = lambda x, y: 10.0 + 0.05 * x + 0.02 * y                  # noqa: E731
    m = heightmap.build(grid_points(plane), (0.0, 0.0), focus_gap=40.0)
    assert m["base"]["x"] == 120.0 and m["base"]["y"] == 100.0       # the lowest corner
    assert m["base"]["z"] == pytest.approx(plane(120, 100))
    assert m["range"] == pytest.approx(0.05 * 60 + 0.02 * 60)
    assert abs(m["worst"]["miss"]) < 0.002
    inside_the_probed_heights(m)
    for s in m["samples"]:
        got = heightmap.evaluate(m["nodes"], m["start"], m["spacing"], s["machine_x"], s["machine_y"])
        assert got == pytest.approx(s["height"], abs=0.002)
    # The object's edge is not on a grid line, so the nodes just outside carry the slope on.
    assert m["dip"] > 0.05 and any("dips up to" in w for w in m["warnings"])
    assert not any("cannot follow" in w or "focus gap" in w for w in m["warnings"])
    assert 0 < m["slope_deg"] < 5


def test_the_mesh_sits_at_machine_xy_not_gcode_xy():
    """LightBurn's start G-code moves the G-code origin; the firmware mesh does not move."""
    shifted = heightmap.build(grid_points(lambda x, y: 0.02 * x), (30.0, -20.0))
    assert shifted["base"]["machine_x"] == 150.0 and shifted["base"]["machine_y"] == 80.0
    for s in shifted["samples"]:
        assert s["machine_x"] == pytest.approx(s["x"] + 30.0)


def test_no_shift_and_outside_the_grid_are_refused():
    pts = grid_points(lambda x, y: 1.0)
    with pytest.raises(heightmap.MapError, match="shift"):
        heightmap.build(pts, None)
    with pytest.raises(heightmap.MapError, match="outside the firmware's mesh"):
        heightmap.build(grid_points(lambda x, y: 1.0, x0=10.0), (0.0, 0.0))
    with pytest.raises(heightmap.MapError, match="no measured points"):
        heightmap.build([dict(x=100.0, y=100.0, z=None)], (0.0, 0.0))


def test_a_small_dome_is_more_than_the_grid_can_follow_and_says_so():
    dome = lambda x, y: 8.0 * math.exp(-((x - 150) ** 2 + (y - 130) ** 2) / 200.0)   # noqa: E731
    m = heightmap.build(grid_points(dome), (0.0, 0.0), tolerance=0.5, focus_gap=40.0)
    assert abs(m["worst"]["miss"]) > 0.5
    assert any("cannot follow" in w for w in m["warnings"])
    inside_the_probed_heights(m, slack=0.0, check=False)


def test_focus_gap_warnings():
    tall = lambda x, y: 0.4 * (x - 120)                               # noqa: E731  range 24 mm
    assert any("focus gap in the settings" in w
               for w in heightmap.build(grid_points(tall), (0, 0))["warnings"])
    assert any("would hit the tallest part" in w
               for w in heightmap.build(grid_points(tall), (0, 0), focus_gap=20.0)["warnings"])
    assert not any("focus gap" in w
                   for w in heightmap.build(grid_points(tall), (0, 0), focus_gap=40.0)["warnings"])


def test_nodes_far_from_the_object_stay_level():
    """Away from the samples the head neither dives nor climbs: the far nodes copy the nearest
    fitted ones, which are bounded by one cell's slope past the probed heights."""
    m = heightmap.build(grid_points(lambda x, y: 0.05 * x), (0.0, 0.0))
    nodes = m["nodes"]
    assert nodes[0][0] == pytest.approx(nodes[1][1], abs=0.5)
    assert nodes[0][0] == pytest.approx(-m["dip"], abs=0.5)
    flat = [v for col in nodes for v in col]
    assert min(flat) >= -m["range"] and max(flat) <= 2 * m["range"]
    assert min(flat) == pytest.approx(-m["dip"], abs=0.001)


def test_a_single_point_is_a_flat_map_with_a_warning():
    m = heightmap.build([dict(x=150.0, y=150.0, z=12.0)], (0.0, 0.0), focus_gap=30.0)
    assert all(v == 0.0 for col in m["nodes"] for v in col)
    assert any("Only 1 point" in w for w in m["warnings"])


def test_commands_and_comparison():
    nodes = [[0.0, 1.2345], [2.0, 3.0]]
    assert heightmap.commands(nodes) == ["M421 I0 J0 Z0.000", "M421 I0 J1 Z1.234",
                                         "M421 I1 J0 Z2.000", "M421 I1 J1 Z3.000"]
    assert heightmap.same(nodes, [[0.0, 1.235], [2.0, 3.0]])
    assert not heightmap.same(nodes, [[0.0, 1.24], [2.0, 3.0]])
    assert not heightmap.same(nodes, [[0.0, 1.2345]])


def test_the_lowest_point_carries_its_focus_in_machine_z():
    pts = [dict(x=150.0, y=150.0, z=10.0, focus=27.2, machine_z=52.0),
           dict(x=180.0, y=150.0, z=11.0, focus=28.2, machine_z=53.0),
           dict(x=150.0, y=180.0, z=10.5, focus=27.7, machine_z=52.5)]
    base = heightmap.build(pts, (0.0, 0.0))["base"]
    assert base["focus"] == 27.2 and base["machine_focus"] == pytest.approx(69.2)
    assert heightmap.build([dict(x=150.0, y=150.0, z=10.0)], (0.0, 0.0))["base"]["machine_focus"] is None


def test_a_refit_leaves_out_points_beyond_the_printers_grid_and_says_so():
    # 2026-11-20: the printer's laser-mode grid ends at machine X312; the paper grid's last
    # column was at X319. Those points cannot shape the mesh (the printer holds the edge
    # heights out there), so they are left out with a warning instead of refusing the load.
    tilt = lambda x, y: 0.007 * x                             # noqa: E731
    m = heightmap.build(grid_points(tilt), (0.0, 0.0))
    xs = sorted({s["machine_x"] for s in m["samples"]})
    end_x = xs[-1] - 5.0                                      # the printer's grid stops short
    spacing = [(end_x - 42.0) / (m["grid"] - 1), m["spacing"][1]]
    again = heightmap.refit(m, [42.0, m["start"][1]], spacing)
    assert len(again["samples"]) < len(m["samples"])
    assert all(s["machine_x"] <= end_x + 1e-6 for s in again["samples"])
    assert any("outside the printer's mesh" in w for w in again["warnings"])
    assert abs(again["worst"]["miss"]) < 0.05


def test_evaluate_can_extrapolate_past_the_grid_like_the_a350():
    # Measured 2026-11-20: past the grid edge the firmware carries the edge cell's slope on
    # (machine Y7.5: 0.78 applied, 0.71 extrapolated, 0.43 clamped).
    nodes = [[0.1 * i + 0.05 * j for j in range(11)] for i in range(11)]
    start, spacing = (42.0, 34.0), (27.0, 29.0)
    inside = heightmap.evaluate(nodes, start, spacing, 100.0, 100.0)
    assert heightmap.evaluate(nodes, start, spacing, 100.0, 100.0, extrapolate=True) == pytest.approx(inside)
    past = (42.0 - 27.0, 34.0 - 29.0)                         # one cell before the first node
    assert heightmap.evaluate(nodes, start, spacing, *past) == pytest.approx(0.0)
    # in Y the A350 lays the mesh over 27 mm per point (29 // 3 * 3), so 29 mm past is 29/27 cells
    assert heightmap.evaluate(nodes, start, spacing, *past, extrapolate=True) == \
        pytest.approx(-0.1 - 0.05 * 29.0 / 27.0)



# Measured on the A350 on 2026-11-21: the probed paper map's nodes as loaded, and the lift the
# printer applied (step counter minus commanded Z) at 24 spots (machine X, Y, lift).
A350_NODES = [[0.428, 0.331, 0.22, 0.068, -0.013, -0.002, 0.065, 0.217, 0.435, 0.567, 0.684], [0.328, 0.428, 0.235, -0.002, -0.03, -0.006, 0.031, 0.171, 0.45, 0.684, 0.649], [0.251, 0.229, 0.135, 0.045, 0.082, 0.156, 0.21, 0.272, 0.418, 0.614, 0.645], [0.143, 0.096, 0.073, 0.116, 0.197, 0.302, 0.379, 0.402, 0.458, 0.637, 0.687], [0.149, 0.104, 0.14, 0.225, 0.347, 0.431, 0.49, 0.585, 0.726, 0.809, 0.801], [0.27, 0.247, 0.352, 0.436, 0.577, 0.656, 0.712, 0.836, 0.946, 0.957, 0.931], [0.467, 0.458, 0.595, 0.657, 0.835, 0.908, 0.935, 1.043, 1.053, 1.026, 1.027], [0.731, 0.694, 0.727, 0.837, 1.02, 1.071, 1.061, 1.112, 1.055, 1.098, 1.157], [1.07, 1.041, 1.063, 1.135, 1.246, 1.293, 1.296, 1.314, 1.305, 1.347, 1.391], [1.257, 1.473, 1.529, 1.519, 1.523, 1.571, 1.615, 1.637, 1.719, 1.727, 1.537], [1.473, 1.501, 1.507, 1.524, 1.538, 1.57, 1.608, 1.657, 1.694, 1.723, 1.727]]
A350_LIFT = [[89.0, 70.0, 0.255], [169.0, 70.0, 0.215], [249.0, 70.0, 0.912], [319.0, 70.0, 1.49], [89.0, 120.0, 0.025], [169.0, 120.0, 0.392], [249.0, 120.0, 1.05], [319.0, 120.0, 1.517], [89.0, 170.0, 0.112], [169.0, 170.0, 0.588], [249.0, 170.0, 1.213], [319.0, 170.0, 1.562], [89.0, 220.0, 0.228], [169.0, 220.0, 0.748], [249.0, 220.0, 1.23], [319.0, 220.0, 1.647], [89.0, 270.0, 0.59], [169.0, 270.0, 0.918], [249.0, 270.0, 1.233], [319.0, 270.0, 1.7], [89.0, 310.0, 0.64], [169.0, 310.0, 0.888], [249.0, 310.0, 1.332], [319.0, 310.0, 1.793]]


def test_the_model_matches_the_lift_the_a350_applied():
    for mx, my, lift in A350_LIFT:
        assert heightmap.evaluate(A350_NODES, (42.0, 34.0), (27.0, 29.0), mx, my, extrapolate=True) == \
            pytest.approx(lift, abs=0.004), (mx, my)


def test_catmull_rom_reproduces_a_plane_and_the_virtual_grid_hits_the_nodes():
    nodes = [[0.02 * i - 0.01 * j for j in range(11)] for i in range(11)]
    virt = heightmap.virtual_grid(nodes)
    for i in range(11):
        for j in range(11):
            assert virt[3 * i][3 * j] == pytest.approx(nodes[i][j])
    # a plane comes back a plane (in the firmware's own frame: 27 mm a point in Y)
    assert heightmap.evaluate(nodes, (42.0, 34.0), (27.0, 29.0), 42 + 27 * 2.5, 34 + 27 * 4.25) == \
        pytest.approx(0.02 * 2.5 - 0.01 * 4.25)


def test_a_fitted_map_makes_the_printer_apply_the_probed_heights():
    # a curved bed, probed on a grid: through the firmware's model the samples come back
    import math
    points = [dict(x=x, y=y, z=12.8 + 0.4 * math.sin(x / 70.0) + 0.3 * (y / 250.0) ** 2, focus=None)
              for x in range(60, 301, 40) for y in range(80, 261, 30)]
    built = heightmap.build(points, (19.0, 10.0))
    assert built["rms"] < 0.02
    for m in built["samples"]:
        applied = heightmap.evaluate(built["nodes"], built["start"], built["spacing"], m["machine_x"], m["machine_y"])
        assert applied == pytest.approx(m["height"], abs=0.04)


def test_an_old_map_is_refitted_through_the_model():
    import math
    points = [dict(x=x, y=y, z=12.8 + 0.3 * math.sin(y / 60.0), focus=None)
              for x in range(60, 301, 40) for y in range(80, 261, 30)]
    built = heightmap.build(points, (19.0, 10.0))
    old = dict(built)
    old.pop("model")
    again = heightmap.refit(old, old["start"], old["spacing"])
    assert again["model"] == heightmap.MODEL and again["rms"] == pytest.approx(built["rms"], abs=1e-6)


def test_a_refit_keeps_the_plateau():
    points = [dict(x=x, y=y, z=12.8 + 0.25 * (y - 150), focus=None) for x in (140, 156, 172) for y in range(150, 231, 16)]
    built = heightmap.build(points, (19.0, 10.0), plateau=True)
    again = heightmap.refit(built, [42.0, 34.0], [27.0, 29.0])       # the grid the printer reports
    flat = [v for col in again["nodes"] for v in col]
    assert again["plateau"] and min(flat) > -0.25 * 27 and max(flat) < again["range"] + 0.25 * 27
