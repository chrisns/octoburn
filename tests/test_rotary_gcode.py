"""LightBurn rotary G-code -> Snapmaker B-axis G-code."""
import math
import re

import pytest

from lhp_pkg import rotary_gcode as rg


def result(e=0.0):
    return dict(
        params=dict(diameter=78.0),
        axis=dict(y0=130.0, x0=164.4, z0=103.0, dx_dy=0.01, dz_dy=-0.005),
        summary=dict(body_from_y=40.0, body_to_y=225.0),
        runout=[dict(y=60.0, e=e, phase_deg=0.0), dict(y=200.0, e=e, phase_deg=0.0)],
    )


LB = """; LightBurn 2.1.04
; GRBL device profile, absolute coords
M112 ; RAW LIGHTBURN ROTARY FILE
G00 G17 G40 G21 G54
G90
M4
G0X10A0
G1X30S200F1200
G1A10
G0X10A20
G1X30A20S0F1200
M5
G0X0A0
M2
"""


def test_translation_maps_axes_and_keeps_the_beam_safe():
    s = rg.Surface(result(), focus_offset=1.5)
    out, rep = rg.translate(LB.splitlines(), s, y_origin=50.0)
    text = "\n".join(out)
    assert "M112" not in text                       # the guard line is dropped
    assert out[-2].startswith("G0 Z") and out[-3] == "M5" and out[-1] == "; end"
    # LightBurn X10 -> machine Y60; the beam's X follows the axis line there
    first = next(l for l in out if l.startswith("G0 X") and " Z" in l and " B" in l)
    assert "Y60.000" in first and "X%.3f" % (164.4 + 0.01 * (60 - 130)) in first
    # power is inline after one M3 S0, and the S0 burn line became a G0
    assert sum(1 for l in out if l.startswith("M3")) == 1 and any("S200" in l for l in out if l.startswith("G1"))
    assert out.index(next(l for l in out if l.startswith("G53"))) < out.index(next(l for l in out if l.startswith("G0 X")))
    assert not any(l.startswith("G1") and "Y60.000" in l and "B20.000" in l for l in out)
    # focus Z = top + offset
    g1 = next(l for l in out if l.startswith("G1"))
    y = float(g1.split("Y")[1].split()[0])
    z = float(g1.split("Z")[1].split()[0])
    assert z == pytest.approx(103.0 - 0.005 * (y - 130) + 39 + 1.5, abs=1e-3)
    assert rep["burn_moves"] == 2


def test_a_pure_rotation_keeps_lightburns_degree_feed():
    # LightBurn (GRBL convention) writes a pure rotation's feed in degrees/min: 1200 mm/min of
    # surface on 78 mm is F1762.95. The firmware also counts degrees as mm, so it passes through.
    s = rg.Surface(result(), focus_offset=0.0)
    lines = ["G90", "M3", "G0X10A0", "G1A10S100F1762.95", "M5"]
    out, _ = rg.translate(lines, s, y_origin=50.0)
    burn = [l for l in out if l.startswith("G1")][0]
    assert int(burn.split("F")[1]) == pytest.approx(1763, abs=2)


def test_runout_moves_focus_with_b():
    s = rg.Surface(result(e=0.9), focus_offset=0.0)
    assert s.top(60, 0) == pytest.approx(s.top(60, 360))
    assert s.top(60, 180) - s.top(60, 0) == pytest.approx(-1.8, abs=1e-6)


def test_a_burn_off_the_body_is_refused():
    s = rg.Surface(result(), focus_offset=0.0)
    with pytest.raises(rg.TranslateError, match="outside the measured body"):
        rg.translate(LB.splitlines(), s, y_origin=5.0)


@pytest.mark.parametrize("bad,match", [
    ("G2X10A0I1J1", "arcs"), ("G1X10Y5A3S100F900", "both Y and A"),
    ("G1X10A5S600F900", "outside 0..255"), ("G1X10A5Z3S100F900", "Z words"), ("M7", "M7"),
])
def test_anything_not_understood_is_refused(bad, match):
    s = rg.Surface(result(), focus_offset=0.0)
    lines = ["G90", "M4", "G0X10A0", bad, "G1X30A0S100F900", "M5"]
    with pytest.raises(rg.TranslateError, match=match):
        rg.translate(lines, s, y_origin=50.0)


def test_modal_continuation_lines_are_moves():
    s = rg.Surface(result(), focus_offset=0.0)
    out, rep = rg.translate(["G90", "M4", "G0X10A0", "G1X20S100F900", "X30", "A5", "M5"], s, y_origin=50.0)
    assert rep["burn_moves"] == 3


def test_no_focus_offset_no_file():
    s = rg.Surface(result(), focus_offset=None)
    with pytest.raises(rg.TranslateError, match="focus offset"):
        rg.translate(LB.splitlines(), s, y_origin=50.0)


def test_a_negative_focus_offset_is_refused_unless_allowed():
    # 2026-11-17: -5.75 was typed for the 10W instead of +5.75; that drives the hood into the work.
    s = rg.Surface(result(), focus_offset=-5.75)
    with pytest.raises(rg.TranslateError, match="negative"):
        rg.translate(LB.splitlines(), s, y_origin=50.0)
    out, _ = rg.translate(LB.splitlines(), s, y_origin=50.0, allow_negative_offset=True)
    assert out


def test_the_10w_offset_puts_focus_above_the_trigger():
    s = rg.Surface(result(), focus_offset=5.75)
    assert s.focus(130.0) - s.top(130.0) == pytest.approx(5.75)


def test_a_burn_faster_than_the_rotary_can_turn_is_refused():
    # 78 mm: 2400 deg/min of B is 2400 * pi * 78 / 360 = 1633.6 mm/min of surface
    s = rg.Surface(result(), focus_offset=5.75)
    fast = "G90\nM4\nG0X10A0\nG1A40S200F3000\nM5\n"
    with pytest.raises(rg.TranslateError, match=r"1634 mm/min"):
        rg.translate(fast.splitlines(), s, y_origin=50.0)
    ok = fast.replace("F3000", "F1600")
    out, _ = rg.translate(ok.splitlines(), s, y_origin=50.0)
    burn = [l for l in out if l.startswith("G1")][0]
    assert int(burn.split("F")[-1]) <= 2400



import os

DATA = os.path.join(os.path.dirname(__file__), "data", "lightburn")


def lightburn(name):
    with open(os.path.join(DATA, name)) as f:
        return f.read().splitlines()


def burn_times(out):
    """(surface mm, minutes) of each burn, from the translated Snapmaker lines: the firmware
    takes sqrt(dX^2 + dY^2 + dZ^2 + dB^2) / F minutes (B degrees counted as mm)."""
    import re
    pos, res = None, []
    for l in out:
        w = dict((k, float(v)) for k, v in re.findall(r"([XYZBSF])(-?[\d.]+)", l))
        if not l.startswith(("G0", "G1")) or not all(k in w for k in "XYZB"):
            continue
        if pos and l.startswith("G1") and w["S"] > 0:
            d = [w[k] - pos[k] for k in "XYZB"]
            fw = math.sqrt(sum(v * v for v in d))
            surf = math.hypot(math.hypot(d[0], d[1]), d[3] * math.pi * 78 / 360)
            res.append((surf, fw / w["F"]))
        pos = w
    return res


@pytest.mark.parametrize("name", ["rect1.gc", "rect30.gc", "circle.gc", "fill.gc"])
def test_real_lightburn_exports_burn_at_the_layer_speed(name):
    """Real LightBurn 2.1.04 exports (rotary profile, layer at 1200 mm/min): every burn crosses
    the surface at 1200 mm/min once translated, whatever mix of X and A it has."""
    s = rg.Surface(result(), focus_offset=5.75)
    out, rep = rg.translate(lightburn(name), s, y_origin=50.0)
    times = burn_times(out)
    assert times and rep["burn_moves"] == len(times) or name == "fill.gc"
    for surf, minutes in times:
        if surf > 0.05:                         # F is an integer: very short moves round off
            assert surf / minutes == pytest.approx(1200, rel=0.02), (surf, minutes)


def test_real_fill_export_is_relative_and_keeps_its_overscan():
    s = rg.Surface(result(), focus_offset=5.75)
    lines = lightburn("fill.gc")
    assert "G91" in lines
    out, rep = rg.translate(lines, s, y_origin=50.0)
    # absolute extents rebuilt from G91: X 65.5..106.5 (Y 115.5..156.5), A 104.381..163.0
    assert rep["y_from"] == pytest.approx(50 + 66.0, abs=0.6)
    assert rep["y_to"] == pytest.approx(50 + 106.0, abs=0.6)
    # B = -A (the default direction): A 104.381..162.999 -> B -162.999..-104.381
    assert rep["b_from"] == pytest.approx(-162.999, abs=0.01)
    assert rep["b_to"] == pytest.approx(-104.381, abs=0.01)
    # overscan and line steps stay G1 at S0 (beam off, at speed), not G0
    s0 = [l for l in out if l.startswith("G1") and " S0 " in l]
    assert len(s0) > 500
    assert not any(l.startswith("G91") for l in out)


def test_a_relative_move_needs_an_absolute_start():
    s = rg.Surface(result(), focus_offset=5.75)
    with pytest.raises(rg.TranslateError, match="relative move before"):
        rg.translate(["G91", "M3", "G1X10A5S100F900", "M5"], s, y_origin=50.0)


def test_default_direction_is_unmirrored_for_the_probed_rotation():
    """+B turns the top towards -X (probed). The point burned at LightBurn A ends up, once the
    object is turned back to B0, at world angle 90 - B from +X; for canvas +Y to land towards -X
    on the top (canvas X -> machine +Y, right-handed), B must fall as A rises."""
    assert rg.Y_SIGN == 1 and rg.B_SIGN == -1
    s = rg.Surface(result(), focus_offset=5.75)
    out, rep = rg.translate(["G90", "M3", "G0X10A0", "G1A10S100F1762.95", "M5"], s, y_origin=50.0)
    assert rep["b_to"] == pytest.approx(0.0) and rep["b_from"] == pytest.approx(-10.0)



def test_a_translated_file_never_switches_to_g54():
    """G54 holds a stale offset on this machine; the job runs and ends in G53."""
    s = rg.Surface(result(), focus_offset=5.75)
    out, _ = rg.translate(lightburn("rect1.gc"), s, y_origin=50.0)
    assert not any(l.split(";")[0].strip().startswith("G54") for l in out)
    assert any(l.startswith("G53") for l in out)


def eccentric(phase):
    r = dict(params=dict(diameter=78.0),
             axis=dict(y0=130.0, x0=164.4, z0=103.0, dx_dy=0.0, dz_dy=0.0),
             summary=dict(body_from_y=25.0, body_to_y=216.0),
             runout=[dict(y=60.0, e=0.7, phase_deg=phase), dict(y=200.0, e=0.7, phase_deg=phase)])
    return rg.Surface(r, focus_offset=5.75)


def test_the_axis_x_follows_the_eccentric_centre_as_b_turns():
    """Measured on the A350 (Y60, e 0.70, phase 270): the centre X was 1.34 mm further +X at
    B180 than at B0. The model gives 2e = 1.40."""
    s = eccentric(270.0)
    assert s.axis_x(60.0, 0.0) == pytest.approx(164.4)          # B0: the probed axis line
    assert s.axis_x(60.0, 180.0) - s.axis_x(60.0, 0.0) == pytest.approx(1.4)


@pytest.mark.parametrize("phase", [0.0, 77.0, 270.0])
def test_x_and_z_stay_on_the_true_top_of_a_circle_turning_off_centre(phase):
    """A circle whose centre sits e off the spin axis, turned by B with +B taking the top towards
    -X (ROT_SIGN): the model's X and Z must be its centre X and top Z at every B."""
    s = eccentric(phase)
    e = 0.7
    # the centre angle at B0, from the runout phase: z_c = e cos(B - phase) = e sin(a0 + B)
    a0 = math.radians(90.0 - phase)
    xs = 164.4 - e * math.cos(a0)                  # spin axis X (the model's B0 X is the centre)
    zs = 103.0 - e * math.sin(a0)
    for b in range(0, 360, 30):
        a = a0 + rg.ROT_SIGN * math.radians(b)
        cx, cz = xs + e * math.cos(a), zs + e * math.sin(a)
        assert s.axis_x(60.0, b) == pytest.approx(cx, abs=1e-9)
        assert s.top(60.0, b) == pytest.approx(cz + 39.0, abs=1e-9)


def test_a_pure_rotation_on_an_eccentric_flask_moves_x_too():
    s = eccentric(270.0)
    out, _ = rg.translate(["G90", "M3", "G0X35A0", "G1A180S100F1762.95", "M5"], s, y_origin=25.0)
    burn = [l for l in out if l.startswith("G1") and " S100 " in l][0]
    x = float(re.search(r"X(-?[\d.]+)", burn).group(1))
    # B = -A: -180 swings the centre the same 2e as +180
    assert x == pytest.approx(s.axis_x(60.0, -180.0), abs=1e-3)
    assert abs(x - 164.4) == pytest.approx(1.4, abs=1e-3)


def test_a_b_shift_keeps_the_art_and_the_runout_on_the_object():
    # A head swap is a power cycle: the firmware calls wherever B was "B0". With the old reading
    # kept as b_shift (object angle = firmware B + shift), the art lands where it did before the
    # swap and Z follows the runout measured in the object's own angles.
    s = rg.Surface(result(e=1.0), focus_offset=0.0)
    out0, _ = rg.translate(LB.splitlines(), s, y_origin=50.0)
    out, _ = rg.translate(LB.splitlines(), s, y_origin=50.0, b_shift=-68.0)

    def moves(lines):
        got = []
        for l in lines:
            if l.startswith(("G0 X", "G1 X")) and " B" in l:
                got.append((float(l.split("Y")[1].split()[0]), float(l.split("B")[1].split()[0]),
                            float(l.split("Z")[1].split()[0]) if " Z" in l else None))
        return got
    m0, m = moves(out0), moves(out)
    assert len(m0) == len(m)
    for (y0, b0, z0), (y1, b1, z1) in zip(m0, m):
        assert y1 == pytest.approx(y0)
        assert b1 == pytest.approx(b0 + 68.0)             # firmware B = object angle - shift
        if z0 is not None and z1 is not None:
            assert z1 == pytest.approx(z0, abs=1e-3)       # same object angle, same height


def _mv(u, v, g=0, fire=False, power=0.0, minutes=None):
    return dict(u=u, v=v, g=g, fire=fire, power=power, minutes=minutes)


def test_a_long_flat_travel_hops_over_whatever_is_beside_the_work():
    e = rg.FlatEmitter(21.4)
    e.emit(_mv(230.0, 120.0))                                    # the first move: up, across, down
    burn = e.emit(_mv(240.0, 120.0, g=1, fire=True, power=76, minutes=10.0 / 1200))
    assert all(" Z21.400 " in l for l in burn if l.startswith("G1"))
    short = e.emit(_mv(250.0, 130.0))                             # 14 mm: stays down
    assert short == ["G0 X250.000 Y130.000 Z21.400 F3000"]
    park = e.emit(_mv(0.0, 300.0))                                # LightBurn's finish move
    assert park == ["G0 Z101.400 F1200", "G0 X0.000 Y300.000 F3000", "G0 Z21.400 F600"]
    e2 = rg.FlatEmitter(300.0)
    e2.emit(_mv(10.0, 10.0))
    assert e2.emit(_mv(200.0, 10.0))[0] == "G0 Z330.000 F1200"   # never past the top


def _physical(lines, b):
    """The head on a machine with X backlash b: moving +X it stops b short of the command,
    moving -X it lands on it. Returns [(x, y, burning)] after every move."""
    out, x, y, cmd_x, burning = [], None, None, None, False
    for line in lines:
        mx = re.search(r"\bX(-?[\d.]+)", line)
        my = re.search(r"\bY(-?[\d.]+)", line)
        if not (mx or my) or not line.startswith(("G0", "G1")):
            continue
        nx = float(mx.group(1)) if mx else cmd_x
        ny = float(my.group(1)) if my else y
        if x is None:
            x = nx - b                                       # arrived moving +X
        elif nx > x + b:                                     # pushing the carriage +X
            x = nx - b
        elif nx < x:                                         # pulling it -X
            x = nx
        cmd_x, y = nx, ny
        s = re.search(r"\bS(\d+)", line)
        burning = line.startswith("G1") and s is not None and int(s.group(1)) > 0
        out.append((round(x, 4), y, burning))
    return out


def test_backlash_compensation_lands_both_sides_of_a_split_line_on_the_line():
    pts = [(0, -5), (0, 0), (10, 0), (10, 10), (20, 10), (20, 20), (10, 20), (10, 30), (0, 30), (0, 35)]
    for b, expect_gap in ((0.0, 0.52), (0.52, 0.0)):
        e = rg.FlatEmitter(21.4, backlash_x=b)
        lines = []
        for k, (u, v) in enumerate(pts):
            u, v = 100.0 + u, 100.0 + v
            lines += e.emit(_mv(u, v)) if k == 0 else e.emit(_mv(u, v, g=1, fire=True, power=76, minutes=10.0 / 1200))
        phys = _physical(lines, 0.52)
        # the head's X while it burns along Y at the two verticals drawn at X110
        xs = sorted(set(p[0] for p in phys if p[2] and abs(p[0] - 110.0) < 1.0))
        gap = max(xs) - min(xs)
        assert gap == pytest.approx(expect_gap, abs=0.01), (b, xs)
        if b:
            assert xs == [pytest.approx(110.0, abs=0.01)]
            lead = [p[0] for p in phys if p[2] and abs(p[0] - 100.0) < 1.0]
            assert all(abs(v - 100.0) < 0.01 for v in lead)         # both lead lines on X100


def test_no_backlash_setting_changes_nothing():
    e = rg.FlatEmitter(21.4)
    assert e.emit(_mv(230.0, 120.0))[:3] == ["G0 Z200.000 F1200", "G0 X230.000 Y120.000 F3000", "G0 Z21.400 F600"]
    assert e.emit(_mv(240.0, 120.0, g=1, fire=True, power=76, minutes=10.0 / 1200))[-1].startswith("G1 X240.000 Y120.000")


def test_the_flat_emitter_refuses_a_move_that_brings_the_module_over_an_obstacle():
    box = dict(box=[112.0, 145.0, 182.0, 235.0], top=38.0, name="the box")
    e = rg.FlatEmitter(21.4, obstacles=[box])
    e.emit(_mv(10.0, 150.0))                                     # far left: the module reaches X10+40
    e.emit(_mv(30.0, 150.0, g=1, fire=True, power=76, minutes=20.0 / 1200))
    with pytest.raises(rg.TranslateError, match="the box"):
        e.emit(_mv(72.0, 150.0, g=1, fire=True, power=76, minutes=42.0 / 1200))   # +40 +5 reaches X117
    e2 = rg.FlatEmitter(21.4, obstacles=[box])
    e2.emit(_mv(10.0, 150.0))
    e2.emit(_mv(10.0, 30.0))                                     # a long travel round, passing in front: fine
    with pytest.raises(rg.TranslateError):
        rg.FlatEmitter(21.4, obstacles=[box]).emit(_mv(150.0, 270.0))   # landing just behind it: the module reaches Y228


def test_no_obstacles_no_guard():
    e = rg.FlatEmitter(21.4)
    e.emit(_mv(150.0, 190.0))
    assert e.emit(_mv(160.0, 190.0, g=1, fire=True, power=76, minutes=10.0 / 1200))
