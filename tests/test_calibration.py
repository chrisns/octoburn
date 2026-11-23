# coding=utf-8
"""calibration.py: the focus offset checks, repeatability and the focus ladder file."""
import re

import pytest

from lhp_pkg import calibration as cal


# -- the focus offset ---------------------------------------------------------------------

def test_cad_offset_is_5_75_and_hood_ends_10_above_the_surface():
    assert cal.CAD_OFFSET_10W == 5.75
    # with no pre-travel, at focus Z the hood is offset + 4.25 above the surface: must be 10
    assert cal.CAD_OFFSET_10W + cal.HOOD_ABOVE_TIP == pytest.approx(cal.FOCAL_10W)


def test_card_thickness_is_subtracted_worked_numbers():
    # Probe trigger Z100.00 at the spot. The 10W hood is 4.25 above the pin tip, so with no
    # pre-travel a 0.30 card drags under the hood at Z100 - 4.25 + 0.30 = 96.05. The hood would
    # touch the bare surface at 95.75; focus is 10 above that, 105.75: offset +5.75, the CAD.
    out = cal.offset_from_card(drag_z=96.05, card_thickness=0.30, z_ref=100.0)
    assert out["touch_z"] == pytest.approx(95.75)
    assert out["focus_z"] == pytest.approx(105.75)
    assert out["offset"] == pytest.approx(5.75)


def test_without_the_card_correction_the_offset_is_one_card_too_high():
    with_card = cal.offset_from_card(96.05, 0.30, 100.0)["offset"]
    ignored = cal.offset_from_card(96.05, 0.0, 100.0)["offset"]
    assert ignored - with_card == pytest.approx(0.30)


def test_offset_sign_focus_is_above_the_trigger():
    # With 0.8 mm of pin pre-travel the head is 0.8 lower at the trip, so the card drags at
    # 96.85 and the offset is 5.75 + 0.8. The head goes UP from trigger to focus.
    out = cal.offset_from_card(drag_z=96.85, card_thickness=0.30, z_ref=100.0)
    assert out["offset"] == pytest.approx(6.55)
    assert out["focus_z"] > 100.0
    assert cal.cad_check(out["offset"])["implied_pretravel"] == pytest.approx(0.8)


@pytest.mark.parametrize("t", [-0.1, 2.5, "x", None])
def test_card_thickness_is_checked(t):
    with pytest.raises(cal.CalibrationError):
        cal.offset_from_card(96.0, t, 100.0)


def test_cad_check_reports_pretravel_and_stays_quiet_when_plausible():
    out = cal.cad_check(6.55)
    assert out["implied_pretravel"] == pytest.approx(0.80)
    assert out["warning"] is None


def test_cad_check_warns_far_from_cad():
    assert "pre-travel" in cal.cad_check(9.5)["warning"]
    assert "pre-travel" in cal.cad_check(4.6)["warning"]


def test_cad_check_negative_offset_is_the_sign_slip():
    # 2026-11-17: -5.75 was first saved by a sign slip; it would put the hood in the work.
    warning = cal.cad_check(-5.75)["warning"]
    assert "sign" in warning and "ABOVE" in warning


# -- the re-touch ---------------------------------------------------------------------------

REF = dict(x=160.0, y=160.0, z=100.0, machine_z=100.0)


def test_retouch_ok():
    out = cal.retouch_check(REF, dict(x=160.1, y=159.9, z=100.02, machine_z=100.02))
    assert out["level"] == "ok" and out["delta"] == pytest.approx(0.02)


def test_retouch_warns_on_seating_shift():
    assert cal.retouch_check(REF, dict(x=160, y=160, z=100.12, machine_z=100.12))["level"] == "warn"


def test_retouch_lost_steps_says_rehome():
    # the 27.67 mm loss of 2026-11-17: the head was higher than reported, so a surface
    # read lower; G-code and machine Z both come from the same counter, so they agree
    out = cal.retouch_check(REF, dict(x=160, y=160, z=72.33, machine_z=72.33))
    assert out["level"] == "rehome" and "G28" in out["message"]


def test_retouch_origin_change():
    out = cal.retouch_check(REF, dict(x=160, y=160, z=100.0, machine_z=111.0))
    assert out["level"] == "origin"


def test_retouch_elsewhere_is_not_compared():
    out = cal.retouch_check(REF, dict(x=170, y=160, z=100.0))
    assert out["level"] == "moved"


def test_retouch_without_machine_z_or_xy_still_checks_z():
    assert cal.retouch_check(dict(z=100.0), dict(z=100.3))["level"] == "warn"


def test_quantisation_bias():
    assert cal.quantisation_bias(0.02) == 0.01
    assert cal.quantisation_bias(0.1) == 0.05


# -- repeatability ------------------------------------------------------------------------

def test_forced_samples_overrides():
    assert cal.forced_samples(5) == dict(samples=5, max_samples=5, tolerance=1.0)
    for bad in (2, 11, "x"):
        with pytest.raises(cal.CalibrationError):
            cal.forced_samples(bad)


def test_forced_samples_make_agreeing_keep_every_touch():
    from lhp_pkg.probe import agreeing
    over = cal.forced_samples(5)
    samples = [100.00, 100.04, 99.98, 100.10, 100.02]
    # after 4 touches the group of 5 is not there yet, so the probe keeps going
    assert len(agreeing(samples[:4], over["tolerance"], need=over["samples"])[0]) < over["samples"]
    group, ambiguous = agreeing(samples, over["tolerance"], need=over["samples"])
    assert len(group) == 5 and not ambiguous


def test_stats():
    s = cal.stats([1.0, 2.0, 3.0, 4.0])
    assert s["n"] == 4 and s["mean"] == 2.5 and s["range"] == 3.0
    assert s["sd"] == pytest.approx(1.291, abs=1e-3) and s["two_sd"] == pytest.approx(2.582, abs=1e-3)
    assert s["slope"] == pytest.approx(1.0)
    one = cal.stats([5.0])
    assert one["sd"] is None and one["slope"] is None
    with pytest.raises(cal.CalibrationError):
        cal.stats([])


def test_swap_repeatability_separates_seating_from_noise():
    # three seatings, touches within each agree to 0.01, but the seatings sit 0.1 apart
    pts = [dict(x=0, y=0, z=0, samples=[100.00, 100.01, 100.00]),
           dict(x=0, y=0, z=0, samples=[100.10, 100.11, 100.10]),
           dict(x=0, y=0, z=0, samples=[100.20, 100.20, 100.21])]
    out = cal.swap_repeatability(pts)
    assert out["seatings"] == 3 and out["touches"] == 9
    assert out["within_sd"] < 0.01
    assert out["between_sd"] == pytest.approx(0.1, abs=1e-3)
    assert out["between_range"] == pytest.approx(0.2, abs=1e-3)


def test_swap_repeatability_needs_two():
    with pytest.raises(cal.CalibrationError):
        cal.swap_repeatability([dict(z=1.0)])


def test_point_heights_falls_back_to_z():
    assert cal.point_heights(dict(z=3.0)) == [3.0]
    assert cal.point_heights(dict(z=3.0, samples=[2.9, 3.1])) == [2.9, 3.1]


def test_report_uses_points_near_the_reference_only():
    ref = dict(x=160.0, y=160.0, z=100.0, machine_z=100.0)
    pts = [dict(x=160.0, y=160.0, z=100.0, machine_z=100.0, samples=[100.0, 100.02]),
           dict(x=50.0, y=50.0, z=80.0, samples=[80.0]),
           dict(x=160.2, y=160.0, z=100.03, machine_z=100.03, samples=[100.02, 100.04])]
    out = cal.report(pts, ref)
    assert out["swaps"]["seatings"] == 2
    assert out["retouch"]["level"] == "ok"
    assert out["last"]["n"] == 2
    assert cal.report([], None) == dict(last=None, swaps=None, retouch=None)


# -- the focus ladder ---------------------------------------------------------------------

def _ladder(**kw):
    args = dict(x=150.0, y=150.0, z_ref=100.0, work_offset=(0.0, 0.0, 0.0))
    args.update(kw)
    return cal.focus_ladder(**args)


def test_ladder_rows_centre_on_the_guess_highest_first():
    rows = cal.ladder_rows(100.0, 5.75, 0.25, 5, 10.0)
    assert [r[0] for r in rows] == [2, 1, 0, -1, -2]
    assert [r[1] for r in rows] == [106.25, 106.0, 105.75, 105.5, 105.25]
    assert rows[2][2] == 16.0 and rows[0][2] == 10.0


def test_ladder_never_fires_while_z_moves():
    lines = _ladder()
    beam = False
    for line in lines:
        code = line.split(";")[0].strip()
        if not code:
            continue
        if code.startswith("M3"):
            beam = True
            assert " S0" in code
        elif code.startswith("M5"):
            beam = False
        if re.search(r"\bZ", code) and code.startswith(("G0", "G1")):
            assert not beam, line
        if code.startswith("G0"):
            assert not beam, line           # no travel with the beam armed
        if code.startswith("G1"):
            assert beam and re.search(r"S\d+", code)
            assert "Z" not in code
    assert lines[-2] == "M5"


def test_ladder_starts_in_machine_coordinates_and_z_only_descends():
    lines = _ladder()
    codes = [l.split(";")[0].strip() for l in lines if l.split(";")[0].strip()]
    assert codes[:5] == ["M5", "G53", "G90", "G21", "M211 S1"]
    zs = [float(re.search(r"Z(-?[\d.]+)", c).group(1)) for c in codes if c.startswith("G0 Z")]
    assert zs[1:-1] == sorted(zs[1:-1], reverse=True)
    assert zs[0] > zs[1] and zs[-1] == zs[0]


def test_ladder_has_no_heat_or_fan_commands():
    for line in _ladder():
        code = line.split(";")[0].strip()
        assert not code.startswith(("M104", "M109", "M140", "M190", "M106", "M107", "M4", "G92", "G28"))


def test_ladder_refuses_a_low_hood():
    # offset guess 0: the lowest line at trigger - 1.5 would bring the hood within 2 mm
    with pytest.raises(cal.CalibrationError):
        _ladder(offset_guess=0.0)


@pytest.mark.parametrize("kw", [dict(power=0), dict(power=300), dict(count=4), dict(count=1),
                                dict(step=2.0), dict(pitch=0.1), dict(feed=50), dict(travel_z=100.0)])
def test_ladder_checks_its_inputs(kw):
    with pytest.raises(cal.CalibrationError):
        _ladder(**kw)


def test_ladder_offset():
    assert cal.ladder_offset(100.0, 106.375) == pytest.approx(6.375)


def test_report_never_compares_the_reference_with_itself():
    ref = dict(x=160.0, y=160.0, z=100.0, machine_z=100.0, time=1000.0)
    same = dict(x=160.0, y=160.0, z=100.0, machine_z=100.0, time=999.0, samples=[100.0, 100.0])
    assert cal.report([same], ref)["retouch"] is None
    later = dict(x=160.0, y=160.0, z=100.6, machine_z=100.6, time=1200.0, samples=[100.6, 100.6])
    out = cal.report([same, later], ref)
    assert out["retouch"]["level"] == "rehome"
    assert out["swaps"]["seatings"] == 2


def test_ladder_needs_the_homed_frame():
    # 2026-11-17: G54 held a stale LightBurn offset of about X-147.5 Y-153.5 Z-245.1
    with pytest.raises(cal.CalibrationError, match="work offset is in force"):
        _ladder(work_offset=(-147.5, -153.5, -245.1))
    with pytest.raises(cal.CalibrationError, match="could not be read"):
        _ladder(work_offset=None)
    with pytest.raises(cal.CalibrationError):
        _ladder(work_offset="0")
    assert _ladder(work_offset=(0.004, -0.003, 0.0))     # rounding noise is fine


def test_ladder_has_no_default_frame():
    with pytest.raises(TypeError):
        cal.focus_ladder(150.0, 150.0, 100.0)


def test_the_10w_hood_meets_the_bed_ten_millimetres_below_focus():
    # focus Z = trigger Z + focus offset, and the hood is FOCAL_10W (10 mm) above the surface
    # at focus, so it meets the bare bed at trigger + offset - 10.
    assert cal.hood_touch_z(13.61, 8.6) == pytest.approx(12.21)
    assert cal.hood_touch_z(13.61, 8.6) + cal.FOCAL_10W == pytest.approx(13.61 + 8.6)
