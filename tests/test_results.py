"""Focus, offsets and the LightBurn column."""
import pytest

from lhp_pkg import results


def point(x, y, z, spread=0.0, n=2):
    return dict(x=x, y=y, z=z, spread=spread, n=n, machine_z=z + 40)


def test_calibration_math():
    assert results.focus_offset(29.4, 12.3) == pytest.approx(17.1)
    assert results.laser_focus_from_touch(5.0, 17.0) == pytest.approx(22.0)
    off = results.focus_offset(results.laser_focus_from_touch(5.0, 17.0), 12.3)
    assert results.focus_z(12.3, off) == pytest.approx(22.0)
    assert results.focus_z(12.3, None) is None
    with pytest.raises(ValueError):
        results.focus_offset(None, 12.0)


def test_table_with_reference_and_first_point_datum():
    pts = [point(10, 10, 12.0), point(50, 10, 15.5), point(90, 10, 11.25)]
    rows = results.table(pts, offset=17.0, reference=2)
    assert [r["height"] for r in rows] == [pytest.approx(0.75), pytest.approx(4.25), 0.0]
    assert [r["focus"] for r in rows] == [pytest.approx(29.0), pytest.approx(32.5), pytest.approx(28.25)]
    assert [r["lightburn"] for r in rows] == [0.0, pytest.approx(3.5), pytest.approx(-0.75)]
    assert rows[0]["is_datum"] and rows[2]["is_reference"]


def test_lightburn_needs_no_calibration_and_follows_the_datum():
    pts = [point(10, 10, 12.0), point(50, 10, 15.5)]
    rows = results.table(pts, offset=None, datum=1)
    assert [r["focus"] for r in rows] == [None, None]
    assert [r["lightburn"] for r in rows] == [pytest.approx(-3.5), 0.0]
    assert [r["height"] for r in rows] == [None, None]


def test_offset_cancels_out_of_the_lightburn_column():
    pts = [point(10, 10, 12.0), point(50, 10, 15.5)]
    a = [r["lightburn"] for r in results.table(pts, offset=3.0)]
    b = [r["lightburn"] for r in results.table(pts, offset=-40.0)]
    assert a == b


def test_invert_flips_the_lightburn_sign():
    pts = [point(10, 10, 12.0), point(50, 10, 15.5)]
    assert results.table(pts, invert=True)[1]["lightburn"] == pytest.approx(-3.5)


def test_failed_points_and_bad_indices():
    pts = [None, point(50, 10, 15.5), point(90, 10, 16.0)]
    rows = results.table(pts, reference=0, datum=7)
    assert rows[0]["failed"] and rows[1]["is_datum"] and rows[2]["lightburn"] == pytest.approx(0.5)
    assert rows[1]["height"] is None


def test_text_table_is_numbers_only():
    pts = [point(10, 10, 12.0, 0.01), point(50, 10, 15.5)]
    text = results.as_text(results.table(pts, offset=17.0, reference=0), "2W IR")
    lines = text.splitlines()
    assert lines[0].split() == ["#", "X", "Y", "trigger_Z", "spread", "n", "height",
                                "focus_Z(2W_IR)", "LightBurn_Z"]
    assert lines[1].split() == ["1*", "10.00", "10.00", "12.000", "0.010", "2", "+0.000",
                                "29.000", "+0.000"]
    assert lines[2].split() == ["2", "50.00", "10.00", "15.500", "0.000", "2", "+3.500",
                                "32.500", "+3.500"]


def test_text_table_shows_rejected_samples():
    p = dict(point(10, 10, 12.0), n=2, taken=3)
    assert results.as_text(results.table([p])).splitlines()[1].split()[5] == "2/3"


def test_machine_offset_check():
    assert results.machine_offset_check(12.0, 52.0, 29.0, 69.0) is None
    assert "work origin" in results.machine_offset_check(12.0, 52.0, 29.0, 75.0)
    assert results.machine_offset_check(12.0, None, 29.0, 69.0) is None
