# coding=utf-8
"""The air purifier's brain: laser energy from the G-code, and when the purifier runs."""
import math

import pytest

from lhp_pkg import purifier as pu


# -- BurnMeter ---------------------------------------------------------------

def feed(meter, lines):
    return sum(meter.feed(l) for l in lines)


def test_a_powered_line_counts_its_energy():
    m = pu.BurnMeter(watts=10.0)
    # 60 mm at 1200 mm/min = 3 s, at S128 of 255 (~50 %) on a 10 W laser = ~15 J
    j = feed(m, ["G90", "G0 X0 Y0 F3000", "M3 S0", "G1 X60 F1200 S128", "M5"])
    assert j == pytest.approx(3.0 * 128 / 255 * 10.0)
    assert m.burn_s == pytest.approx(3.0)


def test_travel_and_laser_off_lines_cost_nothing():
    m = pu.BurnMeter(watts=10.0)
    assert feed(m, ["G0 X100 Y100 F3000", "G1 X0 Y0 F1200"]) == 0       # no M3
    assert feed(m, ["M3 S200", "G0 X50 Y50", "M5", "G1 X0 Y0 S200"]) == 0  # G0, then laser off
    assert feed(m, ["M3 S0", "G1 X10 S0 F600"]) == 0                       # S0 overscan


def test_the_pointer_power_is_not_a_burn():
    m = pu.BurnMeter(watts=10.0, pointer_s_max=10)
    assert feed(m, ["M3 S10", "G1 X100 F600"]) == 0
    assert feed(m, ["M3 S11", "G1 X0 F600"]) > 0


def test_power_and_feed_are_modal_and_relative_moves_work():
    m = pu.BurnMeter(watts=2.0)
    j = feed(m, ["G90", "G1 X0 Y0 F600", "M3 S255", "G91", "G1 X10", "G1 Y10", "G90"])
    assert j == pytest.approx(2 * 1.0 * 2.0)          # two 10 mm lines at 600 mm/min = 1 s each, full power
    assert m.pos["X"] == pytest.approx(10) and m.pos["Y"] == pytest.approx(10)


def test_comments_and_case_are_ignored_and_g92_moves_nothing():
    m = pu.BurnMeter(watts=10.0)
    assert feed(m, ["g1 x0 y0 f600", "G92 X100 Y100 ; reset", "m3 s255 (full)", "G1 X110 Y100"]) == \
        pytest.approx(1.0 * 10.0)


def test_a_home_forgets_the_position_until_an_absolute_move():
    m = pu.BurnMeter(watts=10.0)
    feed(m, ["G1 X0 Y0 F600", "M3 S255", "G28"])
    assert m.feed("G1 X10 Y0") == 0                    # where from? not counted
    assert m.feed("G1 X20 Y0") == pytest.approx(1.0 * 10.0)


def test_a_rotary_b_move_is_timed_by_its_feed():
    m = pu.BurnMeter(watts=10.0)
    assert feed(m, ["G1 X0 Y0 B0 F600", "M3 S255", "G1 B60"]) == pytest.approx(6.0 * 10.0)


def test_inline_s_without_m3_is_not_a_burn():
    # The firmware fires an inline S only after an M3/M4 (seen 2026-11-19).
    m = pu.BurnMeter(watts=10.0)
    assert feed(m, ["G1 X0 F600", "G1 X10 S255"]) == 0


def test_watts_for_the_toolheads():
    assert pu.module_watts("10W LASER") == 10.0
    assert pu.module_watts("2W RED LASER") == 2.0
    assert pu.module_watts("40W LASER") == 40.0
    assert pu.module_watts("3DP") == 0.0
    assert pu.module_watts(None) == 0.0


# -- AirPolicy ---------------------------------------------------------------

CFG = dict(pu.DEFAULTS)


def clean_air(p, t0, pm25=4.0, hcho=10.0, n=10):
    for i in range(n):
        p.sensor(pm25, hcho, t0 - 300 + i * 30)


def test_idle_is_off():
    p = pu.AirPolicy(CFG)
    d = p.tick(0.0)
    assert (d.power, d.gear, d.phase) == (False, 0, "idle")


def test_a_short_mark_does_not_start_the_purifier():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    p.add_burn(10.0, 0.0)                     # today's paper crosses: ~10 J
    d = p.tick(1.0)
    assert d.power is False and d.phase == "burning"
    d = p.tick(60.0)                          # job over, nothing more
    assert d.power is False


def test_a_long_burn_starts_it_and_the_gear_follows_the_power():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    for t in range(0, 120):                   # 10 W at full power for 2 minutes
        p.add_burn(10.0, float(t))
    d = p.tick(120.0)
    assert d.power and d.phase == "burning" and d.gear == 3


def test_a_gentle_long_burn_runs_a_low_gear():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    for t in range(0, 300):                   # 1.5 W average for 5 minutes = 450 J
        p.add_burn(1.5, float(t))
    d = p.tick(300.0)
    assert d.power and d.gear == 1


def test_smoke_the_sensor_sees_starts_it_even_for_a_short_job():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    p.add_burn(10.0, 0.0)
    p.sensor(4.0 + CFG["alarm_pm25"] + 1, 10.0, 30.0)
    d = p.tick(31.0)
    assert d.power and "smoke" in d.reason


def test_run_on_stops_once_the_air_is_bearable_for_two_readings():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    for t in range(0, 120):
        p.add_burn(10.0, float(t))
    p.sensor(4.0 + CFG["runon_gear3"] + 100, 80.0, 125.0)
    d = p.tick(200.0)                          # 80 s after the last burn: run-on
    assert d.power and d.phase == "runon" and d.gear == 3
    bearable = 4.0 + CFG["pm25_margin"] - 1
    p.sensor(bearable, 12.0, 260.0)
    d = p.tick(261.0)
    assert d.power                            # one reading is not enough
    p.sensor(bearable, 12.0, 290.0)
    d = p.tick(291.0)
    assert d.power                            # bearable, but a heavy burn holds 3 min (smouldering)
    d = p.tick(149.0 + CFG["heavy_runon_min_s"] + 2)
    assert not d.power and d.phase == "idle" and "bearable" in d.reason


def test_run_on_lasts_at_least_the_minimum():
    p = pu.AirPolicy(dict(CFG, runon_min_s=120))
    clean_air(p, 0)
    for t in range(0, 60):
        p.add_burn(10.0, float(t))
    p.tick(61.0)
    for t in (100, 130):
        p.sensor(4.0, 10.0, t)
    assert p.tick(131.0).power                # clean, but run-on started ~90 s ago (gap 30 s)
    assert not p.tick(60 + 30 + 121.0).power


def test_run_on_never_exceeds_the_cap_even_in_smoke():
    p = pu.AirPolicy(dict(CFG, runon_max_s=600))
    clean_air(p, 0)
    for t in range(0, 120):
        p.add_burn(10.0, float(t))
    end = 120 + CFG["session_gap_s"]
    t = end
    while t < end + 700:
        p.sensor(500.0, 100.0, t)
        d = p.tick(t + 1)
        t += 30
    assert not d.power and "limit" in d.reason


def test_a_stale_sensor_falls_back_to_a_dose_timer_not_to_running_on():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    for t in range(0, 120):
        p.add_burn(10.0, float(t))
    # no readings after the burn: the sensor went quiet (Home Assistant restarting)
    secs = pu.fallback_runon_s(1200.0, CFG)
    assert CFG["runon_min_s"] <= secs <= CFG["runon_max_s"]
    end = 119 + CFG["session_gap_s"]
    assert p.tick(end + secs - 5).power
    d = p.tick(end + secs + 5)
    assert not d.power and "timer" in d.reason


def test_fallback_grows_with_the_dose_but_stays_capped():
    small = pu.fallback_runon_s(300.0, CFG)
    big = pu.fallback_runon_s(30000.0, CFG)
    huge = pu.fallback_runon_s(1e9, CFG)
    assert CFG["runon_min_s"] <= small < big <= huge == CFG["runon_max_s"]


def test_baseline_is_the_air_before_the_job():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0, pm25=30.0, hcho=20.0)      # a smoky room before the job
    for t in range(0, 90):                     # 900 J: not a heavy burn
        p.add_burn(10.0, float(t))
    p.tick(150.0)
    for t in (160, 190):
        p.sensor(30.0 + CFG["pm25_margin"] - 1, 20.0, t)
    assert not p.tick(215.0).power            # back to the room's own level is bearable


def test_gear_does_not_drop_while_burning():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    for t in range(0, 60):
        p.add_burn(10.0, float(t))
    assert p.tick(60.0).gear == 3
    for t in range(60, 200):
        p.add_burn(0.5, float(t))
    assert p.tick(200.0).gear == 3


def test_run_on_gear_steps_down_with_the_smoke():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    for t in range(0, 120):
        p.add_burn(10.0, float(t))
    p.sensor(4.0 + CFG["runon_gear3"] + 100, 80.0, 160.0)
    assert p.tick(161.0).gear == 3
    p.sensor(4.0 + CFG["runon_gear2"] + 10, 20.0, 190.0)
    assert p.tick(191.0).gear == 2
    p.sensor(4.0 + CFG["pm25_margin"] + 5, 12.0, 220.0)
    assert p.tick(221.0).gear == 1


def test_stop_ends_everything_until_the_next_burn():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    for t in range(0, 120):
        p.add_burn(10.0, float(t))
    assert p.tick(121.0).power
    p.stop(122.0, "Stop pressed")
    d = p.tick(123.0)
    assert not d.power and "Stop pressed" in d.reason
    for t in range(200, 320):
        p.add_burn(10.0, float(t))
    assert p.tick(320.0).power                # a new job starts afresh


def test_decay_is_learnt_from_the_run_on():
    readings = [(0, 200.0), (60, 100.0), (120, 50.0), (180, 25.0)]   # halves every minute
    tau = pu.decay_tau(readings, baseline=0.0)
    assert tau == pytest.approx(60 / math.log(2), rel=0.01)
    assert pu.decay_tau([(0, 200.0), (60, 199.0)], 0.0) is None       # too few / not falling


def test_status_reports_dose_and_air():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    p.add_burn(5.0, 0.0)
    d = p.tick(1.0)
    s = d.status()
    assert s["dose_j"] == pytest.approx(5.0) and s["baseline_pm25"] == pytest.approx(4.0)
    assert s["phase"] == "burning"


def test_nothing_is_learnt_when_the_fan_never_ran():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    p.add_burn(10.0, 0.0)                     # a short mark: no purifier
    for t, v in ((40, 60.0), (70, 40.0), (100, 25.0), (130, 15.0)):
        p.sensor(v, 10.0, t)
        p.tick(t + 1)
    p.tick(1000.0)
    assert p.learned_tau is None


def test_a_heavy_burn_holds_the_run_on_for_the_smouldering():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    for t in range(0, 200):                   # 10 W for 200 s = 2000 J: heavy
        p.add_burn(10.0, float(t))
    start = 199 + CFG["session_gap_s"]
    for k, t in enumerate(range(int(start) + 10, int(start) + 300, 30)):
        p.sensor(5.0, 10.0, t)                # clean at once (it never is, but the hold must apply)
        d = p.tick(t + 1)
        if t + 1 - start < CFG["heavy_runon_min_s"]:
            assert d.power, t
    assert not d.power


def test_a_light_burn_keeps_the_short_minimum():
    p = pu.AirPolicy(CFG)
    clean_air(p, 0)
    for t in range(0, 40):                    # 400 J: over the start, not heavy
        p.add_burn(10.0, float(t))
    start = 39 + CFG["session_gap_s"]
    for t in (start + 20, start + 50, start + 70):
        p.sensor(5.0, 10.0, t)
    assert not p.tick(start + CFG["runon_min_s"] + 15).power
