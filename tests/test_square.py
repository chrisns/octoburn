"""Probe here over a square: n x n points around where the user jogged, report the highest.

Also the busy wait: the firmware resets the probe 150 ms after every trigger (1 s of PWM), and a
deploy pressed meanwhile would queue behind it and come out late.
"""
import pytest

from helpers import Rig, is_step
from test_here import jogged, run, tapped

SQUARE = dict(here_square=dict(size=3.0, n=3))


def xy_moves(rig):
    out = []
    for c in rig.machine.sent:
        words = c.split()
        if words[0] == "G0" and any(w[0] == "X" for w in words[1:]):
            out.append(tuple(float(w[1:]) for w in words[1:3]))
    return out


def test_the_square_reports_its_highest_point():
    # a slope rising towards +X, +Y: the highest point is the (+1.5, +1.5) corner
    rig = Rig(surface=lambda x, y: 4.0 + 0.2 * (x - 50.0) + 0.1 * (y - 60.0))
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 15.0)
    job, _ = run(rig, "here", session, settings=SQUARE)
    assert job.phase == "done", job.error
    p = job.result["points"][0]
    assert (p["x"], p["y"]) == (51.5, 61.5)
    t = rig.device.surface(51.5, 61.5) + rig.device.trigger_travel
    assert 0 <= t - p["z"] < 0.02 + 1e-9
    sq = p["square"]
    assert len(sq["points"]) == 9 and sq["misses"] == []
    assert sq["max"] == p["z"] and sq["range"] == pytest.approx(sq["max"] - sq["min"], abs=1e-4)
    assert p["focus"] == pytest.approx(p["z"] + 17.0)


def test_the_square_is_a_serpentine_at_the_start_z_and_ends_where_it_started():
    rig = Rig(surface=lambda x, y: 4.0)
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 15.0)
    job, _ = run(rig, "here", session, settings=SQUARE)
    assert job.phase == "done", job.error
    assert xy_moves(rig) == [(48.5, 58.5), (50.0, 58.5), (51.5, 58.5),
                             (51.5, 60.0), (50.0, 60.0), (48.5, 60.0),
                             (48.5, 61.5), (50.0, 61.5), (51.5, 61.5),
                             (50.0, 60.0)]
    assert (rig.machine.x, rig.machine.y, rig.machine.z) == (50.0, 60.0, pytest.approx(15.0))


def test_every_xy_move_is_at_the_start_z_with_the_pin_stowed():
    rig = Rig(surface=lambda x, y: 4.0 + 0.1 * (x - 50.0))
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 15.0)
    z_at_xy = []
    rig.machine.listeners.append(lambda m: None)
    original = rig.machine.sent.append

    deployed_at_xy = []

    class Watch(list):
        def append(self, command):
            if command.startswith("G0 X"):
                z_at_xy.append(rig.machine.z)
                deployed_at_xy.append(rig.device.deployed)
            list.append(self, command)
    rig.machine.sent = Watch(rig.machine.sent)
    job, _ = run(rig, "here", session, settings=SQUARE)
    assert job.phase == "done", job.error
    assert z_at_xy and all(z == pytest.approx(15.0) for z in z_at_xy)
    assert not any(deployed_at_xy)


def test_no_unprobed_move_goes_below_the_start():
    rig = Rig(surface=lambda x, y: 4.0 + 0.3 * (y - 60.0))
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 15.0)
    job, _ = run(rig, "here", session, settings=SQUARE)
    assert job.phase == "done", job.error
    previous = 15.0
    for c in rig.machine.sent:
        if c.startswith("G0 Z"):
            z = float(c.split()[1][1:])
            assert is_step(c) or z >= previous - 1e-9, c
            previous = z


def test_points_off_the_edge_are_misses_not_failures():
    # the object ends at X 49: the left column has nothing under it within the window
    rig = Rig(surface=lambda x, y: 4.0 if x > 49.0 else -50.0)
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 15.0)
    job, _ = run(rig, "here", session, settings=SQUARE)
    assert job.phase == "done", job.error
    sq = job.result["points"][0]["square"]
    assert len(sq["points"]) == 6 and len(sq["misses"]) == 3
    assert all(m["x"] == 48.5 and "no trigger" in m["error"] for m in sq["misses"])
    assert rig.machine.z == pytest.approx(15.0)


def test_a_square_with_nothing_under_it_fails():
    rig = Rig(surface=lambda x, y: -50.0)
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 15.0)
    job, _ = run(rig, "here", session, settings=SQUARE)
    assert job.phase == "failed" and "no point of the 3.0 mm square triggered" in job.error
    assert rig.machine.z == pytest.approx(15.0)


def test_a_touch_on_deploy_stops_the_square():
    # one corner rises so high that the pin, deployed at the start Z, lands on it
    rig = Rig(surface=lambda x, y: 11.0 if (x > 51.0 and y > 61.0) else 4.0)
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 13.0)
    job, _ = run(rig, "here", session, settings=SQUARE)
    assert job.phase == "failed" and "touched while deploying" in job.error
    assert rig.machine.z == pytest.approx(13.0)
    assert "probe_reset" in rig.device.presses


def test_a_square_with_a_held_trigger_clone():
    rig = Rig(holds_trigger=True, surface=lambda x, y: 4.0)
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 15.0)
    job, _ = run(rig, "here", session, settings=SQUARE)
    assert job.phase == "done", job.error
    assert len(job.result["points"][0]["square"]["points"]) == 9


def test_no_deploy_is_sent_while_the_esp_is_busy():
    rig = Rig(busy_reads=3)
    rig.prober().measure(100, 100)
    assert rig.device.presses.count("probe_arm") == 4
    assert rig.device.armed_while_busy == 0


def test_the_busy_wait_matters():
    """Guard: with the waits removed, the same rig deploys while busy."""
    rig = Rig(busy_reads=3)
    rig.esp.wait_idle = lambda sleep, timeout=4.0: None
    rig.prober().measure(100, 100)
    assert rig.device.armed_while_busy > 0


def test_an_esp_that_stays_busy_stops_the_point():
    from lhp_pkg.esp import EspError
    rig = Rig(busy_reads=10 ** 6)
    with pytest.raises(EspError, match="stayed busy"):
        rig.prober().measure(100, 100)
