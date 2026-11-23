"""Probe here: the hand workflow. The user jogged the head; the plugin only lowers it."""
import pytest

from lhp_pkg.routine import Job, TapSession
from helpers import Rig, is_step

TEN_W = dict(name="10W", focus_offset=17.0, dx=0.0, dy=0.0)


def run(rig, mode, session, module=None, tap=True, settings=None):
    said = []

    def say(payload):
        said.append(payload)
        if tap and payload and payload.get("waiting_for_tap"):
            rig.device.tap()
    job = Job(rig.bridge, rig.esp, say, dict(rig.config, **(settings or {})), [], mode=mode,
              module=module or TEN_W, clock=rig.clock, session=session)
    job.run()
    return job, said


def tapped(rig):
    session = TapSession()
    job, _ = run(rig, "tap", session)
    assert job.phase == "done", job.error
    assert session.ok
    return session


def jogged(rig, x, y, z):
    rig.machine.x, rig.machine.y, rig.machine.z = x, y, z
    rig.machine.sent[:] = []


def test_tap_test_moves_nothing_and_opens_the_session():
    rig = Rig()
    session = tapped(rig)
    assert not any(c.split()[0] in ("G0", "G1", "G28") for c in rig.machine.sent)
    assert rig.device.presses[-1] in ("probe_stow", "latch_clear")


def test_probe_here_from_a_jogged_position_finds_the_surface():
    rig = Rig(surface=lambda x, y: 4.0 + 0.03 * x + 0.01 * y)
    session = tapped(rig)
    jogged(rig, 123.4, 77.0, 18.3)
    job, said = run(rig, "here", session)
    assert job.phase == "done", job.error
    r = job.result["points"][0]
    t = rig.device.surface(123.4, 77.0) + rig.device.trigger_travel
    assert 0 <= t - r["z"] < 0.02 + 1e-9
    assert (r["x"], r["y"], r["z_start"]) == (123.4, 77.0, 18.3)
    assert r["focus"] == pytest.approx(r["z"] + 17.0)
    assert rig.machine.z == pytest.approx(18.3)              # back up to Z_start
    assert rig.device.presses[-1] == "probe_stow"


def test_probe_here_sends_no_xy_move_and_nothing_below_z_start_but_steps():
    rig = Rig()
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 15.0)
    run(rig, "here", session)
    assert rig.machine.sent[0] == "M5"
    for command in rig.machine.sent:
        words = command.split()
        assert not any(w[0] in "XY" for w in words[1:] if words[0] != "M118"), command
    z_moves = [c for c in rig.machine.sent if c.startswith("G0 Z")]
    assert all(is_step(c) or float(c.split()[1][1:]) >= 12.7345 for c in z_moves)
    assert (rig.machine.x, rig.machine.y) == (50.0, 60.0)


def test_probe_here_never_goes_below_its_window():
    rig = Rig(surface=lambda x, y: -50.0)
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 30.0)
    job, _ = run(rig, "here", session, settings=dict(here_max_descent=6.0))
    assert job.phase == "failed" and "no trigger" in job.error
    assert min(rig.gcode_z_commands()) >= 24.0 - 1e-9
    assert rig.machine.z == pytest.approx(30.0)


def test_probe_here_respects_z_floor_when_set():
    rig = Rig(surface=lambda x, y: -50.0)
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 30.0)
    job, _ = run(rig, "here", session, settings=dict(here_max_descent=20.0, z_floor=27.0))
    assert job.phase == "failed"
    assert min(rig.gcode_z_commands()) >= 27.0 - 1e-9


def test_probe_here_needs_no_safe_z():
    rig = Rig()
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 15.0)
    job, _ = run(rig, "here", session, settings=dict(safe_z=None, z_floor=None))
    assert job.phase == "done", job.error


def test_probe_here_refuses_too_close_to_the_floor():
    rig = Rig()
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 5.2)
    job, _ = run(rig, "here", session, settings=dict(z_floor=5.0))
    assert job.phase == "failed" and "too close to the Z floor" in job.error


def test_probe_here_alarm_says_raise_z_and_resets():
    rig = Rig(surface=lambda x, y: 20.0)                     # the head sits below the surface
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 19.0)
    job, _ = run(rig, "here", session)
    assert job.phase == "failed" and "raise Z" in job.error
    assert "probe_reset" in rig.device.presses
    assert not any(is_step(c) for c in rig.machine.sent)
    assert rig.machine.z == pytest.approx(19.0)


def test_probe_here_touching_on_deploy_is_refused():
    rig = Rig(surface=lambda x, y: 10.0)                     # trips at 12.5; deploy at 12.0
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 12.0)
    job, _ = run(rig, "here", session)
    assert job.phase == "failed" and "touched while deploying" in job.error


def test_probe_here_abort_returns_to_z_start():
    rig = Rig()
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 20.0)
    said = []
    job = Job(rig.bridge, rig.esp, said.append, rig.config, [], mode="here", module=TEN_W,
              clock=rig.clock, session=session)

    def hook(machine, command):
        if command == "G0 Z16.000 F300" and not job.aborted:
            job.abort()
            rig.bridge.interrupt()
    rig.machine.hooks.append(hook)
    job.run()
    assert job.phase == "aborted"
    assert rig.machine.z == pytest.approx(20.0) and rig.device.presses[-1] == "probe_stow"


def test_probe_here_refuses_without_a_tap_test():
    rig = Rig()
    jogged(rig, 50.0, 60.0, 20.0)
    job, _ = run(rig, "here", TapSession())
    assert job.phase == "failed" and "tap test first" in job.error
    assert rig.machine.sent == []                            # not even M5: nothing was sent


def test_a_reboot_after_the_tap_test_closes_the_session():
    rig = Rig()
    session = tapped(rig)
    rig.device.reboot()
    jogged(rig, 50.0, 60.0, 20.0)
    job, _ = run(rig, "here", session)
    assert job.phase == "failed" and "restarted since the tap test" in job.error
    assert not session.ok and rig.machine.sent == []
    # a new tap test opens it again
    session2 = tapped(rig)
    job, _ = run(rig, "here", session2)
    assert job.phase == "done", job.error


def test_a_reboot_during_probe_here_closes_the_session():
    rig = Rig()
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 20.0)

    def hook(machine, command):
        if command == "G0 Z17.000 F300":
            rig.device.reboot()
    rig.machine.hooks.append(hook)
    job, _ = run(rig, "here", session)
    assert job.phase == "failed" and "restarted" in job.error
    assert not session.ok
    assert rig.machine.z == pytest.approx(20.0)


def test_a_failed_tap_test_keeps_the_session_closed():
    rig = Rig()
    session = TapSession()
    job, _ = run(rig, "tap", session, tap=False)
    assert job.phase == "failed" and "no tap" in job.error and not session.ok


def test_probe_here_sends_only_whitelisted_gcode():
    from test_routine import ALLOWED
    rig = Rig()
    session = tapped(rig)
    jogged(rig, 50.0, 60.0, 20.0)
    run(rig, "here", session)                                # a good one
    rig.device.surface = lambda x, y: 25.0
    run(rig, "here", session)                                # an alarm
    rig.device.surface = lambda x, y: -50.0
    run(rig, "here", session)                                # no trigger
    sent = rig.machine.sent
    assert len(sent) > 50
    for command in sent:
        words = command.split()
        assert words[0] in ALLOWED, command
        if words[0] in ("M5", "M92"):
            assert len(words) == 1, command
    assert not any(c.split()[0] in ("G1", "G28", "M3", "M4", "M106", "M104", "M140") for c in sent)


def test_an_assumed_tap_opens_the_session_without_a_finger():
    """The user tested the probe with its own button: deploy and stow are still checked."""
    rig = Rig()
    session = TapSession()
    job, said = run(rig, "tap", session, tap=False, settings=dict(tap_assumed=True))
    assert job.phase == "done", job.error
    assert session.ok
    assert "probe_arm" in rig.device.presses and rig.device.presses[-1] in ("probe_stow", "latch_clear")
    assert not any(c.split()[0] in ("G0", "G1", "G28") for c in rig.machine.sent)
