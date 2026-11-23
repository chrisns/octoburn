"""Whole jobs against the simulated machine and probe."""
import pytest

from lhp_pkg import results
from lhp_pkg.routine import Job
from helpers import Rig, is_step

ALLOWED = {"G0", "G28", "G90", "G91", "M5", "M400", "M114", "M118", "M211", "M92", "M503"}
POINTS = [dict(x=40.0, y=40.0, ref=True, datum=False), dict(x=120.0, y=60.0, ref=False, datum=False),
          dict(x=200.0, y=150.0, ref=False, datum=True)]


def surface(x, y):
    return 3.0 + (6.0 if 100 <= x <= 250 else 0.0) + 0.002 * y


TEN_W = dict(name="10W", focus_offset=None, dx=0.0, dy=0.0)


def run_job(rig, points=POINTS, mode="job", offset=None, abort_when=None, module=None, tap=True):
    said = []
    module = dict(module or TEN_W)
    if offset is not None:
        module["focus_offset"] = offset
    job = Job(rig.bridge, rig.esp, said.append, rig.config, points, mode=mode, module=module,
              clock=rig.clock)
    moments = []

    def say(payload):
        said.append(payload)
        if tap and payload and payload.get("waiting_for_tap"):
            rig.device.tap()                       # the user taps the pin when asked
        if payload and payload.get("type") in ("done", "failed", "aborted"):
            # where everything was when the outcome was said
            moments.append(dict(z=rig.machine.z, deployed=rig.device.deployed,
                                last_press=rig.device.presses[-1] if rig.device.presses else None))
    job._say = say
    if abort_when:
        def hook(machine, command):
            if abort_when(machine, command) and not job.aborted:
                job.abort()
                rig.bridge.interrupt()
        rig.machine.hooks.append(hook)
    job.run()                          # in this thread: no timing races in tests
    return job, said, moments


def test_full_job_measures_every_point():
    rig = Rig(surface=surface)
    job, said, moments = run_job(rig, offset=17.0)
    assert job.phase == "done", job.error
    r = job.result
    assert r["complete"] and r["reference"] == 0 and r["datum"] == 2
    for p, measured in zip(POINTS, r["points"]):
        t = surface(p["x"], p["y"]) + rig.device.trigger_travel
        assert 0 <= t - measured["z"] < 0.02 + 1e-9
    assert r["points"][1]["focus"] == pytest.approx(r["points"][1]["z"] + 17.0)
    rows = results.table(r["points"], 17.0, r["reference"], r["datum"])
    assert rows[1]["height"] == pytest.approx(6.0 + 0.04, abs=0.021)
    assert rows[2]["lightburn"] == 0.0
    assert rows[0]["lightburn"] == pytest.approx(-(6.0 + 0.22), abs=0.021)
    # homed first, soft endstops on
    assert rig.machine.sent[0] == "M5"                      # laser off before anything
    assert "G28" in rig.machine.sent and "M211 S1" in rig.machine.sent
    # the tap test ran before the first probing step
    taps = [i for i, p in enumerate(said) if p and p.get("waiting_for_tap")]
    first_point = next(i for i, p in enumerate(said) if p and "Point 1 of" in (p.get("message") or ""))
    assert taps and taps[0] < first_point
    # the outcome was said only after the head was up and the pin stowed
    assert said[-1]["type"] == "done"
    assert moments == [dict(z=60.0, deployed=False, last_press="probe_stow")]
    assert rig.device.display["job_state"] == "done"
    assert rig.device.display["result_z"] == pytest.approx(r["points"][-1]["z"], abs=0.001)
    assert rig.device.display["result_focus"] == pytest.approx(r["points"][-1]["focus"], abs=0.001)


def test_home_first_off_does_not_home():
    rig = Rig(settings=dict(home_first=False))
    job, _, _ = run_job(rig, points=POINTS[:1])
    assert job.phase == "done" and "G28" not in rig.machine.sent


def test_calibrate_mode_measures_one_spot():
    rig = Rig()
    job, said, _ = run_job(rig, points=[dict(x=150.0, y=150.0)], mode="calibrate")
    assert job.phase == "done" and said[-1]["mode"] == "calibrate"
    assert len(job.result["points"]) == 1


def test_a_failed_point_stops_the_job_and_keeps_the_points_before_it():
    # point 3's surface is above safe_z but below where the head is after point 2: the arm at
    # travel height (60) finds the pin already on it, an alarm
    rig = Rig(surface=lambda x, y: 70.0 if x > 150 else 5.0)
    job, said, moments = run_job(rig)
    assert job.phase == "failed" and "probe alarm" in job.error
    assert said[-1]["type"] == "failed"
    assert len(said[-1]["result"]["points"]) == 2 and not said[-1]["result"]["complete"]
    assert moments[0]["z"] == 60.0 and moments[0]["last_press"] == "probe_stow"
    assert rig.device.display["job_state"] == "error"


def test_abort_mid_descent_ends_safe():
    rig = Rig(surface=surface)
    job, said, moments = run_job(rig, abort_when=lambda m, c: c == "G0 Z20.000 F300")
    assert job.phase == "aborted"
    assert said[-1]["type"] == "aborted"
    assert moments == [dict(z=60.0, deployed=False, last_press="probe_stow")]
    assert min(rig.gcode_z_commands()) >= 20.0 - 1e-9
    assert rig.device.display["job_state"] == "idle"


def test_display_failures_do_not_fail_a_job():
    rig = Rig()
    rig.device.fail = {"/select/": -1, "/number/": -1}
    job, _, _ = run_job(rig)
    assert job.phase == "done", job.error


def test_an_unreachable_probe_stops_the_job_before_anything_moves():
    rig = Rig()
    rig.device.fail = {"/binary_sensor/probe_latched": -1}
    job, said, _ = run_job(rig)
    assert job.phase == "failed" and "probe_latched" in job.error
    assert not any(c.split()[0] in ("G0", "G1", "G28") for c in rig.machine.sent
                   if c != "G0 Z60.000 F600")


def test_a_latched_read_failing_mid_job_fails_it():
    rig = Rig()
    seen = dict(n=0)

    def hook(esp, method, path):
        if path == "/binary_sensor/probe_latched":
            seen["n"] += 1
            if seen["n"] == 40:
                esp.fail[path] = 4                 # every try of one read fails
    rig.device.hooks.append(hook)
    job, _, moments = run_job(rig)
    assert job.phase == "failed" and "probe_latched" in job.error
    # it failed high in the first descent from the homed top, so the head stays up there
    assert moments[0]["z"] > 60.0 and moments[0]["last_press"] == "probe_stow"


def test_a_single_dropped_latched_read_is_retried():
    rig = Rig()
    seen = dict(n=0)

    def hook(esp, method, path):
        if path == "/binary_sensor/probe_latched":
            seen["n"] += 1
            if seen["n"] == 40:
                esp.fail[path] = 1
    rig.device.hooks.append(hook)
    job, _, _ = run_job(rig)
    assert job.phase == "done", job.error


def test_no_points_is_refused():
    rig = Rig()
    job, said, _ = run_job(rig, points=[])
    assert job.phase == "failed" and "no points" in job.error


def test_only_whitelisted_gcode_is_ever_sent():
    """A job, a calibration touch, a position read, an aborted job and a failed job.

    The laser stays plugged in, so the firmware is in laser mode: no laser, fan or heater
    command may ever leave the plugin."""
    rig = Rig(surface=surface)
    run_job(rig, offset=17.0)
    run_job(rig, points=[dict(x=150.0, y=150.0)], mode="calibrate")
    rig.bridge.run([])                                       # "Add current position"
    run_job(rig, abort_when=lambda m, c: c == "G0 Z15.000 F300")
    rig.device.surface = lambda x, y: 70.0
    run_job(rig)
    run_job(rig, tap=False)                                  # a refused tap test
    rig.machine.z = 20.0
    rig.device.surface = surface
    run_job(rig, abort_when=lambda m, c: c == "G0 Z15.000 F300")   # starting below safe_z
    sent = rig.machine.sent
    assert len(sent) > 200
    for command in sent:
        words = command.split()
        assert words[0] in ALLOWED, command
        if words[0] == "M92":
            assert words == ["M92"], command
        if words[0] == "M211":
            assert words == ["M211", "S1"], command
        if words[0] == "M5":
            assert words == ["M5"], command
        if words[0] != "M211":
            assert not any(w[0] in "SEPT" for w in words[1:] if words[0] != "M118"), command
    for banned in ("G1", "M3", "M4", "M106", "M107", "M104", "M109", "M140", "M190", "G92"):
        assert not any(c.split()[0] == banned for c in sent), banned


def test_the_tap_test_refuses_the_job_without_a_tap():
    rig = Rig()
    job, said, moments = run_job(rig, tap=False)
    assert job.phase == "failed" and "no tap" in job.error
    assert not any(is_step(c) for c in rig.machine.sent)      # no probing step
    assert moments[0]["last_press"] == "probe_stow" and not rig.device.deployed
    assert rig.clock.now - 1000.0 >= 60.0                              # it waited the full minute


def test_the_tap_test_runs_before_calibration_too():
    rig = Rig()
    job, said, _ = run_job(rig, points=[dict(x=150.0, y=150.0)], mode="calibrate", tap=False)
    assert job.phase == "failed" and "no tap" in job.error


def test_the_tap_test_can_be_turned_off_with_a_warning():
    rig = Rig(settings=dict(tap_test=False))
    job, said, _ = run_job(rig, tap=False)
    assert job.phase == "done", job.error
    assert any(p.get("warning") and "tap test is turned off" in p["message"] for p in said if p)
    assert job.result["tap_test"] is False


def test_the_tap_test_refuses_a_stuck_output():
    rig = Rig()
    rig.device.deploy_glitch = False
    real = rig.device._deploy

    def stuck():
        real()
        rig.device.alarm = True                    # output HIGH while deployed
    rig.device._deploy = stuck
    job, _, _ = run_job(rig)
    assert job.phase == "failed" and "before the tap test" in job.error
    assert "probe_reset" in rig.device.presses


def test_a_job_started_above_safe_z_never_moves_down_unprobed():
    rig = Rig(surface=surface, settings=dict(home_first=False))
    rig.machine.z = 140.0
    downs = []

    def hook(machine, command):
        words = command.split()
        z = [float(w[1:]) for w in words[1:] if w.startswith("Z")]
        if words[0] == "G0" and z and z[0] < machine.z - 1e-6 and not is_step(command):
            downs.append((command, machine.z))
    rig.machine.hooks.append(hook)
    job, _, _ = run_job(rig)
    assert job.phase == "done", job.error
    assert downs == []
    first = next(c for c in rig.machine.sent if is_step(c))
    assert first == "G0 Z139.500 F300"


def test_homing_goes_up_and_nothing_moves_down_before_probing():
    rig = Rig(surface=surface)
    rig.machine.z = 30.0
    job, _, _ = run_job(rig)
    assert job.phase == "done", job.error
    first_step = next(i for i, c in enumerate(rig.machine.sent) if is_step(c))
    zs = [float(w[1:]) for c in rig.machine.sent[:first_step] if c.startswith("G0")
          for w in c.split()[1:] if w.startswith("Z")]
    assert zs == []                                          # after G28 the head is at the top


def test_a_reboot_mid_job_aborts_safely():
    rig = Rig(surface=surface)

    def hook(machine, command):
        if command == "G0 Z30.000 F300":
            rig.device.reboot()
    rig.machine.hooks.append(hook)
    job, said, moments = run_job(rig)
    assert job.phase == "failed" and "restarted" in job.error
    assert moments[0]["z"] == 60.0 and moments[0]["last_press"] == "probe_stow"


def test_a_reboot_is_caught_without_an_uptime_sensor():
    rig = Rig(surface=surface, uptime_path=None)

    def hook(machine, command):
        if command == "G0 Z30.000 F300":
            rig.device.reboot()
    rig.machine.hooks.append(hook)
    job, _, _ = run_job(rig)
    assert job.phase == "failed" and "trigger_count" in job.error


def test_the_module_xy_offset_places_the_pin():
    rig = Rig()
    ir = dict(name="2W IR", focus_offset=None, dx=-10.0, dy=5.0)
    job, _, _ = run_job(rig, points=[dict(x=100.0, y=100.0)], module=ir)
    assert job.phase == "done", job.error
    assert "G0 X110.000 Y95.000 F3000" in rig.machine.sent
    assert job.result["dx"] == -10.0 and job.result["xy_calibrated"] is True


def test_an_unmeasured_module_xy_offset_warns_and_uses_zero():
    rig = Rig()
    ir = dict(name="2W IR", focus_offset=None, dx=None, dy=None)
    job, said, _ = run_job(rig, points=[dict(x=100.0, y=100.0)], module=ir)
    assert job.phase == "done", job.error
    assert any(p.get("warning") and "2W IR has no measured probe XY offset" in p["message"]
               for p in said if p)
    assert "G0 X100.000 Y100.000 F3000" in rig.machine.sent
    assert job.result["xy_calibrated"] is False


def test_a_job_without_heights_is_refused():
    rig = Rig()
    rig.config = dict(rig.config, safe_z=None)
    job, _, _ = run_job(rig)
    assert job.phase == "failed" and "safe_z" in job.error
    assert rig.machine.sent == []


def test_an_unconfirmed_safe_end_is_not_called_safe():
    rig = Rig(surface=surface)
    job, said, _ = run_job(rig, abort_when=lambda m, c: c == "G0 Z20.000 F300")
    assert said[-1]["type"] == "aborted" and "safe height" in said[-1]["message"]
    assert not said[-1].get("unconfirmed")
    rig2 = Rig(surface=surface)
    rig2.device.fail = {"/button/probe_stow/press": -1}      # the pin cannot be stowed
    job, said, _ = run_job(rig2)
    assert said[-1]["unconfirmed"] and "could NOT confirm" in said[-1]["message"]


def _job_with(rig, session, mode="job", tap=False, points=POINTS):
    said = []
    job = Job(rig.bridge, rig.esp, said.append, rig.config, points if mode != "tap" else [], mode=mode,
              module=dict(TEN_W), clock=rig.clock, session=session)
    def say(payload):
        said.append(payload)
        if tap and payload and payload.get("waiting_for_tap"):
            rig.device.tap()
    job._say = say
    job.run()
    return job, said


def test_a_job_uses_the_sessions_tap_instead_of_asking_again():
    # Chris tests the probe once (or it is assumed); the grid job must not wait for a finger.
    from lhp_pkg.routine import TapSession
    rig = Rig(surface=surface)
    session = TapSession()
    tap, _ = _job_with(rig, session, mode="tap", tap=True)
    assert tap.phase == "done" and session.ok
    job, said = _job_with(rig, session, tap=False)
    assert job.phase == "done", job.error
    assert not any(p and p.get("waiting_for_tap") for p in said)
    assert any(p and "passed earlier this session" in p.get("message", "") for p in said)


def test_a_stale_session_tap_is_asked_for_again():
    from lhp_pkg.routine import TapSession
    rig = Rig(surface=surface)
    session = TapSession()
    _job_with(rig, session, mode="tap", tap=True)
    rig.device.reboot()                                       # the probe board restarted since
    job, said = _job_with(rig, session, tap=False)
    assert job.phase == "failed" and "no tap" in job.error
