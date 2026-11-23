"""One point against the simulated machine and probe."""
import pytest

from lhp_pkg.probe import Aborted, ProbeError
from helpers import Rig, is_step


def trigger_height(rig, x, y):
    return rig.device.surface(x, y) + rig.device.trigger_travel


def assert_within_one_fine_step(rig, result):
    t = trigger_height(rig, result["x"], result["y"])
    fine = rig.config["stages"][-1][0]
    # z tripped and z + fine_step did not: the true trigger lies in [z, z + fine_step)
    assert result["z"] <= t + 1e-9
    assert t - result["z"] < fine + 1e-9


def test_flat_surface_within_one_fine_step():
    rig = Rig()
    result = rig.prober().measure(150.0, 120.0)
    assert_within_one_fine_step(rig, result)
    assert result["n"] == 2 and result["spread"] == 0.0
    assert rig.machine.z == pytest.approx(60.0)            # back at safe_z
    assert not rig.device.deployed and rig.device.presses[-1] == "probe_stow"


@pytest.mark.parametrize("x,y", [(20.0, 30.0), (160.0, 175.0), (300.0, 330.0)])
def test_sloped_surface_within_one_fine_step(x, y):
    rig = Rig(surface=lambda x, y: 4.0 + 0.031 * x + 0.017 * y)
    assert_within_one_fine_step(rig, rig.prober().measure(x, y))


def test_heights_between_points_match_the_surface():
    rig = Rig(surface=lambda x, y: 8.0 if x < 100 else 11.137)
    p = rig.prober()
    low, high = p.measure(50, 50), p.measure(150, 50)
    assert high["z"] - low["z"] == pytest.approx(3.137, abs=0.02)


def test_rearms_between_stages_and_for_each_sample():
    rig = Rig()
    rig.prober().measure(100, 100)
    arms = rig.device.presses.count("probe_arm")
    # 3 stages + 1 repeat sample
    assert arms == 4
    # each arm is followed by a latch clear, and each stage's start is above its trigger
    assert rig.device.presses.count("latch_clear") == 4


def test_stage_starts_follow_the_triggers():
    rig = Rig()
    r = rig.prober().measure(100, 100)
    coarse, medium, fine = r["stage_triggers"]
    zs = rig.gcode_z_commands()
    assert round(coarse + 0.6, 3) in zs
    assert round(medium + 0.15, 3) in zs
    assert coarse - medium < 0.5 + 1e-9 and medium - fine < 0.1 + 1e-9


def test_probe_offset_moves_the_head_not_the_target():
    rig = Rig()
    r = rig.prober(dx=12.5, dy=-4.0).measure(100.0, 100.0)
    assert (r["head_x"], r["head_y"]) == (87.5, 104.0)
    assert any(c.startswith("G0 X87.500 Y104.000") for c in rig.machine.sent)


def test_never_commands_below_the_floor_and_reports_no_trigger():
    # the surface is below the floor: the probe can never trip
    rig = Rig(surface=lambda x, y: -10.0, settings=dict(z_floor=2.0))
    with pytest.raises(ProbeError, match="no trigger"):
        rig.prober().measure(100, 100)
    assert min(rig.gcode_z_commands()) >= 2.0
    assert rig.machine.z == pytest.approx(60.0) and rig.device.presses[-1] == "probe_stow"


def test_never_descends_more_than_max_descent():
    rig = Rig(surface=lambda x, y: -10.0, settings=dict(max_descent=7.0))
    with pytest.raises(ProbeError, match="no trigger"):
        rig.prober().measure(100, 100)
    assert min(rig.gcode_z_commands()) >= 60.0 - 7.0 - 1e-9


def test_alarm_on_arm_is_an_error_and_resets_the_probe():
    rig = Rig(surface=lambda x, y: 70.0)                   # the head is below the surface
    rig.machine.z = 50.0
    with pytest.raises(ProbeError, match="can't deploy|touched while deploying"):
        rig.prober().measure(100, 100)
    assert "probe_reset" in rig.device.presses
    assert all(not is_step(c) for c in rig.machine.sent)   # never stepped down


def test_a_deploy_edge_is_refused_as_a_touch():
    """An edge during the deploy counts, so it is refused. If a real probe made one on every
    deploy, every arm would fail this way: that would need this check revisited."""
    rig = Rig(deploy_glitch=True)
    with pytest.raises(ProbeError, match="touched while deploying"):
        rig.prober().measure(100, 100)
    assert "probe_reset" in rig.device.presses


def test_pin_deployed_onto_the_surface_is_caught_by_trigger_count():
    """The blind spot: the deploy trips and stows, and latch_clear would hide it. Trigger
    height 12.7345; coarse trips at 12.5, medium (from 13.1) at 12.7; a fine start_above of
    0.03 deploys at 12.73, below the trigger height. trigger_count went up during the deploy, so
    no step is taken from there: the head climbs 1 mm, deploys again and measures."""
    rig = Rig(settings=dict(stages="0.5; 0.1, 0.6; 0.02, 0.03"))
    r = rig.prober().measure(100, 100)
    zs = rig.gcode_z_commands()
    assert zs[zs.index(12.73) + 1] == pytest.approx(13.73)    # climbed, no step down from 12.73
    assert "probe_reset" in rig.device.presses
    assert 0 <= 12.7345 - r["z"] < 0.02 + 1e-9
    assert rig.machine.z == pytest.approx(60.0)


def test_a_deploy_that_always_touches_still_fails_safe():
    rig = Rig(settings=dict(stages="0.5; 0.1, 0.6; 0.02, 0.03"))
    real = rig.device._deploy

    def deploy():
        real()
        if rig.machine.z < 50:                   # every deploy near the surface trips
            rig.device.count += 1
    rig.device._deploy = deploy
    with pytest.raises(ProbeError, match="touched while deploying"):
        rig.prober().measure(100, 100)
    assert rig.machine.z == pytest.approx(60.0)


def test_the_window_still_bounds_a_stage_that_never_trips():
    """If the pin stows without a trace (no count), the window ends the fine stage one
    coarser step below the coarser trigger, not at z_floor."""
    rig = Rig(settings=dict(stages="0.5; 0.1, 0.6; 0.02, 0.15"))
    real = rig.device._deploy
    deploys = dict(n=0)

    def deploy():
        deploys["n"] += 1
        real()
        if deploys["n"] == 3:                    # the fine stage's pin is silently dead
            rig.device.deployed = False
    rig.device._deploy = deploy
    with pytest.raises(ProbeError, match="no trigger"):
        rig.prober().measure(100, 100)
    zs = rig.gcode_z_commands()
    fine = zs[zs.index(12.85):]
    assert min(fine) >= 12.6 - 1e-9


def test_repeat_samples_take_more_when_one_disagrees():
    rig = Rig(settings=dict(samples=2, tolerance=0.05, max_samples=4))
    # per deploy: coarse, medium, fine, then repeats. The first repeat is a stray +0.08.
    rig.device.jitter = [0.0, 0.0, 0.0, 0.08, 0.01]
    r = rig.prober().measure(100, 100)
    assert len(r["samples"]) == 3 and r["n"] == 2
    assert r["rejected"] == [pytest.approx(12.81)]          # on the 12.73 + k * 0.02 grid
    assert r["spread"] <= 0.05 + 1e-9
    kept = [s for s in r["samples"] if s not in r["rejected"]]
    assert r["z"] == pytest.approx(sum(kept) / 2)


def test_repeat_samples_agree_on_the_same_grid():
    rig = Rig()
    r = rig.prober().measure(100, 100)
    assert r["samples"][0] == r["samples"][1] and r["rejected"] == []


def test_repeat_samples_give_up_beyond_max_samples():
    rig = Rig(settings=dict(samples=2, tolerance=0.03, max_samples=3))
    rig.device.jitter = [0.0, 0.0, 0.0, 0.08, -0.06]
    with pytest.raises(ProbeError, match="tolerance"):
        rig.prober().measure(100, 100)
    assert rig.machine.z == pytest.approx(60.0)
    assert len([c for c in rig.device.presses if c == "probe_arm"]) == 5


def test_agreeing_groups():
    from lhp_pkg.probe import agreeing
    assert agreeing([1.0, 1.02, 1.5], 0.05) == ([0, 1], False)
    assert agreeing([1.5, 1.0, 1.02, 1.04], 0.05) == ([1, 2, 3], False)
    assert agreeing([1.0], 0.05) == ([0], False)
    # two different pairs qualify: the samples do not say which is right
    assert agreeing([12.72, 12.80, 12.76], 0.05)[1] is True
    assert agreeing([12.72, 12.80, 12.76, 12.74], 0.05) == ([0, 2, 3], False)
    # A350 2026-11-17: two groups of three share the two 143.46 touches: that core agrees.
    assert agreeing([143.44, 143.50, 143.46, 143.46], 0.05, need=2) == ([2, 3], False)
    # ... but a core smaller than needed stays ambiguous.
    assert agreeing([12.72, 12.80, 12.76], 0.05, need=2)[1] is True


def test_an_ambiguous_tie_takes_another_sample():
    rig = Rig(settings=dict(samples=2, tolerance=0.05, max_samples=4))
    # fine 12.73, then 12.81 and 12.77: two pairs qualify, so a fourth sample decides
    rig.device.jitter = [0.0, 0.0, 0.0, 0.08, 0.04, 0.0]
    r = rig.prober().measure(100, 100)
    assert r["taken"] == 4 and r["n"] == 3 and r["rejected"] == [pytest.approx(12.81)]


def test_an_ambiguous_tie_at_max_samples_fails():
    rig = Rig(settings=dict(samples=2, tolerance=0.05, max_samples=3))
    rig.device.jitter = [0.0, 0.0, 0.0, 0.08, 0.04]
    with pytest.raises(ProbeError, match="tolerance"):
        rig.prober().measure(100, 100)


def test_a_single_sample_when_asked():
    rig = Rig(settings=dict(samples=1, max_samples=1))
    assert rig.prober().measure(100, 100)["n"] == 1


def test_abort_mid_descent_raises_and_stows():
    rig = Rig()
    state = dict(aborted=False)

    def hook(machine, command):
        if command == "G0 Z30.000 F300":
            state["aborted"] = True
            rig.bridge.interrupt()
    rig.machine.hooks.append(hook)
    with pytest.raises((Aborted, RuntimeError)):
        rig.prober(aborted=lambda: state["aborted"]).measure(100, 100)
    assert rig.machine.z == pytest.approx(60.0)
    assert rig.device.presses[-1] == "probe_stow" and not rig.device.deployed
    assert min(rig.gcode_z_commands()) >= 29.999


def test_abort_during_the_arm_wait():
    rig = Rig()
    state = dict(n=0)

    def aborted():
        state["n"] += 1
        return rig.device.deployed           # abort as soon as the pin is out
    with pytest.raises(Aborted):
        rig.prober(aborted=aborted).measure(100, 100)
    assert not any(is_step(c) for c in rig.machine.sent)
    assert rig.device.presses[-1] == "probe_stow"


def test_a_clamped_move_is_refused():
    rig = Rig()
    with pytest.raises(ProbeError, match="soft endstop"):
        rig.prober().measure(330.0, 100.0)          # X max is 320


def test_failed_latch_read_stops_the_descent():
    rig = Rig()
    calls = dict(n=0)

    def hook(esp, method, path):
        if path == "/binary_sensor/probe_latched":
            calls["n"] += 1
            if calls["n"] > 5:
                esp.fail[path] = -1
    rig.device.hooks.append(hook)
    from lhp_pkg.esp import EspError
    with pytest.raises(EspError):
        rig.prober().measure(100, 100)
    assert rig.machine.z == pytest.approx(60.0)


def step_sizes(rig):
    zs = [float(c.split()[1][1:]) for c in rig.machine.sent if is_step(c)]
    return [round(a - b, 3) for a, b in zip(zs, zs[1:]) if a > b]


def test_refinement_stages_approach_at_the_coarser_step_and_only_go_fine_at_the_end():
    # 2026-11-19: with the shipped stages each fine sample climbed ~2 mm (start_above plus a
    # deploy climb) and stepped all of it at 0.02 mm, ~97 steps at ~0.18 s each. A refinement
    # stage now steps at the coarser stage's size down to just above where that stage last
    # did not trip, then fine: the same steps through the surface, a fifth of the steps above it.
    rig = Rig(settings=dict(stages="0.5; 0.1, 1.5; 0.02, 1.0"))
    bad = unprobed_down_moves(rig)
    r = rig.prober().measure(150.0, 120.0)
    assert bad == []
    assert_within_one_fine_step(rig, r)
    assert r["n"] == 2 and r["spread"] == 0.0
    sizes = step_sizes(rig)
    assert sizes.count(0.02) <= 2 * 10
    assert sizes.count(0.1) <= 10 + 2 * 10


def test_a_trip_during_the_approach_restarts_the_stage_from_it():
    # The surface reads 0.3 mm higher after the coarse trip (noise, or it moved): the next
    # stage's approach trips. That trip brackets the surface like a coarse one, so the stage
    # restarts from it and the result is still within one fine step of the new surface.
    base, state = 20.0, {"bumped": False}
    rig = Rig(surface=lambda x, y: base + (0.3 if state["bumped"] else 0.0),
              settings=dict(stages="0.5; 0.1, 1.5; 0.02, 1.0"))
    p = rig.prober()
    real_descend = p.descend

    def descend(*a, **k):
        out = real_descend(*a, **k)
        state["bumped"] = True
        return out
    p.descend = descend
    bad = unprobed_down_moves(rig)
    r = p.measure(150.0, 120.0)
    assert bad == []
    t = base + 0.3 + rig.device.trigger_travel
    assert r["z"] <= t + 1e-9 and t - r["z"] < 0.02 + 1e-9


def test_a_probe_left_in_a_silent_alarm_is_reset_and_checked_before_the_first_descent():
    # A probe that keeps its output LOW and ignores deploy passed the old deploy check, and the
    # head would step down with the pin up. The first arm of a point must catch it.
    rig = Rig()
    rig.device.silent_alarm = True
    bad = unprobed_down_moves(rig)
    r = rig.prober().measure(150.0, 120.0)
    assert bad == []
    assert_within_one_fine_step(rig, r)
    presses = rig.device.presses
    assert presses.index("probe_reset") < presses.index("probe_arm")
    assert "probe_touch_mode" in presses


def test_a_pin_that_will_not_come_out_stops_the_point_before_any_step():
    rig = Rig()
    rig.device.stuck_up = True
    with pytest.raises(ProbeError, match="did not deploy"):
        rig.prober().measure(150.0, 120.0)
    assert not any(is_step(c) for c in rig.machine.sent)


def test_a_stow_that_fails_between_points_is_caught_at_the_next_point():
    rig = Rig()
    rig.device.stow_fails = [True]
    p = rig.prober()
    p.measure(150.0, 120.0)
    rig.esp.stow()                                                          # this stow fails
    assert rig.device.silent_alarm
    r = p.measure(150.0, 120.0)
    assert_within_one_fine_step(rig, r)


def test_a_slow_reset_gets_a_second_try_before_the_point_is_refused():
    # 2026-11-19 02:04: the installed guard refused 2 of 3 healthy in-air runs: touch mode read
    # too soon after a reset. One more reset with a longer wait before calling it an alarm.
    rig = Rig()
    rig.device.slow_resets = 1          # the first reset has not finished when touch mode reads
    r = rig.prober().measure(150.0, 120.0)
    assert_within_one_fine_step(rig, r)
    assert rig.device.presses.count("probe_reset") >= 3


def test_a_stow_then_a_reset_is_the_last_try_before_an_alarm():
    # 2026-11-22 21:05: two resets left the probe in alarm by the chuck; a stow then a reset cleared it.
    rig = Rig()
    rig.device.slow_resets = 2
    r = rig.prober().measure(150.0, 120.0)
    assert_within_one_fine_step(rig, r)
    presses = rig.device.presses
    third = [i for i, n in enumerate(presses) if n == "probe_reset"][2]
    assert presses[third - 1] == "probe_stow"


def test_the_pin_check_can_be_switched_off():
    rig = Rig(settings=dict(verify_pin=False))
    rig.prober().measure(150.0, 120.0)
    assert "probe_touch_mode" not in rig.device.presses


def test_machine_z_follows_the_step_counter():
    rig = Rig(shift=-3.25)
    r = rig.prober().measure(100, 100)
    assert r["machine_z"] == pytest.approx(r["z"] - 3.25, abs=0.003)


def unprobed_down_moves(rig):
    """Watch every Z move: a probing step may only go down, by at most one coarse step, and
    every other Z move may only go up."""
    bad = []

    def hook(machine, command):
        words = command.split()
        z = [float(w[1:]) for w in words[1:] if w.startswith("Z")]
        if not z or words[0] not in ("G0", "G1") or machine.relative:
            return
        if is_step(command):
            if not machine.z - 0.5 - 1e-6 <= z[0] <= machine.z + 1e-6:
                bad.append((command, machine.z))
        elif z[0] < machine.z - 1e-6:
            bad.append((command, machine.z))
    rig.machine.hooks.append(hook)
    return bad


def test_a_head_above_safe_z_travels_where_it_is_and_probes_down():
    rig = Rig()
    rig.machine.z = 150.0
    bad = unprobed_down_moves(rig)
    r = rig.prober().measure(100, 100)
    assert bad == []
    first_step = next(i for i, c in enumerate(rig.machine.sent) if is_step(c))
    before = [c for c in rig.machine.sent[:first_step] if c.startswith("G0")]
    assert before == ["G0 X100.000 Y100.000 F3000"]          # no Z move at all before probing
    assert rig.machine.sent[first_step] == "G0 Z149.500 F300"
    assert_within_one_fine_step(rig, r)


def test_a_head_below_safe_z_goes_up_first():
    rig = Rig()
    rig.machine.z = 30.0
    bad = unprobed_down_moves(rig)
    rig.prober().measure(100, 100)
    assert bad == []
    assert rig.machine.sent[:6].count("G0 Z60.000 F600") == 1


def test_an_object_taller_than_safe_z_fails_the_point_and_backs_up():
    rig = Rig(surface=lambda x, y: 80.0)                     # trips at 82.5, above safe_z 60
    rig.machine.z = 120.0
    bad = unprobed_down_moves(rig)
    with pytest.raises(ProbeError, match="does not clear the object"):
        rig.prober().measure(100, 100)
    assert bad == []
    assert rig.machine.z == pytest.approx(120.0)             # back up the probed column
    last_xy = max(i for i, c in enumerate(rig.machine.sent) if c.startswith("G0 X"))
    first_step = next(i for i, c in enumerate(rig.machine.sent) if is_step(c))
    assert last_xy < first_step                              # no XY travel after the touch
    assert rig.device.presses[-1] == "probe_stow"


def test_every_non_probing_move_in_a_failure_is_upward():
    rig = Rig(surface=lambda x, y: -10.0, settings=dict(z_floor=40.0))
    rig.machine.z = 90.0
    bad = unprobed_down_moves(rig)
    with pytest.raises(ProbeError):
        rig.prober().measure(100, 100)
    assert bad == [] and rig.machine.z == pytest.approx(60.0)


def test_the_guard_refuses_an_unprobed_downward_move():
    rig = Rig()
    p = rig.prober()
    p.where()
    with pytest.raises(ProbeError, match="unprobed downward"):
        p._move_z(10.0)
    assert not any(c.startswith("G0 Z10") for c in rig.machine.sent)


def test_a_reboot_mid_descent_stops_the_point():
    rig = Rig()
    rig.device.count = 3                                     # triggers since boot, before the job

    def hook(machine, command):
        if command == "G0 Z40.000 F300":
            rig.device.reboot()
    rig.machine.hooks.append(hook)
    from lhp_pkg.esp import EspRebooted
    with pytest.raises(EspRebooted, match="restarted"):
        rig.prober().measure(100, 100)
    assert min(rig.gcode_z_commands()) >= 40.0 - 0.5
    assert rig.machine.z == pytest.approx(60.0) and rig.device.presses[-1] == "probe_stow"


@pytest.mark.parametrize("path", ["/sensor/uptime", "/sensor/Uptime", None])
def test_the_watchdog_finds_the_uptime_sensor(path):
    from lhp_pkg.esp import Watchdog
    rig = Rig(uptime_path=path)
    w = Watchdog(rig.esp, clock=rig.clock).start()
    assert w.uptime_path == path


def test_uptime_going_backwards_is_a_reboot_even_with_counts_intact():
    from lhp_pkg.esp import EspRebooted, Watchdog
    rig = Rig()
    w = Watchdog(rig.esp, clock=rig.clock).start()
    w.check("one", force_uptime=True)
    rig.device.uptime = 1.0                                  # restarted and already published
    with pytest.raises(EspRebooted, match="uptime"):
        w.check("two", force_uptime=True)


def test_uptime_unknown_after_it_was_known_is_a_reboot():
    from lhp_pkg.esp import EspRebooted, Watchdog
    rig = Rig()
    w = Watchdog(rig.esp, clock=rig.clock).start()
    rig.device.uptime = None
    with pytest.raises(EspRebooted):
        w.check("x", force_uptime=True)


def test_uptime_is_read_at_most_every_few_seconds_during_a_descent():
    from lhp_pkg.esp import Watchdog
    rig = Rig()
    w = Watchdog(rig.esp, clock=rig.clock, uptime_every_s=5.0).start()
    before = len([c for c in rig.device.calls if c[1] == "/sensor/uptime"])
    for _ in range(10):
        w.check("step")
    assert len([c for c in rig.device.calls if c[1] == "/sensor/uptime"]) == before
    rig.clock.now += 6.0
    w.check("step")
    assert len([c for c in rig.device.calls if c[1] == "/sensor/uptime"]) == before + 1


class FlakyBridge(object):
    """A bridge whose run() fails a set number of times, as a second Abort would make it."""

    def __init__(self, inner, failures):
        self.inner = inner
        self.failures = failures
        self.sent = []

    def run(self, commands, timeout=60.0):
        if self.failures:
            self.failures -= 1
            raise RuntimeError("interrupted while waiting for the printer")
        return self.inner.run(commands, timeout)

    def send(self, commands):
        self.sent.append(list(commands))
        self.inner.send(commands)


def test_safe_end_survives_a_second_abort():
    from lhp_pkg.probe import raise_to_safe
    rig = Rig()
    rig.machine.z = 12.0
    bridge = FlakyBridge(rig.bridge, failures=1)
    assert raise_to_safe(bridge, 60.0, 600) is True
    assert rig.machine.z == pytest.approx(60.0)
    assert not any(c == "G91" for c in rig.machine.sent)


def test_safe_end_falls_back_to_the_absolute_raise_when_the_position_is_known():
    from lhp_pkg.probe import raise_to_safe
    rig = Rig()
    rig.machine.z = 12.0
    bridge = FlakyBridge(rig.bridge, failures=0)
    original = bridge.run
    calls = dict(n=0)

    def run(commands, timeout=60.0):
        calls["n"] += 1
        if calls["n"] > 1:                                   # the read works, the moves do not
            raise RuntimeError("interrupted")
        return original(commands, timeout)
    bridge.run = run
    assert raise_to_safe(bridge, 60.0, 600) is False
    assert bridge.sent == [["G90", "G0 Z60.000 F600"]]
    assert rig.machine.z == pytest.approx(60.0)


def test_safe_end_raises_blind_only_when_the_position_is_unknown():
    from lhp_pkg.probe import raise_to_safe
    rig = Rig()
    rig.machine.z = 12.0
    bridge = FlakyBridge(rig.bridge, failures=10)
    assert raise_to_safe(bridge, 60.0, 600) is False
    assert bridge.sent == [["G91", "G0 Z5 F600", "G90"]]
    assert rig.machine.z == pytest.approx(17.0)


def test_probe_patterns():
    from lhp_pkg.probe import pattern_offsets, ProbeError
    sq = pattern_offsets("square", 4.0, 3)
    assert sq[:3] == [(-2.0, -2.0), (0.0, -2.0), (2.0, -2.0)] and sq[3] == (2.0, 0.0)   # serpentine
    di = pattern_offsets("diamond", 4.0, 3)
    assert set(di) >= {(0.0, -2.0), (2.0, 0.0), (0.0, 2.0), (-2.0, 0.0), (0.0, 0.0)} and len(di) == 9
    assert pattern_offsets("x", 10.0, 3) == [(-5.0, 0.0), (0.0, 0.0), (5.0, 0.0)]
    assert pattern_offsets("y", 10.0, 2) == [(0.0, -5.0), (0.0, 5.0)]
    import pytest
    with pytest.raises(ProbeError):
        pattern_offsets("circle", 4.0, 3)
