"""A clone that holds its output HIGH after a trigger and ignores deploy and stow until a reset.

Chris's 3DTouch-type clone does this (found on the real probe, 2026-11-16): after each tap the
output stayed HIGH, a plain probe_arm or probe_stow changed nothing, and only probe_reset brought
it back. Without a reset before each deploy, the second arm of a staged descent found the
output HIGH and refused with "probe alarm".
"""
import pytest

from lhp_pkg.probe import ProbeError
from helpers import Rig
from test_here import jogged, run, tapped


def resets_before_arms(presses):
    """For each probe_arm, whether a probe_reset came since the previous arm."""
    out, since = [], False
    for p in presses:
        if p == "probe_reset":
            since = True
        elif p == "probe_arm":
            out.append(since)
            since = False
    return out


def test_a_staged_descent_resets_the_held_output_before_each_rearm():
    rig = Rig(holds_trigger=True)
    result = rig.prober().measure(100, 100)
    t = rig.device.surface(100, 100) + rig.device.trigger_travel
    assert 0 <= t - result["z"] < rig.config["stages"][-1][0] + 1e-9
    # every arm follows a reset: the first is the verified arm's own (2026-11-19), every later
    # one follows a trigger the clone holds
    assert resets_before_arms(rig.device.presses) == [True, True, True, True]


def test_a_probe_that_does_not_hold_is_reset_only_by_the_verified_first_arm():
    rig = Rig()
    rig.prober().measure(100, 100)
    first_arm = rig.device.presses.index("probe_arm")
    assert "probe_reset" not in rig.device.presses[first_arm:]           # no reset at a re-arm


def test_tap_test_then_probe_here_with_a_held_trigger():
    rig = Rig(holds_trigger=True, surface=lambda x, y: 4.0)
    session = tapped(rig)                    # leaves the output held HIGH
    assert rig.device.held
    jogged(rig, 50.0, 60.0, 12.0)
    job, _ = run(rig, "here", session)
    assert job.phase == "done", job.error
    assert job.result["points"][0]["z"] == pytest.approx(6.5, abs=0.02)


def test_a_second_tap_test_resets_the_held_trigger_first():
    rig = Rig(holds_trigger=True)
    tapped(rig)
    tapped(rig)
    assert resets_before_arms(rig.device.presses) == [False, True]


def test_an_output_still_high_after_the_reset_is_an_alarm():
    rig = Rig()
    rig.device.alarm = True
    original = rig.device.transport

    def stuck(method, url, timeout):         # the reset does not clear it: a real fault
        reply = original(method, url, timeout)
        rig.device.alarm = True
        return reply
    rig.esp._transport = stuck
    with pytest.raises(ProbeError, match="stays HIGH after a reset"):
        rig.prober().measure(100, 100)
    assert rig.machine.sent == [] or not any(c.startswith("G0 Z") for c in rig.machine.sent[1:])
