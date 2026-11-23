"""The serial link guard, replayed against the real stalls of 2026-11-23."""
import os
import re
from datetime import datetime

from lhp_pkg import linkguard

DATA = os.path.join(os.path.dirname(__file__), "data", "serial")
LINE = re.compile(r"^(\S+ \S+) - (Send|Recv): (.*)$")


class Clock(object):
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def replay(name, guard=None, step=0.1, hang_s=0.0):
    """Feed a serial.log excerpt through a guard with a clock driven by the log's own times,
    stepping the guard every `step` seconds of log time. `hang_s` carries on past the excerpt's
    end for that long with only the 2 s temperature reports arriving, as the real hangs did (they
    lasted minutes). Returns (guard, actions)."""
    clock = Clock()
    acts = []
    g = guard or linkguard.LinkGuard(fake_ack=lambda: acts.append("fake_ack"), stop=lambda: acts.append("stop"),
                                     power_off=lambda: acts.append("power"), clock=clock)
    g.clock = clock
    last_step = None
    for raw in open(os.path.join(DATA, name), errors="replace"):
        m = LINE.match(raw.rstrip("\n"))
        if not m:
            continue
        t = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S,%f").timestamp()
        if last_step is None:
            last_step = t
        while last_step + step <= t:              # time passes between lines: the guard checks
            last_step += step
            clock.t = last_step
            g.step()
        clock.t = t
        if m.group(2) == "Send":
            g.sent(m.group(3))
            g.last_send_t = t
        else:
            g.received(m.group(3))
    end = last_step
    g.hang_end = end
    while last_step is not None and last_step + step <= end + hang_s:
        last_step += step
        clock.t = last_step
        if int((last_step - end) / 2.0) != int((last_step - step - end) / 2.0):
            g.received(" T:0.00 /0.00 B:-12.81 /0.00")
        g.step()
        if acts and not hasattr(g, "caught_at"):
            g.caught_at = last_step
    return g, acts


def test_a_burn_line_is_a_g1_that_fires():
    assert linkguard.is_burn("G1 X156.091 Y166.549 Z145.824 B-243.894 S46 F2219")
    assert not linkguard.is_burn("G1 X10 S0 F1200")
    assert not linkguard.is_burn("G0 X156.597 Y116.827 B217.716 F3000")
    assert not linkguard.is_burn("M3 S0")
    assert not linkguard.is_burn("M400")


def test_each_real_stall_is_caught_by_faking_the_lost_ok():
    for name in ("stall1_glued_report.log", "stall2_band_pass.log", "stall3_after_travel.log"):
        g, acts = replay(name, hang_s=30.0)
        assert acts[:1] == ["fake_ack"], (name, acts)
        # caught once the queued motion has had its time plus ok_s: within 10 s of the last line sent
        assert g.events[0][0] - g.last_send_t <= 10.0, (name, g.events[0][0] - g.last_send_t)


def test_normal_burning_and_a_long_m3_wait_are_left_alone():
    for name in ("normal_minute.log", "m3_wait_after_travel.log"):
        g, acts = replay(name)
        assert acts == [], (name, acts)


def test_a_silent_printer_stops_the_burn_then_cuts_the_power():
    clock = Clock()
    acts = []
    g = linkguard.LinkGuard(fake_ack=lambda: acts.append("fake_ack"), stop=lambda: acts.append("stop"),
                            power_off=lambda: acts.append("power"), clock=clock)
    g.sent("G1 X1 B10 S46 F2000")
    for t in (1.0, 2.5):                           # no temperature reports: the printer is silent
        clock.t = t
        g.step()
    assert acts == ["stop"]
    clock.t = 6.0
    g.step()
    assert acts == ["stop", "power"]


def test_a_second_lost_ok_before_a_real_one_stops_the_burn_and_a_real_ok_cancels_the_power_cut():
    clock = Clock()
    acts = []
    g = linkguard.LinkGuard(fake_ack=lambda: acts.append("fake_ack"), stop=lambda: acts.append("stop"),
                            power_off=lambda: acts.append("power"), clock=clock)
    g.sent("G1 X1 B10 S46 F2000")
    clock.t = 1.0
    g.received(" T:0.00 /0.00 B:-12.81 /0.00")
    clock.t = 2.2
    g.step()
    assert acts == ["fake_ack"]
    g.sent("G1 X2 B11 S46 F2000")                  # OctoPrint sends on after the fake ok
    clock.t = 3.0
    g.received(" T:0.00 /0.00 B:-12.81 /0.00")
    clock.t = 4.5
    g.step()
    assert acts == ["fake_ack", "stop"]
    clock.t = 5.0
    g.received("ok")                               # the printer answers again: no power cut
    clock.t = 9.0
    g.step()
    assert acts == ["fake_ack", "stop"]


def test_a_real_ok_resets_the_count_so_occasional_losses_each_get_one_fake():
    clock = Clock()
    acts = []
    g = linkguard.LinkGuard(fake_ack=lambda: acts.append("fake_ack"), stop=lambda: acts.append("stop"),
                            clock=clock)
    for k in range(3):
        base = k * 10.0
        clock.t = base
        g.sent("G1 X1 B10 S46 F2000")
        clock.t = base + 1.0
        g.received(" T:0.00 /0.00")
        clock.t = base + 2.5
        g.step()
        g.sent("G1 X2 B11 S46 F2000")
        clock.t = base + 2.6
        g.received("ok")
    assert acts == ["fake_ack"] * 3


def test_an_ok_held_back_by_a_full_queue_of_slow_moves_is_not_lost():
    # 21:30 and 21:34 on 2026-11-23: the guard faked oks that were only late behind a 6 s turn,
    # then stopped the burn; the oks all arrived at the quickstop
    for name in ("slow_queue_not_lost.log", "slow_queue_not_lost2.log"):
        g, acts = replay(name)
        assert acts == [], (name, acts)


def test_the_motion_estimate_uses_length_over_feed():
    g = linkguard.LinkGuard(fake_ack=lambda: None, stop=lambda: None)
    assert g.move_seconds("G0 X0 Y0 Z0 B0 F3000") == 0.0          # first sight of each axis: no estimate
    assert abs(g.move_seconds("G0 B-148 F1500") - 5.92) < 0.01
    assert abs(g.move_seconds("G1 X30 S10") - 60 * (30 ** 2) ** 0.5 / 1500) < 1e-9   # F is modal
    assert g.move_seconds("M400") == 0.0
