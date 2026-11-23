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


def replay(name, guard=None, step=0.1):
    """Feed a serial.log excerpt through a guard with a clock driven by the log's own times,
    stepping the guard every `step` seconds of log time. Returns (guard, actions)."""
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
        else:
            g.received(m.group(3))
    return g, acts


def test_a_burn_line_is_a_g1_that_fires():
    assert linkguard.is_burn("G1 X156.091 Y166.549 Z145.824 B-243.894 S46 F2219")
    assert not linkguard.is_burn("G1 X10 S0 F1200")
    assert not linkguard.is_burn("G0 X156.597 Y116.827 B217.716 F3000")
    assert not linkguard.is_burn("M3 S0")
    assert not linkguard.is_burn("M400")


def test_each_real_stall_is_caught_by_faking_the_lost_ok():
    for name in ("stall1_glued_report.log", "stall2_band_pass.log", "stall3_after_travel.log"):
        g, acts = replay(name)
        assert acts[:1] == ["fake_ack"], (name, acts)


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
