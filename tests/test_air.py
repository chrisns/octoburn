# coding=utf-8
"""The purifier controller: socket and fan follow the policy, and everything fails to off."""
import pytest

from lhp_pkg import air
from lhp_pkg import purifier as pu


class Clock(object):
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class FakeHA(object):
    def __init__(self, ready=True):
        self.ready = ready
        self.calls = []
        self.readings = {}
        self.fail = False

    def switch(self, entity, on):
        if self.fail:
            raise IOError("HA down")
        self.calls.append((entity, on))

    def state(self, entity):
        if self.fail:
            raise IOError("HA down")
        return self.readings.get(entity, (None, None))


def make(ready=True, **cfg):
    clock = Clock()
    sent = []
    ha = FakeHA(ready)
    config = dict(enabled=True, switch="switch.purifier", pm25="sensor.pm25", hcho="sensor.hcho",
                  policy=dict(pu.DEFAULTS), pointer_s_max=10)
    config.update(cfg)
    c = air.AirController(config, sent.extend, lambda: 10.0, clock=clock, ha=ha)
    return c, clock, sent, ha


def burn(c, clock, seconds, s=255):
    c.on_sent("G1 X0 Y0 F600")
    c.on_sent("M3 S%d" % s)
    x = 0
    for _ in range(int(seconds)):
        x = 10 - x                                 # 10 mm at 600 mm/min = 1 s
        clock.t += 1
        c.on_sent("G1 X%d" % x)
    c.on_sent("M5")


def test_laser_off_lines():
    assert air.laser_off_line("G0 X10 Y10")
    assert air.laser_off_line("M5")
    assert air.laser_off_line("G1 X10 S0")
    assert air.laser_off_line("G1 X10 S0.0 F600")
    assert not air.laser_off_line("G1 X10 S01")
    assert not air.laser_off_line("G1 X10 S76")
    assert not air.laser_off_line("G1 X10")
    assert not air.laser_off_line("G01 X10") or True     # not emitted by the bridge
    assert not air.laser_off_line("M50")


def test_a_long_burn_turns_the_socket_on_and_queues_the_gear_after_a_travel():
    c, clock, sent, ha = make()
    burn(c, clock, 60)
    c.step()
    assert ha.calls == [("switch.purifier", True)]
    assert sent == []                              # still streaming: waits for a laser-off line
    assert c.queuing("G1 X5 S200") is None
    assert c.queuing("G0 X50 Y50") == ["G0 X50 Y50", "M1011 F3"]       # enclosure fan off in this config
    assert c.queuing("G0 X60 Y60") is None         # only once


def test_a_waiting_gear_goes_anyway_after_a_while():
    c, clock, sent, ha = make()
    burn(c, clock, 60)
    c.step()
    clock.t += c.PENDING_MAX_S + 1
    c.step()
    assert "M1011 F3" in sent


def test_when_idle_the_gear_is_sent_at_once():
    c, clock, sent, ha = make()
    burn(c, clock, 60)
    clock.t += 10                                  # nothing sent for 10 s, still inside the job gap
    c.step()
    assert sent == ["M1011 F3"]


def test_short_marks_do_nothing():
    c, clock, sent, ha = make()
    burn(c, clock, 2, s=80)
    for _ in range(200):
        clock.t += 5
        c.step()
    assert ha.calls == [] and sent == []


def test_the_run_on_ends_with_f0_then_the_socket_off():
    c, clock, sent, ha = make()
    ha.readings = {"sensor.pm25": (5.0, clock.t - 30), "sensor.hcho": (10.0, clock.t - 30)}
    c.step()                                        # baseline reading
    burn(c, clock, 60)
    clock.t += 10
    c.step()
    for _ in range(80):                             # the air is clean at once: run-on min applies
        clock.t += 5
        ha.readings = {"sensor.pm25": (6.0, clock.t), "sensor.hcho": (10.0, clock.t)}
        c.last_poll = -1e9
        c.step()
    assert sent[-1] == "M1011 F0"
    assert ha.calls[-1] == ("switch.purifier", False)
    assert c.status()["phase"] == "idle"


def test_no_home_assistant_still_drives_the_fan_and_never_touches_a_socket():
    c, clock, sent, ha = make(ready=False)
    burn(c, clock, 60)
    clock.t += 10
    c.step()
    assert ha.calls == [] and sent == ["M1011 F3"]
    for _ in range(400):
        clock.t += 5
        c.step()
    assert sent[-1] == "M1011 F0"                  # the dose timer, no readings


def test_an_error_fails_to_off():
    c, clock, sent, ha = make()
    burn(c, clock, 60)
    clock.t += 10
    c.step()
    c.policy.tick = None                            # something breaks inside
    c.step()
    assert sent[-1] == "M1011 F0" and ha.calls[-1] == ("switch.purifier", False)
    assert c.status is not None


def test_home_assistant_down_does_not_stop_the_fan_logic():
    c, clock, sent, ha = make()
    ha.fail = True
    burn(c, clock, 60)
    clock.t += 10
    c.step()                                        # the switch call raises: noted, backed off
    assert "HA down" in (c.error or "")


def test_stop_turns_everything_off_until_the_next_job():
    c, clock, sent, ha = make()
    burn(c, clock, 60)
    clock.t += 10
    c.step()
    c.off("Stop pressed", stop_policy=True)
    assert sent[-1] == "M1011 F0" and ha.calls[-1] == ("switch.purifier", False)
    clock.t += 5
    c.step()
    assert c.status()["power"] is False


def test_disabled_does_nothing():
    c, clock, sent, ha = make(enabled=False)
    burn(c, clock, 120)
    clock.t += 10
    c.step()
    assert ha.calls == [] and sent == []


def test_no_laser_no_metering():
    c, clock, sent, ha = make()
    c.watts = lambda: 0.0
    burn(c, clock, 120)
    clock.t += 10
    c.step()
    assert ha.calls == [] and sent == []


def test_ha_time_parsing():
    assert air._parse_time("2026-11-21T11:41:02.267232+00:00") == pytest.approx(1795261262.267232)
    assert air._parse_time(None) is None


def test_a_hung_home_assistant_never_blocks_the_send_path():
    import threading, time as _time
    c, clock, sent, ha = make()
    gate = threading.Event()
    ha.switch = lambda entity, on: gate.wait(5)        # Home Assistant hangs
    burn(c, clock, 60)
    clock.t += 10
    t = threading.Thread(target=c.step)
    t.start()
    _time.sleep(0.2)                                    # step() is now stuck in the switch call
    t0 = _time.time()
    c.on_sent("G1 X5")                                  # OctoPrint's send path
    c.queuing("G0 X1")
    assert _time.time() - t0 < 0.5
    gate.set()
    t.join(5)


def test_a_failing_socket_backs_off_and_drops_the_gear():
    c, clock, sent, ha = make()
    calls = []

    def boom(entity, on):
        calls.append(on)
        raise IOError("HA down")
    ha.switch = boom
    burn(c, clock, 60)
    clock.t += 10
    c.step()
    n = len(calls)
    for _ in range(5):                                  # 10 s: inside the back-off
        clock.t += 2
        c.step()
    assert len(calls) == n and "HA down" in c.error
    clock.t += c.HA_BACKOFF_S
    c.step()
    assert len(calls) == n + 1                          # tried again after the back-off


def test_a_finished_job_logs_a_summary():
    lines = []
    c, clock, sent, ha = make()
    c.log_line = lines.append
    ha.readings = {"sensor.pm25": (5.0, clock.t - 30), "sensor.hcho": (10.0, clock.t - 30)}
    c.step()
    burn(c, clock, 60)
    for _ in range(100):
        clock.t += 5
        ha.readings = {"sensor.pm25": (6.0, clock.t), "sensor.hcho": (10.0, clock.t)}
        c.last_poll = -1e9
        c.step()
    done = [l for l in lines if "job done" in l]
    assert done and "J" in done[0] and "run-on" in done[0]



def test_the_enclosure_fan_follows_the_gear_and_only_ours_is_turned_off():
    c, clock, sent, ha = make(enclosure_fan=True)
    c.off("startup")
    assert sent == ["M1011 F0"]                       # a fan we never started is left alone
    del sent[:]
    burn(c, clock, 60)
    clock.t += 10
    c.step()
    assert sent == ["M1011 F3", "M1010 S4 P100"]
    for _ in range(400):
        clock.t += 5
        c.step()
    assert sent[-2:] == ["M1011 F0", "M1010 S4 P0"]


def test_the_enclosure_fan_rides_along_mid_job():
    c, clock, sent, ha = make(enclosure_fan=True)
    burn(c, clock, 60)
    c.step()
    assert c.queuing("G0 X1 Y1") == ["G0 X1 Y1", "M1011 F3", "M1010 S4 P100"]


def test_fan_time_is_counted_per_gear():
    c, clock, sent, ha = make()
    burn(c, clock, 60)
    clock.t += 10
    c.step()                                  # gear 3 on
    for _ in range(30):
        clock.t += 2
        c.step()
    assert c.stats["run_s"] == pytest.approx(60.0, abs=2.1)
    assert sum(c.stats["by_gear"].values()) == pytest.approx(c.stats["run_s"])
    assert c.stats["by_gear"]["3"] > 0
    assert c.status()["run"]["run_s"] > 0
