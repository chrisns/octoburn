"""The HTTP client for the probe's ESP: URLs, fail-closed reads, best-effort display."""
import pytest

from lhp_pkg.esp import EspError, EspProbe, parse_state


class Recorder(object):
    def __init__(self, replies=None, failures=0):
        self.calls = []
        self.replies = replies or {}
        self.failures = failures

    def __call__(self, method, url, timeout):
        self.calls.append((method, url, timeout))
        if self.failures:
            self.failures -= 1
            raise IOError("timed out")
        return self.replies.get(url, "")


def test_urls_and_methods():
    t = Recorder({"http://p/binary_sensor/probe_latched": '{"value": true, "state": "ON"}'})
    esp = EspProbe("http://p/", 1.5, transport=t)
    esp.arm()
    esp.clear()
    esp.stow()
    esp.reset()
    esp.check()
    assert esp.latched() is True
    assert [c[:2] for c in t.calls] == [
        ("POST", "http://p/button/probe_arm/press"), ("POST", "http://p/button/latch_clear/press"),
        ("POST", "http://p/button/probe_stow/press"), ("POST", "http://p/button/probe_reset/press"),
        ("POST", "http://p/button/probe_check/press"), ("GET", "http://p/binary_sensor/probe_latched")]
    assert all(c[2] == 1.5 for c in t.calls)


def test_display_urls():
    t = Recorder()
    esp = EspProbe("http://p", transport=t)
    assert esp.show_state("probing") and esp.show_z(12.3456) and esp.show_focus(-1.5)
    assert [c[1] for c in t.calls] == ["http://p/select/job_state/set?option=probing",
                                       "http://p/number/result_z/set?value=12.346",
                                       "http://p/number/result_focus/set?value=-1.500"]
    assert esp.show_state("lasering") is False
    assert esp.show_focus(None)
    assert t.calls[-1][1] == "http://p/number/result_focus/set?value=-1000.000"


def test_parse_state_fails_closed():
    assert parse_state('{"value": false, "state": "OFF"}', "x") is False
    assert parse_state('{"state": "ON"}', "x") is True
    for body in ("", "<html>", "[]", '{"state": "unknown"}', '{"value": "yes"}', '{"value": null}'):
        with pytest.raises(EspError):
            parse_state(body, "x")


def test_get_rides_out_a_slow_wifi_spell_then_fails():
    # 2026-11-18: the probe's Wi-Fi went to 1 s pings and two quick tries both timed out,
    # failing a Probe here. Reads happen with the head stopped, so waiting is safe.
    waits = []
    t = Recorder({"http://p/binary_sensor/probe_raw": '{"value": false}'}, failures=3)
    assert EspProbe("http://p", transport=t, sleep=waits.append).raw() is False
    assert len(t.calls) == 4 and waits == [0.5, 1.0, 2.0]
    waits = []
    t = Recorder(failures=4)
    with pytest.raises(EspError) as caught:
        EspProbe("http://p", transport=t, sleep=waits.append).latched()
    assert len(t.calls) == 4 and waits == [0.5, 1.0, 2.0]
    assert "4 tries" in str(caught.value)


def test_post_failures_raise_but_display_failures_do_not():
    t = Recorder(failures=10)
    esp = EspProbe("http://p", transport=t)
    with pytest.raises(EspError):
        esp.arm()
    with pytest.raises(EspError):
        esp.stow()
    assert esp.show_state("done") is False
    assert esp.show_z(1.0) is False


def test_status_reports_errors_instead_of_raising():
    t = Recorder(failures=10)
    status = EspProbe("http://p", transport=t).status()
    assert status["latched"] is None and "error" in status


def test_trigger_count():
    t = Recorder({"http://p/sensor/trigger_count": '{"id":"sensor-trigger_count","value":7,"state":"7"}'})
    assert EspProbe("http://p", transport=t).trigger_count() == 7


def test_a_wifi_blip_on_a_safe_press_is_retried():
    # 2026-11-20 02:30: one timed-out latch_clear stopped a 42-point grid at point 10.
    calls, waits = [], []
    def t(method, url, timeout):
        calls.append((method, url))
        if len(calls) < 3:
            raise OSError("timed out")
        return ""
    esp = EspProbe("http://p", transport=t, sleep=waits.append)
    esp.clear()
    assert calls == [("POST", "http://p/button/latch_clear/press")] * 3 and len(waits) == 2
    def dead(method, url, timeout):
        raise OSError("timed out")
    import pytest as _pytest
    with _pytest.raises(EspError, match="latch_clear"):
        EspProbe("http://p", transport=dead, sleep=lambda s: None).clear()
