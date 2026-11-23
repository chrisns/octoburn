"""Parsers and the bridge."""
import logging

import pytest

from lhp_pkg import gcode
from fakes import ScriptPrinter

M114 = "X:304.00 Y:128.50 Z:-1.12 E:-14.00 Count X:134400 Y:60600 Z:16656 B:0"


def test_position_reads_the_m114_reply_not_the_counters():
    assert gcode.position([M114]) == (304.0, 128.5, -1.12)
    assert gcode.position(["ok", "X:1.00 Y:2.00 Z:3.00 E:0.00 Count X:0 Y:0 Z:0", "ok"]) == (1.0, 2.0, 3.0)
    assert gcode.position(["ok"]) is None
    # a G92-style echo without counters is not taken as a position
    assert gcode.position(["X:5.00 Y:5.00 Z:5.00 E:0.00"]) is None


def test_count_z_uses_the_step_counter():
    assert gcode.count_z([M114], 400.0) == pytest.approx(41.64)
    assert gcode.count_z(["ok"], 400.0) is None


def test_z_steps_per_mm():
    assert gcode.z_steps_per_mm(["echo: M92 X400.00 Y400.00 Z400.00 B888.89"]) == 400.0
    assert gcode.z_steps_per_mm(["echo:  M92 X160.00 Y160.00 Z800.00 E830.00"]) == 800.0
    assert gcode.z_steps_per_mm(["ok"], default=123.0) == 123.0


def bridge_with(printer):
    bridge = gcode.GcodeBridge(printer, logging.getLogger("t"))
    printer.bridge = bridge
    return bridge


def test_run_appends_m400_m114_and_a_numbered_marker():
    printer = ScriptPrinter(replies={"M114": [M114]})
    bridge = bridge_with(printer)
    lines = bridge.run(["G1 Z5"])
    assert printer.sent == ["G1 Z5", "M400", "M114", "M118 LHPSYNC1"]
    assert M114 in lines and "LHPSYNC1" in lines
    bridge.run([])
    assert printer.sent[-1] == "M118 LHPSYNC2"


def test_run_times_out_without_the_marker():
    bridge = bridge_with(ScriptPrinter(drop_marker=True))
    with pytest.raises(gcode.Timeout):
        bridge.run(["G0 Z5"], timeout=0.05)


def test_interrupt_during_a_run_raises():
    printer = ScriptPrinter()
    bridge = bridge_with(printer)
    original = printer.commands

    def commands(cmds, tags=None):
        bridge.interrupt()
        original(cmds, tags)
    printer.commands = commands
    with pytest.raises(RuntimeError, match="interrupted"):
        bridge.run(["G0 Z5"])


def test_refuses_when_not_connected():
    bridge = bridge_with(ScriptPrinter(operational=False))
    with pytest.raises(RuntimeError):
        bridge.run(["M114"])
    with pytest.raises(RuntimeError):
        bridge.send(["M114"])


# 2026-11-19 23:09:53, 2W IR: the M400's "ok" landed inside the laser's unasked status report,
# OctoPrint never saw an ok and stalled until a fake ack.
GLUED = "Laser temp: 29, imu temp: 37, roll: 0, pitch: 0, pwm_ok"


def test_an_ok_glued_into_the_laser_report_is_handed_back_as_ok():
    assert gcode.unglue_ok(GLUED) == "ok"
    assert gcode.unglue_ok("Laser temp: 29, imok") == "ok"
    assert gcode.unglue_ok("Laser security state: 0x0ok") == "ok"
    assert gcode.unglue_ok("pin_pulldown_state_: 0, pwm_pin_pullup_state_: 1ok") == "ok"


def test_ordinary_lines_pass_through_unchanged():
    for line in ("ok", "ok T:0.00", "Laser temp: 29, imu temp: 33, roll: 0, pitch: 0, "
                 "pwm_pin_pulldown_state_: 0, pwm_pin_pullup_state_: 1",
                 "Laser security state: 0x0", "echo:Unknown command: \"Mok\"", "Tool Head: 2W RED LASER",
                 "", None):
        assert gcode.unglue_ok(line) == line


def test_enclosure_door_from_m1010_and_events():
    from lhp_pkg import gcode as g
    report = ["Enclosure online: On", "Enclosure: On", "Enclosure door: Open", "Enclosure light power: 0"]
    assert g.enclosure_door(report) == "open"
    assert g.enclosure_door(["Enclosure door: Closed"]) == "closed"
    assert g.enclosure_door(["Enclosure online: Off"]) is None
    assert g.door_event("laser limit: 10") == "open"
    assert g.door_event("laser limit: 255") == "closed"
    assert g.door_event("Enclosure door: Closed") == "closed"
    assert g.door_event("ok") is None


def test_purifier_report():
    from lhp_pkg import gcode as g
    lines = ["Purifier Filter lifetime:low", "Purifier Fan work: 1", "Purifier Fan speed: 4050 rpm",
             "Purifier Fan gears: 1", "Purifier Fan extend power: 23815 mv", "Purifier err code:0X2"]
    info = g.purifier_info(lines)
    assert info == dict(lifetime="low", work=1, rpm=4050, gear=1, power_mv=23815, err=2)
    assert g.purifier_info(["Purifier is not exist."]) is None
    assert g.purifier_info(["ok"]) is None
