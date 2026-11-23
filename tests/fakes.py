"""Fakes that behave like the machine, not mocks that record calls.

FakeSnapmaker is a Snapmaker 2.0 in laser mode, as far as this plugin can see it: a head
that moves, soft endstops that clamp silently, a G92 shift between G-code and machine Z, and
M114 replies in the printer's own format (two decimals, then the step counters).

FakeEsp is the probe head's ESP32 behind its REST API. It watches the machine's moves. While
the pin is deployed, it latches when the head reaches the trigger height over the surface and
then stows the pin itself, as the probe does in its default mode. A pin deployed with the head
below the surface goes into alarm; one deployed with the head between the surface and the
trigger height trips at once and stows, which the plugin cannot see once the latch is cleared.

MachinePrinter is OctoPrint's printer interface over a FakeSnapmaker, so the tests drive the
real GcodeBridge, and the real EspProbe sits on FakeEsp.transport.
"""
import json
import re
from urllib.parse import parse_qs, urlparse


class FakeSnapmaker(object):

    def __init__(self, steps=400.0, shift=0.0, xy_max=(320.0, 350.0), xy_shift=(0.0, 0.0),
                 toolhead="10W LASER"):
        self.steps = steps
        self.shift = shift              # machine Z = G-code Z + shift (a G92 / work origin)
        self.home_offset = (0.0, 0.0, 0.0)   # M206 (the laser head's is X-19 Y-10): machine = G-code - M206
        self.xy_shift = xy_shift        # machine X, Y = G-code X, Y + xy_shift
        # The bed mesh as the A350 firmware keeps it: the print mesh (absolute, ~42 mm on a
        # quick swap printer), levelling off in laser mode, fade 10 mm.
        self.toolhead = toolhead
        self.mesh = dict(n=11, start=(52.0, 43.0), spacing=(28.0, 28.0),
                         z=[[round(42.0 + 0.01 * i - 0.007 * j, 3) for j in range(11)] for i in range(11)])
        self.levelling = False
        self.fade = 10.0
        self.fail_m421_at = None        # (i, j) refused with an error
        self.g1029_start = (52.0, 43.0)
        self.xy_max = xy_max
        self.x = self.y = 0.0
        self.b = 0.0                    # rotary, degrees
        self.rotary = True              # rotary module online: M114 reports B
        self.security, self.roll = 0x4, -86   # the 10W's safety report: parked in the bay
        self.crosshair = (21.5, -11.0)        # the 2W IR's stored crosshair offset (M2002 T8)
        self.z = 200.0
        self.relative = False
        self.soft_endstops = True
        self.sent = []
        self.listeners = []             # called after every move with the machine
        self.hooks = []                 # called before every command with (machine, command)


    def levelling_dz(self):
        """What the firmware adds to Z with levelling on: the mesh at the machine X, Y (the step
        count shows it, the reported Z does not, as on the A350)."""
        if not self.levelling:
            return 0.0
        from lhp_pkg.heightmap import evaluate
        m = self.mesh
        return evaluate(m["z"], m["start"], m["spacing"], self.x + self.xy_shift[0], self.y + self.xy_shift[1],
                        extrapolate=True)
    def execute(self, command):
        for hook in list(self.hooks):
            hook(self, command)
        self.sent.append(command)
        words = command.split()
        code = words[0]
        params = dict((w[0], float(w[1:])) for w in words[1:] if len(w) > 1 and w[0].isalpha()
                      and re.match(r"^-?\d*\.?\d+$", w[1:]))
        replies = []
        if code == "G90":
            self.relative = False
        elif code == "G91":
            self.relative = True
        elif code in ("G0", "G1"):
            for axis in "XYZ":
                if axis not in params:
                    continue
                value = getattr(self, axis.lower()) + params[axis] if self.relative else params[axis]
                if self.soft_endstops:
                    high = {"X": self.xy_max[0], "Y": self.xy_max[1], "Z": 330.0}[axis]
                    value = min(max(value, 0.0), high)
                setattr(self, axis.lower(), value)
            if "B" in params:
                self.b = self.b + params["B"] if self.relative else params["B"]
            for listener in list(self.listeners):
                listener(self)
        elif code == "G28":
            axes = [w[0] for w in words[1:] if w[0] in "XYZ"] or ["X", "Y", "Z"]
            for axis, home in (("X", 0.0), ("Y", 0.0), ("Z", 330.0)):
                if axis in axes:
                    setattr(self, axis.lower(), home)
            self.relative = False
            for listener in list(self.listeners):
                listener(self)
        elif code == "M211":
            if "S" in params:
                self.soft_endstops = params["S"] != 0
            replies.append("echo:Soft endstops: %s  Min:  X0.00 Y0.00 Z0.00   Max:  X%.2f Y%.2f Z330.00"
                           % ("On" if self.soft_endstops else "Off", self.xy_max[0], self.xy_max[1]))
        elif code == "M114":
            replies.append("X:%.2f Y:%.2f Z:%.2f E:0.00%s Count X:%d Y:%d Z:%d"
                           % (self.x, self.y, self.z, " B:%.2f" % self.b if self.rotary else "",
                              int(round((self.x + self.xy_shift[0]) * 400)),
                              int(round((self.y + self.xy_shift[1]) * 400)),
                              int(round((self.z + self.shift + self.levelling_dz()) * self.steps))))
        elif code == "M2002" and "T8" in words and self.toolhead in ("2W RED LASER", "20W LASER", "40W LASER"):
            replies.append("Get CrossLightOffset X: %f, Y: %f" % self.crosshair)
        elif code == "M2002" and "S1" in words and "LASER" in self.toolhead:
            replies += ["Get security status", "Laser security state: 0x%x" % self.security,
                        "Laser temp: 24, imu temp: 28, roll: %d, pitch: 2, pwm_pin_pulldown_state_: 0, "
                        "pwm_pin_pullup_state_: 1" % self.roll, "close Laser control"]
        elif code == "M1006":
            replies.append("echo:Tool Head: %s" % self.toolhead)
            if "LASER" in self.toolhead:
                replies += ["Current Status: OFF", "Current Power: 0.000000", "Focus Height: 11.20"]
        elif code == "M420":
            if "Z" in params:
                self.fade = params["Z"]
            if "S" in params:
                self.levelling = params["S"] != 0
            if "V" in words:
                n = self.mesh["n"]
                replies += ["Bilinear Leveling Grid:", "      " + "      ".join(str(i) for i in range(n))]
                for j in range(n):
                    replies.append(" %d %s" % (j, " ".join("%+.3f" % self.mesh["z"][i][j] for i in range(n))))
                replies += ["", "Subdivided with CATMULL ROM Leveling Grid:", " 0 +1.000 +2.000"]
            replies.append("echo:Bed Leveling %s" % ("ON" if self.levelling else "OFF"))
            replies.append("echo:Fade Height %s" % ("Off" if not self.fade else "%.2f" % self.fade))
        elif code == "G1029":
            n = int(params["P"])
            dx, dy = int(280 / (n - 1)), int(285 / (n - 1))
            self.mesh = dict(n=n, start=self.g1029_start, spacing=(float(dx), float(dy)),
                             z=[[9.0] * n for _ in range(n)])
            self.levelling = True
            replies += ["X:%g - %d" % (self.g1029_start[0], dx), "Y:%g - %d" % (self.g1029_start[1], dy),
                        "Set grid size : %d" % n]
        elif code == "M421":
            i, j = int(params["I"]), int(params["J"])
            if self.fail_m421_at == (i, j):
                self.fail_m421_at = None            # one refusal, then it works again
                replies.append("Error:Mesh XY or IJ cannot be resolved")
            else:
                self.mesh["z"][i][j] = round(params["Z"], 3)
        elif code == "M92":
            replies.append("echo: M92 X400.00 Y400.00 Z%.2f B888.89" % self.steps)
        elif code == "M503":
            replies.append("echo:  M92 X400.00 Y400.00 Z%.2f B888.89" % self.steps)
            replies.append("echo:  M206 X%.2f Y%.2f Z%.2f" % self.home_offset)
        elif code == "M118":
            replies.append(words[1])
        replies.append("ok")
        return replies


class MachinePrinter(object):
    """Just enough of OctoPrint's PrinterInterface, executing on a FakeSnapmaker."""

    def __init__(self, machine, operational=True):
        self.machine = machine
        self.operational = operational
        self.bridge = None

    def is_operational(self):
        return self.operational

    def fake_ack(self):
        self.fake_acks = getattr(self, "fake_acks", 0) + 1

    def commands(self, commands, tags=None, **kwargs):
        self.sent_commands = getattr(self, "sent_commands", []) + list(commands)
        for command in commands:
            for line in self.machine.execute(command):
                if self.bridge is not None:
                    self.bridge.on_gcode_received(None, line)


class FakeEsp(object):
    """The ESPHome probe device, reached through `transport(method, url, timeout)`."""

    def __init__(self, machine, surface=None, trigger_travel=2.5, deploy_glitch=False,
                 uptime_path="/sensor/uptime", holds_trigger=False, busy_reads=0):
        self.machine = machine
        self.surface = surface or (lambda x, y: 10.2345)
        self.trigger_travel = trigger_travel
        self.deploy_glitch = deploy_glitch
        self.holds_trigger = holds_trigger  # output stays HIGH after a trigger until a reset
        self.held = False
        # probe_busy reads ON this many times after each probe command (deploy, stow, reset).
        self.busy_reads = busy_reads
        self.busy_left = 0
        self.armed_while_busy = 0
        self.jitter = []                # trigger height offsets, one used per deploy
        self._jitter_now = 0.0
        self.deployed = False
        self.latched = False
        self.alarm = False
        # A probe that, after a failed stow, keeps its output LOW and ignores deploy until a reset
        # (one explanation considered on 2026-11-19, not proven). One entry per stow: True fails it.
        self.stow_fails = []
        self.silent_alarm = False
        self.stuck_up = False           # the pin never comes out, even after a reset
        self.touch = False              # touch (switch) mode: the output shows the pin position
        self.slow_resets = 0            # resets whose touch mode reads LOW (not finished yet)
        self._slow_now = False
        self.count = 0
        self.check_ok = True
        self.display = {}
        self.calls = []                 # (method, path)
        self.presses = []
        self.fail = {}                  # path prefix -> number of failures left (-1 = always)
        self.garbage = set()            # paths that answer with junk
        self.hooks = []                 # called with (esp, method, path) before each request
        self.uptime_path = uptime_path  # None: the device has no uptime sensor
        self.uptime = 100.0             # None: "NA", not published since boot
        self.boots = 1
        machine.listeners.append(self._moved)

    def tap(self):
        """A finger pushes the pin up."""
        if self.deployed and not self.alarm:
            self._trip()

    def reboot(self):
        """The ESP restarts: the latch and counters are gone, the probe keeps its pin."""
        self.boots += 1
        self.latched = False
        self.count = 0
        self.uptime = None

    # -- physics ------------------------------------------------------------

    def trigger_z(self, x, y):
        return self.surface(x, y) + self.trigger_travel + self._jitter_now

    def _moved(self, machine):
        if self.deployed and not self.alarm and machine.z <= self.trigger_z(machine.x, machine.y) + 1e-9:
            self._trip()

    def _trip(self):
        self.latched = True
        self.count += 1
        self.deployed = False           # default mode stows the pin after a trigger
        if self.holds_trigger:
            self.held = True

    def _deploy(self):
        m = self.machine
        self._jitter_now = self.jitter.pop(0) if self.jitter else 0.0
        if m.z <= self.surface(m.x, m.y) + 1e-9:
            self.alarm = True           # the pin cannot come out
            self.latched = True
            self.deployed = False
        elif m.z <= self.trigger_z(m.x, m.y) + 1e-9:
            self._trip()                # deployed onto the surface: trips and stows at once
        else:
            self.deployed = True
            if self.deploy_glitch:
                self.latched = True     # a spurious edge from the deploy itself
                self.count += 1

    # -- HTTP ---------------------------------------------------------------

    def transport(self, method, url, timeout):
        parsed = urlparse(url)
        path = parsed.path
        for hook in list(self.hooks):
            hook(self, method, path)
        self.calls.append((method, path))
        for prefix, left in list(self.fail.items()):
            if path.startswith(prefix) and left != 0:
                if left > 0:
                    self.fail[prefix] = left - 1
                raise IOError("timed out (fake) %s" % path)
        if path in self.garbage:
            return "<html>busy</html>"
        query = parse_qs(parsed.query)
        if method == "POST" and path.startswith("/button/") and path.endswith("/press"):
            name = path.split("/")[2]
            self.presses.append(name)
            if name == "probe_arm" and self.busy_left > 0:
                self.armed_while_busy += 1      # a real ESP would queue it and deploy late
            if name in ("probe_arm", "probe_stow", "probe_reset"):
                self.busy_left = self.busy_reads
            if name != "probe_touch_mode" and name != "latch_clear":
                self.touch = False
            if name == "probe_arm":
                self.latched = False
                if not self.held and not self.silent_alarm and not self.stuck_up:
                    self._deploy()      # a held or alarmed clone ignores deploy until a reset
            elif name == "probe_deploy":
                if not self.held and not self.silent_alarm and not self.stuck_up and not self.deployed:
                    self._deploy()
            elif name == "probe_touch_mode":
                if not self.silent_alarm:
                    if not self.deployed and self.count is not None:
                        self.latched = True     # pin up: the output goes HIGH, a rising edge
                        self.count += 1
                    self.touch = True
            elif name == "latch_clear":
                self.latched = False
            elif name == "probe_stow":
                self.deployed = False
                if self.stow_fails and self.stow_fails.pop(0):
                    self.silent_alarm = True
            elif name == "probe_reset":
                self._slow_now = self.slow_resets > 0
                self.slow_resets = max(0, self.slow_resets - 1)
                self.alarm = False
                self.silent_alarm = False
                self.held = False
                self.deployed = False
            elif name == "probe_check":
                self.deployed = False
            elif name == "probe_set_od_mode":
                self.od_mode = True
            else:
                raise IOError("404 %s" % path)
            return ""
        if method == "GET" and path == "/binary_sensor/probe_busy":
            busy = self.busy_left > 0
            self.busy_left = max(0, self.busy_left - 1)
            return json.dumps({"id": "binary_sensor-probe_busy", "value": busy,
                               "state": "ON" if busy else "OFF"})
        if method == "GET" and path.startswith("/binary_sensor/"):
            name = path.split("/")[2]
            touch_high = self.touch and not self.deployed and not self.silent_alarm and not self._slow_now
            value = dict(probe_latched=self.latched, probe_raw=self.alarm or self.held or touch_high,
                         probe_check_ok=self.check_ok)[name]
            return json.dumps({"id": "binary_sensor-%s" % name, "value": value,
                               "state": "ON" if value else "OFF"})
        if method == "GET" and self.uptime_path is not None and path == self.uptime_path:
            if self.uptime is None:
                self.uptime = 0.5        # the first publish after the NA reply
                return json.dumps({"id": "sensor-uptime", "value": None, "state": "NA"})
            self.uptime += 0.1
            return json.dumps({"id": "sensor-uptime", "value": self.uptime,
                               "state": "%.0f s" % self.uptime})
        if method == "GET" and path == "/sensor/trigger_count":
            return json.dumps({"id": "sensor-trigger_count", "value": float(self.count),
                               "state": "%d" % self.count})
        if method == "POST" and path == "/select/job_state/set":
            self.display["job_state"] = query["option"][0]
            return ""
        if method == "POST" and path.startswith("/number/") and path.endswith("/set"):
            self.display[path.split("/")[2]] = float(query["value"][0])
            return ""
        raise IOError("404 %s %s" % (method, path))


class FakeClock(object):
    """Time that passes only when someone sleeps."""

    def __init__(self):
        self.now = 1000.0

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += max(seconds, 0.001)


class ScriptPrinter(object):
    """A printer that answers set replies, for the bridge tests."""

    def __init__(self, replies=None, operational=True, drop_marker=False):
        self.operational = operational
        self.sent = []
        self.replies = dict(replies or {})
        self.bridge = None
        self.drop_marker = drop_marker

    def is_operational(self):
        return self.operational

    def commands(self, commands, tags=None):
        self.sent.extend(commands)
        for command in commands:
            for line in self.replies.get(command.split()[0], []):
                self.bridge.on_gcode_received(None, line)
            if command.startswith("M118 ") and not self.drop_marker:
                self.bridge.on_gcode_received(None, command.split()[1])
