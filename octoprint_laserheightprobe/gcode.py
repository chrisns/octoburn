# coding=utf-8
"""Serial helpers: send G-code, wait for proof that it ran, and read the replies.

The bridge is the one from OctoPrint-NozzleTouch. Each batch ends with M400, M114 and a
numbered M118 echo, and the caller waits for that number. M400 matters: the Snapmaker 2.0
answers a state query at once, before queued moves have run, so a reading taken without it
describes where the head was one step ago. Because every batch ends with M400 and M114, a
probe read made after run() returns describes the head at the position just commanded.

NozzleTouch's split-ok workaround is not here. It repairs an "ok" swallowed by the dual
toolhead's "active extruder mismatch" message, and the laser bay never sends that message.
Without it, a lost marker is a plain Timeout, which stops the job with the head raised.

The parsers at the bottom are plain functions so the tests can feed them captured lines.
"""
from __future__ import absolute_import

import re
import threading

TAG = "plugin:laserheightprobe"
SYNC = "LHPSYNC"
_SYNC_RE = re.compile(SYNC + r"(?P<n>\d+)")

_COUNT_RE = re.compile(r"Count X:\s*(-?\d+)\s+Y:\s*(-?\d+)\s+Z:\s*(-?\d+)")
_POSITION_RE = re.compile(r"X:\s*(-?\d+\.?\d*)\s+Y:\s*(-?\d+\.?\d*)\s+Z:\s*(-?\d+\.?\d*)")
_STEPS_RE = re.compile(r"M92\b.*?\bZ(?P<z>\d+(?:\.\d+)?)")
_STEPS_XY_RE = re.compile(r"M92\b.*?\bX(?P<x>\d+(?:\.\d+)?)\s+Y(?P<y>\d+(?:\.\d+)?)")
# The mesh parsers are NozzleTouch's, which were checked against the A350's own replies.
_GRID_AXIS_RE = re.compile(r"^(?:Recv:\s*)?(?:echo:)?\s*(?P<axis>[XY]):\s*(?P<start>-?\d+(?:\.\d+)?)"
                           r"\s+-\s+(?P<spacing>-?\d+(?:\.\d+)?)\s*$")
_GRID_SIZE_RE = re.compile(r"Set grid size\s*:\s*(\d+)")
_GRID_ROW_RE = re.compile(r"^(?:Recv:\s*)?\s*(?P<j>\d+)\s+(?P<values>[+-]\d+\.\d+(?:\s+[+-]\d+\.\d+)*)\s*$")
_LEVELLING_RE = re.compile(r"Bed Leveling (?P<state>ON|OFF)")
_TOOLHEAD_RE = re.compile(r"Tool Head:\s*(?P<name>[A-Za-z0-9 ]+?)\s*$")


class Timeout(Exception):
    """The printer did not answer in time."""


def position(lines):
    """(x, y, z) in G-code coordinates from the last M114 reply in `lines`, or None.

    Only the part before "Count" is read: the step counters after it are not millimetres.
    """
    found = None
    for line in lines:
        head = line.split("Count")[0]
        match = _POSITION_RE.search(head)
        if match is not None and "Count" in line:
            found = tuple(float(v) for v in match.groups())
    return found


_B_RE = re.compile(r"\bB:\s*(-?\d+(?:\.\d+)?)")


def b_position(lines):
    """B in degrees from the last M114 reply in `lines` (the part before "Count"), or None."""
    found = None
    for line in lines:
        if "Count" not in line:
            continue
        match = _B_RE.search(line.split("Count")[0])
        if match is not None:
            found = float(match.group(1))
    return found


def count_z(lines, steps_per_mm):
    """Machine Z from the step counter of an M114 reply, or None.

    G92 and a work origin shift the G-code Z but not the counter, so this tells a changed
    coordinate system apart from a changed object.
    """
    value = None
    for line in lines:
        match = _COUNT_RE.search(line)
        if match is not None:
            value = int(match.group(3)) / float(steps_per_mm)
    return value


def z_steps_per_mm(lines, default=400.0):
    """Z steps/mm from a bare M92 (or M503) reply, or `default` when the reply has none."""
    for line in lines:
        match = _STEPS_RE.search(line)
        if match is not None:
            return float(match.group("z"))
    return default


def machine_xy(lines, steps_xy):
    """Machine X, Y from the step counters of the last M114 reply, or None.

    The firmware's bed mesh is indexed by machine (native) X, Y. A G92 or work origin (LightBurn's
    start G-code sets one) moves the G-code X, Y but not the counters.
    """
    value = None
    for line in lines:
        match = _COUNT_RE.search(line)
        if match is not None:
            value = (int(match.group(1)) / float(steps_xy[0]), int(match.group(2)) / float(steps_xy[1]))
    return value


_M206_RE = re.compile(r"M206\s+X\s*(-?\d+(?:\.\d+)?)\s+Y\s*(-?\d+(?:\.\d+)?)\s+Z\s*(-?\d+(?:\.\d+)?)")


def home_offset(lines):
    """(x, y, z) from the "M206 X.. Y.. Z.." line of an M503 reply, or None. The Snapmaker's
    laser mode has M206 X-19 Y-10: M114 = step counters + M206 + any G92/work origin."""
    found = None
    for line in lines:
        match = _M206_RE.search(line)
        if match is not None:
            found = tuple(float(v) for v in match.groups())
    return found


def work_offset(lines):
    """(dx, dy, dz) of a G92 or work origin from one reply holding M503 (for M92 and M206) and
    M114, or None if any part is missing. Zero means the G-code frame is the homed one."""
    steps_xy, steps_z = xy_steps_per_mm(lines), z_steps_per_mm(lines, default=None)
    found, home = position(lines), home_offset(lines)
    if steps_xy is None or steps_z is None or found is None or home is None:
        return None
    mxy, mz = machine_xy(lines, steps_xy), count_z(lines, steps_z)
    if mxy is None or mz is None:
        return None
    return tuple(round(v, 3) for v in (mxy[0] - found[0] + home[0], mxy[1] - found[1] + home[1],
                                       mz - found[2] + home[2]))


def xy_steps_per_mm(lines):
    """(X, Y) steps/mm from a bare M92 reply, or None. No default: a wrong guess would put the
    height map in the wrong place."""
    for line in lines:
        match = _STEPS_XY_RE.search(line)
        if match is not None:
            return float(match.group("x")), float(match.group("y"))
    return None


_SECURITY_RE = re.compile(r"Laser security state:\s*0x(?P<state>[0-9a-fA-F]+)")
_ATTITUDE_RE = re.compile(r"Laser temp:\s*(?P<temp>-?\d+),\s*imu temp:\s*-?\d+,\s*roll:\s*(?P<roll>-?\d+),"
                          r"\s*pitch:\s*(?P<pitch>-?\d+)")


# Fragments of the laser's unasked status report (M2002 S1, and after M3/M5). It arrives over the
# toolhead bus a moment after the command's "ok", and once (2W IR, 2026-11-19 23:09:53) the two
# interleaved: "...pitch: 0, pwm_ok" then "pin_pulldown_state_: 0, ...". OctoPrint only
# recognises a line starting with "ok", so it waited for an ok it had already been sent.
_REPORT_BITS = ("Laser security state", "Laser temp", "imu temp", "roll:", "pitch:", "pwm_", "pin_pull")


def unglue_ok(line):
    """A laser-report fragment with the printer's "ok" glued to its end -> "ok"; else the line."""
    if line and line != "ok" and line.endswith("ok") and any(b in line for b in _REPORT_BITS):
        return "ok"
    return line


def laser_security(lines):
    """The laser's own safety report (M2002 S1, or unasked at connect), or None:
    dict(state=int, roll=deg, pitch=deg, temp=C). The firmware will not fire while state is
    not 0. Seen on the 10W parked in the bay: state 0x4, roll -86 (lying on its side)."""
    out = {}
    for line in lines:
        m = _SECURITY_RE.search(line)
        if m:
            out["state"] = int(m.group("state"), 16)
        m = _ATTITUDE_RE.search(line)
        if m:
            out.update(temp=int(m.group("temp")), roll=int(m.group("roll")), pitch=int(m.group("pitch")))
    return out if "state" in out else None


_PURIFIER_RE = {
    "lifetime": re.compile(r"Purifier Filter lifetime:\s*(\w+)"),
    "work": re.compile(r"Purifier Fan work:\s*(\d+)"),
    "rpm": re.compile(r"Purifier Fan speed:\s*(\d+)"),
    "gear": re.compile(r"Purifier Fan gears:\s*(\d+)"),
    "power_mv": re.compile(r"Purifier Fan extend power:\s*(\d+)"),
    "err": re.compile(r"Purifier err code:\s*0[xX]([0-9a-fA-F]+)"),
}


def purifier_info(lines):
    """The purifier's report to a bare M1011 (firmware Purifier::DisplayInfo), or None when it is
    not connected ("Purifier is not exist."). lifetime: "low" / "medium" / "normal" (the filter)."""
    out = {}
    for line in lines:
        if "Purifier is not exist" in line:
            return None
        for key, rx in _PURIFIER_RE.items():
            m = rx.search(line)
            if m:
                out[key] = m.group(1).lower() if key == "lifetime" else int(m.group(1), 16 if key == "err" else 10)
    return out or None


_DOOR_RE = re.compile(r"Enclosure door:\s*(?P<state>Open|Closed)", re.I)
_LIMIT_RE = re.compile(r"laser limit:\s*(?P<pwm>\d+)")


def enclosure_door(lines):
    """"open", "closed" from M1010's report ("Enclosure door: Open"), or None (no enclosure or no
    report). The firmware keeps it from the enclosure's CAN events."""
    found = None
    for line in lines:
        m = _DOOR_RE.search(line)
        if m:
            found = m.group("state").lower()
    return found


def door_event(line):
    """"open" / "closed" from an unasked line: the firmware drops the laser's power limit when the
    door opens ("laser limit: 10" on the 10W, 2026-11-19) and restores 255 when it shuts; M1010's
    report says it outright. None for any other line."""
    m = _DOOR_RE.search(line or "")
    if m:
        return m.group("state").lower()
    m = _LIMIT_RE.search(line or "")
    if m:
        return "closed" if int(m.group("pwm")) >= 255 else "open"
    return None


_CROSS_RE = re.compile(r"Get CrossLightOffset X:\s*(?P<x>-?\d+(?:\.\d+)?),\s*Y:\s*(?P<y>-?\d+(?:\.\d+)?)")


def crosshair_offset(lines):
    """(x, y) from M2002 T8 ("Get CrossLightOffset X: %f, Y: %f"), or None. Stored in the
    module; taken as crosshair minus beam (the firmware adds it to the target in WORK mode)."""
    found = None
    for line in lines:
        m = _CROSS_RE.search(line)
        if m:
            found = (float(m.group("x")), float(m.group("y")))
    return found


def rotary_online(lines):
    """(online, unusable) from an M114 reply: the firmware prints B only while the rotary is
    online, and "B unusable" when it is attached but cannot be used."""
    return b_position(lines) is not None, any("B unusable" in line for line in lines)


def toolhead(lines):
    """The toolhead name from an M1006 reply ("10W LASER", "LASER", "3DP", ...), or None."""
    for line in lines:
        text = line[5:] if line.startswith("Recv:") else line
        match = _TOOLHEAD_RE.search(text.strip())
        if match is not None:
            return match.group("name").strip()
    return None


def levelling(lines):
    """True when an M420 reply says "Bed Leveling ON", False for OFF, None if it says neither."""
    state = None
    for line in lines:
        match = _LEVELLING_RE.search(line)
        if match is not None:
            state = match.group("state") == "ON"
    return state


def grid_geometry(lines):
    """(size, (x0, y0), (dx, dy)) from the reply to G1029 P<size>, or None."""
    axes, size = {}, None
    for line in lines:
        match = _GRID_AXIS_RE.search(line.strip())
        if match is not None:
            axes[match.group("axis")] = (float(match.group("start")), float(match.group("spacing")))
        match = _GRID_SIZE_RE.search(line)
        if match is not None:
            size = int(match.group(1))
    if size is None or set(axes) != {"X", "Y"}:
        return None
    return size, (axes["X"][0], axes["Y"][0]), (axes["X"][1], axes["Y"][1])


def bilinear_grid(lines):
    """The stored mesh from an M420 V reply, as z[i][j] (i along X), or None.

    The printout has one row per Y index. Reading stops at the subdivided grid that follows.
    """
    rows = {}
    reading = False
    for line in lines:
        text = line[5:] if line.startswith("Recv:") else line
        if "Bilinear Leveling Grid" in text:
            reading = True
            continue
        if not reading:
            continue
        if "Subdivided" in text or "Leveling" in text:
            break
        match = _GRID_ROW_RE.search(text)
        if match is not None:
            rows[int(match.group("j"))] = [float(v) for v in match.group("values").split()]
    if not rows:
        return None
    ny = max(rows) + 1
    nx = len(rows[0])
    if sorted(rows) != list(range(ny)) or any(len(r) != nx for r in rows.values()):
        return None
    return [[rows[j][i] for j in range(ny)] for i in range(nx)]


def errors(lines):
    return [line for line in lines if "Error" in line or "error:" in line.lower()]


class GcodeBridge(object):
    """Sends G-code and waits for the reply that proves it ran."""

    def __init__(self, printer, logger):
        self._printer = printer
        self._logger = logger
        self._lock = threading.Lock()
        self._receive_lock = threading.Lock()
        self._event = threading.Event()
        self._collecting = False
        self._interrupted = False
        self._lines = []
        self._marker = 0

    # -- OctoPrint hook ---------------------------------------------------

    def on_gcode_received(self, comm_instance, line, *args, **kwargs):
        if self._collecting and line:
            with self._receive_lock:
                if not self._collecting:
                    return line
                self._lines.append(line)
                match = _SYNC_RE.search(line)
                if match is not None and int(match.group("n")) == self._marker:
                    self._event.set()
        return line

    def interrupt(self):
        """Wake a waiting caller at once, for an abort or a disconnect.

        run() clears this flag when it starts, so a caller that must not miss an abort
        between two batches also checks its own abort flag before each batch.
        """
        self._interrupted = True
        self._event.set()

    # -- primitives -------------------------------------------------------

    def run(self, commands, timeout=60.0):
        """Send commands, wait for them to finish, and return every reply line."""
        if not self._printer.is_operational():
            raise RuntimeError("printer is not connected")
        with self._lock:
            with self._receive_lock:
                self._lines = []
                self._interrupted = False
                self._event.clear()
                self._marker += 1
                marker = self._marker
                self._collecting = True
            try:
                payload = list(commands) + ["M400", "M114", "M118 %s%d" % (SYNC, marker)]
                self._printer.commands(payload, tags={TAG})
                if not self._event.wait(timeout):
                    raise Timeout("the printer did not reach marker %d in %.0f s" % (marker, timeout))
                if self._interrupted:
                    raise RuntimeError("interrupted while waiting for the printer")
                with self._receive_lock:
                    return list(self._lines)
            finally:
                with self._receive_lock:
                    self._collecting = False

    def send(self, commands):
        """Send commands and return at once."""
        if not self._printer.is_operational():
            raise RuntimeError("printer is not connected")
        self._printer.commands(list(commands), tags={TAG})
