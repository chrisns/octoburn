# coding=utf-8
"""A GRBL 1.1 face for LightBurn, in front of the Snapmaker. Pure, no OctoPrint.

LightBurn (any computer) connects to OctoPrint over TCP as a "GRBL, Ethernet/TCP" device. This
module speaks GRBL to it (GrblSession) and turns what it sends into Snapmaker lines through the
same Reader and emitters as the file translator (Bridge), so jog, Frame, Start, Pause and Stop
all work from LightBurn while OctoPrint keeps the serial port.

What LightBurn 2.1.04 sends (recorded, tests/data/lightburn/stream_capture_2_1_04.log):
- On connect: "G0", "$I", "$#"; then "?\\n" about once a second. The "\\n" after "?" is a blank
  line, and GRBL answers a blank line with "ok": LightBurn counts it against its 128-byte buffer,
  so without that ok it stalls after about two minutes.
- Jog: "$J=G91G21  X10 Y0 F800", "$J=G91G21  Z0 A10 F800" (rotate buttons), "$J=G90G21 X0 Y0
  F800" (Go to Origin). In rotary mode the Y arrows still send Y (mm around the object).
- Frame: G0 to a corner, then "G1 ... S0 F800" round the box or outline: never fires.
- Start: "G00 G17 G40 G21 G54", G90, M4, M8, G0, then G91 relative G1 lines with inline S; no
  comments. Pause: "!" (twice). Stop: 0x18 (soft reset), then "M5".

Safety (with Bridge):
- Nothing fires unless the context says the laser is armed (OctoPrint, with an expiry); framing
  and jogging (S0) only need a mode with its measurement.
- Bounded queues: at most `window` lines inside OctoPrint and `depth` in the bridge, and moves are
  split short (emitter seg_max), so a stop waits behind little.
- Stop (0x18) and a disconnect: the queue is dropped, then M410 (quickstop) and M5. The firmware
  has no emergency parser, so M410 still waits behind the few lines already sent.
- The firmware leaves the PWM at the last block's power when its queue runs dry (stepper.cpp:
  nothing turns it off). So when LightBurn goes quiet after a burn and the queued motion is nearly
  done, the bridge sends M5 (it waits for the planner, then turns the beam off), and re-arms with
  "M3 S0" before the next burn. Pause does the same at once. This covers a LightBurn or network
  stall, not an OctoPrint or serial one.
"""
from __future__ import absolute_import, division

import collections
import re
import threading
import time

import math

from .rotary_gcode import FlatEmitter, Reader, RotaryEmitter, TranslateError

BANNER = "Grbl 1.1h ['$' for help]"
TAG = "lhp-bridge"

# GRBL error codes LightBurn knows how to show.
ERR_UNSUPPORTED = 20          # unsupported or invalid g-code command
ERR_SETTING = 3               # "$" statement not recognised
ERR_LOCKED = 9                # g-code locked out (here: not armed / no mode)
ERR_TRAVEL = 15               # jog target exceeds machine travel
ERR_LINE = 14                 # line too long

# Snapmaker's crosshair command, sent by a LightBurn macro: handed to the plugin's pointer switch
# (offset read, head shifted, beam held off, timeout) rather than passed to the printer.
_CROSSHAIR = re.compile(r"^M0*2002\s+T0*3\s+P([01])\s*$", re.I)
# Marlin's "trigger camera": M240 (or P0) the 10W overview photo, M240 P1 the last close-up again
_CAMERA = re.compile(r"^M0*240(?:\s+P(\d+))?\s*$", re.I)

SETTINGS = [
    ("$0", "10"), ("$1", "25"), ("$2", "0"), ("$3", "0"), ("$4", "0"), ("$5", "0"), ("$6", "0"),
    ("$10", "1"), ("$11", "0.010"), ("$12", "0.002"), ("$13", "0"), ("$20", "0"), ("$21", "0"),
    ("$22", "0"), ("$23", "0"), ("$24", "25.000"), ("$25", "500.000"), ("$26", "250"),
    ("$27", "1.000"), ("$30", "255"), ("$31", "0"), ("$32", "1"),
    ("$100", "400.000"), ("$101", "400.000"), ("$102", "400.000"), ("$103", "888.890"),
    ("$110", "7200.000"), ("$111", "7200.000"), ("$112", "2400.000"), ("$113", "2700.000"),
    ("$120", "3000.000"), ("$121", "3000.000"), ("$122", "100.000"), ("$123", "3000.000"),
    ("$130", "320.000"), ("$131", "350.000"), ("$132", "330.000"), ("$133", "360.000"),
]

_JOG_WORD = re.compile(r"([XYZAF])\s*(-?\d*\.?\d+)", re.I)
_MOTION = re.compile(r"[XYZA]\s*-?\d")
# The job marker each LightBurn device sends at the start of a job (its Start GCode):
# "(lhp head=10W mode=rotary)". LightBurn drops ";" comments from Start GCode but sends "( )"
# ones, and only on Start (not Frame or jog).
_MARKER = re.compile(r"^\s*\(\s*lhp\b(.*)\)\s*$", re.I)
_MARKER_FIELD = re.compile(r"(\w+)\s*=\s*(.*?)\s*(?=\w+\s*=|$)")
MODES = ("rotary", "flat")


# LightBurn's rotary diameter (dia= in the marker) may differ from the measured one by this much:
# 0.5 mm is 0.6 % of a 78 mm flask, about 1.5 mm over a full wrap.
DIA_TOLERANCE = 0.5


def job_marker(line):
    """{"head": "2W IR", "mode": "rotary", "dia": 78.0} from "(lhp head=2W IR mode=rotary dia=78)"
    (any field may be missing; dia is None unless it is a number), or None for any other line."""
    m = _MARKER.match(line)
    if not m:
        return None
    fields = dict((k.lower(), v) for k, v in _MARKER_FIELD.findall(m.group(1).strip()))
    mode = fields.get("mode")
    try:
        dia = float(fields["dia"]) if fields.get("dia") else None
    except ValueError:
        dia = None
    return dict(head=fields.get("head") or None, mode=mode.lower() if mode else None, dia=dia)


def device_name(module, mode):
    """The LightBurn device that sends this module and mode's marker."""
    return "Snapmaker %s %s via OctoPrint" % (module, mode)


def same_head(a, b):
    return "".join(a.split()).lower() == "".join(b.split()).lower()


class BridgeError(Exception):
    def __init__(self, code, message):
        Exception.__init__(self, message)
        self.code = code
        self.message = message


class Reset(Exception):
    """The line was overtaken by a soft reset or a disconnect: drop it, reply nothing."""


class GrblSession(object):
    """GRBL protocol on one connection. receive() is called from the socket reader with raw
    bytes; real-time bytes act at once, lines go to a worker thread so a full queue (the backend
    blocks) never delays a "?" or a stop. write(text) sends to LightBurn."""

    MAX_LINE = 256

    def __init__(self, backend, write, logger=None):
        self.backend = backend
        self._write = write
        self.log = logger
        self.buf = b""
        self.lines = collections.deque()
        self.cond = threading.Condition()
        self.gen = 0
        self.alive = True
        self.worker = threading.Thread(target=self._work, name="lhp-grbl-lines")
        self.worker.daemon = True

    def write(self, text):
        try:
            self._write(text)
        except Exception:                                      # noqa: BLE001
            self.alive = False

    def start(self):
        self.write("\r\n" + BANNER + "\r\n")
        self.worker.start()

    def close(self):
        with self.cond:
            self.alive = False
            self.gen += 1
            self.lines.clear()
            self.cond.notify_all()
        self.backend.disconnect()

    # -- bytes in ------------------------------------------------------------

    def receive(self, data):
        for ch in bytearray(data):
            if ch == 0x3F:                                     # "?"
                self.write(self.backend.status_report() + "\r\n")
            elif ch == 0x21:                                   # "!" feed hold
                self.backend.hold()
            elif ch == 0x7E:                                   # "~" cycle start / resume
                self.backend.resume()
            elif ch == 0x18:                                   # soft reset (LightBurn's Stop)
                with self.cond:
                    self.gen += 1
                    self.lines.clear()
                    self.buf = b""
                    self.cond.notify_all()
                self.backend.reset()
                self.write("\r\n" + BANNER + "\r\n")
            elif ch == 0x85:                                   # jog cancel
                self.backend.jog_cancel()
            elif ch >= 0x80:
                pass                                           # overrides: not supported, ignored
            elif ch == 0x0D:
                pass
            elif ch == 0x0A:
                line, self.buf = self.buf, b""
                with self.cond:
                    self.lines.append((self.gen, line.decode("ascii", "replace")))
                    self.cond.notify_all()
            else:
                self.buf += bytes(bytearray([ch]))
                if len(self.buf) > self.MAX_LINE:
                    self.buf = b""
                    with self.cond:
                        self.lines.append((self.gen, "\x00too long"))
                        self.cond.notify_all()

    # -- lines ---------------------------------------------------------------

    def _work(self):
        while True:
            with self.cond:
                while self.alive and not self.lines:
                    self.cond.wait(0.5)
                if not self.alive:
                    return
                gen, line = self.lines.popleft()
                if gen != self.gen:
                    continue
            try:
                reply = self.handle(line, gen)
            except Reset:
                continue
            with self.cond:
                if gen != self.gen:
                    continue                                   # a reset overtook it: no reply
            self.write(reply + "\r\n")

    def current(self, gen):
        return gen == self.gen and self.alive

    def handle(self, line, gen=None):
        s = line.strip()
        if s == "\x00too long":
            return "error:%d" % ERR_LINE
        if not s:
            return "ok"
        u = s.upper()
        try:
            if u.startswith("$"):
                return self._dollar(s, u)
            self.backend.gcode(s, lambda: gen is None or self.current(gen))
            return "ok"
        except BridgeError as exception:
            if self.log:
                self.log.info("GRBL bridge refused %r: %s", s, exception.message)
            return "[MSG:%s]\r\nerror:%d" % (exception.message.replace("\r", " ").replace("\n", " "),
                                             exception.code)

    def _dollar(self, s, u):
        if u == "$$":
            return "\r\n".join("%s=%s" % kv for kv in SETTINGS) + "\r\nok"
        if u == "$I":
            return "[VER:1.1h.20190825:OctoPrint-LaserHeightProbe]\r\n[OPT:VZ,15,128]\r\nok"
        if u == "$#":
            return ("[G54:0.000,0.000,0.000,0.000]\r\n[G92:0.000,0.000,0.000,0.000]\r\n"
                    "[TLO:0.000]\r\nok")
        if u == "$G":
            return "[GC:G0 G54 G17 G21 G90 G94 M5 M9 T0 F0 S0]\r\nok"
        if u == "$X":
            return "ok"
        if u == "$H":
            self.backend.home()
            return "ok"
        if u.startswith("$J="):
            self.backend.jog(s[3:])
            return "ok"
        return "error:%d" % ERR_SETTING


class Bridge(object):
    """The backend: LightBurn lines -> Snapmaker lines -> sender, with a bounded queue.

    sender: send(lines, counted, gen=None) hands lines to the printer; a counted line (tagged
    with the generation, so a stop can drop it if it is still queued) comes back through
    on_sent once it has gone out (the queue's lines), an uncounted one does not (M5 from hold,
    stop or the starvation guard); urgent(lines) jumps the queue (M410); position() -> machine
    (x, y, z, b) or None.
    context: None until OctoPrint picks a mode; then an object with mode ("flat"/"rotary"),
    reader(), emitter(), laser_armed() -> (bool, reason), jog_limits, to_lightburn(x, y, z, b).
    """

    def __init__(self, sender, clock=time.monotonic, logger=None, window=4, depth=24,
                 starve_s=0.3, low_water_s=0.6, pointer_switch=None, camera_trigger=None):
        self.sender = sender
        self.pointer_switch = pointer_switch                   # fn(on): the crosshair macro; ValueError = refused
        self.camera_trigger = camera_trigger                   # fn(variant): M240 (0) / M240 P1 (1); ValueError = refused
        self.locked = None                                     # why the plugin holds the machine (a close-up)
        self.clock = clock
        self.log = logger
        self.window, self.depth = window, depth
        self.starve_s, self.low_water_s = starve_s, low_water_s
        self.cond = threading.Condition(threading.RLock())
        self.context = None
        self.reader = self.emitter = None
        self.queue = collections.deque()                       # (line, lb_pos or None, seconds, kind)
        self.inflight = collections.deque()                    # what went to the sender, in order
        self.paused = False
        self.gen = 0
        self.pos = (0.0, 0.0, 0.0, 0.0)                         # LightBurn X, Y, Z, A of the last sent move
        self.busy_until = 0.0
        self.last_input = 0.0
        self.jogging = False
        self.error = None
        self.job_armed = False
        self.job = None                                        # this job's marker: dict(head, mode)
        self.frame_session = None                              # the moves of the stream in progress
        self.last_frame = None                                 # the last stream that burned nothing
        self.pointer = None                                    # dict(kind, s_max | dx, dy, until)
        self.machine_z = None                                  # where the head is, from M114 at a stop
        self.burn_watch = None                                 # fn(mv, start, context): each burn move (Recent burns)

    # -- context -------------------------------------------------------------

    # -- the pointer: where the beam would land, without burning ----------------

    def set_pointer(self, pointer):
        """pointer: None (off), dict(kind="spot", s_max=S) for a laser whose beam is the pointer
        (burns are capped at s_max and need no arm), or dict(kind="cross", dx, dy) for a module
        with a crosshair (beam held off, crosshair on, head shifted by -dx, -dy so the cross
        lands where the beam would; dx, dy = crosshair - beam). until: a clock() deadline."""
        with self.cond:
            self._pointer_off_locked()
            if pointer is None or self.context is None:
                return
            self.pointer = dict(pointer)
            if pointer["kind"] == "cross":
                self.sender.send(["M2002 T3 P1"], False)
                if self.emitter is not None:
                    self.emitter.shift(-pointer["dx"], -pointer["dy"])

    def _pointer_off_locked(self):
        old, self.pointer = self.pointer, None
        if old and old["kind"] == "cross":
            self.sender.send(["M2002 T3 P0"], False)
            if self.emitter is not None:
                self.emitter.shift(old["dx"], old["dy"])

    def pointer_state(self):
        with self.cond:
            self._pointer_expired_locked()
            return dict(self.pointer) if self.pointer else None

    def _pointer_expired_locked(self):
        if self.pointer and self.pointer.get("until") is not None and self.clock() > self.pointer["until"]:
            self._pointer_off_locked()
        return self.pointer

    def set_context(self, context):
        """A new mode/measurement (or None to disarm). Anything moving is stopped first."""
        with self.cond:
            self._pointer_off_locked()
            if self.context is not None:
                if self.queue or self.inflight or self.paused or self.jogging:
                    self._stop_locked()                # something is still being sent: stop it
                else:
                    # nothing left to send: no quickstop. The job's last moves (LightBurn's finish
                    # move) may still be running past the time estimate, and M410 would abort them
                    # and lose the position (2026-11-21, a slope job's finish move). M5 queues
                    # behind them.
                    self._soft_stop_locked()
            self.context = context
            self.reader = context.reader() if context else None
            self.emitter = context.emitter() if context else None
            self.error = None
            self.gen += 1
            self.cond.notify_all()
        self._resync()

    def _need_context(self, line=""):
        if self.context is None:
            raise BridgeError(ERR_LOCKED, "pick rotary or flat in OctoPrint first (OctoBurn tab, "
                                          "Burn)")

    # -- LightBurn side ------------------------------------------------------

    def status_report(self):
        with self.cond:
            state = self.state()
            x, y, z, a = self.pos
        return "<%s|MPos:%.3f,%.3f,%.3f,%.3f|FS:0,0>" % (state, x, y, z, a)

    def state(self):
        if self.error:
            return "Alarm"
        if self.locked:
            return "Run"
        if self.paused:
            return "Hold:0"
        if self.queue or self.inflight or self.clock() < self.busy_until:
            return "Jog" if self.jogging else "Run"
        return "Idle"

    def lock_out(self, reason):
        """The plugin moves the head itself (a close-up): refuse LightBurn's motion until released."""
        with self.cond:
            self.locked = reason

    def lock_release(self):
        with self.cond:
            self.locked = None

    def _refuse_if_locked(self):
        if self.locked:
            raise BridgeError(ERR_LOCKED, "the OctoPrint plugin is %s: wait for it to finish" % self.locked)

    def gcode(self, line, still_current=lambda: True):
        bare = line.split(";", 1)[0].strip()
        camera = _CAMERA.match(bare)
        if self.locked and (camera or _MOTION.search(bare.upper())):
            self._refuse_if_locked()
        if camera:
            variant = int(camera.group(1) or 0)
            if variant not in (0, 1):
                raise BridgeError(ERR_UNSUPPORTED, "M240 P%d: the plugin's camera macros are M240 (overview) "
                                                   "and M240 P1 (close-up again)" % variant)
            self._camera_macro(variant)
            return
        cross = _CROSSHAIR.match(line.split(";", 1)[0].strip())
        if cross:                                              # outside the lock: the switch takes it
            if self.pointer_switch is None:
                raise BridgeError(ERR_UNSUPPORTED, "the crosshair is switched by the OctoPrint plugin, "
                                                   "which is not connected to this bridge")
            try:
                self.pointer_switch(cross.group(1) == "1")
            except ValueError as exception:
                raise BridgeError(ERR_LOCKED, str(exception))
            return
        with self.cond:
            self.last_input = self.clock()
            marker = job_marker(line)
            if marker is not None:
                self.job = marker
                why = self._head_mismatch() if self.context is not None and not self.error else None
                if why:
                    self._alarm(why)
                    raise BridgeError(ERR_LOCKED, why)
                return
            if line.strip().upper() == "M2":
                self.job = None                                # LightBurn's end of job
            if self.context is None:
                if _MOTION.search(line.split(";", 1)[0].upper()):
                    self._need_context()
                return                                         # no motion: harmless, accept
            if self.error:
                raise BridgeError(ERR_LOCKED, "alarm: %s. Press Stop in LightBurn to clear it" % self.error)
            gen = self.gen
            try:
                self._gcode_locked(line)
            except (BridgeError, TranslateError) as exception:
                message = exception.message if isinstance(exception, BridgeError) else str(exception)
                code = exception.code if isinstance(exception, BridgeError) else ERR_UNSUPPORTED
                self._alarm(message)
                raise BridgeError(code, message)
            self._wait_room(gen, still_current)

    FRAME_GAP_S = 2.5          # a pause this long between moves ends a stream (a frame or a job)

    def _track(self, mv, burns):
        """Group LightBurn's moves into streams: a stream that burned nothing (LightBurn's Frame, or a
        dry run) leaves its outline in last_frame, for focusing there before the job."""
        now = self.clock()
        s = self.frame_session
        if s is None or now - s["last"] > self.FRAME_GAP_S:
            self._close_stream()
            s = self.frame_session = dict(x0=mv["u"], y0=mv["v"], x1=mv["u"], y1=mv["v"], n=0, burn=False,
                                          last=now)
        s["x0"], s["x1"] = min(s["x0"], mv["u"]), max(s["x1"], mv["u"])
        s["y0"], s["y1"] = min(s["y0"], mv["v"]), max(s["y1"], mv["v"])
        s["n"] += 1
        s["burn"] = s["burn"] or burns
        s["last"] = now

    def _close_stream(self):
        s, self.frame_session = self.frame_session, None
        if s and s["n"] >= 3 and not s["burn"] and (s["x1"] - s["x0"]) * (s["y1"] - s["y0"]) > 1.0:
            self.last_frame = dict(bounds=[round(s["x0"], 2), round(s["y0"], 2), round(s["x1"], 2), round(s["y1"], 2)],
                                   time=time.time(), mode=getattr(self.context, "mode", None))

    def frame(self):
        """The last frame's outline (LightBurn mm) and when, or None. A stream still going is not one."""
        with self.cond:
            s = self.frame_session
            if s is not None and self.clock() - s["last"] > self.FRAME_GAP_S:
                self._close_stream()
            return self.last_frame

    def _gcode_locked(self, line):
        was_on = self.reader.laser_on
        start = (self.reader.u, self.reader.v)
        mv = self.reader.feed_line(line)
        if was_on and not self.reader.laser_on:
            self._enqueue(["M5"], None, 0.0, "off")             # LightBurn's own M5
            self.emitter.laser_off()
        pointer = self._pointer_expired_locked()
        armed = self.context.laser_armed()[0]
        if mv is not None:
            self._track(mv, bool(mv["fire"] and mv["power"] > 0 and armed
                                 and not getattr(self.context, "dry_run", False)))
        if mv is None:
            # LightBurn's Fire button: M3/M4 with S and no motion. Only the spot pointer answers
            # it, capped; anything else leaves the beam off.
            if (pointer and pointer["kind"] == "spot" and not armed and self.reader.laser_on
                    and self.reader.power > 0 and re.search(r"\bM0*[34]\b", line.upper())):
                self._enqueue(["M3 S%d" % int(min(self.reader.power, pointer["s_max"]))], None, 0.0, "pointer")
            return
        if mv["fire"] and pointer and not armed and not getattr(self.context, "dry_run", False):
            if pointer["kind"] == "spot":
                mv = dict(mv, power=min(mv["power"], pointer["s_max"]))
            else:
                mv = dict(mv, fire=False, power=0.0)           # the crosshair shows it instead
            lines = self._start_lines() + self.emitter.emit(mv)
            self.jogging = False
            self._enqueue_move(lines, mv)
            return
        if mv["fire"] and getattr(self.context, "dry_run", False):
            if self.context.mode == "rotary":
                self.emitter.check_burn(self.emitter.machine(mv)[0])
            mv = dict(mv, fire=False, power=0.0)               # same path and timing, beam off
        if mv["fire"]:
            if self.state() == "Idle":
                self.job_armed = False                         # a new job: check the arm again
            if not self.job_armed:
                ok, why = self.context.laser_armed()
                if ok:
                    why = self._head_mismatch(need=True)
                if why:
                    raise BridgeError(ERR_LOCKED, why)
                self.job_armed = True                          # held for the rest of this job
        lines = self._start_lines() + self.emitter.emit(mv)
        self._watch_burn(mv, start)
        self.jogging = False
        self._enqueue_move(lines, mv)

    def _head_mismatch(self, need=False):
        """Why this job may not run on the module and mode OctoPrint picked, or None. need: a
        burn, which must come from a device that says which laser and mode it is for."""
        module = getattr(self.context, "module", None)
        if module is None:
            return None
        mode = self.context.mode
        job = self.job or {}
        if need and (not job.get("head") or not job.get("mode")):
            return ("this job does not say which laser and mode it is for: send it from the LightBurn "
                    "device \"%s\"" % device_name(module, mode))
        if job.get("head") and not same_head(job["head"], module):
            return ("this job is for the %s but OctoPrint has the %s selected: fit the %s and pick it "
                    "in OctoPrint, or switch the LightBurn device" % (job["head"], module, job["head"]))
        if job.get("mode") and job["mode"] != mode:
            return ("this job is for %s but OctoPrint has %s picked: pick %s in OctoPrint, or use the "
                    "LightBurn device \"%s\"" % (job["mode"], mode, job["mode"], device_name(module, mode)))
        measured = getattr(self.context, "diameter", None)
        if mode == "rotary" and measured:
            if job.get("dia") is None:
                if need:
                    return ("this job does not say LightBurn's rotary diameter: add dia=%.1f to the Start "
                            "GCode of the LightBurn device \"%s\", and set its rotary diameter to %.1f"
                            % (measured, device_name(module, mode), measured))
            elif abs(job["dia"] - measured) > DIA_TOLERANCE:
                return ("LightBurn's rotary diameter is %g but the object measures %.1f mm: set the "
                        "rotary diameter in LightBurn and dia= in the Start GCode of \"%s\" to %.1f"
                        % (job["dia"], measured, device_name(module, mode), measured))
        return None

    def _start_lines(self):
        """The preamble (G53, G90, G21, M211 S1, M5) before the first motion after a connect, a
        mode change or a stop; the emitter then goes up before it goes across."""
        if self.emitter.prev is not None:
            return []
        self.emitter.start_z = self.machine_z
        return self.emitter.preamble()

    def _alarm(self, message):
        """A refusal in the middle of a stream: whatever follows would be drawn from a position
        LightBurn did not mean, so stop, laser off, and refuse everything until a reset."""
        self._stop_locked()
        self.error = message
        if self.log:
            self.log.warning("GRBL bridge alarm: %s", message)

    def jog(self, text):
        with self.cond:
            self._refuse_if_locked()
            self.last_input = self.clock()
            self._need_context()
            if self.error:
                raise BridgeError(ERR_LOCKED, "alarm: %s. Press Stop in LightBurn to clear it" % self.error)
            gen = self.gen
            body = text.upper()
            relative = "G91" in body
            words = dict((k.upper(), float(v)) for k, v in _JOG_WORD.findall(body.replace("G91", "")
                                                                             .replace("G90", "")
                                                                             .replace("G21", "")))
            if words.get("Z", 0.0) != 0.0:
                raise BridgeError(ERR_UNSUPPORTED, "Z is set by the probe measurement, not jogged")
            feed = words.get("F")
            if not feed:
                raise BridgeError(ERR_UNSUPPORTED, "a jog needs F")
            u0, v0 = self._target()
            mv = self.context.jog_move(u0, v0, words, relative, feed)
            try:
                lines = self._start_lines() + self.emitter.emit(mv)
            except TranslateError as exception:
                raise BridgeError(ERR_TRAVEL, str(exception))
            self.reader.u, self.reader.v = mv["u"], mv["v"]   # the next move starts here
            self.jogging = True
            self._enqueue_move(lines, mv)
            self._wait_room(gen, lambda: True)

    def _target(self):
        """The LightBurn (u, v) the next move starts from."""
        if self.reader is not None and self.reader.u is not None and self.reader.v is not None:
            return self.reader.u, self.reader.v
        return self.context.lb_uv(self.pos)

    def hold(self):
        with self.cond:
            if self.paused:
                return
            self.paused = True
            if self.emitter is not None and self.emitter.armed:
                self.sender.send(["M5"], False)                # beam off once the planner drains
                self.emitter.laser_off()
            self.cond.notify_all()

    def resume(self):
        with self.cond:
            self.paused = False
            self.cond.notify_all()

    def _stop_locked(self):
        """Drop everything not yet written to the printer (the glue's sending hook drops lines
        of an old generation that are still in OctoPrint's queue), quickstop, laser off."""
        self.gen += 1
        self.queue.clear()
        self.inflight.clear()
        self.paused = False
        self.jogging = False
        self.job_armed = False
        self.job = None
        self.busy_until = 0.0
        self.sender.urgent(["M410"])
        self.sender.send(["M5"], False)
        if self.reader is not None:
            self.reader.reset()
        if self.emitter is not None:
            self.emitter.laser_off()
            self.emitter.prev = None                           # next motion: preamble, then up
        self.cond.notify_all()

    def _soft_stop_locked(self):
        """Beam off behind whatever the printer is still doing; nothing aborted."""
        self.gen += 1
        self.job_armed = False
        self.job = None
        self.busy_until = 0.0
        self.sender.send(["M5"], False)
        if self.reader is not None:
            self.reader.reset()
        if self.emitter is not None:
            self.emitter.laser_off()
            self.emitter.prev = None
        self.cond.notify_all()

    def reset(self):
        """LightBurn's Stop (0x18), or a disconnect. Also clears an alarm and the pointer."""
        with self.cond:
            self._pointer_off_locked()
            self._stop_locked()
            self.error = None
        self._resync()

    def jog_cancel(self):
        with self.cond:
            if not self.jogging:
                return
            self._stop_locked()
        self._resync()

    def home(self):
        with self.cond:
            self._refuse_if_locked()
            self._need_context()
            self._enqueue(["G28 X Y Z"], None, 30.0, "home")
            if self.emitter is not None:
                self.emitter.prev = None
            if self.reader is not None:
                self.reader.u = self.reader.v = None

    def disconnect(self):
        self.reset()

    def _camera_macro(self, variant=0):
        """LightBurn's camera macro (M240): the plugin moves the head to the photo spot and asks
        the touchscreen for the 10W's photo. Outside the lock (it takes seconds); only on an idle
        bridge, since the plugin moves the head itself. Then the head position is read again."""
        if self.camera_trigger is None:
            raise BridgeError(ERR_UNSUPPORTED, "the head camera is run by the OctoPrint plugin, "
                                               "which is not connected to this bridge")
        with self.cond:
            if self.state() != "Idle":
                raise BridgeError(ERR_LOCKED, "the head photo needs an idle machine: wait for the "
                                              "job or jog to finish")
        try:
            self.camera_trigger(variant)
        except ValueError as exception:
            raise BridgeError(ERR_LOCKED, str(exception))
        finally:
            self.resync()

    def resync(self):
        """The head moved outside the bridge (a head photo): read where it is."""
        self._resync()

    def _resync(self):
        try:
            where = self.sender.position()
        except Exception:                                      # noqa: BLE001
            where = None
        with self.cond:
            self.machine_z = where[2] if where is not None else None
            if where is not None and self.context is not None:
                if self.pointer and self.pointer["kind"] == "cross":
                    # the head sits -dx, -dy off so the cross is where LightBurn put the beam
                    where = (where[0] + self.pointer["dx"], where[1] + self.pointer["dy"]) + tuple(where[2:])
                self.pos = self.context.to_lightburn(*where)
                if self.reader is not None:
                    self.reader.u, self.reader.v = self.context.lb_uv(self.pos)

    # -- queue ---------------------------------------------------------------

    def _enqueue_move(self, lines, mv):
        g1 = [l for l in lines if l.startswith("G1") or l.startswith("G0")]
        sec = ((mv.get("minutes") or 0.0) * 60.0) / max(1, len(g1))
        for line in lines:
            moving = line.startswith("G1") or line.startswith("G0")
            self.queue.append((line, (mv["u"], mv["v"]) if moving else None,
                               sec if moving else 0.0, "jog" if self.jogging else "move"))
        self.cond.notify_all()

    def _enqueue(self, lines, lb, sec, kind):
        for line in lines:
            self.queue.append((line, lb, sec, kind))
        self.cond.notify_all()

    def _wait_room(self, gen, still_current):
        while len(self.queue) > self.depth:
            if gen != self.gen or not still_current():
                raise Reset()
            if not self.pump():                                # the pump thread does this too
                self.cond.wait(0.2)
        if gen != self.gen:
            raise Reset()

    def pump(self):
        """Move lines from the queue to the sender while there is room. Call often (the glue
        runs it on a thread); returns how many lines went out."""
        sent = 0
        with self.cond:
            while self.queue and not self.paused and len(self.inflight) < self.window:
                line, lb, sec, kind = self.queue.popleft()
                self.inflight.append((line, lb, sec, kind))
                self.sender.send([line], True, self.gen)
                sent += 1
            self._starvation_guard()
            if sent:
                self.cond.notify_all()
        return sent

    def _starvation_guard(self):
        e = self.emitter
        if e is None or not e.armed or self.queue or self.paused:
            return
        now = self.clock()
        if now - self.last_input >= self.starve_s and self.busy_until - now <= self.low_water_s:
            self.sender.send(["M5"], False)
            e.laser_off()
            if self.log:
                self.log.info("GRBL bridge: LightBurn went quiet after a burn: M5 sent")

    def stale(self, gen):
        """A queued line of an older generation (before a stop): the glue drops it unsent."""
        return gen is not None and gen != self.gen

    def on_sent(self, line, gen=None):
        """The printer connection has written one of our lines (OctoPrint's sent hook)."""
        with self.cond:
            if not self.inflight or self.stale(gen):
                return
            l, lb, sec, kind = self.inflight.popleft()
            now = self.clock()
            self.busy_until = max(now, self.busy_until) + sec
            if lb is not None and self.context is not None:
                self.pos = self.context.lightburn_pos(lb)
            self.cond.notify_all()


    def _watch_burn(self, mv, start):
        """Tell the burn watcher about a move with the beam on (never the pointer's or a dry
        run's: those reach here with the beam off). A watcher failure never stops a burn."""
        if self.burn_watch is None or not mv.get("fire"):
            return
        try:
            self.burn_watch(mv, start, self.context)
        except Exception:                                      # noqa: BLE001
            if self.log:
                self.log.exception("GRBL bridge: burn watcher")


class _Context(object):
    """Shared by the two modes: the laser arm (with an expiry) and a description."""

    def __init__(self, laser_until=None, clock=time.monotonic, describe="", dry_run=False, module=None):
        self.module = module                       # the laser module name it was built for ("10W")
        self.laser_until = laser_until
        self.clock = clock
        self.describe = describe
        self.dry_run = dry_run                     # every burn goes out at S0: a laser-off rehearsal

    def laser_armed(self):
        if self.laser_until is None:
            return False, ("the laser is not armed: arm it in OctoPrint (LightBurn bridge) once the "
                           "laser module is in the bay and the work is in place")
        if self.clock() > self.laser_until:
            return False, "the laser arm has expired: arm it again in OctoPrint"
        return True, ""


class RotaryContext(_Context):
    """Rotary: LightBurn X along the object, A in degrees, Y jogs as mm round the surface."""

    mode = "rotary"

    def __init__(self, surface, y_origin, y_sign=1, b_sign=-1, seg_max=5.0, beam_dx=0.0, beam_dy=0.0,
                 b_shift=0.0, **kw):
        _Context.__init__(self, **kw)
        self.surface, self.y_origin, self.y_sign, self.b_sign = surface, y_origin, y_sign, b_sign
        self.beam_dx, self.beam_dy = float(beam_dx or 0.0), float(beam_dy or 0.0)
        self.b_shift = float(b_shift or 0.0)                 # object angle = firmware B + b_shift
        self.seg_max = seg_max
        self.diameter = 2 * surface.r
        self.deg_per_mm = 360.0 / (math.pi * self.diameter)
        lo = (surface.y_lo - y_origin) * y_sign
        hi = (surface.y_hi - y_origin) * y_sign
        self.u_range = (min(lo, hi), max(lo, hi))

    def reader(self):
        return Reader(("X", "A"), deg_per_mm=self.deg_per_mm)

    def emitter(self):
        return RotaryEmitter(self.surface, self.y_origin, self.y_sign, self.b_sign,
                             seg_max=self.seg_max, beam_dx=self.beam_dx, beam_dy=self.beam_dy,
                             b_shift=self.b_shift)

    def jog_move(self, u0, v0, words, relative, feed):
        u, v = u0, v0
        if relative:
            u += words.get("X", 0.0)
            v += words.get("A", 0.0) + words.get("Y", 0.0) * self.deg_per_mm
        else:
            u = words.get("X", u)
            if "A" in words:
                v = words["A"]
            elif "Y" in words:
                v = words["Y"] * self.deg_per_mm
        lo, hi = self.u_range
        if not lo - 0.5 <= u <= hi + 0.5:
            raise BridgeError(ERR_TRAVEL, "X%.1f is off the measured body (X%.1f to X%.1f)" % (u, lo, hi))
        du, dv = abs(u - u0), abs(v - v0)
        if du > 1e-9:
            minutes = du / feed
        elif "A" in words or not relative:
            minutes = dv / feed
        else:
            minutes = (dv / self.deg_per_mm) / feed
        return dict(g=1, fire=False, u=u, v=v, power=0.0, minutes=minutes)

    def lb_uv(self, pos):
        return pos[0], pos[3]

    def lightburn_pos(self, uv):
        return (uv[0], 0.0, 0.0, uv[1])

    def to_lightburn(self, x, y, z, b):
        return ((y - self.beam_dy - self.y_origin) / self.y_sign, 0.0, 0.0, (b + self.b_shift) / self.b_sign)


class FlatContext(_Context):
    """Flat: LightBurn X, Y on the bed (plus the beam offset), one focus Z."""

    mode = "flat"

    def __init__(self, focus_z, offset_x=0.0, offset_y=0.0, bounds=(0.0, 0.0, 320.0, 350.0),
                 seg_max=5.0, backlash_x=0.0, obstacles=None, **kw):
        _Context.__init__(self, **kw)
        self.focus_z, self.ox, self.oy, self.bounds = focus_z, offset_x, offset_y, bounds
        self.seg_max = seg_max
        self.backlash_x = backlash_x
        self.obstacles = list(obstacles or [])

    def reader(self):
        return Reader(("X", "Y"))

    def emitter(self):
        return FlatEmitter(self.focus_z, self.ox, self.oy, self.bounds, seg_max=self.seg_max,
                           backlash_x=self.backlash_x, obstacles=self.obstacles)

    def jog_move(self, u0, v0, words, relative, feed):
        if words.get("A", 0.0):
            raise BridgeError(ERR_UNSUPPORTED, "no rotary in flat mode")
        if relative:
            u, v = u0 + words.get("X", 0.0), v0 + words.get("Y", 0.0)
        else:
            u, v = words.get("X", u0), words.get("Y", v0)
        x0, y0, x1, y1 = self.bounds
        if not (x0 <= u <= x1 and y0 <= v <= y1):
            raise BridgeError(ERR_TRAVEL, "X%.1f Y%.1f is outside X%g..%g Y%g..%g" % (u, v, x0, x1, y0, y1))
        return dict(g=1, fire=False, u=u, v=v, power=0.0, minutes=math.hypot(u - u0, v - v0) / feed)

    def lb_uv(self, pos):
        return pos[0], pos[1]

    def lightburn_pos(self, uv):
        return (uv[0], uv[1], 0.0, 0.0)

    def to_lightburn(self, x, y, z, b):
        return (x - self.ox, y - self.oy, 0.0, 0.0)
