# coding=utf-8
"""Runs the air purifier from the plugin: the purifier policy's decisions turned into Home
Assistant calls (the socket, the air readings) and M1011 commands (the fan).

Two levers, kept apart:

- the socket (a Home Assistant switch) powers the purifier's fan. Off, the firmware still sees
  the purifier over CAN but reports "extend power" lost and the fan cannot run;
- M1011 F<0-3> sets the fan's gear (firmware snapmaker/src/gcode/M1011.cpp). M1011 F<g> does a
  short status read in the firmware (up to 200 ms), so mid-job it goes in straight after a
  laser-off line (travel, M5, an S0 move), never between burning moves. M1011 F0 D<s> is NOT a
  delayed stop: it sets gear 0 at once (seen in the source and the log, 2026-11-21).

Everything fails towards off: no Home Assistant token means no socket and no readings (the fan
timer still works if the socket is on); a quiet sensor means the dose timer; an exception, a
disconnect, a restart or Stop turns the fan to F0 and the socket off.
"""
from __future__ import absolute_import

import json
import re
import ssl
import threading
import time

from . import purifier as pu

try:                                    # Python 3
    from urllib.request import Request, urlopen
except ImportError:                     # pragma: no cover
    from urllib2 import Request, urlopen

def laser_off_line(line):
    """True for a line after which a slow command cannot leave a burning move waiting:
    travel (G0), M5, or a move at S0."""
    text = (line or "").split(";", 1)[0].strip().upper()
    if text.startswith("G0") and not text[2:3].isdigit() or text.startswith("M5") and not text[2:3].isdigit():
        return True
    return bool(re.match(r"^G1\b", text)) and bool(re.search(r"\bS0+(\.0*)?(?![\d.])", text))


class HomeAssistant(object):
    """The few REST calls the purifier needs. Raises on any failure; the caller decides."""

    def __init__(self, url, token, timeout=3.0):
        self.url = (url or "").rstrip("/")
        self.token = token or ""
        self.timeout = timeout

    @property
    def ready(self):
        return bool(self.url and self.token)

    def _call(self, method, path, body=None):
        data = json.dumps(body).encode("utf-8") if body is not None else None
        req = Request(self.url + path, data=data, method=method,
                      headers={"Authorization": "Bearer " + self.token,
                               "Content-Type": "application/json"})
        resp = urlopen(req, timeout=self.timeout, context=ssl.create_default_context())
        raw = resp.read()
        return json.loads(raw.decode("utf-8")) if raw else None

    def switch(self, entity, on):
        self._call("POST", "/api/services/switch/turn_%s" % ("on" if on else "off"), dict(entity_id=entity))

    def state(self, entity):
        """(value, time) of a sensor, or (None, None) when it is unavailable. The time is when it
        last reported (HA's last_reported), so an unchanged reading is still fresh."""
        st = self._call("GET", "/api/states/%s" % entity)
        try:
            value = float(st["state"])
        except (TypeError, ValueError, KeyError):
            return None, None
        when = st.get("last_reported") or st.get("last_updated")
        return value, _parse_time(when)


def _parse_time(text):
    import calendar
    if not text:
        return None
    m = re.match(r"(\d{4})-(\d\d)-(\d\d)T(\d\d):(\d\d):(\d\d)(\.\d+)?", text)
    if not m:
        return None
    parts = [int(g) for g in m.groups()[:6]]
    return calendar.timegm(parts) + float(m.group(7) or 0)


class AirController(object):
    """Glue between the plugin and the purifier policy. The plugin calls on_sent() from its sent
    hook, queuing() from its queuing hook, and status(); a thread calls step() every few
    seconds."""

    POLL_ACTIVE_S = 15.0
    POLL_IDLE_S = 60.0
    PENDING_MAX_S = 15.0        # a gear change waiting for a laser-off line goes anyway after this
    STREAM_S = 3.0              # lines sent this recently mean a job is streaming

    def __init__(self, config, send, watts, logger=None, log_line=None, clock=time.time, ha=None):
        """config: dict from settings.purifier_config. send(lines): to the printer.
        watts(): the fitted laser's optical watts (0: not a laser)."""
        self.config = config
        self.send = send
        self.watts = watts
        self.logger = logger
        self.log_line = log_line or (lambda text: None)
        self.clock = clock
        self.ha = ha if ha is not None else HomeAssistant(config.get("ha_url"), config.get("ha_token"))
        self.lock = threading.RLock()
        self.policy = pu.AirPolicy(config.get("policy"), tau_s=config.get("tau_s"))
        self.meter = None
        self.socket = None          # True/False once set by us, None unknown
        self.gear = None            # last gear sent
        self.pending = None         # ([lines], since)
        self.enc_fan = 0            # the enclosure fan % we last set (0: ours is off)
        self.last_sent = 0.0
        self.last_poll = -1e9
        self.decision = None
        self.error = None
        self.ha_failed = 0.0        # when a socket call last failed (0: fine)
        self.stats = dict(run_s=0.0, by_gear={}, since=None)   # fan time, kept by the plugin
        self.filter = None          # the purifier's last M1011 report (lifetime etc.)
        self._ran_at = None         # when the fan time was last added

    # -- hooks -----------------------------------------------------------

    def on_sent(self, line):
        if not self.config.get("enabled"):
            return
        watts = self.watts()
        if not watts:
            self.meter = None
            return
        with self.lock:
            now = self.clock()
            self.last_sent = now
            if self.meter is None or self.meter.watts != watts:
                self.meter = pu.BurnMeter(watts=watts, pointer_s_max=self.config.get("pointer_s_max", 10))
            joules = self.meter.feed(line)
            if joules:
                self.policy.add_burn(joules, now)

    def queuing(self, line):
        """Lines to queue instead of `line`, or None. Puts a waiting M1011 straight after a
        laser-off line."""
        with self.lock:
            if self.pending is None or not laser_off_line(line):
                return None
            extra = list(self.pending[0])
            self.pending = None
        return [line] + extra

    # -- the loop --------------------------------------------------------
    #
    # Decisions are made under the lock (on_sent and queuing take it from OctoPrint's send path);
    # Home Assistant calls are made outside it, so a hung Home Assistant can never stall the
    # serial line. A failed socket call backs off instead of retrying every pass.

    HA_BACKOFF_S = 30.0

    def step(self):
        """One pass: read the air when due, decide, act. Never raises."""
        try:
            self._poll()
            with self.lock:
                now = self.clock()
                self._count_run(now)
                d = self.policy.tick(now)
                self.decision = d
                lines, socket = self._plan(d, now)
                learned, self.policy.learned_tau = self.policy.learned_tau, None
                summary, self.policy.finished = self.policy.finished, None
            if lines:
                self.send(lines)
            if socket is not None:
                self._switch(socket, d.reason)
            if summary:
                self.log_line("Air purifier, job done: %s." % summary)
            if learned:
                self.log_line("Air purifier: the smoke fell with a %.0f s decay time this run." % learned)
                if self.config.get("on_learned"):
                    self.config["on_learned"](learned)
            if not self.ha_failed:
                self.error = None
        except Exception as exception:          # anything at all: fail to off
            self.error = str(exception)
            if self.logger:
                self.logger.exception("air purifier step failed")
            self.off("an error (%s)" % exception)

    def _poll(self):
        if not self.config.get("enabled") or not self.ha.ready:
            return
        now = self.clock()
        active = self.policy.session is not None
        if now - self.last_poll < (self.POLL_ACTIVE_S if active else self.POLL_IDLE_S):
            return
        self.last_poll = now
        try:
            pm25, t = self.ha.state(self.config["pm25"])
            hcho = None
            if self.config.get("hcho"):
                hcho, _ = self.ha.state(self.config["hcho"])
        except Exception as exception:
            self.error = "air reading: %s" % exception
            return
        if pm25 is not None:
            with self.lock:
                self.policy.sensor(pm25, hcho, min(t or now, now))

    # The enclosure's exhaust fan blows into the purifier's hose (Chris, 2026-11-21): it pushes
    # while the purifier pulls, costs no filter, and its noise does not matter, so it runs flat
    # out whenever the purifier does. M1010 S4 P<0-100> (firmware M1010.cpp, a CAN command, no wait).
    ENCLOSURE_PCT = {1: 100, 2: 100, 3: 100}

    def _gear_lines(self, gear):
        lines = ["M1011 F%d" % gear]
        pct = self.ENCLOSURE_PCT.get(gear, 0) if self.config.get("enclosure_fan") else 0
        if pct != self.enc_fan and (pct or self.enc_fan):
            lines.append("M1010 S4 P%d" % pct)
            self.enc_fan = pct
        return lines

    def _off_lines(self):
        lines = ["M1011 F0"]
        if self.enc_fan:                        # only a fan we started: never one set by hand
            lines.append("M1010 S4 P0")
            self.enc_fan = 0
        return lines

    def _plan(self, d, now):
        """Under the lock: the printer lines to send now and the socket state wanted (or None)."""
        lines, socket = [], None
        if not self.config.get("enabled"):
            if self.gear or self.enc_fan:
                lines += self._off_lines()
                self.gear = 0
            self.pending = None
            return lines, (False if self.socket else None)
        if d.power:
            if self.socket is not True:
                socket = True
            if d.gear != self.gear:
                change = self._gear_lines(d.gear)
                self.gear = d.gear
                if now - self.last_sent < self.STREAM_S:
                    self.pending = (change, now)  # a job is streaming: after the next laser-off line
                else:
                    self.pending = None
                    lines += change
        else:
            self.pending = None
            if self.gear or self.enc_fan:
                lines += self._off_lines()
            self.gear = 0
            if self.socket:                     # only a socket we switched on (startup turns it off)
                socket = False
        if self.pending and now - self.pending[1] >= self.PENDING_MAX_S:
            lines += self.pending[0]
            self.pending = None
        return lines, socket

    def _switch(self, on, reason):
        """The socket, outside the lock, with a back-off after a failure."""
        if not self.ha.ready:
            return
        now = self.clock()
        if self.ha_failed and now - self.ha_failed < self.HA_BACKOFF_S:
            return
        try:
            self.ha.switch(self.config["switch"], on)
        except Exception as exception:
            self.ha_failed = now
            self.error = "socket %s failed: %s" % ("on" if on else "off", exception)
            return
        self.ha_failed = 0.0
        with self.lock:
            self.socket = on
        self.log_line("Air purifier: %s (%s)." % ("on" if on else "off", reason))

    def off(self, reason, stop_policy=False):
        """Fan to F0 and the socket off, whatever we think the state is."""
        with self.lock:
            if stop_policy:
                self.policy.stop(self.clock(), reason)
            self.decision = None                    # the status shows the stop at once
            self.pending = None
            self.gear = 0
            lines = self._off_lines()
        try:
            self.send(lines)
        except Exception:
            pass
        if self.ha.ready:
            self.ha_failed = 0.0                    # an explicit off always tries
            self._switch(False, reason)
        else:
            self.log_line("Air purifier: off (%s)." % reason)

    def _count_run(self, now):
        """Add the fan's running time since the last pass (gear set and the socket ours or unknown)."""
        if self._ran_at is not None and self.gear and self.socket is not False:
            dt = max(0.0, min(now - self._ran_at, 60.0))
            self.stats["run_s"] = self.stats.get("run_s", 0.0) + dt
            g = str(self.gear)
            self.stats.setdefault("by_gear", {})[g] = self.stats["by_gear"].get(g, 0.0) + dt
            if not self.stats.get("since"):
                self.stats["since"] = now
        self._ran_at = now

    def status(self):
        with self.lock:
            now = self.clock()
            d = self.decision or self.policy.tick(now)
            s = d.status()
            s.update(enabled=bool(self.config.get("enabled")), ha=self.ha.ready, socket=self.socket,
                     fan_gear=self.gear, enclosure_fan=self.enc_fan,
                     waiting=" ".join(self.pending[0]) if self.pending else None,
                     error=self.error, run=dict(self.stats), filter=self.filter)
            return s
