# coding=utf-8
"""The serial link guard: a burn line whose "ok" never comes.

Seen three times on 2026-11-23 during dense rotary burns: the Snapmaker stops answering one
line with "ok" but stays alive (its temperature reports keep arriving every 2 s, so OctoPrint
never times out). OctoPrint then waits for that "ok" for ever, nothing more is sent, and the
firmware finishes its queued moves and leaves the laser at the last line's power on one spot
(28 minutes, the first time).

The guard watches every line sent and every line received. When a burn line (G1 with S > 0)
has waited `ok_s` for its "ok" while the printer still talks, the "ok" was lost: it fakes it
(OctoPrint's fake_ack) and the job carries on. A second loss before any real "ok", or a printer
that has gone silent, stops the burn (drop the queue, quickstop, M5). If no real "ok" follows
within `power_s`, it cuts the power (when a Home Assistant switch is configured).

A slow queue is not a lost ok: the firmware holds back the "ok" while its planner is full, so with
long moves queued (a 148 deg turn at the 1500 deg/min B ceiling takes 6 s) an ok can be seconds late
and then arrive. Seen at 21:34 on 2026-11-23: the guard faked two oks that were only late, and stopped
the burn; all three arrived the moment it quickstopped. So the guard estimates when the queued motion
ends (each G0/G1's length over its feed, X/Y/Z in mm and B in degrees) and waits `ok_s` past that.

Pure logic with an injected clock and actions, so it can be replayed against real serial logs.
"""
from __future__ import absolute_import, division

import collections
import re
import time

PLANNER_MOVES = 32     # at most this many moves sit in the firmware's queue (Marlin's block buffer is 16;
                       # doubled for the serial buffer), so only the last 32 sent can still hold an ok back

_POWER_RE = re.compile(r"\bS(\d+(?:\.\d+)?)")
_WORD_RE = re.compile(r"([XYZBF])\s*(-?\d+(?:\.\d+)?)")


def is_burn(cmd):
    """A G1 that fires: S above 0 (the bridge writes power inline on each G1)."""
    if not cmd:
        return False
    c = cmd.strip().upper()
    if not (c.startswith("G1 ") or c.startswith("G01 ")):
        return False
    m = _POWER_RE.search(c)
    return m is not None and float(m.group(1)) > 0


class LinkGuard(object):
    def __init__(self, fake_ack, stop, power_off=None, log=None, clock=time.monotonic,
                 ok_s=2.0, power_s=3.0):
        self.fake_ack, self.stop, self.power_off = fake_ack, stop, power_off
        self.log = log or (lambda text: None)
        self.clock = clock
        self.ok_s, self.power_s = float(ok_s), float(power_s)
        self.waiting = None          # (since, cmd) of the line sent after the last ok
        self.alive_at = None         # the last line received that was not an ok
        self.faked = 0               # oks faked since the last real one
        self.stopped_at = None       # when the guard stopped a burn and is waiting for a real ok
        self.powered_off = False
        self.events = []             # what the guard did, for the status and the tests
        self.pos = {}                # last commanded X/Y/Z/B (absolute mode), for the motion estimate
        self.feed = None             # modal F (mm/min, or deg/min for a B-only move)
        self.relative = False
        self.busy_until = None       # when the motion queued so far should end
        self.recent = collections.deque(maxlen=PLANNER_MOVES)   # durations of the last moves sent

    def move_seconds(self, cmd):
        """Estimated duration of a G0/G1 (0 for anything else or when it cannot be told)."""
        c = (cmd or "").split(";", 1)[0].strip().upper()
        head = c.split(" ", 1)[0] if c else ""
        if head in ("G90",):
            self.relative = False
        elif head in ("G91",):
            self.relative = True
        if head == "G92":
            for axis, value in _WORD_RE.findall(c):
                if axis != "F":
                    self.pos[axis] = float(value)
            return 0.0
        if head not in ("G0", "G00", "G1", "G01"):
            return 0.0
        words = dict(_WORD_RE.findall(c[len(head):]))
        if "F" in words:
            self.feed = float(words["F"])
        d2 = 0.0
        for axis in "XYZB":
            if axis not in words:
                continue
            v = float(words[axis])
            if self.relative:
                d2 += v * v
                self.pos.pop(axis, None)
            else:
                if axis in self.pos:
                    d2 += (v - self.pos[axis]) ** 2
                self.pos[axis] = v
        if not self.feed or self.feed <= 0:
            return 0.0
        return 60.0 * d2 ** 0.5 / self.feed

    # -- the hooks ---------------------------------------------------------------------------
    def sent(self, cmd):
        now = self.clock()
        seconds = self.move_seconds(cmd)
        if seconds > 0:
            self.recent.append(seconds)
            # the queue cannot hold more than the last PLANNER_MOVES moves: a long run of short
            # moves must not push the deadline out (23:54 on 2026-11-23: a real lost ok waited 6 s)
            self.busy_until = min(max(now, self.busy_until or now) + seconds, now + sum(self.recent))
        if self.waiting is None:
            self.waiting = (now, cmd)

    def received(self, line):
        if line is None:
            return
        if line.strip().lower().startswith("ok"):
            self.waiting = None
            self.faked = 0
            self.stopped_at = None
        elif line.strip():
            self.alive_at = self.clock()

    # -- the check (called a few times a second) ---------------------------------------------
    def step(self):
        now = self.clock()
        if self.stopped_at is not None and not self.powered_off and now - self.stopped_at > self.power_s:
            self._act("power", "no real ok %.1f s after stopping the burn: cutting the power" % (now - self.stopped_at))
            if self.power_off is not None:
                try:
                    self.power_off()
                except Exception as exception:                     # noqa: BLE001
                    self.log("Link guard: cutting the power failed: %s" % exception)
            self.powered_off = True
            return
        if self.waiting is None:
            return
        since, cmd = self.waiting
        start = max(since, self.busy_until or since)      # the ok may wait for the queued motion
        if not is_burn(cmd) or now - start <= self.ok_s:
            return
        alive = self.alive_at is not None and self.alive_at > since
        if alive and self.faked == 0:
            self._act("fake_ack", "a burn line waited %.1f s for its ok while the printer kept talking: "
                                  "the ok was lost, faking it (%s)" % (now - since, cmd.strip()))
            self.faked += 1
            self.waiting = None
            self.fake_ack()
            return
        why = ("the printer has gone silent" if not alive else
               "another ok was lost before any real one")
        self._act("stop", "a burn line waited %.1f s for its ok and %s: stopping the burn, beam off (%s)"
                  % (now - since, why, cmd.strip()))
        self.waiting = None
        self.stopped_at = now
        self.stop()

    def _act(self, kind, text):
        self.events.append((self.clock(), kind))
        self.log("Link guard: " + text)
