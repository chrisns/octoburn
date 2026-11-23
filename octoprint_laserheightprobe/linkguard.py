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

Pure logic with an injected clock and actions, so it can be replayed against real serial logs.
"""
from __future__ import absolute_import, division

import re
import time

_POWER_RE = re.compile(r"\bS(\d+(?:\.\d+)?)")


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

    # -- the hooks ---------------------------------------------------------------------------
    def sent(self, cmd):
        if self.waiting is None:
            self.waiting = (self.clock(), cmd)

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
        if not is_burn(cmd) or now - since <= self.ok_s:
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
