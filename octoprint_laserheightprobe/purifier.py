# coding=utf-8
"""The air purifier for laser work: how much the laser burned, when the purifier runs, how long after.

Pure Python, no OctoPrint: the plugin feeds it the G-code it sends and the air readings from
Home Assistant, and asks it every few seconds what the purifier should be doing.

What the sensor showed before any of this existed (Printmonitor2, a PMS5003ST inside the
enclosure, 30 s readings, purifier off, 19-21 Nov 2026):

- it sees a burn within one reading. Even 3 s of 10 W line on paper (the B15 crosses, ~10 J)
  took PM2.5 from 3 to 53 ug/m3;
- with the purifier off the excess halves in about 5-7 min, so a heavy fill (600+ ug/m3,
  formaldehyde 130, 30 min of 70 % fills) takes about an hour to clear by itself;
- paper marks are not worth the filter (Chris, 2026-11-21): the purifier is for real burns, and
  it only has to make the air bearable again (not choking, not a fire alarm), not clean;
- a basswood engrave and cut took it past 2600 (beyond the PMS5003's range) and formaldehyde to 282;
  gear 1 (4050 rpm) halved PM2.5 every ~4 min and formaldehyde every ~3 min.

So the rules are:

1. A burn session is laser energy with gaps under session_gap_s. Its dose is the optical energy
   commanded: time x S/255 x the module's watts. The pointer (S at or below pointer_s_max) and
   travel never count.
2. The purifier starts once the dose passes start_j, or straight away if the sensor sees the
   PM2.5 or formaldehyde excess over the pre-job baseline pass an alarm level (a short job that
   still makes a lot of smoke: wood, leather, plastic).
3. Its gear while burning follows the recent laser power, and never drops mid-job (each change
   is an M1011 between burning moves).
4. After the last burn it runs on until two fresh readings in a row are bearable (excess under
   pm25_margin and hcho_margin), at least runon_min_s, at most runon_max_s. The gear steps down
   with the smoke.
5. A sensor that goes quiet (Home Assistant restarting, the ESP offline) means a timer from the
   dose (fallback_runon_s), never "keep running". Every limit fails towards off.
6. Each run-on's decay time is measured (decay_tau), so the timer can use what this enclosure
   and this filter actually do.
"""
from __future__ import absolute_import, division

import math
import re

S_MAX = 255.0

# Optical watts by M1006 name: what decides how much smoke a second of burning makes.
WATTS = {"10W LASER": 10.0, "20W LASER": 20.0, "40W LASER": 40.0, "2W RED LASER": 2.0}

DEFAULTS = dict(
    start_j=300.0,          # 10 W at full power for 30 s, or about 100 s at 30 %
    session_gap_s=30.0,     # burns closer than this are one job
    runon_min_s=60.0,
    heavy_runon_min_s=180.0, # after a heavy burn: the char keeps smoking for 2-3 min (seen 2026-11-21)
    heavy_dose_j=1000.0,     # a heavy burn: this much energy, or smoke past runon_gear2
    runon_max_s=1200.0,     # 20 min: gear 1 takes ~17 min from a basswood cut to bearable
    pm25_margin=150.0,      # ug/m3 over the pre-job air that is bearable: not choking, no fire alarm (Chris)
    hcho_margin=30.0,
    alarm_pm25=300.0,       # an excess this high starts the purifier whatever the dose
    alarm_hcho=60.0,
    stale_s=120.0,          # a reading older than this is no reading (30 s updates; 62 s ages seen)
    watch_s=300.0,          # how long after a job without the purifier to keep watching for smoke
    gear3_w=8.0,            # recent laser power (W) for gear 3 / gear 2; under gear2_w: gear 1
    gear2_w=4.0,            # (gear 1 halves the smoke every ~4 min, measured 2026-11-21: usually enough)
    runon_gear3=1000.0,     # excess PM2.5 for gear 3 / gear 2 during the run-on
    runon_gear2=400.0,
)

_WORD = re.compile(r"([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))")
_AXES = ("X", "Y", "Z", "B")


def module_watts(toolhead):
    """The optical power of what M1006 reports, 0 for anything that is not a laser."""
    return WATTS.get(str(toolhead or "").strip().upper(), 0.0)


def _clean(line):
    line = re.sub(r"\([^)]*\)", "", str(line or ""))
    return line.split(";", 1)[0].strip().upper()


class BurnMeter(object):
    """Follows the G-code sent to the printer and adds up the laser energy it commands.

    The firmware fires only after M3/M4 (an inline S on a G1 with no M3 does nothing, seen
    2026-11-19) and M5 stops it. G0 is travel. Arcs are not used by LightBurn's GRBL output.
    The time of a move is its length over its feed: sent time is not run time (the firmware
    buffers), but over a job the sums agree.
    """

    def __init__(self, watts=10.0, pointer_s_max=10, rapid=3000.0):
        self.watts = float(watts)
        self.pointer_s_max = pointer_s_max
        self.feed_rate = float(rapid)
        self.absolute = True
        self.pos = dict((a, 0.0) for a in _AXES)
        self.known = True
        self.laser = False
        self.s = 0.0
        self.burn_s = 0.0
        self.energy_j = 0.0

    def feed(self, line):
        """Read one sent line. Returns the joules it commands (0 for anything else)."""
        text = _clean(line)
        if not text:
            return 0.0
        words = dict(_WORD.findall(text))
        if not words:
            return 0.0
        code = text[0] + str(int(float(words.get(text[0], "0")))) if text[0] in "GM" else ""
        if "S" in words and code in ("M3", "M4", "G1", "G0"):
            self.s = float(words["S"])
        if code in ("M3", "M4"):
            self.laser = True
            return 0.0
        if code == "M5":
            self.laser = False
            return 0.0
        if code == "G90":
            self.absolute = True
            return 0.0
        if code == "G91":
            self.absolute = False
            return 0.0
        if code == "G28":
            self.known = False
            return 0.0
        if code == "G92":
            for a in _AXES:
                if a in words:
                    self.pos[a] = float(words[a])
            return 0.0
        if code not in ("G0", "G1"):
            return 0.0
        if "F" in words and float(words["F"]) > 0:
            self.feed_rate = float(words["F"])
        target = dict(self.pos)
        for a in _AXES:
            if a in words:
                target[a] = (self.pos[a] + float(words[a])) if not self.absolute else float(words[a])
        was_known = self.known
        if self.absolute and all(a in words for a in ("X", "Y")):
            self.known = True
        moved = [target[a] - self.pos[a] for a in _AXES]
        self.pos = target
        if code == "G0" or not self.laser or self.s <= self.pointer_s_max or not was_known:
            return 0.0
        dist = math.sqrt(sum(d * d for d in moved[:3])) or abs(moved[3])
        if dist <= 0:
            return 0.0
        seconds = dist / self.feed_rate * 60.0
        joules = seconds * min(self.s, S_MAX) / S_MAX * self.watts
        self.burn_s += seconds
        self.energy_j += joules
        return joules


def fallback_runon_s(dose_j, config, peak_excess=None, tau_s=None):
    """How long to run on without a live sensor. With a decay time learnt from earlier run-ons
    and the peak this job reached, the time for that peak to fall to the bearable margin;
    otherwise a slow log of the dose (300 J ~3 min, 30 kJ ~8 min). Always within min..max."""
    lo, hi = config["runon_min_s"], config["runon_max_s"]
    if tau_s and peak_excess and peak_excess > config["pm25_margin"]:
        secs = tau_s * math.log(peak_excess / config["pm25_margin"])
    else:
        secs = 60.0 + 90.0 * math.log(1.0 + max(dose_j, 0.0) / 100.0)
    return max(lo, min(hi, secs))


def decay_tau(readings, baseline):
    """The e-folding time (s) of the excess over baseline in [(t, value)], by a least-squares
    line through log(excess). None unless there are 3 readings well above baseline and they
    fall."""
    pts = [(t, math.log(v - baseline)) for t, v in readings if v - baseline > 2.0]
    if len(pts) < 3:
        return None
    n = len(pts)
    mt = sum(t for t, _ in pts) / n
    my = sum(y for _, y in pts) / n
    var = sum((t - mt) ** 2 for t, _ in pts)
    if var <= 0:
        return None
    slope = sum((t - mt) * (y - my) for t, y in pts) / var
    if slope >= -1e-5:
        return None
    return -1.0 / slope


def _median(values):
    values = sorted(values)
    if not values:
        return None
    mid = len(values) // 2
    return values[mid] if len(values) % 2 else (values[mid - 1] + values[mid]) / 2.0


class Decision(object):
    def __init__(self, policy, now, power, gear, phase, reason):
        self.power, self.gear, self.phase, self.reason = power, gear, phase, reason
        self._s = policy.snapshot(now)

    def status(self):
        return dict(self._s, power=self.power, gear=self.gear, phase=self.phase, reason=self.reason)


class AirPolicy(object):
    """What the purifier should be doing, from the burns and the air. Time is passed in."""

    def __init__(self, config=None, tau_s=None):
        self.c = dict(DEFAULTS, **(config or {}))
        self.tau_s = tau_s                  # learnt decay time with the purifier running
        self.readings = []                  # (t, pm25, hcho), the last 20 min
        self.learned_tau = None             # set when a run-on ends with a measurable decay
        self.finished = None                # a one-line summary when a job's purifier work ends
        self._reset()
        self.stopped = None                 # (t, reason) after Stop, until a new job
        self.last_reason = "no burn yet"

    def _reset(self):
        self.session = None                 # dict while a job (or its run-on/watch) is on
        self.burns = []                     # (t, joules) of this session, the last minute

    # -- inputs ----------------------------------------------------------

    def add_burn(self, joules, now):
        if joules <= 0:
            return
        s = self.session
        if s is not None and s["phase"] == "stopped":
            if now - s["last_burn"] < self.c["session_gap_s"]:
                s["last_burn"] = now               # Stop holds until this job's burns end
                return
            self._end(now, s["stop_reason"])
            s = None
        if s is None:
            s = self._new_session(now)
        # a burn during the watch or the run-on carries on the same job (dose, gear, baseline)
        s["phase"] = "burning"
        self.stopped = None
        s["dose"] += joules
        s["last_burn"] = now
        self.burns.append((now, joules))
        cut = now - 60.0
        while self.burns and self.burns[0][0] < cut:
            self.burns.pop(0)

    def sensor(self, pm25, hcho, t):
        if pm25 is None:
            return
        if self.readings and t <= self.readings[-1][0]:
            return
        self.readings.append((t, float(pm25), None if hcho is None else float(hcho)))
        cut = t - 1200.0
        while self.readings and self.readings[0][0] < cut:
            self.readings.pop(0)
        if self.session is not None:
            s = self.session
            excess = float(pm25) - s["base_pm25"]
            s["peak"] = max(s["peak"], excess)
            if s["phase"] == "runon":
                s["decay"].append((t, float(pm25)))

    def stop(self, now, reason):
        self.stopped = (now, reason)
        if self.session is not None:
            self.session["phase"] = "stopped"
            self.session["stop_reason"] = reason

    # -- the decision ----------------------------------------------------

    def _new_session(self, now):
        before = [r for r in self.readings if now - 600.0 <= r[0] < now]
        base_pm25 = _median([r[1] for r in before])
        base_hcho = _median([r[2] for r in before if r[2] is not None])
        self.burns = []
        self.session = dict(start=now, last_burn=now, dose=0.0, phase="burning", on=False, gear=0,
                            base_pm25=5.0 if base_pm25 is None else base_pm25,
                            base_hcho=10.0 if base_hcho is None else base_hcho,
                            base_known=base_pm25 is not None, peak=0.0, runon_start=None,
                            clean_run=0, clean_last=None, decay=[], stop_reason=None)
        return self.session

    def _fresh(self, now):
        if self.readings and now - self.readings[-1][0] <= self.c["stale_s"]:
            return self.readings[-1]
        return None

    def _excess(self, reading):
        s = self.session
        pm = reading[1] - s["base_pm25"]
        hc = (reading[2] - s["base_hcho"]) if reading[2] is not None else 0.0
        return pm, hc

    def _power_w(self, now):
        s = self.session
        span = max(1.0, min(60.0, now - s["start"]))
        return sum(j for t, j in self.burns if t >= now - 60.0) / span

    def tick(self, now):
        s = self.session
        c = self.c
        if s is None:
            reason = self.stopped[1] if self.stopped else self.last_reason
            return Decision(self, now, False, 0, "idle", reason)
        if s["phase"] == "stopped":
            if now - s["last_burn"] >= c["session_gap_s"]:
                self._end(now, s["stop_reason"])
            return Decision(self, now, False, 0, "idle", s["stop_reason"] or "stopped")
        reading = self._fresh(now)
        pm, hc = self._excess(reading) if reading else (0.0, 0.0)
        smoky = reading is not None and (pm >= c["alarm_pm25"] or hc >= c["alarm_hcho"])

        if s["phase"] == "burning":
            if now - s["last_burn"] >= c["session_gap_s"]:
                if not s["on"] and s["dose"] >= c["start_j"]:
                    s["on"] = True                   # decided on the dose, however rarely ticked
                    s["why"] = "%.0f J burned (start at %.0f J)" % (s["dose"], c["start_j"])
                s["phase"] = "runon" if s["on"] else "watch"
                s["runon_start"] = s["last_burn"] + c["session_gap_s"]
                s["decay"] = [(r[0], r[1]) for r in self.readings if r[0] >= s["runon_start"] - 60]
            else:
                if not s["on"] and (s["dose"] >= c["start_j"] or smoky):
                    s["on"] = True
                    s["why"] = ("smoke over the alarm level (PM2.5 +%.0f, formaldehyde +%.0f)" % (pm, hc)
                                if smoky and s["dose"] < c["start_j"] else
                                "%.0f J burned (start at %.0f J)" % (s["dose"], c["start_j"]))
                if s["on"]:
                    w = self._power_w(now)
                    gear = 3 if w >= c["gear3_w"] else 2 if w >= c["gear2_w"] else 1
                    if reading is not None:
                        gear = max(gear, 3 if pm >= c["runon_gear3"] * 2 else 2 if pm >= c["runon_gear3"] else 1)
                    s["gear"] = max(s["gear"], gear)          # never drops mid-job
                    s["gear_max"] = max(s.get("gear_max", 0), s["gear"])
                    return Decision(self, now, True, s["gear"], "burning",
                                    "burning: %s, %.1f W lately" % (s["why"], w))
                return Decision(self, now, False, 0, "burning",
                                "burning, %.0f J so far (purifier from %.0f J)" % (s["dose"], c["start_j"]))

        if s["phase"] == "watch":
            if smoky:
                s["on"] = True
                s["phase"] = "runon"
                s["runon_start"] = now
                s["why"] = "smoke after the job (PM2.5 +%.0f, formaldehyde +%.0f)" % (pm, hc)
            elif now - s["runon_start"] >= c["watch_s"]:
                self._end(now, "short job (%.0f J): purifier not needed" % s["dose"])
                return Decision(self, now, False, 0, "idle", self.last_reason)
            else:
                return Decision(self, now, False, 0, "watch",
                                "job done (%.0f J), watching the air" % s["dose"])

        # run-on
        elapsed = now - s["runon_start"]
        if elapsed >= c["runon_max_s"]:
            self._end(now, "run-on limit (%d min) reached" % round(c["runon_max_s"] / 60.0))
            return Decision(self, now, False, 0, "idle", self.last_reason)
        if reading is None:
            secs = fallback_runon_s(s["dose"], c, s["peak"] or None, self.tau_s)
            if elapsed >= secs:
                self._end(now, "no air reading: stopped on the %d s timer" % secs)
                return Decision(self, now, False, 0, "idle", self.last_reason)
            gear = max(1, min(s["gear"] or 2, 2))
            return Decision(self, now, True, gear, "runon",
                            "run-on, no air reading: timer %d s of %d" % (elapsed, secs))
        # the last two readings since the run-on began must both be bearable
        since = [r for r in self.readings if r[0] >= s["runon_start"]][-2:]
        bearable = len(since) == 2 and all(
            e[0] <= c["pm25_margin"] and e[1] <= c["hcho_margin"] for e in map(self._excess, since))
        heavy = s["dose"] >= c["heavy_dose_j"] or s["peak"] >= c["runon_gear2"]
        least = max(c["runon_min_s"], c["heavy_runon_min_s"] if heavy else 0.0)
        if bearable and elapsed >= least:
            self._end(now, "air bearable again (PM2.5 +%.0f, formaldehyde +%.0f)" % (pm, hc))
            return Decision(self, now, False, 0, "idle", self.last_reason)
        gear = 3 if pm >= c["runon_gear3"] else 2 if pm >= c["runon_gear2"] else 1
        s["gear"] = gear
        s["gear_max"] = max(s.get("gear_max", 0), gear)
        return Decision(self, now, True, gear, "runon",
                        "run-on %d s (%s): PM2.5 +%.0f, formaldehyde +%.0f"
                        % (elapsed, s.get("why") or "after the job", pm, hc))

    def _end(self, now, reason):
        s = self.session
        if s is not None and s.get("decay") and s.get("on"):          # only a run with the fan on
            tau = decay_tau(s["decay"], s["base_pm25"])
            if tau:
                self.learned_tau = tau
        if s is not None:
            runon = (now - s["runon_start"]) if s.get("runon_start") and s.get("on") else 0
            self.finished = ("%.0f J, purifier %s, peak PM2.5 +%.0f over %.0f, run-on %d s: %s"
                             % (s["dose"], "on (gear up to %d)" % s.get("gear_max", s["gear"]) if s.get("on")
                                else "not needed", s["peak"], s["base_pm25"], runon, reason))
        self.last_reason = reason
        self.session = None
        self.burns = []

    def snapshot(self, now):
        s = self.session or {}
        r = self.readings[-1] if self.readings else None
        return dict(dose_j=s.get("dose", 0.0), baseline_pm25=s.get("base_pm25"),
                    baseline_hcho=s.get("base_hcho"), peak_pm25=s.get("peak"),
                    pm25=r[1] if r else None, hcho=r[2] if r else None,
                    air_age_s=(now - r[0]) if r else None,
                    runon_s=(now - s["runon_start"]) if s.get("runon_start") and s.get("phase") == "runon" else None,
                    tau_s=self.tau_s)
