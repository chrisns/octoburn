# coding=utf-8
"""Translate LightBurn rotary G-code into Snapmaker 2.0 B-axis G-code. Pure, no OctoPrint.

LightBurn's rotary (chuck, A axis, 360 per rotation) writes X along the object and A in degrees
around it: A = workspace Y * 360 / circumference (seen in real LightBurn 2.1.04 output from the
"Snapmaker 10W Rotary (translate first)" GRBL-M3 profile, 2026-11-17). On the Snapmaker the
rotary axis runs along machine Y and turns B, in degrees. So:

    machine Y = y_origin + y_sign * LightBurn X
    B         = b_sign * LightBurn A
    machine X = the measured axis X at that Y (the beam stays over the top of the object)
    Z         = focus Z at that Y and B: the measured top (axis line + radius) plus the runout
                at that B, plus the focus offset (focus Z - probe trigger Z)

What LightBurn writes (real exports in tests/data/lightburn):
- Line mode: absolute G0/G1 X..A..; circles come as G1 segments (no arcs in rotary mode).
- Fill (and image) mode: an absolute G0 to the start, then G91 relative G1 moves, with S0
  overscan and line-step moves, then G90.
- Feed is GRBL's convention: F applies to the X distance alone, and to the degrees when a move
  has no X (a pure 20 mm rotation at 1200 mm/min is written F1762.95 = 1200 * 360 / (pi * 78);
  a move of X 10 with 17.32 mm of surface is F600). So a move takes |dX| / F minutes, or
  |dA| / F with no X. But LightBurn works F out from X steps a little different from the 3-decimal
  ones it prints, so on a move that is nearly all rotation (a circle's top) that convention is up
  to 15% out. LightBurn also writes each layer's surface speed as a comment ("; Cut @ 1200
  mm/min"), so a burn takes its surface length / that speed, and the per-move F is only the
  fallback (and is used for S0 and travel moves). The Snapmaker firmware times a move by its
  full length with degrees counted as mm, so each move is re-written F = that length / the time.

Safety, by construction:
- LightBurn's own start and end G-code are dropped, and so is a guard line (M112) that the
  LightBurn device profile puts first, so the raw file halts the machine if it is ever sent
  untranslated.
- The beam fires only on G1 at power above 0. A G1 at S0 stays a G1 with S0 (beam off, at its
  own speed, so fill overscan still gets the head up to speed); a G0 is a G0 (the firmware zeroes
  the inline power on every G0). Power is inline on each G1 (S 0-255) after one "M3 S0": M3
  calls planner.synchronize(), so an M3 per power change would dwell the beam. Ends with M5.
- G53 first: machine coordinates, so a G92 left by LightBurn's start code cannot shift the job.
- Anything not understood is refused: G2/G3, Z words, the wrong rotary letter, S above 255,
  a relative move before any absolute position.
- Every burn is checked against the measured body (Y range) before anything is written.
- The firmware caps B at 45 deg/s (DEFAULT_MAX_FEEDRATE) and silently slows a faster move with
  the power unchanged, which over-burns, so a burn that would turn B faster than max_b_feed
  (Luban's 2400 deg/min) is refused with the LightBurn speed that would fit.

One core for files and for the live GRBL bridge: a Reader turns LightBurn lines into moves (in
LightBurn's coordinates, with a time estimate) and an emitter (RotaryEmitter, FlatEmitter) turns
moves into Snapmaker lines. translate() reads a whole file, checks it, then emits it; the bridge
feeds the same Reader and emitter a line at a time and checks each move as it goes. A streamed
job carries no comments (LightBurn strips them), so a burn's speed comes from GRBL's convention,
with the last well-conditioned speed carried over a move that is nearly all rotation.
"""
from __future__ import absolute_import, division

import math
import re

GUARD = "M112 ; RAW LIGHTBURN ROTARY FILE"
# LightBurn X runs from the free end towards the chuck (machine +Y). B_SIGN makes the design read
# the right way round (not mirrored). Probed 2026-11-17 at Y60 (0.68 mm eccentric): the axis X
# was 164.066 at B0 and 165.409 at B180, lowest at B90 and highest at B270, so +B turns the top
# of the object towards -X. With LightBurn's canvas (X right, Y up) laid X -> +Y, canvas +Y has
# to go to -X on the top surface for a right-handed (unmirrored) result: B = -A.
Y_SIGN = 1
B_SIGN = -1
MAX_B_FEED = 2400.0              # deg/min (Luban's MAX_B_SPEED; the firmware's cap is 45 deg/s)
TRAVEL_FEED = 3000.0
# Where the head goes before it crosses when nobody knows where it is (a file run): above the
# rotary chuck with room to spare (the flask top is about Z143 in the probe's frame).
SAFE_HIGH = 200.0
FLAT_HOP_MIN = 30.0      # mm: a flat travel this long or longer goes up first (Z hop)
FLAT_HOP = 80.0          # mm above focus for it: clears a tape roll or a lid beside the work
Z_MAX = 330.0
MODULE_REACH = (74.0, 37.0, 40.0, 37.0)   # the 10W's outline round the beam: -X, -Y, +X, +Y (its CAD)
OBSTACLE_MARGIN = 5.0
_WORD = re.compile(r"([A-Za-z])\s*(-?\d*\.?\d+)")
_PAREN = re.compile(r"\([^)]*\)")                  # "( )" comments
# LightBurn names each layer's surface speed in a comment: "; Cut @ 1200 mm/min, 20% power"
_LAYER_SPEED = re.compile(r";\s*(?:Cut|Scan|Image|Fill|Line)\b.*?@\s*([\d.]+)\s*mm/(min|s)\b", re.I)
# A mixed X+A move whose X share of the surface length is below this is "nearly all rotation":
# LightBurn's F for it is ill-conditioned (see the module notes).
CONDITIONED = 0.5


# The turning sense of B (probed 2026-11-17): +B turns the top of the object towards -X, so an
# eccentric centre at angle a (from +X towards +Z) moves to a + B. With the runout fit's
# z = mean + e cos(B - phase), the centre's X then moves by -e (sin(B - phase) + sin(phase))
# from where it was at B 0 (checked: Y60 e 0.70, phase 270 gives +1.40 mm from B0 to B180; the
# chord probes measured +1.34).
ROT_SIGN = 1


class TranslateError(ValueError):
    pass


def words(line):
    code = line.split(";", 1)[0].strip()
    return [(w.upper(), float(v)) for w, v in _WORD.findall(code)]


class Surface(object):
    """Top-of-object model from a rotary measurement (rotary_job result)."""

    def __init__(self, result, focus_offset):
        a = result["axis"]
        # The measured diameter (housing reference + runout) wins over the one typed in.
        self.r = ((result.get("summary") or {}).get("diameter") or result["params"]["diameter"]) / 2.0
        self.y0, self.x0, self.z0 = a["y0"], a["x0"], a["z0"]
        self.dx, self.dz = a["dx_dy"], a["dz_dy"]
        s = result["summary"]
        self.y_lo, self.y_hi = s["body_from_y"], s["body_to_y"]
        self.focus_offset = None if focus_offset is None else float(focus_offset)
        # Runout: the stations were probed at B 0, so the axis line is the top at B 0. The top
        # at B is that plus e (cos(B - phase) - cos(phase)), interpolated in Y between stations.
        self.runout = sorted((r["y"], r.get("e", 0.0), r.get("phase_deg", 0.0))
                             for r in result.get("runout", []) if "e" in r)

    def axis_x(self, y, b=0.0):
        """X of the object's centre at Y and B: the axis line (probed at B 0) plus the eccentric
        centre's swing as B turns, so the beam stays on the true top of the section."""
        return self.x0 + self.dx * (y - self.y0) + self._runout(y, b, which="x")

    def _runout(self, y, b, which="z"):
        if not self.runout:
            return 0.0
        pts = self.runout
        if len(pts) == 1 or y <= pts[0][0]:
            ys = [pts[0]]
        elif y >= pts[-1][0]:
            ys = [pts[-1]]
        else:
            i = max(k for k in range(len(pts)) if pts[k][0] <= y)
            ys = [pts[i], pts[i + 1]]
        vals = []
        for py, e, ph in ys:
            if which == "z":
                v = e * (math.cos(math.radians(b - ph)) - math.cos(math.radians(-ph)))
            else:
                v = -ROT_SIGN * e * (math.sin(math.radians(b - ph)) + math.sin(math.radians(ph)))
            vals.append((py, v))
        if len(vals) == 1:
            return vals[0][1]
        (y1, v1), (y2, v2) = vals
        t = (y - y1) / (y2 - y1)
        return v1 + t * (v2 - v1)

    def top(self, y, b=0.0):
        return self.z0 + self.dz * (y - self.y0) + self.r + self._runout(y, b)

    def focus(self, y, b=0.0):
        return self.top(y, b) + self.focus_offset

    def highest_focus(self):
        """The highest focus Z anywhere on the measured body, any B (for travel height)."""
        best = -float("inf")
        n = 24
        for i in range(n + 1):
            y = self.y_lo + (self.y_hi - self.y_lo) * i / float(n)
            for b in range(0, 360, 15):
                best = max(best, self.focus(y, b))
        return best


def check_offset(focus_offset, allow_negative=False):
    if focus_offset is None:
        raise TranslateError("the focus offset is not set: calibrate it before a rotary job")
    if focus_offset < 0 and not allow_negative:
        # The 10W focuses 29.75 mm below the plate's bottom edge and the probe tip is 24.0 below
        # it, so its offset is about +5.75 (the head goes UP from the trigger for focus). A
        # negative offset means a sign slip that drives the hood into the work.
        raise TranslateError("the focus offset %+.3f is negative: the head would go below the probe "
                             "trigger. Check the sign (the 10W is about +5.75)" % focus_offset)


class Reader(object):
    """LightBurn lines -> moves, in LightBurn's coordinates. Stateful: feed it a whole file or a
    line at a time. axes is ("X", "A") for rotary (A in degrees) or ("X", "Y") for flat.

    A move is dict(g, fire, u, v, power, minutes): u, v the absolute LightBurn position on the
    two axes, minutes how long LightBurn means the move to take (None for G0 and for a G1 whose
    time cannot be told, e.g. the first move), fire whether the beam is on.
    """

    def __init__(self, axes=("X", "A"), deg_per_mm=None, s_max=255.0):
        self.axes = axes
        self.rotary = axes[1] == "A"
        self.deg_per_mm = deg_per_mm
        self.s_max = s_max
        self.u = self.v = None
        self.power, self.laser_on, self.feed, self.g, self.relative = 0.0, False, None, None, False
        self.layer_speed = None
        self.v_last = None                       # last well-conditioned burn speed (mm/min)
        self.n = 0

    def reset(self):
        """A GRBL soft reset: modal state back to defaults, position kept."""
        self.power, self.laser_on, self.feed, self.g, self.relative = 0.0, False, None, None, False
        self.v_last = None

    def _surface(self, du, dv):
        return math.hypot(du, dv / self.deg_per_mm) if self.rotary else math.hypot(du, dv)

    def feed_line(self, raw):
        """One LightBurn line -> a move dict or None. Raises TranslateError."""
        self.n += 1
        n = self.n
        m = _LAYER_SPEED.search(raw)
        if m:
            self.layer_speed = float(m.group(1)) * (60.0 if m.group(2).lower() == "s" else 1.0)
        line = _PAREN.sub(" ", raw.split(";", 1)[0]).strip()
        if not line or line.upper().startswith("M112"):
            return None                                       # blank, or the profile's guard
        w = words(line)
        if not w:
            raise TranslateError("line %d: cannot read %r" % (n, raw.strip()))
        gs = [int(v) for k, v in w if k == "G"]
        ms = [int(v) for k, v in w if k == "M"]
        params = dict((k, v) for k, v in w if k not in "GMN")
        for code in gs:
            if code in (2, 3):
                raise TranslateError("line %d: arcs (G2/G3) are not supported: turn them off in the "
                                     "LightBurn device settings" % n)
            if code in (0, 1):
                self.g = code
            elif code == 90:
                self.relative = False
            elif code == 91:
                self.relative = True
            elif code not in (17, 21, 40, 54, 94):
                raise TranslateError("line %d: G%d is not supported" % (n, code))
        for code in ms:
            if code in (3, 4):
                self.laser_on = True
            elif code == 5:
                self.laser_on = False
            elif code in (2, 8, 9, 106, 107, 2002):
                pass
            else:
                raise TranslateError("line %d: M%d is not supported" % (n, code))
        a, b = self.axes
        if self.rotary and "Y" in params:
            # LightBurn still writes Y in rotary mode for some moves (the return to the finish
            # position is "G0 X0Y0"): Y is millimetres round the surface, as in its Y jogs.
            if "A" in params:
                raise TranslateError("line %d: both Y and A in one rotary move" % n)
            params["A"] = params.pop("Y") * self.deg_per_mm
        for k in params:
            if k in "XYZABC" and k not in (a, b):
                if k == "Z":
                    raise TranslateError("line %d: Z words: turn Z off for this LightBurn profile "
                                         "(focus comes from the measurement)" % n)
                raise TranslateError("line %d: %s word: this profile's axes are %s and %s"
                                     % (n, k, a, b))
        if "S" in params:
            if params["S"] > self.s_max + 1e-9 or params["S"] < 0:
                raise TranslateError("line %d: S%g is outside 0..%g: set the profile's S value max "
                                     "to %g" % (n, params["S"], self.s_max, self.s_max))
            self.power = params["S"]
        if "F" in params:
            self.feed = params["F"]
        if a not in params and b not in params:
            return None                                       # setup words, no motion
        if self.g is None:
            raise TranslateError("line %d: a move with no G0/G1 before it" % n)
        pu, pv = self.u, self.v
        if self.relative:
            if self.u is None or self.v is None:
                raise TranslateError("line %d: a relative move before any absolute %s and %s"
                                     % (n, a, b))
            self.u += params.get(a, 0.0)
            self.v += params.get(b, 0.0)
        else:
            if a in params:
                self.u = params[a]
            if b in params:
                self.v = params[b]
            if self.u is None or self.v is None:
                raise TranslateError("line %d: the first move must give both %s and %s" % (n, a, b))
        fire = self.g == 1 and self.laser_on and self.power > 0
        minutes = None
        if self.g == 1 and pu is not None:
            du, dv = abs(self.u - pu), abs(self.v - pv)
            surf = self._surface(du, dv)
            if fire and self.layer_speed:
                minutes = surf / self.layer_speed
            elif self.feed is None:
                if fire:
                    raise TranslateError("line %d: a burn with no feed rate" % n)
            elif not self.rotary:
                minutes = surf / self.feed
            elif du > 1e-9 and (du >= CONDITIONED * surf or not fire or self.v_last is None):
                minutes = du / self.feed                      # GRBL: F over the X distance
                if fire and du >= CONDITIONED * surf and minutes > 0:
                    self.v_last = surf / minutes
            elif du <= 1e-9:
                minutes = dv / self.feed                      # pure rotation: F in deg/min
                if fire and minutes > 0:
                    self.v_last = surf / minutes
            else:
                minutes = surf / self.v_last                  # nearly all rotation: carry over
        return dict(g=self.g, fire=fire, u=self.u, v=self.v, power=self.power if fire else 0.0,
                    minutes=minutes)


class RotaryEmitter(object):
    """Moves -> Snapmaker B-axis lines, one move at a time, checking each as it goes."""

    def __init__(self, surface, y_origin, y_sign=Y_SIGN, b_sign=B_SIGN, diameter=None,
                 travel_z=None, margin=0.5, max_b_feed=MAX_B_FEED, travel_feed=TRAVEL_FEED,
                 allow_negative_offset=False, seg_max=None, beam_dx=0.0, beam_dy=0.0, b_shift=0.0):
        check_offset(surface.focus_offset, allow_negative_offset)
        self.s = surface
        # Object angle = firmware B + b_shift. The measurement fixes the object's angles; a power
        # cycle (a head swap) makes the firmware call wherever B was "B0", and the plugin keeps
        # that old reading here. Everything inside works in object angles; only the B written
        # out is the firmware's.
        self.b_shift = float(b_shift or 0.0)
        # The module's probe pin minus its beam (settings "dx, dy"): the surface was measured
        # under the pin, so the head goes this much further to put the beam there instead.
        self.bx, self.by = float(beam_dx or 0.0), float(beam_dy or 0.0)
        self.shifted = (0.0, 0.0)                 # the crosshair pointer's extra head shift
        self.y_origin, self.y_sign, self.b_sign = y_origin, y_sign, b_sign
        self.d = float(diameter or 2 * surface.r)
        self.deg_per_mm = 360.0 / (math.pi * self.d)
        self.travel_z = travel_z if travel_z is not None else surface.highest_focus() + 10.0
        self.margin, self.max_b_feed, self.travel_feed = margin, max_b_feed, travel_feed
        self.prev = None                          # machine dict(y, b, z) of the last move sent
        self.armed = False                        # "M3 S0" sent since the last M5
        self.burned, self.fastest_b = 0.0, 0.0
        self.slowed = 0                     # burn moves slowed (power scaled with them) to the B limit
        # Split moves longer than this (mm of surface) so a stop waits behind short blocks and
        # Z follows the runout between the ends (None: no splitting, as for a file).
        self.seg_max = seg_max
        self.start_z = None

    def machine(self, mv):
        return self.y_origin + self.y_sign * mv["u"], self.b_sign * mv["v"]

    def shift(self, dx, dy):
        """Move every following head position by dx, dy (the crosshair pointer)."""
        self.bx += dx
        self.by += dy
        self.shifted = (self.shifted[0] + dx, self.shifted[1] + dy)

    def check_burn(self, y):
        if y < self.s.y_lo - self.margin or y > self.s.y_hi + self.margin:
            raise TranslateError("a burn at machine Y%.2f is outside the measured body Y%.2f to "
                                 "Y%.2f: move it in LightBurn or change y_origin"
                                 % (y, self.s.y_lo, self.s.y_hi))

    def preamble(self):
        # No Z move here: the first emit goes up before it goes across (never down in place).
        return ["M5", "G53 ; machine coordinates: no G92 offset can move this job", "G90", "G21",
                "M211 S1"]

    def end(self):
        self.armed = False
        return ["M5", "G0 Z%.3f F1200" % self.travel_z]

    def laser_off(self):
        """An M5 went out (the end of a burst, or the bridge's starvation guard)."""
        self.armed = False

    def emit(self, mv, check=True):
        y, b = self.machine(mv)
        if mv["fire"] and check:
            self.check_burn(y)
        if not mv["fire"]:
            y = min(max(y, self.s.y_lo - self.margin), self.s.y_hi + self.margin)
        mx, mz = self.s.axis_x(y, b), self.s.focus(y, b)
        out = []
        if self.prev is None:
            # Up first (never down where the head is: it may be over the chuck), across, then
            # down over the body. start_z: where the head is, when the caller knows.
            up = max(self.travel_z, SAFE_HIGH) if self.start_z is None else max(self.start_z, self.travel_z)
            out += ["G0 Z%.3f F1200" % up,
                    "G0 X%.3f Y%.3f B%.3f F%d" % (mx + self.bx, y + self.by, b - self.b_shift, self.travel_feed)]
            if up > self.travel_z + 1e-6:
                out.append("G0 Z%.3f F1200" % self.travel_z)
            out.append("G0 Z%.3f F600" % mz)
            self.prev = dict(y=y, b=b, z=mz, x=mx)
        if mv["fire"] and not self.armed:
            out.append("M3 S0 ; beam on at zero: power is inline on each G1 (M3 waits for the planner)")
            self.armed = True
        p = self.prev
        n = 1
        if self.seg_max and mv["g"] == 1:
            surf_all = math.hypot(y - p["y"], (b - p["b"]) / self.deg_per_mm)
            n = max(1, int(math.ceil(surf_all / float(self.seg_max) - 1e-9)))
        for k in range(1, n + 1):
            t = k / float(n)
            sy, sb = p["y"] + (y - p["y"]) * t, p["b"] + (b - p["b"]) * t
            sub = dict(mv, minutes=None if mv["minutes"] is None else mv["minutes"] / n)
            out += self._one(sub, sy, sb)
        return out

    def _one(self, mv, y, b):
        out = []
        p = self.prev
        mx, mz = self.s.axis_x(y, b), self.s.focus(y, b)
        dlin = math.hypot(y - p["y"], mx - p["x"])
        db, dz = abs(b - p["b"]), abs(mz - p["z"])
        fw = math.sqrt(dlin * dlin + db * db + dz * dz)       # the firmware's length (B deg as mm)
        if mv["g"] == 0:
            out.append("G0 X%.3f Y%.3f Z%.3f B%.3f F%d" % (mx + self.bx, y + self.by, mz, b - self.b_shift,
                                                          self.travel_feed))
        elif fw < 1e-9:
            if not mv["fire"]:
                out.append("G1 S0")
        else:
            minutes = mv["minutes"]
            f = self.travel_feed if not minutes else fw / minutes
            if mv["fire"]:
                if not minutes:
                    raise TranslateError("a burn with no speed")
                b_rate = db / minutes
                power = mv["power"]
                if b_rate > self.max_b_feed + 1e-6:
                    # Too fast for the rotary (the firmware would slow it and over-burn; LightBurn's
                    # feed on near-pure rotations runs high). Slow this move to the limit and scale its
                    # power by the same factor: the energy per mm stays as LightBurn meant.
                    k = self.max_b_feed / b_rate
                    minutes /= k
                    f = fw / minutes
                    power = power * k
                    b_rate = self.max_b_feed
                    self.slowed += 1
                self.fastest_b = max(self.fastest_b, b_rate)
                self.burned += math.hypot(dlin, db / self.deg_per_mm)
            else:
                power = mv["power"]
            out.append("G1 X%.3f Y%.3f Z%.3f B%.3f S%d F%d"
                       % (mx + self.bx, y + self.by, mz, b - self.b_shift, int(round(power)),
                          max(1, int(round(f)))))
        self.prev = dict(y=y, b=b, z=mz, x=mx)
        return out


class FlatEmitter(object):
    """Moves (LightBurn X, Y) -> Snapmaker lines for a flat job at one focus Z, in G53.

    machine X = LightBurn X + offset_x, Y = LightBurn Y + offset_y (the old start code's
    "G0 X2 Y0 / G92 X0 Y0" is offset_x 2, without leaving a G92 behind). Every move must stay
    inside the bounds (LightBurn coordinates): the soft endstops would clamp it silently and
    distort the burn.
    """

    def __init__(self, focus_z, offset_x=0.0, offset_y=0.0, bounds=(0.0, 0.0, 320.0, 350.0),
                 travel_feed=TRAVEL_FEED, seg_max=None, backlash_x=0.0, obstacles=None):
        if focus_z is None:
            raise TranslateError("no focus height: probe the work first")
        self.z = float(focus_z)
        self.ox, self.oy = float(offset_x), float(offset_y)
        self.bounds = bounds
        self.travel_feed, self.seg_max = travel_feed, seg_max
        self.prev = None
        self.armed = False
        self.burned = 0.0
        self.travel_z = self.z
        self.start_z = None
        # X backlash: moving +X the head stops this far short (measured 0.52 mm, 2026-11-22,
        # split-line burns); moving -X it lands on target. So every position reached moving +X
        # gets it added, and a burn that reverses in X first takes the slack up on the spot.
        self.bx = float(backlash_x or 0.0)
        self.xdir = None
        self.raw = None                     # the last target before the backlash term
        # objects taller than the module clears at this focus: [{"box": [x0, y0, x1, y1] (LB),
        # "name": ...}]. A move at focus height whose module outline would pass over one is
        # refused before it is sent (2026-11-22: the module came down onto a box beside a burn).
        self.obstacles = list(obstacles or [])
        self.prev_uv = None

    def preamble(self):
        return ["M5", "G53 ; machine coordinates: no G92 offset can move this job", "G90", "G21",
                "M211 S1"]

    def end(self):
        self.armed = False
        return ["M5"]

    def laser_off(self):
        self.armed = False

    def machine(self, mv):
        return mv["u"] + self.ox, mv["v"] + self.oy

    def shift(self, dx, dy):
        """Move every following head position by dx, dy (the crosshair pointer)."""
        self.ox += dx
        self.oy += dy

    def check(self, mv):
        x0, y0, x1, y1 = self.bounds
        if not (x0 - 1e-6 <= mv["u"] <= x1 + 1e-6 and y0 - 1e-6 <= mv["v"] <= y1 + 1e-6):
            raise TranslateError("a move to X%.2f Y%.2f is outside the work area X%g..%g Y%g..%g"
                                 % (mv["u"], mv["v"], x0, x1, y0, y1))

    def guard(self, u0, v0, u1, v1):
        """Refuse a move at focus height whose module outline sweeps over an obstacle."""
        if not self.obstacles:
            return
        rx0, ry0, rx1, ry1 = MODULE_REACH
        m = OBSTACLE_MARGIN
        sweep = (min(u0, u1) - rx0 - m, min(v0, v1) - ry0 - m, max(u0, u1) + rx1 + m, max(v0, v1) + ry1 + m)
        for ob in self.obstacles:
            x0, y0, x1, y1 = ob["box"]
            if sweep[0] < x1 and x0 < sweep[2] and sweep[1] < y1 and y0 < sweep[3]:
                raise TranslateError("the laser module would come down over %s (LightBurn X%.0f..%.0f Y%.0f..%.0f, "
                                     "%.0f mm tall) for the move to X%.1f Y%.1f: move the design, or take the "
                                     "object away and clear it from the obstacle list"
                                     % (ob.get("name") or "an obstacle", x0, x1, y0, y1, ob.get("top", 0.0), u1, v1))

    def _comp(self, x):
        return x + (self.bx if self.xdir == 1 else 0.0)

    def emit(self, mv, check=True):
        self.check(mv)
        x, y = self.machine(mv)
        out = []
        u, v = mv["u"], mv["v"]
        pu, pv = self.prev_uv if self.prev_uv is not None else (u, v)
        long_travel = (self.prev_uv is not None and mv["g"] == 0 and not mv["fire"]
                       and math.hypot(u - pu, v - pv) >= FLAT_HOP_MIN)
        # every move ends at focus height: a long travel only there, the rest all the way
        if long_travel:
            self.guard(u, v, u, v)
        else:
            self.guard(pu, pv, u, v)
        self.prev_uv = (u, v)
        if self.prev is None:
            up = max(self.z, SAFE_HIGH) if self.start_z is None else max(self.start_z, self.z)
            out.append("G0 Z%.3f F1200" % up)
            if self.bx and x - 2.0 >= self.bounds[0] + self.ox - 1e-6:
                # arrive moving +X, so the slack is known from the first mark on
                out.append("G0 X%.3f Y%.3f F%d" % (x - 2.0, y, self.travel_feed))
                self.xdir = 1
            out.append("G0 X%.3f Y%.3f F%d" % (self._comp(x), y, self.travel_feed))
            if up > self.z + 1e-6:
                out.append("G0 Z%.3f F600" % self.z)
            self.prev = (self._comp(x), y)
            self.raw = (x, y)
        if self.bx and self.raw is not None and abs(x - self.raw[0]) > 1e-4:
            new = 1 if x > self.raw[0] else -1
            if new != self.xdir:
                old = self.xdir
                self.xdir = new
                if mv["fire"] and old is not None and mv["g"] == 1:
                    # take the slack up where the head is (the beam does not move meanwhile)
                    tx = self.prev[0] + (self.bx if new == 1 else -self.bx)
                    minutes = mv["minutes"]
                    f = self.travel_feed if not minutes else max(1.0, math.hypot(x - self.raw[0], y - self.raw[1]) / minutes)
                    out.append("G1 X%.3f Y%.3f Z%.3f S%d F%d"
                               % (tx, self.prev[1], self.z, int(round(mv["power"])), max(1, int(round(f)))))
                    self.prev = (tx, self.prev[1])
        self.raw = (x, y)
        x = self._comp(x)
        if mv["fire"] and not self.armed:
            out.append("M3 S0 ; beam on at zero: power is inline on each G1 (M3 waits for the planner)")
            self.armed = True
        px, py = self.prev
        if (mv["g"] == 0 and not mv["fire"] and math.hypot(x - px, y - py) >= FLAT_HOP_MIN):
            # a long travel (between distant shapes, LightBurn's finish move to its park) goes up,
            # across and down: at focus height the module drags across anything taller beside
            # the work (2026-11-22: the finish move to X0 Y300 crossed a 50 mm tape roll)
            hop = min(self.z + FLAT_HOP, Z_MAX)
            out += ["G0 Z%.3f F1200" % hop, "G0 X%.3f Y%.3f F%d" % (x, y, self.travel_feed),
                    "G0 Z%.3f F600" % self.z]
            self.prev = (x, y)
            return out
        n = 1
        if self.seg_max and mv["g"] == 1:
            n = max(1, int(math.ceil(math.hypot(x - px, y - py) / float(self.seg_max) - 1e-9)))
        for k in range(1, n + 1):
            t = k / float(n)
            sx, sy = px + (x - px) * t, py + (y - py) * t
            d = math.hypot(sx - self.prev[0], sy - self.prev[1])
            if mv["g"] == 0:
                out.append("G0 X%.3f Y%.3f Z%.3f F%d" % (sx, sy, self.z, self.travel_feed))
            elif d < 1e-9:
                if not mv["fire"]:
                    out.append("G1 S0")
            else:
                minutes = None if mv["minutes"] is None else mv["minutes"] / n
                if mv["fire"] and not minutes:
                    raise TranslateError("a burn with no speed")
                f = self.travel_feed if not minutes else d / minutes
                if mv["fire"]:
                    self.burned += d
                out.append("G1 X%.3f Y%.3f Z%.3f S%d F%d"
                           % (sx, sy, self.z, int(round(mv["power"])), max(1, int(round(f)))))
            self.prev = (sx, sy)
        return out


def translate(lines, surface, y_origin, y_sign=Y_SIGN, b_sign=B_SIGN, rotary_letter="A",
              diameter=None, travel_clearance=10.0, margin=0.5, s_max=255.0,
              allow_negative_offset=False, max_b_feed=MAX_B_FEED, travel_feed=TRAVEL_FEED, b_shift=0.0):
    """A whole LightBurn file -> (Snapmaker lines, report). Refuses (TranslateError) anything it
    does not understand, before writing a line."""
    check_offset(surface.focus_offset, allow_negative_offset)
    d = float(diameter or 2 * surface.r)
    reader = Reader(("X", rotary_letter), deg_per_mm=360.0 / (math.pi * d), s_max=s_max)
    moves = [mv for mv in (reader.feed_line(raw) for raw in lines) if mv is not None]
    if not moves:
        raise TranslateError("no moves in the LightBurn file")
    burn = [mv for mv in moves if mv["fire"]]
    if not burn:
        raise TranslateError("nothing in the file burns")
    ys = [y_origin + y_sign * mv["u"] for mv in burn]
    blo, bhi = min(ys), max(ys)
    if blo < surface.y_lo - margin or bhi > surface.y_hi + margin:
        raise TranslateError("the burn runs from machine Y%.2f to Y%.2f, outside the measured body "
                             "Y%.2f to Y%.2f: move it in LightBurn or change y_origin"
                             % (blo, bhi, surface.y_lo, surface.y_hi))
    clamp = lambda y: min(max(y, surface.y_lo - margin), surface.y_hi + margin)
    travel_z = max(surface.focus(clamp(y_origin + y_sign * mv["u"]), b_sign * mv["v"])
                   for mv in moves) + travel_clearance
    em = RotaryEmitter(surface, y_origin, y_sign, b_sign, diameter=d, travel_z=travel_z,
                       margin=margin, max_b_feed=max_b_feed, travel_feed=travel_feed,
                       allow_negative_offset=allow_negative_offset, b_shift=b_shift)
    out = [
        "; Snapmaker rotary job translated from LightBurn by OctoPrint-LaserHeightProbe",
        "; centre at B0: X%.3f at Y%.3f (%+.3f mm per 100 mm Y), top Z%.3f, focus offset %+.3f"
        % (surface.x0, surface.y0, 100 * surface.dx, surface.top(surface.y0), surface.focus_offset),
        "; body Y%.2f to Y%.2f, diameter %.2f, y_origin %.2f, y_sign %d, b_sign %d"
        % (surface.y_lo, surface.y_hi, d, y_origin, y_sign, b_sign),
    ] + em.preamble()
    for mv in moves:
        out += em.emit(mv, check=False)                       # checked above, as a whole
    # No G54 at the end: on this machine G54 holds a stale offset from old LightBurn flat jobs
    # (seen 2026-11-17: about X-147.5 Y-153.5 Z-245.1), and G53 (which keeps M206) is the frame
    # every measurement is in. G53 stays in force after the job.
    out += em.end() + ["; end"]
    bs = [b_sign * mv["v"] for mv in moves]
    report = dict(moves=len(moves), burn_moves=len(burn), y_from=blo, y_to=bhi,
                  b_from=min(bs), b_to=max(bs), travel_z=round(travel_z, 3),
                  burned_mm=round(em.burned, 1), fastest_b_deg_min=round(em.fastest_b))
    return out, report
