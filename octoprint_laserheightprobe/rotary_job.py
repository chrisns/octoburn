# coding=utf-8
"""The rotary routine: measure a round object in the rotary chuck with the probe.

Order (Chris, 2026-11-17): runout first (is the object true in the chuck: touch the top, turn B
90 degrees, touch again, all the way round), then the axis, then the ends.

1. Runout at each runout Y: the top at the axis X for each B angle. B only turns with the head
   at travel height and the pin stowed. B goes back to 0 afterwards.
2. Axis: at each station Y, a chord of points across the top. A circle of the known diameter
   gives the axis X and Z there; a free fit checks the diameter. Each station starts from the
   last axis X, so the plate need not be seated exactly: X always comes from the probe.
3. Free end: walk from the first station towards lower Y until the top drops off the axis line
   (the edge round-over) or the pin misses; then halve the gap to the resolution.
4. Shoulder: walk from the last station towards the chuck until the top starts to drop, halve
   the gap, then profile the shoulder until it has dropped `shoulder_drop` or reaches
   `y_chuck_limit` (never closer to the jaws than that).

Every probe has its own floor, from the geometry: never lower than the head can go without the
probe body or the plate's bottom edge touching the object (rotary.lowest_safe_trigger), and
never much lower than the surface is expected (a miss is then reported, not chased).

0. Reference (when rotary_ref_offset is set, or known_diameter is given): touch the top of the
   rotary's motor housing, which does not turn and is reachable whatever is in the chuck. Its
   top only flattens out past the soft limit, so soft endstops go off for it (Y322.5; the
   endstop is at about Y322.97), back on after, and Y is homed again. The housing top has a
   seam dip in the middle, so the touches are ref_half either side of ref_x.
   Diameter at each runout Y = 2 x (mean top over the B angles - spin axis), with
   spin axis = housing - rotary_ref_offset + rotary_ref_slope x (Y - ref_y). Both are top
   touches with the same pin, so its pre-travel cancels. The measured diameter is then the one
   the stations fit. A run with known_diameter measures offset and slope instead.

All heights are trigger Z (commanded Z when the deployed pin trips).
"""
from __future__ import absolute_import, division

import math

from . import rotary

# The Y endstop triggers at about Y322.97 (2026-11-18): the reference never goes past this.
REF_Y_MAX = 322.8

DEFAULTS = dict(
    diameter=None,               # typed in; optional when the reference step measures it
    known_diameter=None,         # a calibrated object: measure ref_offset and ref_slope instead
    ref_offset=None,             # housing touch - spin axis (settings rotary_ref_offset)
    ref_slope=None,              # spin axis rise per mm of Y (settings rotary_ref_slope)
    ref_x=164.8,                 # housing centre X (head position)
    ref_y=322.5,
    ref_half=5.5,                # touch this far either side of ref_x, off the seam dip
    ref_radius=30.65,            # housing top radius (profile fit, A350 rotary, 2026-11-18)
    ref_recentre=0.5,            # a touch pair this far off the housing centre moves over and repeats
    ref_recentre_max=3,          # moves allowed before the housing is called not found
    max_diameter=None,           # the most the object can measure: bounds travel before its top is touched
    ref_repeats=2,
    ref_expected=None,           # housing trigger Z, for the floor: ref_housing_z, else 133.55
    ref_window=6.0,
    ref_housing_z=None,          # the calibration's housing touch (settings rotary_ref_housing_z)
    probe_drift_warn=1.0,        # a housing this far from ref_housing_z: the probe moved on the head
    ref_travel=None,             # head Z for the move to the housing: max(160, crown + 15)
    ref_spread=0.3,              # the housing touches must agree this well
    dia_warn=0.5,                # typed vs measured diameter
    taper_warn=1.0,              # measured diameters along Y
    chord_half=15.0,
    chord_n=5,
    runout_angles=[0.0, 90.0, 180.0, 270.0],
    clearance=4.0,               # deployed tip above the highest expected top, for travel
    below_expected=3.0,          # a chord point may trip at most this far below the fit
    end_step=10.0,
    end_resolution=1.0,
    edge_drop=0.5,               # the top this far below the axis line = off the full diameter
    miss_window=6.0,             # walking off the end: no trip within this below the line = miss
    shoulder_drop=8.0,
    shoulder_step=5.0,
    y_min=0.0,
    y_chuck_limit=None,
    # The rotary body (settings rotary_head_y_limit): the head never goes past this Y. Objects
    # can sit over the chuck, so this, not the jaws, is the limit (Chris jogged it, 2026-11-19).
    # A head position, so it holds for any module: in touch Y it is this + the module's dy.
    head_y_limit=None,
    # Chris, 2026-11-19: where the body ends is a guess, like the diameter (nobody knows exactly
    # how far into the chuck the object went). The stations go inside the guesses, the walks
    # find the real ends, and an end this far from its guess warns.
    free_end_guess=None,
    chuck_end_guess=None,
    station_inset=20.0,          # stations this far inside the guesses (a quarter of a short body)
    end_warn=10.0,
    max_rms=0.3,                 # a station or runout fit worse than this stops the run
    # Chris, 2026-11-19: a flask is never perfect. A top that rises and falls by less than this per
    # turn stays within the laser's focal range (the bridge also follows the runout in Z), so it
    # is accepted and reported; more is measured and warned about. Travel allows it from the
    # first B turn, and the max_diameter bound allows half of it (the centre off the axis).
    runout_tolerance=3.0,
    probe=None,                  # per-job probe override: stages, samples, max_samples
)

# Fast is the default since 2026-11-17: on the 78 mm flask it took 13.4 min against 43 for precise,
# with the top within 0.006 mm, the axis X within 0.06 mm and the body ends within 0.7 mm.
# Fast (Chris, 2026-11-17): focus tolerance is far wider than the probe's 0.02 mm fine stage, and
# the object's shape between points comes from the model (a straight axis, a runout sinusoid),
# so one touch at 0.1 mm per point and fewer points are enough. A bad single touch shows as a
# fit residual over max_rms instead of as a repeat that disagrees.
PRESETS = dict(
    fast=dict(chord_n=4, end_step=20.0, end_resolution=2.0, shoulder_step=10.0,
              probe=dict(stages="0.5; 0.1, 1.2", samples=1, max_samples=1)),
    precise=dict(),
)


class RotaryError(ValueError):
    pass


def swing(runout):
    """How far the top rises and falls per turn at one runout Y: twice the fitted eccentricity
    (the touches at four angles can miss the peaks), never less than the spread touched."""
    return max(runout["spread"], 2.0 * runout.get("e", 0.0))


def runout_summary(runs, tolerance):
    """Summary fields and warnings for the runout: the largest swing, per Y, within tolerance."""
    if not runs:
        return dict(runout_tolerance=tolerance, runout_swing=None, runout_swing_by_y=[], runout_within=None), []
    by_y = [(r["y"], round(swing(r), 3)) for r in runs]
    worst = max(sw for _, sw in by_y)
    warnings = []
    if worst > tolerance:
        warnings.append("the object swings %.1f mm per turn (%s), more than the %.1f mm tolerance: focus "
                        "drifts by up to %.1f mm either way unless it is reseated truer in the chuck"
                        % (worst, ", ".join("Y%.0f %.1f" % t for t in by_y), tolerance, worst / 2.0))
    return dict(runout_tolerance=tolerance, runout_swing=round(worst, 3), runout_swing_by_y=by_y,
                runout_within=worst <= tolerance), warnings


MOVED_X = 0.5      # mm: the housing this far sideways from the last measurement means the rotary moved
MOVED_Z = 1.0      # mm: in Z the probe's seat on the head also shifts readings, so warn only past this


def housing_shift(ref, seed_ref):
    """(dx, dz) of the housing from the last measurement's reference, or None."""
    if not ref or not seed_ref or seed_ref.get("centre_x") is None or seed_ref.get("z") is None:
        return None
    return round(ref["centre_x"] - float(seed_ref["centre_x"]), 3), round(ref["z"] - float(seed_ref["z"]), 3)


def params(raw):
    """Fill defaults and check what the routine needs. Raises RotaryError."""
    p = dict(DEFAULTS)
    raw = dict(raw or {})
    preset = raw.pop("preset", None) or "fast"
    if preset not in PRESETS:
        raise RotaryError("rotary: preset must be one of %s" % ", ".join(sorted(PRESETS)))
    p.update(PRESETS[preset])
    p.update(dict((k, v) for k, v in raw.items() if v is not None))
    p["preset"] = preset
    if p.get("probe"):
        p["probe"] = _probe_override(p["probe"])
    if p.get("stations") in (None, [], "") and p["free_end_guess"] is not None and p["chuck_end_guess"] is not None:
        lo, hi = float(p["free_end_guess"]), float(p["chuck_end_guess"])
        if hi - lo < 20.0:
            raise RotaryError("rotary: the guessed ends (Y%.1f to Y%.1f) must be at least 20 mm apart" % (lo, hi))
        if p["y_chuck_limit"] is not None and hi > float(p["y_chuck_limit"]):
            raise RotaryError("rotary: the chuck end guessed (Y%.1f) is past the never past Y%.1f limit"
                              % (hi, float(p["y_chuck_limit"])))
        inset = min(float(p["station_inset"]), (hi - lo) / 4.0)
        p["stations"] = [round(lo + inset, 3), round((lo + hi) / 2.0, 3), round(hi - inset, 3)]
    for key in ("axis_x", "crown", "stations"):
        if p.get(key) in (None, [], ""):
            raise RotaryError("rotary: %s is required (probe the top once by hand first)" % key)
    p["stations"] = sorted(float(y) for y in p["stations"])
    p["runout_ys"] = [float(y) for y in (p.get("runout_ys") or [p["stations"][0], p["stations"][-1]])]
    if p.get("steps") is None:
        p["steps"] = (["reference"] if p["ref_offset"] is not None or p["known_diameter"] else []) + \
            ["runout", "stations", "free_end", "shoulder"]
    measures = "reference" in p["steps"] or (p.get("resume") or {}).get("reference")
    if p["known_diameter"] and "reference" not in p["steps"]:
        raise RotaryError("rotary: known_diameter needs the reference step")
    if p["diameter"] is None and p["known_diameter"]:
        p["diameter"] = p["known_diameter"]
    if p["diameter"] is None and not (measures and p["ref_offset"] is not None):
        raise RotaryError("rotary: diameter is required unless the reference step measures it "
                          "(set rotary_ref_offset)")
    if p["diameter"] is not None:
        check_chord(p, p["diameter"])
    if p["ref_expected"] is None:
        p["ref_expected"] = p["ref_housing_z"] if p["ref_housing_z"] is not None else 133.55
    if p["ref_y"] > REF_Y_MAX:
        raise RotaryError("rotary: ref_y %.2f is past Y%.1f: the Y endstop is at about Y322.97"
                          % (p["ref_y"], REF_Y_MAX))
    if int(p["chord_n"]) < 3:
        raise RotaryError("rotary: chord_n must be 3 or more")
    if p["y_chuck_limit"] is None and p["head_y_limit"] is None:
        raise RotaryError("rotary: y_chuck_limit is required: the Y the probe must not pass "
                          "towards the rotary body (or set rotary_head_y_limit)")
    return p


def check_chord(p, diameter):
    limit = rotary.max_chord_half_width(diameter / 2.0)
    if not 0 < p["chord_half"] <= limit:
        raise RotaryError("rotary: chord_half %.1f must be within 0..%.1f mm (35 degree slope on a "
                          "%.1f mm diameter)" % (p["chord_half"], limit, diameter))


def _line_fit(points):
    """(slope, mean x, mean y) of the least-squares line through (x, y) points."""
    n = float(len(points))
    mx = sum(x for x, _ in points) / n
    my = sum(y for _, y in points) / n
    sxx = sum((x - mx) ** 2 for x, _ in points)
    sxy = sum((x - mx) * (y - my) for x, y in points)
    return (sxy / sxx if sxx > 0 else 0.0), mx, my


def diameters(reference, runout, offset, slope, ref_y):
    """[(y, diameter)] from the housing reference and the runout means."""
    out = []
    for r in runout or []:
        if r.get("mean") is None:
            continue
        axis = reference - offset + (slope or 0.0) * (r["y"] - ref_y)
        out.append((r["y"], round(2.0 * (r["mean"] - axis), 3)))
    return out


def calibrate(reference, runout, known_diameter, slope, ref_y, min_span=100.0):
    """offset (and slope, if the runout Ys span min_span mm) from an object of known diameter."""
    axes = [(r["y"], r["mean"] - known_diameter / 2.0) for r in runout or [] if r.get("mean") is not None]
    if not axes:
        raise RotaryError("rotary: calibrating needs the runout means")
    measured = False
    if len(axes) >= 2 and max(y for y, _ in axes) - min(y for y, _ in axes) >= min_span:
        slope, measured = _line_fit(axes)[0], True
    slope = slope or 0.0
    offset = sum(reference - a + slope * (y - ref_y) for y, a in axes) / len(axes)
    return dict(offset=round(offset, 4), slope=round(slope, 6), slope_measured=measured)


def _probe_override(raw):
    from .settings import parse_stages, SettingsError
    out = dict(raw)
    try:
        stages = parse_stages(str(out["stages"]))
    except (KeyError, SettingsError) as exception:
        raise RotaryError("rotary: probe stages: %s" % exception)
    out["stages"] = "; ".join("%g" % s if a is None else "%g, %g" % (s, a) for s, a in stages)
    for key, lo, hi in (("samples", 1, 10), ("max_samples", 1, 10)):
        if key in out:
            out[key] = int(out[key])
            if not lo <= out[key] <= hi:
                raise RotaryError("rotary: probe %s must be %d to %d" % (key, lo, hi))
    if out.get("max_samples", 10) < out.get("samples", 1):
        raise RotaryError("rotary: probe max_samples must be at least samples")
    return out


def probe_config(config, p):
    """The routine config with the job's probe override applied (stages parsed)."""
    o = p.get("probe")
    if not o:
        return config
    from .settings import parse_stages
    c = dict(config, stages=parse_stages(o["stages"]))
    for key in ("samples", "max_samples", "tolerance"):
        if key in o:
            c[key] = o[key]
    return c


class Rotary(object):
    """Runs the routine on a Prober. tell(text) reports progress; check() raises on abort."""

    def __init__(self, prober, p, tell, check=lambda: None):
        self.pr = prober
        self.p = p
        self.r = (p["diameter"] or 0.0) / 2.0
        self.tell = tell
        self.check = check
        self.axis_x = float(p["axis_x"])
        self.crown = float(p["crown"])
        self.line = None
        self.log = []
        self.out = None
        self.top_confirmed = False   # a touch in this run found the object where crown said
        self.clear_top = None        # clear air over the object from max_diameter (reference step)
        limits = [float(p["y_chuck_limit"])] if p.get("y_chuck_limit") is not None else []
        if p.get("head_y_limit") is not None:
            limits.append(float(p["head_y_limit"]) + float(getattr(prober, "_dy", 0.0) or 0.0))
        self.y_limit = round(min(limits), 3) if limits else None

    # -- probing helpers ------------------------------------------------

    def travel(self, extra=0.0):
        return round(self.crown + self.p["clearance"] + extra, 3)

    def floor_for(self, x, expected, window):
        safe = rotary.lowest_safe_trigger(self.crown, self.r, x - self.axis_x)
        return round(max(expected - window, safe + 0.5), 3)

    def touch(self, x, y, expected, window, travel_extra=0.0):
        self.check()
        floor = self.floor_for(x, expected, window)
        travel, drop = self.travel(travel_extra), self.top_confirmed
        if not drop and self.clear_top is not None:
            travel, drop = max(travel, self.clear_top), True      # the max_diameter bound is clear air
        res = self.pr.probe_at(x, y, travel, floor, drop=drop)
        z = None if res is None else res["z"]
        if z is not None:
            self.top_confirmed = True
        self.log.append(dict(x=round(x, 3), y=round(y, 3), z=z, floor=floor))
        return z

    def crown_at(self, y):
        return self.line["z_at"](y) + self.r if self.line else self.crown

    def axis_x_at(self, y):
        return self.line["x_at"](y) if self.line else self.axis_x

    # -- the steps ------------------------------------------------------

    def reference(self):
        """Touch the housing top past the soft Y limit; returns dict(z, touches, spread, centre_x).

        The head crosses the chuck at `cross`. The first touch probes down from there and
        confirms the housing; later touches start clearance above the highest found (it moves a
        few mm between remounts) on the crossing height's coarse grid (Chris, 2026-11-18: 26 mm
        climbs between touches). Each pair either side of the seam gives the housing's sideways
        shift from the known radius; a pair more than ref_recentre off moves over and repeats,
        so the kept touches sit as they did in the calibration and rotary_ref_offset holds.
        The head leaves as high as it came, or at the max_diameter bound when one is given.
        """
        p = self.p
        cross = p["ref_travel"] or round(max(160.0, self.crown + 15.0), 3)
        coarse = self.pr.coarse_step
        arrival = max(cross, self.pr.where()[2])

        def low_above(found):
            return round(cross - math.floor(max(0.0, cross - found - p["clearance"]) / coarse) * coarse, 3)
        floor = p["ref_expected"] - p["ref_window"]
        half, radius = p["ref_half"], p["ref_radius"]
        centre, moves = p["ref_x"], 0
        touches, kept, shifts = [], [], []
        leave = arrival
        self.pr.lift_to(cross)
        # A fresh Y frame: the housing top slopes along Y near its edge (2026-11-19).
        self.pr.rehome_y()
        self.pr.soft_endstops(False)
        try:
            while len(kept) < 2 * int(p["ref_repeats"]):
                pair = []
                for x in (centre - half, centre + half):
                    self.check()
                    # never lower than over the housing's expected top: a touch on its side (the
                    # housing off centre) reads low, and the other side is higher (2026-11-22: the
                    # pin deployed 0.4 mm over the housing after a side touch at Z128.6)
                    travel = low_above(max([z for _, z in touches] + [p["ref_expected"]])) if touches else cross
                    res = self.pr.probe_at(x, p["ref_y"], travel, floor, head=True, drop=True)
                    if res is None:
                        raise RotaryError("reference: nothing under the pin at X%.1f Y%.1f above Z%.2f: "
                                          "is the rotary fitted?" % (x, p["ref_y"], floor))
                    touches.append((round(x, 3), res["z"]))
                    pair.append(res["z"])
                    self.tell("Reference X%.1f Y%.1f: Z%.3f" % (x, p["ref_y"], res["z"]))
                shift = (pair[1] - pair[0]) * radius / (2.0 * half)
                if abs(shift) > p["ref_recentre"]:
                    moves += 1
                    if moves > p["ref_recentre_max"]:
                        raise RotaryError("reference: the housing centre would not settle (last shift %+.2f mm)"
                                          % shift)
                    centre = round(centre + shift, 3)
                    self.tell("Reference: housing %.2f mm off centre; touching again at X%.2f" % (shift, centre))
                    continue
                kept += [(round(centre - half, 3), pair[0]), (round(centre + half, 3), pair[1])]
                shifts.append(shift)
            leave = self._leave_height(cross, arrival, sum(z for _, z in kept) / len(kept))
        finally:
            try:
                self.pr.lift_to(leave)
            finally:
                self.pr.soft_endstops(True)
                self.pr.rehome_y()
        zs = [z for _, z in kept]
        spread = max(zs) - min(zs)
        if spread > p["ref_spread"]:
            raise RotaryError("reference: the housing touches disagree by %.3f mm (more than %.2f): %s"
                              % (spread, p["ref_spread"], ", ".join("X%.1f Z%.3f" % t for t in kept)))
        z = sum(zs) / len(zs)
        centre_x = round(centre + sum(shifts) / len(shifts), 3)
        self.tell("Reference: housing Z%.3f (spread %.3f mm), centre X%.2f" % (z, spread, centre_x))
        return dict(z=round(z, 4), y=p["ref_y"], touches=kept, all_touches=touches, spread=round(spread, 3),
                    centre_x=centre_x)

    def _leave_height(self, cross, arrival, housing_z):
        """Where the head crosses the object after the housing. With max_diameter and the
        reference constants: the highest the object can reach (axis + max radius) + clearance,
        never below the chuck crossing; it also bounds the first object touch. Otherwise as high
        as the head arrived: nothing yet says a lower height clears the object."""
        p = self.p
        if not p["max_diameter"] or p["ref_offset"] is None:
            return arrival
        ys = list(p["runout_ys"]) + list(p["stations"])
        axis_hi = max(housing_z - p["ref_offset"] + (p["ref_slope"] or 0.0) * (y - p["ref_y"]) for y in ys)
        top = axis_hi + p["max_diameter"] / 2.0 + p["runout_tolerance"] / 2.0
        if self.crown > top + 1.0:
            raise RotaryError("reference: the crown given (Z%.2f) is above the highest a %.1f mm max_diameter "
                              "object can reach here (Z%.2f): check max_diameter and crown"
                              % (self.crown, p["max_diameter"], top))
        self.clear_top = round(top + p["clearance"], 3)
        return max(cross, self.clear_top)

    def _provisional_radius(self, ref):
        """A radius for the runout floors before the diameter is measured."""
        if self.r or ref is None or self.p["ref_offset"] is None:
            return
        ys = self.p["runout_ys"]
        y = sum(ys) / len(ys)
        axis = ref["z"] - self.p["ref_offset"] + (self.p["ref_slope"] or 0.0) * (y - ref["y"])
        self.r = max(5.0, self.crown - axis)

    def measured_diameter(self, out):
        """Work out the diameter from the reference and runout; set self.r. Returns warnings."""
        p, ref, warnings = self.p, out.get("reference"), []
        if ref is None:
            return warnings
        offset, slope = p["ref_offset"], p["ref_slope"]
        if p["known_diameter"]:
            cal = calibrate(ref["z"], out.get("runout"), float(p["known_diameter"]), slope, ref["y"])
            ref["calibration"] = cal
            offset, slope = cal["offset"], cal["slope"]
            self.tell("Calibrated on a %.2f mm object: housing - axis %.3f mm, axis slope %.5f per mm%s"
                      % (p["known_diameter"], offset, slope, "" if cal["slope_measured"] else " (kept)"))
        if offset is None:
            return warnings
        by_y = diameters(ref["z"], out.get("runout"), offset, slope, ref["y"])
        if not by_y:
            raise RotaryError("rotary: the runout gave no mean top, so the diameter cannot be measured")
        ds = [d for _, d in by_y]
        d = round(sum(ds) / len(ds), 3)
        ref.update(diameter=d, diameter_by_y=by_y)
        self.tell("Diameter %.2f mm (%s)" % (d, ", ".join("Y%.0f %.2f" % t for t in by_y)))
        if max(ds) - min(ds) > p["taper_warn"]:
            warnings.append("the diameter changes by %.2f mm along Y (%s): the object is tapered or "
                            "sits tilted; LightBurn assumes a cylinder"
                            % (max(ds) - min(ds), ", ".join("Y%.0f %.2f" % t for t in by_y)))
        if p["ref_housing_z"] is not None and not p["known_diameter"] and \
                abs(ref["z"] - p["ref_housing_z"]) > p["probe_drift_warn"]:
            warnings.append("the housing reads %+.2f mm from the calibration's (Z%.3f): the probe sits "
                            "differently on the head, or the rotary moved. This diameter uses this run's "
                            "own housing touch, so it holds; focus heights calibrated before the change "
                            "do not: check the probe mount and calibrate the focus again"
                            % (ref["z"] - p["ref_housing_z"], p["ref_housing_z"]))
        typed = p["diameter"] if not p["known_diameter"] else None
        if typed is not None and abs(typed - d) > p["dia_warn"]:
            warnings.append("the diameter typed in (%.2f) is %.2f mm off the measured %.2f: the "
                            "measured one is used" % (typed, abs(typed - d), d))
        check_chord(p, d)
        self.r = d / 2.0
        for w in warnings:
            self.tell("Warning: " + w)
        return warnings

    def runout(self, y):
        # allow: how far the top may sit above the crown at the next angle. The first angle is
        # where the crown was probed by hand, so a top well above it there means the crown is
        # wrong (stop, as for any touch near travel). After it, the tolerance (the crown says
        # nothing about the other angles), or twice the largest rise seen if the object is worse.
        samples, allow = [], float(self.p["runout_tolerance"])
        try:
            for i, b in enumerate(self.p["runout_angles"]):
                self.pr.turn_b(b, self.travel(allow))
                z = self.touch(self.axis_x, y, self.crown, 3.0 + allow, allow if i else 0.0)
                if z is None:
                    raise RotaryError("runout at Y%.1f B%.0f: no touch near the top" % (y, b))
                samples.append((b, z))
                allow = max(allow, 2 * abs(z - samples[0][1]))
                self.tell("Runout Y%.1f B%.0f: Z%.3f" % (y, b, z))
        finally:
            self.pr.turn_b(0.0, self.travel(allow))
        fit = rotary.fit_runout(samples) if len(samples) >= 3 else None
        if fit and fit["rms"] > self.p["max_rms"]:
            raise RotaryError("runout at Y%.1f: the touches do not fit a turning circle (rms %.3f > "
                              "%.3f mm): %s" % (y, fit["rms"], self.p["max_rms"],
                                                ", ".join("B%.0f Z%.3f" % s for s in samples)))
        out = dict(y=y, samples=samples, spread=max(z for _, z in samples) - min(z for _, z in samples))
        if fit:
            out.update(e=fit["e"], phase_deg=fit["phase_deg"], mean=fit["mean"])
        out["swing"] = round(swing(out), 3)
        self.tell("Runout Y%.1f: spread %.3f mm%s, swing %.2f mm per turn (%s the %.1f mm tolerance)"
                  % (y, out["spread"], ", eccentricity %.3f mm" % out["e"] if fit else "", out["swing"],
                     "within" if out["swing"] <= self.p["runout_tolerance"] else "over",
                     self.p["runout_tolerance"]))
        return out

    def verify(self, measured, ys, angles, b_shift=0.0, window=3.0):
        """Touch the top where a measurement (a rotary result) says it is, at each Y and object
        angle: how far rotary mode's focus would be off there, runout model included. B turns
        at a height over the measured object's highest point at any angle."""
        from .rotary_gcode import Surface
        surface = Surface(measured, 0.0)
        self.r = surface.r
        self.crown = max(surface.top(y, b) for y in (surface.y_lo, surface.y_hi) for b in range(0, 360, 15))
        start_b = self.pr.where_b() if hasattr(self.pr, "where_b") else 0.0
        rows, sideways = [], []
        h = float(self.p.get("chord_half") or 15.0)
        drop = self.r - math.sqrt(max(0.0, self.r * self.r - h * h))
        try:
            for k, b in enumerate(angles):
                self.pr.turn_b(b - b_shift, self.travel(window))
                if k == 0:
                    # either side of the crest the surface slopes, so a sideways error shows there
                    # (the crest itself hardly moves: 2 mm sideways is 0.05 mm on a 78 mm tube)
                    for y in ys:
                        x = surface.axis_x(y, b)
                        top = surface.top(y, b)
                        self.axis_x = x
                        zl = self.touch(x - h, y, top - drop, window, window)
                        zr = self.touch(x + h, y, top - drop, window, window)
                        if zl is None or zr is None:
                            raise RotaryError("verify at Y%.1f: nothing beside the crest where the measurement "
                                              "puts it: the object or the rotary has moved" % y)
                        dx = (zr - zl) * math.sqrt(max(1.0, self.r * self.r - h * h)) / (2.0 * h)
                        sideways.append(dict(y=round(y, 2), dx=round(dx, 3)))
                        self.tell("Verify Y%.1f: the crest is %+.2f mm sideways of the measurement" % (y, dx))
                for y in ys:
                    x = surface.axis_x(y, b)
                    expected = surface.top(y, b)
                    self.axis_x = x
                    z = self.touch(x, y, expected, window, window)
                    if z is None:
                        raise RotaryError("verify at Y%.1f angle %.0f: nothing within %.1f mm of where the "
                                          "measurement puts the top (Z%.2f): the object or the rotary has moved"
                                          % (y, b, window, expected))
                    rows.append(dict(y=round(y, 2), b=b, expected=round(expected, 3), z=z, miss=round(z - expected, 3)))
                    self.tell("Verify Y%.1f angle %.0f: Z%.3f, %+.3f mm from the measurement" % (y, b, z, z - expected))
        finally:
            self.pr.turn_b(start_b if start_b is not None else 0.0, self.travel(window))
        misses = [r["miss"] for r in rows]
        rms = (sum(m * m for m in misses) / len(misses)) ** 0.5
        worst = max(rows, key=lambda r: abs(r["miss"]))
        side = max(sideways, key=lambda r: abs(r["dx"])) if sideways else None
        self.tell("Verify: %d touches, rms %.3f mm, worst %+.3f mm at Y%.1f angle %.0f%s"
                  % (len(rows), rms, worst["miss"], worst["y"], worst["b"],
                     "; sideways up to %+.2f mm" % side["dx"] if side else ""))
        return dict(points=rows, rms=round(rms, 3), worst=worst, sideways=sideways,
                    sideways_worst=side["dx"] if side else None)

    def station(self, y):
        xs = rotary.chord_xs(self.axis_x, self.p["chord_half"], int(self.p["chord_n"]))
        pts = []
        zc = self.crown - self.r
        for x in xs:
            d = x - self.axis_x
            expected = zc + math.sqrt(self.r ** 2 - d ** 2)
            z = self.touch(x, y, expected, self.p["below_expected"])
            if z is None:
                raise RotaryError("station Y%.1f: no touch at X%.2f within %.1f mm below the "
                                  "expected Z%.3f" % (y, x, self.p["below_expected"], expected))
            pts.append((x, z))
        fit = rotary.fit_circle(pts, self.r)
        if fit["rms"] > self.p["max_rms"]:
            raise RotaryError("station Y%.1f: the touches do not fit a %.1f mm circle (rms %.3f > "
                              "%.3f mm): %s" % (y, 2 * self.r, fit["rms"], self.p["max_rms"],
                                                ", ".join("X%.2f Z%.3f" % q for q in pts)))
        free = rotary.fit_circle_free(pts)
        self.axis_x, self.crown = fit["xc"], fit["crown"]
        self.tell("Station Y%.1f: axis X%.3f Z%.3f (top Z%.3f), rms %.3f; free fit diameter %.2f"
                  % (y, fit["xc"], fit["zc"], fit["crown"], fit["rms"], free["diameter"]))
        return dict(y=y, points=pts, xc=fit["xc"], zc=fit["zc"], crown=fit["crown"], rms=fit["rms"],
                    free_diameter=free["diameter"], free_rms=free["rms"])

    def _walk(self, y0, direction, step, ok, y_stop):
        """Step from y0 while ok(y) holds; return (last ok y, first not-ok y or None). A step
        that would pass y_stop touches y_stop itself instead, so the limit is always tried."""
        last, y = y0, y0
        while True:
            y = round(y + direction * step, 3)
            if (direction < 0 and y < y_stop) or (direction > 0 and y > y_stop):
                if abs(last - y_stop) < 1e-6:
                    return last, None
                y = round(y_stop, 3)
            if not ok(y):
                return last, y
            last = y

    def _bisect(self, good, bad, ok):
        while abs(bad - good) > self.p["end_resolution"]:
            mid = round((good + bad) / 2.0, 3)
            if ok(mid):
                good = mid
            else:
                bad = mid
        return good, bad

    def free_end(self, y_from):
        drops = {}

        def full(y):
            line = self.crown_at(y)
            z = self.touch(self.axis_x_at(y), y, line, self.p["miss_window"])
            drops[y] = None if z is None else round(line - z, 3)
            return z is not None and line - z <= self.p["edge_drop"]
        good, bad = self._walk(y_from, -1, self.p["end_step"], full, self.p["y_min"])
        if bad is None:
            self.tell("Free end: still full diameter at Y%.1f (the Y limit)" % good)
            return dict(full_to=good, edge=None, miss=None, drops=drops)
        good, bad = self._bisect(good, bad, full)
        misses = sorted(y for y, d in drops.items() if d is None)
        self.tell("Free end: full diameter to Y%.1f, off it by Y%.1f" % (good, bad))
        return dict(full_to=good, edge=bad, miss=max(misses) if misses else None, drops=drops)

    def shoulder(self, y_from):
        limit = self.y_limit

        def full(y):
            line = self.crown_at(y)
            z = self.touch(self.axis_x_at(y), y, line, self.p["miss_window"])
            return z is not None and line - z <= self.p["edge_drop"]
        good, bad = self._walk(y_from, +1, self.p["end_step"], full, limit)
        if bad is None:
            self.tell("Shoulder: still full diameter at Y%.1f (the chuck limit)" % good)
            return dict(full_to=good, starts=None, profile=[])
        good, bad = self._bisect(good, bad, full)
        profile, y = [], good
        while True:
            y = round(y + self.p["shoulder_step"], 3)
            if y > limit:
                break
            line = self.crown_at(y)
            window = self.p["shoulder_drop"] + 3.0
            z = self.touch(self.axis_x_at(y), y, line, window)
            if z is None:
                profile.append(dict(y=y, z=None))
                break
            profile.append(dict(y=y, z=z, radius=round(z - self.line["z_at"](y), 3)))
            if line - z >= self.p["shoulder_drop"]:
                break
        self.tell("Shoulder: full diameter to Y%.1f, dropping from Y%.1f" % (good, bad))
        return dict(full_to=good, starts=bad, profile=profile)

    def run(self):
        """Run the steps, keeping what is done in self.out so a failure loses nothing.

        p["resume"] (a previous result, complete or not) supplies the steps not run again;
        p["steps"] picks which to run (default all four).
        """
        p = self.p
        prev = p.get("resume") or {}
        steps = p.get("steps") or ["runout", "stations", "free_end", "shoulder"]
        out = self.out = dict(params=dict((k, v) for k, v in p.items() if k not in ("resume", "verify_against")),
                              complete=False)
        if "verify" in steps:
            out["verify"] = self.verify(p["verify_against"], p["verify_ys"], p["verify_angles"],
                                        float(p.get("verify_b_shift") or 0.0))
            out["verify_only"] = True
            out["touches"] = self.log
            return out
        if p.get("chuck_end_guess") is not None and float(p["chuck_end_guess"]) > self.y_limit:
            raise RotaryError("rotary: the chuck end guessed (Y%.1f) is past Y%.1f, the nearest the rotary "
                              "body allows" % (float(p["chuck_end_guess"]), self.y_limit))
        if max(p["stations"]) > self.y_limit:
            raise RotaryError("rotary: a station (Y%.1f) is past Y%.1f, the nearest the rotary body allows"
                              % (max(p["stations"]), self.y_limit))
        for key in ("reference", "runout", "stations", "free_end", "shoulder"):
            if key not in steps and prev.get(key) is not None:
                out[key] = prev[key]
        if "reference" in steps:
            out["reference"] = self.reference()
            shift = housing_shift(out["reference"], p.get("seed_ref"))
            if shift is not None:
                dx, dz = shift
                moved = abs(dx) > MOVED_X
                out["housing_shift"] = dict(dx=dx, dz=dz, moved=moved, z_differs=abs(dz) > MOVED_Z)
                if p.get("check_only"):
                    self.tell("Housing X%+.2f Z%+.2f mm from the last measurement: %s" % (
                        dx, dz, "the rotary has moved" if moved else "the rotary has not moved"))
                    out["check_only"] = True
                    out["touches"] = self.log
                    return out
                if abs(dx) > 0.3 or abs(dz) > 0.3:
                    # the object moves with the housing: start where it should be now
                    self.axis_x = round(self.axis_x + dx, 3)
                    self.crown = round(self.crown + dz, 3)
                    self.tell("The housing is X%+.2f Z%+.2f mm from the last measurement: starting the object "
                              "at X%.2f, top Z%.2f" % (dx, dz, self.axis_x, self.crown))
        if p.get("check_only"):
            raise RotaryError("rotary check: no earlier measurement's housing to compare with")
        self._provisional_radius(out.get("reference"))
        if "runout" in steps:
            out["runout"] = []
            for y in p["runout_ys"]:
                out["runout"].append(self.runout(y))
        warnings = self.measured_diameter(out)
        runout_fields, runout_warnings = runout_summary(out.get("runout"), float(p["runout_tolerance"]))
        for w in runout_warnings:
            self.tell("Warning: " + w)
        warnings = warnings + runout_warnings
        if not self.r:
            raise RotaryError("rotary: no diameter: type one, or set rotary_ref_offset so the "
                              "reference step measures it")
        if "stations" in steps:
            out["stations"] = []
            for y in p["stations"]:
                out["stations"].append(self.station(y))
        if not out.get("stations"):
            if "stations" not in steps and (out.get("reference") or {}).get("calibration"):
                # Calibrating on an object of known diameter needs only the reference and the runout.
                out["summary"] = dict(
                    calibration_only=True, diameter=round(2 * self.r, 3),
                    diameter_measured=(out.get("reference") or {}).get("diameter"),
                    diameter_set=p["diameter"], warnings=warnings,
                    runout_max_spread=round(max(r["spread"] for r in out["runout"]), 3) if out.get("runout") else None,
                    **runout_fields)
                out["touches"] = (prev.get("touches") or []) + self.log
                return out
            raise RotaryError("rotary: no stations measured or resumed")
        st = out["stations"]
        if len(st) >= 2:
            self.line = rotary.axis_line([(s["y"], s["xc"], s["zc"]) for s in st])
        else:
            self.line = rotary.axis_line([(st[0]["y"], st[0]["xc"], st[0]["zc"]),
                                          (st[0]["y"] + 1.0, st[0]["xc"], st[0]["zc"])])
        self.axis_x, self.crown = self.line["x0"], self.line["z0"] + self.r
        out["axis"] = dict((k, self.line[k]) for k in ("y0", "x0", "z0", "dx_dy", "dz_dy"))
        ys = sorted(s["y"] for s in st)
        if "free_end" in steps:
            out["free_end"] = self.free_end(ys[0])
        if "shoulder" in steps:
            out["shoulder"] = self.shoulder(ys[-1])
        fe, sh = out.get("free_end"), out.get("shoulder")
        for name, guess, end, found, limit, label in (
                ("the free end", p["free_end_guess"], fe, fe and fe.get("edge") is not None, p["y_min"],
                 "the lowest Y"),
                ("the body near the chuck", p["chuck_end_guess"], sh, sh and sh.get("starts") is not None,
                 self.y_limit, "the never past limit")):
            if not end:
                continue
            if not found:
                w = ("%s was not found: the object still has its full diameter at Y%.0f (%s), so it "
                     "goes on past there. Its end is taken as Y%.0f; move the limit only if nothing "
                     "else is in the way" % (name, float(limit), label, float(limit)))
                warnings = warnings + [w]
                self.tell("Warning: " + w)
                continue
            if guess is None:
                continue
            off = end["full_to"] - float(guess)
            self.tell("%s: full diameter ends at Y%.1f, guessed Y%.1f (%+.1f mm)"
                      % (name[0].upper() + name[1:], end["full_to"], float(guess), off))
            if abs(off) > p["end_warn"]:
                w = ("%s ends at Y%.1f, %.1f mm from the Y%.0f guessed: the measured end is used"
                     % (name, end["full_to"], abs(off), float(guess)))
                warnings = warnings + [w]
                self.tell("Warning: " + w)
        diam = [s["free_diameter"] for s in st]
        out["summary"] = dict(
            centre_at="B0",
            axis_x=round(self.line["x0"], 3), axis_z=round(self.line["z0"], 3), at_y=round(self.line["y0"], 3),
            top_z=round(self.line["z0"] + self.r, 3),
            axis_x_per_100mm=round(100 * self.line["dx_dy"], 3),
            axis_z_per_100mm=round(100 * self.line["dz_dy"], 3),
            diameter=round(2 * self.r, 3),
            diameter_measured=(out.get("reference") or {}).get("diameter"),
            diameter_set=p["diameter"], diameter_free_mean=round(sum(diam) / len(diam), 3),
            warnings=warnings,
            runout_max_spread=round(max(r["spread"] for r in out["runout"]), 3) if out.get("runout") else None,
            body_from_y=fe["full_to"] if fe else None, body_to_y=sh["full_to"] if sh else None,
            body_length=round(sh["full_to"] - fe["full_to"], 3) if fe and sh else None,
            free_end_edge_y=fe["edge"] if fe else None, shoulder_starts_y=sh["starts"] if sh else None,
            free_end_guess=p["free_end_guess"], chuck_end_guess=p["chuck_end_guess"],
            **runout_fields
        )
        out["touches"] = (prev.get("touches") or []) + self.log
        out["complete"] = all(out.get(k) is not None for k in ("runout", "stations", "free_end", "shoulder"))
        return out
