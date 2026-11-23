# coding=utf-8
"""Checks and numbers behind the probe's calibration. Pure: no OctoPrint, no I/O, nothing sent.

Heights are G-code Z with no G92 or work offset in force (the plugin refuses one), so a
probe trigger Z and a laser Z taken at the same spot compare directly.

The focus offset (focus Z = trigger Z + offset) from geometry, 10W module:

    pin tip, deployed     24.00 below the QS plate's bottom edge (the head's scad)
    10W hood bottom       19.75 below it (Snapmaker STEP)
    10W focus             10.00 below the hood (Snapmaker's touchscreen: touch Z + 10.0)

The focus point is 29.75 below the plate edge and the pin tip 24.00, so with the tip on the
surface the head must rise 5.75 to focus there. But the probe trips only after the surface has
pushed the pin up by its pre-travel p, so the head is p lower than first contact at the trip:

    offset = 5.75 + p          (hood above the surface at focus Z = offset - p + 4.25 = 10)

A measured offset therefore says what p is, if the CAD and the 10.0 are right; a p well
outside 0 to 2.5 mm means one of them is not. See `cad_check`.

The probe steps down and reports the first commanded Z that latched, so the true trip is
somewhere in the step above it: reported Z is low by step / 2 on average (0.01 mm at the
0.02 mm fine step, 0.05 at 0.1). The offset and every later point share that bias as long
as they use the same fine step, so it cancels; mixing presets shifts focus by the difference
of the half-steps, far inside the 10W's focus depth (about +-1 mm).
"""
from __future__ import absolute_import, division

import math

# 10W geometry (see the module docstring).
TIP_BELOW_EDGE = 24.0
HOOD_BELOW_EDGE_10W = 19.75
FOCAL_10W = 10.0
CAD_OFFSET_10W = round(HOOD_BELOW_EDGE_10W + FOCAL_10W - TIP_BELOW_EDGE, 4)   # 5.75
HOOD_ABOVE_TIP = TIP_BELOW_EDGE - HOOD_BELOW_EDGE_10W                       # 4.25

def hood_touch_z(flat_bed_z, focus_offset):
    """The head Z at which the 10W's hood meets the bare bed: focus Z (Probe-here trigger Z on
    the bed + the module's focus offset) less the hood's height above the surface at focus."""
    return flat_bed_z + focus_offset - FOCAL_10W


# Implied pre-travel outside this range: warn (never refuse), the CAD is only good to +-1.5 mm.
PRETRAVEL_PLAUSIBLE = (-0.5, 2.5)
# The closing re-touch against the reference: noise below WARN, a lost Z above REHOME.
RETOUCH_WARN = 0.05
RETOUCH_REHOME = 0.5
RETOUCH_XY = 1.0
# A forced repeat run: whole N touches, with a tolerance loose enough that none is rejected.
FORCED_RANGE = (3, 10)
FORCED_TOLERANCE = 1.0
CARD_MAX = 2.0


class CalibrationError(ValueError):
    pass


def _num(value, what):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise CalibrationError("%s must be a number (got %r)" % (what, value))
    if math.isnan(number) or math.isinf(number):
        raise CalibrationError("%s must be a finite number" % what)
    return number


# -- the focus offset ---------------------------------------------------------------------

def offset_from_card(drag_z, card_thickness, z_ref, focal=FOCAL_10W):
    """The focus offset from a card that just drags under the laser's hood.

    With a card of thickness t under the hood, the hood is t above the surface, not on it,
    so the hood would touch the surface at drag_z - t and focus is focal above that:
    focus Z = drag_z - t + focal; offset = focus Z - the probe's trigger Z at the same spot.
    """
    drag_z = _num(drag_z, "the Z where the card drags")
    t = _num(card_thickness, "the card thickness")
    if not 0.0 <= t <= CARD_MAX:
        raise CalibrationError("the card thickness must be 0 to %g mm (got %g): 0 if the hood "
                               "touches the bare surface" % (CARD_MAX, t))
    focal = _num(focal, "the focal distance")
    z_ref = _num(z_ref, "the probe's reference Z")
    touch = round(drag_z - t, 4)
    focus = round(touch + focal, 4)
    return dict(touch_z=touch, focus_z=focus, offset=round(focus - z_ref, 4))


def cad_check(offset, cad_offset=CAD_OFFSET_10W):
    """What a measured 10W offset says against the CAD, and a warning when they disagree.

    implied_pretravel = offset - 5.75 (see the module docstring). Positive is expected: the
    pin trips after it is pushed up. A warning, never a refusal: the CAD figure is only good to
    about +-1.5 mm (plate seating, a STEP vs drawing difference on the hood height).
    """
    offset = _num(offset, "the focus offset")
    p = round(offset - cad_offset, 4)
    low, high = PRETRAVEL_PLAUSIBLE
    warning = None
    if offset <= 0:
        warning = ("A focus offset of %+.3f mm puts focus at or below the probe's trigger: the "
                   "10W hood would be in the work. Check the sign: focus Z is ABOVE trigger Z."
                   % offset)
    elif not low <= p <= high:
        warning = ("The offset %+.3f mm is %.2f mm from the CAD %+.2f, an implied pin pre-travel "
                   "of %+.2f mm (expected %g to %g). Re-check the card thickness and the reference "
                   "touch, then confirm with a focus ladder." % (offset, abs(p), cad_offset, p, low, high))
    return dict(offset=offset, cad_offset=cad_offset, implied_pretravel=p, warning=warning)


def retouch_check(reference, retouch, warn=RETOUCH_WARN, rehome=RETOUCH_REHOME, xy=RETOUCH_XY):
    """The probe swapped back in and touched the reference spot again: did anything move?

    reference, retouch: dicts with z and, if known, machine_z, x, y. Returns delta (retouch -
    reference, G-code Z), level "ok" | "warn" | "rehome" | "origin" | "moved", and a message.

    - G-code and machine deltas disagree: the work origin changed in between ("origin").
    - The re-touch is not over the same spot ("moved"): the comparison means nothing.
    - |delta| above `rehome`: Z steps were lost (on 2026-11-17 a stall lost 27.67 mm and only
      a re-home found it); re-home with G28 and redo the calibration.
    - |delta| above `warn`: the head seats differently after a swap, or the surface moved;
      that much uncertainty carries into the offset.
    """
    z0, z1 = _num(reference.get("z"), "the reference Z"), _num(retouch.get("z"), "the re-touch Z")
    delta = round(z1 - z0, 4)
    out = dict(delta=delta, machine_delta=None)
    if None not in (reference.get("x"), reference.get("y"), retouch.get("x"), retouch.get("y")):
        off = math.hypot(retouch["x"] - reference["x"], retouch["y"] - reference["y"])
        if off > xy:
            out.update(level="moved", message="The re-touch is %.2f mm from the reference spot: probe "
                       "the same X, Y (within %g mm) or the heights do not compare." % (off, xy))
            return out
    m0, m1 = reference.get("machine_z"), retouch.get("machine_z")
    if m0 is not None and m1 is not None:
        out["machine_delta"] = round(m1 - m0, 4)
        if abs(out["machine_delta"] - delta) > 0.05:
            out.update(level="origin", message="G-code Z moved %+.3f but machine Z moved %+.3f: the work "
                       "origin changed between the touches. Home (G28) without a G92 and redo the "
                       "calibration." % (delta, out["machine_delta"]))
            return out
    if abs(delta) > rehome:
        out.update(level="rehome", message="The re-touch is %+.3f mm from the reference: Z steps were "
                   "probably lost (a stall). Re-home with G28 X Y Z and redo the calibration from "
                   "step 1." % delta)
    elif abs(delta) > warn:
        out.update(level="warn", message="The re-touch is %+.3f mm from the reference (more than %g): "
                   "the head seats differently after a swap, or the surface moved. The offset is no "
                   "better than that; repeat the swap to see which." % (delta, warn))
    else:
        out.update(level="ok", message="The re-touch agrees with the reference within %.3f mm."
                   % abs(delta))
    return out


def quantisation_bias(step):
    """Reported trigger Z is low by this much on average (the trip is within the step above)."""
    return round(_num(step, "the step") / 2.0, 4)


# -- repeatability ------------------------------------------------------------------------

def forced_samples(n):
    """Settings overrides for a Probe here that takes exactly n fine touches and keeps them all.

    The probe stops once `samples` touches agree within `tolerance`; asking for n of n within
    1 mm makes it take all n (a spread over 1 mm fails the run, which a sound probe never
    does). The result's `samples` list then holds every touch, for `stats`.
    """
    try:
        n = int(n)
    except (TypeError, ValueError):
        raise CalibrationError("the repeat count must be a whole number")
    low, high = FORCED_RANGE
    if not low <= n <= high:
        raise CalibrationError("the repeat count must be %d to %d (got %d)" % (low, high, n))
    return dict(samples=n, max_samples=n, tolerance=FORCED_TOLERANCE)


def stats(values):
    """n, mean, sample SD, range, 2 SD and the per-touch trend of a list of heights."""
    values = [_num(v, "a height") for v in values]
    n = len(values)
    if n == 0:
        raise CalibrationError("no heights to summarise")
    mean = sum(values) / n
    sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1)) if n > 1 else None
    slope = None
    if n > 2:
        # least-squares slope against the touch number: a drifting pin or a warming machine
        xm = (n - 1) / 2.0
        slope = sum((i - xm) * (v - mean) for i, v in enumerate(values)) / sum((i - xm) ** 2 for i in range(n))
    return dict(n=n, mean=round(mean, 4), sd=None if sd is None else round(sd, 4),
                two_sd=None if sd is None else round(2 * sd, 4),
                range=round(max(values) - min(values), 4), min=min(values), max=max(values),
                slope=None if slope is None else round(slope, 5))


def point_heights(point):
    """Every fine touch a Probe here result took (rejected ones too), or its one height."""
    samples = point.get("samples")
    return list(samples) if samples else [point["z"]]


def swap_repeatability(points):
    """Each point is one seating of the probe head over the same spot (swap it out and back in
    between). within: pooled SD of the touches inside each seating; between: SD of the seatings'
    means. between much larger than within means the head seats differently each time."""
    points = [p for p in points if p]
    if len(points) < 2:
        raise CalibrationError("swap repeatability needs at least two Probe here results, one per "
                               "seating of the head")
    groups = [point_heights(p) for p in points]
    means = [sum(g) / len(g) for g in groups]
    num = sum(sum((v - m) ** 2 for v in g) for g, m in zip(groups, means))
    dof = sum(len(g) - 1 for g in groups)
    within = math.sqrt(num / dof) if dof > 0 else None
    between = stats(means)
    return dict(seatings=len(groups), touches=sum(len(g) for g in groups),
                within_sd=None if within is None else round(within, 4),
                between_sd=between["sd"], between_range=between["range"], mean=between["mean"])


def report(points, reference=None):
    """What the UI shows under the calibration: the last result's repeatability, the swap
    repeatability over every result near the reference spot, and the re-touch check."""
    points = [p for p in (points or []) if p]
    out = dict(last=None, swaps=None, retouch=None)
    if points:
        out["last"] = stats(point_heights(points[-1]))
    if reference and reference.get("z") is not None:
        near = [p for p in points if reference.get("x") is None or
                math.hypot(p["x"] - reference["x"], p["y"] - reference["y"]) <= RETOUCH_XY]
        if len(near) >= 2:
            out["swaps"] = swap_repeatability(near)
        # The re-touch is a result taken after the reference was set (a reference taken from
        # a Probe here result would otherwise be compared with itself).
        after = [p for p in near if reference.get("time") is None or
                 (p.get("time") is not None and p["time"] > reference["time"])]
        if after:
            out["retouch"] = retouch_check(reference, after[-1])
    return out


# -- the focus ladder ---------------------------------------------------------------------
#
# A G-code FILE for a person to check and burn; the plugin never sends it. Short lines along X,
# one per Z, stacked in Y. The beam is off (M5) before every Z change, so it never fires while
# Z moves. Power is inline on each G1 after one "M3 S0" (rotary_gcode's convention: M3 waits for
# the planner). The sharpest, thinnest line is focus: offset = its Z - the probe's trigger Z.

PRETRAVEL_MAX = 2.0           # assumed worst case when checking hood clearance
HOOD_CLEARANCE_MIN = 2.0


def ladder_rows(z_ref, offset_guess=CAD_OFFSET_10W, step=0.25, count=13, length=10.0):
    """[(k, z, length)]: k = -(count // 2)..+(count // 2), z = z_ref + offset_guess + k * step.
    The middle line is 1.6x long and every 4th from it 1.3x, so lines can be counted."""
    z_ref, guess = _num(z_ref, "the probe's reference Z"), _num(offset_guess, "the offset guess")
    step, length = _num(step, "the ladder step"), _num(length, "the line length")
    count = int(count)
    if count < 3 or count > 41 or count % 2 == 0:
        raise CalibrationError("the ladder needs an odd count of 3 to 41 lines (got %d)" % count)
    if not 0.05 <= step <= 1.0:
        raise CalibrationError("the ladder step must be 0.05 to 1.0 mm (got %g)" % step)
    if not 2.0 <= length <= 40.0:
        raise CalibrationError("the line length must be 2 to 40 mm (got %g)" % length)
    half = count // 2
    rows = []
    for k in range(half, -half - 1, -1):           # highest first: Z only ever goes down
        scale = 1.6 if k == 0 else (1.3 if k % 4 == 0 else 1.0)
        rows.append((k, round(z_ref + guess + k * step, 3), round(length * scale, 3)))
    return rows


WORK_OFFSET_TOLERANCE = 0.01


def check_work_offset(work_offset):
    """Refuse unless the G-code frame was the homed one when the probe values were read.

    work_offset: gcode.work_offset() of an M503 + M114 reply taken with the probe values, a
    (dx, dy, dz) that is zero with no G92 / work origin in force (M206 is added back, so the
    laser mode home offset does not count), or None if it could not be read. Measured
    2026-11-17: G54 on this machine holds a stale offset of about X-147.5 Y-153.5 Z-245.1 from
    old LightBurn flat jobs, so values read in G54 would put a G53 ladder 245 mm off in Z.
    """
    if work_offset is None:
        raise CalibrationError("the work offset could not be read (M503 + M114), so the probe "
                               "values may not be in the G53 frame: home with G28 X Y Z, select "
                               "G53 and probe again")
    try:
        dx, dy, dz = (float(v) for v in work_offset)
    except (TypeError, ValueError):
        raise CalibrationError("the work offset must be (dx, dy, dz), got %r" % (work_offset,))
    if max(abs(dx), abs(dy), abs(dz)) > WORK_OFFSET_TOLERANCE:
        raise CalibrationError("a work offset is in force (X%+.3f Y%+.3f Z%+.3f): the probe values "
                               "are not G53 coordinates. Select G53 (or home with G28 X Y Z and no "
                               "G92), probe again, and generate the ladder from those values"
                               % (dx, dy, dz))


def focus_ladder(x, y, z_ref, work_offset, offset_guess=CAD_OFFSET_10W, step=0.25, count=13,
                 length=10.0, pitch=1.5, power=30, feed=1500, travel_feed=3000, travel_z=None,
                 module="10W"):
    """The ladder's G-code lines under G53. x, y: where the first line starts (head XY over
    the spot probed for z_ref). x, y, z_ref are M114 values (Probe here's x, y, z, not the step
    counter's machine_z): with no G92 or work offset in force and no laser mesh loaded, G53
    lines land in that frame: measured 2026-11-17, this firmware's G53 drops the work offset
    and keeps M206, and M114 Z equals the step counter Z (M206 Z0). `work_offset` is required
    (see check_work_offset): the caller proves the frame, this module stays pure. Inline S power
    after M3 S0 is unproven until the first ladder burn. Refuses a ladder that would bring the hood within
    2 mm of the surface even with a 2 mm pin pre-travel."""
    check_work_offset(work_offset)
    x, y = _num(x, "X"), _num(y, "Y")
    pitch, power = _num(pitch, "the line pitch"), _num(power, "the power")
    feed, travel_feed = _num(feed, "the feed"), _num(travel_feed, "the travel feed")
    if not 0.5 <= pitch <= 10.0:
        raise CalibrationError("the line pitch must be 0.5 to 10 mm (got %g)" % pitch)
    if not 1 <= power <= 255:
        raise CalibrationError("the power must be S1 to S255 (got %g)" % power)
    if not 100 <= feed <= 6000 or not 300 <= travel_feed <= 6000:
        raise CalibrationError("feeds must be 100 to 6000 (burn) and 300 to 6000 (travel) mm/min")
    rows = ladder_rows(z_ref, offset_guess, step, count, length)
    z_low = min(z for _, z, _ in rows)
    z_high = max(z for _, z, _ in rows)
    clearance = (z_low - float(z_ref)) + HOOD_ABOVE_TIP - PRETRAVEL_MAX
    if module == "10W" and clearance < HOOD_CLEARANCE_MIN:
        raise CalibrationError("the lowest line (Z%.3f) would leave the 10W hood only %.2f mm above "
                               "the surface with a %g mm pin pre-travel: raise the offset guess or "
                               "shorten the ladder" % (z_low, clearance, PRETRAVEL_MAX))
    travel = round(z_high + 5.0, 3) if travel_z is None else _num(travel_z, "the travel Z")
    if travel < z_high + 1.0:
        raise CalibrationError("travel Z (%.3f) must be at least 1 mm above the top line (Z%.3f)"
                               % (travel, z_high))
    s = int(round(power))
    out = ["; Focus ladder, %s: %d lines, Z%.3f to Z%.3f in %g mm steps (trigger Z%.3f + %+.3f + k * %g)"
           % (module, len(rows), z_high, z_low, float(step), float(z_ref), float(offset_guess), float(step)),
           "; k counts from the long middle line (k=0); every 4th line from it is a little longer.",
           "; Highest Z first, the first line is furthest -Y. Pick the thinnest, sharpest line:",
           "; focus offset = its Z - Z%.3f. CHECK THIS FILE AND THE MODULE BEFORE BURNING." % float(z_ref),
           "; Probe values read with no work offset (checked): G53 lines are in the same frame.",
           "M5", "G53 ; machine coordinates: no G92 offset can move this job", "G90", "G21", "M211 S1",
           "G0 Z%.3f F1200" % travel]
    for i, (k, z, l) in enumerate(rows):
        ly = round(y + i * pitch, 3)
        out += ["; k=%+d Z%.3f" % (k, z),
                "M5",
                "G0 X%.3f Y%.3f F%d" % (x, ly, int(travel_feed)),
                "G0 Z%.3f F600" % z,
                "M3 S0 ; beam on at zero: power is inline on the G1",
                "G1 X%.3f Y%.3f S%d F%d" % (x + l, ly, s, int(feed)),
                "M5"]
    out += ["M5", "G0 Z%.3f F1200" % travel]
    return out


def ladder_offset(z_ref, best_z):
    """The focus offset from the ladder: the best line's Z (or a Z between two lines) minus the
    probe's trigger Z at the same spot."""
    return round(_num(best_z, "the best line's Z") - _num(z_ref, "the probe's reference Z"), 4)
