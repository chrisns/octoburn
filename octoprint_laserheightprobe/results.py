# coding=utf-8
"""The numbers the user types into LightBurn, and the calibration behind them.

Three heights matter, all in G-code Z:

- trigger Z: where the probe tripped over a point (probe.Prober.measure);
- focus Z: where the laser head must be for the beam to focus on that point,
  trigger Z + the module's focus offset;
- the focus offset itself: laser focus Z minus the probe's trigger Z, both over the same
  reference spot (the calibration wizard).

The LightBurn number is focus_i - focus_datum. The focus offset cancels out of it, so it is
just trigger_i - trigger_datum, and it is known before any module is calibrated. Positive
means the point is higher than the datum, so the head must go up (G-code Z+). LightBurn's own
docs say a positive cut-setting Z Offset moves the head "inward", towards the material, so the
sign the device needs depends on its Reverse Z setting; `invert` flips the column for that.
"""
from __future__ import absolute_import


def focus_offset(laser_focus_z, z_ref_probe):
    """Calibration: the head height for best focus minus the probe trigger, same spot."""
    if laser_focus_z is None or z_ref_probe is None:
        raise ValueError("the calibration needs both the laser focus Z and the probe's reference Z")
    return round(float(laser_focus_z) - float(z_ref_probe), 4)


def laser_focus_from_touch(touch_z, focal_distance):
    """The other way in: the Z where the laser just touches the spot, plus its focal distance."""
    if touch_z is None or focal_distance is None:
        raise ValueError("give both the laser touch Z and the focal distance")
    return round(float(touch_z) + float(focal_distance), 4)


def focus_z(trigger_z, offset):
    """Where the head goes for focus, or None while the module is uncalibrated."""
    if trigger_z is None or offset is None:
        return None
    return round(float(trigger_z) + float(offset), 4)


def machine_offset_check(z_ref, machine_ref, laser_z, laser_machine, limit=0.05):
    """A warning when G-code Z and machine Z disagree about the offset, else None.

    They differ only if the coordinate system moved between the probe touch and the laser
    focus (a G92 or a work origin set in between), which would make the offset wrong.
    """
    if None in (z_ref, machine_ref, laser_z, laser_machine):
        return None
    by_gcode = float(laser_z) - float(z_ref)
    by_machine = float(laser_machine) - float(machine_ref)
    if abs(by_gcode - by_machine) > limit:
        return ("The G-code Z offset (%.3f) and the machine Z offset (%.3f) differ by %.3f mm: the "
                "work origin changed between the probe touch and the laser focus. Re-run the "
                "calibration without changing the origin." % (by_gcode, by_machine,
                                                             by_gcode - by_machine))
    return None


def table(points, offset=None, reference=None, datum=None, invert=False):
    """Rows for the results table.

    points: dicts from probe.Prober.measure (x, y, z, spread, n, ...), or None for a point
    that failed. reference: index of the bed/reference point, or None. datum: index of the
    LightBurn zero, or None for the first measured point.
    """
    measured = [i for i, p in enumerate(points) if p is not None]
    if datum is None or datum >= len(points) or points[datum] is None:
        datum = measured[0] if measured else None
    if reference is not None and (reference >= len(points) or points[reference] is None):
        reference = None
    z_ref = points[reference]["z"] if reference is not None else None
    z_datum = points[datum]["z"] if datum is not None else None
    sign = -1.0 if invert else 1.0
    rows = []
    for i, p in enumerate(points):
        if p is None:
            rows.append(dict(index=i, failed=True))
            continue
        z = p["z"]
        rows.append(dict(
            index=i, failed=False, x=p["x"], y=p["y"], z=z, spread=p.get("spread"), n=p.get("n"),
            taken=p.get("taken", p.get("n")),
            machine_z=p.get("machine_z"), time=p.get("time"),
            height=round(z - z_ref, 4) if z_ref is not None else None,
            focus=focus_z(z, offset),
            lightburn=round(sign * (z - z_datum), 4) + 0.0 if z_datum is not None else None,
            is_reference=i == reference, is_datum=i == datum,
            square=_square_summary(p.get("square")),
        ))
    return rows


def _square_summary(square):
    """Probe here over a square: the row is its highest point; this says how it was found."""
    if not square:
        return None
    return dict(size=square["size"], n=square["n"], hits=len(square["points"]),
                total=square["n"] * square["n"], range=square["range"])


def _cell(value, digits=3, signed=False):
    if value is None:
        return "-"
    return ("%+.*f" if signed else "%.*f") % (digits, value)


def as_text(rows, module_name=None):
    """A plain table for "Copy as text": numbers only, columns separated by spaces."""
    focus_title = "focus_Z" if not module_name else "focus_Z(%s)" % module_name.replace(" ", "_")
    header = ["#", "X", "Y", "trigger_Z", "spread", "n", "height", focus_title, "LightBurn_Z"]
    lines = [header]
    for row in rows:
        if row.get("failed"):
            lines.append([str(row["index"] + 1), "failed"])
            continue
        mark = "*" if row["is_datum"] else ("r" if row["is_reference"] else "")
        lines.append([
            "%d%s" % (row["index"] + 1, mark), "%.2f" % row["x"], "%.2f" % row["y"],
            _cell(row["z"]), _cell(row["spread"]),
            str(row["n"]) if row["taken"] == row["n"] else "%s/%s" % (row["n"], row["taken"]),
            _cell(row["height"], signed=True), _cell(row["focus"]),
            _cell(row["lightburn"], signed=True),
        ])
    widths = [max(len(line[k]) for line in lines if k < len(line)) for k in range(len(header))]
    out = ["  ".join(cell.rjust(widths[k]) for k, cell in enumerate(line)) for line in lines]
    out.append("* = LightBurn datum (0), r = reference point, n/m = n of m samples kept. "
               "Heights in mm.")
    return "\n".join(out)
