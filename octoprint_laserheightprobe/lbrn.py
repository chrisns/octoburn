# coding=utf-8
"""LightBurn project files (.lbrn2) from a rotary measurement.

The format is LightBurn's own XML, copied from a file LightBurn 2.1.04 saved (FormatVersion 1:
Rect and Ellipse shapes centred by their XForm, paths as VertList / PrimList strings, and the
rotary setup in GantryRotaryConfig). Only tool layers are written (T1 = index 30, T2 = 31):
LightBurn shows them and never outputs them, so the template cannot burn anything.

The canvas is the bridge's (grbl_bridge.RotaryContext): LightBurn X is machine Y from
y_origin (the free end of the body by default), LightBurn Y is millimetres round the object,
and Y 0 is the top of the object at B 0. A touch at B b sits at Y = -b * r (B = -A), and one
off the crown at B 0 sits at Y = -asin(dx / r) * r. Y values are wrapped into 0..circumference.
"""
from __future__ import absolute_import, division

import math
from xml.sax.saxutils import escape

T1, T2 = 30, 31


def _attr(text):
    return escape(str(text), {'"': "&quot;", "\n": "&#10;"})


def tool_layer(index, name):
    return "\n".join([
        '    <CutSetting type="Tool">',
        '        <index Value="%d"/>' % index,
        '        <name Value="%s"/>' % _attr(name),
        '        <priority Value="%d"/>' % (index - T1),
        "    </CutSetting>",
    ])


def rect(cut, x0, y0, x1, y1):
    w, h = abs(x1 - x0), abs(y1 - y0)
    return "\n".join([
        '    <Shape Type="Rect" CutIndex="%d" W="%.4f" H="%.4f" Cr="0">' % (cut, w, h),
        "        <XForm>1 0 0 1 %.4f %.4f</XForm>" % ((x0 + x1) / 2.0, (y0 + y1) / 2.0),
        "    </Shape>",
    ])


def ellipse(cut, x, y, r):
    return "\n".join([
        '    <Shape Type="Ellipse" CutIndex="%d" Rx="%.4f" Ry="%.4f">' % (cut, r, r),
        "        <XForm>1 0 0 1 %.4f %.4f</XForm>" % (x, y),
        "    </Shape>",
    ])


def path(cut, points, closed=False):
    """Straight segments through points (LightBurn's VertList: "V<x> <y>c0x1c1x1" = no handles)."""
    verts = "".join("V%.4f %.4fc0x1c1x1" % p for p in points)
    n = len(points)
    lines = ["L%d %d" % (i, i + 1) for i in range(n - 1)]
    if closed and n > 2:
        lines.append("L%d 0" % (n - 1))
    return "\n".join([
        '    <Shape Type="Path" CutIndex="%d">' % cut,
        "        <XForm>1 0 0 1 0 0</XForm>",
        "        <VertList>%s</VertList>" % verts,
        "        <PrimList>%s</PrimList>" % "".join(lines),
        "    </Shape>",
    ])


def project(shapes, layers, notes, rotary_diameter=None, device=None):
    head = ['<?xml version="1.0" encoding="UTF-8"?>',
            '<LightBurnProject AppVersion="2.1.04"%s FormatVersion="1" MaterialHeight="0" MirrorX="False" '
            'MirrorY="False">' % (' DeviceName="%s"' % _attr(device) if device else "")]
    if rotary_diameter:
        head += ["    <GantryRotaryConfig>",
                 '        <Enabled Value="1"/>',
                 '        <Axis Value="3"/>',
                 '        <Diameter Value="%.3f"/>' % rotary_diameter,
                 '        <StepsPerRotation Value="360"/>',
                 '        <MirrorOutput Value="0"/>',
                 '        <IsChuck Value="1"/>',
                 '        <ObjectDiameter Value="%.3f"/>' % rotary_diameter,
                 "    </GantryRotaryConfig>"]
    return "\n".join(head + list(layers) + list(shapes) + [
        '    <Notes ShowOnLoad="1" Notes="%s"/>' % _attr(notes), "</LightBurnProject>"]) + "\n"


def _wrap(v, circ):
    return v % circ if circ else v


def rotary_template(result, y_origin=None, y_sign=1, module=None):
    """The .lbrn2 text for a rotary result (rotary_job), and a summary dict."""
    s = result.get("summary") or {}
    d = s.get("diameter") or (result.get("params") or {}).get("diameter")
    if not d:
        raise ValueError("the rotary result has no diameter")
    r = d / 2.0
    circ = math.pi * d
    lo, hi = s.get("body_from_y"), s.get("body_to_y")
    if y_origin is None:
        y_origin = lo if lo is not None else 0.0

    def u(y):
        return (y - y_origin) / float(y_sign)

    axis = result.get("axis") or {}

    def axis_x(y):
        if not axis:
            return s.get("axis_x")
        return axis["x0"] + axis["dx_dy"] * (y - axis["y0"])

    shapes = []
    if lo is not None and hi is not None:
        u0, u1 = sorted((u(lo), u(hi)))
        shapes.append(rect(T1, u0, 0.0, u1, circ))                       # usable body x full wrap
        shapes.append(path(T1, [(u0 - 5.0, 0.0), (u1 + 5.0, 0.0)]))       # seam: top at B 0
        chuck = u1 if u(hi) >= u(lo) else u0                              # chuck side arrow
        step = 6.0 if chuck == u1 else -6.0
        shapes.append(path(T1, [(chuck + step, circ / 2.0 - 4.0), (chuck + 2 * step, circ / 2.0),
                                (chuck + step, circ / 2.0 + 4.0)], closed=True))
    touches = 0
    for r_ in result.get("runout") or []:                                 # crown touches at B angles
        for b, _z in r_.get("samples") or []:
            shapes.append(ellipse(T2, u(r_["y"]), _wrap(-math.radians(b) * r, circ), 0.8))
            touches += 1
    for t in result.get("touches") or []:                                 # chord and end touches at B 0
        if t.get("z") is None or t.get("y") is None or t.get("x") is None:
            continue
        ax = axis_x(t["y"])
        if ax is None or abs(t["x"] - ax) >= r:
            continue
        shapes.append(ellipse(T2, u(t["y"]), _wrap(-math.asin((t["x"] - ax) / r) * r, circ), 0.5))
        touches += 1
    notes = ("Rotary template from OctoPrint (OctoBurn).\n"
             "Measured diameter %.2f mm (%s); circumference %.2f mm.\n"
             "Set LightBurn's rotary diameter to %.2f and dia=%.2f in the rotary device's Start GCode.\n"
             "X 0 is machine Y%.2f (%s); Y 0 is the top of the object at B 0 (the T1 line).\n"
             "T1: the usable body%s and the arrow on the chuck side. T2: where the probe touched.\n"
             "T1 and T2 are tool layers: they never burn. Put your art on C layers inside the T1 box."
             % (d, "measured" if s.get("diameter_measured") else "typed in", circ, d, d, y_origin,
                "free end of the body" if y_origin == lo else "given origin",
                " (%.1f mm long)" % abs(hi - lo) if lo is not None and hi is not None else ""))
    text = project(shapes, [tool_layer(T1, "Measured outline"), tool_layer(T2, "Probe touches")], notes,
                   rotary_diameter=d, device=("Snapmaker %s rotary via OctoPrint" % module) if module else None)
    return text, dict(diameter=round(d, 3), circumference=round(circ, 3), y_origin=y_origin, touches=touches,
                      body_length=round(abs(hi - lo), 3) if lo is not None and hi is not None else None)
