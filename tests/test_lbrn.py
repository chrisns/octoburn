"""LightBurn rotary templates: valid XML, tool layers only, on the bridge's canvas."""
import math
import xml.etree.ElementTree as ET

import pytest

from lhp_pkg import lbrn


def result(diameter=73.48, measured=True):
    return dict(
        params=dict(diameter=None),
        summary=dict(diameter=diameter, diameter_measured=diameter if measured else None,
                     body_from_y=30.0, body_to_y=210.0, axis_x=164.3),
        axis=dict(y0=120.0, x0=164.3, z0=105.0, dx_dy=0.0, dz_dy=0.0),
        runout=[dict(y=60.0, samples=[[0.0, 141.4], [90.0, 141.3], [180.0, 141.1], [270.0, 141.3]])],
        touches=[dict(x=164.3, y=120.0, z=141.7), dict(x=164.3 + 36.74 * math.sin(math.radians(30)), y=120.0,
                                                      z=140.0), dict(x=None, y=1.0, z=None)])


def test_the_template_is_lightburn_xml_with_tool_layers_and_the_rotary_diameter():
    text, info = lbrn.rotary_template(result(), module="10W")
    root = ET.fromstring(text)
    assert root.tag == "LightBurnProject" and root.get("FormatVersion") == "1"
    assert root.get("DeviceName") == "Snapmaker 10W rotary via OctoPrint"
    assert root.find("GantryRotaryConfig/Diameter").get("Value") == "73.480"
    cuts = root.findall("CutSetting")
    assert [c.get("type") for c in cuts] == ["Tool", "Tool"]
    assert {int(s.get("CutIndex")) for s in root.findall("Shape")} == {lbrn.T1, lbrn.T2}
    body = root.find("Shape[@Type='Rect']")
    circ = math.pi * 73.48
    assert float(body.get("W")) == pytest.approx(180.0) and float(body.get("H")) == pytest.approx(circ, abs=1e-3)
    assert body.find("XForm").text.split()[4:] == ["90.0000", "%.4f" % (circ / 2)]
    assert info["touches"] == 6 and info["body_length"] == 180.0
    assert "dia=73.48" in root.find("Notes").get("Notes")


def test_touches_sit_round_the_object_where_the_bridge_puts_them():
    text, _ = lbrn.rotary_template(result())
    root = ET.fromstring(text)
    spots = [tuple(float(v) for v in e.find("XForm").text.split()[4:]) for e in root.findall("Shape[@Type='Ellipse']")]
    r, circ = 73.48 / 2, math.pi * 73.48
    assert (30.0, 0.0) in [(round(x, 3), round(y, 3)) for x, y in spots]                 # Y60 B0: X 30, top
    b90 = [y for x, y in spots if round(x, 3) == 30.0][1]
    assert b90 == pytest.approx(circ - math.radians(90) * r, abs=1e-3)                   # B90 -> A -90
    off = [y for x, y in spots if round(x, 3) == 90.0 and y > 0.1][0]
    assert off == pytest.approx(circ - math.radians(30) * r, abs=0.05)                   # +X of crown


def test_no_diameter_no_template():
    with pytest.raises(ValueError):
        lbrn.rotary_template(dict(summary={}, params={}))
