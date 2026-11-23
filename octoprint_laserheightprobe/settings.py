# coding=utf-8
"""Every setting the plugin reads, and the checks that keep them safe to send.

This lives apart from the plugin class so the tests can check it without OctoPrint.

Lists are stored as text, as in NozzleTouch, because OctoPrint's settings pane edits text
well and nested lists badly. Each has a parser that gives the structured form, and that form
is what the API, the routine and the results JSON use:

- points: one "X Y" per line, optionally followed by "ref" (the bed/reference point) or
  "datum" (the LightBurn zero), e.g. "120 80 ref";
- stages: "step, start_above" separated by ";" or new lines, coarse first. The first stage
  starts at safe_z, so its start_above may be left out;
- laser modules: one "name = focus offset, dx, dy" per line, any field left blank until it
  is calibrated. dx, dy is the probe pin's position minus that module's beam position: the
  modules' beams sit in different places, so the offset belongs to the module. Parsed:
  [{"name": "10W", "focus_offset": None, "dx": 0.0, "dy": 0.0}, ...].

safe_z and z_floor have no default. The laser-mode Z frame on this machine was never measured,
so any number here would be invented. The user captures both from the machine (jog, then "Use
current Z"). The automatic job needs both. Probe here (the hand workflow) needs neither: it
starts where the user put the head, and uses z_floor only if it is set.

Every number here ends up in a G-code command or decides when the head stops going down.
The checks run on save and again before every job.
"""
from __future__ import absolute_import

import re


class SettingsError(ValueError):
    """A setting was refused before it could reach the machine."""


# The pin was designed onto the 10W's beam axis, so 10W starts at 0, 0, but the printed plate
# put it about 3 mm off (+2.0, -2.5 measured 2026-11-18): measure it with Pin on dot / Light on
# dot. The IR beam sits elsewhere and is unmeasured: null, which the job reports as a warning.
DEFAULT_MODULES = "10W = , 0, 0\n2W IR ="

DEFAULTS = dict(
    esp_url="http://laserprobe.local",
    http_timeout=2.0,
    safe_z=None,
    z_floor=None,
    max_descent=80.0,
    here_max_descent=10.0,
    # Probe here over a square: side and points per side. "Highest" is what it reports.
    here_square_size=3.0,
    here_square_n=3,
    # The clone pin overshoots as it deploys: a stage that starts 0.4 mm above the last trigger
    # trips during the deploy (2026-11-17), so the fine stage starts 1.0 above.
    stages="0.5; 0.1, 1.5; 0.02, 1.0",
    samples=2,
    tolerance=0.05,
    max_samples=4,
    probe_feed=300,
    z_feed=600,
    xy_feed=3000,
    arm_wait_s=1.0,
    home_first=True,
    # The first arm of every point resets the probe and proves the pin position with touch mode
    # (added 2026-11-19 after descents that never tripped). Off skips that check.
    verify_pin=True,
    tap_test=True,
    lightburn_invert=False,
    laser_modules=DEFAULT_MODULES,
    points="",
    selected_module="10W",
    # The calibration's reference touch. Flat scalars, so OctoPrint stores them plainly.
    calib_x=None,
    calib_y=None,
    calib_z_ref=None,
    calib_machine_z=None,
    calib_time=None,
    # Flat bed: a Probe here trigger Z on the bare bed ("Use as bed"), so a Probe here can start
    # from an object's height above it, like the rotary's known diameter.
    flat_bed_z=None,
    # Height map: how far the firmware mesh may miss a probed height before the plugin warns,
    # and the selected laser's gap between its lowest part and its focus (unset: unknown).
    map_tolerance=0.5,
    map_focus_gap=None,
    # LightBurn bridge: OctoPrint answers as a GRBL laser on this TCP port, to these addresses
    # only. Flat jobs: machine X, Y = LightBurn X, Y + the beam offset (the old start code's
    # "G0 X2 Y0 / G92 X0 Y0"). The laser arm lasts this long. Moves are split this short.
    bridge_enabled=False,
    bridge_port=2323,
    bridge_allow="127.0.0.1",
    bridge_offset_x=2.0,
    bridge_offset_y=0.0,
    bridge_backlash_x=0.52,
    focus_after_frame=False,
    auto_mode=True,             # set flat or rotary mode by itself, from whether the rotary is plugged in
    flat_focus_height=0.0,      # the automatic flat mode's material thickness (the last one used)    # after each LightBurn Frame in flat mode, focus there with the red dot     # mm short the head stops moving +X (split-line burns 2026-11-22)
    bridge_arm_minutes=30,
    bridge_seg_mm=5.0,
    # The pointer (LightBurn's Fire button and laser-on framing, without an arm): a beam
    # pointer is capped at this S (Luban frames at S10 of 255); it switches off after this long.
    pointer_s_max=10,
    pointer_minutes=5,
    # Rotary diameter from the motor housing (rotary_job reference step): the housing touch
    # minus the spin axis height, and how much the axis rises per mm of Y. Measured on the A350
    # on 2026-11-18 from the 78 mm flask; a run with known_diameter measures them again.
    rotary_ref_offset=28.678,
    rotary_ref_slope=0.002321,
    # The housing touch the calibration saw: where the next run looks for the housing, and a
    # check that the probe still sits on the head as it did (a run far from it warns).
    rotary_ref_housing_z=None,
    # The nearest the head may go to the rotary body (head Y, any module): objects can sit over
    # the chuck, so this is the rotary's own limit. Chris jogged the head there on 2026-11-19.
    rotary_head_y_limit=293.4,
    # The 10W's head camera, through the touchscreen's HTTP server (empty: off), and where the
    # head goes for the photo: Luban's A350 spot, X232 Y178 Z290 (above anything on the bed or
    # the rotary). photoQuality is JPEG compression, smaller = finer: 1 is 460 KB and 12 s a photo
    # (the touchscreen pulls it over Bluetooth slowly), 8 is 108 KB and 3.9 s with the same cross
    # position to 0.02 px and no visible loss (measured 2026-11-21); 20 starts to look blocky.
    headcam_url="",
    headcam_x=232.0,
    headcam_y=178.0,
    headcam_z=290.0,
    headcam_quality=8,
    # Close-up camera (closeup_plan.py, compositor.py). The camera is sharp from about 40 to
    # 60 mm above a surface (Tests B3-B5): stand-off is the head Z above the object's top.
    closeup_standoff=50.0,
    closeup_overlap=0.25,
    closeup_min_clearance=10.0,     # the 10W hood's least gap over the object
    closeup_max_height=60.0,
    closeup_hood_radius=5.0,        # margin added round the 10W's outline (from its CAD)
    closeup_dpm=15.0,               # px/mm of the image LightBurn gets (native at Z50 is ~14.8)
    closeup_max_mp=45.0,            # LightBurn took 6400 x 7000 (B6)
    closeup_max_tiles=25,
    closeup_feather_mm=2.0,
    closeup_jpeg_quality=90,
    closeup_stream_s=2.0,
    closeup_capture_s=6.0,          # one photo at photoQuality 8 (3.9 s; it was 25 s at quality 1, B8)
    closeup_approach_mm=2.0,        # every photo arrives from -X, -Y: X backlash is ~0.42 mm
    closeup_fresh_overview=True,    # each close-up takes its own overview first (the bed as it is now)
    burn_closeup_after=True,        # a close-up of the burned area once a LightBurn burn has finished
    closeup_repeat_minutes=30,      # M240 P1 repeats a close-up request only this fresh
    closeup_shadow_frac=0.4,        # the right part of a photo the head may shade (room light, 2026-11-21)
    # The close-up image's base (the whole-bed picture the tiles are blended over): "headcam",
    # the 10W's overview from X232 Y178 Z290, or "usb", the fixed USB bed camera (bedcam.py).
    # The bed moves in Y under that camera, so for its photo the head parks at bedcam_park_*
    # (out of its view, the bed at the Y the camera was calibrated at; LightBurn's own "Bed cam -
    # fixed" was aligned at Y300). The calibration is closeup/bedcam.json (bedcam_calibrate).
    closeup_base="headcam",
    bedcam_url="",
    bedcam_park_x=0.0,
    bedcam_park_y=300.0,
    bedcam_park_z=200.0,
    # Air purifier (purifier.py, air.py). The fan is M1011 F0-3; its socket and the air sensor
    # are in Home Assistant, which needs a long-lived token (create one for the plugin).
    # No token: no socket and no readings, the fan alone on a timer from the dose.
    purifier_enabled=True,
    purifier_ha_url="",
    purifier_ha_token="",
    purifier_switch="",
    purifier_pm25="",                        # a PM2.5 sensor inside the enclosure, e.g. a PMS5003ST
    purifier_hcho="",
    purifier_start_j=300.0,         # laser energy before the purifier starts (paper marks are ~10 J)
    purifier_runon_min_s=60.0,
    purifier_runon_max_s=1200.0,
    purifier_pm25_margin=150.0,     # PM2.5 over the pre-job air that is bearable: not choking, no fire alarm
    purifier_hcho_margin=30.0,
    purifier_alarm_pm25=300.0,      # this much smoke starts it whatever the dose
    purifier_tau_s=None,            # the smoke's decay time with the purifier on, learnt from run-ons
    purifier_enclosure_fan=True,    # the enclosure fan blows into the purifier's hose: run it with it
    # The serial link guard (linkguard.py): a burn line whose "ok" never comes. After link_ok_s the
    # lost ok is faked; a second loss, or a silent printer, stops the burn; no real ok link_power_s
    # after that cuts the power with emergency_power_switch (a Home Assistant switch on the
    # printer's mains, through purifier_ha_url and its token). Empty: no power cut.
    link_ok_s=2.0,
    link_power_s=3.0,
    emergency_power_switch="",
)

# (low, high) for each number.
RANGES = dict(
    link_ok_s=(0.5, 10.0),
    link_power_s=(1.0, 30.0),
    http_timeout=(0.2, 10.0),
    # Wide on purpose: the laser-mode Z frame is the machine's, captured not assumed. Z 0 is
    # the bottom (soft endstops clamp there); safe_z > z_floor is checked separately.
    safe_z=(0.0, 400.0),
    z_floor=(0.0, 400.0),
    max_descent=(1.0, 330.0),
    # Probe here starts wherever the user jogged to, so its window is relative to that.
    here_max_descent=(0.5, 100.0),
    here_square_size=(0.5, 50.0),
    here_square_n=(2, 9),
    samples=(1, 10),
    tolerance=(0.001, 1.0),
    max_samples=(1, 10),
    probe_feed=(10, 1200),
    z_feed=(60, 3000),
    xy_feed=(100, 12000),
    # The ESP holds the deploy pulse for 680 ms; reading sooner reads a pin still moving.
    arm_wait_s=(0.7, 5.0),
    map_tolerance=(0.02, 5.0),
    map_focus_gap=(1.0, 200.0),
    bridge_port=(1024, 65535),
    bridge_offset_x=(-60.0, 60.0),
    bridge_offset_y=(-60.0, 60.0),
    bridge_backlash_x=(0.0, 2.0),
    flat_focus_height=(0.0, 150.0),
    bridge_arm_minutes=(1, 240),
    bridge_seg_mm=(1.0, 20.0),
    pointer_s_max=(1, 25),
    pointer_minutes=(1, 30),
    rotary_ref_offset=(5.0, 100.0),
    rotary_ref_slope=(-0.02, 0.02),
    rotary_ref_housing_z=(0.0, 400.0),
    rotary_head_y_limit=(100.0, 330.0),
    flat_bed_z=(0.0, 400.0),
    headcam_x=(0.0, 330.0),
    headcam_y=(0.0, 350.0),
    # Above the rotary's chuck and any object in it (its top is about Z145), below the Z home.
    headcam_z=(200.0, 310.0),
    headcam_quality=(0, 255),
    closeup_standoff=(35.0, 150.0),
    closeup_overlap=(0.1, 0.5),
    closeup_min_clearance=(5.0, 50.0),
    closeup_max_height=(0.0, 150.0),
    closeup_hood_radius=(0.0, 50.0),
    closeup_dpm=(2.0, 20.0),
    closeup_max_mp=(1.0, 45.0),
    closeup_max_tiles=(1, 64),
    closeup_feather_mm=(0.0, 10.0),
    closeup_jpeg_quality=(50, 98),
    closeup_stream_s=(0.5, 10.0),
    closeup_capture_s=(2.0, 60.0),
    closeup_approach_mm=(0.0, 5.0),
    closeup_shadow_frac=(0.0, 0.6),
    closeup_repeat_minutes=(1, 240),
    bedcam_park_x=(0.0, 330.0),
    bedcam_park_y=(0.0, 350.0),
    bedcam_park_z=(200.0, 310.0),        # never below the close-up's travel floor (closeup_plan.TRAVEL_FLOOR)
    closeup_height=(0.0, 150.0),         # an API argument (the object height), not a setting
    purifier_start_j=(0.0, 100000.0),
    purifier_runon_min_s=(0.0, 1800.0),
    purifier_runon_max_s=(30.0, 3600.0),
    purifier_pm25_margin=(1.0, 500.0),
    purifier_hcho_margin=(1.0, 500.0),
    purifier_alarm_pm25=(10.0, 2000.0),
    purifier_tau_s=(10.0, 7200.0),
)
HEIGHTS = ("safe_z", "z_floor")              # in RANGES, but with no default
OPTIONAL = ("map_focus_gap", "flat_bed_z", "rotary_ref_offset", "rotary_ref_slope", "rotary_ref_housing_z",
            "closeup_height", "purifier_tau_s")  # in RANGES, may stay unset
XY_OFFSET_LIMIT = 60.0
INTEGERS = ("samples", "max_samples", "probe_feed", "z_feed", "xy_feed", "here_square_n",
            "bridge_port", "bridge_arm_minutes", "pointer_s_max", "pointer_minutes", "headcam_quality",
            "closeup_max_tiles", "closeup_jpeg_quality", "closeup_repeat_minutes")
BOOLEANS = ("home_first", "focus_after_frame", "auto_mode", "lightburn_invert", "tap_test", "bridge_enabled", "verify_pin", "closeup_fresh_overview",
            "purifier_enabled", "purifier_enclosure_fan", "burn_closeup_after")

CLOSEUP_BASES = ("headcam", "usb")          # where the close-up image's base comes from

# A loose sanity box only. Positions are G-code coordinates, which a work origin can shift,
# so the real check is the M114 read-back after each move (probe.Prober._verify).
XY_LIMIT = 400.0
MAX_POINTS = 200
MAX_STAGES = 4
OFFSET_LIMIT = 100.0

_NUMBER = r"[-+]?\d+(?:\.\d+)?"


def number(key, value):
    """One setting checked against its range (public, for API arguments)."""
    return _number(key, value)


def _number(key, value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise SettingsError("%s is %r, which is not a number" % (key, value))
    if number != number:                                 # NaN
        raise SettingsError("%s is not a number" % key)
    low, high = RANGES[key]
    if not low <= number <= high:
        raise SettingsError("%s is %g. It must be from %g to %g" % (key, number, low, high))
    return int(round(number)) if key in INTEGERS else number


def _boolean(value):
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _text(value):
    if isinstance(value, (list, tuple)):
        return "\n".join(str(v) for v in value)
    return "" if value is None else str(value)


# -- points ------------------------------------------------------------------

def parse_points(text):
    """'120 80 ref\\n150, 80' -> [{"x": 120.0, "y": 80.0, "ref": True, "datum": False}, ...]."""
    points = []
    for number, line in enumerate(_text(text).splitlines(), 1):
        line = line.split("#")[0].strip()
        if not line:
            continue
        words = re.split(r"[\s,;]+", line)
        flags = [w.lower() for w in words[2:]]
        if len(words) < 2 or not all(re.match("^%s$" % _NUMBER, w) for w in words[:2]):
            raise SettingsError("points, line %d: %r is not \"X Y\"" % (number, line))
        unknown = [f for f in flags if f not in ("ref", "datum")]
        if unknown:
            raise SettingsError("points, line %d: %r is not ref or datum" % (number, unknown[0]))
        x, y = float(words[0]), float(words[1])
        if abs(x) > XY_LIMIT or abs(y) > XY_LIMIT:
            raise SettingsError("points, line %d: X%g Y%g is outside +/-%g" % (number, x, y, XY_LIMIT))
        points.append(dict(x=x, y=y, ref="ref" in flags, datum="datum" in flags))
    if len(points) > MAX_POINTS:
        raise SettingsError("points: %d points, %d at most" % (len(points), MAX_POINTS))
    for flag in ("ref", "datum"):
        if sum(1 for p in points if p[flag]) > 1:
            raise SettingsError("points: only one point may be marked %s" % flag)
    return points


def format_points(points):
    lines = []
    for p in points:
        words = ["%g" % p["x"], "%g" % p["y"]]
        if p.get("ref"):
            words.append("ref")
        if p.get("datum"):
            words.append("datum")
        lines.append(" ".join(words))
    return "\n".join(lines)


def grid(x0, y0, x1, y1, nx, ny):
    """nx x ny points from corner (x0, y0) to (x1, y1), in a serpentine to save travel."""
    try:
        x0, y0, x1, y1 = (float(v) for v in (x0, y0, x1, y1))
        nx, ny = int(nx), int(ny)
    except (TypeError, ValueError):
        raise SettingsError("grid: the corners must be numbers and the counts whole numbers")
    if not (1 <= nx <= 20 and 1 <= ny <= 20):
        raise SettingsError("grid: 1 to 20 points along each side")
    if nx * ny > MAX_POINTS:
        raise SettingsError("grid: %d points, %d at most" % (nx * ny, MAX_POINTS))
    for v in (x0, y0, x1, y1):
        if abs(v) > XY_LIMIT:
            raise SettingsError("grid: %g is outside +/-%g" % (v, XY_LIMIT))
    xs = [x0 if nx == 1 else round(x0 + (x1 - x0) * i / (nx - 1), 3) for i in range(nx)]
    ys = [y0 if ny == 1 else round(y0 + (y1 - y0) * j / (ny - 1), 3) for j in range(ny)]
    points = []
    for j, y in enumerate(ys):
        row = xs if j % 2 == 0 else xs[::-1]
        points.extend(dict(x=x, y=y, ref=False, datum=False) for x in row)
    return points


# -- stages ------------------------------------------------------------------

def parse_stages(text):
    """'0.5; 0.1, 0.6; 0.02, 0.15' -> [(0.5, None), (0.1, 0.6), (0.02, 0.15)]."""
    stages = []
    chunks = [c.strip() for c in re.split(r"[;\n]+", _text(text)) if c.strip()]
    for index, chunk in enumerate(chunks):
        numbers = re.findall(_NUMBER, chunk)
        if re.sub(_NUMBER, "", chunk).strip(" ,\t") != "" or not 1 <= len(numbers) <= 2:
            raise SettingsError("stages: %r is not \"step\" or \"step, start_above\"" % chunk)
        step = round(float(numbers[0]), 3)
        above = round(float(numbers[1]), 3) if len(numbers) == 2 else None
        if index == 0:
            above = None                               # the coarse stage starts at safe_z
        elif above is None:
            raise SettingsError("stages: stage %d needs a start_above" % (index + 1))
        stages.append((step, above))
    if not 2 <= len(stages) <= MAX_STAGES:
        raise SettingsError("stages: give 2 to %d stages, a coarse search first and the fine "
                            "measurement last" % MAX_STAGES)
    for index, (step, above) in enumerate(stages):
        if not 0.001 <= step <= 5.0:
            raise SettingsError("stages: step %g must be from 0.001 to 5 mm" % step)
        if index and step >= stages[index - 1][0]:
            raise SettingsError("stages: each step must be finer than the one before")
        if index and not step < above <= 20.0:
            raise SettingsError("stages: stage %d's start_above, %g, must be more than its step "
                                "(%g) and at most 20 mm" % (index + 1, above, step))
    return stages


def format_stages(stages):
    return "; ".join("%g" % s if a is None else "%g, %g" % (s, a) for s, a in stages)


# -- laser modules ------------------------------------------------------------

def _optional(name, label, value, limit):
    value = value.strip()
    if not value:
        return None
    if not re.match("^%s$" % _NUMBER, value):
        raise SettingsError("laser modules: %s's %s %r is not a number" % (name, label, value))
    number = float(value)
    if abs(number) > limit:
        raise SettingsError("laser modules: %s's %s %g is beyond +/-%g mm" % (name, label, number, limit))
    return number


def parse_modules(text):
    """'10W = 1.25, 0, 0\n2W IR =' -> [{"name": "10W", "focus_offset": 1.25, "dx": 0.0,
    "dy": 0.0}, {"name": "2W IR", "focus_offset": None, "dx": None, "dy": None}]."""
    modules = []
    for line in _text(text).splitlines():
        line = line.strip()
        if not line:
            continue
        name, _, value = line.partition("=")
        name = name.strip()
        if not name:
            raise SettingsError("laser modules: %r has no name" % line)
        if any(m["name"] == name for m in modules):
            raise SettingsError("laser modules: %r is listed twice" % name)
        fields = value.split(",")
        if len(fields) > 3:
            raise SettingsError("laser modules: %r has more than \"focus offset, dx, dy\"" % line)
        fields += [""] * (3 - len(fields))
        dx = _optional(name, "dx", fields[1], XY_OFFSET_LIMIT)
        dy = _optional(name, "dy", fields[2], XY_OFFSET_LIMIT)
        if (dx is None) != (dy is None):
            raise SettingsError("laser modules: %s needs both dx and dy, or neither" % name)
        modules.append(dict(name=name, focus_offset=_optional(name, "focus offset", fields[0], OFFSET_LIMIT),
                            dx=dx, dy=dy))
    if not modules:
        raise SettingsError("laser modules: list at least one")
    return modules


def _short(value):
    """1.2345 -> "1.2345", -0.5 -> "-0.5": four decimals at most, no trailing zeros."""
    if value is None:
        return ""
    text = ("%.4f" % value).rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def format_modules(modules):
    lines = []
    for m in modules:
        fields = [_short(m.get("focus_offset"))]
        if m.get("dx") is not None or m.get("dy") is not None:
            fields += [_short(m.get("dx")), _short(m.get("dy"))]
        value = ", ".join(fields)
        lines.append(("%s = %s" % (m["name"], value)).rstrip())
    return "\n".join(lines)


def set_module_fields(text, name, **fields):
    """The modules text with some of `name`'s fields (focus_offset, dx, dy) set or cleared
    (None). Adds the module if it is new."""
    name = (name or "").strip()
    if not name or "=" in name or "\n" in name or "," in name:
        raise SettingsError("laser modules: %r is not a usable name" % name)
    limits = dict(focus_offset=OFFSET_LIMIT, dx=XY_OFFSET_LIMIT, dy=XY_OFFSET_LIMIT)
    for key, value in fields.items():
        if key not in limits:
            raise SettingsError("laser modules: no field %r" % key)
        if value is not None:
            value = float(value)
            if abs(value) > limits[key]:
                raise SettingsError("laser modules: %s %g is beyond +/-%g mm" % (key, value, limits[key]))
            fields[key] = value
    modules = parse_modules(text)
    for module in modules:
        if module["name"] == name:
            module.update(fields)
            break
    else:
        module = dict(name=name, focus_offset=None, dx=None, dy=None)
        module.update(fields)
        modules.append(module)
    text = format_modules(modules)
    parse_modules(text)                  # dx and dy must still come as a pair
    return text


# What M1006 reports for the modules whose reply is known (captured, or in the firmware source).
# A module missing here (the 2W IR) is only checked against these: it must not be one of them.
TOOLHEADS = {"10W": "10W LASER", "20W": "20W LASER", "40W": "40W LASER",
             # The 2W infrared: "RED_2W" in the firmware (Snapmaker2-Controller v4.7.2 M1005_1007.cpp).
             "2W IR": "2W RED LASER"}
# Modules with a crosshair (toolhead_laser.cpp is_there_cross_light): M2002 T3 P1/P0, offset
# from M2002 T8. The 10W has none, so its pointer is its own beam at low power.
CROSSHAIR = ("20W LASER", "40W LASER", "2W RED LASER")
# Modules with a camera whose photo the touchscreen fetches (toolhead_laser.cpp is_there_camera:
# the 1.6W and the 10W; only the 10W's single wide shot is used here).
CAMERA = ("10W LASER",)


def has_camera(toolhead):
    return bool(toolhead) and toolhead.strip().upper() in CAMERA


def is_laser(toolhead):
    return bool(toolhead) and "LASER" in toolhead.upper()


def module_for_toolhead(modules, toolhead):
    """The laser_modules name for what M1006 reports, or None when M1006 cannot tell (not a
    laser, or a laser whose M1006 name is not in TOOLHEADS yet)."""
    if not is_laser(toolhead):
        return None
    for name, reported in TOOLHEADS.items():
        if reported == toolhead.strip().upper():
            for m in modules:
                if "".join(m["name"].split()).lower() == "".join(name.split()).lower():
                    return m["name"]
    return None


def toolhead_mismatch(module_name, toolhead):
    """None when M1006's toolhead can be the selected laser module, else the reason it cannot.
    The firmware sees the laser module that is plugged in, not what is in the bay, so this
    catches the wrong module selected, not the probe parked in front of the laser."""
    if not toolhead:
        return "the printer did not say which toolhead is fitted (M1006)"
    head = toolhead.strip().upper()
    if "LASER" not in head:
        return "the printer says the %s toolhead is fitted, not a laser (M1006)" % toolhead
    want = TOOLHEADS.get(module_name)
    if want is not None:
        if head != want:
            return ("OctoPrint has the %s selected but the printer has the %s plugged in (M1006)"
                    % (module_name, toolhead))
        return None
    for name, known in TOOLHEADS.items():
        if head == known:
            return ("OctoPrint has the %s selected but the printer has the %s plugged in (M1006)"
                    % (module_name, name))
    return None


def set_module_offset(text, name, offset):
    """The modules text with `name`'s focus offset set (or cleared with None)."""
    return set_module_fields(text, name, focus_offset=offset)


# -- the whole lot --------------------------------------------------------------

def heights_missing(raw):
    """The names of the heights that are not set yet, for the tab's message."""
    raw = dict(DEFAULTS, **(raw or {}))
    return [key for key in HEIGHTS if raw.get(key) in (None, "")]


def routine_config(raw, require_heights=True):
    """The checked settings, in the form the routine takes. Raises SettingsError.

    require_heights=False is for saving the settings before safe_z and z_floor are captured:
    a value that is given must still be valid, but a missing one is allowed. Every job uses
    the default, True.
    """
    raw = dict(DEFAULTS, **(raw or {}))
    config = {}
    for key in RANGES:
        if key in OPTIONAL and raw.get(key) in (None, ""):
            config[key] = None
            continue
        if key in HEIGHTS and raw.get(key) in (None, ""):
            if require_heights:
                raise SettingsError("%s is not set. Jog to it and press \"Use current Z\" on the tab"
                                    % key)
            config[key] = None
            continue
        config[key] = _number(key, raw[key])
    for key in BOOLEANS:
        config[key] = _boolean(raw[key])

    url = str(raw["esp_url"] or "").strip()
    if not re.match(r"^https?://[^\s/]+", url):
        raise SettingsError("esp_url %r must start with http:// or https:// and name a host" % url)
    config["esp_url"] = url.rstrip("/")
    url = str(raw.get("headcam_url") or "").strip()
    if url and not re.match(r"^https?://[^\s/]+", url):
        raise SettingsError("headcam_url %r must start with http:// or https:// and name a host" % url)
    config["headcam_url"] = url.rstrip("/") or None
    url = str(raw.get("bedcam_url") or "").strip()
    if url and not re.match(r"^https?://[^\s/]+", url):
        raise SettingsError("bedcam_url %r must start with http:// or https:// and name a host" % url)
    config["bedcam_url"] = url or None
    base = str(raw.get("closeup_base") or "").strip().lower()
    if base not in CLOSEUP_BASES:
        raise SettingsError("closeup_base is %r. It must be one of: %s" % (raw.get("closeup_base"), ", ".join(CLOSEUP_BASES)))
    config["closeup_base"] = base

    if None not in (config["safe_z"], config["z_floor"]) and not config["safe_z"] > config["z_floor"]:
        raise SettingsError("safe_z (%g) must be above z_floor (%g)"
                            % (config["safe_z"], config["z_floor"]))
    if config["max_samples"] < config["samples"]:
        raise SettingsError("max_samples (%d) must be at least samples (%d)"
                            % (config["max_samples"], config["samples"]))
    config["stages"] = parse_stages(raw["stages"])
    config["laser_modules"] = parse_modules(raw["laser_modules"])
    config["points"] = parse_points(raw["points"])
    config["selected_module"] = str(raw["selected_module"] or "")
    config["bridge_allow"] = parse_allow(raw["bridge_allow"])
    return config


_ENTITY = re.compile(r"^[a-z_]+\.[a-z0-9_]+$")


def purifier_config(raw):
    """The purifier's settings in the form air.AirController takes. Raises SettingsError."""
    config = routine_config(raw, require_heights=False)
    raw = dict(DEFAULTS, **(raw or {}))
    url = _text(raw.get("purifier_ha_url")).strip().rstrip("/")
    if url and not re.match(r"^https?://[^\s/]+", url):
        raise SettingsError("purifier_ha_url %r must start with http:// or https:// and name a host" % url)
    entities = {}
    for key in ("purifier_switch", "purifier_pm25", "purifier_hcho"):
        value = _text(raw.get(key)).strip()
        if value and not _ENTITY.match(value):
            raise SettingsError("%s %r is not a Home Assistant entity id (like switch.name)" % (key, value))
        entities[key] = value
    if config["purifier_runon_max_s"] < config["purifier_runon_min_s"]:
        raise SettingsError("purifier_runon_max_s must be at least purifier_runon_min_s")
    policy = dict(start_j=config["purifier_start_j"], runon_min_s=config["purifier_runon_min_s"],
                  runon_max_s=config["purifier_runon_max_s"], pm25_margin=config["purifier_pm25_margin"],
                  hcho_margin=config["purifier_hcho_margin"], alarm_pm25=config["purifier_alarm_pm25"])
    return dict(enabled=config["purifier_enabled"], ha_url=url, ha_token=_text(raw.get("purifier_ha_token")),
                switch=entities["purifier_switch"], pm25=entities["purifier_pm25"],
                hcho=entities["purifier_hcho"], policy=policy, tau_s=config["purifier_tau_s"],
                enclosure_fan=config["purifier_enclosure_fan"],
                pointer_s_max=config["pointer_s_max"])


def link_config(raw):
    """The link guard's settings. Raises SettingsError."""
    raw = dict(DEFAULTS, **(raw or {}))
    switch = _text(raw.get("emergency_power_switch")).strip()
    if switch and not _ENTITY.match(switch):
        raise SettingsError("emergency_power_switch %r is not a Home Assistant entity id (like switch.name)" % switch)
    return dict(ok_s=number("link_ok_s", raw.get("link_ok_s")), power_s=number("link_power_s", raw.get("link_power_s")),
                switch=switch, ha_url=_text(raw.get("purifier_ha_url")).strip().rstrip("/"),
                ha_token=_text(raw.get("purifier_ha_token")))


def parse_allow(text):
    """"127.0.0.1, 192.168.1.20" -> ["127.0.0.1", "192.168.1.20"]: plain IPv4 addresses only."""
    out = []
    for item in re.split(r"[\s,;]+", str(text or "").strip()):
        if not item:
            continue
        parts = item.split(".")
        if len(parts) != 4 or not all(p.isdigit() and 0 <= int(p) <= 255 for p in parts):
            raise SettingsError("bridge_allow: %r is not an IPv4 address" % item)
        out.append(item)
    return out


def find_module(modules, name):
    for module in modules:
        if module["name"] == name:
            return module
    return modules[0] if modules else None
