#!/usr/bin/env python3
"""Keep LightBurn in step with what OctoBurn measures.

Runs on the computer that runs LightBurn. It reads OctoBurn's status (the rotary measurement,
the selected module and the close-up camera) and LightBurn's prefs.ini, and plans these changes:

- the selected module's rotary device ("Snapmaker 10W rotary via OctoPrint"): its rotary and
  object diameter (rotaryDiameter, rotaryObjDiameter) and the dia= of the job marker in its
  Start GCode, "(lhp head=10W mode=rotary dia=78.28)", all set to the measured diameter;
- the module's flat device ("Snapmaker 10W flat via OctoPrint"): the marker
  "(lhp head=10W mode=flat)" in its Start GCode;
- the flat device's network camera "10W close-up" present and pointing at OctoBurn's
  closeup.mjpg.

A diameter is only ever taken from a complete measurement that is not stale. The default is a
dry run that prints what would change. --apply writes only while LightBurn is closed (LightBurn
rewrites prefs.ini while it runs), after copying prefs.ini into a timestamped folder under
LightBurn's backup folder. prefs.ini is LightBurn's settings as JSON (Qt's indented format);
the tool splices the changed values into the original text, so every other byte stays as it was.

Exit codes: 0 nothing to do, or applied; 1 changes pending (dry run); 2 refused (LightBurn
running on --apply, stale or incomplete measurement, device not found, status unreadable).

Standard library only.
"""
from __future__ import print_function

import argparse
import copy
import datetime
import importlib.util
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

DEFAULT_PREFS = os.path.join("~", "Library", "Preferences", "LightBurn", "prefs.ini")
DEFAULT_OCTOPRINT = "http://octopi.local"
STATUS_PATH = "/api/plugin/laserheightprobe"
CAMERA_NAME = "10W close-up"          # the plugin's CLOSEUP_NAME
CAMERA_PATH = "/plugin/laserheightprobe/closeup.mjpg"
BACKUP_SUBDIR = os.path.join("backup", "octoburn-sync")
# LightBurn stores the diameter as a float32 widened to double (78.28 comes back as
# 78.27999877929688), so a stored value this close to the wanted one is already right.
DIA_EPS = 0.005

EXIT_OK, EXIT_PENDING, EXIT_REFUSED = 0, 1, 2

# The job marker, as the plugin's GRBL bridge reads it (grbl_bridge.job_marker).
_MARKER = re.compile(r"^\s*\(\s*lhp\b(.*)\)\s*$", re.I)
_MARKER_FIELD = re.compile(r"(\w+)\s*=\s*(.*?)\s*(?=\w+\s*=|$)")
_DEVICE = re.compile(r"^\s*snapmaker\s+(?P<head>.+?)\s+(?P<mode>rotary|flat)\b(?P<rest>.*)$", re.I)


class Refused(Exception):
    pass


# -- the job marker ---------------------------------------------------------------------------

def job_marker(line):
    """{"head", "mode", "dia"} from a marker line, or None for any other line."""
    m = _MARKER.match(line)
    if not m:
        return None
    fields = dict((k.lower(), v) for k, v in _MARKER_FIELD.findall(m.group(1).strip()))
    mode = fields.get("mode")
    try:
        dia = float(fields["dia"]) if fields.get("dia") else None
    except ValueError:
        dia = None
    return dict(head=fields.get("head") or None, mode=mode.lower() if mode else None, dia=dia)


def same_head(a, b):
    return "".join(a.split()).lower() == "".join(b.split()).lower()


def num(value, places=2):
    text = ("%.*f" % (places, value)).rstrip("0").rstrip(".")
    return text if text not in ("-0", "") else "0"


def marker_line(head, mode, dia=None):
    return "(lhp head=%s mode=%s%s)" % (head, mode, "" if dia is None else " dia=%s" % num(dia))


def synced_start_gcode(text, head, mode, dia=None):
    """text with its marker line right for head, mode and dia (dia only for rotary): the marker
    line is rewritten in place, or a new one goes first. Everything else stays."""
    lines = text.split("\n")
    for i, line in enumerate(lines):
        found = job_marker(line.rstrip("\r"))
        if found is None:
            continue
        ok = (found["head"] is not None and same_head(found["head"], head) and found["mode"] == mode
              and (dia is None or (found["dia"] is not None and abs(found["dia"] - dia) < DIA_EPS)))
        if ok:
            return text
        lines[i] = marker_line(head, mode, dia) + ("\r" if line.endswith("\r") else "")
        return "\n".join(lines)
    new = marker_line(head, mode, dia)
    return new + ("\n" + text if text else "")


# -- prefs.ini: Qt's indented JSON, edited in place -------------------------------------------

def qt_dumps(value, indent=""):
    """value as Qt's QJsonDocument::Indented writes it: four spaces, sorted keys, every list and
    object over several lines (empty ones too), numbers in their shortest form."""
    inner = indent + "    "
    if isinstance(value, dict):
        items = sorted(value.items(), key=lambda kv: kv[0].encode("utf-16-be"))
        body = ",\n".join(inner + json.dumps(k, ensure_ascii=False) + ": " + qt_dumps(v, inner)
                          for k, v in items)
        return "{\n" + (body + "\n" if body else "") + indent + "}"
    if isinstance(value, list):
        body = ",\n".join(inner + qt_dumps(v, inner) for v in value)
        return "[\n" + (body + "\n" if body else "") + indent + "]"
    if value is True:
        return "true"
    if value is False:
        return "false"
    if value is None:
        return "null"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if value.is_integer() and abs(value) < 1e15:
            return str(int(value))
        return repr(value)
    return json.dumps(value, ensure_ascii=False)


class JsonIndex(object):
    """Where every value of a JSON text sits: spans[path] = (start, end), and for each object
    or list members[path] = [(key or index, line start, value start, value end)]."""

    _WS = " \t\r\n"

    def __init__(self, text):
        self.text = text
        self.spans, self.members = {}, {}
        self._decoder = json.JSONDecoder()
        end = self._value(self._ws(0), ())
        if self._ws(end) != len(text):
            raise ValueError("extra data after the JSON at offset %d" % end)

    def _ws(self, i):
        while i < len(self.text) and self.text[i] in self._WS:
            i += 1
        return i

    def _line_start(self, i):
        return self.text.rfind("\n", 0, i) + 1

    def _value(self, i, path):
        text, start = self.text, i
        if text[i] == "{":
            members, i = [], self._ws(i + 1)
            if text[i] != "}":
                while True:
                    key_start = i
                    key, i = json.decoder.scanstring(text, i + 1)
                    i = self._ws(i)
                    if text[i] != ":":
                        raise ValueError("expected ':' at offset %d" % i)
                    vstart = self._ws(i + 1)
                    vend = self._value(vstart, path + (key,))
                    members.append((key, self._line_start(key_start), vstart, vend))
                    i = self._ws(vend)
                    if text[i] == ",":
                        i = self._ws(i + 1)
                        continue
                    break
            if text[i] != "}":
                raise ValueError("expected '}' at offset %d" % i)
            self.members[path] = members
            end = i + 1
        elif text[i] == "[":
            members, i = [], self._ws(i + 1)
            if text[i] != "]":
                while True:
                    n = len(members)
                    vend = self._value(i, path + (n,))
                    members.append((n, self._line_start(i), i, vend))
                    i = self._ws(vend)
                    if text[i] == ",":
                        i = self._ws(i + 1)
                        continue
                    break
            if text[i] != "]":
                raise ValueError("expected ']' at offset %d" % i)
            self.members[path] = members
            end = i + 1
        else:
            _, end = self._decoder.raw_decode(text, i)
        self.spans[path] = (start, end)
        return end

    def indent_of(self, path):
        """The indent of the line where the value at path starts."""
        start = self.spans[path][0]
        line = self._line_start(start)
        return self.text[line:start][:len(self.text[line:start]) - len(self.text[line:start].lstrip())]


def get_path(obj, path):
    for part in path:
        obj = obj[part]
    return obj


class Change(object):
    """One edit: set a value, insert a key into an object, or append to a list."""

    def __init__(self, kind, path, value, device, label, old=None, key=None):
        self.kind, self.path, self.value, self.key = kind, tuple(path), value, key
        self.device, self.label, self.old = device, label, old

    def apply_to(self, obj):
        if self.kind == "set":
            get_path(obj, self.path[:-1])[self.path[-1]] = copy.deepcopy(self.value)
        elif self.kind == "insert":
            get_path(obj, self.path)[self.key] = copy.deepcopy(self.value)
        elif self.kind == "append":
            get_path(obj, self.path).append(copy.deepcopy(self.value))

    def edit(self, index):
        """(start, end, replacement) in index.text."""
        text = index.text
        if self.kind == "set":
            start, end = index.spans[self.path]
            return start, end, qt_dumps(self.value, index.indent_of(self.path))
        if self.kind == "insert":
            members = index.members[self.path]
            if not members:
                start, end = index.spans[self.path]
                return start, end, qt_dumps({self.key: self.value}, index.indent_of(self.path))
            first_key_line = members[0][1]
            indent = text[first_key_line:members[0][2]]
            indent = indent[:len(indent) - len(indent.lstrip())]
            entry = indent + json.dumps(self.key, ensure_ascii=False) + ": " + qt_dumps(self.value, indent)
            sort_key = self.key.encode("utf-16-be")
            for key, line, _vstart, _vend in members:
                if key.encode("utf-16-be") > sort_key:
                    return line, line, entry + ",\n"
            last_end = members[-1][3]
            return last_end, last_end, ",\n" + entry
        if self.kind == "append":
            members = index.members[self.path]
            if not members:
                start, end = index.spans[self.path]
                return start, end, qt_dumps([self.value], index.indent_of(self.path))
            indent = text[members[0][1]:members[0][2]]
            last_end = members[-1][3]
            return last_end, last_end, ",\n" + indent + qt_dumps(self.value, indent)
        raise ValueError(self.kind)


def render(text, changes):
    """text with the changes spliced in, checked: it must parse to exactly the original with
    the changes applied, or nothing is written."""
    if not changes:
        return text
    index = JsonIndex(text)
    edits = sorted((c.edit(index) for c in changes), key=lambda e: (e[0], e[1]), reverse=True)
    for (s1, e1, _r1), (s2, e2, _r2) in zip(edits[1:], edits):
        if e1 > s2:
            raise ValueError("two changes overlap; nothing written")
    out = text
    for start, end, replacement in edits:
        out = out[:start] + replacement + out[end:]
    expected = json.loads(text)
    for change in changes:
        change.apply_to(expected)
    if json.loads(out) != expected:
        raise ValueError("the edited prefs.ini does not parse to the planned settings; nothing written")
    return out


# -- devices and the plan ---------------------------------------------------------------------

def find_devices(prefs):
    """Every LightBurn device named like "Snapmaker <head> <rotary|flat> [via OctoPrint]"."""
    found = []
    for i, dev in enumerate(prefs.get("DeviceList") or []):
        name = dev.get("DisplayName") or ""
        m = _DEVICE.match(name)
        if m:
            found.append(dict(index=i, name=name, head=m.group("head").strip(), mode=m.group("mode").lower(),
                              octoprint="octoprint" in m.group("rest").lower()))
    return found


def pick_device(devices, head, mode):
    match = [d for d in devices if same_head(d["head"], head) and d["mode"] == mode]
    if len(match) > 1:
        match = [d for d in match if d["octoprint"]] or match
    if len(match) > 1:
        raise Refused("more than one LightBurn device for %s %s: %s"
                      % (head, mode, ", ".join('"%s"' % d["name"] for d in match)))
    if not match:
        raise Refused('no LightBurn device for %s %s: expected one named like "Snapmaker %s %s via OctoPrint"'
                      % (head, mode, head, mode))
    return match[0]


def measurement_problem(rotary):
    """Why the rotary measurement cannot give LightBurn a diameter, or None."""
    if not rotary:
        return "OctoBurn has no rotary measurement"
    stale = rotary.get("stale")
    if stale:
        reason = stale.get("reason") if isinstance(stale, dict) else stale
        return "the rotary measurement is stale (%s): measure again" % (reason or "marked stale")
    if rotary.get("complete") is not True:
        return "the rotary measurement is not complete: finish or measure again"
    dia = rotary.get("diameter")
    if not isinstance(dia, (int, float)) or isinstance(dia, bool) or dia <= 0:
        return "the rotary measurement has no diameter"
    return None


def load_lightburn_camera():
    """The plugin's lightburn_camera module, loaded by path (the package itself needs OctoPrint)."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                        "octoprint_laserheightprobe", "lightburn_camera.py")
    spec = importlib.util.spec_from_file_location("octoburn_lightburn_camera", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Plan(object):
    def __init__(self):
        self.changes, self.ok, self.notes, self.refusals = [], [], [], []
        self.devices, self.module, self.synced = [], None, set()


def make_plan(prefs, status, base_url, module=None, skip_rotary=False, camera_entry=None):
    """What would change in prefs (the parsed prefs.ini) to match status (OctoBurn's status)."""
    plan = Plan()
    plan.devices = find_devices(prefs)
    names = [m.get("name") for m in status.get("modules") or [] if m.get("name")]
    plan.module = module or status.get("selected_module") or (names[0] if names else None)
    if not plan.module:
        plan.refusals.append("OctoBurn's status names no module: give --module")
        return plan
    devices = {}
    for mode in ("rotary", "flat"):
        try:
            devices[mode] = pick_device(plan.devices, plan.module, mode)
            plan.synced.add(devices[mode]["index"])
        except Refused as exc:
            plan.refusals.append(str(exc))

    rotary = status.get("rotary")
    problem = measurement_problem(rotary)
    dia = None if problem else round(float(rotary["diameter"]), 2)
    if problem:
        if skip_rotary:
            plan.notes.append("rotary diameter not synced (--skip-rotary): %s" % problem)
        else:
            plan.refusals.append(problem)

    dev = devices.get("rotary")
    if dev and dia is not None:
        path = ("DeviceList", dev["index"], "Settings")
        settings = get_path(prefs, path)
        for key, label in (("rotaryDiameter", "rotary diameter"), ("rotaryObjDiameter", "object diameter")):
            have = settings.get(key)
            what = "%s (%s)" % (label, key)
            if key not in settings:
                plan.changes.append(Change("insert", path, dia, dev["name"], what, key=key))
            elif not isinstance(have, (int, float)) or abs(have - dia) >= DIA_EPS:
                plan.changes.append(Change("set", path + (key,), dia, dev["name"], what, old=have))
            else:
                plan.ok.append((dev["name"], "%s %s mm" % (what, num(have))))
        if settings.get("rotaryMode") is False:
            plan.notes.append('"%s" has rotary turned off in LightBurn (rotaryMode false)' % dev["name"])
        _start_gcode(plan, prefs, dev, "rotary", dia)
    dev = devices.get("flat")
    if dev:
        _start_gcode(plan, prefs, dev, "flat", None)
        _camera(plan, prefs, dev, status, base_url, camera_entry)

    for other in plan.devices:
        if other["index"] in plan.synced:
            continue
        start = (get_path(prefs, ("DeviceList", other["index"])).get("Settings") or {}).get("StartGCode") or ""
        markers = [job_marker(l.rstrip("\r")) for l in start.split("\n")]
        markers = [m for m in markers if m]
        extra = ""
        if other["mode"] == "rotary" and dia is not None and markers and markers[0]["dia"] is not None \
                and abs(markers[0]["dia"] - dia) >= DIA_EPS:
            extra = ": its marker says dia=%s (run with --module \"%s\" to sync it)" % (num(markers[0]["dia"]),
                                                                                        other["head"])
        plan.notes.append('left alone (not module %s): "%s"%s' % (plan.module, other["name"], extra))
    return plan


def _start_gcode(plan, prefs, dev, mode, dia):
    path = ("DeviceList", dev["index"], "Settings")
    settings = get_path(prefs, path)
    have = settings.get("StartGCode")
    old = have if isinstance(have, str) else ""
    new = synced_start_gcode(old, dev["head"], mode, dia)
    if "StartGCode" not in settings:
        plan.changes.append(Change("insert", path, new, dev["name"], "Start GCode", key="StartGCode"))
    elif new != old:
        plan.changes.append(Change("set", path + ("StartGCode",), new, dev["name"], "Start GCode", old=old))
    else:
        plan.ok.append((dev["name"], "Start GCode marker %s" % marker_line(dev["head"], mode, dia)))


def _camera(plan, prefs, dev, status, base_url, camera_entry):
    closeup = status.get("closeup") or {}
    if not closeup.get("available"):
        plan.notes.append("the close-up camera is not available in OctoBurn: camera not checked")
        return
    url = base_url.rstrip("/") + CAMERA_PATH
    path = ("DeviceList", dev["index"], "Settings")
    settings = get_path(prefs, path)
    cams = settings.get("Cameras")
    for n, cam in enumerate(cams if isinstance(cams, list) else []):
        if cam.get("DisplayName") == CAMERA_NAME:
            if cam.get("CameraURL") == url:
                plan.ok.append((dev["name"], 'camera "%s" at %s' % (CAMERA_NAME, url)))
            else:
                plan.changes.append(Change("set", path + ("Cameras", n, "CameraURL"), url, dev["name"],
                                           'camera "%s" URL' % CAMERA_NAME, old=cam.get("CameraURL")))
            return
    try:
        if camera_entry is None:
            dpm = closeup.get("dpm")
            camera_entry = load_lightburn_camera().camera_entry(
                dpm, url, CAMERA_NAME, time.strftime("%Y-%m-%d_%H-%M"))
    except (OSError, ValueError, ImportError, AttributeError) as exc:
        plan.refusals.append('cannot build the camera "%s": %s' % (CAMERA_NAME, exc))
        return
    label = 'camera "%s" (new, %s)' % (CAMERA_NAME, url)
    if isinstance(cams, list):
        plan.changes.append(Change("append", path + ("Cameras",), camera_entry, dev["name"], label))
    else:
        plan.changes.append(Change("insert", path, [camera_entry], dev["name"], label, key="Cameras"))


# -- reading status, LightBurn running, writing -------------------------------------------------

def fetch_status(base_url, api_key=None, timeout=15):
    req = urllib.request.Request(base_url.rstrip("/") + STATUS_PATH)
    if api_key:
        req.add_header("X-Api-Key", api_key)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        raise Refused("OctoBurn status: HTTP %d from %s" % (exc.code, base_url.rstrip("/") + STATUS_PATH))
    except (urllib.error.URLError, OSError, ValueError) as exc:
        raise Refused("OctoBurn status unreadable from %s: %s" % (base_url, getattr(exc, "reason", exc)))


def read_api_key(path=None):
    if path:
        with open(os.path.expanduser(path)) as handle:
            return handle.read().strip()
    return os.environ.get("OCTOPRINT_API_KEY") or None


def lightburn_running():
    """True while LightBurn runs (pgrep). Refuses if that cannot be told."""
    try:
        return subprocess.run(["pgrep", "-x", "LightBurn"], stdout=subprocess.DEVNULL,
                              stderr=subprocess.DEVNULL).returncode == 0
    except OSError as exc:
        raise Refused("cannot tell whether LightBurn is running (%s)" % exc)


def default_backup_dir(prefs_path):
    return os.path.join(os.path.dirname(os.path.abspath(prefs_path)), BACKUP_SUBDIR)


def write_prefs(prefs_path, original, new, backup_dir=None, is_running=None, now=None):
    """Back up prefs.ini, then replace it atomically with new (bytes). Returns the backup path."""
    if (is_running or lightburn_running)():
        raise Refused("LightBurn is running: close it first (it rewrites prefs.ini while it runs)")
    with open(prefs_path, "rb") as handle:
        if handle.read() != original:
            raise Refused("prefs.ini changed while planning: run again")
    stamp = (now or datetime.datetime.now()).strftime("%Y%m%d_%H%M%S")
    folder = os.path.join(backup_dir or default_backup_dir(prefs_path), stamp)
    n = 1
    while os.path.exists(folder):
        n += 1
        folder = os.path.join(backup_dir or default_backup_dir(prefs_path), "%s_%d" % (stamp, n))
    os.makedirs(folder)
    backup = os.path.join(folder, os.path.basename(prefs_path))
    shutil.copy2(prefs_path, backup)
    directory = os.path.dirname(os.path.abspath(prefs_path))
    fd, tmp = tempfile.mkstemp(dir=directory, prefix=".%s." % os.path.basename(prefs_path), suffix=".tmp")
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(new)
            handle.flush()
            os.fsync(handle.fileno())
        shutil.copymode(prefs_path, tmp)
        os.replace(tmp, prefs_path)
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return backup


# -- report -------------------------------------------------------------------------------------

def _show(value):
    if isinstance(value, bool) or value is None:
        return json.dumps(value)
    if isinstance(value, (int, float)):
        return num(value, 4)
    return repr(value)


def report(plan, status, prefs_path, running):
    out = ["LightBurn prefs: %s (LightBurn %s)" % (prefs_path, "is running" if running else "is not running")]
    rotary = status.get("rotary") or {}
    if rotary.get("diameter") is not None:
        when = ""
        if isinstance(rotary.get("finished"), (int, float)):
            when = ", measured %s" % datetime.datetime.fromtimestamp(rotary["finished"]).strftime("%Y-%m-%d %H:%M")
        state = "complete" if rotary.get("complete") is True else "NOT complete"
        if rotary.get("stale"):
            state += ", STALE"
        out.append("OctoBurn: module %s, rotary diameter %s mm (%s%s), close-up %s"
                   % (plan.module, num(rotary["diameter"]), state, when,
                      "available" if (status.get("closeup") or {}).get("available") else "not available"))
    else:
        out.append("OctoBurn: module %s, no rotary measurement" % plan.module)
    out.append("Devices found:")
    if not plan.devices:
        out.append("  (none named like \"Snapmaker <head> <rotary|flat> via OctoPrint\")")
    for d in plan.devices:
        out.append("  [%d] %-40s head %-6s %-6s %s" % (d["index"], '"%s"' % d["name"], d["head"], d["mode"],
                                                    "synced" if d["index"] in plan.synced else "left alone"))
    if plan.changes:
        out.append("Changes:")
        device = None
        for c in plan.changes:
            if c.device != device:
                out.append('  "%s"' % c.device)
                device = c.device
            if c.label == "Start GCode":
                out.append("    Start GCode:")
                for line in (c.old or "").split("\n") if c.old else []:
                    out.append("      - %s" % line)
                for line in c.value.split("\n"):
                    out.append("      + %s" % line)
            elif c.kind == "set":
                out.append("    %s: %s -> %s" % (c.label, _show(c.old), _show(c.value)))
            elif c.kind == "insert" and not isinstance(c.value, list):
                out.append("    %s: (missing) -> %s" % (c.label, _show(c.value)))
            else:
                out.append("    %s: added" % c.label)
    if plan.ok:
        out.append("Already right:")
        for device, what in plan.ok:
            out.append('  "%s": %s' % (device, what))
    if plan.notes:
        out.append("Notes:")
        out.extend("  %s" % n for n in plan.notes)
    if plan.refusals:
        out.append("Refused:")
        out.extend("  %s" % r for r in plan.refusals)
    return "\n".join(out)


# -- main ---------------------------------------------------------------------------------------

def main(argv=None, is_running=None, now=None):
    parser = argparse.ArgumentParser(description="Keep LightBurn's devices in step with OctoBurn's "
                                     "measurements (dry run unless --apply).")
    parser.add_argument("--prefs", default=os.path.expanduser(DEFAULT_PREFS),
                        help="LightBurn's prefs.ini (default: %(default)s)")
    parser.add_argument("--status-json", help="a saved OctoBurn status JSON instead of fetching it")
    parser.add_argument("--octoprint", default=DEFAULT_OCTOPRINT,
                        help="OctoPrint's address, for the status and the camera URL (default: %(default)s)")
    parser.add_argument("--api-key-file", help="a file holding the OctoPrint API key "
                        "(default: the OCTOPRINT_API_KEY environment variable)")
    parser.add_argument("--module", help="the module to sync (default: OctoBurn's selected module)")
    parser.add_argument("--skip-rotary", action="store_true",
                        help="leave the rotary diameter alone when the measurement is missing or stale")
    parser.add_argument("--apply", action="store_true", help="write the changes (LightBurn must be closed)")
    parser.add_argument("--backup-dir", help="where the timestamped backup folder goes "
                        "(default: LightBurn's backup folder beside prefs.ini, under octoburn-sync)")
    args = parser.parse_args(argv)
    is_running = is_running or lightburn_running

    try:
        if args.status_json:
            with open(args.status_json) as handle:
                status = json.load(handle)
        else:
            status = fetch_status(args.octoprint, read_api_key(args.api_key_file))
        with open(args.prefs, "rb") as handle:
            original = handle.read()
        text = original.decode("utf-8")
        prefs = json.loads(text)
        running = is_running()
    except Refused as exc:
        print("Refused: %s" % exc)
        return EXIT_REFUSED
    except (OSError, ValueError) as exc:
        print("Refused: %s" % exc)
        return EXIT_REFUSED

    plan = make_plan(prefs, status, args.octoprint, module=args.module, skip_rotary=args.skip_rotary)
    try:
        new = render(text, plan.changes).encode("utf-8")
    except ValueError as exc:
        plan.refusals.append(str(exc))
        new = original
    print(report(plan, status, args.prefs, running))

    if plan.refusals:
        print("Result: refused, nothing written.")
        return EXIT_REFUSED
    if not plan.changes:
        print("Result: LightBurn already matches OctoBurn, nothing to do.")
        return EXIT_OK
    if not args.apply:
        print("Result: %d change%s pending (dry run). Run again with --apply%s to write them."
              % (len(plan.changes), "" if len(plan.changes) == 1 else "s",
                 " once LightBurn is closed" if running else ""))
        return EXIT_PENDING
    try:
        backup = write_prefs(args.prefs, original, new, args.backup_dir, is_running=is_running, now=now)
    except Refused as exc:
        print("Refused: %s" % exc)
        return EXIT_REFUSED
    print("Result: applied %d change%s. Backup: %s" % (len(plan.changes), "" if len(plan.changes) == 1 else "s",
                                                       backup))
    return EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
