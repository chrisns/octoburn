# coding=utf-8
"""Laser height probe for the Snapmaker 2.0: measure a delicate object with a BLTouch-style
probe in the laser's Quick Swap bay, and turn the heights into laser focus Z and LightBurn
Z offsets.

The main way in is Probe here, a compact panel moved into OctoPrint's Control tab: the user
jogs the head over a spot with the normal controls and presses one button for the final
lower. The automatic multi-point job lives on the plugin's own tab.

This is the only file that imports OctoPrint. Every other module in the package is plain
Python, so the tests run whole jobs against a simulated printer and probe.

The laser module stays plugged in while the probe sits in its bay, so the firmware stays in
laser mode. Nothing here may send a laser, fan or heater command; routine.py lists the only
G-code the plugin sends, and a test checks it.
"""
from __future__ import absolute_import

import collections
import functools
import json
import math
import os
import socket
import re
import threading
import time

import flask
import octoprint.plugin
from flask_babel import gettext
from octoprint.access import ADMIN_GROUP, USER_GROUP
from octoprint.access.permissions import Permissions
from octoprint.events import Events

from . import linkguard
from . import calibration, gcode, grbl_bridge, headcam, heightmap, meshload, results, settings as config_module
from . import bedcam, camera_model, closeup_plan, lightburn_camera
from . import air as air_mod, purifier as purifier_mod, redot
from .closeup_job import CloseupJob
from .closeup_libs import CloseupError, versions as closeup_versions
from .esp import EspError, EspProbe
from .gcode import GcodeBridge
from .routine import Job, TapSession
from .settings import DEFAULTS, SettingsError, routine_config

__plugin_name__ = "OctoBurn"
__plugin_pythoncompat__ = ">=3.7,<4"

LAST_FILE = "last_results.json"
HERE_FILE = "here_results.json"
# The rotary's angle frame across power cycles (head swaps): object angle = firmware B + shift.
B_FRAME_FILE = "b_frame.json"
MESH_Z_TOLERANCE = 0.1          # mm: the work offset check's Z slack while a height map is loaded (the A350 model matches to 0.002)
_COUNT_B_RE = re.compile(r"Count .*\bB:\s*(-?\d+)")
# The head height for turning the rotary by hand from the UI: above the chuck jaws and every object
# measured so far (the XY-near-the-chuck rule, 2026-11-19).
ROTARY_TURN_Z = 160.0
ROTARY_FILE = "rotary.json"
FRAME_FROM_HOUSING_MM = 3.0               # housing check vs a 10W-frame measurement: the 10W frame within this
ROTARY_PARTIAL_FILE = "rotary_partial.json"     # a run that stopped part way, kept to resume from
PIN_FILE = "beam_offset_pin.json"     # the pin-on-the-dot record, kept across a restart
MAP_FILE = "height_map.json"
# Present only while a height map may be in the printer's RAM: the print mesh copy to put back.
MESH_FILE = "mesh_backup.json"
LOG_LINES = 400
# The ESP's probe_check holds three pulses (680 + 100 + 680 ms) and a 50 ms read.
CHECK_WAIT_S = 2.0
# The close-up camera's files, in the plugin data folder's closeup/ folder.
CLOSEUP_DIR = "closeup"
CAMERA_MODEL_FILE = "camera_model.json"
CLOSEUP_FILE = "closeup.jpg"
HEADCAM_POSE_FILE = "headcam.json"     # where the overview photo was taken (the close-up's base)
# The fixed USB bed camera as the close-up base (closeup_base = "usb", bedcam.py), in closeup/.
BEDCAM_MODEL_FILE = "bedcam.json"            # its calibration
BEDCAM_CANDIDATE_FILE = "bedcam_candidate.json"  # a calibration not yet accepted
BEDCAM_PHOTO_FILE = "bedcam.jpg"             # its last overview photo (never headcam.jpg: LightBurn's
BEDCAM_POSE_FILE = "bedcam_pose.json"        # "10W head cam" reads that) and where the head was
BEDCAM_CALIB_PHOTO_FILE = "bedcam_calib.jpg"  # the photo the last crosses calibration used
BEDCAM_SETTLE_S = 1.0                        # after the park move, before the snapshot (the stream lags)
BEDCAM_Y_TOLERANCE = 0.05                    # mm: the bed must be where the calibration was made
CLOSEUP_NAME = "10W close-up"
PREVIEW_WIDTH = 1200                   # the tab's copy of the close-up image
AIR_TAG = "laserheightprobe-air"        # purifier lines: kept out of the bridge's count
AIR_STEP_S = 2.0
AIR_STATS_FILE = "purifier_stats.json"   # the purifier fan's running time, kept across restarts
AIR_FILTER_EVERY_S = 600.0              # how often the filter's life is read (only while idle)
DOT_MODEL_FILE = "dot_model.json"       # the red autofocus dot's calibration (redot.py)
DOT_Z_HIGH = 170.0                      # the first red-dot reading: high enough for 60 mm objects
DOT_STANDOFF = 100.0                    # the second: this far above the first reading's surface
DOT_AGREE_MM = 0.5                      # the two readings must agree this well
DOT_CALIB_Z = (100.0, 120.0, 140.0, 160.0, 180.0)
DOT_ROTARY_CLEAR = 80.0      # mm the head stays above the rotary object's highest top for a red-dot check
DOT_ROTARY_TRAVEL = 290.0    # head Z for moves across the rotary
DOT_CALIB_SPREAD = 2.0       # mm the dot may wander over flat paper across the calibration heights
DOT_Z_FEED = 1500                       # Z travel between readings (the close-up settle uses the same)
DOT_MAP_Z = 100.0                       # a red-dot map reads every point from here (surfaces up to ~35 mm)
DOT_MAP_MAX_POINTS = 120
MAP_PREVIOUS_FILE = "height_map_previous.json"   # the map a red-dot map replaced
OBSTACLES_FILE = "obstacles.json"               # tall objects on the bed, LightBurn boxes + top height
OBSTACLE_CLEARANCE_MARGIN = 2.0                 # mm: an object this close to the module's underside counts
BED_MAP_FILE = "height_map_bed.json"             # the last probed (whole-bed) map, kept for "Back to the bed map"
# the light's ray before any calibration (measured 2026-11-21): where it is level with the head,
# and its direction, head-relative in the bed frame
DOT_RAY = ((19.4, -0.4, 0.0), (-0.5209, 0.0141, -0.8535))
AIR_TEST_MAX_S = 10.0                  # the Test button: a few seconds of fan, no more (the filter)


def _float(data, key, required=True):
    value = (data or {}).get(key)
    if value is None or value == "":
        if required:
            raise ValueError("%s is missing" % key)
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        raise ValueError("%s is %r, which is not a number" % (key, value))



def _g28_homes_x(cmd):
    """True when this G28 homes X: a bare G28 (all axes) or one that names X."""
    words = cmd.strip().upper().split(";")[0].split()[1:]
    axes = {w[0] for w in words if w and w[0] in "XYZ"}
    return not axes or "X" in axes

class _OctoSender(object):
    """The bridge's way to the printer: OctoPrint's queue, tagged so the sent hook can count."""

    def __init__(self, plugin):
        self.p = plugin

    def send(self, lines, counted, gen=None):
        if counted:
            tags = {grbl_bridge.TAG, "%s-gen:%d" % (grbl_bridge.TAG, gen or 0)}
        else:
            tags = {grbl_bridge.TAG + "-direct"}
        self.p._printer.commands(list(lines), tags=tags)

    def urgent(self, lines):
        # M410 is one of OctoPrint's emergency commands (serial.emergencyCommands): it goes out
        # ahead of OctoPrint's own queue. The firmware still parses it behind what it holds.
        self.p._printer.commands(list(lines), tags={grbl_bridge.TAG + "-direct"}, force=True)

    def position(self):
        lines = self.p._bridge.run([], 15.0)
        found = gcode.position(lines)
        if found is None:
            return None
        b = gcode.b_position(lines)
        return (found[0], found[1], found[2], b or 0.0)


def rotary_mismatch(mode, lines):
    """None when M114 agrees with the mode, else why not. The firmware adds "B:" to M114 only
    while the rotary module is online, and prints "B unusable" when it is attached but cannot
    be used (Snapmaker2-Controller report_current_position)."""
    if mode not in ("rotary", "flat"):
        return None
    if gcode.position(lines) is None:
        return "the printer did not report its position (M114), so the rotary cannot be checked"
    attached = gcode.b_position(lines) is not None
    if any("B unusable" in line for line in lines):
        return "the printer says the rotary module is unusable (M114): reseat it and power-cycle"
    if mode == "rotary" and not attached:
        return ("the printer does not report the rotary module (no B in M114): plug it in and "
                "power-cycle, or pick flat")
    if mode == "flat" and attached:
        return ("the rotary module is attached (M114 reports B): take it off the bed for a flat "
                "job, or pick rotary")
    return None


def _bytes_file(name, body):
    """A file object for the file manager: OctoPrint's StreamWrapper, or the same interface."""
    import io
    try:
        from octoprint.filemanager.util import StreamWrapper
        return StreamWrapper(name, io.BytesIO(body))
    except ImportError:
        class _Bytes(object):
            filename = name

            def save(self, path):
                with open(path, "wb") as f:
                    f.write(body)

            def stream(self):
                return io.BytesIO(body)
        return _Bytes()


class LaserHeightProbePlugin(
    octoprint.plugin.SettingsPlugin,
    octoprint.plugin.TemplatePlugin,
    octoprint.plugin.AssetPlugin,
    octoprint.plugin.SimpleApiPlugin,
    octoprint.plugin.StartupPlugin,
    octoprint.plugin.ShutdownPlugin,
    octoprint.plugin.EventHandlerPlugin,
    octoprint.plugin.BlueprintPlugin,
):

    def __init__(self):
        self._bridge = None
        self._air = None                          # air_mod.AirController once started
        self._dot_last = None                     # the last red-dot height
        self._job = None
        self._lock = threading.RLock()
        self._log = collections.deque(maxlen=LOG_LINES)
        self._progress = None
        self._last = None
        self._last_error = None
        self._held = False
        self._clock = time                    # the tests put a fake clock here
        self._session = TapSession()          # the once-per-session tap test
        self._headcam_get = headcam.http_get  # the tests put a fake touchscreen here
        self._here = dict(points=[], datum=None)
        self._grbl = None              # the LightBurn bridge (grbl_bridge.Bridge)
        self._grbl_server = None
        self._grbl_client = None
        self._grbl_stop = threading.Event()
        self._rotary = None
        self._b_frame = dict(last_b=0.0, shift=0.0, check=False)
        self._b_frame_saved = 0.0
        self._detected = None                 # what the printer says is fitted (_detect)
        self._security_seen = None            # (time, dict) of the last laser safety report
        self._door = None                     # (time, "open" | "closed") from M1010 or the firmware's events
        self._pin = "up"                      # as last commanded by hand; the ESP cannot say
        self._map = None                      # the last heightmap.build() result
        self._mesh = None                     # dict(backup=..., loaded_at=...) while loaded
        self._mesh_warning = None
        self._camera_model = None             # the close-up camera model (camera_model.json)
        self._closeup_jpeg = None             # the last close-up image, served to LightBurn
        self._closeup_preview = None          # a light copy for the OctoPrint tab
        self._closeup_photos_time = None      # when the last close-up's tiles were taken
        self._closeup_disk_read = False       # the last close-up read back from disk (after a restart)
        self._closeup_last = None             # what it was: tiles, height, seam, time
        self._closeup_request = None          # the last box, height and stand-off (M240 P1)
        self._closeup_tiles = []              # [(jpeg, (x, y, z))] of the last close-up
        self._closeup_base = None             # (key, canvas) of the overview resampled
        self._headcam_pose = None             # where the overview photo was taken
        self._homed = False                   # a G28 sent since the printer connected
        self._closeup_h = 0.0                 # the height the last close-up's tiles were placed at
        # the fixed USB bed camera (closeup_base = "usb"); the tests put a fake camera here
        self._bedcam_get = functools.partial(headcam.http_get, what="the bed camera")
        self._bedcam_settle_s = BEDCAM_SETTLE_S
        self._bedcam_cache = None             # its calibration (closeup/bedcam.json)
        self._bedcam_jpeg = None              # its last overview photo
        self._bedcam_pose_cache = None        # where the head was parked for it

    # -- permissions ------------------------------------------------------

    def get_additional_permissions(self):
        """Two permissions, split by what the action can do to the machine.

        Reading results is harmless. Probing moves the head down onto the object, so it is
        kept to administrators by default.
        """
        return [
            {
                "key": "VIEW",
                "name": "View OctoBurn",
                "description": gettext("Allows reading the progress, the probe state and the results."),
                "default_groups": [USER_GROUP, ADMIN_GROUP],
                "roles": ["view"],
            },
            {
                "key": "PROBE",
                "name": "Run OctoBurn (probe, camera, laser)",
                "description": gettext(
                    "Allows moving the head, deploying the probe, measuring points and "
                    "saving the laser focus calibration."
                ),
                "default_groups": [ADMIN_GROUP],
                "roles": ["probe"],
            },
        ]

    # Which permission each API command needs. A command missing from here is refused, so a
    # new command cannot be added without deciding who may run it. Anyone who can watch a job
    # can stop it.
    COMMAND_PERMISSIONS = {
        "here": "PROBE",
        "detect": "PROBE",
        "rotary_template": "VIEW",
        "closeup_plan": "PROBE",
        "closeup": "PROBE",
        "closeup_clear": "PROBE",
        "burns_clear": "PROBE",
        "closeup_recompose": "PROBE",
        "closeup_selftest": "PROBE",
        "closeup_lightburn": "VIEW",
        "closeup_calib_capture": "PROBE",
        "closeup_calib_solve": "PROBE",
        "closeup_calib_mark": "PROBE",
        "closeup_calib_tie": "PROBE",
        "closeup_calib_accept": "PROBE",
        "bedcam_calibrate": "PROBE",
        "rotary": "PROBE",
        "rotary_check": "PROBE",
        "rotary_verify": "PROBE",
        "rotary_translate": "PROBE",
        "rotary_turn": "PROBE",
        "bridge_mode": "PROBE",
        "rotary_stale": "PROBE",
        "bridge_laser": "PROBE",
        "bridge_pointer": "PROBE",
        "pointer": "PROBE",
        "beam_offset": "PROBE",
        "tap_test": "PROBE",
        "here_datum": "PROBE",
        "here_clear": "PROBE",
        "here_as_reference": "PROBE",
        "here_as_bed": "PROBE",
        "head_photo": "PROBE",
        "start": "PROBE",
        "calibrate_probe": "PROBE",
        "abort": "VIEW",
        "purifier_stop": "VIEW",
        "ready_to_print": "PROBE",
        "map_bed": "PROBE",
        "focus_frame": "PROBE",
        "set_option": "PROBE",
        "obstacle_add": "PROBE",
        "dot_rotary_check": "PROBE",
        "obstacle_clear": "PROBE",
        "ready_for_laser": "PROBE",
        "dot_calibrate": "PROBE",
        "dot_measure": "PROBE",
        "dot_focus": "PROBE",
        "dot_map": "PROBE",
        "purifier_test": "PROBE",
        "position": "PROBE",
        "set_height": "PROBE",
        "set_xy_offset": "PROBE",
        "save_points": "PROBE",
        "grid": "VIEW",
        "select_module": "PROBE",
        "set_offset": "PROBE",
        "table": "VIEW",
        "probe_status": "VIEW",
        "deploy": "PROBE",
        "stow": "PROBE",
        "reset": "PROBE",
        "check": "PROBE",
        "set_od_mode": "PROBE",
        "map_build": "PROBE",
        "map_load": "PROBE",
        "map_restore": "PROBE",
    }

    @staticmethod
    def _allowed(key):
        return getattr(Permissions, "PLUGIN_LASERHEIGHTPROBE_" + key).can()

    # -- settings ---------------------------------------------------------

    def get_settings_defaults(self):
        return dict(DEFAULTS)

    def get_settings_version(self):
        return 1

    def on_settings_save(self, data):
        """Check the settings before they are saved, not after."""
        merged = self._config()
        merged.update(dict((k, v) for k, v in (data or {}).items() if k in DEFAULTS))
        try:
            # safe_z and z_floor may still be unset here; every job requires them.
            routine_config(merged, require_heights=False)
        except SettingsError as exception:
            self._logger.warning("refused a settings save: %s", exception)
            raise ValueError(str(exception))
        octoprint.plugin.SettingsPlugin.on_settings_save(self, data)
        self._air_reconfigure()

    def get_settings_restricted_paths(self):
        # the Home Assistant token: admins only, never in a page for other users
        return dict(admin=[["purifier_ha_token"]])

    def _config(self):
        return dict((key, self._settings.get([key])) for key in DEFAULTS)

    def _store(self, values):
        """Write settings from code, and tell open pages to reload them."""
        for key, value in values.items():
            self._settings.set([key], value)
        self._settings.save(trigger_event=True)

    def _checked(self):
        try:
            return routine_config(self._config())
        except SettingsError as exception:
            raise ValueError("the settings are not safe to use: %s" % exception)

    def _checked_loose(self):
        """The settings, allowing safe_z and z_floor to be unset (for actions that do not move)."""
        try:
            return routine_config(self._config(), require_heights=False)
        except SettingsError as exception:
            raise ValueError("the settings are not valid: %s" % exception)

    def _esp(self, config=None):
        config = config or self._checked_loose()
        return EspProbe(config["esp_url"], config["http_timeout"], logger=self._logger)

    # -- templates and assets ---------------------------------------------

    def is_template_autoescaped(self):
        return True

    def get_template_configs(self):
        return [
            dict(type="tab", name="OctoBurn", template="laserheightprobe_tab.jinja2",
                 custom_bindings=True),
            # OctoPrint has no "control" template type. The panel renders as a generic template
            # and the view model moves it into the Control tab after binding (after
            # #control-jog-general, as in OctoPrint 1.11). The empty sidebar slot is the
            # fallback if that place is not found, and is hidden otherwise.
            dict(type="generic", template="laserheightprobe_control.jinja2", custom_bindings=True),
            dict(type="sidebar", name="OctoBurn", template="laserheightprobe_sidebar.jinja2",
                 custom_bindings=True, icon="fas fa-ruler-vertical"),
            # False on purpose, as in NozzleTouch: the pane binds straight through OctoPrint's
            # own settings view model, so settings.plugins.laserheightprobe.<key> resolves.
            dict(type="settings", name="OctoBurn",
                 template="laserheightprobe_settings.jinja2", custom_bindings=False),
        ]

    def get_assets(self):
        return dict(js=["js/laserheightprobe.js"], css=["css/laserheightprobe.css"])

    # -- lifecycle --------------------------------------------------------

    def on_after_startup(self):
        self._bridge = GcodeBridge(self._printer, self._logger)
        self._last = self._read_json(LAST_FILE)
        self._here = self._read_json(HERE_FILE) or dict(points=[], datum=None)
        self._rotary = self._read_json(ROTARY_FILE)
        self._rotary_partial = self._read_json(ROTARY_PARTIAL_FILE)
        self._b_frame = dict(self._b_frame, **(self._read_json(B_FRAME_FILE) or {}))
        self._b_frame["check"] = False
        self._map = self._read_json(MAP_FILE)
        if self._read_json(BED_MAP_FILE) is None:
            # first start with this version: the bed map is the stored map, or the one a red-dot
            # map replaced
            for name in (MAP_FILE, MAP_PREVIOUS_FILE):
                m = self._read_json(name)
                if m and m.get("source") != "red dot":
                    self._write_json(BED_MAP_FILE, m)
                    break
        self._mesh = self._read_json(MESH_FILE)
        self._bridge_start()
        self._air_start()
        self._frame_watching = True
        threading.Thread(target=self._frame_watch, name="laserheightprobe-frame", daemon=True).start()
        self._link_start()
        self._logger.info("laser height probe ready")

    def on_shutdown(self):
        self._frame_watching = False
        self._stop_job("OctoPrint is shutting down")
        self._bridge_stop()
        self._air_stop("OctoPrint is shutting down")

    def on_event(self, event, payload):
        if event in (getattr(Events, "CONNECTED", "Connected"), getattr(Events, "DISCONNECTED", "Disconnected"),
                     getattr(Events, "ERROR", "Error")):
            self._homed = False                   # a reconnect may follow a power cycle: position unknown
        if event == getattr(Events, "CONNECTED", "Connected"):
            self._b_frame_reconnected()
            self._detected = None
            threading.Thread(target=self._detect_quietly, name="laserheightprobe-detect",
                             daemon=True).start()
        if event == getattr(Events, "CONNECTED", "Connected") and self._mesh:
            threading.Thread(target=self._check_mesh_after_connect, name="laserheightprobe-mesh",
                             daemon=True).start()
        if event == Events.PRINT_STARTED and self._mesh:
            threading.Thread(target=self._check_mesh_before_print, name="laserheightprobe-mesh",
                             daemon=True).start()
        if event in (Events.DISCONNECTED, Events.ERROR) and self._air is not None:
            self._air.off("the printer disconnected", stop_policy=True)
        if event in (Events.DISCONNECTED, Events.ERROR):
            self._b_frame_save(force=True)
            self._detected = None
            self._stop_job("the printer disconnected")
            self._bridge_disarm("the printer disconnected")
        elif event == Events.PRINT_STARTED and self._running():
            # The job is on hold, so no line of it has been sent. Stop the probing, and let
            # the job go once the head is up and the pin stowed.
            self._stop_job("a print was started")

    @staticmethod
    def _bridge_gen(tags):
        for tag in tags:
            if tag.startswith(grbl_bridge.TAG + "-gen:"):
                return int(tag.rsplit(":", 1)[1])
        return None

    def gcode_sending(self, comm_instance, phase, cmd, cmd_type, gcode_, *args, **kwargs):
        """Drop a bridge line still in OctoPrint's queue from before a stop: it must not run."""
        tags = kwargs.get("tags") or set()
        if self._grbl is not None and grbl_bridge.TAG in tags and \
                self._grbl.stale(self._bridge_gen(tags)):
            return None,
        return None

    def gcode_sent(self, comm_instance, phase, cmd, cmd_type, gcode_, *args, **kwargs):
        link = getattr(self, "_link", None)
        if link is not None and cmd:
            link.sent(cmd)
        tags = kwargs.get("tags") or set()
        if AIR_TAG in tags:
            return                                # a purifier line: not the bridge's, not a burn
        if self._air is not None:
            self._air.on_sent(cmd)
        if cmd and cmd.lstrip().upper().startswith("G28"):
            self._homed = True                    # any home (ours, a job's, LightBurn's)
            self._homed_at = time.time()
            if _g28_homes_x(cmd):
                # X homes ~12 mm differently with the probe on than with the 10W seated (2026-11-22):
                # note which head this home was made with, once it has finished. A G28 Y or Z (the
                # rotary routine's, back from the housing touch) leaves X, and that note, alone.
                self._homed_head = None
                threading.Thread(target=self._note_homed_head, name="laserheightprobe-homed-head", daemon=True).start()
            if self._mesh_loaded():
                # in laser mode G28 leaves levelling OFF (seen 2026-11-21): put the map back on at
                # once, queued straight after the home, before any burn can follow it
                threading.Thread(target=self._printer.commands, args=(["M420 S1"],),
                                 name="laserheightprobe-relevel", daemon=True).start()
        tags = kwargs.get("tags") or set()
        if self._grbl is not None and grbl_bridge.TAG in tags:
            self._grbl.on_sent(cmd, self._bridge_gen(tags))

    # -- LightBurn bridge -------------------------------------------------

    # -- the red autofocus dot (redot.py) ------------------------------------

    def _bed_map(self):
        """The probed bed map (kept apart from a red-dot object map), or None."""
        bed = self._read_json(BED_MAP_FILE)
        if bed and bed.get("nodes") and bed.get("base"):
            return bed
        hm = self._map
        if hm and hm.get("nodes") and hm.get("base") and hm.get("source") != "red dot":
            return hm
        return None

    @staticmethod
    def _raw_of(hm):
        from . import compositor
        b = hm["base"]
        return compositor.surface_from_map(hm["nodes"], hm["start"], hm["spacing"],
                                           (b["machine_x"] - b["x"], b["machine_y"] - b["y"]))

    def _dot_model(self):
        """The red-dot model. If the bed map was remade since the calibration, the calibration's
        own photos are refitted against the new map (the bed's height under the dot is the only
        term the map gives it), so a re-probe never leaves the red dot reading off. No motion."""
        path = os.path.join(CLOSEUP_DIR, DOT_MODEL_FILE)
        data = self._read_json(path)
        self._dot_known = bool(data)
        if not data:
            return None
        samples = data.get("samples") or []
        bed = self._bed_map()
        if (bed is not None and samples and all("dot_b" in q and "at_head" in q for q in samples)
                and data.get("bed_created") != bed.get("created")):
            try:
                raw = self._raw_of(bed)
                again = [dict(q, lift=float(q["at_head"]) - float(raw(q["dot_b"][0], q["dot_b"][1])))
                         for q in samples]
                dm = redot.fit(again)
                dm.time = data.get("time")
                new = dict(dm.to_dict(), samples=again, bed_created=bed.get("created"),
                           refitted=time.time(), lb=data.get("lb"))
                self._save_json(path, new)
                self._log_line("Red dot refitted to the new bed map from its calibration photos (c %+.3f "
                               "mm, residual %.2f px)." % (dm.c - float(data["c"]), dm.residual_px or 0.0))
                return dm
            except Exception as exception:                   # noqa: BLE001 - keep the old fit
                self._logger.warning("could not refit the red dot to the new bed map: %s", exception)
        return redot.DotModel.from_dict(data)

    def _dot_calibrated(self):
        if getattr(self, "_dot_known", None) is None:
            self._dot_model()
        return self._dot_known

    def _dot_ray_offset(self, dm, z, h):
        """Where the dot lands relative to the head, from the model or the measured default ray."""
        if dm is not None:
            return dm.offset(z, h)
        (px, py, pz), (dx, dy, dz) = DOT_RAY
        t = (h - z - pz) / dz
        return px + t * dx, py + t * dy

    DOT_MAP_MARGIN = 15.0                  # mm past the probed points where the map is still trusted

    def _dot_lift(self, head, dot_b):
        """(lift, mapped): the firmware's lift at the head (map loaded) less the bed's height under
        the dot, and whether the dot lies where the map was probed. The head's lift is real
        wherever the printer's grid reaches (it applies the same nodes); the bed under the dot is
        only known where it was probed. Outside, the map's guess was 0.5 mm out at the front edge
        (2026-11-21: the 2.7 mm wood read 3.21), so the reading is reported as rougher there."""
        loaded_raw = self._map_raw()               # what the printer applies (the stored map)
        hm = self._bed_map() or {}                # the bed under the dot: never an object map
        raw = self._raw_of(hm) if hm else None
        pts = [(p["x"], p["y"]) for p in (hm.get("samples") or []) if "x" in p and "y" in p]
        if raw is None or not pts:
            at_head = float(loaded_raw(head[0], head[1])) if (loaded_raw is not None and self._mesh_loaded()) else 0.0
            return at_head, False
        m = self.DOT_MAP_MARGIN
        x0, x1 = min(p[0] for p in pts) - m, max(p[0] for p in pts) + m
        y0, y1 = min(p[1] for p in pts) - m, max(p[1] for p in pts) + m
        inside = bool(x0 <= float(dot_b[0]) <= x1 and y0 <= float(dot_b[1]) <= y1)
        at_head = float(loaded_raw(head[0], head[1])) if (loaded_raw is not None and self._mesh_loaded()) else 0.0
        return at_head - float(raw(dot_b[0], dot_b[1])), inside

    def _dot_pair(self, config, x, y, z):
        """Light-on and light-off photos from the head's spot (it is already there)."""
        from . import cvio
        url = config["headcam_url"]
        t = [time.time()]
        self._bridge.run(["M2002 L1"], 30.0)
        t.append(time.time())
        try:
            time.sleep(0.5)
            on = headcam.capture(self._headcam_get, url, x, y, z, quality=config["headcam_quality"],
                                 before=getattr(self, "_dot_last_jpeg", None))
            t.append(time.time())
        finally:
            self._bridge.run(["M2002 L0"], 30.0)
        t.append(time.time())
        time.sleep(0.3)
        off = headcam.capture(self._headcam_get, url, x, y, z, quality=config["headcam_quality"], before=on)
        self._dot_last_jpeg = off                  # the touchscreen holds this one now
        t.append(time.time())
        self._dot_pair_seconds = [round(b - a, 1) for a, b in zip(t, t[1:])]   # light on, photo, light off, photo
        return cvio.decode(on), cvio.decode(off)

    def _dot_go(self, config, ctx, head, travel):
        """Up to the travel height first, across, then to the reading's height. Beam off."""
        lo, hi = ctx["limits"]
        if not (lo[0] <= head[0] <= hi[0] and lo[1] <= head[1] <= hi[1]):
            raise ValueError("the head would have to go to X%.1f Y%.1f, past its limits: the red dot "
                             "lands 40 to 90 mm to -X of the head, so pick a point further left"
                             % (head[0], head[1]))
        where = ctx["current"]
        moves = ["M5", "G90"]
        if where[2] < travel - 1e-6:
            moves.append("G0 Z%.1f F%d" % (travel, DOT_Z_FEED))
        moves.append("G0 X%.1f Y%.1f F%d" % (head[0], head[1], config["xy_feed"]))
        moves.append("G0 Z%.1f F%d" % (head[2], DOT_Z_FEED))
        t0 = time.time()
        self._bridge.run(moves, 120.0)
        t1 = time.time()
        where = gcode.position(self._bridge.run([], 15.0))
        ctx["go_seconds"] = (round(t1 - t0, 1), round(time.time() - t1, 1))
        if where is None or abs(where[0] - head[0]) > 0.05 or abs(where[1] - head[1]) > 0.05 \
                or abs(where[2] - head[2]) > 0.05:
            raise ValueError("the head is not where it was sent (M114 %s)" % (where,))
        ctx["current"] = where

    def _dot_read(self, config, ctx, model, dm, target_b, z, h_guess):
        """One reading with the dot aimed at target_b (bed frame): dict(h, px, head, dot)."""
        t0 = time.time()
        dx, dy = self._dot_ray_offset(dm, z, h_guess)
        head = (round(target_b[0] - dx, 1), round(target_b[1] - dy, 1), z)
        # at Z100 and up the hood clears the work by ~89 mm (60 mm objects at most): move across at
        # that height instead of climbing to the close-up's travel floor every point (20 s a point)
        travel = max(z, ctx["current"][2]) if z >= DOT_STANDOFF else max(z, ctx["current"][2], closeup_plan.TRAVEL_FLOOR)
        travel = max(travel, ctx.get("travel_floor", 0.0))
        self._dot_go(config, ctx, head, travel)
        t_moved = time.time()
        try:
            on, off = self._dot_pair(config, head[0], head[1], z)
            found = redot.find_dot(on, off)
        except (CloseupError, headcam.HeadcamError) as exception:   # a glint, an exposure change, Wi-Fi: once more
            self._logger.info("red dot at Z%.1f, retrying: %s", z, exception)
            on, off = self._dot_pair(config, head[0], head[1], z)
            found = redot.find_dot(on, off)
        if dm is not None and dm.off_line(found["x"], found["y"]) > redot.LINE_TOL_PX:
            raise ValueError("the bright spot at px (%.0f, %.0f) is off the red dot's line: not the dot"
                             % (found["x"], found["y"]))
        dot_b = model.to_plane(head, [[found["x"], found["y"]]], 0.0)[0]
        out = dict(head=list(head), px=[found["x"], found["y"]], dot_b=[float(dot_b[0]), float(dot_b[1])],
                   rise=found["rise"], unique=found["unique"], area=found["area"],
                   seconds=dict(move=round(t_moved - t0, 1), photos=round(time.time() - t_moved, 1),
                                go=ctx.get("go_seconds"), pair=getattr(self, "_dot_pair_seconds", None)))
        if dm is not None:
            if ctx.get("rotary"):
                lift, mapped = 0.0, False           # the dot is on the object, not the bed; no mesh with the rotary
            else:
                lift, mapped = self._dot_lift(head, (float(dot_b[0]), float(dot_b[1])))
            out["h"] = dm.height(z, found["x"], lift)
            out["lift"] = lift
            out["mapped"] = mapped
            out["mm_per_px"] = dm.mm_per_px(z, out["h"])
        return out

    def _api_dot_measure(self, data):
        """The height of the surface at LightBurn (x, y) from the 10W's red dot: a reading from
        Z170, then a finer one from 100 mm above what it found; they must agree. Moves the head
        (beam off, LightBurn locked out). Nothing else uses the answer by itself: the page shows
        it in the close-up's Height field."""
        config, model, ctx = self._closeup_setup()
        dm = self._dot_model()
        if dm is None:
            raise ValueError("calibrate the red dot first (dot_calibrate over flat paper)")
        lb = (float(data["x"]), float(data["y"]))
        o = ctx["o"]
        target = (lb[0] + o[0], lb[1] + o[1])
        self._closeup_settle(config, ctx)
        if self._grbl is not None:
            self._grbl.lock_out("measuring a height with the red dot")
        try:
            z1 = float(data.get("z") or DOT_Z_HIGH)
            first = self._dot_read(config, ctx, model, dm, target, z1, 0.0)
            readings = [first]
            z2 = round(max(first["h"] + DOT_STANDOFF, DOT_STANDOFF), 1)
            if z2 < z1 - 15.0:
                second = self._dot_read(config, ctx, model, dm, target, z2, first["h"])
                readings.append(second)
                if abs(second["h"] - first["h"]) > DOT_AGREE_MM + 0.02 * abs(first["h"]):
                    raise ValueError("the two red-dot readings disagree (%.2f mm from Z%.0f, %.2f mm from "
                                     "Z%.0f): something shiny, dark or uneven under the dot?"
                                     % (first["h"], z1, second["h"], z2))
        finally:
            if self._grbl is not None:
                self._grbl.lock_release()
        best = readings[-1]
        dot_lb = (best["dot_b"][0] - o[0], best["dot_b"][1] - o[1])
        # 2026-11-21: paper (true 0) at 8 spots on the probed map read -0.09..+0.38 (6 of them
        # -0.09..+0.20; without the map term -0.36..+0.08); the 2.7 mm wood, in front of the map,
        # read 3.21
        accuracy = 0.4 if best.get("mapped") else 0.6
        result = dict(ok=True, height=round(best["h"], 2), lb=[round(dot_lb[0], 1), round(dot_lb[1], 1)],
                      resolution=round(best["mm_per_px"], 3), accuracy=accuracy, mapped=best.get("mapped"),
                      readings=readings, time=time.time())
        self._dot_last = result
        self._log_line("Red dot: %.2f mm above the bed at LightBurn X%.1f Y%.1f (%s)."
                       % (best["h"], dot_lb[0], dot_lb[1],
                          ", ".join("%.2f from Z%.0f" % (r["h"], r["head"][2]) for r in readings)))
        return result

    def _api_dot_rotary_check(self, data):
        """The rotary object's top from the red dot at points along it, against the stored rotary
        measurement (at the object's present angle, runout included). No laser, no probe. The head
        travels at Z290 and reads from at least DOT_ROTARY_CLEAR above the object's highest top."""
        config, model, ctx = self._closeup_setup(allow_rotary=True)
        if not ctx["rotary"]:
            raise ValueError("the rotary is not fitted (no B in M114)")
        if not self._rotary or not self._rotary.get("complete"):
            raise ValueError("there is no complete rotary measurement to check against")
        dm = self._dot_model()
        if dm is None:
            raise ValueError("calibrate the red dot first (dot_calibrate over flat paper, rotary off)")
        from . import rotary_gcode
        module = ctx["module"]
        surface = rotary_gcode.Surface(self._rotary, module.get("focus_offset"))
        b_now = gcode.b_position(self._bridge.run(["M114"], 30.0)) or 0.0
        angle = b_now + float(self._b_frame.get("shift") or 0.0)     # the object's angle, as measured
        bed = config["flat_bed_z"]
        across = data.get("across")
        if across:
            # across the top at one Y: where the crest really is, sideways, in the beam's frame
            y_at = float(data.get("y") or (surface.y_lo + surface.y_hi) / 2.0)
            offsets = [float(o) for o in across][:12]
            ys = [y_at] * len(offsets)
        else:
            offsets = None
            n = min(max(int(data.get("points") or 6), 2), 12)
            y0, y1 = surface.y_lo + 15.0, surface.y_hi - 20.0
            ys = [round(y0 + (y1 - y0) * i / float(n - 1), 1) for i in range(n)]
        highest = max(surface.top(y, b) for y in (surface.y_lo, surface.y_hi) for b in range(0, 360, 15))
        tops = [surface.top(y, angle) for y in ys]
        radius = surface.r
        if offsets:
            tops = [t - (radius - math.sqrt(max(0.0, radius * radius - o * o))) for t, o in zip(tops, offsets)]
        z = round(max(max(tops) - bed + DOT_STANDOFF, highest + DOT_ROTARY_CLEAR), 1)
        if z > rotary_gcode.Z_MAX - 5.0:
            raise ValueError("the reading height Z%.0f is too near the top of Z" % z)
        ctx["travel_floor"] = max(DOT_ROTARY_TRAVEL, ctx["current"][2])
        if self._grbl is not None:
            self._grbl.lock_out("checking the rotary object with the red dot")
        points = []
        try:
            self._bridge.run(["M5", "G90", "G0 Z%.1f F%d" % (ctx["travel_floor"], DOT_Z_FEED)], 120.0)
            ctx["current"] = gcode.position(self._bridge.run([], 15.0)) or ctx["current"]
            for k, (y, top) in enumerate(zip(ys, tops)):
                off = offsets[k] if offsets else 0.0
                target = (surface.axis_x(y, angle) + off + module["dx"], y + module["dy"])
                pred = top - bed
                try:
                    r = self._dot_read(config, ctx, model, dm, target, z, pred)
                    points.append(dict(y=y, offset=off, predicted=round(pred, 2), measured=round(r["h"], 2),
                                       diff=round(r["h"] - pred, 2), head=r["head"]))
                except (ValueError, CloseupError, headcam.HeadcamError) as exception:
                    points.append(dict(y=y, predicted=round(pred, 2), error=str(exception)))
        finally:
            try:
                self._bridge.run(["M5", "G90", "G0 Z%.1f F%d" % (ctx["travel_floor"], DOT_Z_FEED)], 120.0)
            finally:
                if self._grbl is not None:
                    self._grbl.lock_release()
        good = [p_["diff"] for p_ in points if "diff" in p_]
        if not good:
            raise ValueError("no red-dot reading worked on the object: %s" % points[0].get("error"))
        mean = sum(good) / len(good)
        crest = None
        if offsets:
            # fit h(o) = c - (r - sqrt(r^2 - (o - e)^2)): e is how far the real crest sits from the measured one
            pts = [(p_["offset"], p_["measured"]) for p_ in points if "measured" in p_]
            best = None
            for i in range(-400, 401):
                e = i * 0.025
                drops = [radius - math.sqrt(max(0.0, radius * radius - (o - e) ** 2)) for o, _ in pts]
                c = sum(h + dr for (o, h), dr in zip(pts, drops)) / len(pts)
                err = sum((c - dr - h) ** 2 for (o, h), dr in zip(pts, drops))
                if best is None or err < best[0]:
                    best = (err, e, c)
            if best is not None and len(pts) >= 3:
                crest = dict(dx=round(best[1], 2), top=round(best[2], 2), rms=round(math.sqrt(best[0] / len(pts)), 3))
                self._log_line("Red dot across the top at Y%.1f: the crest is %+.2f mm sideways of the measurement's (fit "
                               "rms %.3f mm)." % (ys[0], crest["dx"], crest["rms"]))
        self._log_line("Rotary check with the red dot at B%.1f: %d of %d points, measured minus stored %+.2f mm "
                       "on average (%+.2f..%+.2f), from Z%.0f." % (b_now, len(good), len(points), mean,
                                                                    min(good), max(good), z))
        return dict(ok=True, points=points, mean_diff=round(mean, 2), spread=round(max(good) - min(good), 2),
                    z=z, b=b_now, crest=crest)

    FRAME_FRESH_S = 1800        # a frame older than this is not the job about to run

    def _ready_checks(self):
        """Ready to burn? Each check: dict(key, level ok|warn|bad|info, text, fix). Built only from
        state the plugin already holds (the page polls it every few seconds: no serial traffic)."""
        from . import rotary_gcode
        checks = []

        def add(key, level, text, fix=None):
            checks.append(dict(key=key, level=level, text=text, fix=fix))
        d = self._detected or {}
        b = self._grbl
        ctx = getattr(b, "context", None) if b is not None else None
        mode = getattr(ctx, "mode", None)
        if not d.get("laser"):
            add("head", "bad", "No laser is fitted (or not detected yet).", "detect")
            return checks
        sec = d.get("security") or {}
        if sec.get("state", 0) != 0:
            add("head", "bad", "The %s is not seated on the head (safety 0x%x): it will not fire."
                % (d.get("module") or "laser", sec["state"]), "detect")
        else:
            add("head", "ok", "The %s is on the head." % (d.get("module") or "laser"))
        door = self._door
        if not door:
            add("door", "warn", "The enclosure door has not been read yet.", "detect")
        elif door[1] == "open":
            add("door", "bad", "The enclosure door is open.")
        else:
            add("door", "ok", "The door is shut.")
        if self._thermal_state() == "enabled":
            add("thermal", "bad", "ThermalRunaway is on: it will stop the printer with the laser fitted.", "ready_for_laser")
        if not self._printer.is_operational() or self._printer.is_printing() or self._printer.is_paused():
            add("printer", "bad", "The printer is not connected, or busy.")
        if b is None:
            add("bridge", "bad", "The LightBurn bridge is off in the settings.")
            return checks
        add("lightburn", "ok" if self._grbl_client else "warn",
            "LightBurn is connected." if self._grbl_client else "LightBurn is not connected.")
        if not mode:
            if not self._settings.get_boolean(["auto_mode"]):
                add("mode", "bad", "No mode set: press %s mode." % ("Rotary" if d.get("rotary") else "Flat"))
            elif getattr(self, "_auto_paused", False):
                add("mode", "bad", "LightBurn's moves are off (you pressed Off): press %s mode to carry on."
                    % ("Rotary" if d.get("rotary") else "Flat"))
            elif getattr(self, "_auto_why", None):
                add("mode", "bad", "The mode cannot be set: %s." % self._auto_why.rstrip("."),
                    "tab:rotary" if d.get("rotary") else None)
            elif not self._homed:
                add("mode", "info", "%s mode is set automatically once the printer has been homed (G28) since it connected."
                    % ("Rotary" if d.get("rotary") else "Flat"))
            else:
                add("mode", "info", "Setting %s mode automatically (%s)." % (
                    "rotary" if d.get("rotary") else "flat", "the rotary is plugged in" if d.get("rotary") else "no rotary"))
        elif mode == "flat":
            add("mode", "warn" if getattr(ctx, "dry_run", False) else "ok",
                "Flat mode, %s." % ("a dry run (nothing burns)" if getattr(ctx, "dry_run", False) else "focus Z%.2f" % ctx.focus_z))
            hm = self._map or {}
            if self._mesh_loaded() and hm.get("source") == "red dot":
                add("surface", "info", "An object map is loaded: the head follows the object's shape.", "map_bed")
        else:
            r = self._rotary or {}
            if not r.get("complete"):
                add("rotary", "bad", "The rotary measurement is %s: measure the object again."
                    % ("stale (%s)" % r["stale"]["reason"] if r.get("stale") else "missing"), "tab:rotary")
            else:
                chk, ver = r.get("last_check") or {}, r.get("last_verify") or {}
                newest = max(chk.get("time") or 0, ver.get("time") or 0, r.get("finished") or 0)
                if time.time() - newest > 4 * 3600:
                    add("rotary", "warn", "The rotary measurement was last confirmed %d h ago: press Has the rotary "
                        "moved? (probe on)" % ((time.time() - newest) // 3600), "tab:rotary")
                else:
                    add("rotary", "ok", "The rotary measurement is current (%.2f mm)." % (r.get("summary") or {}).get("diameter", 0))
        frame = self._fresh_frame()
        if mode == "flat":
            if frame is None:
                add("frame", "info", "Press Frame in LightBurn: OctoBurn checks the job's outline against obstacles.")
            else:
                try:
                    guarded = self._active_obstacles(self._checked_loose(), ctx.focus_z)
                except Exception:                            # noqa: BLE001
                    guarded = []
                fb = frame["bounds"]
                rx0, ry0, rx1, ry1 = rotary_gcode.MODULE_REACH
                m = rotary_gcode.OBSTACLE_MARGIN
                sweep = (fb[0] - rx0 - m, fb[1] - ry0 - m, fb[2] + rx1 + m, fb[3] + ry1 + m)
                hits = [o for o in guarded if sweep[0] < o["box"][2] and o["box"][0] < sweep[2]
                        and sweep[1] < o["box"][3] and o["box"][1] < sweep[3]]
                if hits:
                    add("frame", "bad", "The framed job brings the laser module over %s: move the design."
                        % ", ".join(o.get("name") or "an obstacle" for o in hits))
                else:
                    add("frame", "ok", "The framed job (X%.0f-%.0f Y%.0f-%.0f) keeps clear of obstacles."
                        % (fb[0], fb[2], fb[1], fb[3]))
        p = (self._air.status() if self._air is not None and hasattr(self._air, "status") else {}) or {}
        if p.get("enabled") is False:
            add("air", "info", "The air purifier is off in the settings.")
        elif p.get("pm25") is None:
            add("air", "warn", "No air reading from the sensor: the purifier will run on its timer.")
        else:
            add("air", "ok", "The air purifier is ready.")
        if ctx is not None and ctx.laser_until is not None:
            add("armed", "info", "The laser is armed: LightBurn's burns fire.")
        return checks

    def _fresh_frame(self):
        b = self._grbl
        frame = b.frame() if b is not None and hasattr(b, "frame") else None
        if not frame or time.time() - frame["time"] > self.FRAME_FRESH_S:
            return None
        return frame

    def _api_focus_frame(self, data):
        """Focus at the last LightBurn Frame: the red dot measures the surface at the frame's centre,
        then flat mode is set with focus there (laser not armed)."""
        if not config_module.has_camera((self._detected or {}).get("toolhead")):
            raise ValueError("focusing from the Frame uses the red dot, which only the 10W has")
        frame = self._fresh_frame()
        if frame is None:
            raise ValueError("no Frame from LightBurn in the last 30 minutes: press Frame in LightBurn first")
        b = frame["bounds"]
        out = self._api_dot_focus(dict(x=(b[0] + b[2]) / 2.0, y=(b[1] + b[3]) / 2.0))
        self._focused_frame = frame["time"]
        return dict(out, frame=frame)

    UI_OPTIONS = ("focus_after_frame", "auto_mode")
    AUTO_MODE_RETRY_S = 60.0

    def _auto_mode_step(self):
        """No mode, nothing running: set the mode from what is fitted. Rotary plugged in: rotary
        mode on the stored measurement (if it is current). No rotary: flat, focused on the bed
        plus the last material thickness. Never arms the laser. Off pauses it until a mode is picked."""
        if (not self._settings.get_boolean(["auto_mode"]) or getattr(self, "_auto_paused", False)
                or self._grbl is None or self._grbl.context is not None or self._running()):
            return None
        d = self._detected or {}
        if not d.get("laser") or not self._printer.is_operational() or self._printer.is_printing() \
                or self._printer.is_paused() or not self._homed:
            return None
        if time.time() < getattr(self, "_auto_retry_at", 0.0):
            return None
        if time.time() - getattr(self, "_homed_at", 0.0) < 45.0:
            return None                                      # still homing: its serial replies queue behind G28
        if d.get("rotary"):
            r = self._rotary or {}
            if not r.get("complete"):
                self._auto_why = ("the rotary is plugged in but its measurement is %s: measure the object on the "
                                  "Rotary tab" % ("stale" if r.get("stale") else "missing"))
                self._auto_retry_at = time.time() + self.AUTO_MODE_RETRY_S
                return None
            data = dict(mode="rotary", auto=True)
        else:
            data = dict(mode="flat", height=float(self._settings.get(["flat_focus_height"]) or 0.0), auto=True)
        try:
            out = self._api_bridge_mode(data)
        except Exception as exception:                       # noqa: BLE001
            self._auto_why = str(exception)
            self._auto_retry_at = time.time() + self.AUTO_MODE_RETRY_S
            return None
        self._auto_why = None
        self._log_line("Mode set automatically (%s): %s" % ("the rotary is plugged in" if d.get("rotary") else "no rotary",
                                                            out.get("describe")))
        return out

    def _api_set_option(self, data):
        """A tab switch saved as a setting (only the ones listed in UI_OPTIONS)."""
        key = data.get("key")
        if key not in self.UI_OPTIONS:
            raise ValueError("not a tab option: %r" % key)
        self._settings.set_boolean([key], bool(data.get("value")))
        self._settings.save()
        return dict(ok=True, key=key, value=bool(data.get("value")))

    # -- the serial link guard (linkguard.py) ----------------------------------------------
    def _link_start(self):
        try:
            cfg = config_module.link_config(self._config())
        except SettingsError as exception:
            self._log_line("Link guard: bad settings (%s): using the defaults, no power cut." % exception)
            cfg = dict(ok_s=2.0, power_s=3.0, switch="", ha_url="", ha_token="")
        self._link_cfg = cfg
        self._link = linkguard.LinkGuard(fake_ack=self._printer.fake_ack, stop=self._link_stop,
                                         power_off=self._link_power_off, log=self._log_line,
                                         ok_s=cfg["ok_s"], power_s=cfg["power_s"])
        self._link_running = True
        threading.Thread(target=self._link_loop, name="laserheightprobe-link", daemon=True).start()

    def _link_loop(self):
        while getattr(self, "_link_running", False):
            time.sleep(0.2)
            try:
                self._link.step()
            except Exception:                                # noqa: BLE001 - never kill the guard
                self._logger.exception("link guard")

    def _link_stop(self):
        """A burn whose oks have stopped: unstick OctoPrint's queue, drop the bridge's, quickstop, beam off."""
        try:
            self._printer.fake_ack()
        except Exception:                                    # noqa: BLE001
            self._logger.exception("link guard: fake_ack")
        if self._grbl is not None:
            if self._grbl.context is not None:
                self._grbl.context.laser_until = None
            self._grbl.reset()
        self._printer.commands(["M5"])

    def _link_power_off(self):
        cfg = getattr(self, "_link_cfg", None) or {}
        if not cfg.get("switch") or not cfg.get("ha_url") or not cfg.get("ha_token"):
            self._log_line("Link guard: no emergency_power_switch (with the Home Assistant URL and token) "
                           "set: cannot cut the power. Switch the printer off by hand.")
            return
        from .air import HomeAssistant
        HomeAssistant(cfg["ha_url"], cfg["ha_token"]).switch(cfg["switch"], False)
        self._log_line("Link guard: the printer's power (%s) is switched off." % cfg["switch"])

    def _frame_watch(self):
        """With focus_after_frame on: after each new LightBurn Frame in flat mode (laser not armed,
        nothing running), measure there with the red dot and set the focus."""
        while getattr(self, "_frame_watching", False):
            time.sleep(2.0)
            try:
                self._auto_mode_step()
            except Exception:                                # noqa: BLE001 - never kill the watcher
                self._logger.exception("automatic mode")
            try:
                if not self._settings.get_boolean(["focus_after_frame"]):
                    continue
                frame = self._fresh_frame()
                b = self._grbl
                if not config_module.has_camera((self._detected or {}).get("toolhead")):
                    continue                                 # the red dot and the camera are the 10W's
                if (frame is None or frame["time"] == getattr(self, "_focused_frame", None) or b is None
                        or getattr(b.context, "mode", None) != "flat" or b.state() != "Idle"
                        or (b.context.laser_until is not None) or self._running()
                        or not self._dot_calibrated()):
                    continue
                self._focused_frame = frame["time"]
                self._log_line("A Frame from LightBurn: focusing at its centre with the red dot.")
                self._api_focus_frame({})
            except Exception as exception:                   # noqa: BLE001 - never kill the watcher
                self._log_line("Focus after the Frame failed: %s" % exception)

    def _flat_bed_focus(self, config):
        """(focus Z, where it came from) for the bare bed, for flat mode's "height above the bed".
        Refused while a red-dot object map is loaded: its base is the object, not the bed, and the
        firmware is lifting the head by the object's shape."""
        hm = self._map or {}
        if self._mesh_loaded() and hm.get("source") == "red dot":
            raise ValueError("a red-dot map of an object is loaded, so the bed's focus is not known here: "
                             "press Back to the bed map, or focus from Probe here or the red dot")
        return self._dot_bed_focus(config)

    def _flat_focus_status(self):
        try:
            z, source = self._flat_bed_focus(self._checked_loose())
            return dict(bed_z=round(z, 3), source=source)
        except Exception as exception:                       # noqa: BLE001
            return dict(error=str(exception))

    def _dot_bed_focus(self, config):
        """The focus Z (G-code) for the bare, mapped bed: the map's base focus while the map is
        loaded (the firmware adds the bed's shape), else the bed height plus the 10W's offset."""
        hm = self._map or {}
        base = hm.get("base") or {}
        if self._mesh_loaded() and base.get("focus") is not None:
            return float(base["focus"]), "the height map's base focus Z%.2f" % float(base["focus"])
        module = config_module.find_module(config["laser_modules"], "10W") or {}
        if config.get("flat_bed_z") is not None and module.get("focus_offset") is not None:
            f = config["flat_bed_z"] + module["focus_offset"]
            return f, "bed Z%.2f + 10W offset %.2f" % (config["flat_bed_z"], module["focus_offset"])
        raise ValueError("no bed focus: load the height map, or set the bed height and the 10W's focus offset")

    def _api_dot_focus(self, data):
        """Focus from the red dot: measure the surface at LightBurn (x, y), then put the bridge in
        flat mode with focus = the bed's focus + that height. The laser is NOT armed: arm it as
        usual. Refuses heights it cannot vouch for (a disagreement or no dot)."""
        config = self._checked_loose()
        bed, why = self._dot_bed_focus(config)
        measured = self._api_dot_measure(data)
        h = measured["height"]
        if h < -1.0:
            raise ValueError("the red dot read %.2f mm, below the bed: not trusted for focus" % h)
        focus = round(bed + max(h, 0.0), 2)
        mode = self._api_bridge_mode(dict(mode="flat", focus_z=focus))
        self._log_line("Focus from the red dot: %.2f mm high at LightBurn X%.1f Y%.1f, focus Z%.2f (%s + %.2f)."
                       % (h, measured["lb"][0], measured["lb"][1], focus, why, max(h, 0.0)))
        return dict(ok=True, height=h, focus_z=focus, bed_focus=bed, measured=measured, bridge=mode)

    def _api_dot_map(self, data):
        """A height map from the red dot instead of the probe: one reading from Z100 per point of
        a grid over the LightBurn box (x0, y0, x1, y1) every `step` mm, fitted to the printer's
        mesh like a probed map. Replaces the stored map (the old one is kept in
        height_map_previous.json) and, if a map is loaded in the printer, loads this one in its place
        (the two must match). Heights are taken in the current map's
        frame (the dot is calibrated on it). Moves the head (beam off, LightBurn locked out)."""
        config, model, ctx = self._closeup_setup()
        dm = self._dot_model()
        if dm is None:
            raise ValueError("calibrate the red dot first (dot_calibrate over flat paper)")
        old = self._map or {}
        base = old.get("base") or {}
        if not base or base.get("machine_x") is None:
            raise ValueError("the red-dot map is built in the frame of a probed map: make one first")
        shift = (base["machine_x"] - base["x"], base["machine_y"] - base["y"])
        x0, y0, x1, y1 = (float(data[k]) for k in ("x0", "y0", "x1", "y1"))
        x0, x1 = sorted((x0, x1))
        y0, y1 = sorted((y0, y1))
        step = float(data.get("step") or 40.0)
        if not 10.0 <= step <= 100.0:
            raise ValueError("step %.0f mm: use 10 to 100" % step)
        nx, ny = int((x1 - x0) // step) + 1, int((y1 - y0) // step) + 1
        if nx * ny > DOT_MAP_MAX_POINTS:
            raise ValueError("%d points: at most %d (a bigger step)" % (nx * ny, DOT_MAP_MAX_POINTS))
        if nx * ny < 4:
            raise ValueError("a map needs a box at least one step across both ways")
        z = float(data.get("z") or DOT_MAP_Z)
        o = ctx["o"]
        raw = self._map_raw()
        # points in the probed map's own terms: the surface's Z (G-code and machine) and its focus,
        # each the base point's plus the height above it (heightmap._base derives machine focus)
        z0, mz0, focus0 = base.get("z", 0.0), base.get("machine_z"), base.get("focus")
        targets = []
        for j in range(ny):                          # serpentine, near rows first
            row = [(x0 + i * step, y0 + j * step) for i in range(nx)]
            targets += row if j % 2 == 0 else row[::-1]
        self._closeup_settle(config, ctx)
        if self._grbl is not None:
            self._grbl.lock_out("mapping the bed with the red dot")
        points, failed = [], []
        h_guess = 0.0
        try:
            for n, (lx, ly) in enumerate(targets, 1):
                try:
                    r = self._dot_read(config, ctx, model, dm, (lx + o[0], ly + o[1]), z, h_guess)
                except (ValueError, CloseupError) as exception:
                    failed.append(dict(lb=[lx, ly], why=str(exception)))
                    continue
                bx, by = r["dot_b"]
                height = r["h"] + (float(raw(bx, by)) if raw is not None else 0.0)   # in the map's frame
                h_guess = r["h"]
                points.append(dict(x=round(bx, 2), y=round(by, 2), z=round(z0 + height, 4),
                                   focus=None if focus0 is None else round(focus0 + height, 4),
                                   machine_z=None if mz0 is None else round(mz0 + height, 4),
                                   height=round(height, 4), px=r["px"], head=r["head"], seconds=r.get("seconds")))
                self._plugin_manager.send_plugin_message(self._identifier, dict(
                    type="progress", mode="dot_map", done=n, total=len(targets),
                    message="Red-dot map: point %d of %d." % (n, len(targets))))
        finally:
            if self._grbl is not None:
                self._grbl.lock_release()
        if len(points) < max(4, len(targets) // 2):
            raise ValueError("only %d of %d red-dot readings worked: no map made (%s)"
                             % (len(points), len(targets), failed[0]["why"] if failed else ""))
        # A map of one object: the mesh holds its edge heights flat beyond the scan (heightmap
        # build plateau). Merging the old map's bed points made the fit swing -33..+37 mm at the
        # object's edge (2026-11-21), and the object alone carried its slope across the bed.
        dot_pts = list(points)
        kept = 0
        try:
            # on the printer's own grid (the old map was refitted to it), so loading needs no refit
            built = heightmap.build(points, shift, start=old.get("start"), spacing=old.get("spacing"),
                                    tolerance=config["map_tolerance"], focus_gap=config["map_focus_gap"],
                                    plateau=True)
        except heightmap.MapError as exception:
            raise ValueError(str(exception))
        # how the new map compares with the old one at each reading (the probe check)
        compare = []
        if raw is not None:
            for p in dot_pts:
                compare.append(round(p["height"] - float(raw(p["x"], p["y"])), 3))
        built.update(created=time.time(), module="10W", points=len(points), complete=not failed,
                     source="red dot", step=step, z=z, failed=failed, dot_points=len(dot_pts),
                     kept_from_map=kept)
        # the measured height above the bed where the map is zero: the map holds heights relative
        # to its lowest point, so a close-up needs this to place the object's top
        zb = built.get("base") or {}
        near = min(dot_pts, key=lambda q: (q["x"] - zb.get("x", 0.0)) ** 2 + (q["y"] - zb.get("y", 0.0)) ** 2)
        built["footprint"] = [min(q["x"] for q in dot_pts), min(q["y"] for q in dot_pts),
                              max(q["x"] for q in dot_pts), max(q["y"] for q in dot_pts)]
        tallest = max(q["height"] for q in dot_pts)
        built["zero_h"] = round(float(near["height"]) - float(heightmap.evaluate(
            built["nodes"], built["start"], built["spacing"], near["x"] + shift[0], near["y"] + shift[1],
            extrapolate=True)), 4)
        if failed:
            built["warnings"].append("%d red-dot reading%s failed and %s left out."
                                     % (len(failed), "" if len(failed) == 1 else "s",
                                        "was" if len(failed) == 1 else "were"))
        if old:
            self._write_json(MAP_PREVIOUS_FILE, old)
            if old.get("source") != "red dot":
                self._write_json(BED_MAP_FILE, old)
        if tallest > calibration.FOCAL_10W - OBSTACLE_CLEARANCE_MARGIN:
            # an object the module cannot clear at bed focus: keep it off it in flat burns (the
            # scan's footprint, in LightBurn mm, grown by a margin: the object may reach past it)
            fp, grow = built["footprint"], 10.0
            self._add_obstacle([fp[0] - o[0] - grow, fp[1] - o[1] - grow, fp[2] - o[0] + grow, fp[3] - o[1] + grow],
                               tallest, "the red-dot mapped object", "red dot", map_created=built.get("created"))
        self._map = built
        self._write_json(MAP_FILE, built)
        if self._mesh_loaded():
            # the printer holds the old map: the stored map and the loaded one must match (the
            # work-offset check subtracts the stored one), so load the new one now, or go back
            try:
                self._api_map_load({})
            except ValueError as exception:
                self._map = old
                self._write_json(MAP_FILE, old)
                raise ValueError("the red-dot map could not be loaded (%s): the old map is back" % exception)
        self._log_line("Red-dot height map: %d points, range %.3f mm, fit within %.3f mm (rms %.3f)%s%s." % (
                           len(points), built["range"], abs(built["worst"]["miss"]), built["rms"],
                           ("; against the old map %+.2f..%+.2f mm" % (min(compare), max(compare))) if compare else "",
                           ", loaded into the printer" if self._mesh_loaded() else "; load it with Load map"))
        return dict(ok=True, height_map=self._map_summary(), failed=failed, compare=compare,
                    points=len(dot_pts), kept_from_map=kept, seconds=[p.get("seconds") for p in dot_pts])

    def _api_dot_calibrate(self, data):
        """Calibrate the red dot over flat paper at LightBurn (x, y): photos at five head heights,
        a pinhole fit, and the light's ray from the camera model. clear: true confirms nothing
        but the flat sheet is under the dot and the module (the head goes down to Z100)."""
        if not data.get("clear"):
            raise ValueError("confirm the spot is flat paper with nothing taller nearby (clear: true)")
        config, model, ctx = self._closeup_setup()
        lb = (float(data["x"]), float(data["y"]))
        o = ctx["o"]
        target = (lb[0] + o[0], lb[1] + o[1])
        zs = [float(z) for z in (data.get("z") or DOT_CALIB_Z)]
        if min(zs) < 90.0:
            raise ValueError("the red dot calibrates from Z90 up")
        self._closeup_settle(config, ctx)
        if self._grbl is not None:
            self._grbl.lock_out("calibrating the red dot")
        samples = []
        try:
            for z in sorted(zs, reverse=True):          # from the top down
                r = self._dot_read(config, ctx, model, None, target, z, 0.0)
                rel = (r["dot_b"][0] - r["head"][0], r["dot_b"][1] - r["head"][1], -z)
                lift, mapped = self._dot_lift(r["head"], r["dot_b"])
                if self._map_raw() is not None and not mapped:
                    raise ValueError("calibrate the red dot where the bed map was probed (the dot and the "
                                     "head both inside it): outside it the bed's height is a guess")
                bed = self._bed_map()
                bed_here = float(self._raw_of(bed)(r["dot_b"][0], r["dot_b"][1])) if bed else 0.0
                samples.append(dict(z=z, x=r["px"][0], y=r["px"][1], lift=lift, rel=rel,
                                    dot_b=[round(float(v), 3) for v in r["dot_b"]],
                                    at_head=round(lift + bed_here, 4)))
        finally:
            if self._grbl is not None:
                self._grbl.lock_release()
        # On flat paper the dot lands on one spot at every height. If it wanders, the light was
        # caught by something taller on its way in (it comes from +X at ~36 degrees: 2026-11-22 a
        # box beside the spot gave dot X 109..118 for a target at 99, and a model 20 mm out)
        xs = [q["dot_b"][0] for q in samples]
        ys = [q["dot_b"][1] for q in samples]
        spread = max(max(xs) - min(xs), max(ys) - min(ys))
        if spread > DOT_CALIB_SPREAD:
            raise ValueError("the red dot moved %.1f mm across the heights (X%.1f..%.1f): something taller "
                             "is catching the light on its way in from the +X side. Calibrate where "
                             "nothing stands within about %.0f mm to the right of the spot"
                             % (spread, min(xs), max(xs), max(zs) * 0.75))
        dm = redot.fit(samples)
        dm.time = time.time()
        bed = self._bed_map()
        self._save_json(os.path.join(CLOSEUP_DIR, DOT_MODEL_FILE),
                        dict(dm.to_dict(), samples=samples, lb=list(lb),
                             bed_created=bed.get("created") if bed else None))
        self._log_line("Red dot calibrated at LightBurn X%.1f Y%.1f: residual %.2f px, %.3f mm per px at Z100."
                       % (lb[0], lb[1], dm.residual_px, dm.mm_per_px(100.0)))
        return dict(ok=True, model=dm.to_dict(), samples=samples, mm_per_px_z100=dm.mm_per_px(100.0))

    # -- air purifier -----------------------------------------------------

    def _air_config(self):
        try:
            config = config_module.purifier_config(self._config())
        except SettingsError as exception:
            self._logger.warning("air purifier off: %s", exception)
            config = dict(enabled=False)
        config["on_learned"] = self._air_learned
        return config

    def _air_send(self, lines):
        if self._printer.is_operational():
            self._printer.commands(list(lines), tags={AIR_TAG})

    def _air_watts(self):
        detected = self._detected or {}
        return purifier_mod.module_watts(detected.get("toolhead")) if detected.get("laser") else 0.0

    def _air_start(self):
        self._air = air_mod.AirController(self._air_config(), self._air_send, self._air_watts,
                                          logger=self._logger, log_line=self._log_line)
        saved = self._read_json(AIR_STATS_FILE)
        if saved:
            self._air.stats.update(saved)
        self._air_running = True
        # a restart may have left it running: off first (the socket; the fan if the printer is up)
        threading.Thread(target=self._air.off, args=("OctoPrint started",),
                         name="laserheightprobe-air-off", daemon=True).start()
        threading.Thread(target=self._air_loop, name="laserheightprobe-air", daemon=True).start()

    def _air_loop(self):
        saved_s, next_filter = None, time.time() + 30.0
        while getattr(self, "_air_running", False):
            time.sleep(AIR_STEP_S)
            if self._air is None:
                continue
            self._air.step()
            run_s = round(self._air.stats.get("run_s", 0.0))
            if run_s != saved_s and (saved_s is None or run_s - saved_s >= 60 or not self._air.gear):
                self._save_json(AIR_STATS_FILE, self._air.stats)
                saved_s = run_s
            if time.time() >= next_filter:
                next_filter = time.time() + AIR_FILTER_EVERY_S
                try:
                    self._air_filter_check()
                except Exception:                    # noqa: BLE001 - best effort, never in a job
                    self._logger.debug("purifier filter check skipped", exc_info=True)

    def _air_idle(self):
        """Nothing that a bare M1011 (a status read in the firmware) could get in the way of."""
        return (self._printer.is_operational() and not self._printer.is_printing()
                and not self._printer.is_paused() and not self._running()
                and (self._grbl is None or self._grbl.state() == "Idle")
                and self._air.policy.session is None)

    def _air_filter_check(self):
        if not self._air.config.get("enabled") or not self._air_idle():
            return
        info = gcode.purifier_info(self._bridge.run(["M1011"], 15.0))
        before = (self._air.filter or {}).get("lifetime")
        self._air.filter = dict(info, time=time.time()) if info else None
        now = (info or {}).get("lifetime")
        if now and now != before and now != "normal":
            self._log_line("Air purifier: the filter's life reads %s%s." % (now, ": replace it soon" if now == "low" else ""))

    def _air_stop(self, reason):
        self._air_running = False
        if self._air is not None:
            self._air.off(reason, stop_policy=True)
            self._save_json(AIR_STATS_FILE, self._air.stats)

    def _air_reconfigure(self):
        if self._air is None:
            return
        old = self._air
        config = self._air_config()
        with old.lock:
            old.config = config
            old.policy.c.update(config.get("policy") or {})
            old.policy.tau_s = config.get("tau_s")
            old.ha = air_mod.HomeAssistant(config.get("ha_url"), config.get("ha_token"))

    def _air_learned(self, tau):
        # keep a smoothed decay time: half the old, half the new. Only a plausible one: a stored
        # value outside its range would fail the settings check that every job runs.
        lo, hi = config_module.RANGES["purifier_tau_s"]
        if not lo <= tau <= min(hi, 3600.0):
            self._log_line("Air purifier: ignored a decay time of %.0f s (not plausible)." % tau)
            return
        old = self._settings.get(["purifier_tau_s"])
        new = tau if not old else 0.5 * float(old) + 0.5 * tau
        threading.Thread(target=self._store, args=(dict(purifier_tau_s=round(new, 1)),),
                         name="laserheightprobe-air-learn", daemon=True).start()

    def _api_purifier_stop(self, data):
        self._air.off("Stop pressed in OctoPrint", stop_policy=True)
        return dict(ok=True, purifier=self._air.status())

    def _api_purifier_test(self, data):
        """Power, fan at gear 1 for a few seconds, the firmware's report, off. For checking the
        wiring; the filter is precious, so it is capped."""
        seconds = min(float(data.get("seconds") or 5.0), AIR_TEST_MAX_S)
        if not self._printer.is_operational():
            raise ValueError("the printer is not connected")
        if self._air.policy.session is not None:
            raise ValueError("the purifier is following a job: Stop it first")
        ha = self._air.ha
        lines = []
        try:
            if ha.ready:
                ha.switch(self._air.config["switch"], True)
            time.sleep(2.0)
            self._printer.commands(["M1011 F1"], tags={AIR_TAG})
            time.sleep(max(0.0, seconds - 2.0))
            lines = self._bridge.run(["M1011"], 15.0)
        finally:
            self._air.off("test finished")
        report = [l for l in lines if "Purifier" in l]
        return dict(ok=True, report=report, socket=ha.ready)

    def _bridge_start(self):
        try:
            config = self._checked_loose()
        except ValueError as exception:
            self._logger.warning("LightBurn bridge not started: %s", exception)
            return
        if not config["bridge_enabled"]:
            return
        self._grbl = grbl_bridge.Bridge(_OctoSender(self), logger=self._logger,
                                        pointer_switch=self._macro_pointer,
                                        camera_trigger=self._macro_camera)
        self._grbl_stop.clear()
        try:
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("0.0.0.0", config["bridge_port"]))
            server.listen(1)
            server.settimeout(1.0)
        except (OSError, socket.error) as exception:
            self._logger.error("LightBurn bridge: cannot listen on port %s: %s", config["bridge_port"],
                               exception)
            self._grbl = None
            return
        self._grbl_server = server
        threading.Thread(target=self._bridge_accept, args=(server, config["bridge_allow"]),
                         name="laserheightprobe-grbl", daemon=True).start()
        threading.Thread(target=self._bridge_pump, name="laserheightprobe-grbl-pump",
                         daemon=True).start()
        self._burns_start()
        self._logger.info("LightBurn bridge listening on port %s for %s", config["bridge_port"],
                          ", ".join(config["bridge_allow"]))

    def _bridge_stop(self):
        self._grbl_stop.set()
        if self._grbl_server is not None:
            try:
                self._grbl_server.close()
            except (OSError, socket.error):
                pass
            self._grbl_server = None

    def _bridge_pump(self):
        while not self._grbl_stop.is_set():
            bridge = self._grbl
            if bridge is not None and self._printer.is_operational():
                try:
                    bridge.pump()
                except Exception:                              # noqa: BLE001
                    self._logger.exception("LightBurn bridge pump")
            self._grbl_stop.wait(0.02)

    def _bridge_accept(self, server, allow):
        while not self._grbl_stop.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except (OSError, socket.error):
                return
            if addr[0] not in allow:
                self._logger.warning("LightBurn bridge: refused %s (not in bridge_allow)", addr[0])
                conn.close()
                continue
            if self._grbl_client is not None:
                self._logger.warning("LightBurn bridge: refused %s (%s is connected)", addr[0],
                                     self._grbl_client)
                conn.close()
                continue
            threading.Thread(target=self._bridge_serve, args=(conn, addr[0]),
                             name="laserheightprobe-grbl-client", daemon=True).start()

    def _bridge_serve(self, conn, who):
        self._grbl_client = who
        self._log_line("LightBurn connected from %s." % who)
        lock = threading.Lock()

        def write(text):
            with lock:
                conn.sendall(text.encode("ascii", "replace"))
        session = grbl_bridge.GrblSession(self._grbl, write, logger=self._logger)
        try:
            session.start()
            conn.settimeout(1.0)
            while not self._grbl_stop.is_set() and session.alive:
                try:
                    data = conn.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                session.receive(data)
        except (OSError, socket.error) as exception:
            self._logger.info("LightBurn bridge connection: %s", exception)
        finally:
            session.close()                                    # stop and laser off
            try:
                conn.close()
            except (OSError, socket.error):
                pass
            self._grbl_client = None
            self._log_line("LightBurn disconnected (%s): stopped, laser off." % who)

    def _bridge_disarm(self, why):
        if self._grbl is not None and self._grbl.context is not None:
            self._grbl.set_context(None)
            self._log_line("LightBurn bridge disarmed: %s." % why)

    def _work_offset(self):
        """(dx, dy, dz) of any G92 / work origin beyond the home offsets (M206), or raise."""
        lines = self._bridge.run(["M503"], 30.0)
        shift = gcode.work_offset(lines)
        if shift is None:
            raise ValueError("cannot read the step counters and home offsets (M114, M503), so a work "
                             "offset cannot be ruled out")
        # With the height map loaded the step count holds the levelling at the head too; that is
        # the map, not a work offset.
        if self._mesh and self._mesh.get("loaded_at") and self._map:
            steps_xy = gcode.xy_steps_per_mm(lines)
            mxy = gcode.machine_xy(lines, steps_xy) if steps_xy else None
            if mxy is None:
                raise ValueError("cannot read the head's machine X, Y, so the height map's levelling "
                                 "cannot be told from a work offset")
            dz = heightmap.evaluate(self._map["nodes"], self._map["start"], self._map["spacing"], mxy[0], mxy[1],
                                    extrapolate=True)
            shift = (shift[0], shift[1], round(shift[2] - dz, 3))
        return shift

    def _api_bridge_mode(self, data):
        """Pick what LightBurn drives: rotary (the stored rotary measurement), flat (one focus Z
        from the last Probe here, or focus_z), or off. laser: true arms the laser for
        bridge_arm_minutes. The mode is bound to that measurement: a new one disarms it."""
        from . import rotary_gcode
        if self._grbl is None:
            raise ValueError("the LightBurn bridge is off: set bridge_enabled and restart OctoPrint")
        mode = str(data.get("mode") or "")
        if mode == "off":
            self._bridge_disarm("turned off")
            if not data.get("auto"):
                self._auto_paused = True                     # Off means off: no automatic mode until one is picked
            return dict(ok=True, mode=None)
        if self._running():
            raise ValueError("a probe job is running")
        if not data.get("auto"):
            self._auto_paused = False
        if self._printer.is_printing() or self._printer.is_paused():
            raise ValueError("the printer is busy with a job")
        self._detect()                                     # picks the module M1006 names
        config = self._checked_loose()
        self._mesh_guard()                                 # a home since the map was loaded can fade it out
        shift = self._work_offset()
        # With the height map loaded the firmware's correction at the head can differ from the
        # map's by up to ~0.16 mm (seen 2026-11-20: its grid sits slightly elsewhere); a real Z
        # work offset (LightBurn's G92 Z0) is millimetres.
        z_tolerance = MESH_Z_TOLERANCE if (self._mesh and self._mesh.get("loaded_at")) else 0.02
        if max(abs(shift[0]), abs(shift[1])) > 0.02 or abs(shift[2]) > z_tolerance:
            raise ValueError("a work offset is in force (X%+.2f Y%+.2f Z%+.2f): home with G28 X Y Z "
                             "first" % shift)
        module = config_module.find_module(config["laser_modules"], config["selected_module"])
        if module is None:
            raise ValueError("pick the laser module in the OctoBurn settings first")
        offset = module["focus_offset"]
        if module["dx"] is None:
            raise ValueError("the %s beam offset (dx, dy) is not measured: the bridge cannot put its "
                             "beam where the probe measured" % module["name"])
        why = config_module.toolhead_mismatch(module["name"],
                                              gcode.toolhead(self._bridge.run(["M1006"], 30.0)))
        if why:
            raise ValueError(why)
        why = rotary_mismatch(mode, self._bridge.run(["M114"], 30.0))
        if why:
            raise ValueError(why)
        if data.get("laser"):
            self._door_check()
            self._laser_on_head()
        laser_until = (time.monotonic() + 60.0 * config["bridge_arm_minutes"]) if data.get("laser") else None
        seg = config["bridge_seg_mm"]
        if mode == "rotary":
            if not self._rotary or not self._rotary.get("complete"):
                stale = (self._rotary or {}).get("stale")
                raise ValueError("the rotary measurement is stale (%s): run the rotary routine again"
                                 % stale["reason"] if stale else
                                 "there is no complete rotary measurement: run the rotary routine first")
            surface = rotary_gcode.Surface(self._rotary, offset)
            rotary_gcode.check_offset(offset)
            summary = self._rotary["summary"]
            y_origin = float(data["y_origin"]) if data.get("y_origin") is not None else summary["body_from_y"]
            describe = ("rotary: diameter %.2f mm (LightBurn's rotary diameter and dia= must match), "
                        "body Y%.1f to Y%.1f, top Z%.3f, %s focus %+.2f, LightBurn X0 = Y%.2f" % (
                            2 * surface.r, summary["body_from_y"], summary["body_to_y"], summary["top_z"],
                            module["name"], offset, y_origin))
            b_shift = float(self._b_frame.get("shift") or 0.0)
            if abs(b_shift) > 1e-6:
                describe += (", B shifted %+.2f deg since the measurement (a power cycle reset the "
                             "rotary's count)" % b_shift)
            context = grbl_bridge.RotaryContext(surface, y_origin, rotary_gcode.Y_SIGN, rotary_gcode.B_SIGN,
                                                seg_max=seg, beam_dx=module["dx"], beam_dy=module["dy"],
                                                laser_until=laser_until, describe=describe,
                                                module=module["name"], b_shift=b_shift)
        elif mode == "flat":
            if data.get("focus_z") is not None:
                focus_z = float(data["focus_z"])
                source = "focus Z given"
            elif data.get("height") is not None:
                # the bed's focus plus the work's height above the bed (0: on the bed)
                height = config_module.number("closeup_height", data.get("height"))
                bed_z, bed_source = self._flat_bed_focus(config)
                if not data.get("auto"):
                    self._store(dict(flat_focus_height=height))   # the automatic mode uses it next time
                focus_z = bed_z + height
                source = "%s + %.1f mm above the bed" % (bed_source, height) if height else bed_source
            else:
                points = self._here.get("points") or []
                if not points:
                    raise ValueError("no Probe here result: probe the top of the work first")
                p = points[-1]
                age = time.time() - p["time"] if p.get("time") else None
                if age is None or age > 3600:
                    raise ValueError("the last Probe here is %s: probe the top of the work again"
                                     % ("too old" if age else "from before the bridge"))
                if offset is None:
                    raise ValueError("the %s focus offset is not set" % (module["name"] if module else "module"))
                machine_z = p.get("machine_z", p.get("z"))
                focus_z = machine_z + offset
                source = "Probe here X%.1f Y%.1f trigger Z%.3f + %+.2f" % (p["x"], p["y"], machine_z, offset)
            lo, hi = self._soft_limits()
            ox, oy = self._beam_offset(config, module)
            bounds = (lo[0] - ox, lo[1] - oy, hi[0] - ox, hi[1] - oy)
            describe = "flat: focus Z%.3f (%s), beam offset X%+.1f Y%+.1f, LightBurn X%g..%g Y%g..%g" % (
                focus_z, source, ox, oy, bounds[0], bounds[2], bounds[1], bounds[3])
            guarded = self._active_obstacles(config, focus_z)
            if guarded:
                describe += ", %d obstacle%s guarded" % (len(guarded), "" if len(guarded) == 1 else "s")
            context = grbl_bridge.FlatContext(focus_z, ox, oy, bounds=bounds, seg_max=seg,
                                              backlash_x=config["bridge_backlash_x"], obstacles=guarded,
                                              laser_until=laser_until, describe=describe,
                                              module=module["name"])
        else:
            raise ValueError("mode must be rotary, flat or off")
        context.mode_name = mode
        if data.get("dry_run"):
            context.dry_run = True
            context.describe += " (DRY RUN: every burn at S0)"
            describe = context.describe
        self._grbl.set_context(context)
        self._log_line("LightBurn bridge: %s%s." % (describe, ", LASER ARMED for %d min"
                                                    % config["bridge_arm_minutes"] if laser_until else ""))
        return dict(ok=True, mode=mode, describe=describe, laser=bool(laser_until))

    @staticmethod
    def _beam_offset(config, module):
        """(ox, oy): head = LightBurn + o for the beam to land there. bridge_offset is
        LightBurn's origin for a beam on the probe pin; each module's beam sits dx, dy away from
        the pin. The flat bridge and the close-up camera both use it, so a close-up is in the
        coordinates LightBurn burns in."""
        if module is None or module.get("dx") is None or module.get("dy") is None:
            raise ValueError("the %s beam offset (dx, dy) is not measured"
                             % (module["name"] if module else "laser module"))
        return config["bridge_offset_x"] + module["dx"], config["bridge_offset_y"] + module["dy"]

    def _soft_limits(self):
        """((min x, min y), (max x, max y)) from M211, or raise: a flat job must stay inside them
        (the firmware would clamp a move silently and distort the burn)."""
        import re
        lines = self._bridge.run(["M211"], 15.0)
        text = " ".join(lines)
        lo = re.search(r"Min:\s*X\s*(-?[\d.]+)\s*Y\s*(-?[\d.]+)", text)
        hi = re.search(r"Max:\s*X\s*(-?[\d.]+)\s*Y\s*(-?[\d.]+)", text)
        if not lo or not hi:
            raise ValueError("cannot read the soft endstop limits (M211)")
        return (float(lo.group(1)), float(lo.group(2))), (float(hi.group(1)), float(hi.group(2)))

    def _door_seen(self, state):
        """The enclosure door changed (or was reported). Open while the purifier runs: stop it
        (the smoke leaves by the door; the filter is the cost). Open while the laser is armed: disarm
        now. The firmware also drops the laser to a safe limit and pauses its own jobs, but a
        LightBurn job streams on through the bridge."""
        before = self._door[1] if self._door else None
        self._door = (time.time(), state)
        if state == "open" and before != "open" and self._air is not None:
            st = self._air.status()
            if st.get("power") or st.get("fan_gear"):
                # an open door lets the smoke out anyway: the filter is the cost, so stop
                threading.Thread(target=self._air.off, args=("the enclosure door opened",),
                                 kwargs=dict(stop_policy=True), name="laserheightprobe-air-door",
                                 daemon=True).start()
        pending = getattr(self, "_burn_pending", None)
        if state == "open" and before != "open" and pending is not None and self._running():
            # an automatic close-up moves the head by itself: stop it when the door opens
            self._log_line("The door opened: stopping the automatic close-up.")
            self._stop_job("the enclosure door opened")
        if state == "open" and before != "open":
            ctx = self._grbl.context if self._grbl is not None else None
            if ctx is not None and getattr(ctx, "laser_until", None):
                ctx.laser_until = None
                self._log_line("Enclosure door opened: laser disarmed.")
                self._plugin_manager.send_plugin_message(self._identifier, dict(
                    type="refused", message="The enclosure door opened: the laser is disarmed. Shut it and arm again."))

    def _door_check(self):
        """Refuse while the enclosure door is open (M1010). No enclosure: allowed."""
        state = gcode.enclosure_door(self._bridge.run(["M1010"], 30.0))
        if state is not None:
            self._door = (time.time(), state)
        if state == "open":
            raise ValueError("the enclosure door is open: shut it before the laser is armed")
        return state

    def _laser_on_head(self):
        """Ask the laser where it is (M2002 S1) and refuse unless it can fire safely: its safety
        state must be 0. The 10W reports 0x4 (tilt) lying in the bay, and the firmware's inline
        G1 S path may not check the state, so the plugin does. Arming also needs the door shut
        (_door_check); the pointer does not (the firmware caps the beam while the door is open)."""
        d = self._detect()
        if not d["laser"]:
            raise ValueError("no laser is fitted (M1006 says \"%s\")" % d["toolhead"])
        sec = d["security"]
        if sec is not None and sec.get("state") != 0:
            why = {4: "it is tilted (roll %s, pitch %s): it is not on the head" % (sec.get("roll"), sec.get("pitch")),
                   2: "it is over temperature (%s C)" % sec.get("temp")}.get(sec["state"], "")
            raise ValueError("the laser's safety state is 0x%x%s: it must be 0 before it fires"
                             % (sec["state"], ", " + why if why else ""))
        return d

    def _api_bridge_pointer(self, data):
        """The pointer on or off: the 10W's own beam capped at pointer_s_max, or the crosshair
        (2W IR) with the head shifted by its offset, for LightBurn's Fire button and framing."""
        if self._grbl is None or self._grbl.context is None:
            raise ValueError("pick a bridge mode first")
        if not data.get("on"):
            self._grbl.set_pointer(None)
            self._log_line("LightBurn bridge: pointer off.")
            return dict(ok=True, pointer=None)
        config = self._checked_loose()
        d = self._laser_on_head()
        if data.get("cross_only") and d["toolhead"] not in config_module.CROSSHAIR:
            raise ValueError("the %s has no crosshair: its pointer is its own beam, switched on in "
                             "OctoPrint (OctoBurn tab, Burn)" % d["toolhead"])
        until = time.monotonic() + 60.0 * config["pointer_minutes"]
        if d["toolhead"] in config_module.CROSSHAIR:
            offset = gcode.crosshair_offset(self._bridge.run(["M2002 T8"], 30.0))
            if offset is None:
                raise ValueError("the %s did not report its crosshair offset (M2002 T8)" % d["toolhead"])
            pointer = dict(kind="cross", dx=offset[0], dy=offset[1], until=until)
            text = "crosshair on, head shifted by X%+.2f Y%+.2f" % (-offset[0], -offset[1])
        else:
            pointer = dict(kind="spot", s_max=config["pointer_s_max"], until=until)
            text = "beam pointer, capped at S%d of 255" % config["pointer_s_max"]
        self._grbl.set_pointer(pointer)
        self._log_line("LightBurn bridge: pointer on (%s) for %d min." % (text, config["pointer_minutes"]))
        return dict(ok=True, pointer=dict(pointer, until=None))

    def _macro_pointer(self, on):
        """LightBurn's crosshair macro (M2002 T3 P1 / P0 through the bridge): the crosshair pointer
        on or off. Crosshair heads only: the 10W's pointer is its beam and stays an OctoPrint switch."""
        if not on and (self._grbl is None or self._grbl.context is None):
            return
        self._api_bridge_pointer(dict(on=on, cross_only=True))

    # -- the 10W head camera --------------------------------------------------

    HEADCAM_FILE = "headcam.jpg"

    def _headcam_path(self):
        return os.path.join(self.get_plugin_data_folder(), self.HEADCAM_FILE)

    def _macro_camera(self, variant=0):
        """LightBurn's camera macros through the bridge: M240 takes the overview photo, M240 P1
        does the last close-up again (in the background: the macro returns at once)."""
        if variant == 1:
            # only a fresh request, never one from before a restart or a new overview: its height
            # was stated for the bed as it was then
            req = self._closeup_request
            if not req:
                raise ValueError("no close-up to repeat: draw a box on the Close-up camera section in OctoPrint")
            minutes = self._checked_loose()["closeup_repeat_minutes"]
            if time.time() - req.get("time", 0) > 60 * minutes:
                raise ValueError("the last close-up request is older than %d min: draw the box again in OctoPrint"
                                 % minutes)
            self._api_closeup(dict((k, req[k]) for k in ("box", "height", "standoff")))
            return
        self._api_head_photo({})
        # the close-up image's base is the overview: rebuild it on the new one (in the
        # background; the macro returns at once)
        if self._closeup_tiles or os.path.exists(os.path.join(self.get_plugin_data_folder(), CLOSEUP_DIR, "last",
                                                              "tiles.json")):
            threading.Thread(target=self._recompose_quietly, name="laserheightprobe-recompose", daemon=True).start()

    def _api_head_photo(self, data):
        """The 10W's camera photo: up to headcam_z first (above anything on the bed or the
        rotary), then over to the photo spot, then the touchscreen takes it (headcam.py). The
        JPEG is kept for LightBurn's network camera (GET /plugin/laserheightprobe/headcam.jpg).
        Never G53/G54: the plugin's coordinates are the ones the touchscreen moves in, and G54
        holds an old Luban work origin. With closeup_base = "usb" the overview is the fixed bed
        camera's photo instead (_bedcam_overview)."""
        if self._checked_loose()["closeup_base"] == "usb":
            return self._bedcam_overview()
        if self._running():
            raise ValueError("a probe job is running")
        if self._printer.is_printing() or self._printer.is_paused():
            raise ValueError("the printer is busy with a job")
        if self._grbl is not None and self._grbl.state() != "Idle":
            raise ValueError("LightBurn is running a job or a jog: take the photo when it is idle")
        raw = self._config()
        url = str(raw.get("headcam_url") or "").strip().rstrip("/")
        if not url:
            raise ValueError("set the touchscreen's address in the settings first (headcam_url, "
                             "for example http://192.168.1.60:8080)")
        config = self._checked_loose()
        head = gcode.toolhead(self._bridge.run(["M1006"], 30.0))
        if not config_module.has_camera(head):
            raise ValueError("the %s has no camera: the head camera is on the 10W laser"
                             % (head or "fitted head"))
        x, y, z = config["headcam_x"], config["headcam_y"], config["headcam_z"]
        a = config["closeup_approach_mm"]                 # from -X, -Y: the X axis has backlash
        lines = self._bridge.run(["G90", "G0 Z%.1f F1500" % z, "G0 X%.1f Y%.1f F4000" % (x - a, y - a),
                                  "G0 X%.1f Y%.1f F4000" % (x, y)], 120.0)
        where = gcode.position(lines)
        if where is None or max(abs(where[0] - x), abs(where[1] - y), abs(where[2] - z)) > 0.5:
            raise ValueError("the head did not reach the photo spot X%.1f Y%.1f Z%.1f (M114: %s)"
                             % (x, y, z, where))
        jpeg = headcam.capture(self._headcam_get, url, x, y, z, quality=config["headcam_quality"])
        after = gcode.position(self._bridge.run([], 15.0))
        if after is not None and max(abs(after[0] - x), abs(after[1] - y)) > 0.5:
            self._log_line("Head photo: the touchscreen moved the head to X%.1f Y%.1f." % after[:2])
        self._headcam_jpeg = jpeg
        self._headcam = dict(bytes=len(jpeg), time=time.time())
        self._headcam_save(jpeg)
        self._headcam_pose = dict(x=where[0], y=where[1], z=where[2], quality=config["headcam_quality"],
                                  time=self._headcam["time"], mesh=self._mesh_loaded())
        self._closeup_request = None              # the bed may have changed: M240 P1 must not repeat
        self._save_json(HEADCAM_POSE_FILE, self._headcam_pose)
        self._log_line("Head photo taken at X%.1f Y%.1f Z%.1f (%d kB)." % (x, y, z, len(jpeg) // 1024))
        return dict(ok=True, bytes=len(jpeg), time=self._headcam["time"])

    def _headcam_save(self, jpeg):
        """Keep a disk copy for after a restart. LightBurn is served from memory, because on
        Windows a file that is still being read cannot be replaced (WinError 5); a copy that
        cannot be written only costs the photo after a restart, so it never fails the photo."""
        path = self._headcam_path()
        try:
            with open(path + ".part", "wb") as handle:
                handle.write(jpeg)
            os.replace(path + ".part", path)
        except OSError as exception:
            self._logger.warning("laserheightprobe: head photo not saved to %s: %s", path, exception)
            try:
                os.remove(path + ".part")
            except OSError:
                pass

    def _headcam_bytes(self):
        jpeg = getattr(self, "_headcam_jpeg", None)
        if jpeg is None:
            try:
                with open(self._headcam_path(), "rb") as handle:
                    jpeg = handle.read()
            except (OSError, TypeError, AttributeError):
                return None
            self._headcam_jpeg = jpeg
        return jpeg

    def _headcam_status(self):
        cached = getattr(self, "_headcam", None)
        if cached:
            return cached
        try:
            st = os.stat(self._headcam_path())
        except (OSError, TypeError, AttributeError):
            return None
        return dict(bytes=st.st_size, time=st.st_mtime)

    # -- the fixed USB bed camera (closeup_base = "usb") ------------------------

    def _bedcam_model(self):
        if self._bedcam_cache is None:
            data = self._read_json(os.path.join(CLOSEUP_DIR, BEDCAM_MODEL_FILE))
            if data:
                try:
                    self._bedcam_cache = bedcam.BedcamModel.from_dict(data)
                except Exception as exception:               # noqa: BLE001
                    self._logger.warning("laserheightprobe: bed camera calibration not read: %s", exception)
        return self._bedcam_cache

    def _bedcam_candidate(self):
        data = self._read_json(os.path.join(CLOSEUP_DIR, BEDCAM_CANDIDATE_FILE))
        return bedcam.BedcamModel.from_dict(data) if data else None

    def _bedcam_pose(self):
        if self._bedcam_pose_cache is None:
            self._bedcam_pose_cache = self._read_json(os.path.join(CLOSEUP_DIR, BEDCAM_POSE_FILE))
        return self._bedcam_pose_cache

    def _bedcam_bytes(self):
        if self._bedcam_jpeg is None:
            try:
                with open(os.path.join(self.get_plugin_data_folder(), CLOSEUP_DIR, BEDCAM_PHOTO_FILE), "rb") as handle:
                    self._bedcam_jpeg = handle.read()
            except (OSError, TypeError, AttributeError):
                return None
        return self._bedcam_jpeg

    def _bedcam_fetch(self, config, sleep=None):
        """One snapshot from the bed camera (go2rtc's frame.jpeg), a moment after the move: the
        camera's stream lags the machine."""
        (sleep or time.sleep)(self._bedcam_settle_s)
        url = config["bedcam_url"]
        status, body = self._bedcam_get(url, 15.0)
        if status != 200 or not body.startswith(headcam.JPEG):
            raise headcam.HeadcamError("the bed camera at %s gave no photo (HTTP %s)" % (url, status))
        return body

    def _bedcam_refusals(self, config, need_model=True):
        """What must hold before the head parks for the bed camera, checked before anything
        moves. Returns the calibration (None if there is none and need_model is False)."""
        if not config["bedcam_url"]:
            raise ValueError("set the bed camera's address in the settings first (bedcam_url, for example "
                             "http://octopi.local:1984/api/frame.jpeg?src=bed_cam)")
        x, y = config["bedcam_park_x"], config["bedcam_park_y"]
        online, _ = gcode.rotary_online(self._bridge.run(["M114"], 30.0))
        if online:
            raise ValueError("the rotary is fitted: the bed camera's park pose (X%.0f Y%.0f) is over it" % (x, y))
        model = self._bedcam_model()
        if model is None and need_model:
            raise ValueError("calibrate the bed camera first (bedcam_calibrate: there is no closeup/%s yet)"
                             % BEDCAM_MODEL_FILE)
        if model is not None and abs(model.bed_y - y) > BEDCAM_Y_TOLERANCE:
            raise ValueError("the bed camera was calibrated with the bed at Y%.1f, but bedcam_park_y is %.1f: "
                             "set it back or calibrate again" % (model.bed_y, y))
        lo, hi = self._soft_limits()
        if not (lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1]):
            raise ValueError("the bed camera's park pose X%.1f Y%.1f is outside the soft limits (X%.0f to %.0f, "
                             "Y%.0f to %.0f)" % (x, y, lo[0], hi[0], lo[1], hi[1]))
        shift = self._work_offset()
        z_tolerance = MESH_Z_TOLERANCE if self._mesh_loaded() else 0.02
        if max(abs(shift[0]), abs(shift[1])) > 0.02 or abs(shift[2]) > z_tolerance:
            raise ValueError("a work offset is in force (X%+.2f Y%+.2f Z%+.2f): home with G28 first" % shift)
        return model

    def _bedcam_guard(self, config, need_model=True):
        """Everything a bed camera photo on its own needs (the overview, a calibration), in order."""
        if self._running():
            raise ValueError("a job is running")
        if not self._printer.is_operational():
            raise ValueError("the printer is not connected")
        if self._printer.is_printing() or self._printer.is_paused():
            raise ValueError("the printer is busy with a job")
        if self._grbl is not None and self._grbl.state() != "Idle":
            raise ValueError("LightBurn is running a job or a jog: take the photo when it is idle")
        if not self._homed:
            raise ValueError("the head's position is not known since the printer connected: home it first (G28)")
        return self._bedcam_refusals(config, need_model)

    def _bedcam_park_photo(self, config):
        """Park for the bed camera and take its photo: beam off and LightBurn locked out, up to
        the travel height before any XY move, the park Y reached from -Y (backlash), the position
        checked (M114) before and after the photo, then back over where the head was, staying at
        the travel height. Returns (jpeg, (x, y, z) where it was taken)."""
        x, y, z = config["bedcam_park_x"], config["bedcam_park_y"], config["bedcam_park_z"]
        a = config["closeup_approach_mm"]
        lo, _ = self._soft_limits()
        start = gcode.position(self._bridge.run([], 15.0))
        if start is None:
            raise ValueError("cannot read the head position (M114)")
        travel = max(z, start[2], closeup_plan.TRAVEL_FLOOR)
        if self._grbl is not None:
            self._grbl.lock_out("parking for the bed camera photo")
        try:
            moves = ["M5", "G90", "G0 Z%.1f F1500" % travel]
            if a > 0 and y - a >= lo[1]:
                moves.append("G0 X%.1f Y%.1f F4000" % (x, y - a))
            moves.append("G0 X%.1f Y%.1f F4000" % (x, y))
            if z < travel:
                moves.append("G0 Z%.1f F1500" % z)
            where = gcode.position(self._bridge.run(moves, 120.0))
            if where is None or max(abs(where[0] - x), abs(where[1] - y)) > BEDCAM_Y_TOLERANCE or abs(where[2] - z) > 0.5:
                raise ValueError("the head did not reach the bed camera's park pose X%.1f Y%.1f Z%.1f (M114: %s)"
                                 % (x, y, z, where))
            jpeg = self._bedcam_fetch(config)
            after = gcode.position(self._bridge.run([], 15.0))
            if after is None or abs(after[1] - where[1]) > BEDCAM_Y_TOLERANCE:
                raise ValueError("the bed moved during the bed camera photo (M114: %s)" % (after,))
            self._bridge.run(["G0 X%.1f Y%.1f F4000" % (start[0], start[1])], 120.0)
        finally:
            if self._grbl is not None:
                self._grbl.lock_release()
                try:
                    self._grbl.resync()
                except Exception:                            # noqa: BLE001
                    self._logger.warning("could not read the head position after the bed camera photo")
        return jpeg, tuple(where)

    def _keep_bedcam(self, jpeg, pose, model):
        """A new bed camera overview: the close-up base while closeup_base is "usb"."""
        self._bedcam_jpeg = jpeg
        self._write_bytes(os.path.join(CLOSEUP_DIR, BEDCAM_PHOTO_FILE), jpeg)
        self._bedcam_pose_cache = dict(x=pose[0], y=pose[1], z=pose[2], time=time.time(), source="usb",
                                       bed_y=model.bed_y if model else None,
                                       calibration=model.created if model else None)
        self._save_json(os.path.join(CLOSEUP_DIR, BEDCAM_POSE_FILE), self._bedcam_pose_cache)

    def _bedcam_overview(self):
        """The overview from the fixed bed camera (head_photo, LightBurn's M240 with
        closeup_base = "usb"): park, photograph, back."""
        config = self._checked_loose()
        model = self._bedcam_guard(config)
        jpeg, where = self._bedcam_park_photo(config)
        self._keep_bedcam(jpeg, where, model)
        self._closeup_request = None              # the bed may have changed: M240 P1 must not repeat
        self._log_line("Bed camera photo taken with the head parked at X%.1f Y%.1f Z%.1f (%d kB)."
                       % (where[0], where[1], where[2], len(jpeg) // 1024))
        return dict(ok=True, bytes=len(jpeg), time=self._bedcam_pose_cache["time"], source="usb")

    @staticmethod
    def _bedcam_summary(model):
        if model is None:
            return None
        return dict(source=model.source, created=model.created, bed_y=model.bed_y, residuals=model.residuals,
                    size=[model.width, model.height], lens=model.K is not None, name=model.name)

    def _api_bedcam_calibrate(self, data):
        """The bed camera's calibration (bedcam.py). Every result is a candidate
        (closeup/bedcam_candidate.json) until accepted.
        - lightburn: one of LightBurn's Cameras[] entries, or prefs: the text (or JSON) of LightBurn's
          prefs.ini with camera: its name ("Bed cam - fixed" by default). Moves nothing. Unverified:
          check it with crosses.
        - crosses: true. Parks, photographs the burned crosses and finds them near where the
          calibration (else the candidate, else px: a rough photo pixel for each) puts them;
          reports predicted against found, then fits the homography to them (keeping that
          calibration's lens unless lens is false). crosses_lb: other crosses ([[x, y], ...] mm).
        - accept: true. The candidate becomes the calibration."""
        data = data or {}
        config = self._checked_loose()
        if data.get("accept"):
            cand = self._read_json(os.path.join(CLOSEUP_DIR, BEDCAM_CANDIDATE_FILE))
            if not cand:
                raise ValueError("no bed camera calibration to accept: run bedcam_calibrate first")
            model = bedcam.BedcamModel.from_dict(cand)
            if abs(model.bed_y - config["bedcam_park_y"]) > BEDCAM_Y_TOLERANCE:
                raise ValueError("the candidate was made with the bed at Y%.1f, but bedcam_park_y is %.1f"
                                 % (model.bed_y, config["bedcam_park_y"]))
            self._save_json(os.path.join(CLOSEUP_DIR, BEDCAM_MODEL_FILE), cand)
            self._bedcam_cache = None
            self._closeup_base = None
            self._log_line("Bed camera calibration accepted (%s)." % model.source)
            return dict(ok=True, calibration=self._bedcam_summary(self._bedcam_model()))
        if data.get("lightburn") or data.get("prefs"):
            camera = data.get("lightburn")
            if not camera:
                prefs = data["prefs"]
                if isinstance(prefs, str):
                    try:
                        prefs = json.loads(prefs)
                    except ValueError as exception:
                        raise ValueError("prefs is not LightBurn's prefs.ini (JSON): %s" % exception)
                camera = bedcam.find_lightburn_camera(prefs, data.get("camera") or "Bed cam - fixed")
            bed_y = config_module.number("bedcam_park_y", data.get("bed_y", config["bedcam_park_y"]))
            model = bedcam.from_lightburn(camera, bed_y)
            self._save_json(os.path.join(CLOSEUP_DIR, BEDCAM_CANDIDATE_FILE), model.to_dict())
            self._log_line("Bed camera: LightBurn's calibration read as a candidate (%s)." % model.name)
            return dict(ok=True, candidate=self._bedcam_summary(model))
        if not data.get("crosses"):
            raise ValueError("bedcam_calibrate needs lightburn, prefs, crosses or accept")
        from . import cvio
        crosses = [tuple(float(v) for v in c) for c in (data.get("crosses_lb") or bedcam.CROSSES_LB)]
        guess = self._bedcam_model() or self._bedcam_candidate()
        px = data.get("px")
        if px is not None and len(px) != len(crosses):
            raise ValueError("px needs one rough photo pixel for each of the %d crosses" % len(crosses))
        if guess is None and px is None:
            raise ValueError("nothing to find the crosses with: read LightBurn's calibration first "
                             "(lightburn or prefs), or give px")
        if guess is not None and abs(guess.bed_y - config["bedcam_park_y"]) > BEDCAM_Y_TOLERANCE:
            raise ValueError("the calibration to start from was made with the bed at Y%.1f, but bedcam_park_y "
                             "is %.1f" % (guess.bed_y, config["bedcam_park_y"]))
        search = float(data.get("search_mm") or 15.0)
        self._bedcam_guard(config, need_model=False)
        jpeg, where = self._bedcam_park_photo(config)
        self._write_bytes(os.path.join(CLOSEUP_DIR, BEDCAM_CALIB_PHOTO_FILE), jpeg)
        img = cvio.decode(jpeg)
        height, width = img.shape[:2]
        if guess is not None and (width, height) != (guess.width, guess.height):
            raise ValueError("the bed camera photo is %d x %d, but the calibration is for %d x %d"
                             % (width, height, guess.width, guess.height))
        if px is None:
            u, v, ok = guess.lb_to_px([c[0] for c in crosses], [c[1] for c in crosses])
            rough = [(float(a), float(b)) if k else None for a, b, k in zip(u, v, ok)]
        else:
            rough = [tuple(float(c) for c in p) if p else None for p in px]
        found = []
        for uv in rough:
            hit = cvio.find_cross(img, uv, bedcam.PX_PER_MM, search) if uv is not None else None
            found.append(hit if hit is not None and 0 <= hit[0] < width and 0 <= hit[1] < height else None)
        out = dict(pose=list(where), rough_px=[None if r is None else [round(r[0], 1), round(r[1], 1)] for r in rough],
                   scores=[None if f is None else round(f[2], 2) for f in found])
        if guess is not None:
            out["before"] = bedcam.check(guess, crosses, found)
        lens = None
        if guess is not None and guess.K is not None and data.get("lens", True) is not False:
            lens = (guess.K, guess.dist)
        model, report = bedcam.fit(crosses, found, width, height, where[1], lens)
        self._save_json(os.path.join(CLOSEUP_DIR, BEDCAM_CANDIDATE_FILE), model.to_dict())
        out.update(after=report, candidate=self._bedcam_summary(model))
        self._log_line("Bed camera: %d crosses found; the fitted candidate is within %.2f mm of them (RMS %.2f, "
                       "%d spare equations)%s." % (report["n"], report["max_mm"], report["rms_mm"], report["spare"],
                                                   "" if guess is None else "; the calibration it started from was "
                                                   "%.2f mm out" % (out["before"]["max_mm"] or 0.0)))
        return out

    @octoprint.plugin.BlueprintPlugin.route("/headcam.jpg", methods=["GET"])
    def headcam_image(self):
        """The last head photo, for LightBurn's network camera. It never moves the head: LightBurn
        polls this many times a second. Open without a login (LightBurn cannot send one), like the
        bed camera's stream: it only serves the last photo; taking one needs the PROBE permission
        (the head_photo command or LightBurn's M240 macro through the bridge)."""
        jpeg = self._headcam_bytes()
        if jpeg is None:
            return flask.abort(404)
        return flask.Response(jpeg, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})

    @octoprint.plugin.BlueprintPlugin.route("/closeup.jpg", methods=["GET"])
    def closeup_image(self):
        """/plugin/laserheightprobe/closeup.jpg: the last close-up image (whole bed, LightBurn
        coordinates). Open like headcam.jpg."""
        jpeg = self._closeup_bytes()
        if jpeg is None:
            return flask.abort(404)
        return flask.Response(jpeg, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})

    @octoprint.plugin.BlueprintPlugin.route("/closeup_preview.jpg", methods=["GET"])
    def closeup_preview(self):
        """A light copy of the close-up image (1200 px wide) for the OctoPrint tab."""
        jpeg = self._closeup_preview
        if jpeg is None:
            full = self._closeup_bytes()                    # after a restart: from the saved image
            if full is None:
                return flask.abort(404)
            try:
                from . import cvio
                jpeg = self._closeup_preview = cvio.encode(cvio.shrink_to(cvio.decode(full), PREVIEW_WIDTH), 85)
            except Exception:                                # noqa: BLE001 (no numpy/OpenCV)
                jpeg = full
        return flask.Response(jpeg, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})

    def route_hook(self, server_routes, *args, **kwargs):
        """/plugin/laserheightprobe/headcam.mjpg and closeup.mjpg: the head photo and the close-up
        image as MJPEG streams, the URLs to give LightBurn (headcam_stream.py). Open like
        headcam.jpg: they only show the last image."""
        from .headcam_stream import HeadcamStream
        try:
            interval = float(self._settings.get(["closeup_stream_s"]))
        except Exception:                                    # noqa: BLE001
            interval = DEFAULTS["closeup_stream_s"]
        return [(r"/headcam.mjpg", HeadcamStream, dict(frames=self._headcam_bytes)),
                (r"/closeup.mjpg", HeadcamStream, dict(frames=self._closeup_bytes, interval=interval))]

    # -- close-up camera ------------------------------------------------------

    def _save_json(self, name, data):
        """A JSON file in the data folder (folders made as needed), written whole (.part then replace), best effort."""
        path = os.path.join(self.get_plugin_data_folder(), name)
        try:
            folder = os.path.dirname(path)
            if not os.path.isdir(folder):
                os.makedirs(folder)
            with open(path + ".part", "w") as handle:
                json.dump(data, handle, indent=1)
            os.replace(path + ".part", path)
        except Exception:                                    # noqa: BLE001
            self._logger.exception("could not write %s", path)

    def _write_bytes(self, name, data):
        """A file in the data folder, best effort: a reader holding it open on Windows makes the
        replace fail (WinError 5), and the image is served from memory anyway."""
        path = os.path.join(self.get_plugin_data_folder(), name)
        try:
            folder = os.path.dirname(path)
            if not os.path.isdir(folder):
                os.makedirs(folder)
            with open(path + ".part", "wb") as handle:
                handle.write(data)
            os.replace(path + ".part", path)
        except OSError as exception:
            self._logger.warning("could not write %s: %s", path, exception)
            try:
                os.remove(path + ".part")
            except OSError:
                pass

    def _closeup_model(self):
        if self._camera_model is None:
            data = self._read_json(os.path.join(CLOSEUP_DIR, CAMERA_MODEL_FILE))
            if data:
                self._camera_model = camera_model.CameraModel.from_dict(data)
        return self._camera_model

    def _overview_pose(self):
        if self._headcam_pose is None:
            self._headcam_pose = self._read_json(HEADCAM_POSE_FILE)
        return self._headcam_pose

    def _base_pose(self):
        """Where the close-up base's photo was taken: the 10W's overview, or the bed camera's."""
        if str(self._config().get("closeup_base") or "").strip().lower() == "usb":
            return self._bedcam_pose()
        return self._overview_pose()

    def _closeup_bytes(self):
        if self._closeup_jpeg is None:
            path = os.path.join(self.get_plugin_data_folder(), CLOSEUP_DIR, CLOSEUP_FILE)
            try:
                with open(path, "rb") as handle:
                    self._closeup_jpeg = handle.read()
            except OSError:
                return None
        return self._closeup_jpeg

    def _closeup_status(self):
        if self._closeup_last is None and not self._closeup_disk_read:
            self._closeup_disk_read = True                  # once: the last close-up from disk
            try:
                self._closeup_load_last()
            except Exception as exception:                   # noqa: BLE001
                self._logger.info("last close-up not read: %s", exception)
        libs = closeup_versions()
        model = self._closeup_model()
        try:
            dpm = float(self._settings.get(["closeup_dpm"]))
        except Exception:                                    # noqa: BLE001
            dpm = DEFAULTS["closeup_dpm"]
        return dict(
            available=bool(libs.get("numpy") and libs.get("cv2")), versions=libs, dpm=dpm,
            model=None if model is None else dict(created=model.created, z_range=list(model.z_range or []),
                                                  residuals=model.residuals, t=list(model.t)),
            last=self._closeup_last, request=self._closeup_request, calib=self._calib_status(),
            base=str(self._config().get("closeup_base") or "headcam"),
            bedcam=self._bedcam_summary(self._bedcam_model()))

    def _closeup_setup(self, need_model=True, own_lock=False, allow_rotary=False):
        """Everything a close-up needs, checked in order. Moves nothing: a job calls
        _closeup_settle once its own checks (the plan's) have passed too."""
        if self._running():
            raise ValueError("a job is running")
        if not self._printer.is_operational():
            raise ValueError("the printer is not connected")
        if self._printer.is_printing() or self._printer.is_paused():
            raise ValueError("the printer is busy with a job")
        if self._grbl is not None and self._grbl.state() != "Idle" and not own_lock:
            raise ValueError("LightBurn is running a job or a jog: wait until it is idle")
        if not self._homed:
            raise ValueError("the head's position is not known since the printer connected: home it first (G28)")
        config = self._checked_loose()
        if not config["headcam_url"]:
            raise ValueError("set the touchscreen's address in the settings first (headcam_url, "
                             "for example http://192.168.1.60:8080)")
        head = gcode.toolhead(self._bridge.run(["M1006"], 30.0))
        if not config_module.has_camera(head):
            raise ValueError("the %s has no camera: the close-up camera is the 10W laser's"
                             % (head or "fitted head"))
        online, _ = gcode.rotary_online(self._bridge.run(["M114"], 30.0))
        if online and not allow_rotary:
            raise ValueError("the rotary is fitted: close-ups go down to about Z%.0f, which is not safe "
                             "above the rotary" % config["closeup_standoff"])
        self._mesh_guard()
        module = config_module.find_module(config["laser_modules"], "10W")
        o = self._beam_offset(config, module)
        if config["flat_bed_z"] is None:
            raise ValueError("the bed height is not set: Probe here on the bare bed and press Use as bed")
        if module.get("focus_offset") is None:
            raise ValueError("the 10W focus offset is not set")
        model = self._closeup_model()
        if model is None and need_model:
            raise ValueError("calibrate the close-up camera first (no camera model yet)")
        lo, hi = self._soft_limits()
        where = gcode.position(self._bridge.run([], 15.0))
        if where is None:
            raise ValueError("cannot read the head position (M114)")
        d_bed = config["flat_bed_z"] - model.bed_ref if (model is not None and model.bed_ref is not None) else 0.0
        ctx = dict(o=o, limits=(lo, hi), current=where, d_bed=d_bed, rotary=bool(online), module=module,
                   hood_touch_z=calibration.hood_touch_z(config["flat_bed_z"], module["focus_offset"]),
                   overlap=config["closeup_overlap"], min_clearance=config["closeup_min_clearance"],
                   max_height=config["closeup_max_height"], hood_radius=config["closeup_hood_radius"],
                   max_tiles=config["closeup_max_tiles"], capture_s=config["closeup_capture_s"],
                   # the head shades a band on the photo's right: plan so a lit part covers it
                   margin_px=(40, 40, 40 + int(config["closeup_shadow_frac"] * camera_model.WIDTH), 40),
                   shadow=True,                    # and more where the head's shadow is wider
                   extra_s=config["closeup_capture_s"] if config["closeup_fresh_overview"] else 0.0,
                   standoff_range=config_module.RANGES["closeup_standoff"])
        return config, model, ctx

    def _closeup_settle(self, config, ctx):
        """The last step before a close-up or calibration job, once every refusal has passed:
        off the map's grid (after a home) the printer and the plugin extrapolate the map
        differently, so the head goes up and over the bed first (beam off, LightBurn locked
        out); then the work offset is checked. True if the head moved (re-plan from there)."""
        moved = False
        if self._mesh_loaded() and not self._head_over_map():
            where = ctx["current"]
            x, y, z = config["headcam_x"], config["headcam_y"], config["headcam_z"]
            if self._grbl is not None:
                self._grbl.lock_out("moving the head over the bed")
            try:
                self._bridge.run(["M5", "G90", "G0 Z%.1f F1500" % max(z, where[2], closeup_plan.TRAVEL_FLOOR),
                                  "G0 X%.1f Y%.1f F4000" % (x, y)], 120.0)
            finally:
                if self._grbl is not None:
                    self._grbl.lock_release()
            where = gcode.position(self._bridge.run([], 15.0))
            if where is None:
                raise ValueError("cannot read the head position (M114)")
            ctx["current"] = where
            moved = True
        shift = self._work_offset()
        z_tolerance = MESH_Z_TOLERANCE if (self._mesh and self._mesh.get("loaded_at")) else 0.02
        if max(abs(shift[0]), abs(shift[1])) > 0.02 or abs(shift[2]) > z_tolerance:
            raise ValueError("a work offset is in force (X%+.2f Y%+.2f Z%+.2f): home with G28 first" % shift)
        return moved

    def _closeup_args(self, data, config):
        box = (data or {}).get("box")
        if not isinstance(box, (list, tuple)) or len(box) != 4:
            raise ValueError("box must be [x0, y0, x1, y1] in LightBurn mm")
        box = [float(v) for v in box]
        h = (data or {}).get("height")
        h = config_module.number("closeup_height", h) if h not in (None, "") else None
        standoff = config_module.number("closeup_standoff", (data or {}).get("standoff", config["closeup_standoff"]))
        return box, h, standoff

    def _closeup_object_check(self, model, box, h, o):
        """With a mapped object: the stated height must reach its mapped top under the box (the
        stated height is what keeps the head clear; the map only places the photos)."""
        obj = self._closeup_object(model)
        if obj is None:
            return
        top = max(float(obj(box[0] + (box[2] - box[0]) * u / 8.0 + o[0], box[1] + (box[3] - box[1]) * v / 8.0 + o[1]))
                  for u in range(9) for v in range(9))
        if top < 1.0:
            return                                     # the box is away from the mapped object
        if h is None or float(h) < top - 0.5:
            raise ValueError("the red-dot map puts the object's top at up to %.1f mm under this box: state a "
                             "height of at least that (the tallest thing under the dashed area)" % top)

    def _api_closeup_plan(self, data):
        """The tile plan for a box, with no motion: tiles, the module sweep, the time."""
        config, model, ctx = self._closeup_setup()
        box, h, standoff = self._closeup_args(data, config)
        self._closeup_object_check(model, box, h, ctx["o"])
        return closeup_plan.plan(box, h, standoff, ctx, model)

    def _api_closeup(self, data):
        """Take a close-up of a box (in the background): the head goes up, across, down to
        stand-off above the object's top, photographs each tile, and comes back up."""
        config, model, ctx = self._closeup_setup()
        box, h, standoff = self._closeup_args(data, config)
        self._closeup_object_check(model, box, h, ctx["o"])
        plan = closeup_plan.plan(box, h, standoff, ctx, model)
        usb = config["closeup_base"] == "usb"
        bed_model = None
        if usb:                                    # the bed camera's park: refused before anything moves
            bed_model = self._bedcam_refusals(config) if config["closeup_fresh_overview"] else self._bedcam_model()
            if bed_model is None:
                raise ValueError("calibrate the bed camera first (bedcam_calibrate), or set closeup_base to headcam")
        if self._closeup_settle(config, ctx):
            plan = closeup_plan.plan(box, h, standoff, ctx, model)
        url = config["headcam_url"]
        tiles = []
        # the object's top: its stated height above the bed where it stands (the height map's
        # bed height under the box, relative to where the camera was calibrated)
        surface, _ = self._closeup_bed(model)
        obj = self._closeup_object(model)
        o = ctx["o"]
        if obj is not None:
            # a mapped object: the tiles follow its top; h stays the safety height (the tallest
            # thing under the sweep). The plane used for weights and the levelling is its top at
            # the box's centre.
            cx, cy = (box[0] + box[2]) / 2.0 + o[0], (box[1] + box[3]) / 2.0 + o[1]
            h_plane = float(obj(cx, cy))
        elif h and surface is not None:
            cx, cy = (box[0] + box[2]) / 2.0 + o[0], (box[1] + box[3]) / 2.0 + o[1]
            h_plane = float(h) + float(surface(cx, cy))
        else:
            h_plane = float(h or 0.0)
        job_tiles = list(plan["tiles"])
        if config["closeup_fresh_overview"] and usb:
            # the bed camera's photo first, with the head parked out of its view and the bed at
            # the Y it was calibrated at; Y from -Y (backlash), X straight (the park is at X0)
            px, py, pz = config["bedcam_park_x"], config["bedcam_park_y"], config["bedcam_park_z"]
            a = config["closeup_approach_mm"]
            job_tiles.insert(0, dict(i=-1, x=px, y=py, z=pz,
                                     approach=(0.0, a if py - a >= ctx["limits"][0][1] else 0.0),
                                     capture=lambda sleep: self._bedcam_fetch(config, sleep)))
        elif config["closeup_fresh_overview"]:
            # a fresh overview first (from Z290, before going down): the base and the levelling
            # must show the bed as it is now, not before the object was put there
            job_tiles.insert(0, dict(i=-1, x=config["headcam_x"], y=config["headcam_y"], z=config["headcam_z"]))
        overview_z = config["bedcam_park_z"] if usb else config["headcam_z"]
        run_plan = dict(plan, tiles=job_tiles, travel_z=max(plan["travel_z"], overview_z))

        def capture(x, y, z, sleep):
            return headcam.capture(self._headcam_get, url, x, y, z, quality=config["headcam_quality"], sleep=sleep)

        order = []

        def store(i, jpeg, pose):
            if i == -1 and usb:
                self._keep_bedcam(jpeg, pose, bed_model)
                return
            if i == -1:
                self._keep_overview(jpeg, pose, config)
                return
            tiles.append((jpeg, pose))
            order.append(i)
            self._write_bytes(os.path.join(CLOSEUP_DIR, "last", "tile_%02d.jpg" % i), jpeg)

        def compose():
            self._closeup_tiles = list(tiles)
            self._closeup_h = h_plane
            self._closeup_photos_time = time.time()
            self._save_json(os.path.join(CLOSEUP_DIR, "last", "tiles.json"),
                             dict(plan=dict((k, plan[k]) for k in ("box_lb", "h", "standoff", "head_z", "o")),
                                  h_plane=h_plane, poses=[list(pose) for _, pose in tiles],
                                  files=["tile_%02d.jpg" % i for i in order], d_bed=ctx["d_bed"],
                                  time=self._closeup_photos_time))
            return self._closeup_make(model, o, h_plane, config, ctx["d_bed"])

        with self._lock:
            if self._running():
                raise ValueError("a job is already running")
            self._bridge_disarm("a close-up started")
            self._progress = None
            self._last_error = None
            self._closeup_request = dict(box=box, height=h, standoff=standoff, time=time.time())
            n = len(plan["tiles"])
            self._on_message(dict(type="started", mode="closeup", message="Close-up: %d tile%s at Z%.1f, about %.0f s."
                                  % (n, "" if n == 1 else "s", plan["head_z"], plan["seconds"])))
            self._hold(True)
            if self._grbl is not None:
                self._grbl.lock_out("taking close-up photos")
            try:
                self._job = _CloseupThread(self, self._bridge, capture, self._on_message, run_plan, store, compose,
                                           dict(z_feed=config["z_feed"], xy_feed=config["xy_feed"], approach_mm=config["closeup_approach_mm"]),
                                           logger=self._logger)
                self._job.start()
            except Exception:
                self._closeup_finished()
                raise
        return dict(started=True, tiles=n, seconds=plan["seconds"], head_z=plan["head_z"])

    def _closeup_finished(self):
        """Called by the close-up thread after its safe end."""
        self._hold(False)
        if self._grbl is not None:
            self._grbl.lock_release()
            try:
                self._grbl.resync()
            except Exception:                                # noqa: BLE001
                self._logger.warning("could not read the head position after the close-up")

    def _keep_overview(self, jpeg, pose, config):
        """A new overview photo: served to LightBurn's head camera and used as the close-up base."""
        self._headcam_jpeg = jpeg
        self._headcam = dict(bytes=len(jpeg), time=time.time())
        self._headcam_save(jpeg)
        self._headcam_pose = dict(x=pose[0], y=pose[1], z=pose[2], quality=config["headcam_quality"],
                                  time=self._headcam["time"], mesh=self._mesh_loaded())
        self._save_json(HEADCAM_POSE_FILE, self._headcam_pose)

    def _mesh_guard(self):
        """While the height map is meant to be loaded: levelling on and no fade height, as
        map_load left it. In laser mode a G28 leaves levelling OFF (seen 2026-11-21: none of the
        map's correction at Z290 after a home); gcode_sent re-enables it, and this checks."""
        if not self._mesh_loaded():
            return
        state = meshload.read_state(self._bridge)
        if state["levelling"] and not state["fade"]:
            return
        self._log_line("Height map: the printer had levelling %s and a fade height of %s mm (after a home?): "
                       "set back to levelling on, no fade." % ("ON" if state["levelling"] else "OFF", state["fade"]))
        self._bridge.run(["M420 Z0", "M420 S1"], 30.0)
        state = meshload.read_state(self._bridge)
        if not state["levelling"] or state["fade"]:
            raise ValueError("the printer would not keep the height map on with no fade (M420 S1 Z0): "
                             "press Load height map again")

    def _head_over_map(self):
        """Whether the head is over the height map's grid (machine X, Y)."""
        hm = self._map
        if not hm or not hm.get("nodes") or not hm.get("base"):
            return True
        where = gcode.position(self._bridge.run([], 15.0))
        if where is None:
            return False
        b = hm["base"]
        mx, my = where[0] + b["machine_x"] - b["x"], where[1] + b["machine_y"] - b["y"]
        n = len(hm["nodes"])
        x1, y1 = hm["start"][0] + hm["spacing"][0] * (n - 1), hm["start"][1] + hm["spacing"][1] * (len(hm["nodes"][0]) - 1)
        return hm["start"][0] <= mx <= x1 and hm["start"][1] <= my <= y1

    def _mesh_loaded(self):
        return bool(self._mesh and self._mesh.get("loaded_at"))

    def _closeup_bed(self, model):
        """(surface, head_dz) from the height map. surface(x, y) is the bed's height (B frame)
        relative to where the camera was calibrated; head_dz(x, y) is how far the firmware lifts
        the head there while the map is loaded (0 when it is not). The bed is about 1.8 mm out
        across its width, which moved overview points by up to 0.9 mm (seen 2026-11-21)."""
        from . import compositor
        hm = self._map
        if not hm or not hm.get("nodes") or not hm.get("base"):
            return None, lambda x, y: 0.0
        base = hm["base"]
        offset = (base["machine_x"] - base["x"], base["machine_y"] - base["y"])
        raw = compositor.surface_from_map(hm["nodes"], hm["start"], hm["spacing"], offset, 0.0)
        ref = model.surface_ref if model.surface_ref is not None else 0.0
        surface = compositor.surface_from_map(hm["nodes"], hm["start"], hm["spacing"], offset, ref)
        loaded = self._mesh_loaded()
        return surface, (lambda x, y: float(raw(x, y))) if loaded else (lambda x, y: 0.0)

    def _closeup_object(self, model):
        """With a red-dot map of an object stored: surface(x, y), B frame, of the object's top
        relative to where the camera was calibrated (the object map plus its measured zero, on
        the probed bed map), so close-up tiles follow a sloped or curved top. None otherwise."""
        from . import compositor
        hm = self._map
        if not hm or hm.get("source") != "red dot" or hm.get("zero_h") is None or not hm.get("base"):
            return None
        base = hm["base"]
        offset = (base["machine_x"] - base["x"], base["machine_y"] - base["y"])
        obj = compositor.surface_from_map(hm["nodes"], hm["start"], hm["spacing"], offset, 0.0)
        zero = float(hm["zero_h"])
        bed = self._read_json(BED_MAP_FILE)
        ref = model.surface_ref if model.surface_ref is not None else 0.0
        if bed and bed.get("nodes") and bed.get("base"):
            bb = bed["base"]
            bed_s = compositor.surface_from_map(bed["nodes"], bed["start"], bed["spacing"],
                                                (bb["machine_x"] - bb["x"], bb["machine_y"] - bb["y"]), ref)
        else:
            bed_s = lambda x, y: 0.0
        fp = hm.get("footprint")
        if not fp:
            return lambda x, y: obj(x, y) + zero + bed_s(x, y)
        from .closeup_libs import np as _np
        np = _np()
        m = self.OBJECT_MARGIN
        x0, y0, x1, y1 = fp[0] - m, fp[1] - m, fp[2] + m, fp[3] + m

        def surface(x, y):
            # the object only where it was scanned; elsewhere the bed (the map's plateau beyond
            # the scan is there to keep the head high, not to describe the bed)
            x, y = np.asarray(x, float), np.asarray(y, float)
            inside = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
            return np.where(inside, obj(x, y) + zero + bed_s(x, y), bed_s(x, y))
        return surface

    OBJECT_MARGIN = 5.0                    # mm past the red-dot points still taken as the object

    def _closeup_make(self, model, o, h, config, d_bed, tiles=None):
        """The close-up image: the overview resampled, with the tiles blended in."""
        from . import compositor, cvio
        from .closeup_libs import np as _np
        np = _np()
        geom = compositor.canvas_geometry(config["closeup_dpm"], max_mp=config["closeup_max_mp"])
        usb = config["closeup_base"] == "usb"
        surface, head_dz = self._closeup_bed(model)
        if usb:
            # the fixed bed camera's photo, placed through its own calibration (never the 10W's
            # model: that photo was not taken by the head)
            pose, overview, bed_model = self._bedcam_pose(), self._bedcam_bytes(), self._bedcam_model()
            key = ("usb", pose and pose.get("time"), geom["dpm"], bed_model and bed_model.created)
        else:
            pose, overview, bed_model = self._overview_pose(), self._headcam_bytes(), None
            key = ("headcam", pose and pose.get("time"), geom["dpm"], model.created, tuple(o), round(d_bed, 3),
                   (self._map or {}).get("created"))
        if self._closeup_base is not None and self._closeup_base[0] == key:
            base = self._closeup_base[1]
        elif usb and overview and pose and bed_model is not None:
            if abs(float(pose["y"]) - bed_model.bed_y) > BEDCAM_Y_TOLERANCE:
                raise CloseupError("the bed camera photo was taken with the bed at Y%.2f, not Y%.1f where the "
                                   "camera was calibrated: take the overview again" % (pose["y"], bed_model.bed_y))
            base = compositor.bedcam_canvas(cvio.decode(overview), bed_model, geom)
            self._closeup_base = (key, base)
        elif not usb and overview and pose:
            dz = head_dz(pose["x"], pose["y"]) if pose.get("mesh") else 0.0
            base = compositor.base_canvas(cvio.decode(overview), (pose["x"], pose["y"], pose["z"] + dz), model, o,
                                          geom, 0.0, d_bed, surface)
            self._closeup_base = (key, base)
        else:
            base = np.full((geom["H"], geom["W"], 3), 96, np.uint8)
        use = self._closeup_tiles if tiles is None else tiles
        decoded = [dict(img=cvio.decode(j), head=(p[0], p[1], p[2] + head_dz(p[0], p[1]))) for j, p in use]
        over = None
        if not usb and overview and pose and decoded:
            dz = head_dz(pose["x"], pose["y"]) if pose.get("mesh") else 0.0
            over = dict(img=cvio.decode(overview), head=(pose["x"], pose["y"], pose["z"] + dz))
        # the bed's own unevenness only matters when the object is the bed (height 0); with a
        # red-dot map of the object stored, the tiles follow the object's mapped top instead
        obj = self._closeup_object(model)
        tile_surface = obj if obj is not None else (surface if not h else None)
        canvas, qa = compositor.compose(base, decoded, model, o, h, geom, config["closeup_feather_mm"], d_bed,
                                        tile_surface, config["closeup_shadow_frac"], over)
        jpeg = cvio.encode(canvas, config["closeup_jpeg_quality"])
        self._closeup_jpeg = jpeg
        small = cvio.shrink_to(canvas, PREVIEW_WIDTH)
        self._closeup_preview = cvio.encode(small, 85)
        self._write_bytes(os.path.join(CLOSEUP_DIR, CLOSEUP_FILE), jpeg)
        self._closeup_last = dict(time=time.time(), tiles=len(decoded), height=h, seam_mm=qa.get("seam_mm"),
                                  photos_time=self._closeup_photos_time if decoded else None,
                                  overview_time=(pose or {}).get("time"),
                                  canvas=[geom["W"], geom["H"]], dpm=geom["dpm"], bytes=len(jpeg))
        return dict(qa=dict(seam_mm=qa.get("seam_mm"), tiles=len(decoded)), bytes=len(jpeg),
                    canvas=[geom["W"], geom["H"]])

    def _closeup_load_last(self):
        """The last close-up's tiles and request from disk (after an OctoPrint restart)."""
        info = self._read_json(os.path.join(CLOSEUP_DIR, "last", "tiles.json"))
        if not info:
            return
        tiles = []
        for name, pose in zip(info.get("files") or [], info.get("poses") or []):
            try:
                with open(os.path.join(self.get_plugin_data_folder(), CLOSEUP_DIR, "last", name), "rb") as handle:
                    tiles.append((handle.read(), tuple(pose)))
            except OSError:
                return
        plan = info.get("plan") or {}
        self._closeup_tiles = tiles
        self._closeup_h = float(info.get("h_plane", plan.get("h")) or 0.0)
        self._closeup_photos_time = info.get("time")
        if self._closeup_last is None and tiles:            # the tab's status line after a restart
            self._closeup_last = dict(time=info.get("time"), tiles=len(tiles), height=plan.get("h", 0.0),
                                      seam_mm=None, photos_time=info.get("time"),
                                      overview_time=(self._base_pose() or {}).get("time"))

    def _closeup_recompose(self, tiles, own_lock=False):
        config, model, ctx = self._closeup_setup(own_lock=own_lock)          # photos only: nothing moves
        h = self._closeup_h if tiles else 0.0
        return self._closeup_make(model, ctx["o"], h, config, ctx["d_bed"], tiles=tiles)

    def _recompose_quietly(self):
        """After LightBurn's Head photo: the close-up image again on the new overview. LightBurn
        is locked out meanwhile (about 80 s on the test machine): the compose loads the CPU, and a burn
        streamed through the bridge must not stall."""
        locked = False
        try:
            if self._grbl is not None:
                if self._grbl.state() != "Idle":
                    return
                self._grbl.lock_out("rebuilding the close-up image on the new overview")
                locked = True
            if not self._closeup_tiles:
                self._closeup_load_last()
            self._closeup_recompose(self._closeup_tiles, own_lock=locked)
        except Exception as exception:                       # noqa: BLE001
            self._logger.info("close-up image not rebuilt after the overview: %s", exception)
        finally:
            if locked:
                self._grbl.lock_release()

    def _api_closeup_clear(self, data):
        """Back to the overview alone."""
        self._closeup_tiles = []
        return self._closeup_recompose([])

    def _api_closeup_recompose(self, data):
        """Make the image again from the last tiles (after a new overview photo, say)."""
        if not self._closeup_tiles:
            self._closeup_load_last()
        return self._closeup_recompose(self._closeup_tiles)

    def _api_closeup_selftest(self, data):
        """The synthetic end-to-end on the OctoPrint host's numpy and OpenCV, through the camera model."""
        from . import compositor
        config = self._checked_loose()
        dpm = config_module.number("closeup_dpm", (data or {}).get("dpm", config["closeup_dpm"]))
        model = self._closeup_model() or camera_model.CameraModel(t=(-57.0, 6.0))
        out = compositor.selftest(model, dpm=dpm)
        out["versions"] = closeup_versions()
        return out

    def _api_closeup_lightburn(self, data):
        """The LightBurn camera entry for the close-up stream (written into LightBurn's prefs on
        the computer that runs LightBurn, with LightBurn closed)."""
        config = self._checked_loose()
        base = str((data or {}).get("base_url") or "").rstrip("/")
        if not base:
            try:
                base = flask.request.host_url.rstrip("/")
            except Exception:                                # noqa: BLE001
                raise ValueError("give base_url, the OctoPrint address LightBurn can reach")
        entry = lightburn_camera.camera_entry(config["closeup_dpm"], base + "/plugin/laserheightprobe/closeup.mjpg",
                                              CLOSEUP_NAME, time.strftime("%Y-%m-%d_%H-%M"))
        return dict(camera=entry)

    # -- close-up camera calibration -------------------------------------------

    # The calibration crosses (burned 2026-11-20, 16 mm arms): where the camera can reach them.
    # It looks 57 mm to the -X side of the beam, so nothing right of about LB X260 is reachable.
    CROSSES_LB = ((60.0, 100.0), (200.0, 100.0), (60.0, 250.0), (200.0, 250.0))

    @octoprint.plugin.BlueprintPlugin.route("/closeup_board.png", methods=["GET"])
    def closeup_board(self):
        """The ChArUco board to print (A4, 100 %) for the close-up camera calibration."""
        from . import calib_solve, cvio
        return flask.Response(cvio.board_page(calib_solve.BOARD), mimetype="image/png")

    @octoprint.plugin.BlueprintPlugin.route("/closeup_calib/<session>/<int:n>.jpg", methods=["GET"])
    def closeup_calib_photo(self, session, n):
        path = self._calib_path(session, "%03d.jpg" % n)
        if not os.path.exists(path):
            return flask.abort(404)
        with open(path, "rb") as handle:
            return flask.Response(handle.read(), mimetype="image/jpeg")

    def _calib_path(self, session, name=""):
        if not re.match(r"^[0-9_-]+$", str(session)):
            raise ValueError("bad calibration session %r" % session)
        return os.path.join(self.get_plugin_data_folder(), CLOSEUP_DIR, "calib", session, name)

    def _calib_session(self, data, stage):
        """The session named in data (checked), else the newest one of that stage."""
        session = str((data or {}).get("session") or "")
        if session:
            self._calib_path(session)                        # refuses anything but a session name
            return session
        newest = self._calib_sessions().get(stage)
        if not newest:
            raise ValueError("no %s photos yet: take them first" % ("board" if stage == "board" else "cross"))
        return newest

    def _calib_sessions(self):
        """The newest board and cross sessions on disk."""
        root = os.path.join(self.get_plugin_data_folder(), CLOSEUP_DIR, "calib")
        found = {}
        try:
            names = sorted(os.listdir(root), reverse=True)
        except OSError:
            return found
        for name in names:
            if not re.match(r"^[0-9_-]+$", name):
                continue
            info = self._read_json(os.path.join(CLOSEUP_DIR, "calib", name, "poses.json"))
            stage = (info or {}).get("stage")
            if stage in ("board", "cross") and stage not in found:
                found[stage] = name
            if len(found) == 2:
                break
        return found

    def _calib_status(self):
        found = self._calib_sessions()
        candidate = self._read_json(os.path.join(CLOSEUP_DIR, "camera_model_candidate.json"))
        reports = self._read_json(os.path.join(CLOSEUP_DIR, "camera_model_candidate_report.json")) or {}
        return dict(board=found.get("board"), cross=found.get("cross"),
                    candidate=dict(reports, t=candidate.get("t")) if candidate else None)

    def _calib_report(self, stage, report):
        name = os.path.join(CLOSEUP_DIR, "camera_model_candidate_report.json")
        reports = {} if stage == "board" else (self._read_json(name) or {})
        keep = dict((k, v) for k, v in report.items() if isinstance(v, (int, float, str, type(None))))
        reports[stage] = keep
        self._save_json(name, reports)

    def _calib_guess(self):
        """The camera to aim calibration photos with: the current model, else the nominal one
        (Tests B1-B5: f about 814, e about 5, the camera 57 mm to the -X side of the beam)."""
        return self._closeup_model() or camera_model.CameraModel(f=814.0, e=5.0, t=(-57.0, 6.0))

    def _api_closeup_calib_capture(self, data):
        """Calibration photos (in the background). stage "board": the printed ChArUco board taped
        flat near target (LightBurn mm, default the bed's middle), 56 photos from Z40 to Z290.
        stage "cross": the four burned alignment crosses, 16 photos at Z50/60."""
        if (data or {}).get("clear") is not True:
            raise ValueError("say the bed is clear (clear: true): nothing taller than the calibration board or "
                             "paper under the head, which goes down to Z40")
        config, model, ctx = self._closeup_setup(need_model=False)
        stage = str((data or {}).get("stage") or "")
        guess = self._calib_guess()
        o = ctx["o"]
        if stage == "board":
            target = (data or {}).get("target") or (160.0, 175.0)
            poses = closeup_plan.calib_poses("board", (float(target[0]) + o[0], float(target[1]) + o[1]), guess)
            poses.sort(key=lambda p: -p["z"])                 # from high to low
        elif stage == "cross":
            crosses = [tuple(float(v) for v in c) for c in ((data or {}).get("crosses") or self.CROSSES_LB)]
            poses = closeup_plan.calib_poses("cross", [(x + o[0], y + o[1]) for x, y in crosses], guess)
            for p in poses:
                p["lb"] = list(crosses[p["cross"]])
        else:
            raise ValueError("stage must be board or cross")
        (minx, miny), (maxx, maxy) = ctx["limits"]
        if stage == "cross":
            # the camera looks 57 mm to the -X side of the beam, so the crosses near the right
            # edge may be out of its reach: tie with the ones it can photograph (two or more)
            reach = [p for p in poses if minx <= p["x"] <= maxx and miny <= p["y"] <= maxy]
            whole = set(p["cross"] for p in reach
                        if sum(1 for q in reach if q["cross"] == p["cross"]) == len(closeup_plan.CROSS_POSES))
            poses = [p for p in reach if p["cross"] in whole]
            if len(whole) < 2:
                raise ValueError("the camera can reach only %d of the burned crosses: need 2" % len(whole))
        for p in poses:
            if not (minx <= p["x"] <= maxx and miny <= p["y"] <= maxy):
                raise ValueError("a calibration photo needs the head at X%.1f Y%.1f, outside the machine's "
                                 "limits: put the board nearer the middle" % (p["x"], p["y"]))
        lowest = min(p["z"] for p in poses)
        if lowest - ctx["hood_touch_z"] < ctx["min_clearance"]:
            raise ValueError("the 10W's hood would come within %.1f mm of the bed" % (lowest - ctx["hood_touch_z"]))
        self._closeup_settle(config, ctx)
        session = time.strftime("%Y%m%d-%H%M%S")
        url = config["headcam_url"]
        tiles = [dict(p, i=i) for i, p in enumerate(poses)]
        self._save_json(os.path.join(CLOSEUP_DIR, "calib", session, "poses.json"),
                         dict(stage=stage, o=list(o), flat_bed_z=config["flat_bed_z"], poses=tiles,
                              guess=guess.to_dict(), mesh=self._mesh_loaded()))
        shots = []

        def capture(x, y, z, sleep):
            return headcam.capture(self._headcam_get, url, x, y, z, quality=config["headcam_quality"], sleep=sleep)

        def store(i, jpeg, pose):
            shots.append((i, pose))
            self._write_bytes(os.path.join(CLOSEUP_DIR, "calib", session, "%03d.jpg" % i), jpeg)

        def finish():
            return self._calib_detect(session)

        plan = dict(tiles=tiles, head_z=lowest, travel_z=max(float(ctx["current"][2]), closeup_plan.TRAVEL_FLOOR))
        with self._lock:
            if self._running():
                raise ValueError("a job is already running")
            self._bridge_disarm("a close-up calibration started")
            self._progress = None
            self._last_error = None
            self._on_message(dict(type="started", mode="closeup_calib",
                                  message="Close-up calibration (%s): %d photos, session %s." % (stage, len(tiles), session)))
            self._hold(True)
            if self._grbl is not None:
                self._grbl.lock_out("taking calibration photos")
            try:
                self._job = _CloseupThread(self, self._bridge, capture, self._on_message, plan, store, finish,
                                           dict(z_feed=config["z_feed"], xy_feed=config["xy_feed"], approach_mm=config["closeup_approach_mm"]),
                                           mode="closeup_calib", logger=self._logger)
                self._job.start()
            except Exception:
                self._closeup_finished()
                raise
        return dict(started=True, session=session, photos=len(tiles))

    def _calib_detect(self, session):
        """After a calibration capture: find the board corners, or the crosses (aimed with the
        board-stage model when there is one)."""
        from . import cvio
        info = self._read_json(os.path.join(CLOSEUP_DIR, "calib", session, "poses.json"))
        out = dict(session=session, stage=info["stage"])
        if info["stage"] == "board":
            from . import calib_solve
            lift = self._calib_lift(info)
            obs = []
            for p in info["poses"]:
                path = self._calib_path(session, "%03d.jpg" % p["i"])
                if not os.path.exists(path):
                    continue
                with open(path, "rb") as handle:
                    ids, px = cvio.detect_charuco(cvio.decode(handle.read()), calib_solve.BOARD)
                if len(ids) >= 6:
                    obs.append(dict(head=[p["x"], p["y"], p["z"] + lift(p["x"], p["y"])], ids=ids, px=px.tolist(),
                                    holdout=p["holdout"]))
            self._save_json(os.path.join(CLOSEUP_DIR, "calib", session, "obs.json"), obs)
            out.update(photos=len(info["poses"]), found=len(obs), corners=sum(len(o["ids"]) for o in obs))
        else:
            out.update(self._calib_mark_all(session))
        self._log_line("Close-up calibration %s: %s." % (session, ", ".join(
            "%s %s" % (k, v) for k, v in sorted(out.items()) if k not in ("session", "stage"))))
        return out

    def _calib_mark_all(self, session):
        from . import cvio
        info = self._read_json(os.path.join(CLOSEUP_DIR, "calib", session, "poses.json"))
        model = self._read_candidate() or self._calib_guess()
        o = info["o"]
        marks = []
        for p in info["poses"]:
            path = self._calib_path(session, "%03d.jpg" % p["i"])
            if not os.path.exists(path):
                continue
            lb = p.get("lb") or self.CROSSES_LB[p["cross"]]
            head = (p["x"], p["y"], p["z"])
            guess = model.project(head, [[lb[0] + o[0], lb[1] + o[1], 0.0]])[0]
            with open(path, "rb") as handle:
                img = cvio.decode(handle.read())
            px_mm = model.f / (p["z"] + model.e)
            u, v = cvio.refine_cross(img, guess, px_mm)
            marks.append(dict(photo=p["i"], head=list(head), px=[u, v], lb=list(lb), auto=True))
        self._save_json(os.path.join(CLOSEUP_DIR, "calib", session, "marks.json"), marks)
        return dict(photos=len(info["poses"]), marked=len(marks))

    def _map_raw(self):
        """The height map's bed height at B-frame points, or None (no map)."""
        from . import compositor
        hm = self._map
        if not hm or not hm.get("nodes") or not hm.get("base"):
            return None
        b = hm["base"]
        return compositor.surface_from_map(hm["nodes"], hm["start"], hm["spacing"],
                                           (b["machine_x"] - b["x"], b["machine_y"] - b["y"]))

    def _calib_lift(self, info):
        """How far the firmware lifted the head at each photo: the map's value there while it was
        loaded during the capture, else 0."""
        raw = self._map_raw()
        if raw is None or not info.get("mesh"):
            return lambda x, y: 0.0
        return lambda x, y: float(raw(x, y))

    def _read_candidate(self):
        data = self._read_json(os.path.join(CLOSEUP_DIR, "camera_model_candidate.json"))
        return camera_model.CameraModel.from_dict(data) if data else None

    def _api_closeup_calib_solve(self, data):
        """Stage A: fit the camera to a board session's photos. The result is a candidate
        (accept it with closeup_calib_accept)."""
        from . import calib_solve
        session = self._calib_session(data, "board")
        obs = self._read_json(os.path.join(CLOSEUP_DIR, "calib", session, "obs.json"))
        info = self._read_json(os.path.join(CLOSEUP_DIR, "calib", session, "poses.json"))
        if not obs or not info:
            raise ValueError("no board photos in session %r" % session)
        guess = camera_model.CameraModel.from_dict(info["guess"])
        model, pose, report = calib_solve.solve_board(obs, guess)
        raw = self._map_raw()
        ref = None
        if raw is not None:
            # the map's bed height under the board's middle: what the camera's distances are
            # measured from (surface heights are map - surface_ref). Stage A itself stays flat:
            # solving it against the map's surface fitted the board photos better but put the
            # crosses of an independent 18-tile close-up further out (0.32 vs 0.20 mm RMS,
            # 2026-11-21), so the map does not describe the board that well. Stage B does use it.
            t_hint = tuple(self._calib_guess().t)
            _, bxy = calib_solve.board_corners()
            mid = calib_solve.board_to_world(bxy, pose).mean(0)
            ref = float(raw(mid[0] + t_hint[0], mid[1] + t_hint[1]))
        model.bed_ref = info["flat_bed_z"]
        model.t = tuple(self._calib_guess().t)                   # until the crosses tie it (stage B)
        model.surface_ref = ref
        self._save_json(os.path.join(CLOSEUP_DIR, "camera_model_candidate.json"), model.to_dict())
        self._log_line("Close-up calibration stage A: f %.1f, e %.2f, fit %.2f px, held-out %.3f mm RMS (%.3f max)."
                       % (model.f, model.e, report["fit_rms_px"], report["holdout_rms_mm"] or -1,
                          report["holdout_max_mm"] or -1))
        self._calib_report("board", report)
        return dict(report=report, model=model.to_dict())

    def _api_closeup_calib_mark(self, data):
        """Move one cross mark by hand (a click on the photo), refined to the cross."""
        from . import cvio
        session = str((data or {}).get("session") or "")
        n = int((data or {}).get("photo"))
        marks = self._read_json(os.path.join(CLOSEUP_DIR, "calib", session, "marks.json")) or []
        mark = [m for m in marks if m["photo"] == n]
        if not mark:
            raise ValueError("photo %d of session %s has no cross" % (n, session))
        model = self._read_candidate() or self._calib_guess()
        with open(self._calib_path(session, "%03d.jpg" % n), "rb") as handle:
            img = cvio.decode(handle.read())
        u, v = cvio.refine_cross(img, (float(data["u"]), float(data["v"])), model.f / (mark[0]["head"][2] + model.e))
        mark[0].update(px=[u, v], auto=False)
        self._save_json(os.path.join(CLOSEUP_DIR, "calib", session, "marks.json"), marks)
        return dict(photo=n, px=[u, v])

    def _api_closeup_calib_tie(self, data):
        """Stage B: tie the candidate camera to the beam with a cross session's marks."""
        from . import calib_solve
        session = self._calib_session(data, "cross")
        model = self._read_candidate()
        if model is None:
            raise ValueError("solve a board session first (no candidate camera model)")
        info = self._read_json(os.path.join(CLOSEUP_DIR, "calib", session, "poses.json"))
        if (data or {}).get("remark") and info:
            self._calib_mark_all(session)                    # the automatic marks again (keeps nothing by hand)
        marks = self._read_json(os.path.join(CLOSEUP_DIR, "calib", session, "marks.json"))
        if not info or not marks:
            raise ValueError("no cross marks in session %r" % session)
        lift = self._calib_lift(info)
        raw = self._map_raw()
        surface = None
        if raw is not None and model.surface_ref is not None:
            ref = model.surface_ref
            surface = lambda x, y: raw(x, y) - ref
        fit, report = calib_solve.solve_tie(
            model, [dict(head=(m["head"][0], m["head"][1], m["head"][2] + lift(m["head"][0], m["head"][1])),
                         px=m["px"], lb=tuple(m["lb"])) for m in marks], tuple(info["o"]), surface=surface)
        self._save_json(os.path.join(CLOSEUP_DIR, "camera_model_candidate.json"), fit.to_dict())
        self._log_line("Close-up calibration stage B: camera at X%+.2f Y%+.2f from the beam, crosses %.3f mm RMS, "
                       "leave-one-out %.3f mm max." % (fit.t[0], fit.t[1], report["rms_mm"], report["loo_max_mm"] or -1))
        self._calib_report("cross", report)
        return dict(report=report, model=fit.to_dict())

    def _api_closeup_calib_accept(self, data):
        """Use the candidate camera model for close-ups."""
        model = self._read_candidate()
        if model is None or not model.residuals or "stage_b" not in model.residuals:
            raise ValueError("finish both calibration stages first (board, then crosses)")
        self._save_json(os.path.join(CLOSEUP_DIR, CAMERA_MODEL_FILE), model.to_dict())
        self._camera_model = model
        self._closeup_base = None
        for name in ("camera_model_candidate.json", "camera_model_candidate_report.json"):
            try:
                os.remove(os.path.join(self.get_plugin_data_folder(), CLOSEUP_DIR, name))
            except OSError:
                pass
        self._log_line("Close-up camera model accepted.")
        return dict(ok=True, model=model.to_dict())

    def is_blueprint_protected(self):
        return False                      # the blueprint is only the head photo (see above)

    def is_blueprint_csrf_protected(self):
        return True

    def _pointer_light(self, on, config=None):
        """The pointer for jogging by hand (outside LightBurn): the 10W's beam at pointer_s_max
        (M3 S, static), or the crosshair. Goes off by itself after pointer_minutes."""
        timer = getattr(self, "_pointer_timer", None)
        if timer is not None:
            timer.cancel()
            self._pointer_timer = None
        if not on:
            kind = getattr(self, "_pointer_kind", None)
            self._bridge.send(["M5"] + (["M2002 T3 P0"] if kind == "cross" else []))
            self._pointer_kind = None
            return None
        config = config or self._checked_loose()
        d = self._laser_on_head()
        if d["toolhead"] in config_module.CROSSHAIR:
            self._bridge.send(["M5", "M2002 T3 P1"])
            self._pointer_kind = "cross"
        else:
            self._bridge.send(["M3 S%d" % config["pointer_s_max"]])
            self._pointer_kind = "spot"
        self._pointer_timer = threading.Timer(60.0 * config["pointer_minutes"], self._pointer_off_quietly)
        self._pointer_timer.daemon = True
        self._pointer_timer.start()
        self._log_line("Pointer on (%s) for %d min." % (
            "crosshair" if self._pointer_kind == "cross" else "beam at S%d of 255" % config["pointer_s_max"],
            config["pointer_minutes"]))
        return self._pointer_kind

    def _pointer_off_quietly(self):
        try:
            self._pointer_light(False)
            self._log_line("Pointer off (timed out).")
        except Exception:                                  # noqa: BLE001
            self._logger.exception("laserheightprobe: could not switch the pointer off")

    def _api_pointer(self, data):
        if data.get("on") and (self._running() or self._printer.is_printing()):
            raise ValueError("the printer is busy")
        return dict(ok=True, pointer=self._pointer_light(bool(data.get("on"))))

    def _api_beam_offset(self, data):
        """Measure a module's beam offset from two jogs onto one dot (offsets.py).
        step "pin": the probe is on the head and its pin is on the dot. step "beam": the laser
        is on the head and its pointer is on the dot. Returns dx, dy to save (set_xy_offset)."""
        from . import offsets
        step = data.get("step")
        x, y, _z = gcode.position(self._bridge.run([], 15.0)) or (None, None, None)
        if x is None:
            raise ValueError("the printer did not report its position (M114)")
        if step == "pin":
            self._offset_pin = dict(x=x, y=y, time=time.time())
            self._write_json(PIN_FILE, self._offset_pin)
            self._log_line("Beam offset: pin on the dot at X%.3f Y%.3f. Now swap the laser in, power "
                           "cycle, home X Y Z, and put its pointer on the same dot." % (x, y))
            return dict(ok=True, pin=self._offset_pin)
        if step != "beam":
            raise ValueError("step must be pin or beam")
        pin = getattr(self, "_offset_pin", None) or self._read_json(PIN_FILE)
        if not pin:
            raise ValueError("record the pin on the dot first")
        d = self._detected or self._detect()
        if not d.get("laser"):
            raise ValueError("no laser is fitted")
        cross = None
        if d["toolhead"] in config_module.CROSSHAIR:
            cross = gcode.crosshair_offset(self._bridge.run(["M2002 T8"], 30.0))
            if cross is None:
                raise ValueError("the %s did not report its crosshair offset (M2002 T8)" % d["toolhead"])
        dx, dy = offsets.beam_offset((pin["x"], pin["y"]), (x, y), cross)
        module = d.get("module") or self._settings.get(["selected_module"])
        self._log_line("Beam offset for %s: dx %+.3f, dy %+.3f mm (pin X%.3f Y%.3f, %s X%.3f Y%.3f%s). Save it "
                       "to use it." % (module, dx, dy, pin["x"], pin["y"], "crosshair" if cross else "beam", x, y,
                                       ", crosshair offset %+.2f %+.2f" % cross if cross else ""))
        return dict(ok=True, module=module, dx=dx, dy=dy, pin=pin, beam=dict(x=x, y=y), crosshair=cross)

    def _api_bridge_laser(self, data):
        """Arm (laser: true) or disarm the laser for the current bridge mode."""
        if self._grbl is None or self._grbl.context is None:
            raise ValueError("pick a bridge mode first")
        config = self._checked_loose()
        if data.get("laser"):
            self._door_check()
            self._laser_on_head()
            self._grbl.context.laser_until = time.monotonic() + 60.0 * config["bridge_arm_minutes"]
            self._log_line("LightBurn bridge: LASER ARMED for %d min." % config["bridge_arm_minutes"])
        else:
            self._grbl.context.laser_until = None
            busy = self._grbl.state() != "Idle"
            if busy:
                # queued burn lines, and the firmware's last power, would keep the beam on: stop now
                self._grbl.reset()
            self._log_line("LightBurn bridge: laser disarmed%s." % (": the running job was stopped, beam off" if busy else ""))
        return dict(ok=True, laser=bool(data.get("laser")))

    def _keep_json(self, name, value):
        path = os.path.join(self.get_plugin_data_folder(), name)
        try:
            if value is None:
                if os.path.exists(path):
                    os.remove(path)
            else:
                self._write_json(name, value)
        except Exception:                                    # noqa: BLE001
            self._logger.exception("could not keep %s", name)

    def _rotary_mark_stale(self, reason):
        if self._rotary:
            self._api_rotary_stale(dict(reason=reason))

    def _api_rotary_stale(self, data):
        """The object moved in the chuck (reseated, knocked, fell out): the stored rotary
        measurement no longer describes it. Mark it stale; rotary mode and translation refuse it
        until the rotary routine runs again. Drops a rotary bridge mode at once."""
        if not self._rotary:
            raise ValueError("there is no rotary measurement")
        reason = str(data.get("reason") or "the object moved in the chuck")
        self._rotary["complete"] = False
        self._rotary["stale"] = dict(reason=reason, time=time.time())
        self._keep_rotary(self._rotary)
        if self._grbl is not None and getattr(self._grbl.context, "mode", None) == "rotary":
            self._bridge_disarm("the rotary measurement is stale")
        self._log_line("Rotary measurement marked stale: %s. Run the rotary routine again." % reason)
        return dict(ok=True)

    def _calibration_report(self, raw):
        """Repeatability and the re-touch check over the Probe here results (calibration.report)."""
        if raw.get("calib_z_ref") is None:
            reference = None
        else:
            reference = dict(x=raw.get("calib_x"), y=raw.get("calib_y"), z=raw.get("calib_z_ref"),
                             machine_z=raw.get("calib_machine_z"), time=raw.get("calib_time"))
        try:
            return calibration.report(self._here.get("points"), reference)
        except calibration.CalibrationError:
            return None

    def _bridge_status(self):
        b = self._grbl
        if b is None:
            return dict(enabled=False)
        c = b.context
        left = None
        if c is not None and c.laser_until is not None:
            left = max(0, int(c.laser_until - time.monotonic()))
        pointer = b.pointer_state()
        frame = b.frame() if hasattr(b, "frame") else None
        return dict(enabled=True, client=self._grbl_client, state=b.state(),
                    mode=getattr(c, "mode", None), describe=getattr(c, "describe", None),
                    focus_z=getattr(c, "focus_z", None), dry_run=bool(getattr(c, "dry_run", False)),
                    frame=dict(frame, age_s=int(time.time() - frame["time"])) if frame else None,
                    laser_seconds_left=left,
                    pointer=dict(pointer, until=None, seconds_left=max(0, int(pointer["until"] - time.monotonic()))
                                 if pointer.get("until") else None) if pointer else None)

    def gcode_received(self, comm_instance, line, *args, **kwargs):
        door = gcode.door_event(line) if line else None
        if door is not None:
            self._door_seen(door)
        if line and "Laser security state" in line or line and "roll:" in line:
            seen = (self._security_seen or (0, {}))[1]
            report = gcode.laser_security([line])
            if report is None:                             # the attitude line, after the state
                report = gcode.laser_security(["Laser security state: 0x%x" % seen.get("state", 0), line]) \
                    if "state" in seen else None
            if report:
                self._security_seen = (time.time(), dict(seen, **report))
        if line and "Count" in line:
            try:
                self._b_frame_note(line)
            except Exception:                                # noqa: BLE001
                self._logger.exception("could not follow the rotary's angle")
        fixed = gcode.unglue_ok(line)
        if fixed != line:
            self._logger.info("Repaired an ok glued into the laser report: %r" % line)
            line = fixed
        link = getattr(self, "_link", None)
        if link is not None:
            link.received(line)
        if self._bridge is None:
            return line
        return self._bridge.on_gcode_received(comm_instance, line, *args, **kwargs)

    # -- the rotary's angle frame across power cycles -------------------------

    def _b_frame_note(self, line):
        """Every M114 reply: remember B, and on the first one after a connect, spot a reset.
        A power cycle makes the firmware call wherever B was "B0" (B 0, step count 0); the old
        reading becomes part of the shift, so jobs keep the measured object's angles."""
        b = gcode.b_position([line])
        if b is None:
            return
        match = _COUNT_B_RE.search(line)
        count = int(match.group(1)) if match else None
        f = self._b_frame
        if f.get("check"):
            f["check"] = False
            last = float(f.get("last_b") or 0.0)
            if abs(b) < 1e-6 and count == 0 and abs(last) > 1e-6:
                f["shift"] = float(f.get("shift") or 0.0) + last
                f["last_b"] = 0.0
                self._b_frame_save(force=True)
                self._log_line("The rotary's angle count was reset (a power cycle) with B at %.2f: "
                               "jobs keep the measured object's angles (B shift now %+.2f)." % (last, f["shift"]))
                return
        if f.get("last_b") != b:
            f["last_b"] = b
            self._b_frame_save()

    def _b_frame_reconnected(self):
        self._b_frame["check"] = True

    def _b_frame_new_measurement(self):
        """A rotary measurement fixes the object's angles in today's firmware frame."""
        self._b_frame["shift"] = 0.0
        self._b_frame_save(force=True)

    def _b_frame_save(self, force=False):
        now = time.time()
        if not force and now - self._b_frame_saved < 2.0:
            return
        self._b_frame_saved = now
        self._write_json(B_FRAME_FILE, dict(last_b=self._b_frame.get("last_b", 0.0),
                                            shift=self._b_frame.get("shift", 0.0), at=now))

    HEATERS = ("M104", "M109", "M140", "M190")

    def gcode_queuing(self, comm_instance, phase, cmd, cmd_type, gcode_, *args, **kwargs):
        """With a laser fitted, refuse to heat (the start of any 3D print) and cancel the print:
        laser jobs never heat, and a 3D print would drive the laser head round the bed.
        Also where a waiting purifier gear change goes: straight after a laser-off line, tagged
        so the sent hook keeps it out of the bridge's count."""
        if self._air is not None and gcode_ not in self.HEATERS:
            extra = self._air.queuing(cmd)
            if extra:
                return [(cmd, cmd_type)] + [(line, None, {AIR_TAG}) for line in extra[1:]]
        if gcode_ not in self.HEATERS:
            return None
        m = re.search(r"\bS(\d+(?:\.\d+)?)", cmd or "")
        if m is None or float(m.group(1)) == 0:
            return None                                    # a heater off is always fine
        detected = self._detected or {}
        if not detected.get("laser"):
            # a 3D print head, or nothing detected yet (just connected): a laser never heats, so
            # refusing here costs a laser nothing
            if self._thermal_state() == "disabled":
                return self._refuse_heat(cmd, "ThermalRunaway is off (it was turned off for the laser, "
                                              "whose missing bed reads as -14 C). Press \"Get ready to 3D "
                                              "print\" on the OctoBurn tab first")
            return None
        printing = self._printer.is_printing() or self._printer.is_paused()
        why = ("the %s is fitted (M1006 \"%s\"): refused \"%s\"%s. Fit the 3D print head and power "
               "cycle first" % (self._detected.get("module") or "laser", self._detected.get("toolhead"),
                                cmd.strip(), ", and cancelled the print" if printing else ""))
        self._log_line("Laser fitted: " + why + ".")
        self._plugin_manager.send_plugin_message(self._identifier, dict(type="refused", message=why))
        if printing:
            threading.Thread(target=self._printer.cancel_print, name="laserheightprobe-cancel",
                             daemon=True).start()
        return (None,)

    def _refuse_heat(self, cmd, why):
        printing = self._printer.is_printing() or self._printer.is_paused()
        why = "refused \"%s\"%s: %s" % (cmd.strip(), " and cancelled the print" if printing else "", why)
        self._log_line("3D print head: " + why + ".")
        self._plugin_manager.send_plugin_message(self._identifier, dict(type="refused", message=why))
        if printing:
            threading.Thread(target=self._printer.cancel_print, name="laserheightprobe-cancel",
                             daemon=True).start()
        return (None,)

    THERMAL_PLUGIN = "ThermalRunaway"

    def _thermal_state(self):
        """"enabled", "disabled" or "missing": the ThermalRunaway plugin, which must be on for 3D
        printing and off with a laser fitted (no bed thermistor: -14 C, and it sends M112)."""
        try:
            info = self._plugin_manager.get_plugin_info(self.THERMAL_PLUGIN, require_enabled=False)
        except Exception:                                    # noqa: BLE001
            return "missing"
        if info is None:
            return "missing"
        return "enabled" if getattr(info, "enabled", False) else "disabled"

    def _thermal_saved(self):
        """ThermalRunaway's state after the next restart: "enabled" or "disabled" (OctoPrint keeps
        disabled plugins in config.yaml, plugins._disabled)."""
        disabled = self._settings.global_get(["plugins", "_disabled"]) or []
        return "disabled" if self.THERMAL_PLUGIN in disabled else "enabled"

    def _set_thermal(self, on):
        """Enable or disable ThermalRunaway the way OctoPrint's Plugin Manager does: the
        plugins._disabled list in config.yaml, saved; a restart applies it."""
        disabled = list(self._settings.global_get(["plugins", "_disabled"]) or [])
        if on and self.THERMAL_PLUGIN in disabled:
            disabled.remove(self.THERMAL_PLUGIN)
        elif not on and self.THERMAL_PLUGIN not in disabled:
            disabled.append(self.THERMAL_PLUGIN)
        self._settings.global_set(["plugins", "_disabled"], disabled)
        self._settings.save(force=True)
        if self._thermal_saved() != ("enabled" if on else "disabled"):
            raise ValueError("could not save ThermalRunaway's state in OctoPrint's settings")
        self._thermal_restart = self._thermal_state() != self._thermal_saved()

    def _swap_status(self):
        thermal = self._thermal_state()
        try:
            saved = self._thermal_saved() if thermal != "missing" else "missing"
        except Exception:                                    # noqa: BLE001
            saved = thermal
        return dict(thermal=thermal, thermal_saved=saved, mesh_loaded=self._mesh_loaded(),
                    restart=thermal != saved and thermal != "missing")

    def _api_ready_to_print(self, data):
        """With the 3D print head back on: the print mesh back (if the laser map is loaded) and
        ThermalRunaway on. Refused with a laser fitted."""
        d = self._detect()
        if d.get("laser"):
            raise ValueError("a laser is fitted (M1006 \"%s\"): fit the 3D print head and power cycle first"
                             % d.get("toolhead"))
        done = []
        if self._mesh_loaded():
            self._api_map_restore({})
            done.append("the print mesh is back")
        if self._thermal_state() == "disabled" or self._thermal_saved() == "disabled":
            self._set_thermal(True)
            done.append("ThermalRunaway is saved as enabled (restart OctoPrint to apply)")
        self._log_line("Ready to 3D print: %s." % ("; ".join(done) if done else "nothing to change"))
        return dict(ok=True, done=done, restart=self._swap_status()["restart"], swap=self._swap_status())

    def _api_ready_for_laser(self, data):
        """With a laser fitted: ThermalRunaway off (it would send M112 on the laser's -14 C bed)."""
        d = self._detect()
        if not d.get("laser"):
            raise ValueError("no laser is fitted (M1006 \"%s\")" % d.get("toolhead"))
        done = []
        if self._thermal_state() == "enabled" or self._thermal_saved() == "enabled":
            self._set_thermal(False)
            done.append("ThermalRunaway is saved as disabled (restart OctoPrint to apply)")
        self._log_line("Ready for the laser: %s." % ("; ".join(done) if done else "nothing to change"))
        return dict(ok=True, done=done, restart=self._swap_status()["restart"], swap=self._swap_status())

    # -- what is fitted ---------------------------------------------------

    def _detect(self):
        """Ask the printer what is fitted: M1006 (toolhead), M114 (B only while the rotary is
        online) and, for a laser, M2002 S1 (its safety state and attitude). Picks the module
        when M1006 names it. Returns the detected dict, also kept for the status."""
        head = gcode.toolhead(self._bridge.run(["M1006"], 30.0))
        online, unusable = gcode.rotary_online(self._bridge.run([], 15.0))
        security = None
        laser = config_module.is_laser(head)
        if laser:
            asked = time.time()
            # The report comes from the laser a moment after the ok; the dwell waits for it.
            security = gcode.laser_security(self._bridge.run(["M2002 S1"], 30.0))
            # The report comes from the laser about a second after the ok (seen on the A350):
            # the receive hook keeps it, so wait for one newer than the question.
            deadline = time.time() + 3.0
            while security is None and time.time() < deadline:
                seen = self._security_seen
                if seen and seen[0] >= asked and "roll" in seen[1]:
                    security = seen[1]
                    break
                time.sleep(0.1)
        if laser:
            door = gcode.enclosure_door(self._bridge.run(["M1010"], 30.0))
            if door is not None:
                self._door = (time.time(), door)
        modules = config_module.parse_modules(self._settings.get(["laser_modules"]))
        module = config_module.module_for_toolhead(modules, head)
        if module and module != self._settings.get(["selected_module"]):
            self._store(dict(selected_module=module))
            self._log_line("Detected the %s (M1006 \"%s\"): selected it." % (module, head))
        self._detected = dict(at=time.time(), toolhead=head, laser=laser, module=module,
                              module_known=bool(module) or not laser, rotary=online,
                              rotary_unusable=unusable, security=security,
                              will_fire=None if security is None else security.get("state") == 0)
        self._plugin_manager.send_plugin_message(self._identifier, dict(type="detected",
                                                                       detected=self._detected))
        return self._detected

    def _note_homed_head(self, wait=40.0):
        """After a home: was the 10W seated (safety state 0) or another head, typically the probe?"""
        time.sleep(wait)
        try:
            d = self._detect() if not self._running() else (self._detected or {})
        except Exception:                                    # noqa: BLE001
            d = self._detected or {}
        sec = (d or {}).get("security")
        if sec is None:
            return
        self._homed_head = "laser" if sec.get("state", 1) == 0 else "probe"
        self._log_line("Homed with the %s on the head." % ("10W seated" if self._homed_head == "laser" else "probe (the laser not seated)"))

    def _detect_quietly(self, delay=2.0):
        """_detect after a connect, unless something else is using the printer."""
        time.sleep(delay)
        for _ in range(60):                       # at startup the printer can connect first
            if self._bridge is not None:
                break
            time.sleep(0.5)
        try:
            if self._bridge is None or not self._printer.is_operational():
                return
            if self._running() or self._printer.is_printing() or self._printer.is_paused():
                return
            self._detect()
        except Exception:                                  # noqa: BLE001
            self._logger.exception("laserheightprobe: could not detect what is fitted")

    def _api_rotary_template(self, data):
        """A LightBurn project (.lbrn2) of the stored rotary measurement: tool layers only (the
        measured body, the seam, the chuck side and the probe touches), with LightBurn's rotary
        diameter set to the measured one. Returned as text for the page to save."""
        from . import lbrn
        if not self._rotary:
            raise ValueError("there is no rotary measurement: run the rotary routine first")
        y_origin = float(data["y_origin"]) if data.get("y_origin") is not None else None
        module = self._settings.get(["selected_module"])
        text, summary = lbrn.rotary_template(self._rotary, y_origin=y_origin, module=module)
        name = "rotary-%.1fmm-%s.lbrn2" % (summary["diameter"],
                                          time.strftime("%Y%m%d-%H%M", time.localtime(
                                              self._rotary.get("finished") or time.time())))
        return dict(filename=name, content=text, summary=summary)

    def _api_detect(self, data):
        if self._running():
            raise ValueError("a probe job is running")
        if self._printer.is_printing() or self._printer.is_paused():
            raise ValueError("the printer is busy with a job")
        return dict(detected=self._detect())

    # -- recent burns -----------------------------------------------------
    #
    # A burn through the LightBurn bridge (beam on, power above 0) is tracked by burns.BurnTracker
    # through the bridge's burn_watch. Once the bridge has been idle burns.IDLE_S after it, the
    # job goes into burns.json and, with burn_closeup_after on, the 10W photographs the burned
    # area (a normal close-up, which disarms the bridge). A refusal is kept in the entry.

    def _burns(self):
        """The burn history (burns.json in the data folder), read on first use."""
        history = getattr(self, "_burn_history", None)
        if history is None:
            from . import burns
            history = self._burn_history = burns.BurnHistory(self.get_plugin_data_folder(), logger=self._logger)
        return history

    def _burns_tracker(self):
        tracker = getattr(self, "_burn_tracker", None)
        if tracker is None:
            from . import burns
            tracker = self._burn_tracker = burns.BurnTracker()
        return tracker

    def _burns_start(self):
        """Watch the LightBurn bridge's burns (called once the bridge is listening)."""
        if self._grbl is None:
            return
        for entry in self._burns().entries():
            if (entry.get("closeup") or {}).get("status") in ("running", "starting"):
                # OctoPrint restarted during its close-up
                self._burns().update(entry["id"], closeup=dict(entry["closeup"], status="stopped",
                                                               message="Interrupted: OctoPrint restarted during the close-up."))
        self._grbl.burn_watch = self._burns_tracker().move
        threading.Thread(target=self._burns_loop, name="laserheightprobe-burns", daemon=True).start()

    def _burns_loop(self):
        while not self._grbl_stop.is_set():
            try:
                self._burns_step()
            except Exception:                                # noqa: BLE001
                self._logger.exception("burn history")
            self._grbl_stop.wait(1.0)

    def _burns_step(self):
        """One look: a burn close-up that has finished, then a burn that has finished (its new
        history entry is returned)."""
        pending = getattr(self, "_burn_pending", None)
        if pending is not None and (pending["job"] is None or not pending["job"].is_alive()):
            self._burn_pending = None
            self._burns_closeup_done(pending)
        if self._grbl is None:
            return None
        job = self._burns_tracker().step(self._grbl.state())
        return self._burns_record(job) if job is not None else None

    def _burns_watts(self, job):
        watts = self._air_watts()
        if not watts and job.get("module"):
            watts = purifier_mod.module_watts(config_module.TOOLHEADS.get(job["module"]))
        return watts or None

    def _burns_record(self, job):
        """A finished burn: into the history, then its close-up, or why there is none."""
        from . import burns
        entry = burns.entry_for(job, self._burns_watts(job))
        if not self._settings.get_boolean(["burn_closeup_after"]):
            closeup = dict(status="off", message="Close-up after a burn is off in the settings.")
        elif job.get("alarm"):
            closeup = dict(status="skipped", message="The job ended in an alarm, so no close-up.")
        elif job.get("mode") != "flat" or not job.get("box"):
            closeup = dict(status="skipped", message="Rotary job: close-ups are for flat work only.")
        elif self._burns_no_camera(job):
            closeup = dict(status="skipped", message="No camera on the %s, so no close-up." % self._burns_no_camera(job))
        elif not self._door or self._door[1] != "closed":
            # the head moves by itself: never while the door may be open for someone reaching in
            closeup = dict(status="skipped", message="The enclosure door is %s, so no automatic close-up (the head would "
                                                     "move by itself)." % ("open" if self._door else "not known to be shut"))
        else:
            closeup = None
        entry = self._burns().add(dict(entry, closeup=closeup or dict(status="starting", message="Starting the close-up."),
                                       image=False))
        if closeup is None:
            closeup = self._burns_closeup_start(entry["id"], job)
            entry = self._burns().update(entry["id"], closeup=closeup) or entry
        self._burns_changed()
        energy = entry.get("energy_j")
        self._log_line("Burn finished: %.0f s%s, LightBurn X%.1f..%.1f Y%.1f..%.1f. %s"
                       % (entry["duration_s"], ", %.0f J" % energy if energy else "",
                          entry["bounds"][0], entry["bounds"][2], entry["bounds"][1], entry["bounds"][3],
                          closeup["message"]))
        return entry

    def _burns_no_camera(self, job):
        """The name of the fitted (or the job's) laser module if it has no camera, else None.
        Only the 10W has one; with an unknown module the close-up's own checks decide."""
        detected = self._detected or {}
        toolhead = detected.get("toolhead")
        if toolhead and not config_module.has_camera(toolhead):
            return detected.get("module") or toolhead
        module = job.get("module")
        if module and not config_module.has_camera(config_module.TOOLHEADS.get(module, module)):
            return module
        return None

    def _burns_target(self, job):
        """(box, height) for a burn's close-up: the burned bounds padded and clipped to the
        work area, and the burned surface's height above the bed (the job's focus less the
        bed's). Raises ValueError with the reason it cannot be worked out."""
        from . import burns
        if job.get("focus_z") is None:
            raise ValueError("the job's focus height is not known")
        config = self._checked_loose()
        bed, _ = self._dot_bed_focus(config)
        h = burns.surface_height(job["focus_z"], bed)
        hi = config_module.RANGES["closeup_height"][1]
        if h > hi:
            raise ValueError("the burned surface is %.1f mm above the bed, more than a close-up allows (%g mm)" % (h, hi))
        return burns.closeup_box(job["box"], job.get("area")), h

    def _burns_closeup_start(self, burn_id, job):
        """Start the close-up of a finished burn; the entry's close-up status either way."""
        try:
            box, h = self._burns_target(job)
            self._burns_obstacle_check(box, h)
            out = self._api_closeup(dict(box=box, height=h))
        except Exception as exception:                       # noqa: BLE001 - kept in the entry instead
            return dict(status="refused", message="No close-up: %s." % str(exception).rstrip("."))
        self._burn_pending = dict(id=burn_id, job=self._job, box=box, height=h)
        n = out.get("tiles") or 0
        return dict(status="running", box=box, height=h,
                    message="Close-up: %d tile%s at height %.1f mm, about %.0f s."
                            % (n, "" if n == 1 else "s", h, out.get("seconds") or 0.0))

    def _burns_closeup_done(self, pending):
        """A burn's close-up has finished: keep its crop over the burned area, or why not."""
        from . import burns
        job, burn_id, history = pending["job"], pending["id"], self._burns()
        if history.get(burn_id) is None:
            return                                           # cleared meanwhile
        base = dict(box=pending["box"], height=pending["height"])
        if getattr(job, "result", None) is None:
            if getattr(job, "aborted", False):
                history.update(burn_id, closeup=dict(base, status="stopped", message="The close-up was stopped."))
            else:
                history.update(burn_id, closeup=dict(base, status="failed", message="Close-up failed: %s"
                                                     % (getattr(job, "error", None) or "unknown error")))
            self._burns_changed()
            return
        try:
            jpeg, dpm = self._closeup_bytes(), (self._closeup_last or {}).get("dpm")
            if jpeg is None or not dpm:
                raise ValueError("no close-up image")
            history.save_image(burn_id, burns.crop_closeup(jpeg, dpm, pending["box"]))
        except Exception as exception:                       # noqa: BLE001
            history.update(burn_id, closeup=dict(base, status="done", message="Close-up taken, but the crop "
                                                 "was not saved: %s" % exception))
            self._burns_changed()
            return
        history.update(burn_id, image=True, closeup=dict(base, status="done", message="Close-up taken."))
        self._burns_changed()

    def _burns_changed(self):
        """Tell open tabs the history changed (a skipped or refused close-up sends nothing else)."""
        try:
            self._plugin_manager.send_plugin_message(self._identifier, dict(type="burns"))
        except Exception:                                    # noqa: BLE001
            self._logger.debug("burns message not sent", exc_info=True)

    def _burns_obstacle_check(self, box, h):
        """The close-up's h is the tallest thing under its module sweep. A stored obstacle taller
        than the burned surface under that sweep would break that, and nobody is watching an
        automatic close-up: refuse (ValueError). Plans only when such an obstacle exists."""
        taller = [ob for ob in self._obstacles() if float(ob.get("top", 0.0)) > h + 0.5]
        if not taller:
            return
        sweep = self._api_closeup_plan(dict(box=box, height=h))["sweep_lb"]
        for ob in taller:
            x0, y0, x1, y1 = ob["box"]
            if sweep[0] < x1 and x0 < sweep[2] and sweep[1] < y1 and y0 < sweep[3]:
                raise ValueError("%s (%.0f mm tall) is under the close-up's path: take the close-up by hand "
                                 "with its height, or take it away and clear it from the obstacle list"
                                 % (ob.get("name") or "an obstacle", float(ob.get("top", 0.0))))

    def _burns_status(self):
        from . import burns
        entries = self._burns().entries()
        shown = entries[-burns.SHOW:][::-1]
        for e in shown:
            e["image_url"] = "plugin/laserheightprobe/burns/%s.jpg" % e["id"] if e.get("image") else None
        return dict(enabled=bool(self._settings.get_boolean(["burn_closeup_after"])), entries=shown, total=len(entries))

    def _api_burns_clear(self, data):
        """Wipe the burn history and its photos."""
        n = self._burns().clear()
        self._log_line("Recent burns cleared (%d)." % n)
        self._burns_changed()
        return dict(ok=True, cleared=n)

    @octoprint.plugin.BlueprintPlugin.route("/burns/<burn_id>.jpg", methods=["GET"])
    def burn_image(self, burn_id):
        """/plugin/laserheightprobe/burns/<id>.jpg: a burn's close-up crop (Recent burns). Open
        like closeup.jpg; only ids in the history are served."""
        path = self._burns().image_path(burn_id)
        if path is None:
            return flask.abort(404)
        try:
            with open(path, "rb") as handle:
                jpeg = handle.read()
        except OSError:
            return flask.abort(404)
        return flask.Response(jpeg, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})

    # -- the API ----------------------------------------------------------

    def is_api_protected(self):
        return True

    def get_api_commands(self):
        return dict((name, []) for name in self.COMMAND_PERMISSIONS)

    def on_api_command(self, command, data):
        needed = self.COMMAND_PERMISSIONS.get(command)
        if needed is None:
            return flask.abort(400, "unknown command")
        if not self._allowed(needed):
            return flask.abort(403, "you do not have permission to do that")
        handler = getattr(self, "_api_" + command, None)
        if handler is None:
            return flask.abort(400, "unknown command")
        if self._bridge is None:
            return flask.make_response(flask.jsonify(error="still starting up"), 503)
        try:
            result = handler(data or {})
        except (ValueError, EspError) as exception:
            return flask.make_response(flask.jsonify(error=str(exception)), 400)
        except Exception as exception:                       # noqa: BLE001
            self._logger.exception("api command %s failed", command)
            return flask.make_response(flask.jsonify(error=str(exception)), 500)
        return flask.jsonify(result or dict(ok=True))

    def on_api_get(self, request):
        if not self._allowed("VIEW"):
            return flask.abort(403)
        return flask.jsonify(self._status())

    def _status(self):
        job = self._job
        running = self._running()
        raw = self._config()
        missing = config_module.heights_missing(raw)
        try:
            config = routine_config(raw, require_heights=False)
            settings_error = None
        except SettingsError as exception:
            config, settings_error = None, str(exception)
        modules = config["laser_modules"] if config else []
        selected = config_module.find_module(modules, raw.get("selected_module"))
        return dict(
            running=running,
            mode=job.mode if running else None,
            phase=job.phase if running else None,
            progress=self._progress if running else None,
            log=list(self._log),
            error=self._last_error,
            settings_error=settings_error,
            heights=dict(safe_z=raw.get("safe_z"), z_floor=raw.get("z_floor"), missing=missing),
            tap_test=bool(raw.get("tap_test")),
            points=raw.get("points") or "",
            modules=modules,
            selected_module=selected["name"] if selected else None,
            calibration_report=self._calibration_report(raw),
            calibration=dict(x=raw.get("calib_x"), y=raw.get("calib_y"), z_ref=raw.get("calib_z_ref"),
                             machine_z=raw.get("calib_machine_z"), time=raw.get("calib_time")),
            last=self._last,
            purifier=self._air.status() if self._air is not None else None,
            door=dict(state=self._door[1], time=self._door[0]) if self._door else None,
            swap=self._swap_status(),
            homed_head=getattr(self, "_homed_head", None),
            obstacles=self._obstacles(),
            flat_focus=self._flat_focus_status(),
            ready=self._ready_checks(),
            focus_after_frame=bool(self._settings.get_boolean(["focus_after_frame"])),
            auto_mode=dict(enabled=bool(self._settings.get_boolean(["auto_mode"])),
                           paused=bool(getattr(self, "_auto_paused", False)), why=getattr(self, "_auto_why", None),
                           flat_height=float(self._settings.get(["flat_focus_height"]) or 0.0)),
            dot=dict(last=self._dot_last, calibrated=self._dot_calibrated()),
            table=self._table(selected, None) if self._last else None,
            here=self._here_table(selected),
            rotary=dict((self._rotary or {}).get("summary") or {}, complete=(self._rotary or {}).get("complete"),
                        finished=(self._rotary or {}).get("finished"), stale=(self._rotary or {}).get("stale"),
                        last_check=(self._rotary or {}).get("last_check"),
                        last_verify=dict(((self._rotary or {}).get("last_verify") or {}), points=None) or None,
                        partial=bool(getattr(self, "_rotary_partial", None)))
            if self._rotary else None,
            session=dict(ok=self._session.ok, at=self._session.at, reason=self._session.reason),
            pin=self._pin,
            here_max_descent=raw.get("here_max_descent"),
            flat_bed_z=raw.get("flat_bed_z"),
            headcam=self._headcam_status(),
            closeup=self._closeup_status(),
            burns=self._burns_status(),
            here_square_size=raw.get("here_square_size"),
            here_square_n=raw.get("here_square_n"),
            esp_url=raw.get("esp_url"),
            bridge=self._bridge_status(),
            detected=self._detected,
            height_map=self._map_summary(),
            mesh=dict(loaded=bool(self._mesh), at=(self._mesh or {}).get("loaded_at"),
                      toolhead=((self._mesh or {}).get("backup") or {}).get("toolhead"),
                      warning=self._mesh_warning),
        )

    def _map_summary(self):
        m = self._map
        if not m:
            return None
        keep = ("created", "module", "points", "range", "worst", "rms", "slope_deg", "warnings", "base",
                "start", "spacing", "grid", "nodes", "tolerance", "focus_gap", "free_nodes", "source")
        return dict((k, m.get(k)) for k in keep)

    def _table(self, module, datum):
        last = self._last or {}
        points = last.get("points") or []
        if not points:
            return None
        invert = bool(self._settings.get_boolean(["lightburn_invert"]))
        offset = module["focus_offset"] if module else None
        rows = results.table(points, offset, last.get("reference"),
                             last.get("datum") if datum is None else datum, invert)
        return dict(rows=rows, text=results.as_text(rows, module["name"] if module else None),
                    module=module["name"] if module else None, offset=offset, invert=invert)

    def _here_table(self, module):
        points = self._here.get("points") or []
        invert = bool(self._settings.get_boolean(["lightburn_invert"]))
        offset = module["focus_offset"] if module else None
        rows = results.table(points, offset, None, self._here.get("datum"), invert)
        return dict(rows=rows, text=results.as_text(rows, module["name"] if module else None) if rows else "",
                    module=module["name"] if module else None, offset=offset, invert=invert,
                    last=rows[-1] if rows else None)

    # -- jobs ---------------------------------------------------------------

    def _start(self, points, mode, config):
        with self._lock:
            if self._running():
                raise ValueError("a job is already running")
            if not self._printer.is_operational():
                raise ValueError("the printer is not connected")
            if self._printer.is_printing() or self._printer.is_paused():
                raise ValueError("the printer is busy with a job")
            if self._grbl is not None and self._grbl_client is not None and self._grbl.state() != "Idle":
                raise ValueError("LightBurn is running a job through the bridge")
            if mode != "tap":
                self._bridge_disarm("a probe job started")
            if mode in ("job", "calibrate") and not points:
                raise ValueError("there are no points to measure")
            if self._mesh and mode != "tap":
                # Levelling on would flatten what the probe measures (and a map made from that
                # would cancel the compensation), and the job's G28 can turn levelling off.
                raise ValueError("a height map is loaded in the printer. Press Restore print mesh "
                                 "before probing")
            module = config_module.find_module(config["laser_modules"], config["selected_module"])
            if mode in ("job", "calibrate"):
                self._log.clear()
            self._progress = None
            self._last_error = None
            started = {"here": "Probing here.", "tap": "Tap test: the pin deploys, then tap it."}.get(
                mode, "Started: %d point%s." % (len(points), "" if len(points) == 1 else "s"))
            self._on_message(dict(type="started", mode=mode, message=started))
            # The tap test moves nothing, so it holds nothing. Everything else holds a print
            # until the head is back up.
            if mode != "tap":
                self._hold(True)
            try:
                self._job = _Job(self, self._bridge, self._esp(config), self._on_message, config, points,
                                 mode=mode, module=module, clock=self._clock, logger=self._logger,
                                 session=self._session)
                self._job.start()
            except Exception:
                self._finished()
                raise
        return dict(started=True)

    def _api_here(self, data):
        """Probe here: lower the head from where the user jogged it, and come back up."""
        config = self._checked_loose()
        if not self._session.ok:
            raise ValueError("do the tap test first (%s)" % self._session.reason)
        self._pin = "up"
        if data.get("square"):
            # From the panel, so it can differ from the saved defaults; same ranges.
            try:
                size = config_module.number("here_square_size", data.get("size", config["here_square_size"]))
                n = config_module.number("here_square_n", data.get("n", config["here_square_n"]))
            except SettingsError as exception:
                raise ValueError(str(exception))
            shape = str(data.get("shape") or "square")
            from .probe import SHAPES
            if shape not in SHAPES:
                raise ValueError("the pattern must be one of %s" % ", ".join(sorted(SHAPES)))
            config = dict(config, here_square=dict(size=size, n=n, shape=shape))
        if data.get("height") not in (None, "") or data.get("top") not in (None, ""):
            from .routine import HERE_FROM_CLEAR, HERE_FROM_WINDOW
            if data.get("top") not in (None, ""):
                top = _float(data, "top")
            else:
                if config.get("flat_bed_z") is None:
                    raise ValueError("touch the bare bed first and press Use as bed: the height is "
                                     "measured from the bed")
                height = _float(data, "height")
                if not 0.0 <= height <= 300.0:
                    raise ValueError("the height above the bed must be 0 to 300 mm")
                top = config["flat_bed_z"] + height
            config = dict(config, here_from=dict(top=round(top, 3), z=round(top + HERE_FROM_CLEAR, 3),
                                                 window=HERE_FROM_WINDOW))
        if data.get("repeat"):
            if data.get("square"):
                raise ValueError("a repeat run is over one spot, not a square")
            try:
                config = dict(config, **calibration.forced_samples(data.get("repeat")))
            except calibration.CalibrationError as exception:
                raise ValueError(str(exception))
        return self._start([], "here", config)

    def _api_rotary_translate(self, data):
        """Translate an uploaded LightBurn rotary file (data.path, an OctoPrint local file) into
        a Snapmaker B-axis file next to it, from the stored rotary result and the selected
        module's focus offset. Explicit only: nothing translates on upload."""
        import io
        import re
        from . import rotary_gcode
        if not self._rotary or not self._rotary.get("complete"):
            raise ValueError("there is no complete rotary measurement: run the rotary routine first")
        path = str(data.get("path") or "")
        if not path:
            raise ValueError("give the path of the uploaded LightBurn file")
        config = self._checked_loose()
        module = config_module.find_module(config["laser_modules"],
                                           data.get("module") or config["selected_module"])
        surface = rotary_gcode.Surface(self._rotary, module["focus_offset"] if module else None)
        summary = self._rotary["summary"]
        y_origin = float(data["y_origin"]) if data.get("y_origin") is not None else summary["body_from_y"]
        y_sign = int(data.get("y_sign", rotary_gcode.Y_SIGN))
        b_sign = int(data.get("b_sign", rotary_gcode.B_SIGN))
        if y_sign not in (1, -1) or b_sign not in (1, -1):
            raise ValueError("y_sign and b_sign must be 1 or -1")
        source = self._file_manager.path_on_disk("local", path)
        with open(source, "r") as f:
            lines = f.read().splitlines()
        out, report = rotary_gcode.translate(lines, surface, y_origin, y_sign=y_sign, b_sign=b_sign,
                                             b_shift=float(self._b_frame.get("shift") or 0.0))
        target = str(data.get("output") or (re.sub(r"\.[^./]*$", "", path) + ".snapmaker.gcode"))
        if target == path:
            raise ValueError("the output must not overwrite the LightBurn file")
        body = ("\n".join(out) + "\n").encode("utf-8")
        self._file_manager.add_file("local", target, _bytes_file(target, body), allow_overwrite=True)
        self._log_line("Rotary: translated %s to %s (%d burn moves, Y%.1f to Y%.1f, B%.1f to B%.1f)."
                       % (path, target, report["burn_moves"], report["y_from"], report["y_to"],
                          report["b_from"], report["b_to"]))
        return dict(ok=True, output=target, report=report, module=module["name"] if module else None,
                    y_origin=y_origin, y_sign=y_sign, b_sign=b_sign)

    def _api_rotary(self, data):
        """The rotary routine: runout, axis, ends. data: the rotary parameters (rotary_job)."""
        from . import rotary_job
        config = self._checked_loose()
        if not self._session.ok:
            raise ValueError("do the tap test first (%s)" % self._session.reason)
        homed = getattr(self, "_homed_head", None)
        if homed != "laser" and not (data.get("check") and homed is None):
            # (with the head unknown, after a restart, the housing check may run: it reads the
            # frame and restores the note when it matches a 10W-frame measurement)
            # the burns happen in the laser's frame; X homes ~12 mm differently with the probe on
            raise ValueError("X was %s: home with the 10W seated, then swap to the probe without homing again (X homes "
                             "about 12 mm differently with the probe on, and the burns happen in the laser's frame)"
                             % ("last homed with the probe on" if homed == "probe" else "not homed with the 10W since OctoPrint started"))
        raw = dict(data.get("params") or {})
        # The housing reference constants come from the settings unless the job gives its own.
        for key, setting in (("ref_offset", "rotary_ref_offset"), ("ref_slope", "rotary_ref_slope"),
                             ("ref_housing_z", "rotary_ref_housing_z")):
            if raw.get(key) is None:
                raw[key] = config.get(setting)
        if raw.get("head_y_limit") is None:
            raw["head_y_limit"] = config.get("rotary_head_y_limit")
        if raw.pop("seed", None) == "last" or data.get("check") or data.get("seed_params"):
            self._rotary_seed(raw)
        if data.get("check"):
            raw.update(steps=["reference"], check_only=True)
        if raw.get("resume") is True:
            earlier = getattr(self, "_rotary_partial", None) or self._rotary
            if not earlier:
                raise ValueError("there is no earlier rotary result to resume from")
            raw["resume"] = earlier
        try:
            p = rotary_job.params(raw)
        except rotary_job.RotaryError as exception:
            raise ValueError(str(exception))
        return self._start([], "rotary", dict(rotary_job.probe_config(config, p), rotary=p))

    def _rotary_seed(self, raw):
        """Start from the last measurement instead of a Probe here: its axis, top and ends, its
        housing as the reference to compare with (the routine shifts the start by however far
        the housing has moved), and its limits. What the caller gave wins."""
        last = self._rotary or {}
        s = last.get("summary") or {}
        ref = last.get("reference") or {}
        if s.get("axis_x") is None or s.get("top_z") is None or ref.get("centre_x") is None:
            raise ValueError("there is no earlier complete measurement to start from: Probe here on the top of "
                             "the object and press Measure")
        before = last.get("params") or {}
        seed = dict(axis_x=s["axis_x"], crown=s["top_z"], free_end_guess=s.get("body_from_y"),
                    chuck_end_guess=s.get("body_to_y"), ref_x=ref["centre_x"],
                    seed_ref=dict(centre_x=ref["centre_x"], z=ref["z"]),
                    y_chuck_limit=before.get("y_chuck_limit"), max_diameter=before.get("max_diameter"))
        for key, value in seed.items():
            if raw.get(key) is None and value is not None:
                raw[key] = value

    def _api_rotary_verify(self, data):
        """Touch the object where the stored measurement says its top is, at 4 positions along it
        and 4 angles: how far rotary mode's focus would be off (about 10 minutes, probe on)."""
        if not self._rotary or not self._rotary.get("complete"):
            raise ValueError("there is no complete rotary measurement to verify")
        s = self._rotary.get("summary") or {}
        lo, hi = s.get("body_from_y"), s.get("body_to_y")
        if lo is None or hi is None:
            raise ValueError("the measurement has no body ends to spread the touches over")
        n = min(max(int(data.get("positions") or 4), 2), 8)
        inset = min(20.0, (hi - lo) / 4.0)
        ys = [round(lo + inset + (hi - lo - 2 * inset) * i / float(n - 1), 1) for i in range(n)]
        angles = [float(a) for a in (data.get("angles") or [0.0, 90.0, 180.0, 270.0])]
        params = dict(steps=["verify"], verify_against=self._rotary, verify_ys=ys, verify_angles=angles,
                      verify_b_shift=float(self._b_frame.get("shift") or 0.0),
                      diameter=s.get("diameter"))
        return self._api_rotary(dict(params=params, seed_params=True))

    def _api_rotary_check(self, data):
        """Has the rotary moved? Two touches on the housing, compared with the last measurement's
        (about a minute, probe on). Moved sideways: the measurement is marked stale."""
        return self._api_rotary(dict(data, check=True))

    def _api_rotary_turn(self, data):
        """Turn the rotary by `by` degrees (default 90) or to `to`, for levelling the object with a
        spirit level a quarter turn at a time. The pin is stowed and the head goes up (never down)
        to ROTARY_TURN_Z, or 10 mm over the highest crown measured, before B moves."""
        self._refuse_while_running()
        if self._grbl is not None and getattr(self._grbl, "context", None) is not None:
            raise ValueError("LightBurn's bridge mode is armed: turn it off before turning the rotary")
        lines = self._bridge.run([], 30.0)
        found, b = gcode.position(lines), gcode.b_position(lines)
        if found is None:
            raise ValueError("M114 did not report the position")
        if b is None or any("B unusable" in line for line in lines):
            raise ValueError("the printer does not report a usable rotary (no B in M114)")
        if data.get("to") is not None:
            target = _float(data, "to")
        else:
            target = b + (_float(data, "by", required=False) if data.get("by") is not None else 90.0)
        if not -720.0 <= target <= 720.0:
            raise ValueError("B%.1f is outside -720 to 720 degrees" % target)
        crowns = [((self._rotary or {}).get("params") or {}).get("crown")]
        crowns += [s.get("crown") for s in (self._rotary or {}).get("stations") or []]
        crowns = [c for c in crowns if isinstance(c, (int, float))]
        lift = max([ROTARY_TURN_Z] + [c + 10.0 for c in crowns])
        self._esp().stow()
        self._pin = "up"
        commands = ["G90"]
        if found[2] < lift - 1e-6:
            commands.append("G0 Z%.3f F%d" % (lift, int(self._config().get("z_feed") or 600)))
        commands.append("G0 B%.3f F1200" % target)
        after = gcode.b_position(self._bridge.run(commands, 120.0))
        if after is None or abs(after - target) > 0.05:
            raise ValueError("B is at %s, not %.2f" % (after, target))
        self._log_line("Rotary turned to B%.1f (head at Z%.1f or above, pin stowed)."
                       % (after, max(found[2], lift)))
        return dict(ok=True, b=after, z=max(found[2], lift))

    def _api_tap_test(self, data):
        config = self._checked_loose()
        if data.get("assume"):
            config = dict(config, tap_assumed=True)
        return self._start([], "tap", config)

    def _api_here_datum(self, data):
        index = int(data.get("index", -1))
        if not 0 <= index < len(self._here["points"]):
            raise ValueError("there is no result %d" % (index + 1))
        self._here["datum"] = index
        self._keep_here()
        return dict(ok=True)

    def _api_here_clear(self, data):
        self._refuse_while_running()
        self._here = dict(points=[], datum=None)
        self._keep_here()
        return dict(ok=True)

    def _api_here_as_bed(self, data):
        """Use a Probe here result on the bare bed as the flat bed's height, so a later Probe
        here can start from an object's height above it."""
        points = self._here["points"]
        index = int(data.get("index", len(points) - 1))
        if not 0 <= index < len(points):
            raise ValueError("there is no Probe here result to use: touch the bare bed first")
        z = points[index]["z"]
        self._store(dict(flat_bed_z=z))
        self._log_line("Flat bed: Probe here result %d, trigger Z%.3f, is the bed now." % (index + 1, z))
        return dict(ok=True, bed_z=z)

    def _api_here_as_reference(self, data):
        """Use a Probe here result as the calibration's reference touch (step 1)."""
        points = self._here["points"]
        index = int(data.get("index", len(points) - 1))
        if not 0 <= index < len(points):
            raise ValueError("there is no Probe here result to use")
        p = points[index]
        self._store(dict(calib_x=p["x"], calib_y=p["y"], calib_z_ref=p["z"],
                         calib_machine_z=p.get("machine_z"), calib_time=time.time()))
        self._log_line("Reference for the focus calibration: Probe here result %d, trigger Z%.3f."
                       % (index + 1, p["z"]))
        return dict(ok=True, z_ref=p["z"])

    def _api_start(self, data):
        if "points" in data:
            self._api_save_points(data)
        config = self._checked()
        return self._start(config["points"], "job", config)

    def _api_calibrate_probe(self, data):
        config = self._checked()
        x, y = _float(data, "x"), _float(data, "y")
        point = config_module.parse_points("%.4f %.4f" % (x, y))[0]
        return self._start([point], "calibrate", config)

    def _api_abort(self, data):
        self._stop_job("stopped by the user")
        return dict(ok=True)

    def _refuse_while_running(self):
        if self._running():
            raise ValueError("a job is running: wait for it to finish or stop it first")

    def _read_position(self):
        self._refuse_while_running()
        lines = self._bridge.run([], 30.0)
        found = gcode.position(lines)
        if found is None:
            raise ValueError("M114 did not report the position")
        steps = gcode.z_steps_per_mm(self._bridge.run(["M92"], 30.0))
        return found, gcode.count_z(lines, steps)

    def _api_position(self, data):
        """The head position from M114, and where the pin is in the selected module's frame
        (head + that module's pin offset; 0, 0 while it is unmeasured)."""
        try:
            config = routine_config(self._config(), require_heights=False)
        except SettingsError as exception:
            raise ValueError("the settings are not valid: %s" % exception)
        (x, y, z), machine_z = self._read_position()
        module = config_module.find_module(config["laser_modules"], config["selected_module"]) or {}
        dx, dy = module.get("dx"), module.get("dy")
        return dict(x=x, y=y, z=z, machine_z=machine_z,
                    pin_x=round(x + (dx or 0.0), 3), pin_y=round(y + (dy or 0.0), 3),
                    xy_calibrated=dx is not None and dy is not None)

    def _api_set_height(self, data):
        """Set safe_z or z_floor: typed, or captured from the head's current Z (M114)."""
        which = data.get("which")
        if which not in config_module.HEIGHTS:
            raise ValueError("which must be safe_z or z_floor")
        value = _float(data, "value", required=False)
        if value is None:
            (_, _, value), _ = self._read_position()
        merged = self._config()
        merged[which] = value
        try:
            routine_config(merged, require_heights=False)
        except SettingsError as exception:
            raise ValueError(str(exception))
        self._store({which: value})
        self._log_line("%s set to Z%.3f." % (which, value))
        return dict(ok=True, which=which, value=value)

    def _api_set_xy_offset(self, data):
        """Save a module's probe pin offset (pin minus beam), or clear it."""
        config = self._checked_loose()
        name = str(data.get("module") or "")
        if not any(m["name"] == name for m in config["laser_modules"]):
            raise ValueError("there is no laser module called %r" % name)
        if data.get("clear"):
            dx = dy = None
        else:
            dx, dy = _float(data, "dx"), _float(data, "dy")
        text = config_module.set_module_fields(self._settings.get(["laser_modules"]), name, dx=dx, dy=dy)
        self._store(dict(laser_modules=text))
        self._log_line("Probe XY offset for %s: %s." % (name, "cleared" if dx is None else "%+.3f, %+.3f mm" % (dx, dy)))
        return dict(ok=True, module=name, dx=dx, dy=dy)

    def _api_save_points(self, data):
        text = str(data.get("text", data.get("points", "")) or "")
        points = config_module.parse_points(text)
        self._store(dict(points=text))
        return dict(ok=True, count=len(points))

    def _api_grid(self, data):
        points = config_module.grid(data.get("x0"), data.get("y0"), data.get("x1"), data.get("y1"),
                                    data.get("nx"), data.get("ny"))
        return dict(text=config_module.format_points(points), count=len(points))

    def _api_select_module(self, data):
        config = self._checked_loose()
        name = str(data.get("name") or "")
        if not any(m["name"] == name for m in config["laser_modules"]):
            raise ValueError("there is no laser module called %r" % name)
        self._store(dict(selected_module=name))
        return dict(ok=True)

    def _api_set_offset(self, data):
        """Save a module's focus offset: typed, from a laser focus Z, or from touch + focal."""
        config = self._checked_loose()
        name = str(data.get("module") or "")
        if not any(m["name"] == name for m in config["laser_modules"]):
            raise ValueError("there is no laser module called %r" % name)
        warning = None
        if data.get("clear"):
            offset = None
        elif _float(data, "offset", required=False) is not None:
            offset = _float(data, "offset")
        else:
            z_ref = self._settings.get(["calib_z_ref"])
            if z_ref is None:
                raise ValueError("probe the reference spot first (calibration step 1)")
            if _float(data, "laser_focus_z", required=False) is not None:
                focus = _float(data, "laser_focus_z")
            else:
                focus = results.laser_focus_from_touch(_float(data, "touch_z"),
                                                       _float(data, "focal_distance"))
            offset = results.focus_offset(focus, z_ref)
            warning = results.machine_offset_check(z_ref, self._settings.get(["calib_machine_z"]),
                                                   focus, _float(data, "laser_machine_z", required=False))
        text = config_module.set_module_offset(self._settings.get(["laser_modules"]), name, offset)
        self._store(dict(laser_modules=text, selected_module=name))
        self._log_line("Focus offset for %s: %s." % (name, "cleared" if offset is None else "%+.3f mm" % offset))
        return dict(ok=True, module=name, offset=offset, warning=warning)

    def _api_table(self, data):
        config = self._checked_loose()
        name = data.get("module") or config["selected_module"]
        module = config_module.find_module(config["laser_modules"], name)
        datum = data.get("datum")
        datum = int(datum) if datum not in (None, "") else None
        return self._table(module, datum) or dict(rows=[], text="")

    # -- the height map -------------------------------------------------------

    def _api_map_build(self, data):
        """Fit the last job's points to the firmware mesh. Moves nothing."""
        config = self._checked_loose()
        last = self._last or {}
        if not last.get("points"):
            raise ValueError("run the automatic job over the object first: its points make the map")
        try:
            built = heightmap.build(last["points"], last.get("machine_shift"),
                                    tolerance=config["map_tolerance"], focus_gap=config["map_focus_gap"])
        except heightmap.MapError as exception:
            raise ValueError(str(exception))
        built.update(created=time.time(), module=last.get("module"), points=len(last["points"]),
                     complete=last.get("complete"))
        if last.get("complete") is False:
            built["warnings"].append("The job did not finish: the map is made from the %d points it "
                                     "measured." % len(last["points"]))
        if last.get("xy_calibrated") is False:
            built["warnings"].append("%s has no measured probe XY offset, so the map sits off the "
                                     "beam by that offset." % (last.get("module") or "This module"))
        self._map = built
        self._write_json(MAP_FILE, built)
        self._write_json(BED_MAP_FILE, built)
        self._log_line("Height map from %d points: range %.3f mm, worst miss %+.3f mm, lowest point "
                       "X%.2f Y%.2f (focus LightBurn there)." % (
                           built["points"], built["range"], built["worst"]["miss"],
                           built["base"]["x"], built["base"]["y"]))
        for warning in built["warnings"]:
            self._log_line("Height map: " + warning)
        return dict(ok=True, height_map=self._map_summary())

    # -- obstacles ----------------------------------------------------------

    def _obstacles(self):
        return list(self._read_json(OBSTACLES_FILE) or [])

    def _active_obstacles(self, config, focus_z):
        """The obstacles the module cannot clear at this focus: taller than the burn surface plus
        the module's height above it at focus (FOCAL_10W), less a margin. An object whose own
        red-dot map is loaded is skipped: the printer's mesh already lifts the head over it."""
        try:
            bed_focus = self._dot_bed_focus(config)[0]
            surface = focus_z - bed_focus
        except ValueError:
            surface = 0.0                              # unknown: assume burning on the bed
        limit = surface + calibration.FOCAL_10W - OBSTACLE_CLEARANCE_MARGIN
        loaded = (self._map or {}).get("created") if self._mesh_loaded() else None
        out = []
        for ob in self._obstacles():
            if ob.get("top", 0.0) <= limit:
                continue
            if loaded is not None and ob.get("map_created") == loaded:
                continue
            out.append(ob)
        return out

    def _add_obstacle(self, box, top, name, source, map_created=None):
        obs = [ob for ob in self._obstacles()
               if not (source == "red dot" and ob.get("source") == "red dot"
                       and ob["box"][0] < box[2] and box[0] < ob["box"][2]
                       and ob["box"][1] < box[3] and box[1] < ob["box"][3])]   # a re-scan replaces
        ob = dict(box=[round(float(v), 1) for v in box], top=round(float(top), 1), name=name, source=source,
                  created=time.time(), map_created=map_created)
        obs.append(ob)
        self._write_json(OBSTACLES_FILE, obs)
        self._log_line("Obstacle: %s, LightBurn X%.0f..%.0f Y%.0f..%.0f, %.1f mm tall: flat burns keep the "
                       "laser module off it." % (name, box[0], box[2], box[1], box[3], top))
        return ob

    def _api_obstacle_add(self, data):
        """Mark a box (LightBurn mm) as an object of this height: flat burns keep the module off it."""
        box = data.get("box")
        if not isinstance(box, (list, tuple)) or len(box) != 4:
            raise ValueError("box must be [x0, y0, x1, y1] in LightBurn mm")
        box = [float(v) for v in box]
        box = [min(box[0], box[2]), min(box[1], box[3]), max(box[0], box[2]), max(box[1], box[3])]
        top = config_module.number("closeup_height", data.get("height"))
        if top <= 0:
            raise ValueError("give the object's height in mm (above the bed)")
        ob = self._add_obstacle(box, top, data.get("name") or "a marked object", "marked")
        return dict(ok=True, obstacle=ob, obstacles=self._obstacles())

    def _api_obstacle_clear(self, data):
        """Forget the obstacles (all, or the one at index)."""
        obs = self._obstacles()
        if data.get("index") is not None:
            i = int(data["index"])
            if not 0 <= i < len(obs):
                raise ValueError("no obstacle %d" % i)
            gone = [obs.pop(i)]
        else:
            gone, obs = obs, []
        self._write_json(OBSTACLES_FILE, obs)
        self._log_line("Obstacles cleared: %s." % (", ".join(o.get("name") or "?" for o in gone) or "none"))
        return dict(ok=True, obstacles=obs)

    def _api_map_bed(self, data):
        """Back to the last probed (bed) map after a red-dot map of an object; loaded into the
        printer at once if a map is loaded there, so the stored and loaded maps never differ."""
        bed = self._read_json(BED_MAP_FILE)
        if not bed:
            raise ValueError("no probed bed map is stored: run the automatic probe job and make a map")
        if self._mesh_loaded():
            self._mesh_busy()
        old = self._map
        self._map = bed
        self._write_json(MAP_FILE, bed)
        if self._mesh_loaded():
            try:
                self._api_map_load({})
            except ValueError as exception:
                self._map = old
                self._write_json(MAP_FILE, old)
                raise ValueError("the bed map could not be loaded (%s): the object map is still in use" % exception)
        self._log_line("Back to the bed map (%s points)%s." % (
            bed.get("points"), ", loaded into the printer" if self._mesh_loaded() else ""))
        return dict(ok=True, height_map=self._map_summary())

    def _mesh_busy(self):
        self._refuse_while_running()
        if not self._printer.is_operational():
            raise ValueError("the printer is not connected")
        if self._printer.is_printing() or self._printer.is_paused():
            raise ValueError("the printer is busy with a job")

    def _api_map_load(self, data):
        """Write the height map into the printer's bed mesh and turn levelling on."""
        self._mesh_busy()
        if not self._map:
            raise ValueError("make the height map first")
        if self._map.get("model") != heightmap.MODEL and self._map.get("samples"):
            # fitted before the plugin modelled the A350's own interpolation (Catmull-Rom, Y laid
            # over 27 mm a point): refit from the same samples so the printer applies them
            before = self._map.get("rms")
            try:
                self._map = heightmap.refit(self._map, self._map["start"], self._map["spacing"])
            except heightmap.MapError as exception:
                raise ValueError(str(exception))
            self._write_json(MAP_FILE, self._map)
            self._log_line("Height map refitted for the printer's own interpolation (rms %.3f, was %s)."
                           % (self._map["rms"], before))
        def keep(copy):
            # On disk before the first write, so a failure half way still leaves the guards on.
            if not self._mesh:
                self._mesh = dict(backup=copy, loaded_at=None)
                self._write_json(MESH_FILE, self._mesh)

        self._hold(True)
        try:
            # A second load keeps the first copy, which is the print mesh, not the earlier map.
            backup, loaded = meshload.load(self._bridge, self._map,
                                           backup=(self._mesh or {}).get("backup"), on_backup=keep)
        except meshload.MeshError as exception:
            if exception.restored and self._mesh and not self._mesh.get("loaded_at"):
                self._clear_mesh("Height map not loaded; the printer has its old mesh.")
            elif exception.restored is False:
                self._mesh_warning = str(exception)
            raise ValueError(str(exception))
        finally:
            self._hold(False)
        if loaded is not self._map:
            self._log_line("The printer's grid is at X%g Y%g: the map was refitted to it (worst miss "
                           "%+.3f mm)." % (loaded["start"][0], loaded["start"][1], loaded["worst"]["miss"]))
            self._map = loaded
            self._write_json(MAP_FILE, loaded)
        self._mesh["loaded_at"] = time.time()
        self._mesh["map_nodes"] = loaded["nodes"]
        self._mesh_warning = None
        self._write_json(MESH_FILE, self._mesh)
        self._log_line("Height map loaded into the printer (%s), levelling ON. Focus LightBurn on the "
                       "lowest point, X%.2f Y%.2f. Press Restore print mesh when the laser work is done."
                       % (backup.get("toolhead"), self._map["base"]["x"], self._map["base"]["y"]))
        return dict(ok=True)

    def _api_map_restore(self, data):
        """Put the copied mesh, levelling state and fade height back."""
        self._mesh_busy()
        if not self._mesh:
            raise ValueError("no height map was loaded, so there is nothing to put back")
        self._hold(True)
        try:
            backup = self._mesh["backup"]
            if meshload.matches(self._bridge, backup):
                self._clear_mesh("The printer already has its print mesh: nothing to put back.")
                return dict(ok=True)
            meshload.restore(self._bridge, backup, check_toolhead=True)
        except meshload.MeshError as exception:
            raise ValueError("%s. Power-cycle the printer: that reloads the print mesh from flash"
                             % exception)
        finally:
            self._hold(False)
        self._clear_mesh("Print mesh restored.")
        return dict(ok=True)

    def _clear_mesh(self, text):
        self._mesh = None
        self._mesh_warning = None
        try:
            os.remove(os.path.join(self.get_plugin_data_folder(), MESH_FILE))
        except OSError:
            pass
        self._log_line(text)

    def _check_mesh_after_connect(self):
        """After a reconnect: a power cycle brings the print mesh back from flash."""
        try:
            time.sleep(2.0)
            if not self._mesh:
                return
            grid = meshload.read_state(self._bridge)["grid"]
            if heightmap.same(grid, self._mesh["backup"]["grid"]):
                self._clear_mesh("The printer has its print mesh again (power cycle): height map "
                                 "unloaded.")
            elif not heightmap.same(grid, self._mesh.get("map_nodes")):
                self._mesh_warning = ("After reconnecting, the printer's mesh is neither the height "
                                      "map nor the print mesh copy. Check it before printing.")
                self._log_line(self._mesh_warning)
        except Exception as exception:                       # noqa: BLE001
            self._mesh_warning = ("Could not read the printer's mesh after connecting (%s), so the "
                                  "height map is still marked loaded." % exception)
            self._logger.warning("%s", self._mesh_warning)

    def _check_mesh_before_print(self):
        """A height map is still marked loaded. That is fine for a laser job, never for a print:
        a 3D print over a height map instead of the print mesh drives the nozzle into the plate."""
        self._hold(True)
        try:
            head = meshload.read_toolhead(self._bridge)
            if head.upper() not in meshload.LASER_HEADS:
                if meshload.matches(self._bridge, self._mesh["backup"]):
                    self._clear_mesh("Print start: the printer has its print mesh, height map "
                                     "unloaded.")
                    return
                self._printer.cancel_print()
                self._mesh_warning = ("Print cancelled: the toolhead is %s but the printer's mesh is "
                                      "not the print mesh the plugin copied. Power-cycle the printer "
                                      "(it reloads the print mesh from flash) and reconnect; if the "
                                      "mesh still differs, recalibrate it with NozzleTouch." % head)
                self._log_line(self._mesh_warning)
                self._plugin_manager.send_plugin_message(
                    self._identifier, dict(type="failed", mode="mesh", message=self._mesh_warning))
        except Exception as exception:                       # noqa: BLE001
            self._logger.warning("could not check the toolhead at print start: %s", exception)
        finally:
            self._hold(False)

    def _write_json(self, name, value):
        try:
            with open(os.path.join(self.get_plugin_data_folder(), name), "w") as handle:
                json.dump(value, handle, indent=1)
        except Exception:                                    # noqa: BLE001
            self._logger.exception("could not write %s", name)

    # -- the probe by hand ---------------------------------------------------

    def _api_probe_status(self, data):
        """The probe as the panel shows it. The ESP does not report whether the pin is out, so
        "deployed" is the last hand command, cleared by a trigger (the pin stows itself)."""
        status = self._esp().status()
        status["reachable"] = "error" not in status
        status["deployed"] = self._pin == "down" and status.get("latched") is False
        status["session"] = dict(ok=self._session.ok, reason=self._session.reason)
        return status

    def _api_deploy(self, data):
        self._refuse_while_running()
        esp = self._esp()
        esp.release_held_trigger(time.sleep)   # if still HIGH, the status shows the alarm
        esp.arm()
        self._pin = "down"
        return dict(ok=True)

    def _api_stow(self, data):
        self._refuse_while_running()
        self._esp().stow()
        self._pin = "up"
        return dict(ok=True)

    def _api_reset(self, data):
        self._refuse_while_running()
        self._esp().reset()
        self._pin = "up"
        return dict(ok=True)

    def _api_set_od_mode(self, data):
        """One-off: probe output to open drain, stored in the probe's EEPROM."""
        self._refuse_while_running()
        self._esp().set_od_mode()
        self._log_line("Sent probe_set_od_mode: the probe output is now open drain (stored in "
                       "the probe).")
        return dict(ok=True)

    def _api_check(self, data):
        """The wiring test: touch mode with the pin up must read HIGH. Takes about 2 s."""
        self._refuse_while_running()
        esp = self._esp()
        esp.check()
        time.sleep(CHECK_WAIT_S)
        ok = esp.check_ok()
        self._log_line("Wiring test: %s." % ("passed" if ok else "FAILED (or this probe has no touch mode)"))
        return dict(ok=ok)

    # -- helpers ----------------------------------------------------------

    def _running(self):
        return self._job is not None and self._job.is_alive()

    def _stop_job(self, reason):
        """Stop the job as fast as possible: the abort flag, and wake the bridge too."""
        with self._lock:
            if self._running():
                self._logger.info("stopping the probe job: %s", reason)
                self._job.abort()
                if self._bridge is not None:
                    self._bridge.interrupt()

    def _hold(self, value):
        """Hold any print job while probing, and release it after (as NozzleTouch does).

        OctoPrint lets a job start while the printer looks idle. With the job on hold, not one
        line of it reaches the printer until the head is up and the pin stowed.
        """
        if value == self._held:
            return
        try:
            self._printer.set_job_on_hold(value)
            self._held = value
        except Exception as exception:                       # noqa: BLE001
            self._logger.warning("could not %s the job hold: %s",
                                 "take" if value else "release", exception)
            if not value:
                self._held = False

    def _finished(self):
        """Called by the job thread after its safe end."""
        self._hold(False)

    def _log_line(self, text):
        self._log.append("%s  %s" % (time.strftime("%H:%M:%S"), text))
        self._logger.info("%s", text)

    def _on_message(self, payload):
        kind = payload.get("type")
        if kind == "progress":
            self._progress = payload
        text = payload.get("message")
        result = payload.get("result")
        # Only a job's points make the results table; the calibration touch must not replace it.
        if kind in ("done", "failed", "aborted") and result and payload.get("mode") == "job":
            self._last = result
            self._keep(result)
        if kind == "done" and payload.get("mode") == "here" and result and result.get("points"):
            p = result["points"][0]
            p["time"] = time.time()
            self._here["points"].append(p)
            if self._here.get("datum") is None:
                self._here["datum"] = 0                  # the first result is the datum
            self._keep_here()
            module = config_module.find_module(
                config_module.parse_modules(self._settings.get(["laser_modules"])),
                self._settings.get(["selected_module"]))
            row = self._here_table(module)["last"]
            text = "Probe here X%.2f Y%.2f: trigger Z%.3f, focus Z %s, LightBurn %s." % (
                p["x"], p["y"], p["z"],
                "-" if row["focus"] is None else "%.3f" % row["focus"],
                "-" if row["lightburn"] is None else "%+.3f" % row["lightburn"])
            sq = p.get("square")
            if sq:
                from .probe import SHAPES
                shape = sq.get("shape") or "square"
                total = sq["n"] if shape in ("x", "y") else sq["n"] * sq["n"]
                text = ("%s %.1f mm: highest of %d of %d points at X%.2f Y%.2f, trigger Z%.3f "
                        "(range %.3f mm), focus Z %s, LightBurn %s." % (
                            SHAPES.get(shape, shape).capitalize(), sq["size"], len(sq["points"]), total,
                            p["x"], p["y"], p["z"],
                            sq["range"], "-" if row["focus"] is None else "%.3f" % row["focus"],
                            "-" if row["lightburn"] is None else "%+.3f" % row["lightburn"]))
            payload = dict(payload, here=self._here_table(module))
        elif kind == "done" and payload.get("mode") == "rotary" and result and result.get("verify_only"):
            v = result["verify"]
            text = ("Rotary verify: %d touches where the measurement puts the top, rms %.3f mm, worst %+.3f mm at "
                    "Y%.1f angle %.0f%s." % (len(v["points"]), v["rms"], v["worst"]["miss"], v["worst"]["y"], v["worst"]["b"],
                                             "; the crest is up to %+.2f mm sideways" % v["sideways_worst"]
                                             if v.get("sideways_worst") is not None else ""))
            if self._rotary is not None:
                self._rotary["last_verify"] = dict(time=time.time(), rms=v["rms"], worst=v["worst"], points=v["points"],
                                                   sideways=v.get("sideways"), sideways_worst=v.get("sideways_worst"))
                self._keep_rotary(self._rotary)
        elif kind == "done" and payload.get("mode") == "rotary" and result and result.get("check_only"):
            shift = result.get("housing_shift") or {}
            if shift.get("moved"):
                text = ("The rotary has moved: the housing is X%+.2f Z%+.2f mm from the last measurement. "
                        "The measurement is marked stale: measure the object again." % (shift["dx"], shift["dz"]))
                self._rotary_mark_stale("the housing moved X%+.2f mm since the measurement" % shift["dx"])
            else:
                text = ("The rotary has not moved: the housing is X%+.2f Z%+.2f mm from the last measurement%s."
                        % (shift.get("dx", 0.0), shift.get("dz", 0.0),
                           " (Z differs: the probe may sit differently on the head)" if shift.get("z_differs") else ""))
            if (self._rotary is not None and self._rotary.get("homed_head") == "laser"
                    and getattr(self, "_homed_head", None) != "laser" and shift.get("dx") is not None
                    and abs(shift["dx"]) <= FRAME_FROM_HOUSING_MM):
                # The housing sits where a measurement made in the 10W's frame put it: X is in the
                # 10W's frame (a probe-on home would put it ~12 mm off). Restores the note after a restart.
                self._homed_head = "laser"
                text += (" The housing agrees with the last measurement (made with X homed with the 10W) to "
                         "%.2f mm: X is still in the 10W's frame." % abs(shift["dx"]))
            if self._rotary is not None:
                self._rotary["last_check"] = dict(time=time.time(), **shift)
                self._keep_rotary(self._rotary)
        elif kind in ("done", "failed", "aborted") and payload.get("mode") == "rotary":
            if kind == "done" and result:
                result["homed_head"] = getattr(self, "_homed_head", None)
                self._rotary_partial = None
                self._keep_json(ROTARY_PARTIAL_FILE, None)
                self._rotary = result
                self._keep_rotary(result)
                self._b_frame_new_measurement()
                s = result["summary"]
                fmt = lambda v, f: "-" if v is None else f % v            # noqa: E731
                if s.get("calibration_only"):
                    text = "Calibration run: diameter %s mm, runout spread %s mm." % (
                        fmt(s.get("diameter"), "%.2f"), fmt(s.get("runout_max_spread"), "%.3f"))
                else:
                    text = ("Rotary: diameter %s mm%s, axis X%.3f Z%.3f at Y%.1f, top Z%.3f, body Y%s to Y%s "
                            "(%s mm), runout swing %s mm per turn%s." % (
                                fmt(s.get("diameter"), "%.2f"),
                                " (measured)" if s.get("diameter_measured") else " (typed in)",
                                s["axis_x"], s["axis_z"], s["at_y"], s["top_z"], fmt(s["body_from_y"], "%.1f"),
                                fmt(s["body_to_y"], "%.1f"), fmt(s["body_length"], "%.1f"),
                                fmt(s.get("runout_swing"), "%.2f"),
                                "" if s.get("runout_within") is None else " (%s the %.1f mm tolerance)" % (
                                    "within" if s["runout_within"] else "over", s.get("runout_tolerance") or 3.0)))
                for warning in s.get("warnings") or []:
                    text += " Warning: %s." % warning
                cal = (result.get("reference") or {}).get("calibration")
                if cal:
                    self._store(dict(rotary_ref_offset=cal["offset"], rotary_ref_slope=cal["slope"],
                                     rotary_ref_housing_z=result["reference"]["z"]))
                    text += (" Saved the housing reference: offset %.3f mm, slope %.5f per mm."
                             % (cal["offset"], cal["slope"]))
            elif kind == "failed":
                self._last_error = payload.get("message")
                text = "Failed: %s" % payload.get("message")
            if kind != "done" and result:
                # Keep what was measured, apart, to resume from; the last complete measurement stays
                # in use (a failed re-measure used to replace it and turn rotary mode off), unless
                # this run found the rotary moved.
                self._rotary_partial = result
                self._keep_json(ROTARY_PARTIAL_FILE, result)
                shift = result.get("housing_shift") or {}
                if shift.get("moved") and self._rotary and self._rotary.get("complete"):
                    self._rotary_mark_stale("the housing moved X%+.2f mm since the measurement" % shift["dx"])
                    text = (text or "Stopped.") + (" The housing moved X%+.2f mm, so the last measurement is "
                                                   "marked stale." % shift["dx"])
        elif kind == "done" and payload.get("mode") == "tap":
            text = "Tap seen: Probe here is ready for this session."
        elif kind == "done" and payload.get("mode") == "calibrate" and result and result.get("points"):
            p = result["points"][0]
            self._store(dict(calib_x=p["x"], calib_y=p["y"], calib_z_ref=p["z"],
                             calib_machine_z=p.get("machine_z"), calib_time=time.time()))
            text = "Reference spot X%.2f Y%.2f: probe trigger Z%.3f. Now swap the laser in." % (
                p["x"], p["y"], p["z"])
        elif payload.get("mode") in ("closeup", "closeup_calib") and kind in ("done", "failed", "aborted"):
            if kind == "failed":
                self._last_error = payload.get("message")
                text = "Close-up failed: %s" % payload.get("message")
            elif kind == "done" and result:
                seam = (result.get("qa") or {}).get("seam_mm")
                text = "%s%s" % (payload.get("message"),
                                 " Tiles agree within %.2f mm." % seam if seam is not None else "")
        elif kind == "done":
            text = "Done: %d points measured." % len((result or {}).get("points") or [])
        elif kind == "failed":
            self._last_error = payload.get("message")
            text = "Failed: %s" % payload.get("message")
        if text:
            self._log_line(text)
        self._plugin_manager.send_plugin_message(self._identifier, payload)

    def _keep_here(self):
        try:
            with open(os.path.join(self.get_plugin_data_folder(), HERE_FILE), "w") as handle:
                json.dump(self._here, handle, indent=1)
        except Exception:                                    # noqa: BLE001
            self._logger.exception("could not write the Probe here results")

    def _keep_rotary(self, result):
        try:
            with open(os.path.join(self.get_plugin_data_folder(), ROTARY_FILE), "w") as handle:
                json.dump(result, handle, indent=1)
        except Exception:                                    # noqa: BLE001
            self._logger.exception("could not write the rotary results")

    def _keep(self, result):
        try:
            with open(os.path.join(self.get_plugin_data_folder(), LAST_FILE), "w") as handle:
                json.dump(result, handle, indent=1)
        except Exception:                                    # noqa: BLE001
            self._logger.exception("could not write the results")

    def _read_json(self, name):
        path = os.path.join(self.get_plugin_data_folder(), name)
        if not os.path.exists(path):
            return None
        try:
            with open(path, "r") as handle:
                return json.load(handle)
        except Exception:                                    # noqa: BLE001
            self._logger.exception("could not read %s", path)
            return None


class _CloseupThread(CloseupJob):
    """The close-up job, which tells the plugin when it has finished."""

    def __init__(self, plugin, *args, **kwargs):
        CloseupJob.__init__(self, *args, **kwargs)
        self._plugin = plugin

    def run(self):
        try:
            CloseupJob.run(self)
        finally:
            self._plugin._closeup_finished()


class _Job(Job):
    """The job, which tells the plugin when it has finished."""

    def __init__(self, plugin, *args, **kwargs):
        Job.__init__(self, *args, **kwargs)
        self._plugin = plugin

    def run(self):
        try:
            Job.run(self)
        finally:
            self._plugin._finished()


def __plugin_load__():
    global __plugin_implementation__
    global __plugin_hooks__
    __plugin_implementation__ = LaserHeightProbePlugin()
    __plugin_hooks__ = {
        "octoprint.comm.protocol.gcode.received":
            __plugin_implementation__.gcode_received,
        "octoprint.comm.protocol.gcode.queuing":
            __plugin_implementation__.gcode_queuing,
        "octoprint.comm.protocol.gcode.sending":
            __plugin_implementation__.gcode_sending,
        "octoprint.comm.protocol.gcode.sent":
            __plugin_implementation__.gcode_sent,
        "octoprint.access.permissions":
            __plugin_implementation__.get_additional_permissions,
        "octoprint.server.http.routes":
            __plugin_implementation__.route_hook,
    }
