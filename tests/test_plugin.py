"""The OctoPrint glue, with OctoPrint and flask replaced by stubs.

The plugin class is thin on purpose; these tests check the wiring the pure modules cannot:
every API command has a permission and a handler, the calibration lands in the settings, a
job holds the print queue until its safe end, and results reach the data folder.
"""
import importlib
import json
import logging
import os
import sys
import types

import pytest

from helpers import Rig


def _stub_modules():
    flask = types.ModuleType("flask")

    class Abort(Exception):
        pass
    flask.Abort = Abort

    def abort(code, message=""):
        raise Abort(code, message)
    flask.abort = abort
    flask.jsonify = lambda *a, **k: dict(*a, **k)
    flask.make_response = lambda body, code: (body, code)

    class Response(object):
        def __init__(self, body, mimetype=None, headers=None):
            self.body, self.mimetype, self.headers = body, mimetype, dict(headers or {})
    flask.Response = Response
    babel = types.ModuleType("flask_babel")
    babel.gettext = lambda text: text

    octoprint = types.ModuleType("octoprint")
    plugin = types.ModuleType("octoprint.plugin")
    for name in ("TemplatePlugin", "AssetPlugin", "SimpleApiPlugin", "StartupPlugin",
                 "ShutdownPlugin", "EventHandlerPlugin"):
        setattr(plugin, name, type(name, (object,), {}))

    class BlueprintPlugin(object):
        @staticmethod
        def route(rule, **options):
            return lambda f: f
    plugin.BlueprintPlugin = BlueprintPlugin

    class SettingsPlugin(object):
        def on_settings_save(self, data):
            for key, value in data.items():
                self._settings.set([key], value)
    plugin.SettingsPlugin = SettingsPlugin
    access = types.ModuleType("octoprint.access")
    access.ADMIN_GROUP, access.USER_GROUP = "admins", "users"
    permissions = types.ModuleType("octoprint.access.permissions")

    class Allowed(object):
        def can(self):
            return True

    class Permissions(object):
        PLUGIN_LASERHEIGHTPROBE_VIEW = Allowed()
        PLUGIN_LASERHEIGHTPROBE_PROBE = Allowed()
    permissions.Permissions = Permissions
    events = types.ModuleType("octoprint.events")

    class Events(object):
        DISCONNECTED, ERROR, PRINT_STARTED = "Disconnected", "Error", "PrintStarted"
        CONNECTED = "Connected"
    events.Events = Events
    octoprint.plugin, octoprint.access, octoprint.events = plugin, access, events
    access.permissions = permissions
    return {"flask": flask, "flask_babel": babel, "octoprint": octoprint,
            "octoprint.plugin": plugin, "octoprint.access": access,
            "octoprint.access.permissions": permissions, "octoprint.events": events}


@pytest.fixture
def module(monkeypatch):
    for name, stub in _stub_modules().items():
        monkeypatch.setitem(sys.modules, name, stub)
    sys.modules.pop("lhp_pkg.__init__", None)
    import lhp_pkg
    spec = importlib.util.spec_from_file_location(
        "lhp_pkg.__init__", lhp_pkg.__path__[0] + "/__init__.py",
        submodule_search_locations=None)
    mod = importlib.util.module_from_spec(spec)
    mod.__package__ = "lhp_pkg"
    spec.loader.exec_module(mod)
    return mod


class FakeSettings(object):
    def __init__(self, defaults):
        self.values = dict(defaults)
        self.saves = 0

    def get(self, path):
        return self.values.get(path[0])

    def get_boolean(self, path):
        return bool(self.values.get(path[0]))

    def set(self, path, value):
        self.values[path[0]] = value

    def global_get(self, path):
        return getattr(self, "globals", {}).get(tuple(path))

    def global_set(self, path, value):
        if not hasattr(self, "globals"):
            self.globals = {}
        self.globals[tuple(path)] = value

    def save(self, force=False, trigger_event=False):
        self.saves += 1


class HoldPrinter(object):
    def __init__(self, inner):
        self.inner = inner
        self.holds = []

    def __getattr__(self, name):
        return getattr(self.inner, name)

    def is_printing(self):
        return False

    def is_paused(self):
        return False

    def set_job_on_hold(self, value):
        self.holds.append(value)

    def cancel_print(self):
        self.cancelled = getattr(self, "cancelled", 0) + 1


class Messages(object):
    """The plugin's push channel. A user taps the pin when the tab asks for it."""

    def __init__(self, device=None):
        self.sent = []
        self.device = device

    def send_plugin_message(self, identifier, payload):
        self.sent.append(payload)
        if self.device is not None and payload.get("waiting_for_tap"):
            self.device.tap()


def make_plugin(module, rig, tmp_path):
    p = module.LaserHeightProbePlugin()
    p._settings = FakeSettings(p.get_settings_defaults())
    p._settings.values.update(esp_url="http://laserprobe.test", safe_z=60.0, z_floor=0.5)
    p._printer = HoldPrinter(rig.printer)
    p._plugin_manager = Messages(rig.device)
    p._logger = logging.getLogger("plugin")
    p._identifier = "laserheightprobe"
    p.get_plugin_data_folder = lambda: str(tmp_path)
    p._bridge = rig.bridge
    p._esp = lambda config=None: rig.esp
    p._clock = rig.clock
    return p


def test_every_command_has_a_permission_and_a_handler(module):
    p = module.LaserHeightProbePlugin()
    assert set(p.get_api_commands()) == set(p.COMMAND_PERMISSIONS)
    for command, needed in p.COMMAND_PERMISSIONS.items():
        assert needed in ("VIEW", "PROBE")
        assert callable(getattr(p, "_api_" + command)), command
    keys = [perm["key"] for perm in p.get_additional_permissions()]
    assert keys == ["VIEW", "PROBE"]


def test_no_update_hook_and_the_templates(module):
    module.__plugin_load__()
    assert set(module.__plugin_hooks__) == {"octoprint.comm.protocol.gcode.received",
                                            "octoprint.comm.protocol.gcode.queuing",
                                            "octoprint.comm.protocol.gcode.sending",
                                            "octoprint.comm.protocol.gcode.sent",
                                            "octoprint.access.permissions",
                                            "octoprint.server.http.routes"}
    names = [c["template"] for c in module.LaserHeightProbePlugin().get_template_configs()]
    assert names == ["laserheightprobe_tab.jinja2", "laserheightprobe_control.jinja2",
                     "laserheightprobe_sidebar.jinja2", "laserheightprobe_settings.jinja2"]


def test_settings_save_refuses_unsafe_values(module, tmp_path):
    p = make_plugin(module, Rig(), tmp_path)
    with pytest.raises(ValueError, match="z_floor"):
        p.on_settings_save(dict(z_floor=-1))
    p.on_settings_save(dict(z_floor=1.0))
    assert p._settings.values["z_floor"] == 1.0


def test_job_holds_the_queue_and_keeps_the_results(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._api_save_points(dict(text="40 40 ref\n120 60\n200 150 datum"))
    p._api_start({})
    p._job.join(10)
    assert p._printer.holds == [True, False]
    assert p._plugin_manager.sent[-1]["type"] == "done"
    saved = json.loads((tmp_path / "last_results.json").read_text())
    assert len(saved["points"]) == 3 and saved["datum"] == 2
    table = p._api_table({})
    assert table["rows"][2]["lightburn"] == 0.0 and table["rows"][0]["focus"] is None
    assert "LightBurn_Z" in table["text"]
    assert p._api_table(dict(datum=0))["rows"][0]["lightburn"] == 0.0
    status = p._status()
    assert status["table"]["rows"] and status["modules"][0]["name"] == "10W"


def test_calibration_touch_then_offset(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._api_calibrate_probe(dict(x=150, y=150))
    p._job.join(10)
    z_ref = p._settings.values["calib_z_ref"]
    assert z_ref == pytest.approx(12.73, abs=0.02)            # within one fine step
    assert p._last is None                         # the calibration does not replace results
    out = p._api_set_offset(dict(module="2W IR", laser_focus_z=z_ref + 17.25))
    assert out["offset"] == pytest.approx(17.25) and out["warning"] is None
    modules = module.config_module.parse_modules(p._settings.values["laser_modules"])
    assert modules[1] == dict(name="2W IR", focus_offset=pytest.approx(17.25), dx=None, dy=None)
    assert p._settings.values["selected_module"] == "2W IR"
    p._api_set_offset(dict(module="10W", touch_z=z_ref - 5.0, focal_distance=20.0))
    assert module.config_module.parse_modules(p._settings.values["laser_modules"])[0]["focus_offset"] \
        == pytest.approx(15.0)
    p._api_set_offset(dict(module="10W", offset="-1.5"))
    assert module.config_module.parse_modules(p._settings.values["laser_modules"])[0]["focus_offset"] == -1.5
    with pytest.raises(ValueError):
        p._api_set_offset(dict(module="40W", offset=1))


def test_manual_buttons_are_refused_while_a_job_runs(module, tmp_path):
    p = make_plugin(module, Rig(), tmp_path)

    class Alive(object):
        mode, phase = "job", "probe"

        def is_alive(self):
            return True
    p._job = Alive()
    for command in ("deploy", "stow", "reset", "check", "position"):
        with pytest.raises(ValueError, match="running"):
            getattr(p, "_api_" + command)({})


def test_position_reports_head_and_pin(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._settings.values["laser_modules"] = "10W = , 0, 0\n2W IR = , 10, -2"
    p._settings.values["selected_module"] = "2W IR"
    rig.machine.x, rig.machine.y, rig.machine.z = 100.0, 50.0, 30.0
    pos = p._api_position({})
    assert (pos["x"], pos["y"], pos["z"]) == (100.0, 50.0, 30.0)
    assert (pos["pin_x"], pos["pin_y"]) == (110.0, 48.0) and pos["machine_z"] == pytest.approx(30.0)
    assert pos["xy_calibrated"] is True


def test_grid_command(module, tmp_path):
    p = make_plugin(module, Rig(), tmp_path)
    out = p._api_grid(dict(x0=0, y0=0, x1=10, y1=10, nx=2, ny=2))
    assert out["text"] == "0 0\n10 0\n10 10\n0 10" and out["count"] == 4


def test_heights_are_captured_from_the_machine(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._settings.values.update(safe_z=None, z_floor=None)
    assert p._status()["heights"]["missing"] == ["safe_z", "z_floor"]
    p._api_save_points(dict(text="100 100"))
    with pytest.raises(ValueError, match="safe_z is not set"):
        p._api_start({})
    with pytest.raises(ValueError, match="not set"):
        p._api_calibrate_probe(dict(x=10, y=10))
    assert rig.machine.sent == []
    rig.machine.z = 72.25
    assert p._api_set_height(dict(which="safe_z"))["value"] == pytest.approx(72.25)
    rig.machine.z = 3.5
    p._api_set_height(dict(which="z_floor"))
    assert (p._settings.values["safe_z"], p._settings.values["z_floor"]) == (72.25, 3.5)
    with pytest.raises(ValueError, match="above z_floor"):
        p._api_set_height(dict(which="safe_z", value=2.0))
    with pytest.raises(ValueError):
        p._api_set_height(dict(which="max_descent", value=2.0))
    assert p._status()["heights"]["missing"] == []
    # the rest of the settings can be saved before the heights exist
    p._settings.values.update(safe_z=None)
    p.on_settings_save(dict(probe_feed=200))


def test_xy_offset_is_saved_per_module(module, tmp_path):
    p = make_plugin(module, Rig(), tmp_path)
    p._api_set_xy_offset(dict(module="2W IR", dx="-12.5", dy=4))
    modules = module.config_module.parse_modules(p._settings.values["laser_modules"])
    assert modules[0]["dx"] == 0.0 and (modules[1]["dx"], modules[1]["dy"]) == (-12.5, 4.0)
    p._api_set_xy_offset(dict(module="2W IR", clear=True))
    assert module.config_module.parse_modules(p._settings.values["laser_modules"])[1]["dx"] is None


def test_open_drain_button(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._esp = lambda config=None: rig.esp
    p._api_set_od_mode({})
    assert rig.device.presses[-1] == "probe_set_od_mode"


def test_probe_here_through_the_plugin(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._settings.values.update(safe_z=None, z_floor=None)       # not needed for Probe here
    with pytest.raises(ValueError, match="tap test first"):
        p._api_here({})
    assert rig.machine.sent == []
    p._api_tap_test({})
    p._job.join(10)
    assert p._session.ok and p._printer.holds == []          # the tap test holds nothing
    for x, z in ((40.0, 20.0), (80.0, 22.0)):
        rig.machine.x, rig.machine.z = x, z
        p._api_here({})
        p._job.join(10)
        assert p._plugin_manager.sent[-1]["type"] == "done", p._plugin_manager.sent[-1]
    assert p._printer.holds == [True, False, True, False]
    here = p._status()["here"]
    assert [r["x"] for r in here["rows"]] == [40.0, 80.0]
    assert here["rows"][0]["is_datum"] and here["rows"][1]["lightburn"] == 0.0
    assert "LightBurn_Z" in here["text"]
    saved = json.loads((tmp_path / "here_results.json").read_text())
    assert len(saved["points"]) == 2 and saved["datum"] == 0
    p._api_here_datum(dict(index=1))
    assert p._status()["here"]["rows"][1]["is_datum"]
    with pytest.raises(ValueError):
        p._api_here_datum(dict(index=5))
    out = p._api_here_as_reference(dict(index=0))
    assert p._settings.values["calib_z_ref"] == out["z_ref"]
    p._api_here_clear({})
    assert p._status()["here"]["rows"] == []


def test_probe_here_is_refused_while_printing(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._session.passed(object())
    p._printer.is_printing = lambda: True
    with pytest.raises(ValueError, match="busy"):
        p._api_here({})


def test_probe_status_reports_the_pin_as_last_commanded(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._api_deploy({})
    status = p._api_probe_status({})
    assert status["reachable"] and status["deployed"] is True and status["latched"] is False
    rig.device.tap()                                          # trips and stows itself
    assert p._api_probe_status({})["deployed"] is False
    p._api_stow({})
    rig.device.fail = {"/binary_sensor/": -1, "/sensor/": -1}
    status = p._api_probe_status({})
    assert status["reachable"] is False and status["deployed"] is False


def test_probe_here_square_through_the_plugin(module, tmp_path):
    rig = Rig(surface=lambda x, y: 4.0 + 0.2 * (x - 40.0))
    p = make_plugin(module, rig, tmp_path)
    p._settings.values.update(safe_z=None, z_floor=None)
    p._api_tap_test({})
    p._job.join(10)
    for bad in (dict(square=True, size=0.1), dict(square=True, size=3, n=1), dict(square=True, n=12)):
        with pytest.raises(ValueError, match="here_square"):
            p._api_here(bad)
    rig.machine.x, rig.machine.y, rig.machine.z = 40.0, 50.0, 15.0
    p._api_here(dict(square=True))                            # the saved defaults: 3 mm, 3 x 3
    p._job.join(30)
    done = p._plugin_manager.sent[-1]
    assert done["type"] == "done", done
    row = p._status()["here"]["rows"][-1]
    assert row["x"] == 41.5                                   # the highest column
    assert any("Square 3.0 mm: highest of 9 of 9 points" in line for line in p._log)
    rig.machine.x, rig.machine.y = 40.0, 50.0
    p._api_here(dict(square=True, size=2.0, n=2))
    p._job.join(30)
    assert p._status()["here"]["rows"][-1]["x"] == 41.0


def test_height_map_build_load_and_restore(module, tmp_path):
    import copy
    rig = Rig(surface=lambda x, y: 10.0 + 0.05 * (x - 120))
    rig.machine.xy_shift = (6.0, -4.0)            # a work origin: G-code X, Y != machine X, Y
    p = make_plugin(module, rig, tmp_path)
    p._api_save_points(dict(text="120 100\n150 100\n180 100\n120 140\n150 140\n180 140"))
    p._api_start({})
    p._job.join(10)
    assert p._last["machine_shift"] == [6.0, -4.0]
    with pytest.raises(ValueError, match="make the height map first"):
        p._api_map_load({})
    built = p._api_map_build({})["height_map"]
    assert built["base"]["x"] == 120.0 and built["base"]["machine_x"] == 126.0
    assert built["base"]["machine_z"] is not None
    assert any("no measured probe XY offset" in w for w in built["warnings"]) is False
    assert built["range"] == pytest.approx(3.0, abs=0.03)
    assert abs(built["worst"]["miss"]) < 0.01
    assert (tmp_path / "height_map.json").exists()

    print_mesh = copy.deepcopy(rig.machine.mesh["z"])
    p._api_map_load({})
    assert rig.machine.levelling is True and rig.machine.mesh["z"] == built["nodes"]
    assert p._status()["mesh"]["loaded"] and (tmp_path / "mesh_backup.json").exists()
    p._api_map_load({})                           # again: the copy is still the print mesh
    assert p._mesh["backup"]["grid"] == print_mesh

    p._api_map_restore({})
    assert rig.machine.mesh["z"] == print_mesh and rig.machine.levelling is False
    assert not p._status()["mesh"]["loaded"] and not (tmp_path / "mesh_backup.json").exists()
    with pytest.raises(ValueError, match="nothing to put back"):
        p._api_map_restore({})


def test_a_power_cycle_clears_the_loaded_map(module, tmp_path, monkeypatch):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._map = module.heightmap.build([dict(x=150.0, y=150.0, z=10.0), dict(x=180.0, y=150.0, z=11.0),
                                     dict(x=150.0, y=180.0, z=10.5)], (0.0, 0.0))
    import copy
    print_mesh = copy.deepcopy(rig.machine.mesh["z"])
    p._api_map_load({})
    monkeypatch.setattr(module.time, "sleep", lambda s: None)
    p._check_mesh_after_connect()                 # reconnect, mesh still the map: stays loaded
    assert p._mesh is not None
    rig.machine.mesh["z"] = print_mesh            # power cycle: flash brings the print mesh back
    rig.machine.levelling = False
    p._check_mesh_after_connect()
    assert p._mesh is None


def test_a_print_on_a_3d_head_with_a_map_marked_loaded_is_cancelled(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._map = module.heightmap.build([dict(x=150.0, y=150.0, z=10.0), dict(x=180.0, y=150.0, z=11.0),
                                     dict(x=150.0, y=180.0, z=10.5)], (0.0, 0.0))
    p._api_map_load({})
    p._check_mesh_before_print()                  # a laser job through OctoPrint: allowed
    assert not getattr(p._printer, "cancelled", 0)
    rig.machine.toolhead = "3DP"
    p._check_mesh_before_print()
    assert p._printer.cancelled == 1 and "Power-cycle" in p._status()["mesh"]["warning"]
    assert p._mesh is not None


def test_a_print_on_a_3d_head_with_the_print_mesh_back_goes_ahead(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._map = module.heightmap.build([dict(x=150.0, y=150.0, z=10.0), dict(x=180.0, y=150.0, z=11.0),
                                     dict(x=150.0, y=180.0, z=10.5)], (0.0, 0.0))
    import copy
    print_mesh = copy.deepcopy(rig.machine.mesh["z"])
    p._api_map_load({})
    rig.machine.toolhead, rig.machine.mesh["z"] = "3DP", print_mesh     # swapped: power cycled
    p._check_mesh_before_print()
    assert not getattr(p._printer, "cancelled", 0) and p._mesh is None


def test_probing_is_refused_while_a_map_is_loaded(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._map = module.heightmap.build([dict(x=150.0, y=150.0, z=10.0), dict(x=180.0, y=150.0, z=11.0),
                                     dict(x=150.0, y=180.0, z=10.5)], (0.0, 0.0))
    p._api_map_load({})
    p._session.passed(object())
    with pytest.raises(ValueError, match="Restore print mesh"):
        p._api_here({})
    p._api_save_points(dict(text="150 150"))
    with pytest.raises(ValueError, match="Restore print mesh"):
        p._api_start({})


def test_a_load_that_cannot_put_the_old_mesh_back_keeps_the_guards_on(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._map = module.heightmap.build([dict(x=150.0, y=150.0, z=10.0), dict(x=180.0, y=150.0, z=11.0),
                                     dict(x=150.0, y=180.0, z=10.5)], (0.0, 0.0))
    real = rig.machine.execute

    def broken(command):                       # every M421 refused, so the restore fails too
        if command.startswith("M421"):
            return ["Error:Mesh XY or IJ cannot be resolved", "ok"]
        return real(command)
    rig.machine.execute = broken
    with pytest.raises(ValueError, match="failed too"):
        p._api_map_load({})
    assert p._mesh is not None and (tmp_path / "mesh_backup.json").exists()
    assert "failed too" in p._status()["mesh"]["warning"]


def test_a_load_that_fails_cleanly_leaves_no_mark(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._map = module.heightmap.build([dict(x=150.0, y=150.0, z=10.0), dict(x=180.0, y=150.0, z=11.0),
                                     dict(x=150.0, y=180.0, z=10.5)], (0.0, 0.0))
    rig.machine.fail_m421_at = (3, 3)
    with pytest.raises(ValueError, match="old mesh is back"):
        p._api_map_load({})
    assert p._mesh is None and not (tmp_path / "mesh_backup.json").exists()


def test_reconnect_with_an_unknown_mesh_warns(module, tmp_path, monkeypatch):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._map = module.heightmap.build([dict(x=150.0, y=150.0, z=10.0), dict(x=180.0, y=150.0, z=11.0),
                                     dict(x=150.0, y=180.0, z=10.5)], (0.0, 0.0))
    p._api_map_load({})
    monkeypatch.setattr(module.time, "sleep", lambda s: None)
    rig.machine.mesh["z"][4][4] = 7.0
    p._check_mesh_after_connect()
    assert p._mesh is not None and "neither" in p._status()["mesh"]["warning"]


class FakeFiles(object):
    def __init__(self, root):
        self.root = root
        self.added = {}

    def path_on_disk(self, origin, path):
        assert origin == "local"
        return os.path.join(self.root, path)

    def add_file(self, origin, path, wrapper, allow_overwrite=False):
        import io
        buf = io.BytesIO()
        wrapper.save(os.path.join(self.root, path))
        with open(os.path.join(self.root, path)) as f:
            self.added[path] = f.read()
        return path


def test_rotary_translate_writes_a_snapmaker_file(module, tmp_path):
    import shutil
    p = make_plugin(module, Rig(), tmp_path)
    p._file_manager = FakeFiles(str(tmp_path))
    shutil.copy(os.path.join(os.path.dirname(__file__), "data", "lightburn", "fill.gc"),
                str(tmp_path / "flask.gc"))
    with pytest.raises(ValueError, match="no complete rotary"):
        p._api_rotary_translate(dict(path="flask.gc"))
    p._rotary = dict(
        complete=True, params=dict(diameter=78.0),
        axis=dict(y0=130.0, x0=164.27, z0=104.42, dx_dy=0.004, dz_dy=0.0007),
        summary=dict(body_from_y=25.0, body_to_y=216.25),
        runout=[dict(y=60.0, e=0.7, phase_deg=270.0), dict(y=200.0, e=0.35, phase_deg=270.0)])
    with pytest.raises(ValueError, match="focus offset is not set"):
        p._api_rotary_translate(dict(path="flask.gc"))
    p._settings.values["laser_modules"] = "10W = 5.75, 0, 0"
    p._settings.values["selected_module"] = "10W"
    res = p._api_rotary_translate(dict(path="flask.gc"))
    assert res["output"] == "flask.snapmaker.gcode"
    text = p._file_manager.added["flask.snapmaker.gcode"]
    assert "M112" not in text and "G91" not in text and text.rstrip().endswith("; end")
    assert res["report"]["y_from"] == pytest.approx(25.0 + 66.0, abs=0.6)
    with pytest.raises(ValueError, match="must not overwrite"):
        p._api_rotary_translate(dict(path="flask.gc", output="flask.gc"))



class BridgeSender(object):
    def __init__(self):
        self.lines, self.urgent_lines = [], []

    def send(self, lines, counted, gen=None):
        self.lines += [(l, counted) for l in lines]

    def urgent(self, lines):
        self.urgent_lines += lines

    def position(self):
        return None


def rotary_result():
    return dict(
        complete=True, params=dict(diameter=78.0),
        axis=dict(y0=130.0, x0=164.27, z0=104.42, dx_dy=0.004, dz_dy=0.0007),
        summary=dict(body_from_y=25.0, body_to_y=216.25, top_z=143.42),
        runout=[dict(y=60.0, e=0.7, phase_deg=270.0)])


def test_bridge_mode_binds_a_measurement_and_arms_the_laser(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    with pytest.raises(ValueError, match="bridge is off"):
        p._api_bridge_mode(dict(mode="rotary"))
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    with pytest.raises(ValueError, match="no complete rotary"):
        p._api_bridge_mode(dict(mode="rotary"))
    p._rotary = rotary_result()
    with pytest.raises(ValueError, match="focus offset"):
        p._api_bridge_mode(dict(mode="rotary"))
    p._settings.values["laser_modules"] = "10W = 5.75, 0, 0"
    p._settings.values["selected_module"] = "10W"
    res = p._api_bridge_mode(dict(mode="rotary"))
    assert res["mode"] == "rotary" and not res["laser"]
    assert p._grbl.context.laser_armed()[0] is False
    with pytest.raises(ValueError, match="safety state is 0x4, it is tilted"):
        p._api_bridge_laser(dict(laser=True))                 # the fake 10W lies in the bay
    p._printer.machine.security, p._printer.machine.roll = 0, 1
    p._api_bridge_laser(dict(laser=True))
    assert p._grbl.context.laser_armed()[0] is True
    st = p._status()["bridge"]
    assert st["mode"] == "rotary" and st["laser_seconds_left"] > 0
    p._api_bridge_mode(dict(mode="off"))
    assert p._grbl.context is None


def test_bridge_mode_refuses_a_work_offset(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig(shift=11.0)
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._rotary = rotary_result()
    p._settings.values["laser_modules"] = "10W = 5.75, 0, 0"
    with pytest.raises(ValueError, match="work offset"):
        p._api_bridge_mode(dict(mode="rotary", laser=True))
    assert p._grbl.context is None


def test_flat_mode_takes_focus_from_the_last_probe_here(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._settings.values["laser_modules"] = "10W = 5.75, 0, 0"
    with pytest.raises(ValueError, match="no Probe here"):
        p._api_bridge_mode(dict(mode="flat"))
    p._here = dict(points=[dict(x=100.0, y=120.0, z=20.0, machine_z=20.0)], datum=0)
    with pytest.raises(ValueError, match="before the bridge"):
        p._api_bridge_mode(dict(mode="flat"))
    import time as _t
    p._here["points"][0]["time"] = _t.time() - 7200
    with pytest.raises(ValueError, match="too old"):
        p._api_bridge_mode(dict(mode="flat"))
    p._here["points"][0]["time"] = _t.time()
    res = p._api_bridge_mode(dict(mode="flat"))
    assert p._grbl.context.bounds == (-2.0, 0.0, 318.0, 350.0)       # M211 limits less the X2 offset
    assert p._grbl.context.focus_z == pytest.approx(25.75) and "25.750" in res["describe"]


def test_a_probe_job_disarms_the_bridge_and_the_sent_hook_counts_tagged_lines(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._settings.values["laser_modules"] = "10W = 5.75, 0, 0"
    import time as _t
    p._here = dict(points=[dict(x=100.0, y=120.0, z=20.0, machine_z=20.0, time=_t.time())], datum=0)
    p._api_bridge_mode(dict(mode="flat"))
    p._grbl.inflight.append(("G1 X1", (1.0, 2.0), 0.5, "move"))
    p.gcode_sent(None, "sent", "G1 X1", None, "G1", tags={grbl_bridge.TAG})
    assert not p._grbl.inflight and p._grbl.pos[:2] == (1.0, 2.0)
    p._bridge_disarm("test")
    assert p._grbl.context is None



def test_lightburn_over_tcp_end_to_end(module, tmp_path):
    """A real socket client against the plugin's listener and the fake Snapmaker: banner, the
    blank-line ok, status, a refusal before a mode is picked, then a rotary jog that reaches the
    printer as B-axis lines, and a disconnect that stops and turns the laser off."""
    import socket as so
    import time as t
    from lhp_pkg import grbl_bridge
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    sent = []
    inner = p._printer

    class Tap(object):                       # OctoPrint calls the sent hook for every line
        def __getattr__(self, name):
            return getattr(inner, name)

        def commands(self, commands, tags=None, **kwargs):
            inner.commands(commands, tags=tags)
            for c in commands:
                sent.append((c, set(tags or ())))
                p.gcode_sent(None, "sent", c, None, c.split()[0], tags=set(tags or ()))
    p._printer = Tap()
    probe = so.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    p._settings.values.update(bridge_enabled=True, bridge_port=port, bridge_allow="127.0.0.1",
                              laser_modules="10W = 5.75, 0, 0")
    p._rotary = rotary_result()
    p._bridge_start()
    try:
        c = so.create_connection(("127.0.0.1", port), timeout=5)
        f = c.makefile("rb")

        def line():
            return f.readline().decode().strip()
        while True:
            l = line()
            if l.startswith("Grbl"):
                break
        c.sendall(b"?\n")
        assert line().startswith("<Idle|MPos:")
        assert line() == "ok"                                  # the blank line after "?"
        c.sendall(b"$J=G91G21  Z0 A10 F800\n")
        assert line().startswith("[MSG:") and line() == "error:9"
        p._api_bridge_mode(dict(mode="rotary"))
        c.sendall(b"$J=G90G21  X50 Y0 F800\n$J=G91G21  Z0 A10 F800\n")
        assert line() == "ok" and line() == "ok"
        deadline = t.time() + 5
        while t.time() < deadline and not any(" B-10.000" in s for s, tags in sent):
            t.sleep(0.05)
        assert any(s.startswith("G1 ") and " B-10.000" in s for s, tags in sent), sent[-5:]
        assert all(grbl_bridge.TAG in tags or grbl_bridge.TAG + "-direct" in tags for s, tags in sent)
        f.close()                             # makefile() holds the socket open
        c.close()
        deadline = t.time() + 5
        while t.time() < deadline and p._grbl_client is not None:
            t.sleep(0.05)
        assert p._grbl_client is None
        assert "M410" in [s for s, tags in sent] and "M5" in [s for s, tags in sent]
    finally:
        p._bridge_stop()



def test_the_sending_hook_drops_stale_bridge_lines(module, tmp_path):
    from lhp_pkg import grbl_bridge
    p = make_plugin(module, Rig(), tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    gen = p._grbl.gen
    tags = {grbl_bridge.TAG, "%s-gen:%d" % (grbl_bridge.TAG, gen)}
    assert p.gcode_sending(None, "sending", "G1 X1 S10", None, "G1", tags=tags) is None
    p._grbl.reset()
    assert p.gcode_sending(None, "sending", "G1 X1 S10", None, "G1", tags=tags) == (None,)
    direct = {grbl_bridge.TAG + "-direct"}
    assert p.gcode_sending(None, "sending", "M5", None, "M5", tags=direct) is None



def test_bridge_mode_accepts_the_laser_home_offset(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    rig.machine.home_offset = (-19.0, -10.0, 0.0)
    rig.machine.xy_shift = (19.0, 10.0)
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._rotary = rotary_result()
    p._settings.values["laser_modules"] = "10W = 5.75, 0, 0"
    assert p._api_bridge_mode(dict(mode="rotary"))["mode"] == "rotary"



def test_a_stale_rotary_measurement_is_refused_everywhere(module, tmp_path):
    from lhp_pkg import grbl_bridge
    p = make_plugin(module, Rig(), tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._rotary = rotary_result()
    p._settings.values["laser_modules"] = "10W = 5.75, 0, 0"
    p._api_bridge_mode(dict(mode="rotary"))
    p._api_rotary_stale(dict(reason="flask reseated"))
    assert p._grbl.context is None                          # the rotary mode is dropped
    with pytest.raises(ValueError, match="stale .flask reseated."):
        p._api_bridge_mode(dict(mode="rotary"))
    with pytest.raises(ValueError, match="no complete rotary"):
        p._api_rotary_translate(dict(path="x.gc"))
    assert p._status()["rotary"]["complete"] is False


def test_bridge_mode_needs_the_beam_offset_and_the_plugged_in_laser(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    rig.machine.toolhead = "2W RED LASER"
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._rotary = rotary_result()
    p._settings.values["laser_modules"] = "10W = 7.9, 0, 0\n2W IR = 73.95"
    p._settings.values["selected_module"] = "10W"
    with pytest.raises(ValueError, match="2W IR beam offset"):
        p._api_bridge_mode(dict(mode="rotary"))          # M1006 picked the 2W IR: no dx, dy yet
    assert p._settings.values["selected_module"] == "2W IR"
    p._settings.values["laser_modules"] = "10W = 7.9, 0, 0\n2W IR = 73.95, 1.0, -3.0"
    rig.machine.toolhead = "LASER"                       # a laser M1006 cannot name
    with pytest.raises(ValueError, match="2W IR selected but the printer has the LASER plugged in"):
        p._api_bridge_mode(dict(mode="rotary"))
    rig.machine.toolhead = "3DP"
    with pytest.raises(ValueError, match="not a laser"):
        p._api_bridge_mode(dict(mode="rotary"))
    rig.machine.toolhead = "10W LASER"
    p._api_bridge_mode(dict(mode="rotary"))
    assert p._grbl.context.module == "10W"
    assert p._grbl.context.beam_dx == 0.0

def test_the_mode_must_match_whether_m114_reports_the_rotary(module, tmp_path):
    from lhp_pkg import grbl_bridge
    import time as _t
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._rotary = rotary_result()
    p._settings.values["laser_modules"] = "10W = 5.75, 0, 0"
    p._here = dict(points=[dict(x=100.0, y=120.0, z=20.0, machine_z=20.0, time=_t.time())], datum=0)
    with pytest.raises(ValueError, match="rotary module is attached"):
        p._api_bridge_mode(dict(mode="flat"))
    rig.machine.rotary = False
    with pytest.raises(ValueError, match="does not report the rotary"):
        p._api_bridge_mode(dict(mode="rotary"))
    assert p._grbl.context is None
    p._api_bridge_mode(dict(mode="flat"))
    assert p._grbl.context.mode == "flat"
    lines = ["X:1.00 Y:2.00 Z:3.00 E:0.00 B:0.00 Count X:0 Y:0 Z:0", "B unusable", "ok"]
    assert "unusable" in module.rotary_mismatch("rotary", lines)
    assert "did not report" in module.rotary_mismatch("flat", ["ok"])
    assert module.rotary_mismatch("off", []) is None


def test_flat_mode_adds_the_module_beam_offset(module, tmp_path):
    from lhp_pkg import grbl_bridge
    import time as _t
    rig = Rig()
    rig.machine.toolhead = "2W RED LASER"
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._settings.values["laser_modules"] = "10W = 7.9, 0, 0\n2W IR = 73.95, 1.0, -3.0"
    p._settings.values["selected_module"] = "2W IR"
    p._here = dict(points=[dict(x=100.0, y=120.0, z=20.0, machine_z=20.0, time=_t.time())], datum=0)
    p._api_bridge_mode(dict(mode="flat"))
    ctx = p._grbl.context
    assert (ctx.ox, ctx.oy) == (pytest.approx(3.0), pytest.approx(-3.0))
    assert ctx.focus_z == pytest.approx(93.95) and ctx.module == "2W IR"


def test_a_rotary_job_gets_the_housing_reference_and_a_calibration_is_saved(module, tmp_path):
    p = make_plugin(module, Rig(), tmp_path)
    p._session.passed(object())
    p._homed_head = "laser"                                  # X homed with the 10W seated
    started = {}
    p._start = lambda points, mode, config: started.update(mode=mode, config=config) or dict(started=True)
    p._api_rotary(dict(params=dict(axis_x=164.3, crown=143.0, stations=[60, 200], y_chuck_limit=245)))
    r = started["config"]["rotary"]
    assert r["ref_offset"] == pytest.approx(28.678) and r["ref_slope"] == pytest.approx(0.002321)
    assert r["steps"][0] == "reference" and r["diameter"] is None
    result = rotary_result()
    result["summary"].update(diameter=73.6, diameter_measured=73.6, warnings=[], axis_x=164.3, axis_z=104.6,
                             at_y=130.0, body_length=191.25, runout_max_spread=0.4)
    result["reference"] = dict(z=133.5, calibration=dict(offset=28.5, slope=0.002, slope_measured=True))
    p._on_message(dict(type="done", mode="rotary", result=result))
    assert p._settings.values["rotary_ref_offset"] == 28.5 and p._settings.values["rotary_ref_slope"] == 0.002
    assert "diameter 73.60 mm (measured)" in " ".join(p._log)


def test_a_calibration_only_rotary_run_saves_the_calibration(module, tmp_path):
    p = make_plugin(module, Rig(), tmp_path)
    result = dict(params={}, complete=False, runout=[dict(y=60.0, mean=143.5, spread=0.5)],
                  reference=dict(z=133.46, calibration=dict(offset=28.788, slope=0.00127, slope_measured=True)),
                  summary=dict(calibration_only=True, diameter=78.4, diameter_measured=78.4, warnings=[],
                               runout_max_spread=0.5))
    p._on_message(dict(type="done", mode="rotary", result=result))
    assert p._settings.values["rotary_ref_offset"] == 28.788 and p._settings.values["rotary_ref_slope"] == 0.00127
    assert p._settings.values["rotary_ref_housing_z"] == 133.46
    log = " ".join(p._log)
    assert "Calibration run" in log and "78.40" in log and "Saved the housing reference" in log


def test_detect_picks_the_module_and_reads_the_rotary_and_the_lasers_safety(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._settings.values["laser_modules"] = "2W IR = 73.95\n10W = 7.9, 0, 0"
    p._settings.values["selected_module"] = "2W IR"
    d = p._api_detect({})["detected"]
    assert d["toolhead"] == "10W LASER" and d["laser"] and d["module"] == "10W"
    assert p._settings.values["selected_module"] == "10W"          # follows what is fitted
    assert d["rotary"] and not d["rotary_unusable"]
    assert d["security"]["state"] == 4 and d["security"]["roll"] == -86 and d["will_fire"] is False
    assert p._status()["detected"]["module"] == "10W"
    rig.machine.rotary, rig.machine.security, rig.machine.roll = False, 0, 1
    d = p._api_detect({})["detected"]
    assert not d["rotary"] and d["will_fire"] is True
    rig.machine.toolhead = "LASER"                   # a laser M1006 cannot name: keep the hand pick
    p._settings.values["selected_module"] = "2W IR"
    d = p._api_detect({})["detected"]
    assert d["laser"] and d["module"] is None and not d["module_known"]
    assert p._settings.values["selected_module"] == "2W IR"
    rig.machine.toolhead = "3DP"
    d = p._api_detect({})["detected"]
    assert not d["laser"] and d["security"] is None and d["module_known"]


def test_the_laser_safety_report_parses():
    from lhp_pkg import gcode
    lines = ["Laser security state: 0x4",
             "Laser temp: 24, imu temp: 29, roll: -86, pitch: 2, pwm_pin_pulldown_state_: 0, pwm_pin_pullup_state_: 1"]
    assert gcode.laser_security(lines) == dict(state=4, temp=24, roll=-86, pitch=2)
    assert gcode.laser_security(["ok"]) is None


def test_a_laser_refuses_heating_and_cancels_a_3d_print(module, tmp_path):
    import time as _t
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    cancelled = []
    p._printer.cancel_print = lambda: cancelled.append(True)
    assert p.gcode_queuing(None, "queuing", "M104 S210", None, "M104") is None   # nothing detected yet
    p._api_detect({})
    assert p.gcode_queuing(None, "queuing", "M104 S210", None, "M104") == (None,)
    assert p.gcode_queuing(None, "queuing", "M140 S0", None, "M140") is None     # off is fine
    assert p.gcode_queuing(None, "queuing", "G1 X10 S40", None, "G1") is None     # laser motion
    p._printer.is_printing = lambda: True
    assert p.gcode_queuing(None, "queuing", "M190 S60", None, "M190") == (None,)
    _t.sleep(0.05)
    assert cancelled and "refused \"M190 S60\", and cancelled the print" in " ".join(p._log)
    rig.machine.toolhead = "3DP"
    p._printer.is_printing = lambda: False
    p._api_detect({})
    assert p.gcode_queuing(None, "queuing", "M104 S210", None, "M104") is None


def test_the_pointer_is_the_10w_beam_capped_or_the_2w_crosshair_and_needs_the_laser_on_the_head(module, tmp_path):
    from lhp_pkg import grbl_bridge
    import time as _t
    rig = Rig()
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._settings.values["laser_modules"] = "10W = 7.9, 0, 0\n2W IR = 73.95, 1.0, -3.0"
    p._here = dict(points=[dict(x=100.0, y=120.0, z=20.0, machine_z=20.0, time=_t.time())], datum=0)
    p._api_bridge_mode(dict(mode="flat"))
    with pytest.raises(ValueError, match="tilted"):
        p._api_bridge_pointer(dict(on=True))
    rig.machine.security, rig.machine.roll = 0, 0
    res = p._api_bridge_pointer(dict(on=True))
    assert res["pointer"]["kind"] == "spot" and res["pointer"]["s_max"] == 10
    assert p._status()["bridge"]["pointer"]["seconds_left"] > 0
    p._api_bridge_pointer(dict(on=False))
    assert p._status()["bridge"]["pointer"] is None
    rig.machine.toolhead = "2W RED LASER"
    p._api_bridge_mode(dict(mode="flat"))                   # the 2W IR now, picked from M1006
    res = p._api_bridge_pointer(dict(on=True))
    assert res["pointer"] == dict(kind="cross", dx=21.5, dy=-11.0, until=None)


def test_beam_offset_from_two_jogs_on_one_dot(module, tmp_path):
    from lhp_pkg import offsets
    assert offsets.beam_offset((100.0, 50.0), (100.4, 49.7)) == (0.4, -0.3)
    assert offsets.beam_offset((100.0, 50.0), (78.0, 61.0), crosshair=(21.5, -11.0)) == (-0.5, 0.0)
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    with pytest.raises(ValueError, match="pin on the dot first"):
        p._api_beam_offset(dict(step="beam"))
    rig.machine.x, rig.machine.y = 100.0, 50.0
    p._api_beam_offset(dict(step="pin"))
    rig.machine.toolhead = "2W RED LASER"
    rig.machine.x, rig.machine.y = 78.0, 61.0
    p._api_detect({})
    res = p._api_beam_offset(dict(step="beam"))
    assert (res["dx"], res["dy"]) == (-0.5, 0.0) and res["module"] == "2W IR"


def test_the_pin_record_survives_an_octoprint_restart(module, tmp_path):
    # The laser swap can need OctoPrint restarted; the pin record must not be lost with it.
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    rig.machine.x, rig.machine.y = 4.8, 5.6
    p._api_beam_offset(dict(step="pin"))
    q = make_plugin(module, rig, tmp_path)                  # a new plugin: OctoPrint restarted
    rig.machine.x, rig.machine.y = 6.8, 3.1
    q._api_detect({})
    res = q._api_beam_offset(dict(step="beam"))
    assert (res["dx"], res["dy"]) == (pytest.approx(2.0), pytest.approx(-2.5))


def test_the_hand_pointer_needs_the_laser_on_the_head_and_switches_off(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    with pytest.raises(ValueError, match="tilted"):
        p._api_pointer(dict(on=True))
    rig.machine.security, rig.machine.roll = 0, 0
    assert p._api_pointer(dict(on=True))["pointer"] == "spot"
    assert "M3 S10" in rig.machine.sent
    p._api_pointer(dict(on=False))
    assert rig.machine.sent[-1] == "M5" or "M5" in rig.machine.sent[-3:]


# -- turn the rotary a quarter at a time (Chris, 2026-11-19: level the object with a spirit level) --

def _moves(rig):
    return [c for c in rig.machine.sent if c.startswith(("G0", "G1"))]


def test_rotary_turn_lifts_first_stows_then_turns_a_quarter(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    rig.machine.z, rig.machine.b = 140.0, 0.0
    rig.device.deployed = True
    out = p._api_rotary_turn({})
    assert out["b"] == pytest.approx(90.0) and rig.machine.b == pytest.approx(90.0)
    moves = _moves(rig)
    lift = [i for i, c in enumerate(moves) if "Z160" in c]
    turn = [i for i, c in enumerate(moves) if " B" in c]
    assert lift and turn and lift[0] < turn[0]
    assert rig.device.deployed is False and rig.machine.z == pytest.approx(160.0)


def test_rotary_turn_never_lowers_the_head_and_turns_to_an_angle(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    rig.machine.z, rig.machine.b = 250.0, 270.0
    assert p._api_rotary_turn(dict(by=-90))["b"] == pytest.approx(180.0)
    assert p._api_rotary_turn(dict(to=0))["b"] == pytest.approx(0.0)
    assert rig.machine.z == pytest.approx(250.0)
    assert not [c for c in _moves(rig) if "Z" in c]


def test_rotary_turn_clears_a_tall_measured_object(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._rotary = dict(params=dict(crown=158.0), stations=[dict(y=60.0, crown=158.4)])
    rig.machine.z = 100.0
    p._api_rotary_turn({})
    assert rig.machine.z == pytest.approx(168.4)


def test_rotary_turn_refuses_without_the_rotary_or_mid_job(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    rig.machine.rotary = False
    with pytest.raises(ValueError, match="rotary"):
        p._api_rotary_turn({})
    assert not _moves(rig)
    rig.machine.rotary = True
    with pytest.raises(ValueError, match="degrees"):
        p._api_rotary_turn(dict(by=1000))

    class Alive(object):
        mode, phase = "job", "probe"

        def is_alive(self):
            return True
    p._job = Alive()
    with pytest.raises(ValueError, match="running"):
        p._api_rotary_turn({})


def test_the_receive_hook_repairs_an_ok_glued_into_the_laser_report(module, tmp_path):
    # 2026-11-19 23:09:53: OctoPrint stalled on "...pitch: 0, pwm_ok" until a fake ack.
    p = make_plugin(module, Rig(), tmp_path)
    assert p.gcode_received(None, "Laser temp: 29, imu temp: 37, roll: 0, pitch: 0, pwm_ok") == "ok"
    assert p.gcode_received(None, "Laser security state: 0x0") == "Laser security state: 0x0"
    assert p.gcode_received(None, "ok") == "ok"


M114_B = "X:290.00 Y:150.00 Z:200.00 E:0.00 B:%.2f Count X:123600 Y:64000 Z:80000 B:%d"


def test_a_power_cycle_with_b_away_from_zero_keeps_the_object_angle(module, tmp_path):
    # 2026-11-19 22:44: the 10W came off at B-68; after the power cycle the firmware said B0 and
    # the 2W IR jobs followed the runout 68 degrees out of step.
    p = make_plugin(module, Rig(), tmp_path)
    p._b_frame = dict(last_b=0.0, shift=0.0, check=False)
    p.gcode_received(None, M114_B % (-68.0, -60445))
    assert p._b_frame["last_b"] == -68.0
    p._b_frame_reconnected()
    p.gcode_received(None, M114_B % (0.0, 0))              # reset: wherever B was is now "B0"
    assert p._b_frame["shift"] == pytest.approx(-68.0) and p._b_frame["last_b"] == 0.0
    # a second swap adds on
    p.gcode_received(None, M114_B % (30.0, 26667))
    p._b_frame_reconnected()
    p.gcode_received(None, M114_B % (0.0, 0))
    assert p._b_frame["shift"] == pytest.approx(-38.0)
    # it survives an OctoPrint restart
    assert json.load(open(os.path.join(str(tmp_path), "b_frame.json")))["shift"] == pytest.approx(-38.0)


def test_a_reconnect_without_a_reset_changes_nothing(module, tmp_path):
    p = make_plugin(module, Rig(), tmp_path)
    p._b_frame = dict(last_b=0.0, shift=5.0, check=False)
    p.gcode_received(None, M114_B % (-10.0, -8889))
    p._b_frame_reconnected()
    p.gcode_received(None, M114_B % (-10.0, -8889))        # the firmware kept its count
    assert p._b_frame["shift"] == 5.0 and p._b_frame["check"] is False
    # B genuinely at 0 before and after: no shift either
    p.gcode_received(None, M114_B % (0.0, 0))
    p._b_frame_reconnected()
    p.gcode_received(None, M114_B % (0.0, 0))
    assert p._b_frame["shift"] == 5.0


def test_a_new_rotary_measurement_starts_a_fresh_angle_frame(module, tmp_path):
    p = make_plugin(module, Rig(), tmp_path)
    p._b_frame = dict(last_b=0.0, shift=-68.0, check=False)
    p._b_frame_new_measurement()
    assert p._b_frame["shift"] == 0.0


def test_the_crosshair_macro_is_for_crosshair_heads_only(module, tmp_path):
    from lhp_pkg import grbl_bridge
    import time as _t
    rig = Rig()
    rig.machine.rotary = False
    rig.machine.security, rig.machine.roll = 0, 0
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._settings.values["laser_modules"] = "10W = 7.9, 0, 0\n2W IR = 73.95, 1.0, -3.0"
    p._here = dict(points=[dict(x=100.0, y=120.0, z=20.0, machine_z=20.0, time=_t.time())], datum=0)
    with pytest.raises(ValueError, match="bridge mode"):
        p._macro_pointer(True)
    p._api_bridge_mode(dict(mode="flat"))
    with pytest.raises(ValueError, match="no crosshair"):
        p._macro_pointer(True)                                # the 10W: its pointer is the beam
    assert p._status()["bridge"]["pointer"] is None
    p._macro_pointer(False)                                   # off is always fine
    rig.machine.toolhead = "2W RED LASER"
    p._api_bridge_mode(dict(mode="flat"))
    p._macro_pointer(True)
    assert p._status()["bridge"]["pointer"]["kind"] == "cross"
    p._macro_pointer(False)
    assert p._status()["bridge"]["pointer"] is None


def test_probe_here_from_a_given_height_above_the_bed(module, tmp_path):
    # Flat bed, like the rotary's known diameter: the bed is touched once ("Use as bed"), then
    # an object's height gets the head to just above it without jogging it down by hand.
    bed, block = 4.0, 6.0
    rig = Rig(surface=lambda x, y: bed + (block if x >= 100 else 0.0))
    p = make_plugin(module, rig, tmp_path)
    p._settings.values.update(safe_z=None, z_floor=None)
    p._api_tap_test({})
    p._job.join(10)
    rig.machine.x, rig.machine.z = 100.0, 40.0
    with pytest.raises(ValueError, match="bed"):
        p._api_here(dict(height=6.0))                        # no bed reference yet
    rig.machine.x, rig.machine.z = 40.0, 12.0
    p._api_here({})
    p._job.join(10)
    out = p._api_here_as_bed({})
    bed_z = out["bed_z"]                                     # a trigger Z: the surface + pin travel
    assert bed_z == pytest.approx(bed + 2.5, abs=0.05)
    rig.machine.x, rig.machine.z = 120.0, 40.0
    p._api_here(dict(height=6.0))
    p._job.join(10)
    assert p._plugin_manager.sent[-1]["type"] == "done", p._plugin_manager.sent[-1]
    row = p._status()["here"]["rows"][-1]
    assert row["z"] == pytest.approx(bed_z + block, abs=0.05)
    assert any(abs(z - (bed_z + block + 6.0)) < 0.01 for z in rig.gcode_z_commands())   # the drop
    assert rig.machine.z == pytest.approx(40.0)              # back where it was left
    rig.machine.z = 40.0
    p._api_here(dict(height=0.5))                            # it is really 6 mm: taller than said
    p._job.join(10)
    done = p._plugin_manager.sent[-1]
    assert done["type"] == "failed" and "taller than" in done["message"]
    rig.machine.z = 8.0                                      # below where the height puts the start
    p._api_here(dict(top=bed_z + block))
    p._job.join(10)
    done = p._plugin_manager.sent[-1]
    assert done["type"] == "failed" and "raise" in done["message"]


def test_flat_bridge_arms_with_the_height_map_loaded(module, tmp_path):
    # 2026-11-20 03:40: with the map loaded, the levelling shows in M114's Z count, and the
    # bridge took that for a work offset (Z+0.64) and refused. The map's own height at the head
    # is not a work offset; a real G92 Z still is.
    from lhp_pkg import grbl_bridge
    rig = Rig(surface=lambda x, y: 10.0 + 0.02 * (x - 120))
    rig.machine.security, rig.machine.roll = 0, 0
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._api_save_points(dict(text="120 100\n200 100\n280 100\n120 200\n200 200\n280 200"))
    p._api_start({})
    p._job.join(10)
    p._api_map_build({})
    p._api_map_load({})
    rig.machine.x, rig.machine.y = 250.0, 150.0              # where the mesh is well above 0
    assert abs(rig.machine.levelling_dz()) > 0.5
    out = p._api_bridge_mode(dict(mode="flat", focus_z=30.0, dry_run=True))
    assert out["mode"] == "flat"
    p._api_bridge_mode(dict(mode="off"))
    rig.machine.shift += 0.05                                # within the slack (the model matches to 0.002)
    assert p._api_bridge_mode(dict(mode="flat", focus_z=30.0, dry_run=True))["mode"] == "flat"
    p._api_bridge_mode(dict(mode="off"))
    rig.machine.shift += 0.2                                 # 0.25 mm: a real Z work offset now
    with pytest.raises(ValueError, match="work offset"):
        p._api_bridge_mode(dict(mode="flat", focus_z=30.0, dry_run=True))


class HeadcamScreen(object):
    """The touchscreen's camera API: the photo is new after one capture request."""

    def __init__(self, machine=None):
        self.calls, self.photo, self.machine = [], b"\xff\xd8old\xff\xd9", machine

    def get(self, url, timeout):
        self.calls.append(url)
        if "request_capture_photo" in url:
            if self.machine is not None:
                self.at = (self.machine.x, self.machine.y, self.machine.z)
            self.photo = b"\xff\xd8new bed photo\xff\xd9"
            return 200, b'{"status":true}'
        return 200, self.photo


def test_a_head_photo_lifts_first_moves_to_the_photo_spot_and_keeps_the_jpeg(module, tmp_path):
    # The 10W's camera: the head goes up to Z290 before any XY move (nothing on the bed or the
    # rotary is that tall), then to Luban's photo spot, so the touchscreen has nothing to move.
    import os
    rig = Rig()
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    screen = HeadcamScreen(rig.machine)
    p._headcam_get = screen.get
    with pytest.raises(ValueError, match="headcam_url"):
        p._api_head_photo({})
    p._settings.values["headcam_url"] = "http://screen.test:8080"
    rig.machine.x, rig.machine.y, rig.machine.z = 10.0, 300.0, 21.4
    res = p._api_head_photo({})
    assert res["ok"] and res["bytes"] == len(b"\xff\xd8new bed photo\xff\xd9")
    assert screen.at == (232.0, 178.0, 290.0)                 # already there when it asks
    moves = [c for c in rig.machine.sent if c.startswith("G0")]
    assert moves[0].startswith("G0 Z290") and "X" not in moves[0]
    # approached from -X, -Y like every close-up photo (X backlash ~0.42 mm): the close-up's
    # overview base must be where the camera model says
    assert moves[1].startswith("G0 X230.0 Y176.0")
    assert moves[2].startswith("G0 X232") and "Y178" in moves[2]
    assert not any(c.startswith("G54") or c.startswith("G53") for c in rig.machine.sent)
    path = p._headcam_path()
    assert open(path, "rb").read() == b"\xff\xd8new bed photo\xff\xd9"
    assert os.path.dirname(path) == str(tmp_path)
    assert p._status()["headcam"]["bytes"] == res["bytes"]
    assert p.is_blueprint_protected() is False                # LightBurn's camera cannot log in


def test_a_head_photo_needs_the_10w_and_an_idle_machine(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    p._headcam_get = HeadcamScreen().get
    p._settings.values["headcam_url"] = "http://screen.test:8080"
    rig.machine.toolhead = "2W RED LASER"
    with pytest.raises(ValueError, match="no camera"):
        p._api_head_photo({})
    rig.machine.toolhead = "10W LASER"
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._grbl.busy_until = p._grbl.clock() + 60.0
    with pytest.raises(ValueError, match="LightBurn"):
        p._api_head_photo({})
    assert not any(c.startswith("G0") for c in rig.machine.sent)


def test_the_head_photo_is_served_from_memory_so_a_reader_cannot_lock_it(module, tmp_path,
                                                                      monkeypatch):
    # On Windows a file that LightBurn is still reading cannot be replaced (WinError 5), and
    # LightBurn polls the photo every second: on 2026-11-20 every head_photo failed that way.
    # So the route serves the photo from memory and the disk copy is only kept for a restart.
    import os
    rig = Rig()
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    p._settings.values["headcam_url"] = "http://screen.test:8080"
    p._headcam_get = HeadcamScreen(rig.machine).get
    with pytest.raises(module.flask.Abort):
        p.headcam_image()                                     # no photo yet: 404
    real_replace = os.replace

    def locked(src, dst):
        raise PermissionError(5, "Access is denied")
    monkeypatch.setattr(module.os, "replace", locked)
    res = p._api_head_photo({})
    assert res["ok"]
    response = p.headcam_image()
    assert response.body == b"\xff\xd8new bed photo\xff\xd9"
    assert response.mimetype == "image/jpeg"
    assert response.headers["Cache-Control"] == "no-store"
    assert not os.path.exists(p._headcam_path() + ".part")    # no litter left behind
    monkeypatch.setattr(module.os, "replace", real_replace)
    p._headcam_get = HeadcamScreen(rig.machine).get
    p._api_head_photo({})
    restarted = make_plugin(module, rig, tmp_path)            # after a restart: from the disk copy
    assert restarted.headcam_image().body == b"\xff\xd8new bed photo\xff\xd9"


def test_the_camera_macro_is_the_head_photo(module, tmp_path):
    rig = Rig()
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    screen = HeadcamScreen(rig.machine)
    p._headcam_get = screen.get
    p._settings.values["headcam_url"] = "http://screen.test:8080"
    p._macro_camera()
    assert any("request_capture_photo" in c for c in screen.calls)


def _fake_tornado():
    tornado = types.ModuleType("tornado")
    web = types.ModuleType("tornado.web")
    iostream = types.ModuleType("tornado.iostream")

    class StreamClosedError(Exception):
        pass

    class RequestHandler(object):
        def __init__(self, **kwargs):
            self.headers, self.written, self.flushes = {}, [], 0
            self.initialize(**kwargs)

        def set_header(self, name, value):
            self.headers[name] = value

        def write(self, chunk):
            self.written.append(chunk)

        async def flush(self):
            self.flushes += 1
            if self.flushes > 3:                    # LightBurn closes the camera
                raise StreamClosedError()
    web.RequestHandler, iostream.StreamClosedError = RequestHandler, StreamClosedError
    tornado.web, tornado.iostream = web, iostream
    return {"tornado": tornado, "tornado.web": web, "tornado.iostream": iostream}


def test_the_head_photo_stream_is_a_route_that_sends_the_latest_photo_each_second(
        module, tmp_path, monkeypatch):
    # LightBurn holds an MJPEG stream open (as it does go2rtc's bed camera) but stops polling
    # a still JPEG, so it gets /plugin/laserheightprobe/headcam.mjpg. Open like headcam.jpg.
    import asyncio
    for name, stub in _fake_tornado().items():
        monkeypatch.setitem(sys.modules, name, stub)
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    routes = p.route_hook([])
    assert len(routes) == 2                          # the head photo, then the close-up (test_closeup_plugin)
    rule, handler_class, kwargs = routes[0]
    assert rule == r"/headcam.mjpg"
    photos = [None, b"\xff\xd8one\xff\xd9", b"\xff\xd8two\xff\xd9"]
    kwargs = dict(kwargs, frames=lambda: photos.pop(0) if photos else b"\xff\xd8two\xff\xd9",
                  interval=0.0)
    handler = handler_class(**kwargs)
    asyncio.run(handler.get())                       # ends quietly when the client goes
    from lhp_pkg import headcam
    assert handler.headers["Content-Type"] == headcam.MJPEG_TYPE
    assert handler.headers["Cache-Control"] == "no-store"
    assert handler.written[0] == headcam.mjpeg_part(b"\xff\xd8one\xff\xd9")
    assert handler.written[1] == headcam.mjpeg_part(b"\xff\xd8two\xff\xd9")
    assert len(handler.written) == 4                 # no photo yet: nothing sent for that tick
    assert routes[0][2]["frames"] == p._headcam_bytes


# -- air purifier ----------------------------------------------------------------

class _NoHA(object):
    ready = False

    def switch(self, entity, on):
        raise AssertionError("no socket without a token")


def _air_plugin(module, tmp_path):
    from lhp_pkg import air, purifier
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._detected = dict(laser=True, toolhead="10W LASER")
    sent = []
    config = p._air_config()
    assert config["enabled"] and config["ha_token"] == "" and config["switch"] == ""
    p._air = air.AirController(config, sent.extend, p._air_watts, ha=_NoHA())
    return p, sent


def test_purifier_lines_stay_out_of_the_bridge_count(module, tmp_path):
    from lhp_pkg import grbl_bridge
    p, sent = _air_plugin(module, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._grbl.inflight.append(("G0 X1", (1.0, 2.0), 0.5, "move"))
    p._air.pending = (["M1011 F3", "M1010 S4 P100"], 0.0)
    out = p.gcode_queuing(None, "queuing", "G0 X1", None, "G0", tags={grbl_bridge.TAG})
    assert out == [("G0 X1", None), ("M1011 F3", None, {module.AIR_TAG}), ("M1010 S4 P100", None, {module.AIR_TAG})]
    # OctoPrint gives the added line the original's tags too; the sent hook must skip it
    p.gcode_sent(None, "sent", "M1011 F3", None, "M1011", tags={grbl_bridge.TAG, module.AIR_TAG})
    assert len(p._grbl.inflight) == 1
    p.gcode_sent(None, "sent", "G0 X1", None, "G0", tags={grbl_bridge.TAG})
    assert not p._grbl.inflight


def test_heaters_are_still_refused_with_the_purifier_waiting(module, tmp_path):
    p, sent = _air_plugin(module, tmp_path)
    p._air.pending = (["M1011 F3"], 0.0)
    assert p.gcode_queuing(None, "queuing", "M104 S200", None, "M104") == (None,)


def test_laser_lines_are_metered_and_the_status_reports_the_purifier(module, tmp_path):
    p, sent = _air_plugin(module, tmp_path)
    for line in ("G1 X0 Y0 F600", "M3 S255", "G1 X10"):
        p.gcode_sent(None, "sent", line, None, line.split()[0], tags=set())
    assert p._air.meter.energy_j == pytest.approx(10.0)
    status = p._status()["purifier"]
    assert status["dose_j"] == pytest.approx(10.0) and status["phase"] == "burning"
    assert status["ha"] is False


def test_purifier_stop_turns_the_fan_off(module, tmp_path):
    p, sent = _air_plugin(module, tmp_path)
    out = p._api_purifier_stop({})
    assert sent[-1] == "M1011 F0" and out["purifier"]["power"] is False


def test_the_token_is_admin_only(module):
    p = module.LaserHeightProbePlugin()
    assert ["purifier_ha_token"] in p.get_settings_restricted_paths()["admin"]


def test_a_bad_entity_id_is_refused(module, tmp_path):
    from lhp_pkg import settings
    with pytest.raises(settings.SettingsError):
        settings.purifier_config(dict(purifier_switch="not an entity"))
    with pytest.raises(settings.SettingsError):
        settings.purifier_config(dict(purifier_ha_url="ftp://x"))


def test_an_implausible_decay_time_is_never_stored(module, tmp_path):
    p, sent = _air_plugin(module, tmp_path)
    p._log_line = lambda text: None
    stored = []
    p._store = lambda values: stored.append(values)
    p._air_learned(9000.0)
    import time as _t
    _t.sleep(0.1)
    assert stored == []


def test_arming_is_refused_with_the_door_open_and_an_opening_door_disarms(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._settings.values["laser_modules"] = "10W = 5.75, 0, 0"
    import time as _t
    p._here = dict(points=[dict(x=100.0, y=120.0, z=20.0, machine_z=20.0, time=_t.time())], datum=0)
    p._api_bridge_mode(dict(mode="flat"))
    real_run = p._bridge.run
    p._bridge.run = lambda cmds, timeout=60.0: (["Enclosure door: Open"] if cmds == ["M1010"] else real_run(cmds, timeout))
    with pytest.raises(ValueError, match="door is open"):
        p._api_bridge_laser(dict(laser=True))
    assert p._status()["door"]["state"] == "open"
    # armed, then the door opens mid-job: disarmed at once
    p._grbl.context.laser_until = _t.monotonic() + 600
    p._door = (_t.time(), "closed")
    p.gcode_received(None, "laser limit: 10")
    assert p._grbl.context.laser_until is None


def test_opening_the_door_stops_the_purifier(module, tmp_path):
    p, sent = _air_plugin(module, tmp_path)
    p._log_line = lambda text: None
    for line in ["G1 X0 Y0 F600", "M3 S255"] + ["G1 X%d" % (10 * (i % 2)) for i in range(60)]:
        p.gcode_sent(None, "sent", line, None, line.split()[0], tags=set())
    p._air.last_sent = 0.0                              # not streaming: the gear goes at once
    p._air.step()
    assert p._air.status()["power"]
    p._door = (0.0, "closed")
    p.gcode_received(None, "laser limit: 10")
    import time as _t
    for _ in range(50):
        if "M1011 F0" in sent[-2:]:
            break
        _t.sleep(0.02)
    assert sent[-2:] == ["M1011 F0", "M1010 S4 P0"]      # purifier off, and the enclosure fan it started
    assert p._air.status()["power"] is False


def test_the_filter_is_read_only_when_idle(module, tmp_path):
    p, sent = _air_plugin(module, tmp_path)
    p._log_line = lambda text: None
    asked = []
    real_run = p._bridge.run

    def run(cmds, timeout=60.0):
        if cmds == ["M1011"]:
            asked.append(1)
            return ["Purifier Filter lifetime:low", "Purifier Fan gears: 1"]
        return real_run(cmds, timeout)
    p._bridge.run = run
    p._air.policy.add_burn(10.0, 0.0)          # a job in progress: hands off
    p._air_filter_check()
    assert asked == []
    p._air.policy._reset()
    p._air_filter_check()
    assert asked == [1] and p._air.status()["filter"]["lifetime"] == "low"


class PluginManagerWithThermal(Messages):
    """Messages plus the plugin manager calls for ThermalRunaway."""

    def __init__(self, device=None, enabled=False):
        Messages.__init__(self, device)
        self.info = type("Info", (), {"enabled": enabled})()
        self.calls = []

    def get_plugin_info(self, key, require_enabled=True):
        return self.info if key == "ThermalRunaway" else None



def test_a_3d_print_head_with_thermalrunaway_off_refuses_heat_until_ready(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._plugin_manager = PluginManagerWithThermal(rig.device, enabled=False)
    rig.machine.toolhead = "3DP"
    assert p.gcode_queuing(None, "queuing", "M104 S210", None, "M104") == (None,)       # not detected yet
    p._api_detect({})
    assert p._status()["swap"]["thermal"] == "disabled"
    assert p.gcode_queuing(None, "queuing", "M104 S210", None, "M104") == (None,)
    assert p.gcode_queuing(None, "queuing", "M104 S0", None, "M104") is None          # off is fine
    p._settings.global_set(["plugins", "_disabled"], ["ThermalRunaway", "other"])
    out = p._api_ready_to_print({})
    assert p._settings.global_get(["plugins", "_disabled"]) == ["other"] and out["restart"]
    assert p._status()["swap"]["thermal_saved"] == "enabled"
    p._plugin_manager.info.enabled = True                                               # after the restart
    assert not p._status()["swap"]["restart"]
    assert p.gcode_queuing(None, "queuing", "M104 S210", None, "M104") is None


def test_ready_for_laser_turns_thermalrunaway_off_and_each_button_refuses_the_wrong_head(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._plugin_manager = PluginManagerWithThermal(rig.device, enabled=True)
    with pytest.raises(ValueError):
        p._api_ready_to_print({})                                                       # the laser is on
    out = p._api_ready_for_laser({})
    assert p._settings.global_get(["plugins", "_disabled"]) == ["ThermalRunaway"] and out["restart"]
    rig.machine.toolhead = "3DP"
    with pytest.raises(ValueError):
        p._api_ready_for_laser({})


def test_no_thermalrunaway_plugin_changes_nothing(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    rig.machine.toolhead = "3DP"
    p._api_detect({})
    assert p._thermal_state() == "missing"
    assert p.gcode_queuing(None, "queuing", "M104 S210", None, "M104") is None


def test_flat_mode_focus_at_a_height_above_the_bed(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._settings.values["laser_modules"] = "10W = 8.6, 0, 0"
    p._settings.values["flat_bed_z"] = 13.6
    res = p._api_bridge_mode(dict(mode="flat", height=0))
    assert p._grbl.context.focus_z == pytest.approx(22.2) and "bed Z13.60" in res["describe"]
    p._api_bridge_mode(dict(mode="off"))
    res = p._api_bridge_mode(dict(mode="flat", height=3.0))
    assert p._grbl.context.focus_z == pytest.approx(25.2) and "3.0 mm above the bed" in res["describe"]
    assert p._status()["flat_focus"].get("bed_z") == pytest.approx(22.2), p._status()["flat_focus"]
    # with a red-dot object map in the printer, the bed's focus is unknown: refused
    p._api_bridge_mode(dict(mode="off"))
    p._map = dict(source="red dot", base=dict(focus=40.0))
    p._mesh = dict(loaded_at=1.0)
    with pytest.raises(ValueError, match="Back to the bed map"):
        p._flat_bed_focus(p._checked_loose())
    assert "Back to the bed map" in p._flat_focus_status()["error"]


def _measured():
    r = rotary_result()
    r["summary"].update(axis_x=155.9, top_z=144.1, body_from_y=26.25, body_to_y=236.25)
    r["reference"] = dict(centre_x=152.8, z=133.45, y=322.5)
    r["params"].update(y_chuck_limit=250.0, max_diameter=82)
    return r


def test_a_re_measure_starts_from_the_last_measurement(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    raw = {}
    with pytest.raises(ValueError, match="no earlier complete measurement"):
        p._rotary = None
        p._rotary_seed(raw)
    p._rotary = _measured()
    raw = dict(free_end_guess=40.0)
    p._rotary_seed(raw)
    assert raw["axis_x"] == 155.9 and raw["crown"] == 144.1 and raw["ref_x"] == 152.8
    assert raw["seed_ref"] == dict(centre_x=152.8, z=133.45)
    assert raw["free_end_guess"] == 40.0 and raw["chuck_end_guess"] == 236.25   # the caller's guess wins
    assert raw["y_chuck_limit"] == 250.0


def test_a_failed_re_measure_keeps_the_measurement_unless_the_rotary_moved(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._log_line = lambda text: None
    p._rotary = _measured()
    partial = dict(complete=False, reference=dict(centre_x=152.9, z=133.4), housing_shift=dict(dx=0.1, dz=-0.05, moved=False))
    p._on_message(dict(type="failed", mode="rotary", result=partial, message="station Y50: no touch"))
    assert p._rotary["complete"] is True and p._rotary_partial is partial      # still usable; resumable
    assert p._status()["rotary"]["partial"] is True
    moved = dict(partial, housing_shift=dict(dx=-12.8, dz=2.9, moved=True))
    p._on_message(dict(type="failed", mode="rotary", result=moved, message="station Y50: no touch"))
    assert p._rotary["complete"] is False and "moved" in p._rotary["stale"]["reason"]


def test_the_housing_check_marks_a_moved_rotary_stale(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._log_line = lambda text: None
    p._rotary = _measured()
    still = dict(check_only=True, housing_shift=dict(dx=0.12, dz=0.4, moved=False, z_differs=False))
    p._on_message(dict(type="done", mode="rotary", result=still))
    assert p._rotary["complete"] is True and p._rotary["last_check"]["dx"] == 0.12
    assert p._rotary["summary"]["axis_x"] == 155.9                              # a check never replaces it
    moved = dict(check_only=True, housing_shift=dict(dx=-12.8, dz=2.9, moved=True, z_differs=True))
    p._on_message(dict(type="done", mode="rotary", result=moved))
    assert p._rotary["complete"] is False and p._rotary["stale"]


def test_ready_checks_say_what_stops_a_burn(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._settings.values["laser_modules"] = "10W = 8.6, 0, 0"
    p._settings.values["flat_bed_z"] = 13.6
    p._api_detect({})
    levels = lambda: dict((c["key"], c["level"]) for c in p._status()["ready"])      # noqa: E731
    first = levels()
    assert first["head"] == "bad"                         # the fake 10W lies in the bay (safety 0x4)
    assert first["mode"] == "info" and first["lightburn"] == "warn"     # the automatic mode is on its way
    p._printer.machine.security, p._printer.machine.roll = 0, 1
    p._api_detect({})
    p._door = (0.0, "closed")
    p._api_bridge_mode(dict(mode="flat", height=0))
    now = levels()
    assert now["head"] == "ok" and now["door"] == "ok" and now["mode"] == "ok" and now["frame"] == "info"
    # a framed job beside an obstacle taller than the module clears
    p._api_obstacle_add(dict(box=[112, 145, 182, 235], height=38))
    p._grbl.last_frame = dict(bounds=[60.0, 150.0, 96.0, 200.0], time=__import__("time").time(), mode="flat")
    assert levels()["frame"] == "bad"
    p._grbl.last_frame = dict(bounds=[10.0, 150.0, 30.0, 200.0], time=__import__("time").time(), mode="flat")
    assert levels()["frame"] == "ok"
    p._door = (0.0, "open")
    assert levels()["door"] == "bad"


def test_focus_at_the_last_frame_measures_its_centre(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._detected = dict(laser=True, toolhead="10W LASER", module="10W")
    with pytest.raises(ValueError, match="Frame in LightBurn"):
        p._api_focus_frame({})
    seen = []
    p._api_dot_focus = lambda data: seen.append(data) or dict(ok=True, focus_z=24.0)
    p._grbl.last_frame = dict(bounds=[60.0, 100.0, 100.0, 160.0], time=__import__("time").time(), mode="flat")
    out = p._api_focus_frame({})
    assert seen == [dict(x=80.0, y=130.0)] and out["frame"]["bounds"] == [60.0, 100.0, 100.0, 160.0]


def test_frame_focus_needs_the_10w(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._grbl.last_frame = dict(bounds=[60.0, 100.0, 100.0, 160.0], time=__import__("time").time(), mode="flat")
    p._detected = dict(laser=True, toolhead="2W LASER", module="2W IR")
    with pytest.raises(ValueError, match="only the 10W"):
        p._api_focus_frame({})


def test_the_mode_sets_itself_from_whether_the_rotary_is_plugged_in(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    rig.machine.rotary = False
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._settings.values["laser_modules"] = "10W = 8.6, 0, 0"
    p._settings.values["flat_bed_z"] = 13.6
    p._settings.values["flat_focus_height"] = 3.0
    p._homed = True
    p._api_detect({})
    out = p._auto_mode_step()
    assert p._grbl.context.mode == "flat" and p._grbl.context.focus_z == pytest.approx(25.2)
    assert p._grbl.context.laser_until is None                         # never armed
    assert out is not None and p._auto_mode_step() is None              # a mode is set: nothing more to do
    # Off pauses it until a mode is picked again
    p._api_bridge_mode(dict(mode="off"))
    assert p._auto_mode_step() is None and p._grbl.context is None
    assert any("pressed Off" in c["text"] for c in p._status()["ready"])
    p._api_bridge_mode(dict(mode="flat", height=0))
    assert not p._auto_paused and p._settings.values["flat_focus_height"] == 0.0


def test_with_the_rotary_plugged_in_the_mode_is_rotary_or_says_why_not(module, tmp_path):
    from lhp_pkg import grbl_bridge
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    p._settings.values["laser_modules"] = "10W = 5.75, 0, 0"
    p._settings.values["selected_module"] = "10W"
    p._homed = True
    p._api_detect({})
    p._rotary = None
    assert p._auto_mode_step() is None and "measurement is missing" in p._auto_why
    assert any("cannot be set" in c["text"] for c in p._status()["ready"])
    p._auto_retry_at = 0.0
    p._rotary = rotary_result()
    p._auto_mode_step()
    assert p._grbl.context.mode == "rotary" and p._grbl.context.laser_until is None


def test_rotary_probe_jobs_need_x_homed_with_the_10w_seated(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._session = type("S", (), {"ok": True, "reason": None})()
    p._rotary = _measured()
    p._homed_head = "probe"
    with pytest.raises(ValueError, match="last homed with the probe on"):
        p._api_rotary_check({})
    p._homed_head = None
    with pytest.raises(ValueError, match="not homed with the 10W"):
        p._api_rotary(dict(params={"seed": "last"}))


def test_only_a_home_that_moves_x_forgets_which_head_x_was_homed_with(module, tmp_path):
    """The rotary routine's G28 Y (back from the housing touch) leaves X alone: it must not mark
    X as homed with the probe (seen 2026-11-23: the next measure was refused)."""
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    started = []
    p._note_homed_head = lambda *a, **k: started.append(1)
    for line in ("G28 Y", "G28 Z", "G28 Y Z", "g28 y"):
        p._homed_head = "laser"
        p.gcode_sent(None, "sent", line, None, "G28", tags=set())
        assert p._homed_head == "laser", line
    assert not started
    for line in ("G28", "G28 X", "G28 X Y", "G28 X0 Y0 Z0", "G28 O"):
        p._homed_head = "laser"
        p.gcode_sent(None, "sent", line, None, "G28", tags=set())
        assert p._homed_head is None, line


def test_the_housing_check_restores_the_10w_frame_after_an_octoprint_restart(module, tmp_path):
    """A restart forgets which head X was homed with. The housing check may run anyway (it reads
    the frame), and a housing within 3 mm of a measurement made in the 10W's frame proves X is
    still in that frame; ~12 mm off proves nothing (probe frame, or the rotary moved)."""
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._log_line = lambda text: None
    p._session = type("S", (), {"ok": True, "reason": None})()
    p._rotary = dict(_measured(), homed_head="laser")
    p._homed_head = None
    p._start = lambda points, mode, config: dict(started=True, mode=mode)
    assert p._api_rotary_check({})["started"]                     # the check is allowed
    with pytest.raises(ValueError, match="not homed with the 10W"):
        p._api_rotary(dict(params={"seed": "last"}))                # a measure still is not
    still = dict(check_only=True, housing_shift=dict(dx=0.71, dz=-0.1, moved=False, z_differs=False))
    p._on_message(dict(type="done", mode="rotary", result=still))
    assert p._homed_head == "laser"
    p._homed_head = None
    off = dict(check_only=True, housing_shift=dict(dx=-12.8, dz=0.0, moved=True, z_differs=False))
    p._rotary = dict(_measured(), homed_head="laser")
    p._on_message(dict(type="done", mode="rotary", result=off))
    assert p._homed_head is None
    p._rotary = dict(_measured(), homed_head="probe")              # a probe-frame measurement proves nothing
    p._on_message(dict(type="done", mode="rotary", result=still))
    assert p._homed_head is None


def test_disarming_mid_job_stops_the_beam_at_once(module, tmp_path):
    """A disarm used to stop only new lines: the queued burn lines (and the firmware's last power)
    kept the beam on (2026-11-23, a band test). Disarming a busy bridge now drops the queue,
    quickstops and sends M5, as LightBurn's Stop does."""
    from lhp_pkg import grbl_bridge
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    sender = BridgeSender()
    p._grbl = grbl_bridge.Bridge(sender)
    p._rotary = rotary_result()
    p._settings.values["laser_modules"] = "10W = 5.75, 0, 0"
    p._settings.values["selected_module"] = "10W"
    p._printer.machine.security, p._printer.machine.roll = 0, 1
    p._api_bridge_mode(dict(mode="rotary"))
    p._api_bridge_laser(dict(laser=True))
    p._grbl.queue.append(("G1 X10 S200", None, 0.1, "burn"))
    assert p._grbl.state() == "Run"
    p._api_bridge_laser(dict(laser=False))
    assert not p._grbl.queue and "M410" in sender.urgent_lines and ("M5", False) in sender.lines
    assert p._grbl.context.laser_armed()[0] is False


def test_the_link_guard_fakes_a_lost_ok_and_its_stop_resets_the_bridge_with_the_beam_off(module, tmp_path):
    """Wired through the hooks: a burn line with no ok while temperature reports keep coming
    gets a faked ok; a second loss stops the burn (bridge reset, M5)."""
    from lhp_pkg import grbl_bridge
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._grbl = grbl_bridge.Bridge(BridgeSender())
    clock = [0.0]
    p._link_start()
    p._link_running = False                                   # step by hand
    p._link.clock = lambda: clock[0]
    p._link_power_off = lambda: None
    p.gcode_sent(None, "sent", "G1 X156.1 Y166.5 Z145.8 B-243.9 S46 F2219", None, "G1", tags=set())
    clock[0] = 1.0
    p.gcode_received(None, " T:0.00 /0.00 B:-14.06 /0.00")
    clock[0] = 2.5
    p._link.step()
    assert getattr(p._printer, "fake_acks", 0) == 1
    p.gcode_sent(None, "sent", "G1 X156.1 Y166.6 Z145.8 B-243.7 S46 F2219", None, "G1", tags=set())
    clock[0] = 3.5
    p.gcode_received(None, " T:0.00 /0.00 B:-14.06 /0.00")
    clock[0] = 5.0
    p._link.step()
    assert p._printer.fake_acks == 2 and "M5" in p._printer.sent_commands
    assert [k for _, k in p._link.events] == ["fake_ack", "stop"]
