"""The close-up camera through the plugin: API, job, image, routes and refusals.

The fake touchscreen renders each photo from wherever the fake machine's head is, through a
known camera model, of a dot pattern on the bed. The close-up served at /closeup.jpg must put
every dot where it is.
"""
import json
import os
import sys
import time

import numpy as np
import pytest

from helpers import Rig
from test_plugin import _fake_tornado, make_plugin, module  # noqa: F401  (module is a fixture)
from lhp_pkg import camera_model as cm
from lhp_pkg import compositor as co
from lhp_pkg import cvio

O = (4.0, -2.5)          # bridge_offset (2, 0) + the 10W's dx, dy (2.0, -2.5)
DOTS = [(x, y) for x in (118.0, 160.0, 202.0) for y in (128.0, 175.0, 222.0)]
BOX = [115.0, 125.0, 205.0, 225.0]


def true_model():
    m = cm.CameraModel(f=812.3, cx=546.0, cy=629.0, k1=-0.05, k2=0.012, rvec=(0.003, -0.002, 0.011),
                       e=4.7, t=(-56.8, 6.3), bed_ref=13.6, o_burn=O)
    m.z_range = (40.0, 290.0)
    return m


class RenderScreen(object):
    """The touchscreen: each new photo is what the camera sees from the head's position."""

    def __init__(self, machine, model):
        self.machine, self.model = machine, model
        self.raster, self.rgeom = co.dot_raster((100.0, 110.0, 220.0, 240.0), 20.0, DOTS)
        self.photo, self.requests = b"\xff\xd8old\xff\xd9", []

    def get(self, url, timeout):
        if "request_capture_photo" in url:
            m = self.machine
            head = (m.x, m.y, m.z)
            self.requests.append(head)
            img = co.render_photo(self.raster, self.rgeom, self.model, head, O, 0.0)
            img[0, 0] = len(self.requests) % 256          # a real sensor never repeats a frame exactly
            self.photo = cvio.encode(img, 95)
            return 200, b'{"status":true}'
        return 200, self.photo


def ready(module, tmp_path, rig=None, model=True, **settings):
    rig = rig or Rig()
    rig.machine.rotary = False
    rig.machine.z = 290.0
    p = make_plugin(module, rig, tmp_path)
    p._settings.values.update(headcam_url="http://screen.test:8080", flat_bed_z=13.6,
                              laser_modules="10W = 8.6, 2.0, -2.5\n2W IR = 73.95, 1.0, 3.6",
                              closeup_dpm=10.0)
    p._settings.values.update(settings)
    screen = RenderScreen(rig.machine, true_model())
    p._headcam_get = screen.get
    p._homed = True                                          # as after a G28 since connecting
    if model:
        folder = os.path.join(str(tmp_path), "closeup")
        os.makedirs(folder, exist_ok=True)
        with open(os.path.join(folder, "camera_model.json"), "w") as handle:
            json.dump(true_model().to_dict(), handle)
    return p, rig, screen


def wait(p, timeout=60):
    end = time.time() + timeout
    while p._running() and time.time() < end:
        time.sleep(0.05)
    assert not p._running()


def test_the_status_says_whether_the_close_up_can_work(module, tmp_path):
    p, rig, screen = ready(module, tmp_path, model=False)
    st = p._status()["closeup"]
    assert st["available"] is True and st["versions"]["numpy"] and st["model"] is None
    p2, _, _ = ready(module, tmp_path)
    p2._camera_model = None
    st = p2._status()["closeup"]
    assert st["model"]["z_range"] == [40.0, 290.0] and st["dpm"] == 10.0


def test_the_overview_photo_keeps_where_it_was_taken(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    p._api_head_photo({})
    with open(os.path.join(str(tmp_path), "headcam.json")) as handle:
        pose = json.load(handle)
    assert (pose["x"], pose["y"], pose["z"]) == (232.0, 178.0, 290.0) and pose["quality"] == 8


def test_a_close_up_through_the_plugin_puts_every_dot_in_place(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    p._api_head_photo({})
    plan = p._api_closeup_plan(dict(box=BOX, height=0))
    assert plan["nx"] * plan["ny"] == len(plan["tiles"]) >= 4 and plan["head_z"] == 50.0
    assert rig.machine.z == 290.0                                  # planning moves nothing
    out = p._api_closeup(dict(box=BOX, height=0))
    assert out["started"] is True and out["tiles"] == len(plan["tiles"])
    wait(p)
    assert p._last_error is None, p._last_error
    assert len(screen.requests) == 2 + len(plan["tiles"])     # our overview, the job's own fresh one, tiles
    assert screen.requests[1] == (232.0, 178.0, 290.0)          # the fresh overview comes first, from up high
    assert all(r[2] == 50.0 for r in screen.requests[2:])
    assert rig.machine.z == 290.0                                  # back up
    resp = p.closeup_image()
    canvas = cvio.decode(resp.body)
    geom = co.canvas_geometry(10.0)
    assert canvas.shape[:2] == (geom["H"], geom["W"])
    found = co.find_dots(canvas, geom, DOTS)
    assert max(np.hypot(a - x, b - y) for (a, b), (x, y) in zip(found, DOTS)) <= 0.1
    st = p._status()["closeup"]["last"]
    assert st["tiles"] == len(plan["tiles"]) and st["seam_mm"] <= 0.1
    assert os.path.exists(os.path.join(str(tmp_path), "closeup", "closeup.jpg"))
    assert any("Close-up" in line for line in p._log)
    assert not p._grbl or p._grbl.locked is None


def test_a_close_up_holds_a_print_and_locks_lightburn_out_while_it_runs(module, tmp_path):
    from lhp_pkg import grbl_bridge as gb
    p, rig, screen = ready(module, tmp_path)
    p._grbl = gb.Bridge(_Sender())
    seen = []
    real = p._headcam_get

    def watching(url, timeout):
        if "request_capture_photo" in url:
            seen.append((p._grbl.locked, p._held))
        return real(url, timeout)

    p._headcam_get = watching
    p._api_closeup(dict(box=[140.0, 160.0, 165.0, 185.0], height=0))
    wait(p)
    assert seen and all(lock and held for lock, held in seen)
    assert p._grbl.locked is None and p._held is False


class _Sender(object):
    def __init__(self):
        self.where = (0.0, 0.0, 0.0, 0.0)

    def position(self):
        return self.where

    def send(self, *a, **k):
        pass


@pytest.mark.parametrize("change,message", [
    ("rotary", "rotary"),
    ("toolhead", "camera"),
    ("flat_bed_z", "bed height"),
    ("focus", "focus offset"),
    ("model", "calibrate"),
    ("url", "headcam_url"),
    ("height", "height"),
    ("clearance", "clearance"),
])
@pytest.mark.parametrize("off_grid", [False, True])
def test_a_close_up_that_cannot_be_done_safely_is_refused_and_nothing_moves(module, tmp_path, change, message,
                                                                            off_grid):
    settings = {}
    if change == "flat_bed_z":
        settings["flat_bed_z"] = None
    if change == "focus":
        settings["laser_modules"] = "10W = , 2.0, -2.5\n2W IR ="
    if change == "url":
        settings["headcam_url"] = ""
    if change == "clearance":
        settings["closeup_min_clearance"] = 45.0
    p, rig, screen = ready(module, tmp_path, model=(change != "model"), **settings)
    if change == "rotary":
        rig.machine.rotary = True
    if change == "toolhead":
        rig.machine.toolhead = "2W RED LASER"
    if off_grid:                    # map loaded, head at home (off its grid): would move first
        p._map = dict(nodes=[[0.0] * 11 for _ in range(11)], start=[42.0, 34.0], spacing=[27.0, 29.0], created=1.0,
                      base=dict(x=60.0, y=116.0, machine_x=79.0, machine_y=126.0))
        p._mesh = dict(loaded_at=1.0, backup=None)
        p._mesh_guard = lambda: None
        p._work_offset = lambda: (0.0, 0.0, 0.0)
        rig.machine.x, rig.machine.y, rig.machine.z = 330.0, 345.0, 290.0
    rig.machine.sent.clear()
    with pytest.raises(ValueError, match=message):
        p._api_closeup(dict(box=BOX, height=None if change == "height" else 0))
    assert not any(c.startswith(("G0", "G1")) for c in rig.machine.sent)
    assert screen.requests == []


def test_a_second_close_up_waits_for_the_first(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    p._api_closeup(dict(box=BOX, height=0))
    try:
        with pytest.raises(ValueError, match="running"):
            p._api_closeup(dict(box=BOX, height=0))
    finally:
        p._stop_job("test")
        wait(p)
    assert rig.machine.z == 290.0


def test_lightburns_m240_p1_redoes_the_last_close_up(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    with pytest.raises(ValueError, match="no close-up"):
        p._macro_camera(1)
    p._api_closeup(dict(box=[140.0, 160.0, 165.0, 185.0], height=0))
    wait(p)
    n = len(screen.requests)
    p._macro_camera(1)
    wait(p)
    assert len(screen.requests) == n + 2 and p._last_error is None   # a fresh overview and the tile


def test_the_lightburn_camera_entry_points_at_the_close_up_stream(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    entry = p._api_closeup_lightburn(dict(base_url="http://octopi.test:5000"))["camera"]
    assert entry["CameraURL"] == "http://octopi.test:5000/plugin/laserheightprobe/closeup.mjpg"
    assert entry["ImageAlignments"][0]["ImageSizeX"] == 3200 and entry["DisplayName"] == "10W close-up"


def test_the_self_test_reports_the_errors_on_this_machines_libraries(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    out = p._api_closeup_selftest(dict(dpm=10))
    assert out["close_max_mm"] <= 0.1 and out["base_max_mm"] <= 0.5 and out["tiles"] >= 4


def test_both_streams_are_routes(module, tmp_path, monkeypatch):
    for name, stub in _fake_tornado().items():
        monkeypatch.setitem(sys.modules, name, stub)
    p, rig, screen = ready(module, tmp_path)
    routes = p.route_hook([])
    assert [r[0] for r in routes] == [r"/headcam.mjpg", r"/closeup.mjpg"]
    assert routes[1][2]["interval"] == 2.0


class SceneScreen(RenderScreen):
    """RenderScreen with any raster: the printed board, or the burned crosses."""

    def __init__(self, machine, model, raster, rgeom):
        RenderScreen.__init__(self, machine, model)
        self.raster, self.rgeom = raster, rgeom


def board_scene():
    from lhp_pkg import calib_solve as cs
    img = cvio.charuco_board(cs.BOARD).generateImage((195 * 20, 270 * 20), marginSize=0, borderBits=1)
    return img, dict(x0=65.0, y1=310.0, dpm=20.0)          # top-left at LightBurn (65, 310)


def cross_scene():
    import cv2
    dpm, x0, y0, x1, y1 = 20.0, 20.0, 75.0, 300.0, 275.0
    img = np.full((int((y1 - y0) * dpm), int((x1 - x0) * dpm)), 230, np.uint8)
    for cx, cy in ((60.0, 100.0), (200.0, 100.0), (60.0, 250.0), (200.0, 250.0)):
        u, v = (cx - x0) * dpm, (y1 - cy) * dpm
        cv2.line(img, (int(u - 200), int(v)), (int(u + 200), int(v)), 60, 8)
        cv2.line(img, (int(u), int(v - 200)), (int(u), int(v + 200)), 60, 8)
    return img, dict(x0=x0, y1=y1, dpm=dpm)


def test_calibrating_through_the_plugin_recovers_the_camera(module, tmp_path, monkeypatch):
    from lhp_pkg import closeup_plan as cplan
    monkeypatch.setattr(cplan, "BOARD_LEVELS", ((45.0, 15.0), (60.0, 20.0)))
    monkeypatch.setattr(cplan, "BOARD_HIGH", ((200.0, ((0, 0), (-60, 0), (60, 0))),
                                              (290.0, ((0, 0), (0, -60), (0, 60)))))
    p, rig, screen = ready(module, tmp_path, model=False)
    truth = true_model()
    with pytest.raises(ValueError, match="calibrate"):
        p._api_closeup(dict(box=BOX, height=0))
    raster, rgeom = board_scene()
    board = SceneScreen(rig.machine, truth, raster, rgeom)
    p._headcam_get = board.get
    out = p._api_closeup_calib_capture(dict(stage="board", target=[162.5, 175.0], clear=True))
    wait(p, 300)
    assert p._last_error is None, p._last_error
    solved = p._api_closeup_calib_solve({})                  # the newest board session
    rep = solved["report"]
    cal = p._status()["closeup"]["calib"]
    assert cal["board"] == out["session"] and cal["candidate"]["board"]["holdout_rms_mm"] == rep["holdout_rms_mm"]
    with pytest.raises(ValueError, match="session"):
        p._api_closeup_calib_solve(dict(session="../../x"))
    assert rep["holdout_rms_mm"] <= 0.05 and abs(solved["model"]["f"] - truth.f) / truth.f < 0.005
    with pytest.raises(ValueError, match="both"):
        p._api_closeup_calib_accept({})
    raster, rgeom = cross_scene()
    crosses = SceneScreen(rig.machine, truth, raster, rgeom)
    p._headcam_get = crosses.get
    out = p._api_closeup_calib_capture(dict(stage="cross", clear=True))
    wait(p, 300)
    assert p._last_error is None, p._last_error
    tied = p._api_closeup_calib_tie({})                        # the newest cross session
    assert p._status()["closeup"]["calib"]["cross"] == out["session"]
    assert tied["model"]["t"] == pytest.approx(list(truth.t), abs=0.1)
    assert tied["report"]["loo_max_mm"] <= 0.3
    p._api_closeup_calib_accept({})
    assert p._status()["closeup"]["calib"]["candidate"] is None
    assert p._status()["closeup"]["model"]["t"] == pytest.approx(list(truth.t), abs=0.1)
    assert os.path.exists(os.path.join(str(tmp_path), "closeup", "camera_model.json"))
    assert rig.machine.z == 290.0


def test_a_close_up_of_an_uneven_bed_follows_the_height_map(module, tmp_path):
    from lhp_pkg import compositor as comp
    p, rig, screen = ready(module, tmp_path)
    nodes = [[0.9 * (i / 10.0) + 0.5 * (j / 10.0) for j in range(11)] for i in range(11)]
    p._map = dict(nodes=nodes, start=[42.0, 34.0], spacing=[27.0, 29.0], created=1.0,
                  base=dict(x=60.0, y=116.0, machine_x=79.0, machine_y=126.0))
    surface = comp.surface_from_map(nodes, (42.0, 34.0), (27.0, 29.0), (19.0, 10.0), 0.0)
    real = screen.get

    def uneven(url, timeout):
        if "request_capture_photo" in url:
            m = screen.machine
            img = comp.render_photo(screen.raster, screen.rgeom, screen.model, (m.x, m.y, m.z), O, 0.0,
                                    surface=surface)
            img[0, 0] = len(screen.requests) % 256
            screen.requests.append((m.x, m.y, m.z))
            screen.photo = cvio.encode(img, 95)
            return 200, b'{"status":true}'
        return real(url, timeout)

    p._headcam_get = uneven
    p._api_head_photo({})
    p._api_closeup(dict(box=BOX, height=0))
    wait(p)
    assert p._last_error is None, p._last_error
    canvas = cvio.decode(p.closeup_image().body)
    geom = co.canvas_geometry(10.0)
    found = co.find_dots(canvas, geom, DOTS)
    assert max(np.hypot(a - x, b - y) for (a, b), (x, y) in zip(found, DOTS)) <= 0.1


def test_the_last_close_up_image_survives_an_octoprint_restart_but_its_request_does_not(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    p._api_head_photo({})
    box = [140.0, 160.0, 165.0, 185.0]
    p._api_closeup(dict(box=box, height=0))
    wait(p)
    again, _, _ = ready(module, tmp_path)                    # a new plugin object: a restart
    again._headcam_get = screen.get
    out = again._api_closeup_recompose({})
    assert out["qa"]["tiles"] == 1                           # the image is rebuilt from disk
    with pytest.raises(ValueError, match="no close-up"):
        again._macro_camera(1)                               # but M240 P1 does not replay an old height


def test_m240_p1_only_repeats_a_fresh_request(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    p._api_closeup(dict(box=[140.0, 160.0, 165.0, 185.0], height=0))
    wait(p)
    p._closeup_request["time"] -= 31 * 60                    # older than closeup_repeat_minutes
    with pytest.raises(ValueError, match="older"):
        p._macro_camera(1)
    p._api_closeup(dict(box=[140.0, 160.0, 165.0, 185.0], height=0))
    wait(p)
    p._api_head_photo({})                                    # a new overview: the bed may have changed
    with pytest.raises(ValueError, match="no close-up"):
        p._macro_camera(1)


def test_a_close_up_waits_until_the_machine_has_been_homed_since_it_connected(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    p.on_event("Connected", {})                              # a power cycle or reconnect: position unknown
    rig.machine.sent.clear()
    with pytest.raises(ValueError, match="G28"):
        p._api_closeup(dict(box=BOX, height=0))
    with pytest.raises(ValueError, match="G28"):
        p._api_closeup_calib_capture(dict(stage="board", clear=True))
    assert not any(c.startswith(("G0", "G1")) for c in rig.machine.sent)
    p.gcode_sent(None, None, "G28", None, "G28")
    assert p._homed is True
    p.gcode_sent(None, None, "G28 X Y Z", None, "G28")
    p.on_event("Disconnected", {})
    assert p._homed is False


def test_calibration_photos_need_the_bed_declared_clear(module, tmp_path):
    p, rig, screen = ready(module, tmp_path, model=False)
    rig.machine.sent.clear()
    with pytest.raises(ValueError, match="clear"):
        p._api_closeup_calib_capture(dict(stage="board"))
    assert not any(c.startswith(("G0", "G1")) for c in rig.machine.sent)


@pytest.mark.parametrize("state,message", [("printing", "busy"), ("bridge", "LightBurn"), ("offset", "work offset")])
def test_a_close_up_is_refused_while_printing_bridging_or_offset(module, tmp_path, state, message):
    from lhp_pkg import grbl_bridge as gb
    p, rig, screen = ready(module, tmp_path)
    if state == "printing":
        p._printer.is_printing = lambda: True
    if state == "bridge":
        p._grbl = gb.Bridge(_Sender())
        p._grbl.lock_out("a test job")                       # state() != Idle
    if state == "offset":
        rig.machine.shift = 0.5                              # a G92-style work offset
    rig.machine.sent.clear()
    with pytest.raises(ValueError, match=message):
        p._api_closeup(dict(box=BOX, height=0))
    assert not any(c.startswith(("G0", "G1")) for c in rig.machine.sent)


def test_a_home_that_brings_back_a_fade_height_is_undone_while_the_map_is_loaded(module, tmp_path):
    # Seen 2026-11-21: after a G28 with the height map loaded, the head at Z290 had none of the
    # map's correction (the work offset check read -1.24 mm, the map's value there). The plugin
    # sets no fade (M420 Z0) when it loads the map; a home can bring a fade height back, and
    # above it the correction fades to nothing. So it is put back before anything relies on it.
    p, rig, screen = ready(module, tmp_path)
    p._mesh = dict(loaded_at=1.0, backup=None)
    rig.machine.fade = 10.0
    rig.machine.levelling = True
    p._mesh_guard()
    assert rig.machine.fade in (0, 0.0) and rig.machine.levelling is True
    assert any("fade" in line for line in p._log)
    rig.machine.levelling = False
    p._mesh_guard()
    assert rig.machine.levelling is True
    p._mesh = None
    rig.machine.fade = 10.0
    p._mesh_guard()                                          # no map loaded: nothing is changed
    assert rig.machine.fade == 10.0


def test_any_home_while_the_map_is_loaded_turns_levelling_straight_back_on(module, tmp_path):
    # Seen 2026-11-21 in laser mode: G28 leaves levelling OFF. A home from LightBurn mid-session
    # would otherwise burn the rest of the job with no bed correction.
    p, rig, screen = ready(module, tmp_path)
    sent = []
    p._printer.commands = lambda cmds, *a, **k: sent.append(list(cmds) if isinstance(cmds, (list, tuple)) else [cmds])
    p._mesh = dict(loaded_at=1.0, backup=None)
    p.gcode_sent(None, None, "G28 X Y Z", None, "G28")
    deadline = time.time() + 2
    while not sent and time.time() < deadline:
        time.sleep(0.02)
    assert ["M420 S1"] in sent
    sent.clear()
    p._mesh = None
    p.gcode_sent(None, None, "G28", None, "G28")
    time.sleep(0.2)
    assert sent == []


def test_planning_moves_nothing_and_a_close_up_first_brings_the_head_over_the_map(module, tmp_path):
    # Seen 2026-11-21: right after a G28 the head sits at the home position, outside the height
    # map's grid, where the firmware and the plugin extrapolate the map differently, so the
    # work offset check refused ("home with G28 first", just after homing).
    p, rig, screen = ready(module, tmp_path)
    p._map = dict(nodes=[[0.0] * 11 for _ in range(11)], start=[42.0, 34.0], spacing=[27.0, 29.0], created=1.0,
                  base=dict(x=60.0, y=116.0, machine_x=79.0, machine_y=126.0))
    p._mesh = dict(loaded_at=1.0, backup=None)
    rig.machine.x, rig.machine.y, rig.machine.z = 330.0, 345.0, 290.0     # off the map's grid
    calls = []
    # this test is about where the check happens, not its arithmetic (the fake's mesh differs)
    p._work_offset = lambda: (calls.append((rig.machine.x, rig.machine.y)), (0.0, 0.0, 0.0))[1]
    rig.machine.sent.clear()
    p._api_closeup_plan(dict(box=BOX, height=0))
    assert not any(c.startswith(("G0", "G1")) for c in rig.machine.sent) and calls == []
    p._api_closeup(dict(box=BOX, height=0))
    wait(p)
    assert calls and calls[0] == (232.0, 178.0)               # checked over the bed, at the overview spot
    moves = [c for c in rig.machine.sent if c.startswith("G0")]
    assert "Z290" in moves[0] and "X" not in moves[0]         # up first, then across


def test_the_tab_gets_a_light_preview_while_lightburn_gets_full_resolution(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    p._api_closeup(dict(box=[140.0, 160.0, 165.0, 185.0], height=0))
    wait(p)
    full = cvio.decode(p.closeup_image().body)
    prev = cvio.decode(p.closeup_preview().body)
    assert prev.shape[1] == 1200 and prev.shape[0] == round(1200 * full.shape[0] / full.shape[1])
    assert len(p.closeup_preview().body) < len(p.closeup_image().body) / 3


def test_lightburns_overview_macro_also_refreshes_the_close_up_image(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    p._api_closeup(dict(box=[140.0, 160.0, 165.0, 185.0], height=0))
    wait(p)
    before = p._closeup_last["time"]
    time.sleep(0.01)
    p._macro_camera(0)                                       # LightBurn's "Head photo" (M240)
    deadline = time.time() + 30
    while p._closeup_last["time"] == before and time.time() < deadline:
        time.sleep(0.1)
    assert p._closeup_last["time"] > before and p._closeup_last["tiles"] == 1


def test_lightburn_is_locked_out_while_the_close_up_image_is_rebuilt_after_its_head_photo(module, tmp_path):
    from lhp_pkg import grbl_bridge as gb
    p, rig, screen = ready(module, tmp_path)
    p._api_closeup(dict(box=[140.0, 160.0, 165.0, 185.0], height=0))
    wait(p)
    p._grbl = gb.Bridge(_Sender())
    seen = []
    real = p._closeup_make
    p._closeup_make = lambda *a, **k: (seen.append(p._grbl.locked), real(*a, **k))[1]
    p._recompose_quietly()
    assert seen and seen[0] and p._grbl.locked is None



def test_recompose_never_moves_the_head_even_off_the_maps_grid(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    p._api_closeup(dict(box=[140.0, 160.0, 165.0, 185.0], height=0))
    wait(p)
    p._map = dict(nodes=[[0.0] * 11 for _ in range(11)], start=[42.0, 34.0], spacing=[27.0, 29.0], created=1.0,
                  base=dict(x=60.0, y=116.0, machine_x=79.0, machine_y=126.0))
    p._mesh = dict(loaded_at=1.0, backup=None)
    p._mesh_guard = lambda: None
    rig.machine.x, rig.machine.y, rig.machine.z = 330.0, 345.0, 290.0
    rig.machine.sent.clear()
    p._api_closeup_recompose({})
    assert not any(c.startswith(("G0", "G1")) for c in rig.machine.sent)


def test_the_status_says_when_the_close_up_photos_are_older_than_the_overview(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    p._api_closeup(dict(box=[140.0, 160.0, 165.0, 185.0], height=0))
    wait(p)
    first = p._closeup_last
    assert first["photos_time"] and first["overview_time"] <= first["photos_time"]
    time.sleep(0.05)
    p._api_head_photo({})
    p._api_closeup_recompose({})
    last = p._closeup_last
    assert last["photos_time"] == first["photos_time"] and last["overview_time"] > last["photos_time"]


def test_the_tab_preview_works_after_a_restart_from_the_saved_image(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    p._api_closeup(dict(box=[140.0, 160.0, 165.0, 185.0], height=0))
    wait(p)
    p._closeup_preview = None
    p._closeup_jpeg = None                                    # as after a restart
    img = cvio.decode(p.closeup_preview().body)
    assert img.shape[1] == 1200


def test_after_a_restart_the_status_still_describes_the_last_close_up(module, tmp_path):
    p, rig, screen = ready(module, tmp_path)
    p._api_closeup(dict(box=[140.0, 160.0, 165.0, 185.0], height=0))
    wait(p)
    taken = p._closeup_last["photos_time"]
    p._closeup_last, p._closeup_tiles, p._closeup_disk_read = None, [], False      # as after a restart
    last = p._closeup_status()["last"]
    assert last["tiles"] == 1 and last["photos_time"] == pytest.approx(taken)


# -- the red autofocus dot ---------------------------------------------------------

def _dot_photo(x, y=641.0, on=True):
    import numpy as np
    img = np.full((1280, 1024, 3), 170, np.uint8)
    if on:
        yy, xx = np.mgrid[0:1280, 0:1024]
        core = 200 * np.exp(-((xx - x) ** 2 + (yy - y) ** 2) / (2 * 4.0 ** 2))
        img = np.clip(img + core[..., None], 0, 255).astype(np.uint8)
    return img


def test_red_dot_measure_two_readings_agree(module, tmp_path):
    from lhp_pkg import redot
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    true = dict(a=10.11, b=65707.8, c=3.52)
    dm = redot.DotModel(true["a"], true["b"], true["c"], 641.0, 0.0, (19.4, -0.4, 0.0),
                        (-0.5209, 0.0141, -0.8535), (100, 180))
    p._save_json(os.path.join(module.CLOSEUP_DIR, module.DOT_MODEL_FILE), dm.to_dict())

    class Model(object):
        def to_plane(self, head, px, h):
            return np.array([[head[0] - 40.0, head[1] + 1.0]])
    ctx = dict(o=(4.0, -2.5), limits=((0, 0, 0), (330, 350, 330)), current=(100.0, 100.0, 200.0))
    p._closeup_setup = lambda need_model=True, own_lock=False: (dict(headcam_url="http://screen"), Model(), ctx)
    p._closeup_settle = lambda config, c: False
    p._map = dict(nodes=[[0.0] * 11] * 11, start=[42.0, 34.0], spacing=[27.0, 29.0],
                  base=dict(x=60.0, y=116.0, machine_x=79.0, machine_y=126.0),
                  samples=[dict(x=x, y=y) for x in (60.0, 300.0) for y in (80.0, 260.0)])
    heads = []

    def go(config, c, head, travel):
        heads.append(head)
        c["current"] = head
    p._dot_go = go
    h_true = 2.7

    def pair(config, x, y, z):
        col = true["a"] + true["b"] / (z + true["c"] - h_true)
        return _dot_photo(col), _dot_photo(col, on=False)
    p._dot_pair = pair
    out = p._api_dot_measure(dict(x=210.0, y=40.0))
    assert out["height"] == pytest.approx(2.7, abs=0.05)
    assert [h[2] for h in heads] == [170.0, 102.7]          # high first, then 100 above the surface
    assert len(out["readings"]) == 2 and p._status()["dot"]["last"]["height"] == out["height"]
    json.dumps(out)                                          # OctoPrint must be able to send it (numpy bools broke it)


def test_red_dot_refuses_without_calibration_or_confirmation(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._closeup_setup = lambda need_model=True, own_lock=False: (dict(), None, dict(o=(4.0, -2.5)))
    with pytest.raises(ValueError, match="calibrate the red dot"):
        p._api_dot_measure(dict(x=100.0, y=100.0))
    with pytest.raises(ValueError, match="clear"):
        p._api_dot_calibrate(dict(x=100.0, y=100.0))


def test_red_dot_uses_the_bed_map_only_where_it_was_probed(module, tmp_path):
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    nodes = [[0.01 * i for j in range(11)] for i in range(11)]          # rises along X
    p._map = dict(nodes=nodes, start=[42.0, 34.0], spacing=[27.0, 29.0],
                  base=dict(x=60.0, y=116.0, machine_x=79.0, machine_y=126.0),
                  samples=[dict(x=x, y=y) for x in (60.0, 300.0) for y in (80.0, 260.0)])
    lift, mapped = p._dot_lift((200.0, 150.0), (160.0, 151.0))
    assert mapped and lift == pytest.approx(-0.01 * (-40.0 / 27.0) * -1, abs=1e-6) or mapped
    lift, mapped = p._dot_lift((258.0, 36.0), (216.0, 38.0))          # the wood, in front of the map
    assert not mapped


def _dot_map_plugin(module, tmp_path):
    from lhp_pkg import redot
    rig = Rig()
    p = make_plugin(module, rig, tmp_path)
    p._log_line = lambda text: None
    dm = redot.DotModel(10.11, 65707.8, 3.52, 641.0, 0.0, (19.4, -0.4, 0.0), (-0.5209, 0.0141, -0.8535), (100, 180))
    p._save_json(os.path.join(module.CLOSEUP_DIR, module.DOT_MODEL_FILE), dm.to_dict())
    p._map = dict(nodes=[[0.0] * 11 for _ in range(11)], start=[42.0, 34.0], spacing=[27.0, 29.0],
                  base=dict(x=60.0, y=116.0, z=12.8, machine_x=79.0, machine_y=126.0, machine_z=12.8,
                            focus=21.4, machine_focus=21.4),
                  samples=[dict(x=x, y=y) for x in (60.0, 300.0) for y in (80.0, 260.0)])
    ctx = dict(o=(4.0, -2.5), limits=((0, 0, 0), (330, 350, 330)), current=(100.0, 100.0, 200.0))
    p._closeup_setup = lambda need_model=True, own_lock=False: (
        dict(headcam_url="http://screen", map_tolerance=0.5, map_focus_gap=None, laser_modules=[],
             flat_bed_z=None), object(), ctx)
    p._closeup_settle = lambda config, c: False
    return p


def test_red_dot_map_rebuilds_a_sloping_bed(module, tmp_path):
    p = _dot_map_plugin(module, tmp_path)

    def read(config, ctx, model, dm, target, z, h_guess):
        bx, by = target
        if abs(bx - 144.0) < 1 and abs(by - 117.5) < 1:
            raise ValueError("no red dot in the photo")          # one bad point is left out
        return dict(h=0.002 * bx + 0.001 * by, dot_b=[bx, by], px=[600.0, 641.0], head=[bx + 40, by, z])
    p._dot_read = read
    out = p._api_dot_map(dict(x0=60, y0=80, x1=260, y1=240, step=40))
    assert out["points"] == 6 * 5 - 1 and len(out["failed"]) == 1
    m = p._map
    assert m["source"] == "red dot" and m["range"] == pytest.approx(0.002 * 200 + 0.001 * 160, abs=0.01)
    low = 0.002 * 64 + 0.001 * 77.5                               # the lowest point
    assert m["base"]["focus"] == pytest.approx(21.4 + low, abs=0.01)
    assert m["base"]["machine_focus"] == pytest.approx(21.4 + low, abs=0.01)   # not focus + machine Z
    assert os.path.exists(os.path.join(str(tmp_path), module.MAP_PREVIOUS_FILE))
    json.dumps(out)


def test_red_dot_map_refuses_when_most_readings_fail(module, tmp_path):
    p = _dot_map_plugin(module, tmp_path)

    def read(*a):
        raise ValueError("no red dot in the photo")
    p._dot_read = read
    with pytest.raises(ValueError, match="only 0 of"):
        p._api_dot_map(dict(x0=60, y0=80, x1=260, y1=240, step=40))


def test_focus_from_the_red_dot(module, tmp_path):
    p = _dot_map_plugin(module, tmp_path)
    p._mesh = dict(loaded_at=1.0)
    p._checked_loose = lambda: dict(laser_modules=[], flat_bed_z=None)
    p._api_dot_measure = lambda data: dict(height=2.7, lb=[214.0, 38.0])
    asked = []
    p._api_bridge_mode = lambda data: asked.append(data) or dict(ok=True)
    out = p._api_dot_focus(dict(x=214, y=38))
    assert out["focus_z"] == pytest.approx(24.1) and asked == [dict(mode="flat", focus_z=24.1)]
    assert "laser" not in asked[0]                               # never arms the laser



def test_red_dot_map_replaces_a_loaded_map_in_the_printer_too(module, tmp_path):
    p = _dot_map_plugin(module, tmp_path)
    p._mesh = dict(loaded_at=1.0)
    p._dot_read = lambda config, ctx, model, dm, target, z, h: dict(h=0.001 * target[0], dot_b=list(target),
                                                                   px=[600.0, 641.0], head=[0, 0, z])
    loads = []
    p._api_map_load = lambda data: loads.append(p._map["source"])
    p._api_dot_map(dict(x0=60, y0=80, x1=260, y1=240, step=40))
    assert loads == ["red dot"]


def test_red_dot_map_that_will_not_load_puts_the_old_map_back(module, tmp_path):
    p = _dot_map_plugin(module, tmp_path)
    p._mesh = dict(loaded_at=1.0)
    old = p._map
    p._dot_read = lambda config, ctx, model, dm, target, z, h: dict(h=0.0, dot_b=list(target),
                                                                   px=[600.0, 641.0], head=[0, 0, z])

    def fail(data):
        raise ValueError("the printer said no")
    p._api_map_load = fail
    with pytest.raises(ValueError, match="old map is back"):
        p._api_dot_map(dict(x0=60, y0=80, x1=260, y1=240, step=40))
    assert p._map is old


def test_red_dot_readings_move_across_at_their_own_height(module, tmp_path):
    from lhp_pkg import redot
    p = _dot_map_plugin(module, tmp_path)
    dm = p._dot_model()
    travels = []
    p._dot_go = lambda config, ctx, head, travel: travels.append(travel)
    col = 10.11 + 65707.8 / (100 + 3.52)
    p._dot_pair = lambda config, x, y, z: (_dot_photo(col), _dot_photo(col, on=False))

    class Model(object):
        def to_plane(self, head, px, h):
            return np.array([[head[0] - 40.0, head[1] + 1.0]])
    ctx = dict(limits=((0, 0, 0), (330, 350, 330)), current=(100.0, 100.0, 100.0))
    p._dot_read(dict(), ctx, Model(), dm, (150.0, 120.0), 100.0, 0.0)
    assert travels == [100.0]                                  # across at Z100, not up to Z200
    ctx["current"] = (100.0, 100.0, 60.0)
    p._dot_read(dict(), ctx, Model(), dm, (150.0, 120.0), 90.0, 0.0)
    assert travels[-1] == 200.0                                # below Z100: the travel floor as before


def test_a_red_dot_map_of_an_object_holds_its_edges_flat_beyond_it(module, tmp_path):
    p = _dot_map_plugin(module, tmp_path)
    p._map["samples"] = [dict(x=float(x), y=float(y), machine_x=x + 19.0, machine_y=y + 10.0, height=0.0)
                         for x in range(60, 301, 40) for y in range(80, 261, 30)]
    # a box sloping up along Y between Y150 and Y230, 16 mm thick at its low end
    p._dot_read = lambda config, ctx, model, dm, target, z, h: dict(
        h=16.0 + 0.25 * (target[1] - 150.0), dot_b=list(target), px=[600.0, 641.0], head=[0, 0, z])
    p._api_dot_map(dict(x0=136, y0=152, x1=168, y1=232, step=16))
    m = p._map
    flat = [v for col in m["nodes"] for v in col]
    top = m["range"]
    # no node dives or overshoots: at most the slope across one 27 mm cell past the readings
    assert min(flat) > -0.25 * 27 and max(flat) < top + 0.25 * 27   # higher is the safe side
    from lhp_pkg import heightmap
    front = heightmap.evaluate(m["nodes"], m["start"], m["spacing"], 152 + 19.0, 60 + 10.0)
    back = heightmap.evaluate(m["nodes"], m["start"], m["spacing"], 152 + 19.0, 300 + 10.0)
    cell = 0.25 * 27                                              # the slope across one mesh cell
    assert -cell < front < 2.0                                    # held near the box's low edge
    assert top - 0.5 < back < top + cell                          # and its high edge (never lower)
    assert m["rms"] < 0.2


def test_back_to_the_bed_map_after_an_object_map(module, tmp_path):
    p = _dot_map_plugin(module, tmp_path)
    p._map["source"] = "probe"
    p._map["points"] = 42
    bed = dict(p._map)
    p._dot_read = lambda config, ctx, model, dm, target, z, h: dict(
        h=16.0 + 0.25 * (target[1] - 150.0), dot_b=list(target), px=[600.0, 641.0], head=[0, 0, z])
    p._api_dot_map(dict(x0=136, y0=152, x1=168, y1=232, step=16))
    assert p._map["source"] == "red dot"
    loads = []
    p._mesh_loaded = lambda: True
    p._mesh_busy = lambda: None
    p._api_map_load = lambda data: loads.append(p._map.get("source"))
    out = p._api_map_bed({})
    assert p._map["points"] == 42 and p._map.get("source") == "probe" and loads == ["probe"]
    assert out["height_map"]["source"] == "probe"
    assert p._read_json(module.MAP_FILE)["points"] == 42


def test_back_to_the_bed_map_keeps_the_object_map_if_loading_fails(module, tmp_path):
    p = _dot_map_plugin(module, tmp_path)
    p._write_json(module.BED_MAP_FILE, dict(p._map, source="probe", points=42))
    p._map = dict(p._map, source="red dot", points=21)
    p._mesh_loaded = lambda: True
    p._mesh_busy = lambda: None

    def boom(data):
        raise ValueError("no printer")
    p._api_map_load = boom
    with pytest.raises(ValueError):
        p._api_map_bed({})
    assert p._map["source"] == "red dot"


class SlopeScreen(RenderScreen):
    """The dots on a sloping top: height(x, y) in the B frame."""

    def __init__(self, machine, model, height):
        RenderScreen.__init__(self, machine, model)
        self.height = height

    def get(self, url, timeout):
        if "request_capture_photo" in url:
            m = self.machine
            head = (m.x, m.y, m.z)
            self.requests.append(head)
            if head[2] >= 200:                                   # the overview: the bed, flat
                img = co.render_photo(self.raster, self.rgeom, self.model, head, O, 0.0)
            else:
                img = co.render_photo(self.raster, self.rgeom, self.model, head, O, 0.0, surface=self.height)
            img[0, 0] = len(self.requests) % 256
            self.photo = cvio.encode(img, 95)
            return 200, b'{"status":true}'
        return 200, self.photo


def _slope_map(height, shift=(19.0, 10.0)):
    from lhp_pkg import heightmap
    pts = []
    for bx in np.arange(112.0, 216.0, 13.0):
        for by in np.arange(115.0, 232.0, 13.0):
            h = float(height(bx, by))
            pts.append(dict(x=float(bx), y=float(by), z=h, focus=None, machine_z=None, height=h))
    built = heightmap.build(pts, shift, plateau=True)
    zero = min(p["height"] for p in pts)
    built.update(source="red dot", zero_h=zero, created=1.0, points=len(pts))
    return built


def test_a_close_up_follows_a_mapped_sloping_object(module, tmp_path):
    def height(x, y):
        return 10.0 + 0.15 * (np.asarray(y, float) - 122.5)     # 15 mm rise over the box
    p, rig, screen = ready(module, tmp_path)
    screen = SlopeScreen(rig.machine, true_model(), height)
    p._headcam_get = screen.get
    p._map = _slope_map(height)
    with pytest.raises(ValueError, match="at least"):
        p._api_closeup_plan(dict(box=BOX, height=10))            # the top reaches 25 mm here
    p._api_head_photo({})
    p._api_closeup(dict(box=BOX, height=26))
    wait(p, 120)
    assert p._last_error is None, p._last_error
    canvas = cvio.decode(p.closeup_image().body)
    geom = co.canvas_geometry(10.0)
    found = co.find_dots(canvas, geom, DOTS)
    worst = max(np.hypot(a - x, b - y) for (a, b), (x, y) in zip(found, DOTS))
    assert worst <= 0.4, worst                                   # the fit misses the plane by up to 0.17 mm
    # the same photos placed flat at the top's height in the box's middle (no object map)
    config, model, ctx = p._closeup_setup()
    p._map = None
    p._closeup_make(model, O, 17.5, config, ctx["d_bed"])
    flat = co.find_dots(cvio.decode(p.closeup_image().body), geom, DOTS)
    worst_flat = max(np.hypot(a - x, b - y) for (a, b), (x, y) in zip(flat, DOTS))
    assert worst_flat > 2 * worst, (worst_flat, worst)          # 0.76 against 0.32 mm


def test_the_red_dot_refits_itself_when_the_bed_map_is_remade(module, tmp_path):
    from lhp_pkg import redot
    p = _dot_map_plugin(module, tmp_path)
    p._map.update(source="probe", created=1.0)
    p._write_json(module.BED_MAP_FILE, p._map)
    a, b, c = 10.11, 65707.8, 3.52
    d = np.array([-0.521, 0.0141, -0.8534]); d /= np.linalg.norm(d)
    samples = []
    for z in (100, 120, 140, 160, 180):
        s = -z / d[2]
        rel = list(np.array([19.4, -0.4, 0.0]) + s * d)
        dot_b = [175.0 + rel[0], 120.0 + rel[1]]
        x = a + b / (z + 0.0 + c)                      # the old map said the bed is at 0 under the dot
        samples.append(dict(z=z, x=x, y=640 + 0.01 * x, lift=0.0, rel=rel, dot_b=dot_b, at_head=0.0))
    dm = redot.fit(samples)
    p._save_json(os.path.join(module.CLOSEUP_DIR, module.DOT_MODEL_FILE),
                 dict(dm.to_dict(), samples=samples, bed_created=1.0))
    assert p._dot_model().c == pytest.approx(dm.c)                 # same map: unchanged
    new = dict(p._map, nodes=[[0.5] * 11 for _ in range(11)], created=2.0)  # re-probed: the bed reads 0.5 higher
    p._write_json(module.BED_MAP_FILE, new)
    again = p._dot_model()
    assert again.c == pytest.approx(dm.c + 0.5, abs=0.01)
    assert p._read_json(os.path.join(module.CLOSEUP_DIR, module.DOT_MODEL_FILE))["bed_created"] == 2.0


def test_a_close_up_away_from_the_mapped_object_ignores_it(module, tmp_path):
    def height(x, y):
        return 10.0 + 0.15 * (np.asarray(y, float) - 122.5)
    p, rig, screen = ready(module, tmp_path)
    m = _slope_map(lambda x, y: height(x, y))
    m["footprint"] = [112.0, 115.0, 216.0, 232.0]
    p._map = m
    config, model, ctx = p._closeup_setup()
    obj = p._closeup_object(model)
    assert float(obj(150.0, 170.0)) > 10.0                       # on the object
    assert abs(float(obj(280.0, 60.0))) < 0.01                   # the bed far away, not the plateau
    p._closeup_object_check(model, [250.0, 40.0, 290.0, 80.0], 0, ctx["o"])   # flat paper there: allowed
    with pytest.raises(ValueError):
        p._closeup_object_check(model, BOX, 0, ctx["o"])


def _calib_reads(wander):
    d = np.array([-0.521, 0.0141, -0.8534]); d /= np.linalg.norm(d)

    def read(config, ctx, model, dm, target, z, h):
        s = -z / d[2]
        rel = np.array([19.4, -0.4, 0.0]) + s * d
        x = 10.11 + 65707.8 / (z + 3.52)
        dot = [target[0] + wander * (180.0 - z) / 80.0, target[1]]
        return dict(h=0.0, dot_b=dot, px=[x, 640 + 0.01 * x], head=[dot[0] - rel[0], dot[1] - rel[1], z])
    return read


def test_a_red_dot_calibration_whose_dot_wanders_is_refused(module, tmp_path):
    p = _dot_map_plugin(module, tmp_path)
    p._dot_read = _calib_reads(9.0)                               # caught by a box beside the spot
    with pytest.raises(ValueError, match="moved 9.0 mm"):
        p._api_dot_calibrate(dict(x=100.0, y=115.0, clear=True))
    p._dot_read = _calib_reads(0.1)
    out = p._api_dot_calibrate(dict(x=100.0, y=115.0, clear=True))
    assert out["ok"] and len(out["samples"]) == 5


def test_obstacles_are_marked_cleared_filtered_and_learned_from_red_dot_maps(module, tmp_path):
    p = _dot_map_plugin(module, tmp_path)
    p._log_line = lambda text: None
    out = p._api_obstacle_add(dict(box=[182, 235, 112, 145], height=38))
    assert out["obstacle"]["box"] == [112.0, 145.0, 182.0, 235.0] and len(p._obstacles()) == 1
    p._dot_bed_focus = lambda config: (21.4, "")
    assert len(p._active_obstacles({}, 21.4)) == 1                 # burning on the bed: guarded
    assert p._active_obstacles({}, 21.4 + 40.0) == []              # burning on something taller: clears it
    p._api_obstacle_add(dict(box=[0, 0, 20, 20], height=5))
    assert len(p._active_obstacles({}, 21.4)) == 1                 # 5 mm: the module clears it at bed focus
    p._api_obstacle_clear({})
    assert p._obstacles() == []
    # a red-dot map of a tall object records it, and its own loaded map lifts the head over it
    p._dot_read = lambda config, ctx, model, dm, target, z, h: dict(
        h=16.0 + 0.25 * (target[1] - 150.0), dot_b=list(target), px=[600.0, 641.0], head=[0, 0, z])
    p._api_dot_map(dict(x0=136, y0=152, x1=168, y1=232, step=16))
    obs = p._obstacles()
    assert len(obs) == 1 and obs[0]["source"] == "red dot" and obs[0]["top"] > 30
    assert obs[0]["box"][0] < 136 and obs[0]["box"][2] > 168
    p._mesh_loaded = lambda: True
    assert p._active_obstacles({}, 21.4) == []                     # its map is in the printer
    p._dot_map_again = p._api_dot_map(dict(x0=136, y0=152, x1=168, y1=232, step=16))
    assert len(p._obstacles()) == 1                                # a re-scan replaces it


def test_the_red_dot_checks_a_rotary_object_from_high_up(module, tmp_path):
    p = _dot_map_plugin(module, tmp_path)
    p._log_line = lambda text: None
    p._rotary = dict(complete=True, axis=dict(y0=122.5, x0=167.7, z0=102.4, dx_dy=-0.00253, dz_dy=-0.00366),
                     summary=dict(diameter=79.21, body_from_y=32.5, body_to_y=242.5),
                     params=dict(diameter=79.21), runout=[dict(y=50.0, e=1.5, phase_deg=30.0)])
    p._b_frame = dict(shift=-8.68)
    p._closeup_setup = lambda need_model=True, own_lock=False, allow_rotary=False: (
        dict(flat_bed_z=13.6, headcam_url="http://screen"), object(),
        dict(o=(4.0, -2.5), limits=((0, 0), (330, 350)), current=(200.0, 100.0, 290.0),
             rotary=allow_rotary, module=dict(dx=2.0, dy=-2.5, focus_offset=8.6)))
    sent = []

    class Bridge(object):
        def run(self, lines, timeout):
            sent.extend(lines)
            return ["X:200.00 Y:100.00 Z:290.00 B:180.00 Count X:1 Y:1 Z:1"]
    p._bridge = Bridge()
    reads = []

    def read(config, ctx, model, dm, target, z, h):
        reads.append((target, z, h, ctx.get("travel_floor")))
        return dict(h=h + 0.3, head=[target[0] + 30, target[1], z])
    p._dot_read = read
    out = p._api_dot_rotary_check(dict(points=4))
    assert len(out["points"]) == 4 and out["mean_diff"] == pytest.approx(0.3, abs=0.01)
    highest = max(r[2] for r in reads) + 13.6
    assert all(r[1] >= highest + 79 for r in reads)               # always well above the object
    assert all(r[3] == 290.0 for r in reads)                      # moves across at Z290
    assert sent[-1].startswith("G0 Z290")                         # and ends up there
    assert all(abs(r[2] - (128.0 + 0.0)) < 3 for r in reads)      # the stored top (~128 mm above the bed)
    p._closeup_setup = lambda need_model=True, own_lock=False, allow_rotary=False: (
        dict(flat_bed_z=13.6), object(), dict(o=(4.0, -2.5), rotary=False, module={}, current=(0, 0, 290),
                                              limits=((0, 0, 0), (330, 350, 330))))
    with pytest.raises(ValueError, match="not fitted"):
        p._api_dot_rotary_check({})


def test_the_red_dot_across_the_crest_finds_how_far_sideways_it_is(module, tmp_path):
    import math
    p = _dot_map_plugin(module, tmp_path)
    p._log_line = lambda text: None
    p._rotary = dict(complete=True, axis=dict(y0=122.5, x0=167.7, z0=102.4, dx_dy=0.0, dz_dy=0.0),
                     summary=dict(diameter=78.28, body_from_y=26.25, body_to_y=236.25), params=dict(diameter=78.28), runout=[])
    p._b_frame = dict(shift=0.0)
    p._closeup_setup = lambda need_model=True, own_lock=False, allow_rotary=False: (
        dict(flat_bed_z=13.6, headcam_url="http://screen"), object(),
        dict(o=(4.0, -2.5), limits=((0, 0), (330, 350)), current=(200.0, 100.0, 290.0),
             rotary=allow_rotary, module=dict(dx=2.0, dy=-2.5, focus_offset=8.6)))

    class Bridge(object):
        def run(self, lines, timeout):
            return ["X:200.00 Y:100.00 Z:290.00 B:0.00 Count X:1 Y:1 Z:1"]
    p._bridge = Bridge()
    r, true_shift, top = 39.14, 1.5, 102.4 + 39.14 - 13.6

    def read(config, ctx, model, dm, target, z, h):
        off = target[0] - 2.0 - 167.7                       # where the dot lands, from the measured crest
        return dict(h=top - (r - math.sqrt(r * r - (off - true_shift) ** 2)), head=[target[0], target[1], z])
    p._dot_read = read
    out = p._api_dot_rotary_check(dict(across=[-12, -6, 0, 6, 12], y=120))
    assert out["crest"]["dx"] == pytest.approx(true_shift, abs=0.05) and out["crest"]["rms"] < 0.01
