"""The fixed USB bed camera as the close-up base, through the plugin (closeup_base = "usb").

The fake bed camera renders what it sees with the bed wherever the fake machine has it: only
with the head at the park Y (300) does its photo line up with the calibration.
"""
import json
import os

import numpy as np
import pytest

from helpers import Rig
from test_bedcam import H_LB, K, DIST, cross_raster, render, true_model as bed_truth
from test_closeup_plugin import BOX, DOTS, ready, wait
from test_plugin import make_plugin, module  # noqa: F401  (module is a fixture)
from lhp_pkg import bedcam as bc
from lhp_pkg import compositor as co
from lhp_pkg import cvio
from lhp_pkg.settings import SettingsError, routine_config

PARK = (0.0, 300.0, 200.0)


class FakeBedcam(object):
    """go2rtc's frame.jpeg: the bed as the camera sees it now, the machine's pose recorded."""

    def __init__(self, machine, raster, rgeom, model=None):
        self.machine, self.raster, self.rgeom = machine, raster, rgeom
        self.model = model or bed_truth()
        self.grabs = []

    def get(self, url, timeout):
        m = self.machine
        self.grabs.append((url, (m.x, m.y, m.z)))
        # the bed moves with Y: at a lower Y it sits further back under the camera
        photo = render(self.model, self.raster, self.rgeom, bed_shift=300.0 - m.y)
        return 200, cvio.encode(photo, 92)


def usb_ready(module, tmp_path, raster="crosses", calibrated=True, **settings):
    settings.setdefault("closeup_base", "usb")
    settings.setdefault("bedcam_url", "http://bedcam.test:1984/api/frame.jpeg?src=bed_cam")
    p, rig, screen = ready(module, tmp_path, **settings)
    rig.machine.x, rig.machine.y = 150.0, 120.0
    if raster == "crosses":
        img, rgeom = cross_raster()
    else:                                        # the close-up's dot pattern, as the touchscreen renders it
        img, rgeom = screen.raster, screen.rgeom
    cam = FakeBedcam(rig.machine, img, rgeom)
    p._bedcam_get = cam.get
    p._bedcam_settle_s = 0.0
    if calibrated:
        with open(os.path.join(str(tmp_path), "closeup", "bedcam.json"), "w") as handle:
            json.dump(bed_truth().to_dict(), handle)
    return p, rig, screen, cam


def moves(rig):
    return [c for c in rig.machine.sent if c.startswith(("G0", "G1")) or c.split()[0] == "M5"]


def test_the_usb_overview_parks_before_the_snapshot_and_comes_back(module, tmp_path):
    p, rig, screen, cam = usb_ready(module, tmp_path)
    rig.machine.sent.clear()
    out = p._api_head_photo({})
    assert out["source"] == "usb"
    assert [pose for _, pose in cam.grabs] == [PARK]           # one photo, with the head parked
    assert screen.requests == []                               # the touchscreen is not asked
    sent = moves(rig)
    assert sent[0] == "M5"                                      # beam off first
    first_xy = next(i for i, c in enumerate(sent) if " X" in c)
    assert any(c.startswith("G0 Z290.0") for c in sent[:first_xy])   # up before any XY move
    assert "G0 X0.0 Y298.0 F4000" in sent                       # Y from -Y (backlash), X straight
    assert not any("X-" in c for c in sent)
    assert (rig.machine.x, rig.machine.y) == (150.0, 120.0) and rig.machine.z == 200.0   # back, at travel height
    assert not os.path.exists(os.path.join(str(tmp_path), "headcam.jpg"))   # LightBurn's 10W camera untouched
    with open(os.path.join(str(tmp_path), "closeup", "bedcam_pose.json")) as handle:
        pose = json.load(handle)
    assert (pose["x"], pose["y"], pose["z"]) == PARK and pose["source"] == "usb"


def test_the_usb_base_puts_every_cross_within_half_a_millimetre(module, tmp_path):
    p, rig, screen, cam = usb_ready(module, tmp_path)
    p._api_head_photo({})
    out = p._api_closeup_clear({})                              # the base alone, nothing moves
    assert out["qa"]["tiles"] == 0
    canvas = cvio.decode(p.closeup_image().body)
    geom = co.canvas_geometry(10.0)
    found = co.find_dots(canvas, geom, bc.CROSSES_LB, 6.0)
    err = [np.hypot(a - x, b - y) for (a, b), (x, y) in zip(found, bc.CROSSES_LB)]
    assert max(err) <= 0.5, err
    assert p._status()["closeup"]["base"] == "usb"
    assert p._status()["closeup"]["bedcam"]["bed_y"] == 300.0


def test_a_bed_photo_not_taken_at_the_calibrated_y_is_never_used(module, tmp_path):
    p, rig, screen, cam = usb_ready(module, tmp_path)
    p._api_head_photo({})
    path = os.path.join(str(tmp_path), "closeup", "bedcam_pose.json")
    with open(path) as handle:
        pose = json.load(handle)
    pose["y"], pose["time"] = 178.0, pose["time"] + 1
    with open(path, "w") as handle:
        json.dump(pose, handle)
    p._bedcam_pose_cache = None
    with pytest.raises(ValueError, match="Y178"):
        p._api_closeup_clear({})


@pytest.mark.parametrize("change,message", [
    ("rotary", "rotary"),
    ("uncalibrated", "calibrate the bed camera"),
    ("park_y", "calibrated with the bed at Y300"),
    ("url", "bedcam_url"),
    ("not_homed", "home it first"),
    ("lightburn", "LightBurn"),
])
def test_the_usb_overview_is_refused_like_the_close_up_and_nothing_moves(module, tmp_path, change, message):
    settings = {}
    if change == "park_y":
        settings["bedcam_park_y"] = 290.0
    if change == "url":
        settings["bedcam_url"] = ""
    p, rig, screen, cam = usb_ready(module, tmp_path, calibrated=(change != "uncalibrated"), **settings)
    if change == "rotary":
        rig.machine.rotary = True
    if change == "not_homed":
        p._homed = False
    if change == "lightburn":
        from lhp_pkg import grbl_bridge as gb
        from test_closeup_plugin import _Sender
        p._grbl = gb.Bridge(_Sender())
        p._grbl.lock_out("busy")
    rig.machine.sent.clear()
    with pytest.raises(ValueError, match=message):
        p._api_head_photo({})
    assert not any(c.startswith(("G0", "G1")) for c in rig.machine.sent)
    assert cam.grabs == [] and screen.requests == []


def test_a_usb_close_up_takes_its_overview_from_the_bed_camera_and_its_tiles_from_the_head(module, tmp_path):
    p, rig, screen, cam = usb_ready(module, tmp_path, raster="dots")
    plan = p._api_closeup_plan(dict(box=BOX, height=0))
    rig.machine.sent.clear()
    p._api_closeup(dict(box=BOX, height=0))
    wait(p)
    assert p._last_error is None, p._last_error
    assert [pose for _, pose in cam.grabs] == [PARK]           # the fresh overview, parked
    assert len(screen.requests) == len(plan["tiles"])          # the touchscreen only takes the tiles
    assert all(r[2] == 50.0 for r in screen.requests)
    assert not any("X-" in c for c in rig.machine.sent)         # no approach from beyond X0
    assert rig.machine.z >= 200.0                                # back up
    canvas = cvio.decode(p.closeup_image().body)
    geom = co.canvas_geometry(10.0)
    found = co.find_dots(canvas, geom, DOTS)
    assert max(np.hypot(a - x, b - y) for (a, b), (x, y) in zip(found, DOTS)) <= 0.1
    assert not os.path.exists(os.path.join(str(tmp_path), "headcam.jpg"))
    st = p._status()["closeup"]["last"]
    assert st["overview_time"] and st["tiles"] == len(plan["tiles"])


def test_a_usb_close_up_without_a_bed_calibration_is_refused_and_nothing_moves(module, tmp_path):
    p, rig, screen, cam = usb_ready(module, tmp_path, calibrated=False)
    rig.machine.sent.clear()
    with pytest.raises(ValueError, match="calibrate the bed camera"):
        p._api_closeup(dict(box=BOX, height=0))
    assert not any(c.startswith(("G0", "G1")) for c in rig.machine.sent) and cam.grabs == []


def test_the_default_base_never_asks_the_bed_camera(module, tmp_path):
    p, rig, screen, cam = usb_ready(module, tmp_path, closeup_base="headcam")
    p._api_head_photo({})
    p._api_closeup(dict(box=[140.0, 160.0, 165.0, 185.0], height=0))
    wait(p)
    assert p._last_error is None and cam.grabs == []
    assert screen.requests[0] == (232.0, 178.0, 290.0)


def _lightburn_entry(h):
    return dict(DisplayName="Bed cam - fixed", SelectedAlignmentName="A", SelectedCalibrationName="L",
                ImageAlignments=[dict(AlignmentName="A", CalibrationName="L", ImageSizeX=1280, ImageSizeY=720,
                                      MaterialHeight=0, ImageProjection=list(np.asarray(h).T.ravel()))],
                LensCalibrations=[dict(CalibrationName="L", CameraMatrix=list(np.asarray(K).T.ravel()),
                                       DistortionMatrix=DIST, CameraSizeX=1280, CameraSizeY=720, IsFisheye=False)])


def test_calibrating_from_lightburn_then_the_crosses(module, tmp_path):
    p, rig, screen, cam = usb_ready(module, tmp_path, calibrated=False)
    # LightBurn's alignment, about 4 % and 5 mm out (as seen on 2026-11-21)
    off = np.diag([1.04, 1.04, 1.0]).dot(np.asarray(H_LB)) + np.array([[0, 0, 5.0], [0, 0, -4.0], [0, 0, 0]])
    prefs = dict(DeviceList=[dict(Settings=dict(Cameras=[_lightburn_entry(off)]))])
    rig.machine.sent.clear()
    out = p._api_bedcam_calibrate(dict(prefs=json.dumps(prefs)))
    assert "unverified" in out["candidate"]["source"] and moves(rig) == []
    assert p._bedcam_model() is None                            # a candidate only
    out = p._api_bedcam_calibrate(dict(crosses=True))
    assert [pose for _, pose in cam.grabs] == [PARK]
    assert out["before"]["max_mm"] > 3.0
    assert out["after"]["n"] == 6 and out["after"]["max_mm"] <= 0.5, out["after"]
    assert p._bedcam_model() is None
    p._api_bedcam_calibrate(dict(accept=True))
    model = p._bedcam_model()
    assert model is not None and model.residuals["n"] == 6 and model.bed_y == 300.0
    # and the close-up base from it
    p._api_head_photo({})
    p._api_closeup_clear({})
    canvas = cvio.decode(p.closeup_image().body)
    geom = co.canvas_geometry(10.0)
    found = co.find_dots(canvas, geom, bc.CROSSES_LB, 6.0)
    assert max(np.hypot(a - x, b - y) for (a, b), (x, y) in zip(found, bc.CROSSES_LB)) <= 0.5


def test_the_crosses_calibration_is_refused_with_the_rotary_online(module, tmp_path):
    p, rig, screen, cam = usb_ready(module, tmp_path)
    rig.machine.rotary = True
    rig.machine.sent.clear()
    with pytest.raises(ValueError, match="rotary"):
        p._api_bedcam_calibrate(dict(crosses=True))
    assert moves(rig) == [] and cam.grabs == []


def test_bedcam_calibrate_needs_something_to_do(module, tmp_path):
    p, rig, screen, cam = usb_ready(module, tmp_path)
    with pytest.raises(ValueError, match="needs"):
        p._api_bedcam_calibrate({})
    with pytest.raises(ValueError, match="no bed camera calibration to accept"):
        p._api_bedcam_calibrate(dict(accept=True))


# -- settings ----------------------------------------------------------------

BASE = dict(esp_url="http://laserprobe.test")


def test_the_bed_camera_settings_default_to_the_head_camera_and_the_park_pose():
    config = routine_config(BASE, require_heights=False)
    assert config["closeup_base"] == "headcam"
    assert (config["bedcam_park_x"], config["bedcam_park_y"], config["bedcam_park_z"]) == PARK
    assert config["bedcam_url"] is None          # no bed camera until one is set
    assert routine_config(dict(BASE, closeup_base=" USB "), require_heights=False)["closeup_base"] == "usb"


@pytest.mark.parametrize("key,value,message", [
    ("closeup_base", "webcam", "closeup_base"),
    ("bedcam_url", "ftp://octopi.local/frame.jpeg", "bedcam_url"),
    ("bedcam_park_z", 150.0, "bedcam_park_z"),
    ("bedcam_park_y", 400.0, "bedcam_park_y"),
    ("bedcam_park_x", -5.0, "bedcam_park_x"),
])
def test_bad_bed_camera_settings_are_refused(key, value, message):
    with pytest.raises(SettingsError, match=message):
        routine_config(dict(BASE, **{key: value}), require_heights=False)


def test_the_settings_page_refuses_a_bad_base(module, tmp_path):
    p = make_plugin(module, Rig(), tmp_path)
    with pytest.raises(ValueError, match="closeup_base"):
        p.on_settings_save(dict(closeup_base="webcam"))
