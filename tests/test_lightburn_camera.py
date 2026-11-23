"""The close-up image as a LightBurn camera, written into LightBurn's prefs.ini.

The plugin's close-up image is already flat and in LightBurn workspace coordinates (pixel (i, j)
has its centre at LB X = (i + 0.5) / dpm, Y = bed_h - (j + 0.5) / dpm). So the camera needs no
lens model and an alignment that is exact by construction. LightBurn 2.1 stores both per camera
(seen in prefs.ini on 2026-11-20): CameraMatrix and ImageProjection are 3x3 matrices stored
column-major, and ImageProjection maps image pixels to workspace mm.
"""
import pytest

from lhp_pkg import lightburn_camera as lbc


def _apply(cm, u, v):
    """A column-major 3x3 applied to pixel (u, v)."""
    h = [[cm[0], cm[3], cm[6]], [cm[1], cm[4], cm[7]], [cm[2], cm[5], cm[8]]]
    x = h[0][0] * u + h[0][1] * v + h[0][2]
    y = h[1][0] * u + h[1][1] * v + h[1][2]
    w = h[2][0] * u + h[2][1] * v + h[2][2]
    return x / w, y / w


def test_a_canvas_pixel_and_its_workspace_point_round_trip():
    for dpm in (2.5, 10, 18):
        for x, y in ((0.0, 0.0), (40, 95), (280, 255), (320, 350), (123.45, 67.8)):
            u, v = lbc.canvas_px(x, y, dpm)
            assert _apply(lbc.projection(dpm), u, v) == pytest.approx((x, y), abs=1e-9)
    # pixel (0, 0) is the back-left corner of the bed, half a pixel in
    assert _apply(lbc.projection(10), 0, 0) == pytest.approx((0.05, 349.95))


def test_the_projection_matches_lightburns_storage_order():
    # The head camera's real alignment from prefs.ini: column-major, pixel -> mm.
    real = [0.3629313959197821, -0.004552908131935612, 8.520867648607337e-06,
            -0.004906615770275933, -0.3837077446484609, 3.937431167731721e-06,
            -11.667077681734416, 434.8493396811051, 0.9951355903713878]
    assert _apply(real, 148.496, 468.119) == pytest.approx((40, 255), abs=0.01)
    assert _apply(real, 819.759, 874.594) == pytest.approx((280, 95), abs=0.01)


def test_the_camera_entry_has_an_identity_lens_and_an_exact_alignment():
    e = lbc.camera_entry(dpm=10, url="http://octopi.local:5000/plugin/laserheightprobe/closeup.mjpg",
                         name="10W close-up", stamp="2026-11-21_09-00")
    w, h = 3200, 3500
    assert e["CameraType"] == "http" and e["MountType"] == "over"
    assert e["CameraURL"].endswith("/closeup.mjpg") and e["DisplayName"] == "10W close-up"
    lens = e["LensCalibrations"][0]
    assert lens["CameraSizeX"] == w and lens["CameraSizeY"] == h and lens["IsFisheye"] is False
    assert lens["DistortionMatrix"] == [0, 0, 0, 0, 0]
    assert lens["CameraMatrix"] == [w, 0, 0, 0, w, 0, w / 2.0, h / 2.0, 1]
    align = e["ImageAlignments"][0]
    assert align["CalibrationName"] == lens["CalibrationName"] == e["SelectedCalibrationName"]
    assert align["AlignmentName"] == e["SelectedAlignmentName"]
    assert (align["ImageSizeX"], align["ImageSizeY"], align["ImageDPM"]) == (w, h, 10000)
    assert align["MaterialHeight"] == 0
    feats, seen = align["AlignmentFeatures"], align["DetectedFeatures"]
    assert feats["FIDUCIAL:1"] == "40,255" and feats["FIDUCIAL:3"] == "280,95"
    for key in ("FIDUCIAL:1", "FIDUCIAL:2", "FIDUCIAL:3", "FIDUCIAL:4"):
        mm = tuple(float(s) for s in feats[key].split(","))
        px = tuple(float(s) for s in seen[key].split(","))
        assert _apply(align["ImageProjection"], *px) == pytest.approx(mm, abs=1e-6)


def test_adding_the_camera_replaces_one_of_the_same_name_and_keeps_the_others():
    prefs = {"DeviceList": [{"Settings": {"Cameras": [
        {"DisplayName": "Bed cam - fixed", "GUID": "mNWU4N2E"},
        {"DisplayName": "10W head cam - whole bed", "GUID": "UwZDY2LW"}]}}]}
    e1 = lbc.camera_entry(dpm=10, url="http://x/closeup.mjpg", name="10W close-up", stamp="a")
    out = lbc.add_camera(prefs, e1)
    names = [c["DisplayName"] for c in out["DeviceList"][0]["Settings"]["Cameras"]]
    assert names == ["Bed cam - fixed", "10W head cam - whole bed", "10W close-up"]
    e2 = lbc.camera_entry(dpm=8, url="http://x/closeup.mjpg", name="10W close-up", stamp="b")
    out = lbc.add_camera(out, e2)
    cams = out["DeviceList"][0]["Settings"]["Cameras"]
    assert len(cams) == 3 and cams[2]["ImageAlignments"][0]["ImageDPM"] == 8000
    assert cams[2]["GUID"] == cams[2]["GUID"] and len(cams[2]["GUID"]) == 8
    assert prefs["DeviceList"][0]["Settings"]["Cameras"][-1]["DisplayName"] == "10W head cam - whole bed"


def test_the_guid_is_stable_for_a_name_and_differs_between_names():
    a = lbc.camera_entry(dpm=10, url="u", name="10W close-up", stamp="x")["GUID"]
    b = lbc.camera_entry(dpm=4, url="v", name="10W close-up", stamp="y")["GUID"]
    c = lbc.camera_entry(dpm=10, url="u", name="Other", stamp="x")["GUID"]
    assert a == b != c


def test_a_canvas_that_does_not_fit_the_bed_is_refused():
    with pytest.raises(ValueError, match="dpm"):
        lbc.camera_entry(dpm=0, url="u", name="n", stamp="s")
