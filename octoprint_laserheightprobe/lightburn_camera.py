# coding=utf-8
"""The close-up image as a LightBurn camera: the entry the plugin writes into LightBurn's prefs.

The close-up image is already flat and in LightBurn workspace coordinates: pixel (i, j) has its
centre at LB X = (i + 0.5) / dpm, Y = bed_h - (j + 0.5) / dpm (LB's origin is front-left, the
image's top row is the back of the bed). So the camera needs no lens model and its alignment is
exact by construction. LightBurn 2.1 keeps both per camera in prefs.ini (DeviceList[n].Settings.
Cameras[]; read on 2026-11-20): CameraMatrix [fx,0,0, 0,fy,0, cx,cy,1] and ImageProjection, a
pixel -> mm homography, are 3x3 matrices stored column-major; the alignment also carries its four
fiducials in pixels (DetectedFeatures) and in mm (AlignmentFeatures), and ImageDPM, which matches
the image's pixels per metre. LightBurn rewrites prefs.ini while it runs, so it is edited only
with LightBurn closed. Plain Python.
"""
from __future__ import absolute_import

import base64
import copy
import hashlib

BED_W, BED_H = 320.0, 350.0
# The same four points as LightBurn's own manual alignment pattern (260 x 180, feature 20, centred).
FIDUCIALS = ((40.0, 255.0), (280.0, 255.0), (280.0, 95.0), (40.0, 95.0))


def canvas_px(x, y, dpm, bed_h=BED_H):
    """The canvas pixel (u, v), pixel centres at integers, of workspace point (x, y) in mm."""
    return x * dpm - 0.5, (bed_h - y) * dpm - 0.5


def projection(dpm, bed_h=BED_H):
    """The canvas pixel -> workspace mm homography, column-major as LightBurn stores it."""
    s = 1.0 / dpm
    # row-major [[s, 0, 0.5 s], [0, -s, bed_h - 0.5 s], [0, 0, 1]]
    return [s, 0.0, 0.0, 0.0, -s, 0.0, 0.5 * s, bed_h - 0.5 * s, 1.0]


def _fmt(a, b):
    return "%s,%s" % (_num(a), _num(b))


def _num(value):
    text = ("%.6f" % value).rstrip("0").rstrip(".")
    return text if text not in ("-0", "") else "0"


def _guid(name):
    """LightBurn's camera GUIDs are 8 characters; this one is stable for a name, so writing
    the camera again replaces it rather than adding another."""
    digest = hashlib.sha1(name.encode("utf-8")).digest()
    return base64.b64encode(digest).decode("ascii").replace("+", "A").replace("/", "B")[:8]


def camera_entry(dpm, url, name, stamp, bed_w=BED_W, bed_h=BED_H):
    """The Cameras[] entry for a close-up image of the whole bed at dpm pixels per mm."""
    if not dpm or dpm <= 0:
        raise ValueError("dpm must be above 0 (got %r)" % dpm)
    w, h = int(round(bed_w * dpm)), int(round(bed_h * dpm))
    lens_name = "plugin_identity_%s" % stamp
    align_name = "Plugin_0mm_%s" % stamp
    detected, features = {}, {}
    for n, (x, y) in enumerate(FIDUCIALS, 1):
        features["FIDUCIAL:%d" % n] = _fmt(x, y)
        detected["FIDUCIAL:%d" % n] = _fmt(*canvas_px(x, y, dpm, bed_h))
    features["WORKSPACE:CENTER"] = _fmt(bed_w / 2.0, bed_h / 2.0)
    return {
        "AlignmentInterpolateByHeight": False,
        "AlignmentLayout": {"LayoutType": 0, "TagSize": 20, "ZoneOffset": [0, 0], "ZoneSize": [260, 180]},
        "CameraType": "http",
        "CameraURL": url,
        "CameraURLDownloadLimit": 20,
        "CameraURLRate": 1,
        "CameraWarmUpFrames": 0,
        "DisplayName": name,
        "GUID": _guid(name),
        "ImageAlignments": [{
            "AlignmentFeatures": features,
            "AlignmentName": align_name,
            "CalibrationName": lens_name,
            "DetectedFeatures": detected,
            "ImageDPM": int(round(dpm * 1000)),
            "ImageProjection": projection(dpm, bed_h),
            "ImageSizeX": w,
            "ImageSizeY": h,
            "MaterialHeight": 0,
            "MountType": "over",
        }],
        "LensCalibrations": [{
            "CalibrationName": lens_name,
            "CameraMatrix": [w, 0, 0, 0, w, 0, w / 2.0, h / 2.0, 1],
            "CameraName": "",
            "CameraSizeX": w,
            "CameraSizeY": h,
            "DistortionMatrix": [0, 0, 0, 0, 0],
            "IsFisheye": False,
        }],
        "MountType": "over",
        "OverlayInclude": True,
        "PreviewFPS": 1,
        "SelectedAlignmentName": align_name,
        "SelectedCalibrationName": lens_name,
    }


def add_camera(prefs, entry, device=0):
    """A copy of prefs with entry in the device's Cameras, replacing one with the same name."""
    out = copy.deepcopy(prefs)
    settings = out["DeviceList"][device].setdefault("Settings", {})
    cams = [c for c in settings.get("Cameras", []) if c.get("DisplayName") != entry["DisplayName"]
            and c.get("GUID") != entry["GUID"]]
    cams.append(copy.deepcopy(entry))
    settings["Cameras"] = cams
    return out
