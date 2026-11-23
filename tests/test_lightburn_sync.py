"""tools/octoburn_lightburn_sync.py: LightBurn's prefs.ini kept in step with OctoBurn's status.

The fixture (data/lightburn/prefs_sample.ini) is hand-written and sanitised, in the shape of
LightBurn 2.1's prefs.ini (read on 2026-11-22): the whole file is JSON as Qt writes it
(4-space indent, sorted keys, empty lists and objects over two lines), with the devices in
DeviceList[] and each device's rotary settings, Start GCode and cameras in its Settings.
Nothing here may touch the real LightBurn folder: every test works on a copy in tmp_path.
"""
import copy
import datetime
import importlib.util
import json
import os
import pathlib

import pytest

from lhp_pkg import grbl_bridge as gb

ROOT = pathlib.Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "data" / "lightburn" / "prefs_sample.ini"
BASE = "http://octoprint.example:5000"
URL = BASE + "/plugin/laserheightprobe/closeup.mjpg"

_spec = importlib.util.spec_from_file_location("octoburn_lightburn_sync",
                                               str(ROOT / "tools" / "octoburn_lightburn_sync.py"))
sync = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(sync)


@pytest.fixture(autouse=True)
def _never_the_real_prefs(monkeypatch, tmp_path):
    """No test reaches the real prefs.ini or the real pgrep."""
    monkeypatch.setattr(sync, "DEFAULT_PREFS", str(tmp_path / "nowhere" / "prefs.ini"))

    def _no_pgrep():
        raise AssertionError("a test reached the real LightBurn check")
    monkeypatch.setattr(sync, "lightburn_running", _no_pgrep)


def status(**rotary):
    r = dict(complete=True, diameter=78.28, stale=None, finished=1795365199.0)
    r.update(rotary)
    return {"selected_module": "10W", "modules": [{"name": "10W"}, {"name": "2W IR"}],
            "rotary": r, "closeup": {"available": True, "dpm": 15.0}}


def setup(tmp_path, st=None, edit=None):
    """A LightBurn folder in tmp_path with prefs.ini (the fixture, optionally edited as JSON and
    written back in Qt's format) and a status file. Returns (prefs path, status path)."""
    folder = tmp_path / "LightBurn"
    folder.mkdir()
    text = FIXTURE.read_text()
    if edit:
        prefs = json.loads(text)
        edit(prefs)
        text = sync.qt_dumps(prefs) + "\n"
    (folder / "prefs.ini").write_text(text)
    (tmp_path / "status.json").write_text(json.dumps(status() if st is None else st))
    return folder / "prefs.ini", tmp_path / "status.json"


def run(prefs, st, *extra, running=False):
    return sync.main(["--prefs", str(prefs), "--status-json", str(st), "--octoprint", BASE] + list(extra),
                     is_running=lambda: running, now=datetime.datetime(2026, 11, 22, 18, 30, 0))


def settings(prefs_path, index):
    return json.loads(prefs_path.read_text())["DeviceList"][index]["Settings"]


def plan_for(prefs_obj, st=None, **kw):
    return sync.make_plan(prefs_obj, status() if st is None else st, BASE, **kw)


# -- the format ---------------------------------------------------------------------------------

def test_the_fixture_is_in_qt_format_so_the_serialiser_matches_lightburn():
    text = FIXTURE.read_text()
    assert sync.qt_dumps(json.loads(text)) + "\n" == text
    index = sync.JsonIndex(text)
    start, end = index.spans[("DeviceList", 1, "Settings", "rotaryDiameter")]
    assert text[start:end] == "79.20999908447266"


def test_an_insert_goes_in_sorted_place_and_at_the_end_and_into_an_empty_object():
    text = FIXTURE.read_text()
    path = ("DeviceList", 3, "Settings")
    for key in ("AAA", "rotaryMode", "zzz"):
        out = sync.render(text, [sync.Change("insert", path, 1.5, "d", "x", key=key)])
        assert json.loads(out)["DeviceList"][3]["Settings"][key] == 1.5
        assert sync.qt_dumps(json.loads(out)) + "\n" == out
    out = sync.render(text, [sync.Change("insert", ("DeviceList", 0, "CustomAlarmCodes"), "v", "d", "x", key="k")])
    assert json.loads(out)["DeviceList"][0]["CustomAlarmCodes"] == {"k": "v"}
    assert sync.qt_dumps(json.loads(out)) + "\n" == out


# -- the rotary device -------------------------------------------------------------------------

def test_diameter_and_marker_are_updated_and_nothing_else_changes(tmp_path):
    prefs, st = setup(tmp_path)
    before = prefs.read_text()
    assert run(prefs, st, "--apply") == 0
    after = prefs.read_text()
    s = settings(prefs, 1)
    assert s["rotaryDiameter"] == 78.28 and s["rotaryObjDiameter"] == 78.28
    assert s["StartGCode"] == "(lhp head=10W mode=rotary dia=78.28)"
    changed = [(a, b) for a, b in zip(before.split("\n"), after.split("\n")) if a != b]
    assert len(before.split("\n")) == len(after.split("\n"))
    assert changed == [
        ('                "StartGCode": "(lhp head=10W mode=rotary dia=79.21)",',
         '                "StartGCode": "(lhp head=10W mode=rotary dia=78.28)",'),
        ('                "rotaryDiameter": 79.20999908447266,', '                "rotaryDiameter": 78.28,'),
        ('                "rotaryObjDiameter": 79.20999908447266,', '                "rotaryObjDiameter": 78.28,'),
    ]
    # the 2W IR device is another module's: left alone
    assert settings(prefs, 2)["StartGCode"] == "(lhp head=2W IR mode=rotary dia=79.21)"


def test_untouched_sections_stay_byte_for_byte(tmp_path):
    prefs, st = setup(tmp_path)
    before = prefs.read_bytes()
    assert run(prefs, st, "--apply") == 0
    after = prefs.read_bytes()
    index = sync.JsonIndex(before.decode())
    # everything before the rotary device and everything after it is identical
    start, end = index.spans[("DeviceList", 1)]
    assert after[:start] == before[:start]
    assert after.endswith(before[end:])


def test_the_planned_markers_pass_the_bridge_check(tmp_path):
    prefs, st = setup(tmp_path)
    assert run(prefs, st, "--apply") == 0
    rot = gb.job_marker(settings(prefs, 1)["StartGCode"])
    assert rot["mode"] == "rotary" and gb.same_head(rot["head"], "10W")
    assert abs(rot["dia"] - 78.28) <= gb.DIA_TOLERANCE
    flat = gb.job_marker(settings(prefs, 0)["StartGCode"])
    assert flat["mode"] == "flat" and gb.same_head(flat["head"], "10W")
    for line in ("(lhp head=2W IR mode=rotary dia=78.3)", "(lhp head=10W mode=flat)", "G21"):
        assert sync.job_marker(line) == gb.job_marker(line)


def test_dia_is_added_to_a_marker_without_it(tmp_path):
    def edit(p):
        p["DeviceList"][1]["Settings"]["StartGCode"] = "G21\n(lhp head=10W mode=rotary)\nM5"
    prefs, st = setup(tmp_path, edit=edit)
    assert run(prefs, st, "--apply") == 0
    assert settings(prefs, 1)["StartGCode"] == "G21\n(lhp head=10W mode=rotary dia=78.28)\nM5"


def test_a_marker_is_added_first_when_there_is_none(tmp_path):
    def edit(p):
        p["DeviceList"][1]["Settings"]["StartGCode"] = "G21"
        del p["DeviceList"][0]["Settings"]["StartGCode"]
    prefs, st = setup(tmp_path, edit=edit)
    assert run(prefs, st, "--apply") == 0
    assert settings(prefs, 1)["StartGCode"] == "(lhp head=10W mode=rotary dia=78.28)\nG21"
    assert settings(prefs, 0)["StartGCode"] == "(lhp head=10W mode=flat)"
    assert sync.qt_dumps(json.loads(prefs.read_text())) + "\n" == prefs.read_text()


def test_a_float32_diameter_close_to_the_measurement_is_already_right(tmp_path):
    def edit(p):
        s = p["DeviceList"][1]["Settings"]
        s["rotaryDiameter"] = s["rotaryObjDiameter"] = 78.27999877929688
        s["StartGCode"] = "(lhp head=10W mode=rotary dia=78.28)"
    prefs, st = setup(tmp_path, edit=edit)
    assert run(prefs, st) == 0


def test_a_missing_object_diameter_is_inserted(tmp_path):
    def edit(p):
        del p["DeviceList"][1]["Settings"]["rotaryObjDiameter"]
    prefs, st = setup(tmp_path, edit=edit)
    assert run(prefs, st, "--apply") == 0
    assert settings(prefs, 1)["rotaryObjDiameter"] == 78.28


# -- the flat device and the camera ------------------------------------------------------------

def test_the_flat_marker_is_checked():
    prefs = json.loads(FIXTURE.read_text())
    plan = plan_for(prefs)
    assert ("Snapmaker 10W flat via OctoPrint", "Start GCode marker (lhp head=10W mode=flat)") in plan.ok
    prefs["DeviceList"][0]["Settings"]["StartGCode"] = "(lhp head=2W IR mode=flat)"
    plan = plan_for(prefs)
    flat = [c for c in plan.changes if c.device == "Snapmaker 10W flat via OctoPrint"]
    assert [(c.label, c.old, c.value) for c in flat] == [
        ("Start GCode", "(lhp head=2W IR mode=flat)", "(lhp head=10W mode=flat)")]


def test_the_camera_is_checked_and_its_url_fixed():
    prefs = json.loads(FIXTURE.read_text())
    plan = plan_for(prefs)
    assert ("Snapmaker 10W flat via OctoPrint", 'camera "10W close-up" at %s' % URL) in plan.ok
    plan = sync.make_plan(prefs, status(), "http://other.example:5000")
    cam = [c for c in plan.changes if "camera" in c.label]
    assert len(cam) == 1 and cam[0].kind == "set"
    assert cam[0].path == ("DeviceList", 0, "Settings", "Cameras", 1, "CameraURL")
    assert cam[0].value == "http://other.example:5000/plugin/laserheightprobe/closeup.mjpg"


def test_a_missing_camera_is_added_from_the_plugins_entry(tmp_path):
    def edit(p):
        p["DeviceList"][0]["Settings"]["Cameras"].pop()
    prefs, st = setup(tmp_path, edit=edit)
    assert run(prefs, st, "--apply") == 0
    cams = settings(prefs, 0)["Cameras"]
    assert [c["DisplayName"] for c in cams] == ["Bed cam - fixed", "10W close-up"]
    assert cams[1]["CameraURL"] == URL and cams[1]["ImageAlignments"][0]["ImageDPM"] == 15000
    assert sync.qt_dumps(json.loads(prefs.read_text())) + "\n" == prefs.read_text()


def test_a_device_without_cameras_gets_the_list(tmp_path):
    def edit(p):
        del p["DeviceList"][0]["Settings"]["Cameras"]
    prefs, st = setup(tmp_path, edit=edit)
    assert run(prefs, st, "--apply") == 0
    assert [c["DisplayName"] for c in settings(prefs, 0)["Cameras"]] == ["10W close-up"]


def test_no_close_up_means_the_camera_is_not_checked():
    st = status()
    st["closeup"] = {"available": False}
    prefs = json.loads(FIXTURE.read_text())
    prefs["DeviceList"][0]["Settings"]["Cameras"] = []
    plan = plan_for(prefs, st)
    assert not [c for c in plan.changes if "camera" in c.label]
    assert any("not available" in n for n in plan.notes)


# -- refusals ----------------------------------------------------------------------------------

@pytest.mark.parametrize("rotary", [
    dict(stale={"reason": "the rotary moved", "time": 1.0}),
    dict(complete=False),
    dict(diameter=None),
    None,
])
def test_a_stale_or_incomplete_measurement_is_refused(tmp_path, rotary):
    st = status(**(rotary or {}))
    if rotary is None:
        st["rotary"] = None
    prefs, stp = setup(tmp_path, st=st)
    before = prefs.read_bytes()
    assert run(prefs, stp, "--apply") == 2
    assert prefs.read_bytes() == before
    assert not (prefs.parent / "backup").exists()


def test_skip_rotary_syncs_the_rest_without_a_diameter(tmp_path):
    def edit(p):
        p["DeviceList"][0]["Settings"]["StartGCode"] = ""
    prefs, stp = setup(tmp_path, st=status(stale={"reason": "moved"}), edit=edit)
    assert run(prefs, stp, "--apply", "--skip-rotary") == 0
    assert settings(prefs, 0)["StartGCode"] == "(lhp head=10W mode=flat)"
    assert settings(prefs, 1)["rotaryDiameter"] == pytest.approx(79.21, abs=1e-5)


def test_a_missing_device_is_refused(tmp_path):
    def edit(p):
        p["DeviceList"][1]["DisplayName"] = "My rotary"
    prefs, stp = setup(tmp_path, edit=edit)
    assert run(prefs, stp) == 2


def test_devices_are_matched_flexibly():
    prefs = {"DeviceList": [{"DisplayName": "snapmaker 10w  Rotary"}, {"DisplayName": "Snapmaker 2W IR flat via OctoPrint"},
                            {"DisplayName": "Other"}]}
    found = sync.find_devices(prefs)
    assert [(d["index"], d["head"], d["mode"]) for d in found] == [(0, "10w", "rotary"), (1, "2W IR", "flat")]
    assert sync.pick_device(found, "10W", "rotary")["index"] == 0


def test_apply_is_refused_while_lightburn_runs(tmp_path):
    prefs, st = setup(tmp_path)
    before = prefs.read_bytes()
    assert run(prefs, st, "--apply", running=True) == 2
    assert prefs.read_bytes() == before
    assert not (prefs.parent / "backup").exists()
    # a dry run still reports what is pending
    assert run(prefs, st, running=True) == 1


# -- dry run, backup, atomic write -------------------------------------------------------------

def test_a_dry_run_writes_nothing(tmp_path, capsys):
    prefs, st = setup(tmp_path)
    before, mtime = prefs.read_bytes(), prefs.stat().st_mtime_ns
    assert run(prefs, st) == 1
    assert prefs.read_bytes() == before and prefs.stat().st_mtime_ns == mtime
    assert sorted(os.listdir(prefs.parent)) == ["prefs.ini"]
    out = capsys.readouterr().out
    assert "rotary diameter (rotaryDiameter): 79.21 -> 78.28" in out
    assert "- (lhp head=10W mode=rotary dia=79.21)" in out and "+ (lhp head=10W mode=rotary dia=78.28)" in out
    assert "3 changes pending" in out


def test_apply_backs_up_into_lightburns_backup_folder_and_leaves_no_bak(tmp_path):
    prefs, st = setup(tmp_path)
    before = prefs.read_bytes()
    assert run(prefs, st, "--apply") == 0
    backup = prefs.parent / "backup" / "octoburn-sync" / "20261122_183000" / "prefs.ini"
    assert backup.read_bytes() == before
    assert sorted(os.listdir(prefs.parent)) == ["backup", "prefs.ini"]
    # nothing left to do afterwards
    assert run(prefs, st) == 0


def test_backup_dir_option(tmp_path):
    prefs, st = setup(tmp_path)
    assert run(prefs, st, "--apply", "--backup-dir", str(tmp_path / "bk")) == 0
    assert (tmp_path / "bk" / "20261122_183000" / "prefs.ini").exists()
    assert not (prefs.parent / "backup").exists()


def test_a_prefs_file_changed_since_planning_is_not_overwritten(tmp_path):
    prefs, _ = setup(tmp_path)
    original = prefs.read_bytes()
    prefs.write_bytes(original + b" ")
    with pytest.raises(sync.Refused):
        sync.write_prefs(str(prefs), original, b"{}\n", is_running=lambda: False)
    assert prefs.read_bytes() == original + b" "


def test_a_change_that_does_not_parse_back_is_never_written():
    text = FIXTURE.read_text()
    bad = sync.Change("set", ("DeviceList", 1, "Settings", "rotaryDiameter"), 1.0, "d", "x")
    bad.apply_to = lambda obj: None          # the check expects no change, the edit makes one
    with pytest.raises(ValueError):
        sync.render(text, [bad])
