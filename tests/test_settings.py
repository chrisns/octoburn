"""The checks that run on save and before every job."""
import pytest

from lhp_pkg import settings
from lhp_pkg.settings import SettingsError, routine_config as _routine_config

HEIGHTS = dict(safe_z=60.0, z_floor=0.5)


def routine_config(change=None, **kwargs):
    """The settings with the two captured heights set, as on a machine that has them."""
    return _routine_config(dict(HEIGHTS, **(change or {})), **kwargs)


def test_defaults_are_valid_once_the_heights_are_captured():
    c = routine_config()
    assert c["stages"] == [(0.5, None), (0.1, 1.5), (0.02, 1.0)]
    assert c["laser_modules"] == [dict(name="10W", focus_offset=None, dx=0.0, dy=0.0),
                                  dict(name="2W IR", focus_offset=None, dx=None, dy=None)]
    assert c["probe_feed"] == 300 and c["tap_test"] is True
    assert c["points"] == [] and c["esp_url"] == "http://laserprobe.local"


def test_there_are_no_z_defaults():
    assert settings.DEFAULTS["safe_z"] is None and settings.DEFAULTS["z_floor"] is None
    assert settings.heights_missing({}) == ["safe_z", "z_floor"]
    assert settings.heights_missing(dict(safe_z=40)) == ["z_floor"]
    with pytest.raises(SettingsError, match="safe_z is not set"):
        _routine_config({})
    with pytest.raises(SettingsError, match="z_floor is not set"):
        _routine_config(dict(safe_z=40))
    # saving the other settings before the heights are captured is allowed
    loose = _routine_config({}, require_heights=False)
    assert loose["safe_z"] is None and loose["z_floor"] is None
    # but a value that is given must still be valid
    with pytest.raises(SettingsError, match="z_floor"):
        _routine_config(dict(z_floor=-1), require_heights=False)


def test_safe_z_must_be_above_z_floor():
    with pytest.raises(SettingsError, match="above z_floor"):
        routine_config(dict(safe_z=20.0, z_floor=20.0))
    with pytest.raises(SettingsError, match="above z_floor"):
        _routine_config(dict(safe_z=10.0, z_floor=20.0), require_heights=False)
    assert routine_config(dict(safe_z=20.5, z_floor=20.0))["safe_z"] == 20.5


@pytest.mark.parametrize("change,message", [
    (dict(z_floor=-0.1), "z_floor"),
    (dict(z_floor=60.0), "above z_floor"),
    (dict(safe_z="high"), "not a number"),
    (dict(samples=3, max_samples=2), "max_samples"),
    (dict(arm_wait_s=0.3), "arm_wait_s"),
    (dict(probe_feed=0), "probe_feed"),
    (dict(esp_url="laserprobe.local"), "esp_url"),
    (dict(stages="0.5"), "2 to 4"),
    (dict(stages="0.5; 0, 0.6"), "step"),
    (dict(stages="0.5; 0.1"), "start_above"),
    (dict(stages="0.5; 0.1, 0.6; 0.02, 0.02"), "more than its step"),
    (dict(stages="0.5; 0.6, 1.0"), "finer"),
    (dict(stages="0.5; 0.1, abc"), "not"),
    (dict(laser_modules=""), "at least one"),
    (dict(laser_modules="10W = x"), "not a number"),
    (dict(laser_modules="10W = 1\n10W = 2"), "twice"),
    (dict(laser_modules="10W = 1, 2"), "both dx and dy"),
    (dict(laser_modules="10W = 1, 2, 3, 4"), "more than"),
    (dict(laser_modules="10W = 1, 99, 0"), "beyond"),
    (dict(points="10 20\n30"), "line 2"),
    (dict(points="10 20 ref\n30 40 ref"), "only one"),
    (dict(points="10 20 laser"), "ref or datum"),
    (dict(points="1000 20"), "outside"),
])
def test_refused(change, message):
    with pytest.raises(SettingsError, match=message):
        routine_config(change)


def test_start_above_need_not_exceed_the_coarser_step():
    # the task says this is not required: only start_above > its own step
    assert routine_config(dict(stages="0.5; 0.1, 0.3; 0.02, 0.05"))["stages"][1] == (0.1, 0.3)


def test_points_parse_with_flags_and_comments():
    points = settings.parse_points("120 80 ref\n150, 80   # the lid\n\n90.5;60 datum\n")
    assert points == [dict(x=120.0, y=80.0, ref=True, datum=False),
                      dict(x=150.0, y=80.0, ref=False, datum=False),
                      dict(x=90.5, y=60.0, ref=False, datum=True)]
    assert settings.parse_points(settings.format_points(points)) == points


def test_grid_is_a_serpentine():
    g = settings.grid(0, 0, 20, 10, 3, 2)
    assert [(p["x"], p["y"]) for p in g] == [(0, 0), (10, 0), (20, 0), (20, 10), (10, 10), (0, 10)]
    assert len(settings.grid(5, 5, 5, 5, 1, 1)) == 1
    with pytest.raises(SettingsError):
        settings.grid(0, 0, 10, 10, 0, 2)
    with pytest.raises(SettingsError):
        settings.grid(0, 0, "x", 10, 2, 2)


def test_module_offsets_round_trip():
    text = settings.DEFAULT_MODULES
    text = settings.set_module_offset(text, "2W IR", 1.2345)
    assert settings.parse_modules(text) == [dict(name="10W", focus_offset=None, dx=0.0, dy=0.0),
                                            dict(name="2W IR", focus_offset=1.2345, dx=None, dy=None)]
    text = settings.set_module_offset(text, "10W", -0.5)
    text = settings.set_module_offset(text, "1.6W blue", 0.25)
    text = settings.set_module_offset(text, "2W IR", None)
    assert settings.parse_modules(text) == [dict(name="10W", focus_offset=-0.5, dx=0.0, dy=0.0),
                                            dict(name="2W IR", focus_offset=None, dx=None, dy=None),
                                            dict(name="1.6W blue", focus_offset=0.25, dx=None, dy=None)]
    assert text == "10W = -0.5, 0, 0\n2W IR =\n1.6W blue = 0.25"
    with pytest.raises(SettingsError):
        settings.set_module_offset(text, "bad=name", 1.0)
    with pytest.raises(SettingsError):
        settings.set_module_offset(text, "10W", 500)


def test_module_xy_offsets_are_per_module():
    text = settings.set_module_fields(settings.DEFAULT_MODULES, "2W IR", dx=-12.5, dy=3.25)
    modules = settings.parse_modules(text)
    assert modules[0]["dx"] == 0.0 and modules[1]["dx"] == -12.5 and modules[1]["dy"] == 3.25
    assert modules[1]["focus_offset"] is None
    assert text == "10W = , 0, 0\n2W IR = , -12.5, 3.25"
    text = settings.set_module_fields(text, "2W IR", focus_offset=17.0)
    assert settings.parse_modules(text)[1] == dict(name="2W IR", focus_offset=17.0, dx=-12.5, dy=3.25)
    text = settings.set_module_fields(text, "2W IR", dx=None, dy=None)
    assert settings.parse_modules(text)[1]["dx"] is None
    with pytest.raises(SettingsError, match="both"):
        settings.set_module_fields(text, "2W IR", dx=1.0)
    with pytest.raises(SettingsError):
        settings.set_module_fields(text, "10W", dx=99.0, dy=0.0)


def test_stages_round_trip():
    stages = settings.parse_stages("1\n0.2, 1.0\n0.05, 0.3\n0.01, 0.08")
    assert settings.parse_stages(settings.format_stages(stages)) == stages


def test_booleans_from_text():
    assert routine_config(dict(home_first="false"))["home_first"] is False
    assert routine_config(dict(home_first="on"))["home_first"] is True


def test_toolhead_mismatch_checks_what_m1006_can_tell():
    from lhp_pkg.settings import toolhead_mismatch
    assert toolhead_mismatch("10W", "10W LASER") is None
    assert "plugged in" in toolhead_mismatch("10W", "LASER")
    assert "plugged in" in toolhead_mismatch("2W IR", "10W LASER")
    assert toolhead_mismatch("2W IR", "2W RED LASER") is None    # the firmware's name for it
    assert "plugged in" in toolhead_mismatch("2W IR", "LASER")
    assert toolhead_mismatch("1.6W", "LASER") is None           # a module M1006 cannot name
    assert "not a laser" in toolhead_mismatch("10W", "3DP")
    assert "did not say" in toolhead_mismatch("10W", None)


def test_the_close_up_camera_defaults():
    c = routine_config()
    assert c["headcam_quality"] == 8                     # touchscreen photoQuality: 8 is as precise as 1, 3x faster
    assert c["closeup_standoff"] == 50.0 and c["closeup_overlap"] == 0.25
    assert c["closeup_min_clearance"] == 10.0 and c["closeup_max_height"] == 60.0
    assert c["closeup_hood_radius"] == 5.0 and c["closeup_dpm"] == 15.0
    assert c["closeup_max_mp"] == 45.0 and c["closeup_max_tiles"] == 25
    assert c["closeup_feather_mm"] == 2.0 and c["closeup_jpeg_quality"] == 90
    assert c["closeup_stream_s"] == 2.0 and c["closeup_capture_s"] == 6.0
    assert c["closeup_approach_mm"] == 2.0 and c["closeup_shadow_frac"] == 0.4 and c["closeup_height"] is None


@pytest.mark.parametrize("change,message", [
    (dict(closeup_standoff=20), "closeup_standoff"),       # below the camera's sharp range
    (dict(closeup_overlap=0.8), "closeup_overlap"),
    (dict(closeup_min_clearance=1), "closeup_min_clearance"),
    (dict(closeup_dpm=30), "closeup_dpm"),                  # LightBurn tested to 20 px/mm (B6)
    (dict(closeup_max_tiles=100), "closeup_max_tiles"),
    (dict(closeup_jpeg_quality=20), "closeup_jpeg_quality"),
])
def test_close_up_settings_out_of_range_are_refused(change, message):
    with pytest.raises(SettingsError, match=message):
        routine_config(change)


def test_the_object_height_argument_is_checked_like_a_setting():
    assert settings.number("closeup_height", "12.5") == 12.5
    with pytest.raises(SettingsError, match="closeup_height"):
        settings.number("closeup_height", 200)
