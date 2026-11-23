"""Recent burns: the burned bounds from the bridge, the close-up after a burn, the history."""
import json
import os

import pytest

from lhp_pkg import burns
from lhp_pkg import grbl_bridge as gb
from test_grbl_bridge import Clock, feed, rig
from test_closeup_plugin import ready, wait
from test_plugin import module  # noqa: F401  (module is a fixture)

# 80 mm at S128, 90 mm at S128, 50 mm at S255, all at 1200 mm/min; then the finish move
JOB = [b"(lhp head=10W mode=flat)", b"G90", b"M4 S0", b"G0 X120Y130", b"G1 X200S128F1200", b"G1 Y220",
       b"G1 X150S255", b"G0 X0Y300S0", b"M5"]
BOUNDS = [120.0, 130.0, 200.0, 220.0]
DUTY_S = (80 + 90) / 1200.0 * 60 * 128 / 255.0 + 50 / 1200.0 * 60


class Wall(object):
    def __init__(self):
        self.t = 1.8e9

    def __call__(self):
        self.t += 1.0
        return self.t


def flat(focus=22.2, module="10W", armed=True, **kw):
    clock = Clock()
    ctx = gb.FlatContext(focus, laser_until=(clock.t + 900) if armed else None, clock=clock, module=module, **kw)
    clock, sender, bridge, session, written = rig(ctx)
    tracker = burns.BurnTracker(clock=clock, wall=Wall())
    bridge.burn_watch = tracker.move
    return clock, bridge, session, tracker


def settle(clock, bridge, step):
    """The job's moves done, then the bridge idle past IDLE_S: step() each time."""
    clock.t = max(clock.t, bridge.busy_until) + 0.5
    assert step() is None                                     # idle seen: the wait starts
    clock.t += burns.IDLE_S - 1
    assert step() is None
    clock.t += 2
    return step()


# -- the tracker ------------------------------------------------------------------------------

def test_the_burned_bounds_include_where_each_burn_started():
    clock, bridge, session, tracker = flat()
    assert all(r == "ok" for t, r in feed(session, bridge, clock, JOB))
    assert tracker.busy()
    job = settle(clock, bridge, lambda: tracker.step(bridge.state()))
    assert job["box"] == BOUNDS                               # not the X0 Y300 finish move
    assert job["mode"] == "flat" and job["focus_z"] == 22.2 and job["module"] == "10W"
    assert job["max_power"] == 255 and job["moves"] == 3 and job["area"] == [0.0, 0.0, 320.0, 350.0]
    assert job["duty_s"] == pytest.approx(DUTY_S)
    entry = burns.entry_for(job, 10.0)
    assert entry["energy_j"] == pytest.approx(DUTY_S * 10.0, abs=0.1)
    assert entry["feed"] == pytest.approx(1200.0) and entry["max_power_pct"] == 100.0
    assert entry["bounds"] == BOUNDS and not entry["alarm"]
    assert entry["duration_s"] == job["idle_at"] - job["start"] > 0     # to when the moves were done
    assert not tracker.busy()


def test_the_job_is_not_finished_while_the_bridge_runs():
    clock, bridge, session, tracker = flat()
    feed(session, bridge, clock, JOB)
    clock.t = bridge.busy_until + 0.5
    assert tracker.step("Idle") is None
    clock.t += 20
    assert tracker.step("Run") is None                        # a new burn started: wait again
    assert tracker.step("Idle") is None
    clock.t += burns.IDLE_S + 1
    assert tracker.step("Idle") is not None


@pytest.mark.parametrize("job,kw", [
    # a dry run: every burn goes out at S0
    ([b"(lhp head=10W mode=flat)", b"G90", b"M4 S0", b"G0 X120Y130", b"G1 X200S128F1200", b"M5"],
     dict(armed=False, dry_run=True)),
    # LightBurn's frame: the beam off
    ([b"(lhp head=10W mode=flat)", b"G90", b"M5", b"G0 X120Y130", b"G1 X200F6000", b"G1 Y220", b"G1 X120",
      b"G1 Y130", b"G0 X0Y300"], {}),
    # laser on but every move at S0
    ([b"(lhp head=10W mode=flat)", b"G90", b"M4 S0", b"G0 X120Y130", b"G1 X200S0F1200", b"M5"], {}),
])
def test_a_dry_run_or_a_frame_is_no_burn(job, kw):
    clock, bridge, session, tracker = flat(**kw)
    assert all(r == "ok" for t, r in feed(session, bridge, clock, job))
    assert not tracker.busy()


def test_the_pointer_framing_is_no_burn():
    clock, bridge, session, tracker = flat(armed=False)
    bridge.set_pointer(dict(kind="spot", s_max=10, until=clock.t + 300))
    feed(session, bridge, clock, JOB)
    assert not tracker.busy()


def test_a_watcher_failure_never_stops_a_burn():
    clock, bridge, session, tracker = flat()

    def broken(mv, start, context):
        raise RuntimeError("boom")
    bridge.burn_watch = broken
    assert all(r == "ok" for t, r in feed(session, bridge, clock, JOB))


def test_the_close_up_box_is_padded_and_clipped_and_the_height_never_negative():
    assert burns.closeup_box([10.0, 2.0, 50.0, 348.0], [0.0, 0.0, 320.0, 350.0]) == [5.0, 0.0, 55.0, 350.0]
    assert burns.surface_height(25.2, 22.2) == 3.0
    assert burns.surface_height(21.9, 22.2) == 0.0


# -- the history ------------------------------------------------------------------------------

def test_the_history_keeps_the_last_fifty_and_their_images(tmp_path):
    h = burns.BurnHistory(str(tmp_path))
    ids = []
    for i in range(55):
        e = h.add(dict(time=1.8e9 + (i // 2), duration_s=i))     # two to a second: ids still unique
        h.save_image(e["id"], b"\xff\xd8%d\xff\xd9" % i)
        ids.append(e["id"])
    assert len(set(ids)) == 55
    again = burns.BurnHistory(str(tmp_path))                   # from disk
    kept = again.entries()
    assert len(kept) == burns.KEEP and [e["id"] for e in kept] == ids[5:]
    assert again.image_path(ids[0]) is None and not os.path.exists(os.path.join(str(tmp_path), "burns", ids[0] + ".jpg"))
    assert again.image_path(ids[-1]).endswith(ids[-1] + ".jpg")
    assert again.update(ids[-1], closeup=dict(status="done"))["closeup"]["status"] == "done"
    assert again.update(ids[0], closeup={}) is None
    with open(os.path.join(str(tmp_path), "burns.json")) as handle:
        assert len(json.load(handle)) == burns.KEEP
    assert again.clear() == burns.KEEP
    assert again.entries() == [] and os.listdir(os.path.join(str(tmp_path), "burns")) == []
    assert burns.BurnHistory(str(tmp_path)).entries() == []


def test_the_history_never_builds_a_path_from_a_bad_id(tmp_path):
    h = burns.BurnHistory(str(tmp_path))
    for bad in ("../burns", "x.jpg", "", None, "20261122-120000/../../a"):
        assert h.image_path(bad) is None
    with pytest.raises(ValueError):
        h.save_image("../evil", b"x")


# -- through the plugin -----------------------------------------------------------------------

def plugin_with_bridge(module, tmp_path, focus=22.2, head="10W", **settings):
    """The close-up rig (bed Z13.6, 10W focus offset 8.6: the bed's focus is Z22.2) with a real
    bridge in flat mode at focus."""
    p, rig_, screen = ready(module, tmp_path, **settings)
    p._door = (0.0, "closed")                                # the auto close-up needs the door shut
    clock, bridge, session, tracker = flat(focus=focus, module=head)
    p._grbl = bridge
    p._burn_tracker = tracker
    return p, rig_, screen, clock, bridge, session


def fake_closeup(p):
    calls = []

    def start(data):
        calls.append(data)
        return dict(started=True, tiles=4, seconds=60.0, head_z=50.0)
    p._api_closeup = start
    return calls


def test_a_finished_burn_starts_a_close_up_of_its_bounds_at_its_height(module, tmp_path):
    p, rig_, screen, clock, bridge, session = plugin_with_bridge(module, tmp_path, focus=25.2)
    calls = fake_closeup(p)
    feed(session, bridge, clock, JOB)
    entry = settle(clock, bridge, p._burns_step)
    assert calls == [dict(box=[115.0, 125.0, 205.0, 225.0], height=3.0)]
    assert entry["closeup"]["status"] == "running" and entry["energy_j"] == pytest.approx(DUTY_S * 10.0, abs=0.1)
    assert entry["focus_z"] == 25.2 and entry["mode"] == "flat"
    st = p._status()["burns"]
    assert st["enabled"] is True and [e["id"] for e in st["entries"]] == [entry["id"]]
    assert st["entries"][0]["image_url"] is None
    assert any("Burn finished" in line for line in p._log)


@pytest.mark.parametrize("change,status,needle", [
    ("off", "off", "off in the settings"),
    ("2W IR fitted", "skipped", "No camera on the 2W IR"),
    ("2W IR job", "skipped", "No camera on the 2W IR"),
])
def test_no_close_up_when_it_is_off_or_the_module_has_no_camera(module, tmp_path, change, status, needle):
    settings = dict(burn_closeup_after=False) if change == "off" else {}
    p, rig_, screen, clock, bridge, session = plugin_with_bridge(
        module, tmp_path, head="2W IR" if change == "2W IR job" else "10W", **settings)
    if change == "2W IR fitted":
        p._detected = dict(toolhead="2W RED LASER", laser=True, module="2W IR")
    calls = fake_closeup(p)
    job = [JOB[0].replace(b"10W", b"2W IR")] + JOB[1:] if change == "2W IR job" else JOB
    assert all(r == "ok" for t, r in feed(session, bridge, clock, job))
    entry = settle(clock, bridge, p._burns_step)
    assert calls == []
    assert entry["closeup"]["status"] == status and needle in entry["closeup"]["message"]
    assert p._burns().get(entry["id"]) is not None             # the burn is still in the history


def test_no_close_up_after_a_dry_run(module, tmp_path):
    p, rig_, screen, clock, bridge, session = plugin_with_bridge(module, tmp_path)
    bridge.set_context(gb.FlatContext(22.2, laser_until=None, clock=clock, module="10W", dry_run=True))
    calls = fake_closeup(p)
    feed(session, bridge, clock, JOB)
    clock.t = bridge.busy_until + 30
    for _ in range(3):
        assert p._burns_step() is None
        clock.t += burns.IDLE_S
    assert calls == [] and p._burns().entries() == []


def test_with_the_rotary_fitted_the_refusal_is_kept_in_the_entry(module, tmp_path):
    p, rig_, screen, clock, bridge, session = plugin_with_bridge(module, tmp_path)
    rig_.machine.rotary = True
    feed(session, bridge, clock, JOB)
    entry = settle(clock, bridge, p._burns_step)
    assert entry["closeup"]["status"] == "refused" and "rotary is fitted" in entry["closeup"]["message"]
    assert not p._running() and rig_.machine.z == 290.0          # nothing moved


def test_a_burn_close_up_end_to_end_keeps_a_crop_and_serves_it(module, tmp_path):
    from lhp_pkg import cvio
    p, rig_, screen, clock, bridge, session = plugin_with_bridge(module, tmp_path)
    p._api_head_photo({})
    feed(session, bridge, clock, JOB)
    entry = settle(clock, bridge, p._burns_step)
    assert entry["closeup"]["status"] == "running", entry["closeup"]
    assert bridge.context is None                               # the close-up disarmed the bridge
    wait(p)
    assert p._last_error is None, p._last_error
    p._burns_step()                                             # sees the close-up has finished
    done = p._burns().get(entry["id"])
    assert done["closeup"]["status"] == "done" and done["image"] is True, done["closeup"]
    resp = p.burn_image(entry["id"])
    crop = cvio.decode(resp.body)
    assert resp.mimetype == "image/jpeg"
    assert crop.shape[:2] == (100 * 10, 90 * 10)                # 90 x 100 mm at 10 px/mm
    st = p._status()["burns"]["entries"][0]
    assert st["image_url"] == "plugin/laserheightprobe/burns/%s.jpg" % entry["id"]
    assert crop.std() > 5                                       # a photo of the dots, not a blank


def test_the_route_serves_only_known_burns_and_clear_wipes_them(module, tmp_path):
    p, rig_, screen = ready(module, tmp_path)
    e = p._burns().add(dict(time=1.8e9, duration_s=1.0))
    p._burns().save_image(e["id"], b"\xff\xd8jpeg\xff\xd9")
    p._burns().update(e["id"], image=True)
    assert p.burn_image(e["id"]).body == b"\xff\xd8jpeg\xff\xd9"
    flask = __import__("flask")
    for bad in ("20990101-000000", "../closeup/closeup", "x"):
        with pytest.raises(flask.Abort):
            p.burn_image(bad)
    assert p.COMMAND_PERMISSIONS["burns_clear"] == "PROBE"
    assert p._api_burns_clear({}) == dict(ok=True, cleared=1)
    with pytest.raises(flask.Abort):
        p.burn_image(e["id"])
    assert p._status()["burns"]["entries"] == []


@pytest.mark.parametrize("box,refused", [([208.0, 150.0, 215.0, 160.0], True), ([0.0, 330.0, 10.0, 340.0], False)])
def test_an_obstacle_under_the_close_ups_path_refuses_it(module, tmp_path, box, refused):
    p, rig_, screen, clock, bridge, session = plugin_with_bridge(module, tmp_path)
    p._add_obstacle(box, 40.0, "a tape roll", "manual")
    calls = fake_closeup(p)
    feed(session, bridge, clock, JOB)
    entry = settle(clock, bridge, p._burns_step)
    if refused:
        assert calls == [] and entry["closeup"]["status"] == "refused"
        assert "a tape roll (40 mm tall) is under the close-up's path" in entry["closeup"]["message"]
        assert rig_.machine.z == 290.0
    else:
        assert len(calls) == 1 and entry["closeup"]["status"] == "running"


def test_open_tabs_hear_of_each_change_and_a_lost_job_counts_as_finished(module, tmp_path):
    p, rig_, screen, clock, bridge, session = plugin_with_bridge(module, tmp_path)
    fake_closeup(p)                                           # p._job stays None
    feed(session, bridge, clock, JOB)
    entry = settle(clock, bridge, p._burns_step)
    assert dict(type="burns") in p._plugin_manager.sent
    p._plugin_manager.sent.clear()
    p._burns_step()                                           # no job to wait for: failed, not stuck
    assert p._burn_pending is None
    assert p._burns().get(entry["id"])["closeup"]["status"] == "failed"
    assert dict(type="burns") in p._plugin_manager.sent



def test_no_automatic_close_up_with_the_door_open_or_unknown_and_an_opening_door_stops_it(module, tmp_path):
    for door, needle in (((0.0, "open"), "door is open"), (None, "not known to be shut")):
        p, rig_, screen, clock, bridge, session = plugin_with_bridge(module, tmp_path)
        p._door = door
        calls = fake_closeup(p)
        feed(session, bridge, clock, JOB)
        entry = settle(clock, bridge, p._burns_step)
        assert calls == [] and entry["closeup"]["status"] == "skipped" and needle in entry["closeup"]["message"]
    # a close-up already running stops when the door opens
    p, rig_, screen, clock, bridge, session = plugin_with_bridge(module, tmp_path)
    stopped = []
    p._burn_pending = dict(id="x", job=None, box=[0, 0, 10, 10], height=0.0)
    p._running = lambda: True
    p._stop_job = stopped.append
    p._door_seen("open")
    assert stopped == ["the enclosure door opened"]
