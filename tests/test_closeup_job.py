"""The close-up job: the moves and photos for a tile plan, then the head back up.

Driven through the real G-code bridge against the fake machine. Rules: M5 first; up to the
travel height before any XY move; descend only over the first tile; the head is where it was
sent before and after every photo; it always ends at the travel height, after success, abort or
error; the image is composed only once the head is up.
"""
import threading

import pytest

from helpers import Rig
from lhp_pkg import closeup_job as cj


def plan(tiles=((150.0, 160.0), (180.0, 160.0), (180.0, 190.0)), z=50.0, travel=290.0):
    return dict(head_z=z, travel_z=travel,
                tiles=[dict(i=i, x=x, y=y, z=z) for i, (x, y) in enumerate(tiles)])


class Camera(object):
    """The touchscreen: a photo of where the head is. fail = how many tries fail first;
    move = shift the head during a capture (as the touchscreen would if sent other X, Y, Z)."""

    def __init__(self, machine, fail=0, move=None, on_capture=None):
        self.machine, self.fail, self.move, self.on_capture = machine, fail, move, on_capture
        self.shots = []

    def __call__(self, x, y, z, sleep):
        if self.on_capture:
            self.on_capture(len(self.shots))
        if self.fail:
            self.fail -= 1
            raise ValueError("no new photo from the touchscreen after 30 s")
        m = self.machine
        if self.move:
            m.x += self.move
        self.shots.append(((x, y, z), (m.x, m.y, m.z)))
        return b"\xff\xd8photo %d\xff\xd9" % len(self.shots)


def run_job(rig, p, camera, compose=None, start_z=290.0):
    rig.machine.z = start_z
    said, stored, events = [], [], []

    def store(i, jpeg, pose):
        stored.append((i, jpeg, pose))
        events.append(("store", i, rig.machine.z))

    def composer():
        events.append(("compose", rig.machine.z))
        return compose() if compose else dict(qa=dict(seam_mm=0.03))

    job = cj.CloseupJob(rig.bridge, camera, said.append, p, store, composer,
                        dict(z_feed=1500, xy_feed=4000), retries=3, sleep=lambda s: None)
    job.run()
    return job, said, stored, events


def test_a_close_up_goes_up_across_down_then_photographs_each_tile_and_comes_back_up():
    rig = Rig()
    rig.machine.x, rig.machine.y = 232.0, 178.0
    cam = Camera(rig.machine)
    job, said, stored, events = run_job(rig, plan(), cam, start_z=120.0)
    sent = rig.machine.sent
    assert sent[0] == "M5"
    moves = [c for c in sent if c.startswith("G0")]
    assert moves[0].startswith("G0 Z290.0")                         # up first, from Z120
    assert moves[1].startswith("G0 X150.0 Y160.0") and "Z" not in moves[1]
    assert moves[2].startswith("G0 Z50.0")                          # down only over tile 1
    assert [s[0] for s in cam.shots] == [(150.0, 160.0, 50.0), (180.0, 160.0, 50.0), (180.0, 190.0, 50.0)]
    assert all(abs(a - b) < 1e-6 for s in cam.shots for a, b in zip(s[0], s[1]))
    assert [i for i, _, _ in stored] == [0, 1, 2]
    assert events[-1] == ("compose", 290.0)                         # composed with the head up
    assert rig.machine.z == 290.0
    assert said[-1]["type"] == "done" and said[-1]["result"]["qa"]["seam_mm"] == 0.03
    assert [p["tile"] for p in said if p["type"] == "progress"] == [1, 2, 3]
    for bad in ("G53", "G54", "M500", "G28", "M3", "M4"):
        assert not any(c.split()[0] == bad for c in sent), bad


def test_xy_moves_below_the_travel_height_only_go_between_tiles():
    rig = Rig()
    cam = Camera(rig.machine)
    run_job(rig, plan(), cam)
    z = 290.0
    tiles = {(150.0, 160.0), (180.0, 160.0), (180.0, 190.0)}
    for c in rig.machine.sent:
        words = dict((w[0], float(w[1:])) for w in c.split()[1:] if w[0] in "XYZ")
        if c.startswith("G0") and "Z" in words:
            z = words["Z"]
        if c.startswith("G0") and "X" in words and z < 200.0:
            assert (words["X"], words["Y"]) in tiles


def test_a_photo_that_fails_is_tried_again():
    rig = Rig()
    cam = Camera(rig.machine, fail=2)
    job, said, stored, _ = run_job(rig, plan(), cam)
    assert said[-1]["type"] == "done" and len(stored) == 3


def test_a_photo_that_keeps_failing_stops_the_job_with_the_head_up_and_nothing_composed():
    rig = Rig()
    cam = Camera(rig.machine, fail=99)
    job, said, stored, events = run_job(rig, plan(), cam)
    assert said[-1]["type"] == "failed" and "touchscreen" in said[-1]["message"]
    assert not any(e[0] == "compose" for e in events)
    assert rig.machine.z == 290.0


def test_the_touchscreen_moving_the_head_fails_the_tile():
    rig = Rig()
    cam = Camera(rig.machine, move=0.5)
    job, said, stored, events = run_job(rig, plan(), cam)
    assert said[-1]["type"] == "failed" and "moved" in said[-1]["message"]
    assert stored == [] and rig.machine.z == 290.0


def test_stop_during_the_photos_ends_with_the_head_up():
    rig = Rig()
    holder = {}
    cam = Camera(rig.machine, on_capture=lambda n: holder["job"].abort() if n == 1 else None)
    rig.machine.z = 290.0
    said, stored = [], []
    job = cj.CloseupJob(rig.bridge, cam, said.append, plan(), lambda i, j, p: stored.append(i),
                        lambda: dict(), dict(z_feed=1500, xy_feed=4000), sleep=lambda s: None)
    holder["job"] = job
    job.run()
    assert said[-1]["type"] == "aborted" and len(stored) == 1
    assert rig.machine.z == 290.0


def test_a_composing_error_is_reported_after_the_head_is_up():
    rig = Rig()

    def boom():
        raise ValueError("OpenCV is not installed")

    job, said, stored, events = run_job(rig, plan(), Camera(rig.machine), compose=boom)
    assert said[-1]["type"] == "failed" and "OpenCV" in said[-1]["message"]
    assert len(stored) == 3 and rig.machine.z == 290.0


def test_the_job_is_a_thread_with_the_routine_job_interface():
    rig = Rig()
    job = cj.CloseupJob(rig.bridge, Camera(rig.machine), lambda p: None, plan(), lambda *a: None,
                        lambda: {}, dict(z_feed=1500, xy_feed=4000), mode="closeup")
    assert isinstance(job, threading.Thread) and job.daemon
    assert job.mode == "closeup" and job.phase == "starting" and hasattr(job, "abort")


def test_calibration_poses_at_several_heights_go_up_before_moving_and_down_after():
    rig = Rig()
    poses = [dict(i=0, x=200.0, y=170.0, z=290.0), dict(i=1, x=210.0, y=170.0, z=120.0),
             dict(i=2, x=190.0, y=180.0, z=40.0), dict(i=3, x=215.0, y=165.0, z=40.0),
             dict(i=4, x=205.0, y=170.0, z=200.0)]
    p = dict(head_z=290.0, travel_z=290.0, tiles=poses)
    cam = Camera(rig.machine)
    job, said, stored, events = run_job(rig, p, cam)
    assert said[-1]["type"] == "done"
    assert [s[1] for s in cam.shots] == [(t["x"], t["y"], t["z"]) for t in poses]
    z = 290.0
    for c in rig.machine.sent:                                 # every XY move is at the higher of
        if not c.startswith("G0"):                             # the two poses' heights
            continue
        words = dict((w[0], float(w[1:])) for w in c.split()[1:] if w[0] in "XYZ")
        if "Z" in words:
            z = words["Z"]
        if "X" in words:
            target = [t for t in poses if (t["x"], t["y"]) == (words["X"], words["Y"])][0]
            assert z >= target["z"]
    assert rig.machine.z == 290.0


def test_every_photo_position_is_approached_from_the_same_side():
    # X backlash is about 0.42 mm (calibration, 2026-11-20): a photo reached moving -X lands
    # elsewhere than one reached moving +X. So every position is approached from -X, -Y.
    rig = Rig()
    poses = [dict(i=0, x=200.0, y=170.0, z=60.0), dict(i=1, x=180.0, y=170.0, z=60.0),
             dict(i=2, x=180.0, y=150.0, z=40.0), dict(i=3, x=210.0, y=190.0, z=80.0)]
    p = dict(head_z=60.0, travel_z=290.0, tiles=poses)
    cam = Camera(rig.machine)
    rig.machine.z = 290.0
    said = []
    job = cj.CloseupJob(rig.bridge, cam, said.append, p, lambda *a: None, lambda: {},
                        dict(z_feed=1500, xy_feed=4000, approach_mm=2.0), sleep=lambda s: None)
    job.run()
    assert said[-1]["type"] == "done"
    xy = [c for c in rig.machine.sent if c.startswith("G0") and " X" in c]
    for t in poses:
        i = max(n for n, c in enumerate(xy) if c.startswith("G0 X%.1f Y%.1f" % (t["x"], t["y"])))
        assert xy[i - 1].startswith("G0 X%.1f Y%.1f" % (t["x"] - 2.0, t["y"] - 2.0)), (t, xy[i - 1])
    assert [s[1] for s in cam.shots] == [(t["x"], t["y"], t["z"]) for t in poses]


def test_with_approach_moves_low_xy_moves_still_only_go_between_tiles():
    rig = Rig()
    tiles = [(150.0, 160.0), (180.0, 160.0), (180.0, 190.0)]
    p = plan(tiles)
    rig.machine.z = 290.0
    job = cj.CloseupJob(rig.bridge, Camera(rig.machine), lambda m: None, p, lambda *a: None, lambda: {},
                        dict(z_feed=1500, xy_feed=4000, approach_mm=2.0), sleep=lambda s: None)
    job.run()
    allowed = set(tiles) | set((x - 2.0, y - 2.0) for x, y in tiles)
    z = 290.0
    for c in rig.machine.sent:
        words = dict((w[0], float(w[1:])) for w in c.split()[1:] if w[0] in "XYZ")
        if c.startswith("G0") and "Z" in words:
            z = words["Z"]
        if c.startswith("G0") and "X" in words and z < 200.0:
            assert (words["X"], words["Y"]) in allowed, c
