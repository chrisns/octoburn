"""The rotary routine: runout, axis and ends of a round object in the chuck."""
import math

import pytest

from lhp_pkg import rotary, rotary_job
from lhp_pkg.routine import Job, TapSession
from helpers import Rig, is_step

TRAVEL = 2.5          # the fake probe trips this far above the surface


def test_known_radius_fit_finds_the_axis():
    zc, xc, r = 103.2, 164.3, 39.0
    pts = [(x, zc + math.sqrt(r * r - (x - xc) ** 2)) for x in (150, 157, 164, 171, 178)]
    f = rotary.fit_circle(pts, r)
    assert f["xc"] == pytest.approx(xc, abs=1e-6) and f["zc"] == pytest.approx(zc, abs=1e-6)
    g = rotary.fit_circle_free(pts)
    assert g["diameter"] == pytest.approx(78.0, abs=1e-6)


def test_runout_fit():
    s = [(b, 142.0 + 0.2 * math.cos(math.radians(b - 30))) for b in (0, 90, 180, 270)]
    f = rotary.fit_runout(s)
    assert f["e"] == pytest.approx(0.2, abs=1e-9) and f["phase_deg"] == pytest.approx(30, abs=1e-6)


def test_axis_line():
    line = rotary.axis_line([(80, 164.0, 103.0), (180, 165.0, 102.5)])
    assert 100 * line["dx_dy"] == pytest.approx(1.0) and 100 * line["dz_dy"] == pytest.approx(-0.5)


def test_the_head_never_reaches_the_object_beside_it():
    # pin 50 mm off the axis of a 39 mm radius cylinder (a miss): the plate reaches over the top
    floor = rotary.lowest_safe_trigger(142.0, 39.0, 50.0)
    assert floor == pytest.approx(142.0 - 24.0)
    # on the top, the body clears a slight curve: the floor is just under the pin stroke
    assert rotary.lowest_safe_trigger(142.0, 39.0, 0.0) < 142.0 - 10.0


def flask(machine, e=0.2, phase=30.0):
    """A 78 mm flask: axis X 164.3 at Y117, off square by 1 mm per 100 mm, Z falling 0.5 per
    100; body Y 35 to 210 with a 3 mm round-over at the free end; shoulder to r 20 by Y 240."""
    def surface(x, y):
        xc = 164.3 + 0.01 * (y - 117)
        zc = 103.2 - 0.005 * (y - 117) + e * math.cos(math.radians(machine.b - phase))
        if y < 35.0:
            return -100.0
        if y < 38.0:
            r = 36.0 + math.sqrt(max(0.0, 9.0 - (38.0 - y) ** 2))
        elif y <= 210.0:
            r = 39.0
        elif y <= 240.0:
            r = 39.0 - 19.0 * (y - 210.0) / 30.0
        else:
            r = 20.0
        d = x - xc
        return zc + math.sqrt(r * r - d * d) if abs(d) < r else -100.0
    return surface


def run_rotary(params, e=0.2, phase=30.0, **settings):
    rig = Rig(settings=dict(dict(z_floor=0.5, safe_z=200.0), **settings))
    rig.device.surface = flask(rig.machine, e=e, phase=phase)
    rig.machine.z = 160.0
    session = TapSession()
    said = []

    def say(payload):
        said.append(payload)
        if payload and payload.get("waiting_for_tap"):
            rig.device.tap()
    Job(rig.bridge, rig.esp, say, rig.config, [], mode="tap", clock=rig.clock, session=session).run()
    assert session.ok
    p = rotary_job.params(params)
    config = dict(rotary_job.probe_config(rig.config, p), rotary=p)
    job = Job(rig.bridge, rig.esp, say, config, [], mode="rotary", clock=rig.clock, session=session)
    job.run()
    return rig, job, said


def test_the_rotary_routine_measures_the_flask():
    params = dict(axis_x=162.0, crown=142.1 + TRAVEL, stations=[80.0, 180.0], y_chuck_limit=250.0, diameter=78.0,
                  preset="precise")
    rig, job, said = run_rotary(params)
    assert job.phase == "done", job.error
    s = job.result["summary"]
    # the axis at the mean station Y (130): X 164.3 + 0.13, Z 103.2 - 0.065 (+ trigger travel)
    assert s["axis_x"] == pytest.approx(164.43, abs=0.05)
    assert s["axis_x_per_100mm"] == pytest.approx(1.0, abs=0.1)
    assert s["axis_z_per_100mm"] == pytest.approx(-0.5, abs=0.1)
    assert s["diameter_free_mean"] == pytest.approx(78.0, abs=0.5)
    # runout: 0.2 eccentricity reads about 0.4 mm top to bottom
    assert 0.25 < s["runout_max_spread"] <= 0.41
    assert job.result["runout"][0]["e"] == pytest.approx(0.2, abs=0.03)
    # free end: full diameter to within the resolution of the round-over start
    assert 36.5 <= s["body_from_y"] <= 39.5
    assert 209.0 <= s["body_to_y"] <= 211.5
    assert job.result["shoulder"]["profile"], "the shoulder was profiled"
    assert rig.machine.b == pytest.approx(0.0)                     # B back to 0
    # never past the chuck limit, and B only turned with the pin up at travel height
    assert max(t["y"] for t in job.result["touches"]) <= 250.0


def test_a_swing_within_the_tolerance_is_reported_as_accepted():
    # Chris, 2026-11-19: a flask is never perfect; under a 3 mm swing per turn is within the
    # laser's focal range (and the bridge follows the runout anyway): accept it and say so.
    params = dict(axis_x=162.0, crown=142.1 + TRAVEL + 1.2 * math.cos(math.radians(-30.0)),
                  stations=[80.0, 180.0], y_chuck_limit=250.0, diameter=78.0)
    rig, job, said = run_rotary(params, e=1.2)
    assert job.phase == "done", job.error
    s = job.result["summary"]
    assert s["runout_tolerance"] == pytest.approx(3.0)
    assert s["runout_swing"] == pytest.approx(2.4, abs=0.1)
    assert s["runout_within"] is True
    assert [y for y, _ in s["runout_swing_by_y"]] == [80.0, 180.0]
    assert not any("swing" in w for w in s["warnings"])


def test_an_off_centre_object_is_measured_and_a_swing_over_the_tolerance_warns():
    # 2026-11-19 09:00: the flask gripped by the inside of its mouth swung 4.8 mm. The second B
    # angle came up 3 mm past the B0 crown and the run stopped "within 1 mm of the travel
    # height", because the first turn allowed for no eccentricity. Travel now allows the
    # tolerance from the first turn, and a swing past it is measured and reported instead.
    e, phase = 2.2, 135.0
    params = dict(axis_x=162.0, crown=142.1 + TRAVEL + e * math.cos(math.radians(-phase)),
                  stations=[80.0, 180.0], y_chuck_limit=250.0, diameter=78.0)
    rig, job, said = run_rotary(params, e=e, phase=phase)
    assert job.phase == "done", job.error
    s = job.result["summary"]
    assert s["runout_swing"] == pytest.approx(2 * e, abs=0.1)
    assert s["runout_within"] is False
    assert any("swings 4.4 mm" in w and "3.0 mm" in w for w in s["warnings"]), s["warnings"]


def test_end_guesses_place_the_stations_well_inside_them():
    # Chris, 2026-11-19: nobody knows exactly where the flask sits in the chuck, so the ends are
    # guesses, like the diameter: the stations go inside them and the walks find the real ends.
    base = dict(axis_x=164.3, crown=144.6, y_chuck_limit=245.0, diameter=78.0)
    p = rotary_job.params(dict(base, free_end_guess=30.0, chuck_end_guess=215.0))
    assert p["stations"] == [50.0, 122.5, 195.0]
    assert p["runout_ys"] == [50.0, 195.0]
    short = rotary_job.params(dict(base, free_end_guess=30.0, chuck_end_guess=70.0))
    assert short["stations"] == [40.0, 50.0, 60.0]                 # a quarter of the length in
    with pytest.raises(rotary_job.RotaryError, match="at least 20 mm"):
        rotary_job.params(dict(base, free_end_guess=30.0, chuck_end_guess=45.0))
    with pytest.raises(rotary_job.RotaryError, match="never past"):
        rotary_job.params(dict(base, free_end_guess=30.0, chuck_end_guess=250.0))


def test_the_found_ends_are_checked_against_the_guesses():
    # The flask's full diameter runs Y38 to Y210. A free end guessed at Y30 is close enough; a
    # chuck end guessed at Y190 is 20 mm out and says so.
    params = dict(axis_x=162.0, crown=142.1 + TRAVEL, free_end_guess=30.0, chuck_end_guess=190.0,
                  y_chuck_limit=250.0, diameter=78.0)
    rig, job, said = run_rotary(params)
    assert job.phase == "done", job.error
    s = job.result["summary"]
    assert s["free_end_guess"] == 30.0 and s["chuck_end_guess"] == 190.0
    assert 36.5 <= s["body_from_y"] <= 40.0 and 208.0 <= s["body_to_y"] <= 211.5
    ends = [w for w in s["warnings"] if "guess" in w]
    assert len(ends) == 1 and "near the chuck" in ends[0] and "Y190" in ends[0], ends


def test_an_end_past_the_limit_is_touched_at_the_limit_and_reported_as_not_found():
    # 2026-11-19 15:22: the flask went 20 mm further into the chuck after a remount. The walk
    # stopped at Y235 (the next 20 mm step passed the Y245 limit) and the end was reported as
    # "at Y235, 20 mm from the guess". The walk now touches the limit itself, and an end it did
    # not find is said to be past the limit, not compared with the guess.
    params = dict(axis_x=162.0, crown=142.1 + TRAVEL, free_end_guess=30.0, chuck_end_guess=180.0,
                  y_chuck_limit=195.0, diameter=78.0)
    rig, job, said = run_rotary(params)
    assert job.phase == "done", job.error
    s = job.result["summary"]
    assert max(t["y"] for t in job.result["touches"]) == pytest.approx(195.0)
    assert s["body_to_y"] == pytest.approx(195.0)
    near = [w for w in s["warnings"] if "chuck" in w]
    assert len(near) == 1 and "not found" in near[0] and "Y195" in near[0], near
    assert not any("guessed" in w and "chuck" in w for w in s["warnings"])


def test_the_rotary_body_limit_is_a_head_position_for_every_module():
    # Chris, 2026-11-19: objects can sit over the chuck, so the limit is the rotary body: he
    # jogged the head to the nearest an object could be (head Y293.4). The touches are laser
    # positions; the head goes to y - dy, so the limit in touch Y is the head limit + dy.
    base = dict(axis_x=164.3, crown=144.6, diameter=78.0, free_end_guess=30.0, chuck_end_guess=215.0)
    with pytest.raises(rotary_job.RotaryError, match="y_chuck_limit"):
        rotary_job.params(base)
    p = rotary_job.params(dict(base, head_y_limit=293.4))

    class Pr(object):
        _dy = 3.6
    assert rotary_job.Rotary(Pr(), p, tell=lambda t: None).y_limit == pytest.approx(297.0)
    both = rotary_job.params(dict(base, head_y_limit=293.4, y_chuck_limit=250.0))
    assert rotary_job.Rotary(Pr(), both, tell=lambda t: None).y_limit == pytest.approx(250.0)


def test_the_ends_walk_stops_at_the_head_limit():
    params = dict(axis_x=162.0, crown=142.1 + TRAVEL, free_end_guess=30.0, chuck_end_guess=180.0,
                  head_y_limit=195.0, diameter=78.0)
    rig, job, said = run_rotary(params)
    assert job.phase == "done", job.error
    assert max(t["y"] for t in job.result["touches"]) == pytest.approx(195.0)
    assert any("not found" in w and "Y195" in w for w in job.result["summary"]["warnings"])


def test_a_chuck_end_guess_past_the_head_limit_is_refused():
    params = dict(axis_x=162.0, crown=142.1 + TRAVEL, free_end_guess=30.0, chuck_end_guess=200.0,
                  head_y_limit=195.0, diameter=78.0)
    rig, job, said = run_rotary(params)
    assert job.phase == "failed" and "past" in job.error


def test_after_homing_the_head_drops_to_travel_instead_of_stepping_down_from_the_top():
    # 2026-11-18: a job started with the head parked at the top (Z313 after G28 on the A350)
    # stepped every touch down from there 0.5 mm at a time, then went back up: 2.5 min a touch.
    params = dict(REF, ref_offset=OFFSET, ref_slope=-0.005, max_diameter=80.0, ref_radius=31.0,
                  steps=["reference", "runout", "stations", "free_end", "shoulder"], preset="precise")
    rig, job, said = run_reference(params, start_z=313.0)
    assert job.phase == "done", job.error
    p = rotary_job.params(params)
    highest_travel = max(160.0, p["crown"] + 15.0)                   # the housing move
    steps = [float(c.split()[1][1:]) for c in rig.machine.sent if is_step(c)]
    assert steps and max(steps) <= highest_travel
    drops = [c for c in rig.machine.sent if c.startswith("G0 Z") and not is_step(c)
             and float(c.split()[1][1:]) < 300.0]
    assert drops, "the head came down from the top in one move"


def test_a_crown_typed_too_low_stops_before_anything_travels_at_that_height():
    # travel = crown + clearance is trusted as clear air (probe_at drops to it). An object that
    # reaches within a millimetre of it means the crown typed in is wrong: stop at the first touch.
    params = dict(axis_x=164.3, crown=142.1 + TRAVEL - 3.5, stations=[80.0, 180.0], y_chuck_limit=250.0,
                  diameter=78.0)
    rig, job, said = run_rotary(params)
    assert job.phase == "failed"
    assert "crown" in job.error and "too low" in job.error
    xy_moves = [c for c in rig.machine.sent if c.startswith("G0 X")]
    assert len(xy_moves) <= 2                                   # stopped at the first point


def test_the_first_object_touch_of_a_run_steps_down_and_only_later_ones_drop():
    # A crown from an earlier object (or typed wrong) must not drive the head into a taller one:
    # the first touch on the object probes its way down; drops follow once it has confirmed the top.
    params = dict(REF, ref_offset=OFFSET, ref_slope=-0.005, steps=["reference", "runout", "stations"])
    rig, job, said = run_reference(params, start_z=313.0)
    assert job.phase == "done", job.error
    sent = rig.machine.sent
    housing = [i for i, c in enumerate(sent) if c.startswith("G0 X") and " Y322.500" in c]
    first_object_xy = next(i for i, c in enumerate(sent) if c.startswith("G0 X") and i > max(housing))
    first_trigger = next(i for i, c in enumerate(sent) if i > first_object_xy and is_step(c)
                         and float(c.split()[1][1:]) < 150.0)
    blind = [c for c in sent[first_object_xy:first_trigger] if c.startswith("G0 Z") and not is_step(c)
             and float(c.split()[1][1:]) < 160.0]
    assert blind == []
    later = [c for c in sent[first_trigger:] if c.startswith("G0 Z") and not is_step(c)
             and float(c.split()[1][1:]) < 160.0]
    assert later, "later touches drop to travel"


def test_b_only_turns_at_travel_height():
    params = dict(axis_x=162.0, crown=142.1 + TRAVEL, stations=[80.0, 180.0], y_chuck_limit=250.0, diameter=78.0)
    rig = None
    rig_z = []
    rig_holder = {}

    def hook(machine, command):
        if command.startswith("G0") and " B" in command:
            rig_z.append(machine.z)
    r = Rig(settings=dict(z_floor=0.5, safe_z=200.0))
    r.machine.hooks.append(hook)
    r.device.surface = flask(r.machine)
    r.machine.z = 160.0
    session = TapSession()

    def say(payload):
        if payload and payload.get("waiting_for_tap"):
            r.device.tap()
    Job(r.bridge, r.esp, say, r.config, [], mode="tap", clock=r.clock, session=session).run()
    job = Job(r.bridge, r.esp, say, dict(r.config, rotary=rotary_job.params(params)), [], mode="rotary",
              clock=r.clock, session=session)
    job.run()
    assert job.phase == "done", job.error
    assert rig_z and min(rig_z) >= 142.1 + TRAVEL + 4.0 - 1e-6


def test_params_refuse_a_missing_chuck_limit_and_a_steep_chord():
    with pytest.raises(rotary_job.RotaryError, match="y_chuck_limit"):
        rotary_job.params(dict(axis_x=1, crown=1, stations=[1], diameter=78.0))
    with pytest.raises(rotary_job.RotaryError, match="chord_half"):
        rotary_job.params(dict(axis_x=1, crown=1, stations=[1], y_chuck_limit=5, chord_half=30, diameter=78.0))


def test_a_failure_keeps_the_partial_result_and_a_resume_runs_only_the_shoulder():
    params = dict(axis_x=162.0, crown=142.1 + TRAVEL, stations=[80.0, 180.0], y_chuck_limit=250.0, diameter=78.0)
    rig = Rig(settings=dict(z_floor=0.5, safe_z=200.0))
    surface = flask(rig.machine)
    broken = dict(on=True)
    rig.device.surface = lambda x, y: (-100.0 if broken["on"] and y > 205 else surface(x, y))
    rig.machine.z = 160.0
    session = TapSession()

    def say(payload):
        if payload and payload.get("waiting_for_tap"):
            rig.device.tap()
    Job(rig.bridge, rig.esp, say, rig.config, [], mode="tap", clock=rig.clock, session=session).run()
    # the shoulder sees "nothing" past Y205: it stops there, so make it fail by a bad limit
    p = rotary_job.params(params)
    job = Job(rig.bridge, rig.esp, say, dict(rig.config, rotary=p), [], mode="rotary", clock=rig.clock,
              session=session)
    job.run()
    first = job.result
    assert first["stations"] and first["free_end"] and first["runout"]
    broken["on"] = False
    p2 = rotary_job.params(dict(params, resume=first, steps=["shoulder"]))
    before = len(first["touches"])
    job2 = Job(rig.bridge, rig.esp, say, dict(rig.config, rotary=p2), [], mode="rotary", clock=rig.clock,
               session=session)
    job2.run()
    assert job2.phase == "done", job2.error
    r = job2.result
    assert r["complete"] and r["stations"] == first["stations"] and r["runout"] == first["runout"]
    assert 209.0 <= r["summary"]["body_to_y"] <= 211.5
    new = r["touches"][before:]
    assert new and min(t["y"] for t in new) > 180.0          # only shoulder touches were added


def test_a_failed_rotary_job_returns_what_it_measured():
    params = dict(axis_x=162.0, crown=142.1 + TRAVEL, stations=[80.0, 180.0], y_chuck_limit=250.0, diameter=78.0,
                  runout_ys=[80.0])
    rig = Rig(settings=dict(z_floor=0.5, safe_z=200.0))
    surface = flask(rig.machine)
    rig.device.surface = lambda x, y: surface(x, y) - (5.0 if y > 150 else 0.0)   # station 2 is off
    rig.machine.z = 160.0
    session = TapSession()

    def say(payload):
        if payload and payload.get("waiting_for_tap"):
            rig.device.tap()
    Job(rig.bridge, rig.esp, say, rig.config, [], mode="tap", clock=rig.clock, session=session).run()
    job = Job(rig.bridge, rig.esp, say, dict(rig.config, rotary=rotary_job.params(params)), [], mode="rotary",
              clock=rig.clock, session=session)
    job.run()
    assert job.phase == "failed"
    assert job.result and job.result["runout"] and len(job.result["stations"]) == 1
    assert job.result["complete"] is False


def test_fast_preset_measures_the_same_flask_with_far_fewer_probe_steps():
    params = dict(axis_x=162.0, crown=142.1 + TRAVEL, stations=[80.0, 180.0], y_chuck_limit=250.0, diameter=78.0,
                  preset="precise")
    real = "0.5; 0.1, 1.5; 0.02, 1.0"                      # the shipped default stages
    rig_p, job_p, _ = run_rotary(params, stages=real, home_first=False)       # probe steps only
    rig_f, job_f, _ = run_rotary(dict(params, preset="fast"), stages=real, home_first=False)
    assert job_f.phase == "done", job_f.error
    sp, sf = job_p.result["summary"], job_f.result["summary"]
    # focus needs the top, the axis X and the body ends: fast agrees with precise well inside
    # any laser's depth of focus
    assert sf["top_z"] == pytest.approx(sp["top_z"], abs=0.15)
    assert sf["axis_x"] == pytest.approx(sp["axis_x"], abs=0.3)
    assert abs(sf["body_from_y"] - sp["body_from_y"]) <= 2.5
    assert abs(sf["body_to_y"] - sp["body_to_y"]) <= 2.5
    assert job_f.result["params"]["preset"] == "fast"
    # one touch per point at 0.1 mm instead of 0.02 mm with repeats
    moves = lambda rig: sum(1 for c in rig.machine.sent if c.startswith("G1") or c.startswith("G0"))
    assert moves(rig_f) < 0.35 * moves(rig_p), (moves(rig_f), moves(rig_p))


def test_fast_preset_and_probe_override_are_checked():
    base = dict(axis_x=1, crown=1, stations=[1], y_chuck_limit=5, diameter=78.0)
    p = rotary_job.params(dict(base, preset="fast"))
    assert p["chord_n"] == 4 and p["end_resolution"] == 2.0
    assert rotary_job.params(base)["preset"] == "fast"                # the default
    assert p["probe"]["samples"] == 1
    c = rotary_job.probe_config(dict(stages=[(0.5, None), (0.1, 1.5), (0.02, 1.0)], samples=2,
                                     max_samples=4, tolerance=0.05), p)
    assert c["stages"] == [(0.5, None), (0.1, 1.2)] and c["samples"] == 1 and c["max_samples"] == 1
    # a user value beats the preset
    assert rotary_job.params(dict(base, preset="fast", chord_n=5))["chord_n"] == 5
    with pytest.raises(rotary_job.RotaryError, match="preset"):
        rotary_job.params(dict(base, preset="ludicrous"))
    with pytest.raises(rotary_job.RotaryError, match="stages"):
        rotary_job.params(dict(base, probe=dict(stages="0.1; 0.5, 1")))
    with pytest.raises(rotary_job.RotaryError, match="max_samples"):
        rotary_job.params(dict(base, probe=dict(stages="0.5; 0.1, 1.2", samples=3, max_samples=1)))


def test_a_bad_touch_fails_the_station_fit():
    params = dict(axis_x=162.0, crown=142.1 + TRAVEL, stations=[80.0, 180.0], y_chuck_limit=250.0, diameter=78.0,
                  preset="fast", runout_ys=[80.0])
    rig = Rig(settings=dict(z_floor=0.5, safe_z=200.0))
    good = flask(rig.machine)

    def bumpy(x, y):                      # a 1.5 mm burr on one chord point at the first station
        return good(x, y) + (1.5 if abs(y - 80.0) < 0.1 and abs(x - 167.0) < 0.6 else 0.0)
    rig.device.surface = bumpy
    rig.machine.z = 160.0
    session = TapSession()

    def say(payload):
        if payload and payload.get("waiting_for_tap"):
            rig.device.tap()
    Job(rig.bridge, rig.esp, say, rig.config, [], mode="tap", clock=rig.clock, session=session).run()
    p = rotary_job.params(params)
    job = Job(rig.bridge, rig.esp, say, dict(rotary_job.probe_config(rig.config, p), rotary=p), [],
              mode="rotary", clock=rig.clock, session=session)
    job.run()
    assert job.phase == "failed"
    assert "do not fit" in job.error


@pytest.mark.parametrize("shift,xy", [(11.0, (0.0, 0.0)), (0.0, (2.0, 0.0))])
def test_the_rotary_routine_refuses_a_work_offset(shift, xy):
    """LightBurn's flat 10W start code leaves G92 Z0 at machine Z11 and G92 X0 Y0 at X2: a rotary
    result probed in that frame and burnt in G53 would put the hood off by the shift."""
    rig = Rig(settings=dict(z_floor=0.5, safe_z=200.0), shift=shift)
    rig.machine.xy_shift = xy
    rig.device.surface = flask(rig.machine)
    rig.machine.z = 160.0
    session = TapSession()

    def say(payload):
        if payload and payload.get("waiting_for_tap"):
            rig.device.tap()
    Job(rig.bridge, rig.esp, say, rig.config, [], mode="tap", clock=rig.clock, session=session).run()
    p = rotary_job.params(dict(axis_x=162.0, crown=142.1 + TRAVEL, stations=[80.0, 180.0],
                               y_chuck_limit=250.0, diameter=78.0))
    job = Job(rig.bridge, rig.esp, say, dict(rotary_job.probe_config(rig.config, p), rotary=p), [],
              mode="rotary", clock=rig.clock, session=session)
    job.run()
    assert job.phase == "failed"
    assert "work offset" in job.error and "G28 X Y Z" in job.error
    assert not job.result or not job.result.get("touches")



def test_the_laser_heads_home_offset_is_not_a_work_offset():
    """Laser mode has M206 X-19 Y-10, so M114 and the step counters differ by 19 and 10 with no
    G92 at all (seen on the A350, 2026-11-17). That must not be refused."""
    rig = Rig(settings=dict(z_floor=0.5, safe_z=200.0))
    rig.machine.home_offset = (-19.0, -10.0, 0.0)
    rig.machine.xy_shift = (19.0, 10.0)
    rig.device.surface = flask(rig.machine)
    rig.machine.z = 160.0
    session = TapSession()

    def say(payload):
        if payload and payload.get("waiting_for_tap"):
            rig.device.tap()
    Job(rig.bridge, rig.esp, say, rig.config, [], mode="tap", clock=rig.clock, session=session).run()
    p = rotary_job.params(dict(axis_x=162.0, crown=142.1 + TRAVEL, stations=[80.0, 180.0],
                               y_chuck_limit=250.0, diameter=78.0, steps=["runout"], runout_ys=[80.0]))
    job = Job(rig.bridge, rig.esp, say, dict(rotary_job.probe_config(rig.config, p), rotary=p), [],
              mode="rotary", clock=rig.clock, session=session)
    job.run()
    assert "work offset" not in (job.error or "")


HOUSING = 28.7        # housing top above the spin axis, as the fake builds it
# The reference touches are 5.5 mm off the housing's crown (r 31), so they sit this much lower:
# the offset is touch - axis, as calibrated, not crown - axis.
OFFSET = HOUSING - (31.0 - math.sqrt(31.0 ** 2 - 5.5 ** 2))


def flask_and_housing(machine, e=0.2, dip=0.5, housing_x=164.8):
    """The flask, plus the rotary's motor housing past Y318: a cylinder about the spin axis
    with a seam dip along its top, reachable only past the soft Y limit."""
    body = flask(machine, e=e)

    def surface(x, y):
        if y >= 318.0:
            zc = 103.2 - 0.005 * (y - 117)
            r = 31.0
            d = x - housing_x
            if abs(d) >= r:
                return -100.0
            top = zc + HOUSING - r + math.sqrt(r * r - d * d)
            return top - dip if abs(d) < 1.5 else top
        return body(x, y)
    return surface


def run_reference(params, xy_max=(320.0, 320.0), start_z=160.0, housing_x=164.8, hooks=(), home_first=False):
    rig = Rig(settings=dict(z_floor=0.5, safe_z=200.0, home_first=home_first))
    rig.machine.xy_max = xy_max
    rig.machine.hooks.extend(hooks)
    rig.device.surface = flask_and_housing(rig.machine, housing_x=housing_x)
    rig.machine.z = start_z
    session = TapSession()
    said = []

    def say(payload):
        said.append(payload)
        if payload and payload.get("waiting_for_tap"):
            rig.device.tap()
    Job(rig.bridge, rig.esp, say, rig.config, [], mode="tap", clock=rig.clock, session=session).run()
    p = rotary_job.params(params)
    config = dict(rotary_job.probe_config(rig.config, p), rotary=p)
    job = Job(rig.bridge, rig.esp, say, config, [], mode="rotary", clock=rig.clock, session=session)
    job.run()
    return rig, job, said


REF = dict(axis_x=164.3, crown=142.1 + TRAVEL, stations=[80.0, 180.0], runout_ys=[60.0, 200.0],
           y_chuck_limit=250.0, ref_expected=103.2 - 0.005 * 205.5 + HOUSING + TRAVEL,
           steps=["reference", "runout", "stations"])


def test_the_housing_reference_measures_the_diameter_past_the_soft_limit():
    rig, job, said = run_reference(dict(REF, ref_offset=OFFSET, ref_slope=-0.005))
    assert job.error is None, job.error
    ref = job.result["reference"]
    assert ref["spread"] < 0.05                      # off the seam dip
    assert ref["diameter"] == pytest.approx(78.0, abs=0.1)
    assert job.result["summary"]["diameter"] == pytest.approx(78.0, abs=0.1)
    assert job.result["summary"]["diameter_set"] is None
    sent = rig.machine.sent
    off, on = sent.index("M211 S0"), max(i for i, c in enumerate(sent) if c == "M211 S1")
    assert off < on and "G28 Y" in sent[on:]          # soft limit back on, then Y homed again
    assert rig.machine.soft_endstops


def test_the_housing_touches_stay_low_and_the_head_lifts_before_leaving_the_housing():
    # 2026-11-18 (Chris): each housing touch climbed to Z160 and stepped 26 mm back down. The
    # high travel is for crossing the chuck; on the housing the head stays clearance above it.
    params = dict(REF, ref_offset=OFFSET, ref_slope=-0.005)
    p = rotary_job.params(params)
    cross = max(160.0, p["crown"] + 15.0)
    low = p["ref_expected"] + p["clearance"] + 0.5           # within one coarse step above
    zs_at_rehome = []

    def hook(machine, command):
        if command.startswith("G28"):
            zs_at_rehome.append(machine.z)
    rig, job, said = run_reference(params, start_z=313.0, hooks=[hook])
    assert job.error is None, job.error
    sent = rig.machine.sent
    rehome = max(i for i, c in enumerate(sent) if c == "G28 Y")      # the one after the housing
    housing_xy = [i for i, c in enumerate(sent[:rehome]) if c.startswith("G0 X") and "Y322.500" in c]
    assert len(housing_xy) == 2 * p["ref_repeats"]
    zs = [float(c.split()[1][1:]) for c in sent[housing_xy[1]:rehome] if c.startswith("G0 Z")]
    assert zs[-1] >= cross - 1e-6                             # the last Z move: up to cross the chuck
    assert max(zs[:-1]) <= low + 1e-6                         # the later housing touches stay low
    assert zs_at_rehome and zs_at_rehome[-1] >= cross - 1e-6   # up to the chuck crossing before leaving


def test_the_low_housing_travel_follows_the_first_touch_not_the_expected_height():
    # The housing moves a few mm between remounts (Chris, 2026-11-18): 3 mm higher than expected,
    # the later touches must still start clearance above where the first touch found it.
    params = dict(REF, ref_offset=OFFSET, ref_slope=-0.005)
    params["ref_expected"] = REF["ref_expected"] - 3.0
    p = rotary_job.params(params)
    rig, job, said = run_reference(params, start_z=313.0)
    assert job.error is None, job.error
    found = job.result["reference"]["touches"][0][1]
    sent = rig.machine.sent
    rehome = max(i for i, c in enumerate(sent) if c == "G28 Y")      # the one after the housing
    housing_xy = [i for i, c in enumerate(sent[:rehome]) if c.startswith("G0 X") and "Y322.500" in c]
    starts = [float(next(c for c in sent[i:] if c.startswith("G0 Z")).split()[1][1:]) for i in housing_xy[1:]]
    assert min(starts) >= found + p["clearance"] - 0.5 - 1e-6   # first step: one coarse step below travel


def test_a_housing_moved_sideways_is_re_centred_and_reads_the_same():
    # The housing moves a few mm between remounts (Chris, 2026-11-18). The touch pair either side
    # of the seam disagrees by about 0.36 mm per mm of shift: move over by the shift, touch again.
    base = dict(REF, ref_offset=OFFSET, ref_slope=-0.005, ref_radius=31.0)
    _, job0, _ = run_reference(base)
    _, job, _ = run_reference(base, housing_x=167.8)
    assert job.error is None, job.error
    ref, ref0 = job.result["reference"], job0.result["reference"]
    assert ref["centre_x"] == pytest.approx(167.8, abs=0.3)
    assert ref["z"] == pytest.approx(ref0["z"], abs=0.05)
    assert ref["spread"] <= 0.3


def test_without_a_max_diameter_the_head_leaves_the_housing_as_high_as_it_came():
    # Until the object's top is confirmed, nothing says Z160 clears it: cross back as high as
    # the head arrived (Z313 here), not at the chuck crossing height.
    zs = []

    def hook(machine, command):
        if command == "G28 Y":
            zs.append(machine.z)
    rig, job, _ = run_reference(dict(REF, ref_offset=OFFSET, ref_slope=-0.005, steps=["reference", "runout"]),
                                start_z=313.0, hooks=[hook])
    assert zs and zs[-1] == pytest.approx(313.0)


def test_a_max_diameter_bounds_the_crossing_and_lets_the_first_touch_drop():
    zs = []

    def hook(machine, command):
        if command == "G28 Y":
            zs.append(machine.z)
    params = dict(REF, ref_offset=OFFSET, ref_slope=-0.005, max_diameter=82.0)
    rig, job, _ = run_reference(params, start_z=313.0, hooks=[hook])
    assert job.error is None, job.error
    p = rotary_job.params(params)
    ref = job.result["reference"]
    axis_hi = max(ref["z"] - OFFSET - 0.005 * (y - ref["y"]) for y in p["runout_ys"])
    bound = axis_hi + 41.0 + p["runout_tolerance"] / 2.0 + p["clearance"]   # an off-centre object reaches higher
    assert zs[-1] == pytest.approx(max(160.0, p["crown"] + 15.0, bound), abs=0.01)
    sent = rig.machine.sent
    rehome = max(i for i, c in enumerate(sent) if c == "G28 Y")      # the one after the housing
    first_xy = next(i for i, c in enumerate(sent) if i > rehome and c.startswith("G0 X"))
    drop = next(c for c in sent[first_xy:] if c.startswith("G0 Z"))
    assert not is_step(drop) and float(drop.split()[1][1:]) == pytest.approx(bound, abs=0.01)


def test_a_crown_above_the_max_diameter_is_refused():
    params = dict(REF, ref_offset=OFFSET, ref_slope=-0.005, max_diameter=60.0, steps=["reference", "runout"])
    _, job, _ = run_reference(params)
    assert "max_diameter" in (job.error or "")


def test_the_endstops_come_back_even_when_the_lift_off_the_housing_fails():
    from lhp_pkg.probe import Prober, ProbeError
    real = Prober.lift_to
    calls = []

    def lift(self, z):
        calls.append(z)
        if len(calls) == 2:
            raise ProbeError("stow failed")
        return real(self, z)
    Prober.lift_to = lift
    try:
        rig, job, _ = run_reference(dict(REF, ref_offset=OFFSET, ref_slope=-0.005, steps=["reference"]))
    finally:
        Prober.lift_to = real
    assert "stow failed" in (job.error or "")
    assert rig.machine.soft_endstops and "G28 Y" in rig.machine.sent


def test_a_housing_far_from_the_calibrations_warns_that_the_probe_moved():
    # 2026-11-19: every trigger dropped 3.0 mm (the probe pushed up in its mount). The diameter
    # survives (same-run reference) but focus heights calibrated before it do not: say so.
    _, job0, _ = run_reference(dict(REF, ref_offset=OFFSET, ref_slope=-0.005))
    z = job0.result["reference"]["z"]
    _, job, _ = run_reference(dict(REF, ref_offset=OFFSET, ref_slope=-0.005, ref_housing_z=z + 3.0))
    warnings = job.result["summary"]["warnings"]
    assert any("probe" in w and "-3.00" in w for w in warnings), warnings
    _, ok, _ = run_reference(dict(REF, ref_offset=OFFSET, ref_slope=-0.005, ref_housing_z=z + 0.2))
    assert not any("probe" in w for w in ok.result["summary"]["warnings"])


def test_y_is_homed_before_the_housing_touches():
    # The housing top slopes along Y near its edge: a fresh Y frame makes the touches repeat.
    rig, job, _ = run_reference(dict(REF, ref_offset=OFFSET, ref_slope=-0.005, steps=["reference"]))
    sent = rig.machine.sent
    first_housing = next(i for i, c in enumerate(sent) if c.startswith("G0 X") and "Y322.500" in c)
    assert "G28 Y" in sent[:first_housing]


def test_a_typed_diameter_far_from_the_measured_one_warns_and_the_measured_one_wins():
    rig, job, said = run_reference(dict(REF, ref_offset=OFFSET, ref_slope=-0.005, diameter=80.0))
    s = job.result["summary"]
    assert s["diameter"] == pytest.approx(78.0, abs=0.1) and s["diameter_set"] == 80.0
    assert any("typed in (80.00)" in w for w in s["warnings"])


def test_a_known_diameter_calibrates_the_offset_and_slope():
    rig, job, said = run_reference(dict(REF, known_diameter=78.0))
    cal = job.result["reference"]["calibration"]
    assert cal["offset"] == pytest.approx(OFFSET, abs=0.05)
    assert cal["slope"] == pytest.approx(-0.005, abs=0.0005) and cal["slope_measured"]


def test_a_calibration_only_run_finishes_and_says_so():
    rig, job, said = run_reference(dict(REF, known_diameter=78.0, steps=["reference", "runout"]))
    assert job.error is None, job.error
    s = job.result["summary"]
    assert s["calibration_only"] and s["diameter"] == pytest.approx(78.0, abs=0.1)
    assert s["runout_max_spread"] is not None and not job.result["complete"]
    assert job.result["reference"]["calibration"]["offset"] == pytest.approx(OFFSET, abs=0.05)


def test_runout_alone_without_a_known_diameter_still_needs_stations():
    rig, job, said = run_reference(dict(REF, ref_offset=OFFSET, ref_slope=-0.005, steps=["reference", "runout"]))
    assert "no stations" in (job.error or "")


def test_the_reference_refuses_the_endstop_and_a_missing_diameter():
    with pytest.raises(rotary_job.RotaryError, match="Y endstop"):
        rotary_job.params(dict(REF, ref_offset=OFFSET, ref_y=323.0))
    with pytest.raises(rotary_job.RotaryError, match="diameter is required"):
        rotary_job.params(dict(REF, steps=["runout", "stations"]))
    p = rotary_job.params(dict(axis_x=1, crown=1, stations=[1], y_chuck_limit=5, ref_offset=28.0))
    assert p["steps"][0] == "reference"


def test_the_diameter_maths():
    runout = [dict(y=225.0, mean=141.4475), dict(y=240.0, mean=141.45)]
    by_y = rotary_job.diameters(133.53, runout, 28.678, 0.002321, 322.5)
    assert by_y[0][1] == pytest.approx(73.644, abs=0.002)     # the A350, 2026-11-18 (calipers 73.4)
    cal = rotary_job.calibrate(133.583, [dict(y=60.0, mean=143.275), dict(y=130.0, mean=143.5),
                                         dict(y=200.0, mean=143.6)], 78.0, None, 322.5)
    assert cal["slope"] == pytest.approx(0.002321, abs=1e-5) and cal["offset"] == pytest.approx(28.678, abs=0.01)


def run_moved(params, dx):
    """run_reference with the whole rotary (housing and object) dx mm along X."""
    rig = Rig(settings=dict(z_floor=0.5, safe_z=200.0))
    rig.machine.xy_max = (320.0, 320.0)
    base = flask_and_housing(rig.machine)
    rig.device.surface = lambda x, y: base(x - dx, y)
    rig.machine.z = 160.0
    session = TapSession()

    def say(payload):
        if payload and payload.get("waiting_for_tap"):
            rig.device.tap()
    Job(rig.bridge, rig.esp, say, rig.config, [], mode="tap", clock=rig.clock, session=session).run()
    p = rotary_job.params(params)
    config = dict(rotary_job.probe_config(rig.config, p), rotary=p)
    job = Job(rig.bridge, rig.esp, say, config, [], mode="rotary", clock=rig.clock, session=session)
    job.run()
    return rig, job


def test_a_rotary_refitted_sideways_is_followed_from_the_housing():
    # 2026-11-22: the jig went back 12.8 mm to -X; the stale start missed the object at a station.
    base = dict(REF, ref_offset=OFFSET, ref_slope=-0.005, ref_radius=31.0)
    _, job0 = run_moved(base, 0.0)
    assert job0.error is None, job0.error
    seed_ref = dict(centre_x=job0.result["reference"]["centre_x"], z=job0.result["reference"]["z"])
    # (9 mm here: the simulated housing is a little narrower than the real one)
    _, stale = run_moved(base, -9.0)
    assert stale.error is not None                                   # what happened without it
    _, job = run_moved(dict(base, seed_ref=seed_ref, ref_x=seed_ref["centre_x"]), -9.0)
    assert job.error is None, job.error
    shift = job.result["housing_shift"]
    assert shift["dx"] == pytest.approx(-9.0, abs=0.3) and shift["moved"]
    assert job.result["summary"]["axis_x"] == pytest.approx(164.3 - 9.0, abs=0.3)


def test_the_housing_check_says_whether_the_rotary_moved_and_measures_nothing_else():
    base = dict(REF, ref_offset=OFFSET, ref_slope=-0.005, ref_radius=31.0)
    _, job0 = run_moved(base, 0.0)
    seed_ref = dict(centre_x=job0.result["reference"]["centre_x"], z=job0.result["reference"]["z"])
    check = dict(base, seed_ref=seed_ref, ref_x=seed_ref["centre_x"], steps=["reference"], check_only=True)
    rig, job = run_moved(check, 0.0)
    assert job.error is None, job.error
    assert job.result["check_only"] and not job.result["housing_shift"]["moved"]
    assert "stations" not in job.result and "runout" not in job.result
    assert all(y >= 300.0 for (_, y) in [(t.get("x"), t.get("y")) for t in job.result["touches"] if "y" in t])
    _, moved = run_moved(check, 3.0)
    assert moved.error is None and moved.result["housing_shift"]["moved"]
    assert moved.result["housing_shift"]["dx"] == pytest.approx(3.0, abs=0.3)


def test_verify_touches_the_top_where_the_measurement_says_and_reports_the_misses():
    base = dict(REF, ref_offset=OFFSET, ref_slope=-0.005, ref_radius=31.0)
    _, job0 = run_moved(base, 0.0)
    assert job0.error is None, job0.error
    measured = dict(job0.result, complete=True)
    measured["summary"] = dict(measured.get("summary") or {}, body_from_y=35.0, body_to_y=210.0)
    check = dict(base, steps=["verify"], verify_against=measured, verify_ys=[60.0, 120.0, 180.0],
                 verify_angles=[0.0, 90.0, 180.0, 270.0], diameter=78.0)
    rig, job = run_moved(check, 0.0)
    assert job.error is None, job.error
    v = job.result["verify"]
    assert len(v["points"]) == 12 and v["rms"] < 0.1                  # runout modelled: the top is where it said
    assert "stations" not in job.result and job.result["verify_only"]
    turns = [c for c in rig.machine.sent if c.startswith("G0 B")]
    assert turns and turns[-1].startswith("G0 B0.000")                 # back where it started
    _, shifted = run_moved(check, 2.0)
    assert shifted.error is None, shifted.error
    assert all(abs(r["dx"]) < 0.15 for r in v["sideways"])            # on the measured crest
    assert all(r["dx"] == pytest.approx(2.0, abs=0.3) for r in shifted.result["verify"]["sideways"])


def test_a_housing_touch_on_its_side_does_not_lower_the_next_start_below_its_top():
    # 2026-11-22: the first touch of a pair landed on the housing's side (5 mm low), the second
    # started from just over that and the pin deployed into the housing top.
    base = dict(REF, ref_offset=OFFSET, ref_slope=-0.005, ref_radius=31.0)
    rig, job = run_moved(dict(base, ref_x=164.8 - 12.0), 0.0)        # touches start 12 mm off centre
    assert job.error is None, job.error
    assert job.result["reference"]["centre_x"] == pytest.approx(164.8, abs=0.3)


def test_a_rotary_job_never_homes_x_itself():
    # 2026-11-22: X homes ~12 mm differently with the probe on; the frame must come from a home
    # made with the 10W seated, so the job (run with the probe on) must not home X or all axes.
    base = dict(REF, ref_offset=OFFSET, ref_slope=-0.005, ref_radius=31.0)
    rig, job = run_moved(base, 0.0)
    assert job.error is None, job.error
    assert "G28" not in rig.machine.sent and not any(c.startswith("G28 X") for c in rig.machine.sent)
