"""The GRBL face for LightBurn: protocol, queue, safety, against the recorded LightBurn traffic."""
import ast
import math
import os
import re

import pytest

from lhp_pkg import grbl_bridge as gb
from lhp_pkg import rotary_gcode as rg

DATA = os.path.join(os.path.dirname(__file__), "data", "lightburn")


def result(e=0.0):
    return dict(
        params=dict(diameter=78.0),
        axis=dict(y0=130.0, x0=164.4, z0=103.0, dx_dy=0.01, dz_dy=-0.005),
        summary=dict(body_from_y=25.0, body_to_y=216.0),
        runout=[dict(y=60.0, e=e, phase_deg=0.0), dict(y=200.0, e=e, phase_deg=0.0)],
    )


class Clock(object):
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class Sender(object):
    """Hands lines to a pretend printer that writes each at once (calls on_sent)."""

    def __init__(self):
        self.lines = []           # (line, counted)
        self.urgent_lines = []
        self.where = None
        self.bridge = None
        self.instant = True
        self.gens = []

    def send(self, lines, counted, gen=None):
        for l in lines:
            self.lines.append((l, counted))
            self.gens.append(gen)
            if counted and self.instant and self.bridge is not None:
                self.bridge.on_sent(l, gen)

    def urgent(self, lines):
        self.urgent_lines += lines

    def position(self):
        return self.where


def capture(after=None, until=None):
    """The RX lines LightBurn sent, between two markers of the recording."""
    out, on = [], after is None
    with open(os.path.join(DATA, "stream_capture_2_1_04.log")) as f:
        for row in f:
            if "MARK" in row:
                if after and after in row:
                    on = True
                elif until and until in row and on:
                    break
                continue
            if on and " RX " in row:
                out.append(ast.literal_eval(row.split(" RX ", 1)[1].strip()))
    return out


def rig(context=None, **kw):
    clock, sender = Clock(), Sender()
    bridge = gb.Bridge(sender, clock=clock, **kw)
    sender.bridge = bridge
    if context is not None:
        bridge.set_context(context)
    written = []
    session = gb.GrblSession(bridge, written.append)
    return clock, sender, bridge, session, written


def feed(session, bridge, clock, chunks):
    """Replay LightBurn bytes line by line (as the worker would), pumping as it goes, and let the
    printer 'send' each line at once."""
    replies = []
    for chunk in chunks:
        for raw in chunk.split(b"\n"):
            text = raw.decode()
            if text in ("?", ""):
                continue
            if text.startswith("\x18"):
                bridge.reset()
                text = text[1:]
                if not text:
                    continue
            replies.append((text, session.handle(text)))
            while bridge.pump():
                pass
    return replies


def rotary(armed=True, clock=None, **kw):
    s = rg.Surface(result(), focus_offset=5.75)
    clock = clock or Clock()
    return gb.RotaryContext(s, y_origin=25.0, laser_until=(clock.t + 900) if armed else None,
                            clock=clock, **kw), clock


# -- protocol ---------------------------------------------------------------------------------

def test_a_blank_line_gets_ok_and_status_has_four_axes():
    clock, sender, bridge, session, written = rig()
    assert session.handle("") == "ok"            # LightBurn's "?\n": the "\n" needs an ok
    assert re.match(r"<Idle\|MPos:-?\d+\.\d{3},-?\d+\.\d{3},-?\d+\.\d{3},-?\d+\.\d{3}\|FS:0,0>",
                    bridge.status_report())
    assert session.handle("$I").endswith("ok") and "[OPT:VZ,15,128]" in session.handle("$I")
    assert "$30=255" in session.handle("$$") and "$32=1" in session.handle("$$")
    assert session.handle("$Q") == "error:3"


def test_nothing_moves_until_octoprint_picks_a_mode():
    clock, sender, bridge, session, written = rig()
    reply = session.handle("G0 X10 Y10")
    assert reply.endswith("error:9") and "OctoPrint" in reply
    assert session.handle("$J=G91G21  X10 Y0 F800").endswith("error:9")
    assert sender.lines == []


def test_receive_answers_status_at_once_and_queues_lines():
    clock, sender, bridge, session, written = rig()
    session.receive(b"?\n$I\n")
    assert written[0].startswith("<Idle|")
    assert [l for g, l in session.lines] == ["", "$I"]


def test_a_soft_reset_drops_queued_lines_without_replies():
    ctx, clock = rotary()
    clock2, sender, bridge, session, written = rig(ctx)
    session.receive(b"G0 X10 A0\nG1 X20 S0 F800\n")
    session.receive(b"\x18")
    assert not session.lines                      # dropped: no "ok" for them
    assert written[-1].strip() == gb.BANNER
    assert sender.urgent_lines == ["M410"] and ("M5", False) in sender.lines


# -- rotary -----------------------------------------------------------------------------------

def test_rotary_frame_runs_unarmed_but_a_burn_needs_the_laser_armed():
    ctx, clock = rotary(armed=False)
    clock, sender, bridge, session, written = rig(ctx)
    frame = feed(session, bridge, clock, capture("rotary-frame", "rotary-start"))
    assert all(r == "ok" for t, r in frame), frame
    assert all(" S0 " in l or l.startswith(("G0", "M5", "G1 S0")) for l, c in sender.lines
               if l.startswith("G1"))
    start = feed(session, bridge, clock, capture("rotary-start", "rotary-stop"))
    refused = [r for t, r in start if r != "ok"]
    assert refused and "not armed" in refused[0] and refused[0].endswith("error:9")
    assert not any(re.search(r" S[1-9]", l) for l, c in sender.lines if l.startswith("G1"))


def burns(lines):
    """(surface mm, minutes) of each burning G1, from consecutive Snapmaker positions."""
    pos, out = None, []
    for l in lines:
        w = dict((k, float(v)) for k, v in re.findall(r"([XYZBSF])(-?[\d.]+)", l))
        if not l.startswith(("G0", "G1")) or not all(k in w for k in "XYZB"):
            continue
        if pos and l.startswith("G1") and w["S"] > 0:
            d = [w[k] - pos[k] for k in "XYZB"]
            fw = math.sqrt(sum(v * v for v in d))
            surf = math.hypot(math.hypot(d[0], d[1]), d[3] * math.pi * 78 / 360)
            out.append((surf, fw / w["F"], w["S"]))
        pos = w
    return out


def test_armed_rotary_start_streams_b_axis_lines_at_the_layer_speed():
    ctx, clock = rotary()
    clock, sender, bridge, session, written = rig(ctx)
    replies = feed(session, bridge, clock, capture("rotary-start", "rotary-stop"))
    assert all(r == "ok" for t, r in replies), [r for t, r in replies if r != "ok"][:3]
    out = [l for l, c in sender.lines]
    assert not any(l.startswith("G91") for l in out)
    assert out.index(next(l for l in out if l.startswith("M3 S0"))) < \
        out.index(next(l for l in out if re.search(r" S51 ", l)))
    b = [float(m) for m in re.findall(r" B(-?[\d.]+)", " ".join(out))]
    assert max(b) <= -152.0                       # LightBurn A 152.8.. -> B = -A
    bs = burns(out)
    assert bs and all(s <= 5.0 + 1e-6 for s, m, p in bs)          # split for a quick stop
    for surf, minutes, p in bs:
        if surf > 0.2:
            assert surf / minutes == pytest.approx(1200, rel=0.03)


def test_starvation_after_a_burn_sends_m5_and_rearms_before_the_next():
    ctx, clock = rotary()
    clock, sender, bridge, session, written = rig(ctx)
    feed(session, bridge, clock, [b"G00 G17 G40 G21 G54\nG90\nM4\nG0 X50A10\nG1 X60S100F1200\n"])
    assert bridge.emitter.armed
    clock.t += 30                                  # LightBurn silent, motion done
    bridge.pump()
    assert sender.lines[-1] == ("M5", False) and not bridge.emitter.armed
    feed(session, bridge, clock, [b"G1 X70S100F1200\n"])
    tail = [l for l, c in sender.lines[-4:]]
    assert any(l.startswith("M3 S0") for l in tail)


def test_hold_turns_the_beam_off_and_stops_sending_until_resume():
    ctx, clock = rotary()
    clock, sender, bridge, session, written = rig(ctx, window=1)
    sender.instant = False
    session.handle("G90")
    session.handle("M4")
    session.handle("G0 X50A10")
    session.handle("G1 X60S100F1200")
    bridge.hold()
    assert ("M5", False) in sender.lines and bridge.state() == "Hold:0"
    before = len(sender.lines)
    bridge.pump()
    assert len(sender.lines) == before
    bridge.resume()
    assert bridge.pump() >= 1


def test_stop_drops_the_queue_then_quickstops_and_turns_off():
    ctx, clock = rotary()
    clock, sender, bridge, session, written = rig(ctx, window=1, depth=200)
    sender.instant = False
    for l in ("G90", "M4", "G0 X50A10", "G1 X150S100F1200"):
        session.handle(l)
    assert bridge.queue
    bridge.reset()
    assert not bridge.queue and sender.urgent_lines == ["M410"] and ("M5", False) in sender.lines
    assert bridge.emitter.prev is None             # next motion starts with travel Z


def test_rotary_jogs_follow_the_surface_and_stay_on_the_body():
    ctx, clock = rotary()
    clock, sender, bridge, session, written = rig(ctx)
    feed(session, bridge, clock, [b"$J=G90G21  X50 Y0 F800\n", b"$J=G91G21  Z0 A10 F800\n"])
    out = [l for l, c in sender.lines if l.startswith(("G0", "G1"))]
    last = dict((k, float(v)) for k, v in re.findall(r"([XYZB])(-?[\d.]+)", out[-1]))
    assert last["Y"] == pytest.approx(75.0) and last["B"] == pytest.approx(-10.0)
    s = ctx.surface
    assert last["Z"] == pytest.approx(s.focus(75.0, -10.0), abs=1e-3)
    assert session.handle("$J=G91G21  Z10 F250").endswith("error:20")
    assert session.handle("$J=G90G21  X500 Y0 F800").endswith("error:15")
    feed(session, bridge, clock, [b"$J=G91G21  X0 Y10 F800\n"])        # Y jog = mm round
    last = dict((k, float(v)) for k, v in re.findall(r"([XYZB])(-?[\d.]+)",
                [l for l, c in sender.lines if l.startswith("G1")][-1]))
    assert last["B"] == pytest.approx(-10.0 - 10 * 360 / (math.pi * 78), abs=1e-3)


def test_the_laser_arm_expires():
    ctx, clock = rotary()
    clock2, sender, bridge, session, written = rig(ctx)
    clock.t += 901
    for l in ("G90", "M4", "G0 X50A10"):
        session.handle(l)
    assert "expired" in session.handle("G1 X60S100F1200")


# -- flat ---------------------------------------------------------------------------------------

def test_flat_start_runs_at_the_focus_z_with_the_beam_offset():
    clock = Clock()
    ctx = gb.FlatContext(20.5, offset_x=2.0, laser_until=clock.t + 900, clock=clock)
    clock, sender, bridge, session, written = rig(ctx)
    replies = feed(session, bridge, clock, capture("start-3", "pause-3"))
    assert all(r == "ok" for t, r in replies), [r for t, r in replies if r != "ok"][:3]
    out = [l for l, c in sender.lines if l.startswith("G1") and " S51 " in l]
    assert out and all(" Z20.500 " in l for l in out)
    first_g0 = next(l for l, c in sender.lines if l.startswith("G0 X"))
    assert first_g0.startswith("G0 X94.638 Y104.050")              # LightBurn X92.638 + 2


def test_flat_moves_off_the_bed_are_refused():
    clock = Clock()
    ctx = gb.FlatContext(20.5, laser_until=clock.t + 900, clock=clock)
    clock, sender, bridge, session, written = rig(ctx)
    session.handle("G90")
    assert session.handle("G0 X400 Y10").endswith("error:20")
    assert bridge.state() == "Alarm"                    # a refused stream line latches
    assert "alarm" in session.handle("G0 X10 Y10")
    bridge.reset()                                      # LightBurn's Stop clears it
    assert session.handle("$J=G91G21  X-50 Y0 F800").endswith("error:15")
    assert bridge.state() != "Alarm"                    # a refused jog does not latch


def test_the_whole_recording_replays_without_surprises():
    """Every byte LightBurn sent in the recording, through the protocol, armed rotary for the
    rotary part and flat for the rest: only refusals for things the bridge must refuse."""
    clock = Clock()
    flat = gb.FlatContext(20.5, offset_x=2.0, laser_until=clock.t + 900, clock=clock)
    c, sender, bridge, session, written = rig(flat)
    replies = feed(session, bridge, c, capture(None, "rotary-jog"))
    bad = [(t, r) for t, r in replies if r != "ok" and not t.startswith("$")]
    # The flat part's only refusal is the stray rotate jog (A in flat mode) and Z jogs.
    assert all("error:20" in r for t, r in bad), bad



def test_the_first_motion_sends_the_preamble_and_goes_up_before_across():
    ctx, clock = rotary()
    clock, sender, bridge, session, written = rig(ctx)
    bridge.machine_z = 300.0                            # head parked high (home is Z313)
    feed(session, bridge, clock, [b"G90\nG0 X50A10\n"])
    out = [l for l, c in sender.lines]
    assert out[:5] == ["M5", "G53 ; machine coordinates: no G92 offset can move this job", "G90",
                       "G21", "M211 S1"]
    zs = [float(m) for m in re.findall(r"^G0 Z(-?[\d.]+)", "\n".join(out), re.M)]
    assert zs[0] == pytest.approx(300.0)                # first Z: stays up while it crosses
    assert out.index(next(l for l in out if l.startswith("G0 X"))) < \
        out.index(next(l for l in out if l.startswith("G0 Z") and float(l.split("Z")[1].split()[0]) < 299))
    bridge.reset()
    sender.lines[:] = []
    feed(session, bridge, clock, [b"G90\nG0 X60A10\n"])
    assert sender.lines[0][0] == "M5" and sender.lines[1][0].startswith("G53")


def test_lines_without_motion_are_fine_before_a_mode_is_picked():
    clock, sender, bridge, session, written = rig()
    assert session.handle("G0") == "ok"                 # LightBurn's connect-time bare G0
    assert session.handle("M5") == "ok"                 # and its after-stop M5
    assert session.handle("G0 X1 Y1").endswith("error:9")


def test_stop_makes_queued_lines_stale_and_mode_changes_stop_too():
    ctx, clock = rotary()
    clock, sender, bridge, session, written = rig(ctx, window=2, depth=500)
    sender.instant = False
    for l in ("G90", "M4", "G0 X50A10", "G1 X150S100F1200"):
        session.handle(l)
    bridge.pump()
    old = [g for g in sender.gens if g is not None]
    assert old and not bridge.stale(old[-1])
    bridge.reset()
    assert bridge.stale(old[-1])                        # the glue drops these unsent
    assert not bridge.inflight
    for l in ("G90", "M4", "G0 X50A10", "G1 X150S100F1200"):
        session.handle(l)
    sender.urgent_lines[:] = []
    bridge.set_context(rotary()[0])                     # picking a mode mid-job stops it
    assert sender.urgent_lines == ["M410"] and not bridge.queue


def test_the_arm_is_checked_at_a_jobs_first_burn_and_held_through_it():
    ctx, clock = rotary()
    c, sender, bridge, session, written = rig(ctx, window=1, depth=500)
    sender.instant = False
    for l in ("G90", "M4", "G0 X50A10", "G1 X60S100F1200"):
        assert session.handle(l) == "ok"
    clock.t += 901                                     # the arm expires mid-job
    assert session.handle("G1 X70S100F1200") == "ok"    # the job carries on
    bridge.reset()
    for l in ("G90", "M4", "G0 X50A10"):
        session.handle(l)
    assert "expired" in session.handle("G1 X60S100F1200")   # a new job checks again


def test_relative_jogs_after_a_frame_keep_adding_up():
    ctx, clock = rotary()
    clock, sender, bridge, session, written = rig(ctx)
    feed(session, bridge, clock, capture("rotary-frame", "rotary-start"))
    feed(session, bridge, clock, [b"$J=G91G21  X10 Y0 F800\n", b"$J=G91G21  X10 Y0 F800\n"])
    ys = [float(m) for m in re.findall(r" Y(-?[\d.]+)", " ".join(l for l, c in sender.lines))]
    assert ys[-1] == pytest.approx(25.0 + 74.0 + 20.0)  # frame ends at LightBurn X74; +10 twice


def test_dry_run_sends_every_burn_at_s0_with_no_arm():
    ctx, clock = rotary(armed=False, dry_run=True)
    clock, sender, bridge, session, written = rig(ctx)
    replies = feed(session, bridge, clock, capture("rotary-start", "rotary-stop"))
    assert all(r == "ok" for t, r in replies)
    out = [l for l, c in sender.lines]
    assert not any(re.search(r" S[1-9]", l) for l in out if l.startswith("G1"))
    assert any(l.startswith("G1 ") for l in out)


def strip_comments(lines):
    return [l.split(";", 1)[0] for l in lines]


@pytest.mark.parametrize("name", ["rect1.gc", "rect30.gc", "circle.gc", "fill.gc"])
def test_the_stream_matches_the_file_translator_move_for_move(name):
    """The four real exports, with comments stripped (as LightBurn streams them), through the
    bridge, against translate() on the same file: same burn endpoints, power and speed."""
    with open(os.path.join(DATA, name)) as f:
        lines = f.read().splitlines()
    s = rg.Surface(result(), focus_offset=5.75)
    batch, rep = rg.translate(lines, s, y_origin=25.0)
    ctx = gb.RotaryContext(s, y_origin=25.0, laser_until=2e9, clock=lambda: 0.0, seg_max=None)
    clock, sender, bridge, session, written = rig(ctx)
    for l in strip_comments(lines):
        if l.strip() and not l.upper().startswith("M112"):
            assert session.handle(l) in ("ok",), l
        while bridge.pump():
            pass
    stream = [l for l, c in sender.lines]
    pick = lambda ls: [re.sub(r" F\d+", "", l) for l in ls if l.startswith("G1 ") and " S0 " not in l]
    assert pick(stream) == pick(batch)
    fb, fs = burns(batch), burns(stream)
    assert len(fb) == len(fs)
    for (sb, mb, pb), (ss, ms, ps) in zip(fb, fs):
        if sb > 0.2:
            assert ss / ms == pytest.approx(sb / mb, rel=0.03)



def test_lightburns_return_to_finish_position_in_rotary_is_not_an_alarm():
    """End of the real dry run (2026-11-17): LightBurn sent "G0 X0Y0" then "M2" after the job.
    In rotary mode Y is mm round the surface (Y0 = A0), so it is a travel move, not a refusal."""
    ctx, clock = rotary()
    clock, sender, bridge, session, written = rig(ctx)
    feed(session, bridge, clock, [b"G90\nM4\nG0 X50A10\nG1 X60S100F1200\nM5\nG0 X0Y0\nM2\n"])
    assert bridge.state() != "Alarm"
    last = [l for l, c in sender.lines if l.startswith("G0 X")][-1]
    assert " B0.000 " in last or " B-0.000 " in last


# -- the head marker (each LightBurn device's Start GCode) -----------------------------------

# What LightBurn 2.1.04 sent on Start from a device whose Start GCode was the marker: the
# marker first, as its own line, then LightBurn's own preamble (captured on the Mac).
JOB = b"G00 G17 G40 G21 G54\nG90\nM4\nM8\nG0 X60A10\nG1 X70S51F1200\nM9\nG1 S0\nM5\nG90\nM2\n"


def test_a_burn_needs_the_job_to_name_the_selected_laser():
    ctx, clock = rotary(module="10W")
    clock, sender, bridge, session, written = rig(ctx)
    replies = feed(session, bridge, clock, [JOB])
    refused = [r for t, r in replies if r != "ok"]
    assert refused and "does not say which laser and mode" in refused[0] and refused[0].endswith("error:9")
    assert "Snapmaker 10W rotary via OctoPrint" in refused[0]
    assert not any(re.search(r" S[1-9]", l) for l, c in sender.lines if l.startswith("G1"))
    bridge.reset()
    sender.lines[:] = []
    replies = feed(session, bridge, clock, [b"(lhp head=10W mode=rotary dia=78)\n" + JOB])
    assert all(r == "ok" for t, r in replies), replies
    assert any(re.search(r" S51 ", l) for l, c in sender.lines)
    assert bridge.job is None                         # M2 ends the job: the next must say again


def test_a_job_for_the_other_laser_alarms_before_anything_moves():
    ctx, clock = rotary(module="10W")
    clock, sender, bridge, session, written = rig(ctx)
    replies = feed(session, bridge, clock, [b"(lhp head=2W IR mode=rotary)\n" + JOB])
    assert "for the 2W IR but OctoPrint has the 10W" in replies[0][1]
    assert bridge.state() == "Alarm"
    assert not any(l.startswith(("G0", "G1")) for l, c in sender.lines)
    assert all(r != "ok" for t, r in replies[1:] if t.startswith(("G0", "G1")))


def test_the_marker_matches_names_loosely_and_a_dry_run_needs_none():
    assert gb.job_marker("(LHP head = 2W IR mode=Rotary)") == dict(head="2W IR", mode="rotary", dia=None)
    assert gb.job_marker("(lhp mode=flat head=10W)") == dict(head="10W", mode="flat", dia=None)
    assert gb.job_marker("(lhp head=10W)") == dict(head="10W", mode=None, dia=None)
    assert gb.job_marker("(lhp head=10W mode=rotary dia=73.6)")["dia"] == 73.6
    assert gb.job_marker("(lhp head=10W dia=big)")["dia"] is None
    assert gb.job_marker("(something else)") is None and gb.job_marker("G0 X1") is None
    assert gb.same_head("2WIR", "2W IR") and not gb.same_head("10W", "2W IR")
    ctx, clock = rotary(module="10W", dry_run=True)
    clock, sender, bridge, session, written = rig(ctx)
    replies = feed(session, bridge, clock, [JOB])
    assert all(r == "ok" for t, r in replies), replies


def test_the_beam_offset_moves_the_head_but_not_the_lightburn_position():
    plain, clock = rotary()
    shifted, _ = rotary(beam_dx=1.5, beam_dy=-2.0)
    lines = {}
    for name, ctx in (("plain", plain), ("shifted", shifted)):
        clock, sender, bridge, session, written = rig(ctx)
        feed(session, bridge, clock, [b"G00 G17 G40 G21 G54\nG90\nM4\nG0 X60A10\nG1 X70S51F1200\n"])
        lines[name] = [dict((k, float(v)) for k, v in re.findall(r"([XYZB])(-?[\d.]+)", l))
                       for l, c in sender.lines if l.startswith("G1 X")]
    for a, b in zip(lines["plain"], lines["shifted"]):
        assert b["X"] == pytest.approx(a["X"] + 1.5) and b["Y"] == pytest.approx(a["Y"] - 2.0)
        assert b["Z"] == pytest.approx(a["Z"]) and b["B"] == pytest.approx(a["B"])
    y = 25.0 + 30.0 - 2.0
    assert shifted.to_lightburn(0.0, y, 0.0, -20.0)[0] == pytest.approx(30.0)


def test_a_flat_job_on_the_rotary_alarms_and_a_marker_without_mode_cannot_burn():
    ctx, clock = rotary(module="10W")
    clock, sender, bridge, session, written = rig(ctx)
    replies = feed(session, bridge, clock, [b"(lhp head=10W mode=flat)\n" + JOB])
    assert "for flat but OctoPrint has rotary picked" in replies[0][1]
    assert "Snapmaker 10W rotary via OctoPrint" in replies[0][1]
    assert bridge.state() == "Alarm"
    bridge.reset()
    replies = feed(session, bridge, clock, [b"(lhp head=10W)\n" + JOB])
    refused = [r for t, r in replies if r != "ok"]
    assert refused and "does not say which laser and mode" in refused[0]


def test_a_rotary_burn_needs_lightburns_diameter_to_match_the_measured_one():
    ctx, clock = rotary(module="10W")                 # the stored measurement says 78 mm
    assert ctx.diameter == pytest.approx(78.0)
    clock, sender, bridge, session, written = rig(ctx)
    replies = feed(session, bridge, clock, [b"(lhp head=10W mode=rotary dia=70)\n" + JOB])
    assert "rotary diameter is 70 but the object measures 78.0 mm" in replies[0][1]
    assert bridge.state() == "Alarm"
    assert not any(l.startswith(("G0", "G1")) for l, c in sender.lines)
    bridge.reset()
    replies = feed(session, bridge, clock, [b"(lhp head=10W mode=rotary)\n" + JOB])
    refused = [r for t, r in replies if r != "ok"]
    assert refused and "add dia=78.0 to the Start GCode" in refused[0]
    assert not any(re.search(r" S[1-9]", l) for l, c in sender.lines if l.startswith("G1"))
    bridge.reset()
    sender.lines[:] = []
    replies = feed(session, bridge, clock, [b"(lhp head=10W mode=rotary dia=78.4)\n" + JOB])
    assert all(r == "ok" for t, r in replies), replies     # within 0.5 mm


FRAME = b"G00 G17 G40 G21 G54\nG90\nM4\nG0 X10Y10\nG1 Y20S10F3000\nG1 X20S200\nG1 Y10\nG1 X10\n"


def test_the_spot_pointer_frames_and_fires_capped_without_an_arm():
    clock = Clock()
    ctx = gb.FlatContext(20.5, laser_until=None, clock=clock, module="10W")
    clock, sender, bridge, session, written = rig(ctx)
    replies = feed(session, bridge, clock, [FRAME])
    assert any(r != "ok" for t, r in replies) and "not armed" in " ".join(r for t, r in replies)
    bridge.reset()
    sender.lines[:] = []
    bridge.set_pointer(dict(kind="spot", s_max=10, until=clock.t + 300))
    replies = feed(session, bridge, clock, [FRAME])
    assert all(r == "ok" for t, r in replies), replies
    powers = [int(m) for l, c in sender.lines for m in re.findall(r" S(\d+)", l) if l.startswith("G1")]
    assert powers and max(powers) == 10                       # S200 asked, S10 sent
    sender.lines[:] = []
    feed(session, bridge, clock, [b"M3 S40\nG1 F1000\n"])     # LightBurn's Fire button
    assert ("M3 S10", True) in sender.lines or any(l == "M3 S10" for l, c in sender.lines)
    clock.t += 301                                            # the pointer times out
    sender.lines[:] = []
    replies = feed(session, bridge, clock, [b"G1 X15S10\n"])
    assert replies[-1][1] != "ok"


def test_the_crosshair_pointer_holds_the_beam_off_and_shifts_the_head():
    clock = Clock()
    plain = gb.FlatContext(20.5, laser_until=None, clock=clock, module="2W IR")
    clock, sender, bridge, session, written = rig(plain)
    bridge.set_pointer(dict(kind="cross", dx=21.5, dy=-11.0, until=clock.t + 300))
    assert ("M2002 T3 P1", False) in sender.lines
    feed(session, bridge, clock, [FRAME])
    g1 = [dict((k, float(v)) for k, v in re.findall(r"([XYS])(-?[\d.]+)", l)) for l, c in sender.lines
          if l.startswith("G1")]
    assert g1 and all(m.get("S", 0) == 0 for m in g1)          # never fires
    assert any(m.get("X") == pytest.approx(20 - 21.5) for m in g1)
    assert any(m.get("Y") == pytest.approx(20 + 11.0) for m in g1)
    bridge.reset()                                            # Stop: crosshair off, shift gone
    assert ("M2002 T3 P0", False) in sender.lines and bridge.pointer_state() is None
    assert bridge.emitter.ox == pytest.approx(0.0) and bridge.emitter.oy == pytest.approx(0.0)


def test_lightburns_camera_macro_takes_a_head_photo_through_the_plugin():
    # M240 (Marlin's "trigger camera") from a LightBurn macro: the plugin moves the head to the
    # photo spot and asks the touchscreen for the 10W's photo. The head has moved, so the bridge
    # reads where it is again.
    calls = []
    clock = Clock()
    ctx = gb.FlatContext(20.5, laser_until=None, clock=clock, module="10W")
    clock, sender, bridge, session, written = rig(ctx)
    def shoot(variant=0):
        calls.append("photo")
        sender.where = (232.0, 178.0, 290.0, 0.0)
    bridge.camera_trigger = shoot
    replies = feed(session, bridge, clock, [b"M240\n"])
    assert [r for t, r in replies] == ["ok"] and calls == ["photo"]
    assert not any("M240" in l for l, c in sender.lines)
    assert bridge.machine_z == 290.0                          # read again after the move
    def refuse(variant=0):
        raise ValueError("the 2W IR has no camera")
    bridge.camera_trigger = refuse
    replies = feed(session, bridge, clock, [b"M240\n"])
    assert "no camera" in replies[0][1] and "error:" in replies[0][1]
    assert bridge.state() != "Alarm"
    bridge.camera_trigger = None
    replies = feed(session, bridge, clock, [b"M240\n"])
    assert "error:" in replies[0][1]


def test_the_camera_macro_waits_for_an_idle_bridge():
    clock = Clock()
    ctx = gb.FlatContext(20.5, laser_until=None, clock=clock, module="10W")
    clock, sender, bridge, session, written = rig(ctx)
    calls = []
    bridge.camera_trigger = lambda variant=0: calls.append("photo")
    bridge.busy_until = clock() + 10.0                         # a move still running
    replies = feed(session, bridge, clock, [b"M240\n"])
    assert "error:" in replies[0][1] and "idle" in replies[0][1] and calls == []


def test_a_b_shift_reports_the_object_angle_back_to_lightburn():
    ctx, _ = rotary(b_shift=-68.0)
    # the firmware at B0 is the object at -68: LightBurn A = object / b_sign = 68
    assert ctx.to_lightburn(0.0, 50.0, 0.0, 0.0)[3] == pytest.approx(68.0)
    em = ctx.emitter()
    assert em.b_shift == -68.0


def test_lightburns_crosshair_macro_switches_the_pointer_through_the_plugin():
    # A LightBurn macro sends the Snapmaker crosshair command; the bridge hands it to the plugin
    # (which reads the offset and shifts the head) instead of passing it to the printer.
    calls = []
    def switch(on):
        calls.append(on)
        if on == "boom":
            raise ValueError("never")
    clock = Clock()
    ctx = gb.FlatContext(20.5, laser_until=None, clock=clock, module="2W IR")
    clock, sender, bridge, session, written = rig(ctx)
    bridge.pointer_switch = switch
    replies = feed(session, bridge, clock, [b"M2002 T3 P1\n", b"m2002 t3 p0\n"])
    assert [r for t, r in replies] == ["ok", "ok"] and calls == [True, False]
    assert not any("M2002" in l for l, c in sender.lines)
    def refuse(on):
        raise ValueError("the 10W has no crosshair")
    bridge.pointer_switch = refuse
    replies = feed(session, bridge, clock, [b"M2002 T3 P1\n"])
    assert "the 10W has no crosshair" in replies[0][1] and "error:" in replies[0][1]
    assert bridge.state() != "Alarm"                          # a refused macro is not an alarm
    bridge.pointer_switch = None
    replies = feed(session, bridge, clock, [b"M2002 T3 P1\n"])
    assert "error:" in replies[0][1]


def test_m240_p1_asks_the_plugin_for_the_close_up_again():
    clock = Clock()
    ctx = gb.FlatContext(20.5, laser_until=None, clock=clock, module="10W")
    clock, sender, bridge, session, written = rig(ctx)
    calls = []
    bridge.camera_trigger = lambda variant=0: calls.append(variant)
    replies = feed(session, bridge, clock, [b"M240\n", b"M240 P1\n", b"M240 P0 ; overview\n"])
    assert [r for t, r in replies] == ["ok", "ok", "ok"] and calls == [0, 1, 0]
    replies = feed(session, bridge, clock, [b"M240 P7\n"])
    assert "error:" in replies[0][1]                       # not ours: not passed to the printer either
    assert not any("M240" in l for l, c in sender.lines)


def test_lightburn_is_locked_out_while_the_plugin_moves_the_head():
    # A close-up moves the head for a minute or more; a LightBurn jog, home, job or photo macro
    # in that time would fight it. Harmless lines are still accepted.
    clock = Clock()
    ctx = gb.FlatContext(20.5, laser_until=None, clock=clock, module="10W")
    clock, sender, bridge, session, written = rig(ctx)
    bridge.camera_trigger = lambda variant=0: None
    bridge.lock_out("taking close-up photos")
    assert bridge.state() == "Run"
    for line in (b"G0 X10 Y10\n", b"M240\n", b"$J=G91 X5 F1000\n", b"$H\n"):
        replies = feed(session, bridge, clock, [line])
        assert "close-up" in replies[0][1] and "error:" in replies[0][1], line
    assert [r for t, r in feed(session, bridge, clock, [b"M5\n"])] == ["ok"]
    assert bridge.state() != "Alarm"
    bridge.lock_release()
    assert bridge.state() == "Idle"
    assert [r for t, r in feed(session, bridge, clock, [b"G0 X10 Y10\n"])] == ["ok"]


def test_the_purifier_meter_reads_what_the_bridge_sends():
    """The air purifier counts laser energy from the lines OctoPrint sends: check it against the
    bridge's own output for a known LightBurn job (100 mm at full power, 1200 mm/min = 5 s)."""
    from lhp_pkg import purifier
    clock = Clock()
    ctx = gb.FlatContext(20.5, offset_x=2.0, laser_until=clock.t + 900, clock=clock, module="10W")
    clock, sender, bridge, session, written = rig(ctx)
    job = [b"(lhp head=10W mode=flat)", b"G90", b"M4 S0", b"G0 X10Y10", b"G1 X60S255F1200",
           b"G1 Y60", b"G0 X100Y100S0", b"G1 X102S10", b"M5"]
    replies = feed(session, bridge, clock, job)
    assert all(r == "ok" for t, r in replies), replies
    meter = purifier.BurnMeter(watts=10.0, pointer_s_max=10)
    joules = sum(meter.feed(l) for l, counted in sender.lines)
    assert joules == pytest.approx(50.0, rel=0.02)      # the S10 move is the pointer level: not counted
    assert meter.burn_s == pytest.approx(5.0, rel=0.02)


def test_disarming_after_a_job_does_not_quickstop_the_moves_still_running():
    clock = Clock()
    ctx = gb.FlatContext(20.5, laser_until=clock.t + 900, clock=clock, module="10W")
    clock, sender, bridge, session, written = rig(ctx)
    feed(session, bridge, clock, [b"(lhp head=10W mode=flat)", b"G90", b"M4 S0", b"G1 X10Y10S100F1200", b"M5"])
    assert not bridge.queue and not bridge.inflight            # all sent; the printer may still be moving
    bridge.set_context(None)
    assert "M410" not in sender.urgent_lines                   # nothing aborted
    assert sender.lines[-1][0] == "M5"


def test_disarming_mid_job_still_stops_it():
    clock = Clock()
    ctx = gb.FlatContext(20.5, laser_until=clock.t + 900, clock=clock, module="10W")
    clock, sender, bridge, session, written = rig(ctx)
    bridge.queue.append(("G1 X5", None, 0.1, "move"))          # something still waiting to go
    bridge.set_context(None)
    assert "M410" in sender.urgent_lines


def test_a_frame_leaves_its_outline_and_a_burning_job_does_not():
    clock = Clock()
    ctx = gb.FlatContext(20.5, laser_until=clock.t + 900, clock=clock, module="10W")
    clock, sender, bridge, session, written = rig(ctx)
    # LightBurn's Frame: a loop of power-0 moves (stream_capture_2_1_04.log)
    feed(session, bridge, clock, [b"G00 G17 G40 G21 G54", b"G90", b"G0 X74Y104", b"G1 Y145S0F800", b"G1 X115",
                                  b"G1 Y104", b"G1 X74", b"G90"])
    assert bridge.frame() is None                                    # still within the stream
    clock.t += 3.0
    f = bridge.frame()
    assert f["bounds"] == [74.0, 104.0, 115.0, 145.0] and f["mode"] == "flat"
    # a burning job after it does not replace the frame
    feed(session, bridge, clock, [b"(lhp head=10W mode=flat)", b"G90", b"M4 S0", b"G0 X80Y110",
                                  b"G1 X90S100F1200", b"G1 Y120", b"G1 X80", b"M5"])
    clock.t += 3.0
    assert bridge.frame()["bounds"] == [74.0, 104.0, 115.0, 145.0]


def test_a_dry_run_counts_as_a_frame():
    clock = Clock()
    ctx = gb.FlatContext(20.5, laser_until=clock.t + 900, clock=clock, module="10W")
    ctx.dry_run = True
    clock, sender, bridge, session, written = rig(ctx)
    feed(session, bridge, clock, [b"(lhp head=10W mode=flat)", b"G90", b"M4 S0", b"G0 X10Y10",
                                  b"G1 X30S100F1200", b"G1 Y40", b"G1 X10", b"M5"])
    clock.t += 3.0
    assert bridge.frame()["bounds"] == [10.0, 10.0, 30.0, 40.0]
