# coding=utf-8
"""The jobs: Probe here (the hand workflow), the tap test, the automatic multi-point job and
the calibration's reference touch. All run in this one thread class, by mode.

Probe here ("here"): the user has jogged the head over the spot. M5 (laser off) first, then
the session's tap test must still be valid (TapSession.require), then probe.Prober.probe_here
lowers the head from where it is and brings it back up. No XY move.

The tap test ("tap"): no motion at all. Once per session, and again after the probe board
restarts.

The automatic job ("job") and the calibration touch ("calibrate"):

1. Check the probe answers (a failed read here stops the job before anything moves), stow
   the pin so it cannot drag across the object, and start the restart watchdog.
2. Soft endstops on (M211 S1). Home if home_first; otherwise go up to max(current Z, safe_z).
   Nothing here moves down: on this machine G28 homes Z upwards.
3. The tap test, unless the user turned it off: arm the pin where the head is (high, clear of
   the object), ask the user to tap it with a finger, and wait up to 60 s for the latch. No
   latch, no job. This proves the whole signal path (pin, wire, diode, ESP interrupt, Wi-Fi,
   REST) on the day, which the ESP's own probe_check cannot do on a clone without touch mode.
   It also leaves trigger_count at 1 or more, which the watchdog relies on.
4. Read Z steps/mm (bare M92), then measure each point with probe.Prober. A point that fails
   stops the job: on a delicate object, an alarm or a missing trigger means something is wrong
   with the setup, and the next point would meet the same problem. The points measured so far
   are kept.
5. Always, also after an error or an abort: head up to max(current Z, safe_z), pin stowed.
   Only then is the outcome said, so the UI never shows "done" while the head is still low.

The laser module stays plugged in while the probe sits in its bay, so the firmware is in laser
mode. The one laser command the plugin sends is a bare M5 (laser off), first thing in every
Probe here, job and calibration. Nothing else: no M3/M4, no fan, no heater. The only other
G-code here and in probe.py is G0, G28, G90, G91 (only for a blind raise when the position
cannot be read), M211 S1, M92 and the bridge's M400, M114 and M118. The height map's G-code
(M1006, M420, M421, G1029) is in meshload.py, and only runs when the user presses its buttons.
"""
from __future__ import absolute_import

import threading
import time

from . import gcode, results
from .esp import EspRebooted, Watchdog
from .probe import LATCH_SETTLE_S, Aborted, ProbeError, Prober, raise_to_safe

TAP_TIMEOUT_S = 60.0
TAP_POLL_S = 0.25
LASER_OFF = "M5"


# Probe here from a given height (flat bed): the head drops, pin stowed, to this far above where
# the height puts the top, then probes no more than HERE_FROM_WINDOW down. A touch within
# probe.TRAVEL_MARGIN of that start means the object is taller than given, and fails.
HERE_FROM_CLEAR = 6.0
HERE_FROM_WINDOW = 9.0


class TapSession(object):
    """The once-per-session tap test, and what makes it stale.

    A passed tap test leaves a started Watchdog behind. Before each Probe here it is checked:
    if trigger_count or uptime went backwards, the probe board restarted since the tap, so the
    signal path is unproven again and Probe here refuses until the tap test is repeated.
    """

    def __init__(self):
        self.watch = None
        self.at = None
        self.reason = "no tap test yet this session"

    @property
    def ok(self):
        return self.watch is not None

    def passed(self, watch, when=None):
        self.watch = watch
        self.at = when if when is not None else time.time()
        self.reason = None

    def invalidate(self, reason):
        self.watch = None
        self.at = None
        self.reason = reason

    def require(self):
        """The watchdog to probe with. Raises ProbeError if the tap test is missing or stale."""
        if self.watch is None:
            raise ProbeError("do the tap test first (%s)" % self.reason)
        try:
            self.watch.check("since the tap test", force_uptime=True)
        except EspRebooted as exception:
            self.invalidate(str(exception))
            raise ProbeError("the probe board restarted since the tap test: do the tap test again")
        return self.watch


class Job(threading.Thread):
    """Runs one job, or the calibration touch (mode "calibrate"). `say` gets progress dicts.

    module: the selected laser module, {name, focus_offset, dx, dy}. Its dx, dy place the pin;
    None means the module's XY offset was never measured, which is used as 0, 0 with a warning.
    """

    def __init__(self, bridge, esp, say, config, points, mode="job", module=None,
                 clock=time, logger=None, session=None):
        threading.Thread.__init__(self, name="laserheightprobe-job")
        self.daemon = True
        self._bridge = bridge
        self._esp = esp
        self._say = say
        self._c = config
        self._points = list(points)
        self.mode = mode
        self._module = dict(module or dict(name=None, focus_offset=None, dx=0.0, dy=0.0))
        self._clock = clock
        self._logger = logger
        self._abort = threading.Event()
        self._measured = []
        self._watch = None
        self._session = session if session is not None else TapSession()
        self._here_start = None
        self._rotary_partial = None
        self._shift = None
        self.phase = "starting"
        self.result = None
        self.error = None

    # -- control from the plugin -----------------------------------------

    def abort(self):
        self._abort.set()

    @property
    def aborted(self):
        return self._abort.is_set()

    @property
    def xy_calibrated(self):
        return self._module.get("dx") is not None and self._module.get("dy") is not None

    # -- helpers ----------------------------------------------------------

    def _check(self):
        if self._abort.is_set():
            raise Aborted()

    def _tell(self, phase, message, **extra):
        self.phase = phase
        payload = dict(type="progress", phase=phase, message=message, mode=self.mode)
        payload.update(extra)
        self._say(payload)

    def _sleep(self, seconds):
        end = self._clock.time() + seconds
        while self._clock.time() < end:
            self._check()
            self._clock.sleep(min(0.1, max(0.001, end - self._clock.time())))
        self._check()

    def _result(self, started, complete):
        points = self._measured
        reference = next((i for i, p in enumerate(points) if p.get("ref")), None)
        datum = next((i for i, p in enumerate(points) if p.get("datum")), None)
        m = self._module
        return dict(
            finished=time.time(), minutes=round((self._clock.time() - started) / 60.0, 1),
            mode=self.mode, complete=complete, points=points, reference=reference, datum=datum,
            module=m.get("name"), focus_offset=m.get("focus_offset"),
            dx=m.get("dx"), dy=m.get("dy"), xy_calibrated=self.xy_calibrated,
            stages=[list(s) for s in self._c["stages"]], fine_step=self._c["stages"][-1][0],
            tap_test=self._c["tap_test"], machine_shift=self._shift,
        )

    # -- the phases -------------------------------------------------------

    def _prepare(self):
        c = self._c
        self._bridge.run([LASER_OFF], 30.0)
        self._tell("check", "Checking the probe at %s." % self._esp.base_url)
        self._esp.latched()                 # raises if the ESP cannot be read: nothing has moved
        self._esp.stow()
        self._watch = Watchdog(self._esp, clock=self._clock, logger=self._logger).start()
        self._check()
        if c["home_first"]:
            self._esp.show_state("homing")
            self._tell("home", "Homing.")
            self._bridge.run(["M211 S1", "G28", "G90"], 240.0)
        else:
            self._bridge.run(["M211 S1", "G90"], 30.0)
        self._check()
        # Up to travel height if below it; a head above safe_z stays where it is.
        if not raise_to_safe(self._bridge, c["safe_z"], c["z_feed"], self._logger):
            raise ProbeError("could not read the head position to reach a safe height")
        self._check()
        if c["tap_test"] and self._session_tap_holds():
            self._tell("tap", "The tap test passed earlier this session (%s) and the probe board has not "
                              "restarted since: not asking again."
                       % time.strftime("%H:%M", time.localtime(self._session.at)), waiting_for_tap=False)
        elif c["tap_test"]:
            self._tap_test()
        else:
            self._tell("tap", "The tap test is turned off in the settings: the probe's signal path "
                              "has not been proven today.", warning=True)
        if not self.xy_calibrated:
            self._tell("check", "Warning: %s has no measured probe XY offset. Using 0, 0, so the pin "
                                "touches where the 10W beam would be, not where this module's beam "
                                "will be." % (self._module.get("name") or "this module"), warning=True)
        steps_lines = self._bridge.run(["M92"], 30.0)
        self._shift = self._machine_shift(steps_lines)
        return gcode.z_steps_per_mm(steps_lines)

    def _machine_shift(self, steps_lines):
        """(sx, sy) with machine X, Y = G-code X, Y + shift, from M114, or None.

        The firmware's bed mesh is indexed by machine X, Y, and a G92 or work origin moves the
        G-code frame, so a height map needs the shift that held while it was probed.
        """
        steps_xy = gcode.xy_steps_per_mm(steps_lines)
        if steps_xy is None:
            return None
        lines = self._bridge.run([], 30.0)
        found, machine = gcode.position(lines), gcode.machine_xy(lines, steps_xy)
        if found is None or machine is None:
            return None
        return [round(machine[0] - found[0], 3), round(machine[1] - found[1], 3)]

    def _session_tap_holds(self):
        """True when this session's tap test passed (or was assumed) and the probe board has not
        restarted since; a stale one is dropped, so the job asks for the tap again."""
        if not self._session.ok:
            return False
        try:
            self._session.require()
        except ProbeError:
            return False
        return True

    def _tap_test(self):
        """Arm, ask for a finger tap, wait for the latch. Refuse the job without it."""
        esp = self._esp
        self._watch.check("before the tap test", force_uptime=True)
        esp.wait_idle(self._sleep)
        if not esp.release_held_trigger(self._sleep):
            raise ProbeError("probe output stays HIGH after a reset: the probe is in alarm. "
                             "Check its LED and wiring")
        esp.arm()
        self._sleep(self._c["arm_wait_s"])
        esp.wait_idle(self._sleep)
        esp.clear()
        self._sleep(LATCH_SETTLE_S)
        if esp.raw() or esp.latched():
            try:
                esp.reset()
            except Exception:                                # noqa: BLE001
                if self._logger:
                    self._logger.exception("laserheightprobe: probe_reset failed")
            raise ProbeError("probe alarm before the tap test: the pin did not deploy, or the "
                             "output is stuck HIGH")
        if self._c.get("tap_assumed"):
            # The user proved the signal path another way (the probe board's BOOT button test).
            # The pin still deployed above with no alarm; the watchdog still guards restarts.
            esp.stow()
            esp.clear()
            self._watch.check("after the assumed tap", force_uptime=True)
            self._session.passed(self._watch)
            self._tell("tap", "Tap assumed (the user tested the probe with its button): the pin "
                              "deployed and stowed with no alarm.", waiting_for_tap=False)
            return
        self._tell("tap", "Tap the probe pin with a finger now (%d s)." % TAP_TIMEOUT_S,
                   timeout_s=TAP_TIMEOUT_S, waiting_for_tap=True)
        end = self._clock.time() + TAP_TIMEOUT_S
        tapped = False
        try:
            while self._clock.time() < end:
                self._check()
                if esp.latched():
                    tapped = True
                    break
                self._watch.check("waiting for the tap")
                self._clock.sleep(TAP_POLL_S)
        finally:
            esp.stow()
        if not tapped:
            raise ProbeError("no tap was seen in %d s. The probe signal is not reaching the ESP: "
                             "check the white wire to GPIO3, and that the probe output is open drain "
                             "(Set probe to open-drain) if it measured 5 V, then try again"
                             % TAP_TIMEOUT_S)
        esp.clear()
        self._watch.check("after the tap test", force_uptime=True)
        self._session.passed(self._watch)
        self._tell("tap", "Tap seen: the probe signal reaches the ESP.", waiting_for_tap=False)

    def _probe_points(self, steps):
        m = self._module
        prober = Prober(self._bridge, self._esp, self._c, steps, aborted=self._abort.is_set,
                        sleep=self._clock.sleep, logger=self._logger, dx=m.get("dx"), dy=m.get("dy"),
                        watch=self._watch,
                        say=lambda text: self._tell("probe", text, done=len(self._measured),
                                                    total=len(self._points)))
        self._esp.show_state("calibrate" if self.mode == "calibrate" else "probing")
        total = len(self._points)
        for i, point in enumerate(self._points):
            self._check()
            self._tell("probe", "Point %d of %d: X%.2f Y%.2f." % (i + 1, total, point["x"], point["y"]),
                       done=i, total=total)
            measured = prober.measure(point["x"], point["y"])
            measured.update(ref=bool(point.get("ref")), datum=bool(point.get("datum")))
            measured["focus"] = results.focus_z(measured["z"], m.get("focus_offset"))
            self._measured.append(measured)
            self._esp.show_z(measured["z"])
            self._esp.show_focus(measured["focus"])
            self._tell("probe", "Point %d of %d: trigger Z%.3f (spread %.3f, n=%d of %d)."
                       % (i + 1, total, measured["z"], measured["spread"], measured["n"],
                          measured["taken"]),
                       done=i + 1, total=total, point=measured)

    def _run_tap(self):
        self._tell("check", "Checking the probe at %s." % self._esp.base_url)
        self._esp.latched()
        self._esp.stow()
        self._watch = Watchdog(self._esp, clock=self._clock, logger=self._logger).start()
        self._tap_test()

    def _run_here(self):
        m = self._module
        watch = self._session.require()          # before anything is sent
        self._bridge.run([LASER_OFF], 30.0)
        self._esp.show_state("probing")
        steps = gcode.z_steps_per_mm(self._bridge.run(["M92"], 30.0))
        prober = Prober(self._bridge, self._esp, self._c, steps, aborted=self._abort.is_set,
                        sleep=self._clock.sleep, logger=self._logger, dx=m.get("dx"), dy=m.get("dy"),
                        watch=watch, say=lambda text: self._tell("probe", text))
        self._here_start = prober.where()[2]
        left_at = self._here_start
        given = self._c.get("here_from")
        descent = self._c["here_max_descent"]
        if given:
            if left_at < given["z"] - 1e-6:
                raise ProbeError("the head is at Z%.3f, below Z%.3f where the height given puts the start "
                                 "(%.1f mm above the top at Z%.3f): raise it first"
                                 % (left_at, given["z"], given["z"] - given["top"], given["top"]))
            self._esp.stow()
            prober._drop_to_travel(given["z"])
            self._here_start = given["z"]
            descent = given["window"]
            self._tell("probe", "From the height given: top expected at Z%.3f, starting at Z%.3f."
                       % (given["top"], given["z"]))
        if not self.xy_calibrated:
            self._tell("check", "Warning: %s has no measured probe XY offset; X, Y are recorded with "
                                "0, 0." % (m.get("name") or "this module"), warning=True)
        square = self._c.get("here_square")
        if square:
            from .probe import SHAPES
            shape = square.get("shape") or "square"
            count = square["n"] if shape in ("x", "y") else square["n"] * square["n"]
            self._tell("probe", "Probing a %.1f mm %s (%d points) from Z%.3f, at most %.1f mm down. The "
                                "head moves in X and Y." % (square["size"], SHAPES.get(shape, shape), count,
                                                           self._here_start, descent))
            sq = prober.probe_square(square["size"], square["n"], descent, shape)
            measured = sq["highest"]
            measured["square"] = dict((k, sq[k]) for k in ("size", "n", "shape", "min", "max", "range",
                                                          "points", "misses"))
        else:
            self._tell("probe", "Probing here from Z%.3f, at most %.1f mm down." % (self._here_start, descent))
            measured = prober.probe_here(descent)
        if given:
            from .probe import TRAVEL_MARGIN
            if measured["z"] > given["z"] - TRAVEL_MARGIN:
                prober._move_z(left_at)
                raise ProbeError("touched at Z%.3f, within %.1f mm of the start: the object is taller than "
                                 "the height given (top expected at Z%.3f). Measure it again, or jog down "
                                 "and probe from there" % (measured["z"], TRAVEL_MARGIN, given["top"]))
            measured["height_given"] = given["top"]
            prober._move_z(left_at)
        measured["focus"] = results.focus_z(measured["z"], m.get("focus_offset"))
        self._measured.append(measured)
        self._esp.show_z(measured["z"])
        self._esp.show_focus(measured["focus"])

    def _refuse_work_offset(self, steps_lines, steps_z):
        """The rotary result is used in machine coordinates (the translator runs in G53), so it
        must be measured with no G92 or work origin in force. LightBurn's flat 10W start G-code
        leaves one (G92 Z0 at its laser height, G92 X0 Y0 at its offset), and it lasts until the
        axes are homed. Probing in that frame and burning in G53 would put the hood off by the
        shift, into the work. Refuse, and say how to clear it."""
        lines = self._bridge.run(["M503"], 30.0)
        shift = gcode.work_offset(lines)
        if shift is None:
            raise ProbeError("cannot read the step counters and home offsets (M114, M503), so a work "
                             "offset cannot be ruled out")
        if max(abs(v) for v in shift) > 0.02:
            raise ProbeError("a work offset is in force (X%+.2f Y%+.2f Z%+.2f beyond the home offsets), "
                             "probably from LightBurn's start G-code. The rotary result is used in "
                             "machine coordinates: home with G28 X Y Z first" % shift)

    def _run_rotary(self):
        """The rotary routine (rotary_job). The head starts wherever it is; every move goes up to
        the routine's travel height before it goes sideways."""
        from . import rotary_job
        m = self._module
        p = rotary_job.params(self._c.get("rotary"))
        watch = self._session.require()
        self._bridge.run([LASER_OFF, "M211 S1", "G90"], 30.0)
        # No home here: X homes ~12 mm differently with the probe on, so the frame must come from a
        # home made with the 10W seated (the plugin refuses the job otherwise).
        self._esp.show_state("probing")
        steps_lines = self._bridge.run(["M92"], 30.0)
        steps = gcode.z_steps_per_mm(steps_lines)
        self._refuse_work_offset(steps_lines, steps)
        prober = Prober(self._bridge, self._esp, self._c, steps, aborted=self._abort.is_set,
                        sleep=self._clock.sleep, logger=self._logger, dx=m.get("dx"), dy=m.get("dy"),
                        watch=watch, say=lambda text: self._tell("probe", text))
        self._here_start = prober.where()[2]
        job = rotary_job.Rotary(prober, p, lambda text: self._tell("rotary", text), self._check)
        try:
            result = job.run()
        except Exception:
            if job.out is not None:
                job.out["touches"] = ((p.get("resume") or {}).get("touches") or []) + job.log
                self._rotary_partial = job.out
            raise
        result["finished"] = time.time()
        result["module"] = m.get("name")
        result["focus_offset"] = m.get("focus_offset")
        return result

    def _safe_end(self):
        """Up to a safe height and pin stowed. Returns True only if both are confirmed."""
        c = self._c
        safe = True
        if self.mode == "tap":
            pass                                             # nothing moved
        elif self.mode in ("here", "rotary"):
            if self._here_start is not None:
                safe = raise_to_safe(self._bridge, self._here_start, c["z_feed"], self._logger)
        elif c.get("safe_z") is not None:
            safe = raise_to_safe(self._bridge, c["safe_z"], c["z_feed"], self._logger,
                                 prefix=["M211 S1"])
        try:
            self._esp.stow()
        except Exception:                                    # noqa: BLE001
            safe = False
            if self._logger:
                self._logger.exception("laserheightprobe: could not stow the pin")
        return safe

    # -- the thread -------------------------------------------------------

    def run(self):
        started = self._clock.time()
        outcome = None
        safe = False
        try:
            outcome = self._run(started)
        finally:
            # Always, even for an error nobody expected. Say the outcome only once it is safe.
            safe = self._safe_end()
            kind = (outcome or {}).get("type")
            self._esp.show_state({"done": "done", "failed": "error"}.get(kind, "idle"))
        if not safe and outcome is not None:
            # Never claim a safe height that was not confirmed. A tap test moves nothing, so
            # only the pin is in doubt there.
            if self.mode == "tap":
                warning = (" The plugin could NOT confirm that the pin is stowed: check it before"
                           " you probe.")
            else:
                warning = (" The plugin could NOT confirm that the head reached a safe height and"
                           " the pin is stowed: check the machine before you touch anything.")
            outcome = dict(outcome, unconfirmed=True, message=(
                (outcome.get("message") or "Finished.").replace(
                    " The head is at a safe height and the pin is stowed.", "") + warning))
        self._say(outcome)

    def _run(self, started):
        try:
            if self.mode == "tap":
                self._run_tap()
                self.phase = "done"
                return dict(type="done", mode=self.mode, result=None)
            if self.mode == "here":
                self._run_here()
                self.result = self._result(started, complete=True)
                self.phase = "done"
                return dict(type="done", mode=self.mode, result=self.result)
            if self.mode == "rotary":
                self.result = self._run_rotary()
                self.phase = "done"
                return dict(type="done", mode=self.mode, result=self.result)
            if not self._points:
                raise ValueError("there are no points to measure")
            if self._c.get("safe_z") is None or self._c.get("z_floor") is None:
                raise ValueError("safe_z and z_floor must be set first")
            steps = self._prepare()
            self._probe_points(steps)
            self.result = self._result(started, complete=True)
            self.phase = "done"
            return dict(type="done", mode=self.mode, result=self.result)
        except Exception as exception:                       # noqa: BLE001
            if isinstance(exception, EspRebooted):
                self._session.invalidate(str(exception))
            partial = self._result(started, complete=False) if self._measured else None
            if self.mode == "rotary":
                partial = self._rotary_partial
            self.result = partial
            # An abort also wakes the bridge, which then raises from inside its wait.
            if isinstance(exception, Aborted) or self._abort.is_set():
                self.phase, self.error = "aborted", "aborted"
                return dict(type="aborted", mode=self.mode, result=partial,
                            message="Stopped. The head is at a safe height and the pin is stowed.")
            self.phase, self.error = "failed", str(exception)
            if self._logger:
                self._logger.exception("laserheightprobe: job failed")
            return dict(type="failed", mode=self.mode, result=partial, message=str(exception))
