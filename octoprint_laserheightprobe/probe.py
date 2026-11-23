# coding=utf-8
"""Measuring one point: a staged step-down with the BLTouch-style probe.

Positions are LASER positions: where the beam of the selected module will be. The pin sits
off that beam by the module's (dx, dy), so the head goes to (X - dx, Y - dy) to put the pin
over the target.

The descent is a staircase, not one long probing move. Each step is one G0 Z (at the probe
feed) through the bridge, which ends with M400 and M114, and only then is the ESP's latch read. So the head
never travels more than one step past the trigger, and the ESP's interrupt latch means the
short pulse the probe gives cannot fall between two reads. G30 and M48 are not used: the
firmware is in laser mode with no probe configured, and a probing move that missed its
trigger would carry on down into the object. Probing steps are G0, not G1: in laser mode G0
never fires the beam, even if a power level was left set.

Two ways in. measure() is the automatic job: travel over a laser position and probe it.
probe_here() is the hand workflow: the user has jogged the head over the spot, and the plugin
only lowers it from where it is (no XY move at all), then goes back up to where it started.

No unprobed move goes down, with one exception. Travel happens at max(current Z, safe_z): a
head above safe_z travels where it is and probes its way down from there. The only downward
moves are the probing steps; every other Z move (to a stage start, back to travel height) is
checked to be upward before it is sent, and refused otherwise (_move_z). The exception is
probe_at(drop=True), the rotary routine's touch: a head above that touch's travel height drops
to it in one move, pin stowed (_drop_to_travel). The routine asks for that only over the fixed
rotary housing, or over the object once a touch in the same run has confirmed its top; the
first touch on the object always probes its way down. Travel is that top plus a clearance, the
height the routine already crosses the object at, and a touch that trips within TRAVEL_MARGIN
of it fails before anything travels that low. safe_z and z_floor come from the
machine itself (the user captures them), because the laser-mode Z frame was never measured.

Stages go from coarse to fine (default 0.5, 0.1, 0.02 mm). In its default mode the probe
stows its own pin on a trigger, so between stages the head rises to (trigger + start_above)
and the pin is armed again. From there a stage approaches at the coarser stage's step, down to
just above where that stage last did not trip, and only then takes its own steps (2026-11-19:
the fine stage had been stepping ~2 mm at 0.02 mm, ~97 steps a sample). The approach steps
are never larger than a step already taken towards this surface, and stop above it, so they add
no reach; a trip during one is a new bracket and the stage starts again from it. The final value
is the commanded Z of the fine-stage trigger step. The probe trips somewhere in [z, z + fine_step): z did trip and z + fine_step did not.
That bias is the same at every point and in the calibration, so it cancels out of heights,
focus Z and LightBurn offsets alike.

Limits, all checked before a move is sent, never after:

- never below z_floor (soft endstops clamp Z at 0, and a clamped move would read as a
  descent that did not happen);
- never more than max_descent below safe_z;
- a refinement stage, and every repeat sample, may go at most one coarser step below the
  coarser trigger. The arm check cannot see one case: a pin deployed straight onto the
  surface trips, stows itself, and the latch_clear that follows the deploy erases the
  evidence. The window turns that case into "no trigger" a fraction of a millimetre later,
  instead of a fine-step crawl all the way down to z_floor.

A watchdog (esp.Watchdog) reads the device's counters after every step and stops the point if
the device restarted, because a restart can lose a trigger from the latch.

Whatever happens, the head goes back up to travel height and the pin is stowed, each in its
own try block, so a failure of one cannot skip the other or hide the first error.
"""
from __future__ import absolute_import

import math
import time

from . import gcode
from .esp import RESET_SETTLE_S

# M114 prints two decimals on the Snapmaker 2.0 ("Z:-1.12"), and moves are sent with three, so
# a read-back can be 0.005 off. Kept below the default 0.02 fine step, so a missed step shows.
POSITION_TOLERANCE = 0.015
# ESPHome's web server defers a button press to its main loop, and a template binary_sensor
# publishes its state from that loop. A read straight after latch_clear can return the old
# latch for a loop or two (longer while the OLED redraws), and would look like an alarm.
LATCH_SETTLE_S = 0.15
# How long the safe end waits for the head to reach travel height before it gives up waiting.
SAFE_END_TIMEOUT = 120.0
# If the position cannot be read at the safe end, the head goes up by this much, relatively.
BLIND_RAISE = 5.0
# trigger_count is a template sensor published every 250 ms, so it is read this long after the
# deploy has finished, to be sure a touch during the deploy has reached it.
COUNT_SETTLE_S = 0.3
# A refinement stage whose deploy touches climbs this much and deploys again, this many times.
DEPLOY_CLIMB = 1.0
DEPLOY_RETRIES = 3
# A refinement stage approaches at the coarser stage's step down to this many of its own steps
# above where the coarser stage last did not trip (surface repeatability is about one fine step).
APPROACH_MARGIN_STEPS = 2
# A stage whose approach trips starts again from that trip at most this many times, then
# falls back to stepping all of it at its own step.
APPROACH_RESTARTS = 2
# probe_at: a coarse trigger this close under the travel height means travel does not clear the object.
TRAVEL_MARGIN = 1.0


class Aborted(Exception):
    """The user pressed Abort."""


class ProbeError(Exception):
    """The measurement could not be made safely or did not agree with itself."""


class NoTrigger(ProbeError):
    """A stage reached its window without a trigger: nothing under the pin within reach."""


class DeployTouch(ProbeError):
    """The pin tripped while it deployed: the surface was within the pin's deploy overshoot."""


class ApproachTrip(ProbeError):
    """The pin tripped during a refinement stage's approach: z is a new coarser bracket."""

    def __init__(self, z, machine=None):
        ProbeError.__init__(self, "tripped during the approach at Z%.3f" % z)
        self.z = z
        self.machine = machine


class NoAgreement(ProbeError):
    """The repeat samples never formed a clear group within the tolerance."""


SHAPES = dict(square="square", diamond="diamond", x="line along X", y="line along Y")


def pattern_offsets(shape, size, n):
    """(dx, dy) offsets from where the user jogged, in travel order.

    square: n x n over size x size, in a serpentine. diamond: the same grid turned 45 degrees,
    its corners size/2 out along X and Y. x / y: n points over size along that axis only.
    """
    if shape not in SHAPES:
        raise ProbeError("the pattern must be one of %s" % ", ".join(sorted(SHAPES)))
    if n < 2:
        raise ProbeError("a pattern needs at least 2 points per side")
    steps = [-1.0 + 2.0 * i / (n - 1) for i in range(n)]
    half = size / 2.0
    if shape == "x":
        return [(round(u * half, 3), 0.0) for u in steps]
    if shape == "y":
        return [(0.0, round(v * half, 3)) for v in steps]
    order = []
    for j, v in enumerate(steps):
        row = steps if j % 2 == 0 else list(reversed(steps))
        for u in row:
            if shape == "square":
                order.append((round(u * half, 3), round(v * half, 3)))
            else:
                order.append((round((u + v) / 2.0 * half, 3), round((u - v) / 2.0 * half, 3)))
    return order


class Prober(object):
    """Measures points. `config` is the checked settings from settings.routine_config().

    dx, dy: the selected module's pin offset (pin minus beam). watch: an esp.Watchdog that has
    been started, or None.
    """

    def __init__(self, bridge, esp, config, steps_per_mm=400.0, aborted=lambda: False,
                 sleep=time.sleep, say=None, logger=None, dx=0.0, dy=0.0, watch=None):
        self._bridge = bridge
        self._esp = esp
        self._c = config
        self._steps = steps_per_mm
        self._aborted = aborted
        self._sleep = sleep
        self._say = say or (lambda text: None)
        self._logger = logger
        self._dx = dx or 0.0
        self._dy = dy or 0.0
        self._watch = watch
        self._z = None                    # the head's Z after the last confirmed move

    # -- helpers ----------------------------------------------------------

    def _check(self):
        if self._aborted():
            raise Aborted()

    def _wait(self, seconds):
        """Sleep, but notice an abort within 0.1 s."""
        end = seconds
        while end > 0:
            self._check()
            chunk = min(0.1, end)
            self._sleep(chunk)
            end -= chunk
        self._check()

    def _run(self, commands, timeout=60.0):
        # The bridge clears its interrupt flag on entry, so an abort pressed between two
        # batches is only seen here.
        self._check()
        return self._bridge.run(commands, timeout)

    @staticmethod
    def _verify(lines, x=None, y=None, z=None):
        """Refuse to go on if the printer is not where it was told to go.

        Soft endstops clamp a move silently; without this a clamped Z would be recorded as a
        height the head never reached.
        """
        found = gcode.position(lines)
        if found is None:
            raise ProbeError("M114 did not report the position")
        for axis, wanted, got in (("X", x, found[0]), ("Y", y, found[1]), ("Z", z, found[2])):
            if wanted is not None and abs(got - wanted) > POSITION_TOLERANCE:
                raise ProbeError("the printer reports %s%.3f, not %s%.3f. A soft endstop may have "
                                 "clamped the move" % (axis, got, axis, wanted))
        return found

    def _move_z(self, z, probing=False):
        """One Z move. A probing step is G0 at probe_feed; any other move is G0 at z_feed and
        must go up: an unprobed downward move is refused before it is sent."""
        if probing:
            command = "G0 Z%.3f F%d" % (z, self._c["probe_feed"])
        else:
            if self._z is None or z < self._z - 1e-6:
                raise ProbeError("refusing an unprobed downward move to Z%.3f from Z%s"
                                 % (z, "?" if self._z is None else "%.3f" % self._z))
            command = "G0 Z%.3f F%d" % (z, self._c["z_feed"])
        lines = self._run([command])
        self._verify(lines, z=z)
        self._z = z
        return lines

    # -- the probe --------------------------------------------------------

    def arm(self, where, verify=False):
        """Deploy the pin and prove it is out and clear of the surface.

        verify (the first arm of every point): reset the probe first, then prove the pin
        position with touch mode, up before the deploy and down after it (2026-11-19: twelve
        descents in a row got no trigger, and nothing here could say whether the pin was even
        out). It cannot prove the pin trips on contact: only the tap test does that.

        trigger_count is read before the deploy and again after it, before the latch is
        cleared. A pin deployed onto something trips and stows itself at once; the latch clear
        would hide that, but the count still shows it.
        """
        self._check()
        if self._watch is not None:
            self._watch.check(where, force_uptime=True)
        self._esp.wait_idle(self._wait)       # the firmware's auto-reset after the last trigger
        if verify and self._c.get("verify_pin", True):
            healthy = False
            for stow_first, settle in ((False, RESET_SETTLE_S), (False, 2 * RESET_SETTLE_S), (True, 2 * RESET_SETTLE_S)):
                # A second reset with a longer wait before calling it an alarm: 2026-11-19 the
                # first version refused 2 of 3 healthy in-air runs. 2026-11-22 two resets left it in
                # alarm near the chuck and a stow, then a reset, cleared it at once: so a third try.
                if stow_first:
                    self._esp.stow()
                    self._wait(RESET_SETTLE_S)
                self._esp.reset()
                self._wait(settle)
                self._esp.wait_idle(self._wait)
                if self._esp.touch_read(self._wait):
                    healthy = True
                    break
            if not healthy:
                raise ProbeError("the probe stays in alarm after two resets and a stow (%s): with the pin up, touch "
                                 "mode should read HIGH. Check the probe's LED and pin, or switch off "
                                 "verify_pin" % where)
            self._esp.reset()                 # out of touch mode, pin up, output LOW
            self._wait(RESET_SETTLE_S)
            self._esp.wait_idle(self._wait)
        if not self._esp.release_held_trigger(self._wait):
            raise ProbeError("probe output stays HIGH after a reset (%s): the probe is in alarm. "
                             "Check its LED, then raise Z a little and retry" % where)
        before = self._esp.trigger_count()
        self._esp.arm()
        self._wait(self._c["arm_wait_s"] + COUNT_SETTLE_S)
        self._esp.wait_idle(self._wait)       # the deploy itself has been sent
        after = self._esp.trigger_count()
        if after != before:
            try:
                self._esp.reset()
            except Exception:                              # noqa: BLE001
                if self._logger:
                    self._logger.exception("laserheightprobe: probe_reset failed")
            if after < before:
                from .esp import EspRebooted
                raise EspRebooted("the probe device restarted while deploying (%s): trigger_count "
                                  "went from %d to %d" % (where, before, after))
            raise DeployTouch("pin touched while deploying: raise Z and retry (%s, trigger_count "
                              "%d -> %d)" % (where, before, after))
        # A HIGH output now is the ordinary alarm (deployed onto something), reported below with
        # its "raise Z" advice; the touch-mode check is for the silent case, output LOW.
        if verify and self._c.get("verify_pin", True) and not self._esp.raw():
            if self._esp.touch_read(self._wait):
                try:
                    self._esp.reset()
                except Exception:                          # noqa: BLE001
                    if self._logger:
                        self._logger.exception("laserheightprobe: probe_reset failed")
                raise ProbeError("the pin did not deploy (%s): touch mode reads it up. Check the pin: "
                                 "bent, sticking or blocked" % where)
            self._esp.deploy()                # back to normal mode; the pin is already down
            self._wait(self._c["arm_wait_s"])
            self._esp.wait_idle(self._wait)
        self._esp.clear()
        self._wait(LATCH_SETTLE_S)
        raw = self._esp.raw()
        latched = self._esp.latched()
        if raw or latched:
            try:
                self._esp.reset()
            except Exception:                              # noqa: BLE001
                if self._logger:
                    self._logger.exception("laserheightprobe: probe_reset failed")
            raise ProbeError("probe alarm: pin can't deploy: raise Z a little and retry (%s, output "
                             "%s, latch %s). If this happens at a fine stage, raise its start_above"
                             % (where, "HIGH" if raw else "LOW", "set" if latched else "clear"))

    def _arm_from(self, start, label, retries=DEPLOY_RETRIES):
        """Go up to `start` and deploy. The clone's pin overshoots as it deploys, by a varying
        amount: a deploy that touches climbs DEPLOY_CLIMB and tries again. Returns the Z armed at.
        Only ever moves up, so the next descent still starts above the last trigger."""
        for attempt in range(retries + 1):
            self._move_z(start)
            try:
                self.arm("%s, Z%.3f" % (label, start))
                return start
            except DeployTouch:
                if attempt == retries:
                    raise
                self._say("%s: the pin touched while deploying at Z%.3f; climbing %.1f mm."
                          % (label, start, DEPLOY_CLIMB))
                start = round(start + DEPLOY_CLIMB, 3)

    def _approach_then_descend(self, start, approach_step, approach_to, step, limit, label, plain=False):
        """Step down from `start` at approach_step while the next step stays at or above
        `approach_to` (a height the coarser stage did not trip at, plus a margin), then at
        `step` until the latch is set. Returns (z, machine z). A trip during the approach raises
        ApproachTrip(z). plain=True, or no approach_to, is the old all-fine descent.

        The approach steps are the coarser stage's own size, never larger, and stop above
        where it last did not trip, so the pin meets nothing a step can overrun. The heights
        stay on the fine grid when approach_step is a whole number of steps."""
        z = start
        if not plain and approach_to is not None:
            n = int(math.floor((start - approach_to) / approach_step + 1e-9))
            for _ in range(max(0, n)):
                self._check()
                z = round(z - approach_step, 3)
                if z < limit - 1e-9:
                    break
                lines = self._move_z(z, probing=True)
                latched = self._esp.latched()
                if self._watch is not None:
                    self._watch.check("%s approach, Z%.3f" % (label, z))
                if latched:
                    raise ApproachTrip(z, gcode.count_z(lines, self._steps))
        found, machine, _ = self.descend(z, step, limit, label)
        return found, machine

    def descend(self, start, step, limit, label):
        """Step down from `start` until the latch is set. Returns (z, machine z, steps taken).

        `limit` is the lowest Z this stage may command; the next step is checked against it
        before it is sent.
        """
        z = start
        taken = 0
        while True:
            self._check()
            target = round(z - step, 3)
            if target < limit - 1e-9:
                raise NoTrigger("no trigger in the %s stage: the next step, Z%.3f, would pass "
                                 "Z%.3f" % (label, target, limit))
            z = target
            lines = self._move_z(z, probing=True)
            taken += 1
            latched = self._esp.latched()
            if self._watch is not None:
                # After the latch read: a restart between the trigger and that read shows here.
                self._watch.check("%s stage, Z%.3f" % (label, z))
            if latched:
                return z, gcode.count_z(lines, self._steps), taken

    def where(self):
        """Read the head's position (M114, after M400) and remember its Z."""
        found = self._verify(self._run([], 30.0))
        self._z = found[2]
        return found

    def travel_height(self):
        return max(self._z, self._c["safe_z"])

    def measure(self, x, y):
        """Measure the laser position (x, y). Returns a dict for the results table."""
        c = self._c
        hx, hy = round(x - self._dx, 3), round(y - self._dy, 3)
        ok = False
        try:
            self.where()
            travel = self.travel_height()
            if travel > self._z + 1e-6:
                self._move_z(travel)
            lines = self._run(["G90", "G0 X%.3f Y%.3f F%d" % (hx, hy, c["xy_feed"])], 180.0)
            self._verify(lines, x=hx, y=hy, z=self._z)
            floor = max(c["z_floor"], round(c["safe_z"] - c["max_descent"], 3))
            result = self._measure_here(x, y, hx, hy, start=self._z, floor=floor,
                                        ceiling=c["safe_z"])
            self._move_z(self.travel_height())
            # Fail closed on success too: a pin that will not stow travels to the next point.
            self._esp.stow()
            ok = True
            return result
        finally:
            if not ok:
                self.make_safe()

    def probe_here(self, max_descent):
        """The hand workflow: lower the head from where it is, measure, and go back up.

        No XY move is sent. No Z move goes below the start except the probing steps; stage
        restarts stay above the previous trigger. The window is max_descent below the start,
        and z_floor too when it is set. Returns the result, with X, Y as the laser position
        (head + the module's pin offset) and Z_start recorded.
        """
        c = self._c
        hx, hy, z_start = self.where()
        floor = self._here_floor(z_start, max_descent)
        ok = False
        try:
            result = self._measure_here(round(hx + self._dx, 3), round(hy + self._dy, 3), hx, hy,
                                        start=z_start, floor=floor)
            self._move_z(z_start)
            self._esp.stow()
            result["z_start"] = z_start
            ok = True
            return result
        finally:
            if not ok:
                raise_to_safe(self._bridge, z_start, c["z_feed"], self._logger)
                try:
                    self._esp.stow()
                except Exception:                          # noqa: BLE001
                    if self._logger:
                        self._logger.exception("laserheightprobe: could not stow the pin")

    def _here_floor(self, z_start, max_descent):
        c = self._c
        floor = round(z_start - max_descent, 3)
        if c.get("z_floor") is not None:
            floor = max(floor, c["z_floor"])
        if floor > round(z_start - c["stages"][0][0], 3) + 1e-9:
            raise ProbeError("Z%.3f is too close to the Z floor (%.3f) to probe: raise Z" % (z_start, floor))
        return floor

    def _drop_to_travel(self, z):
        """The one unprobed downward move: to a travel height the caller knows is clear (see
        probe_at), with the pin stowed."""
        lines = self._run(["G0 Z%.3f F%d" % (z, self._c["z_feed"])])
        self._verify(lines, z=z)
        self._z = z
        return lines

    def _travel_xy(self, hx, hy, z):
        lines = self._run(["G90", "G0 X%.3f Y%.3f F%d" % (hx, hy, self._c["xy_feed"])], 180.0)
        self._verify(lines, x=hx, y=hy, z=z)

    def probe_at(self, x, y, travel, floor, head=False, drop=False):
        """Rotary probing: go up to `travel`, move to the laser position (x, y), probe no lower
        than `floor`, and come back to `travel` with the pin stowed.

        With `drop`, a head above `travel` moves across where it is, then drops to `travel` in
        one move with the pin stowed; without it, it probes its way down from where it is. The
        caller passes drop only when `travel` is known clear air at (x, y): the fixed rotary
        housing, or an object whose top a touch in this same run has already confirmed (a crown
        from an earlier object must never drive the head into a taller one). A coarse trigger within TRAVEL_MARGIN
        of `travel` fails the point (the head backs up the column it probed), so a wrong top
        stops the job at its first touch. Probing from the head's parked height instead cost
        2.5 min a touch on the A350 (Z313 down to a Z133 housing, 0.5 mm a step).
        head: (x, y) is where the head goes, whatever the module's pin offset (a fixed feature
        of the machine, like the rotary housing).

        Returns the result dict, or None when nothing was under the pin above `floor` (a miss:
        how the end of an object is found). Any other failure raises, after make_safe_at.
        """
        if head:
            hx, hy = round(x, 3), round(y, 3)
        else:
            hx, hy = round(x - self._dx, 3), round(y - self._dy, 3)
        ok = False
        try:
            self.where()
            if travel > self._z + 1e-6:
                self._move_z(travel)
            self._esp.stow()
            self._travel_xy(hx, hy, self._z)
            if drop and self._z > travel + 1e-6:
                self._drop_to_travel(travel)
            if self._c.get("z_floor") is not None:
                floor = max(floor, self._c["z_floor"])
            if floor > round(self._z - self._c["stages"][0][0], 3) + 1e-9:
                raise ProbeError("X%.2f Y%.2f: the floor Z%.3f is too close below travel Z%.3f"
                                 % (x, y, floor, self._z))
            start = self._z
            try:
                result = self._measure_here(x, y, hx, hy, start=start, floor=floor,
                                            ceiling=round(travel - TRAVEL_MARGIN, 3), ceiling_is="travel")
            except NoTrigger:
                result = None
                self._say("X%.2f Y%.2f: nothing above Z%.3f." % (x, y, floor))
            self._move_z(max(start, travel))
            self._esp.stow()
            ok = True
            return result
        finally:
            if not ok:
                self.make_safe_at(travel)

    @property
    def coarse_step(self):
        return self._c["stages"][0][0]

    def lift_to(self, z):
        """Up to `z` if below it (never down), pin stowed first."""
        self.where()
        self._esp.stow()
        if z > self._z + 1e-6:
            self._move_z(z)

    def soft_endstops(self, on):
        """M211 S1 / S0. Off only for the rotary reference touch past Y320 (the endstop
        itself is at about Y322.97); the M114 read-back after each move still applies."""
        self._run(["M211 S%d" % (1 if on else 0)], 30.0)

    def rehome_y(self):
        """G28 Y at the current height (Y homes to its +Y endstop, nothing else moves), then
        read the position back. Restores Y after work past the soft limit."""
        self._run(["G28 Y", "M400"], 120.0)
        return self.where()

    def where_b(self):
        """The rotary's B in degrees (M114), or None without a rotary."""
        return gcode.b_position(self._run([], 30.0))

    def turn_b(self, degrees, travel):
        """Turn the rotary to B `degrees` with the head at `travel` or above and the pin stowed."""
        self.where()
        if travel > self._z + 1e-6:
            self._move_z(travel)
        self._esp.stow()
        lines = self._run(["G90", "G0 B%.3f F1200" % degrees], 120.0)
        found = gcode.position(lines)
        if found is None:
            raise ProbeError("could not read the position after turning B")
        b = gcode.b_position(lines)
        if b is not None and abs(b - degrees) > 0.05:
            raise ProbeError("B is at %.3f, not %.3f" % (b, degrees))

    def make_safe_at(self, travel):
        """Up to `travel` (or above) and stow, each on its own. Never raises."""
        raise_to_safe(self._bridge, travel, self._c["z_feed"], self._logger)
        try:
            self._esp.stow()
        except Exception:                                  # noqa: BLE001
            if self._logger:
                self._logger.exception("laserheightprobe: could not stow the pin")

    def probe_square(self, size, n, max_descent, shape="square"):
        """Probe here over a size x size square of n x n points centred where the user jogged.

        Unlike probe_here this moves X and Y: point to point at the start Z, pin stowed, in a
        serpentine. Each point is the same staged descent as probe_here, within the same
        window below the start. A point with nothing under it (NoTrigger) or with samples that
        never agree (NoAgreement) is a miss and the square goes on; anything else (a touch on
        deploy, an alarm, a device restart) stops it, because the object is not where the
        start height assumed. The head ends back at the start X, Y, Z.

        Returns dict(highest, points, misses, size, n, min, max, range).
        """
        c = self._c
        hx0, hy0, z_start = self.where()
        floor = self._here_floor(z_start, max_descent)
        order = pattern_offsets(shape, size, n)
        points, misses = [], []
        ok = False
        try:
            for k, (ox, oy) in enumerate(order):
                self._check()
                hx, hy = round(hx0 + ox, 3), round(hy0 + oy, 3)
                self._travel_xy(hx, hy, z_start)
                x, y = round(hx + self._dx, 3), round(hy + self._dy, 3)
                self._say("Point %d of %d: X%.2f Y%.2f." % (k + 1, len(order), x, y))
                try:
                    points.append(self._measure_here(x, y, hx, hy, start=z_start, floor=floor))
                except (NoTrigger, NoAgreement) as miss:
                    misses.append(dict(x=x, y=y, error=str(miss)))
                    self._say("Point %d of %d: missed (%s)." % (k + 1, len(order), miss))
                self._move_z(z_start)
                self._esp.stow()
            self._travel_xy(hx0, hy0, z_start)
            if not points:
                raise ProbeError("no point of the %.1f mm %s triggered within %.1f mm below "
                                 "Z%.3f" % (size, SHAPES[shape], max_descent, z_start))
            zs = [p["z"] for p in points]
            highest = dict(max(points, key=lambda p: p["z"]))
            highest["z_start"] = z_start
            ok = True
            return dict(highest=highest, points=points, misses=misses, size=size, n=n, shape=shape,
                        min=min(zs), max=max(zs), range=round(max(zs) - min(zs), 4))
        finally:
            if not ok:
                raise_to_safe(self._bridge, z_start, c["z_feed"], self._logger)
                try:
                    self._esp.stow()
                except Exception:                          # noqa: BLE001
                    if self._logger:
                        self._logger.exception("laserheightprobe: could not stow the pin")

    def make_safe(self):
        """Back up to travel height and stow, each on its own. Never raises."""
        raise_to_safe(self._bridge, self._c["safe_z"], self._c["z_feed"], self._logger)
        try:
            self._esp.stow()
        except Exception:                                  # noqa: BLE001
            if self._logger:
                self._logger.exception("laserheightprobe: could not stow the pin")

    def _measure_here(self, x, y, hx, hy, start, floor, ceiling=None, ceiling_is="safe_z"):
        """Stages and samples from `start`, never below `floor`. With `ceiling` (safe Z in the
        automatic job, travel - TRAVEL_MARGIN in probe_at), a trigger at or above it fails the
        point."""
        c = self._c
        stages = c["stages"]
        triggers = []

        self.arm("at the start, Z%.3f" % start, verify=True)
        z, machine, _ = self.descend(start, stages[0][0], floor, "coarse")
        if ceiling is not None and z >= ceiling - 1e-9:
            # The object reaches safe Z here, so travel at safe Z would skim it with only the
            # pin stroke to spare. Back up the column just probed clear, and stop.
            self._move_z(start)
            if ceiling_is == "travel":
                raise ProbeError("X%.2f Y%.2f: the probe touched at Z%.3f, within %.1f mm of the travel "
                                 "height Z%.3f. The crown given is too low for this object: probe the "
                                 "top by hand and give that" % (x, y, z, TRAVEL_MARGIN, start))
            raise ProbeError("X%.2f Y%.2f: the probe touched at Z%.3f, at or above safe Z (%.3f). "
                             "Safe Z does not clear the object: capture a higher one"
                             % (x, y, z, ceiling))
        triggers.append(z)
        self._say("X%.2f Y%.2f: coarse trigger at Z%.3f." % (x, y, z))

        window = floor
        for i in range(1, len(stages)):
            step, above = stages[i]
            previous_step = stages[i - 1][0]
            label = "stage %d" % (i + 1)
            for restart in range(APPROACH_RESTARTS + 1):
                window = max(floor, round(z - previous_step, 3))
                stage_start = self._arm_from(round(z + above, 3), label)
                try:
                    margin = max(APPROACH_MARGIN_STEPS * step, c["tolerance"])
                    z, machine = self._approach_then_descend(
                        stage_start, previous_step, z + previous_step + margin,
                        step, window, label, plain=restart == APPROACH_RESTARTS)
                    break
                except ApproachTrip as trip:
                    # The surface reads higher than the coarser stage found: that trip is a new
                    # bracket of the same size, so the stage starts again from it.
                    self._say("X%.2f Y%.2f: %s tripped on the way down at Z%.3f; starting it again "
                              "from there." % (x, y, label, trip.z))
                    z = trip.z
            triggers.append(z)

        fine_step, fine_above = stages[-1]
        first = z
        # Repeats start a whole number of fine steps above the first fine trigger, so every
        # sample is taken on the same Z grid. From any other start, quantisation alone would
        # make two perfect touches differ by one fine step.
        repeat_start = round(first + math.ceil(fine_above / fine_step - 1e-9) * fine_step, 3)
        samples, machines = [z], [machine]
        while True:
            group, ambiguous = agreeing(samples, c["tolerance"], need=c["samples"])
            if len(group) >= c["samples"] and not ambiguous:
                break
            if len(samples) >= c["max_samples"]:
                raise NoAgreement("X%.2f Y%.2f: no clear group of %d of the %d samples agrees within "
                                 "the %.3f mm tolerance: %s"
                                 % (x, y, c["samples"], len(samples), c["tolerance"],
                                    ", ".join("%.3f" % s for s in samples)))
            label = "repeat %d" % (len(samples) + 1)
            start_here = self._arm_from(repeat_start, label)
            approach = stages[-2][0]
            try:
                # Fine steps over the last tolerance + margin above the first sample, so any
                # sample that could agree with it is measured at the fine step.
                z, machine = self._approach_then_descend(
                    start_here, approach,
                    first + fine_step + c["tolerance"] + APPROACH_MARGIN_STEPS * fine_step,
                    fine_step, window, label)
            except ApproachTrip as trip:
                # More than the tolerance above the first sample: it cannot agree. Keep it as
                # a sample at the trip height (a lower bound), so the agreement count sees it.
                z, machine = trip.z, trip.machine
            samples.append(z)
            machines.append(machine)

        kept = [samples[i] for i in group]
        kept_machine = [machines[i] for i in group if machines[i] is not None]
        mean = sum(kept) / len(kept)
        result = dict(
            x=x, y=y, head_x=hx, head_y=hy, dx=self._dx, dy=self._dy,
            z=round(mean, 4), spread=round(max(kept) - min(kept), 4), n=len(kept),
            taken=len(samples), samples=samples,
            rejected=[samples[i] for i in range(len(samples)) if i not in group],
            stage_triggers=triggers, fine_step=fine_step,
            machine_z=round(sum(kept_machine) / len(kept_machine), 4) if kept_machine else None,
        )
        self._say("X%.2f Y%.2f: Z%.3f, spread %.3f mm, n=%d of %d%s."
                  % (x, y, result["z"], result["spread"], result["n"], result["taken"],
                     "" if not result["rejected"] else ", rejected " +
                     ", ".join("%.3f" % r for r in result["rejected"])))
        return result


def raise_to_safe(bridge, safe_z, feed, logger=None, prefix=()):
    """Go up to max(current Z, safe_z) and wait until the move has run. Returns True only when
    the head is confirmed at a safe height. Never raises.

    The current Z is read from M114 first, so this never moves down: a head above safe_z
    stays where it is. run() clears the bridge's interrupt flag, so a read or a move cut short
    by a second Abort is tried once more. If the position is known, the fallback is the
    absolute raise to safe_z, which is known to go up. Only when the position cannot be read at
    all is the head sent up BLIND_RAISE mm relatively, without waiting.
    """
    found = None
    for attempt in range(2):
        try:
            found = gcode.position(bridge.run(list(prefix) if attempt == 0 else [], 30.0))
            if found is not None:
                break
        except Exception:                                  # noqa: BLE001
            if logger:
                logger.warning("laserheightprobe: could not read the position for the safe end")
    if found is None:
        if logger:
            logger.warning("laserheightprobe: position unknown, raising %g mm blind" % BLIND_RAISE)
        try:
            bridge.send(list(prefix) + ["G91", "G0 Z%g F%d" % (BLIND_RAISE, feed), "G90"])
        except Exception:                                  # noqa: BLE001
            if logger:
                logger.exception("laserheightprobe: could not raise the head")
        return False
    if found[2] >= safe_z - 1e-6:
        return True
    move = ["G90", "G0 Z%.3f F%d" % (safe_z, feed)]
    for attempt in range(2):
        try:
            bridge.run(move, SAFE_END_TIMEOUT)
            return True
        except Exception:                                  # noqa: BLE001
            if logger:
                logger.warning("laserheightprobe: could not confirm the raise to safe Z")
    try:
        bridge.send(move)
    except Exception:                                      # noqa: BLE001
        if logger:
            logger.exception("laserheightprobe: could not raise the head")
    return False


def agreeing(samples, tolerance, need=None):
    """The largest group of samples within `tolerance` of each other, and whether it is clear.

    Returns (indices, ambiguous). max - min over every sample can only grow as samples are
    added, so "take more samples when they disagree" needs this instead: one stray touch is
    outvoted by the samples that agree, and is reported as rejected. When two different groups
    of the same size qualify (12.72, 12.76, 12.80 at 0.05), the samples do not say which is
    right, so the group is ambiguous and the caller must take another sample or fail.

    With `need`: if every qualifying group shares a core of at least `need` samples, the core
    is the answer (143.44, 143.46, 143.46, 143.50 at 0.05: both groups of three hold the two
    143.46 touches, so the samples do agree; seen on the A350 2026-11-17).
    """
    order = sorted(range(len(samples)), key=lambda i: samples[i])
    groups = []
    for a in range(len(order)):
        groups.append(sorted(i for i in order[a:]
                             if samples[i] - samples[order[a]] <= tolerance + 1e-9))
    size = max(len(g) for g in groups) if groups else 0
    best = [g for g in groups if len(g) == size]
    distinct = [g for k, g in enumerate(best) if g not in best[:k]]
    if len(distinct) > 1 and need:
        core = sorted(set(distinct[0]).intersection(*distinct[1:]))
        if len(core) >= need:
            return core, False
    return (distinct[0] if distinct else []), len(distinct) > 1
