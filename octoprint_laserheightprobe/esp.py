# coding=utf-8
"""The probe, read over HTTP from the ESPHome device on the probe head.

The ESP32-C3 drives the BLTouch-style probe and latches its trigger in an interrupt, so the
short pulse the probe gives on a touch cannot be missed between two reads. Its web_server
REST API is the whole interface (see hardware/firmware/laserprobe.yaml):

    POST /button/<name>/press           probe_arm, latch_clear, probe_stow, probe_reset,
                                        probe_check, probe_set_od_mode
    GET  /binary_sensor/<name>          probe_latched, probe_raw, probe_check_ok
    GET  /sensor/trigger_count
    GET  /sensor/uptime or /sensor/Uptime  (only if the device has an uptime sensor)
    POST /select/job_state/set?option=  display only
    POST /number/result_z/set?value=    display only
    POST /number/result_focus/set?value=

Two rules shape this module.

Reads fail closed. A probe_latched reply that cannot be read as true or false raises EspError,
because returning False would mean "not touching yet, keep going down".

Display writes never fail anything. The OLED is a convenience; a job must not stop because
the display missed an update. Those methods swallow their own errors, so a call site cannot
forget to.

The HTTP layer is one function, `transport(method, url, timeout) -> body text`, so the tests
can put a simulated probe behind the real code.
"""
from __future__ import absolute_import

import json
import logging
import time

from urllib.parse import quote
from urllib.request import Request, urlopen

JOB_STATES = ("idle", "homing", "probing", "done", "error", "calibrate")

# result_focus value meaning "no focus Z" (module without a focus offset): the OLED shows "F --"
# instead of keeping the last laser's number. The number entity's range is -1000..1000.
NO_FOCUS = -1000.0

# probe_reset holds the reset pulse for 1 s; then the output must have dropped.
RESET_SETTLE_S = 1.3
# probe_touch_mode holds its pulse for 100 ms; the output follows the pin soon after.
TOUCH_SETTLE_S = 0.25
# The longest probe command is a reset (1 s), and the firmware's auto-reset after a trigger can
# queue one behind another, so idle must come within a few seconds.
IDLE_TIMEOUT_S = 4.0


class EspError(Exception):
    """The probe device did not answer, or answered something unreadable."""


class EspRebooted(EspError):
    """The probe device restarted during the job, so its latch may have lost a trigger."""


def urllib_transport(method, url, timeout):
    """The real transport: one HTTP request with the standard library."""
    data = b"" if method == "POST" else None           # an empty body sends Content-Length: 0
    request = Request(url, data=data, method=method)
    response = urlopen(request, timeout=timeout)
    try:
        return response.read().decode("utf-8", "replace")
    finally:
        response.close()


def parse_state(body, entity):
    """True or False from an ESPHome entity reply. Raises EspError for anything else.

    web_server replies {"id": "binary_sensor-probe_latched", "value": true, "state": "ON"}.
    "value" is preferred; "state" is the fallback for versions that leave it out.
    """
    try:
        reply = json.loads(body)
    except (TypeError, ValueError):
        raise EspError("%s: the reply is not JSON: %r" % (entity, (body or "")[:80]))
    if not isinstance(reply, dict):
        raise EspError("%s: unexpected reply %r" % (entity, reply))
    value = reply.get("value")
    if isinstance(value, bool):
        return value
    state = reply.get("state")
    if isinstance(state, str) and state.upper() in ("ON", "OFF"):
        return state.upper() == "ON"
    raise EspError("%s: the reply has no true/false state: %r" % (entity, reply))


def parse_number(body, entity, allow_unknown=False):
    """A sensor's number. A sensor that has not published since boot reports "NA" (or a null
    value): that is None when allow_unknown, else an error."""
    try:
        reply = json.loads(body)
    except (TypeError, ValueError):
        raise EspError("%s: the reply is not JSON: %r" % (entity, (body or "")[:80]))
    value = reply.get("value") if isinstance(reply, dict) else None
    if isinstance(value, bool):
        value = None
    if isinstance(value, (int, float)) and value == value:
        return float(value)
    state = str(reply.get("state") if isinstance(reply, dict) else "").strip()
    try:
        number = float(state.split()[0])
        if number == number:
            return number
    except (IndexError, ValueError):
        pass
    if allow_unknown:
        return None
    raise EspError("%s: the reply has no number: %r" % (entity, (body or "")[:80]))


class EspProbe(object):
    """The probe head's ESP, as the routine sees it."""

    # Waits between GET tries. Reads happen with the head stopped (the staircase reads the
    # latch between moves), so riding out a few seconds of slow Wi-Fi is safe; giving up is not.
    GET_WAITS = (0.5, 1.0, 2.0)

    def __init__(self, base_url, timeout=2.0, transport=None, logger=None, sleep=None):
        self.base_url = (base_url or "").rstrip("/")
        self.timeout = float(timeout)
        self._transport = transport or urllib_transport
        self._logger = logger or logging.getLogger(__name__)
        self._sleep = sleep or time.sleep

    # -- plumbing ---------------------------------------------------------

    def _url(self, path):
        return self.base_url + path

    def _post(self, path):
        try:
            return self._transport("POST", self._url(path), self.timeout)
        except EspError:
            raise
        except Exception as exception:                 # noqa: BLE001
            raise EspError("POST %s failed: %s" % (path, exception))

    def _get(self, path):
        """GET, retried with back-off: a slow Wi-Fi spell should not stop a job."""
        last = None
        tries = len(self.GET_WAITS) + 1
        for attempt in range(tries):
            try:
                return self._transport("GET", self._url(path), self.timeout)
            except Exception as exception:             # noqa: BLE001
                last = exception
                if attempt < len(self.GET_WAITS):
                    self._logger.warning("laserheightprobe: GET %s failed (%s); retrying", path, exception)
                    self._sleep(self.GET_WAITS[attempt])
        raise EspError("GET %s failed %d tries: %s" % (path, tries, last))

    # Presses that are safe to repeat (sent with the head stopped): a lost reply is retried like a
    # read, so one Wi-Fi blip does not stop a whole job.
    SAFE_PRESSES = ("latch_clear", "probe_stow", "probe_arm", "probe_reset")

    def _press(self, name):
        path = "/button/%s/press" % name
        if name not in self.SAFE_PRESSES:
            return self._post(path)
        for attempt in range(len(self.GET_WAITS) + 1):
            try:
                return self._post(path)
            except EspError as exception:
                if attempt == len(self.GET_WAITS):
                    raise
                self._logger.warning("laserheightprobe: %s; retrying", exception)
                self._sleep(self.GET_WAITS[attempt])

    def _binary(self, name):
        return parse_state(self._get("/binary_sensor/%s" % name), name)

    # -- the probe --------------------------------------------------------

    def arm(self):
        """Clear the latch and deploy the pin. The deploy takes about 0.7 s after this returns."""
        self._press("probe_arm")

    def clear(self):
        self._press("latch_clear")

    def stow(self):
        self._press("probe_stow")

    def reset(self):
        """Release a probe alarm (and stow)."""
        self._press("probe_reset")

    def busy(self):
        """True while the ESP is still sending a probe command (deploy, stow, reset)."""
        return self._binary("probe_busy")

    def wait_idle(self, sleep, timeout=IDLE_TIMEOUT_S):
        """Wait until no probe command is being sent.

        The firmware resets the probe 150 ms after each trigger (1 s of PWM). A deploy pressed
        meanwhile would queue behind it and come out late, after the plugin had decided the pin
        was down. Raises EspError if the ESP stays busy.
        """
        waited = 0.0
        while self.busy():
            if waited >= timeout:
                raise EspError("the probe device stayed busy for %.0f s" % timeout)
            sleep(0.1)
            waited += 0.1

    def release_held_trigger(self, sleep):
        """Make the probe ready to deploy. Returns True when its output is LOW.

        Some clones hold the output HIGH after a trigger and ignore deploy and stow until a
        reset (Klipper's pin_up_reports_not_triggered: False). Chris's 3DTouch-type clone does
        this (2026-11-16). A held output can't make the rising edge the latch needs, so reset
        it first. An output still HIGH after the reset is a real alarm.
        """
        if not self.raw():
            return True
        self.reset()
        sleep(RESET_SETTLE_S)
        return not self.raw()

    def deploy(self):
        """Deploy the pin without clearing the latch. Leaves touch mode for normal mode."""
        self._press("probe_deploy")

    def touch_read(self, sleep):
        """Touch (switch) mode, then the output: HIGH with the pin up, LOW with it down. Read the
        pin-up case straight after a reset: after a plain stow it often reads LOW (2026-11-19,
        3 of 12) although the pin is up. A probe that ignores the command also reads LOW. Pin up + touch mode
        makes a rising edge, so this sets the latch and counts a trigger: clear it after."""
        self._press("probe_touch_mode")
        sleep(TOUCH_SETTLE_S)
        return self.raw()

    def check(self):
        """Start the wiring test. The result is in check_ok() about 1.6 s later."""
        self._press("probe_check")

    def set_od_mode(self):
        """Set the probe's output to open drain and store it in the probe's EEPROM. One-off:
        needed only if the white wire measures 5 V, since it now goes straight to GPIO3."""
        self._press("probe_set_od_mode")

    def latched(self):
        """True once the probe has triggered since the last clear."""
        return self._binary("probe_latched")

    def raw(self):
        """The live probe output. HIGH while deployed means alarm or obstructed."""
        return self._binary("probe_raw")

    def check_ok(self):
        return self._binary("probe_check_ok")

    def trigger_count(self):
        return int(parse_number(self._get("/sensor/trigger_count"), "trigger_count"))

    UPTIME_PATHS = ("/sensor/uptime", "/sensor/Uptime")

    def find_uptime(self):
        """The path of the device's uptime sensor, or None if it has none.

        ESPHome's REST paths use the object id ("uptime") in older versions and the entity
        name ("Uptime") in newer ones, so both are tried, once each.
        """
        for path in self.UPTIME_PATHS:
            try:
                parse_number(self._transport("GET", self._url(path), self.timeout), "uptime",
                             allow_unknown=True)
                return path
            except Exception:                          # noqa: BLE001
                continue
        return None

    def uptime(self, path):
        """Seconds since the device booted, or None while the sensor has not published."""
        return parse_number(self._get(path), "uptime", allow_unknown=True)

    def status(self):
        """Everything readable at once, for the tab. Never raises: errors are reported."""
        out = {}
        for key, read in (("latched", self.latched), ("raw", self.raw),
                          ("check_ok", self.check_ok), ("trigger_count", self.trigger_count)):
            try:
                out[key] = read()
            except EspError as exception:
                out[key] = None
                out.setdefault("error", str(exception))
        return out

    # -- the display: best effort ----------------------------------------

    def _display(self, path):
        try:
            self._post(path)
            return True
        except Exception as exception:                 # noqa: BLE001
            self._logger.debug("laserheightprobe: display update %s failed: %s", path, exception)
            return False

    def show_state(self, state):
        if state not in JOB_STATES:
            return False
        return self._display("/select/job_state/set?option=%s" % quote(state))

    def show_z(self, value):
        if value is None:
            return False
        return self._display("/number/result_z/set?value=%.3f" % float(value))

    def show_focus(self, value):
        if value is None:
            value = NO_FOCUS                 # never leave an old focus Z beside a new Z
        return self._display("/number/result_focus/set?value=%.3f" % float(value))


class Watchdog(object):
    """Notices a probe device that restarted in the middle of a job.

    A restart clears the latch. If it happens between a trigger and the read of the latch,
    the trigger is lost and the head would keep stepping down with the pin already stowed.
    Two counters reveal a restart by going backwards: trigger_count (edges since boot) and the
    uptime sensor. After the tap test trigger_count is at least 1, so a restart always takes
    it back to 0, even on a device with no uptime sensor. It is read after every probing step;
    uptime, which ESPHome may publish only once a minute, is read at most every
    `uptime_every_s` and at every arm.

    A counter that cannot be read stops the job, like a latch that cannot be read.
    """

    def __init__(self, esp, clock=time, uptime_every_s=5.0, logger=None):
        self._esp = esp
        self._clock = clock
        self._every = uptime_every_s
        self._logger = logger or logging.getLogger(__name__)
        self.count = None
        self.uptime = None
        self.uptime_path = None
        self._uptime_read_at = None

    def start(self):
        self.count = self._esp.trigger_count()
        self.uptime_path = self._esp.find_uptime()
        if self.uptime_path is None:
            self._logger.info("laserheightprobe: the probe device has no uptime sensor; restarts "
                              "are detected from trigger_count only")
        else:
            self.uptime = self._esp.uptime(self.uptime_path)
            self._uptime_read_at = self._clock.time()
        return self

    def check(self, where, force_uptime=False):
        count = self._esp.trigger_count()
        if self.count is not None and count < self.count:
            raise EspRebooted("the probe device restarted (%s): trigger_count went from %d to %d"
                              % (where, self.count, count))
        self.count = count
        if self.uptime_path is None:
            return
        now = self._clock.time()
        if not force_uptime and self._uptime_read_at is not None and now - self._uptime_read_at < self._every:
            return
        uptime = self._esp.uptime(self.uptime_path)
        self._uptime_read_at = now
        if self.uptime is not None and (uptime is None or uptime < self.uptime):
            raise EspRebooted("the probe device restarted (%s): uptime went from %s to %s"
                              % (where, self.uptime, uptime))
        if uptime is not None:
            self.uptime = uptime
