# coding=utf-8
"""The close-up job: the moves and photos of a tile plan (closeup_plan.py), then the head back up.

It runs as the plugin's job (the routine.Job interface: mode, phase, abort, is_alive), so Stop,
a disconnect or a print starting stop it like a probe job. The rules:
- M5 first.
- Go up to the travel height before any XY move.
- Go down only over the first tile, and stay inside the module sweep for every low move.
- Every photo position is approached from -X, -Y (approach_mm away): the X axis has about
  0.42 mm of backlash (seen in the 2026-11-20 calibration).
- The head must be where it was sent, before and after each photo. The touchscreen moves the
  head itself if it is sent another X, Y, Z, so a moved head fails the tile.
- Always end at the travel height, whether the job succeeded, was stopped or failed.
- Compose only once the head is up.
- A tile may bring its own photo (tile["capture"](sleep), the fixed bed camera's overview with
  the head parked out of its view) and its own approach (tile["approach"] = (ax, ay): the bed
  camera's park pose is at X0, where a step from -X would leave the bed).
Plain Python: the plugin passes in the capture, store and compose functions.
"""
from __future__ import absolute_import

import threading
import time

from . import gcode
from .probe import raise_to_safe

POSITION_TOLERANCE = 0.05
DESCEND_FEED = 600
TILE_FEED = 3000
RETRY_WAIT_S = 2.0


class Aborted(Exception):
    pass


class CloseupJob(threading.Thread):
    def __init__(self, bridge, capture, say, plan, store, compose, config, mode="closeup", clock=time,
                 logger=None, retries=3, sleep=None):
        threading.Thread.__init__(self, name="laserheightprobe-closeup")
        self.daemon = True
        self._bridge, self._capture, self._say = bridge, capture, say
        self._plan, self._store, self._compose = plan, store, compose
        self._c = config
        self.mode = mode
        self._clock, self._logger = clock, logger
        self._retries = retries
        self._sleep_fn = sleep
        self._abort = threading.Event()
        self.phase = "starting"
        self.result = None
        self.error = None

    def abort(self):
        self._abort.set()

    @property
    def aborted(self):
        return self._abort.is_set()

    def _check(self):
        if self._abort.is_set():
            raise Aborted()

    def _sleep(self, seconds):
        """The capture's poll wait: wakes at once on Stop."""
        if self._sleep_fn is not None:
            self._sleep_fn(seconds)
            self._check()
        elif self._abort.wait(seconds):
            raise Aborted()

    def _where(self, lines):
        where = gcode.position(lines)
        if where is None:
            raise RuntimeError("cannot read the head position (M114)")
        return where

    def _at(self, lines, x, y, z, when):
        where = self._where(lines)
        off = max(abs(where[0] - x), abs(where[1] - y), abs(where[2] - z))
        if off > POSITION_TOLERANCE:
            raise RuntimeError("the head %s is at X%.2f Y%.2f Z%.2f, not X%.1f Y%.1f Z%.1f: the touchscreen "
                               "moved it, or the move was cut short" % (when, where[0], where[1], where[2], x, y, z))

    def _shoot(self, tile):
        last = None
        for attempt in range(1, self._retries + 1):
            self._check()
            try:
                if tile.get("capture") is not None:
                    return tile["capture"](self._sleep)
                return self._capture(tile["x"], tile["y"], tile["z"], self._sleep)
            except Aborted:
                raise
            except ValueError as exception:
                last = exception
                if self._logger:
                    self._logger.warning("close-up tile %d photo, try %d: %s", tile["i"] + 1, attempt, exception)
                self._sleep(RETRY_WAIT_S)
        if tile.get("capture") is not None:
            raise RuntimeError("the bed camera gave no photo after %d tries: %s" % (self._retries, last))
        raise RuntimeError("the touchscreen gave no photo for tile %d after %d tries: %s"
                           % (tile["i"] + 1, self._retries, last))

    def run(self):
        p = self._plan
        tiles = p["tiles"]
        travel, low = p["travel_z"], p["head_z"]
        zf, xyf = self._c["z_feed"], self._c["xy_feed"]
        kind, message, result = "failed", None, None
        up = False
        try:
            self.phase = "moving"
            where = self._where(self._bridge.run(["M5", "G90"], 30.0))
            if where[2] < travel - 1e-6:
                self._bridge.run(["G0 Z%.1f F%d" % (travel, zf)], 120.0)
            self._check()
            first = tiles[0]
            a = float(self._c.get("approach_mm") or 0.0)
            ax, ay = first.get("approach", (a, a))
            self._bridge.run(["G0 X%.1f Y%.1f F%d" % (first["x"] - ax, first["y"] - ay, xyf)], 120.0)
            self._check()
            self._bridge.run(["G0 Z%.1f F%d" % (first.get("z", low), DESCEND_FEED)], 120.0)
            self.phase = "photos"
            z_now = first.get("z", low)
            for n, tile in enumerate(tiles, 1):
                self._check()
                z = tile.get("z", low)
                # a higher pose: up first, then across; a lower one: across, then down
                # backlash: arrive from -X, -Y every time (from approach_mm away)
                ax, ay = tile.get("approach", (a, a))
                moves = (["G0 X%.1f Y%.1f F%d" % (tile["x"] - ax, tile["y"] - ay, TILE_FEED)] if ax > 0 or ay > 0 else [])
                moves.append("G0 X%.1f Y%.1f F%d" % (tile["x"], tile["y"], TILE_FEED))
                if z > z_now + 1e-6:
                    moves.insert(0, "G0 Z%.1f F%d" % (z, zf))
                elif z < z_now - 1e-6:
                    moves.append("G0 Z%.1f F%d" % (z, DESCEND_FEED))
                lines = self._bridge.run(moves, 120.0)
                z_now = z
                self._at(lines, tile["x"], tile["y"], tile["z"], "before tile %d" % n)
                jpeg = self._shoot(tile)
                self._check()                            # Stop during the photo: keep nothing more
                self._at(self._bridge.run([], 15.0), tile["x"], tile["y"], tile["z"], "after tile %d's photo" % n)
                self._store(tile["i"], jpeg, (tile["x"], tile["y"], tile["z"]))
                self._say(dict(type="progress", mode=self.mode, tile=n, tiles=len(tiles),
                               message="Close-up: tile %d of %d." % (n, len(tiles))))
            self.phase = "raising"
            up = raise_to_safe(self._bridge, travel, zf, logger=self._logger)
            if not up:
                raise RuntimeError("could not confirm the head is back up at Z%.1f" % travel)
            self.phase = "composing"
            result = self._compose()
            kind, message = "done", "Close-up: %d tile%s." % (len(tiles), "" if len(tiles) == 1 else "s")
        except Aborted:
            kind, message = "aborted", "Close-up stopped."
        except Exception as exception:                       # noqa: BLE001
            if self._abort.is_set():
                kind, message = "aborted", "Close-up stopped."
            else:
                kind, message = "failed", str(exception)
                self.error = message
        finally:
            if not up:
                self.phase = "raising"
                raise_to_safe(self._bridge, travel, zf, logger=self._logger)
            self.phase = "finished"
            self.result = result
            self._say(dict(type=kind, mode=self.mode, message=message, result=result))
