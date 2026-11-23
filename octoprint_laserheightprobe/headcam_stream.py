# coding=utf-8
"""The head photo as an MJPEG stream, for LightBurn's network camera.

LightBurn stops polling a still JPEG URL (on 2026-11-20 it made no request for minutes after an
OctoPrint restart) but holds an MJPEG stream open, as it does go2rtc's bed camera. A flask
route cannot stream (OctoPrint runs flask inside tornado and buffers the whole answer), so this
is a tornado handler, added through the octoprint.server.http.routes hook. The photo only
changes on a capture, so one frame a second is plenty.
"""
from __future__ import absolute_import

import asyncio

import tornado.iostream
import tornado.web

from . import headcam


class HeadcamStream(tornado.web.RequestHandler):
    def initialize(self, frames, interval=headcam.STREAM_S):
        self.frames, self.interval = frames, interval

    async def get(self):
        self.set_header("Content-Type", headcam.MJPEG_TYPE)
        self.set_header("Cache-Control", "no-store")
        try:
            while True:
                jpeg = self.frames()
                if jpeg:
                    self.write(headcam.mjpeg_part(jpeg))
                    await self.flush()
                await asyncio.sleep(self.interval)
        except tornado.iostream.StreamClosedError:
            pass                                  # LightBurn closed the camera
