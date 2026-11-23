# coding=utf-8
"""The 10W laser's head camera, through the Snapmaker touchscreen.

The camera in the 10W module sends its picture to the touchscreen over Bluetooth; nothing of it
reaches the controller, so OctoPrint's serial link cannot fetch it (Snapmaker2-Controller
docs/Hardware-Link.md: "HMI <-> Laser camera: Bluetooth"). Luban asks the touchscreen's HTTP
server (port 8080) for a photo at X, Y, Z and then downloads it (Luban
src/server/lib/image-getPhoto.js):

    GET /api/request_capture_photo?index=0&x=&y=&z=&feedRate=&photoQuality=   {"status":true}
    GET /api/get_camera_image?index=0                                          the last photo (JPEG)

The touchscreen moves the head to X, Y, Z itself, in the coordinates OctoPrint uses (seen on the
A350 on 2026-11-20), so the plugin moves there first and the touchscreen has nothing to move.
These calls need no token. Plain Python: the plugin passes the HTTP getter in.
"""
from __future__ import absolute_import

import time

FEED = 3000                       # Luban's feedRate for the capture move
POLL_S = 0.5                      # Luban polls the image every 500 ms
JPEG = b"\xff\xd8"
BOUNDARY = "headcamframe"
MJPEG_TYPE = "multipart/x-mixed-replace; boundary=" + BOUNDARY
STREAM_S = 1.0                    # one frame a second: the photo only changes on a capture


class HeadcamError(ValueError):
    pass


def http_get(url, timeout, what="the touchscreen"):
    """(status, body). A connection failure raises HeadcamError (naming what did not answer)."""
    try:
        from urllib.request import urlopen
        from urllib.error import HTTPError
    except ImportError:                                  # pragma: no cover (Python 2)
        from urllib2 import urlopen, HTTPError
    try:
        response = urlopen(url, timeout=timeout)
        try:
            return response.getcode(), response.read()
        finally:
            response.close()
    except HTTPError as exception:
        return exception.code, b""
    except Exception as exception:                       # noqa: BLE001
        base = url.split("/api/")[0]
        raise HeadcamError("%s at %s did not answer: %s" % (what, base, exception))


def request_url(base, x, y, z, quality):
    return "%s/api/request_capture_photo?index=0&x=%.1f&y=%.1f&z=%.1f&feedRate=%d&photoQuality=%d" % (
        base.rstrip("/"), x, y, z, FEED, quality)


def image_url(base):
    return "%s/api/get_camera_image?index=0" % base.rstrip("/")


def capture(get, base, x, y, z, quality=10, timeout=30.0, clock=time.monotonic, sleep=time.sleep,
            before=None):
    """Ask the touchscreen for a photo at x, y, z and return the new JPEG. The photo it held
    before is read first, so it is never mistaken for the new one, unless the caller already
    holds it (before=: the last photo it fetched, which saves a 0.5-2 s read)."""
    if before is None:
        status, before = get(image_url(base), 10.0)
        if status != 200:
            before = None
    status, body = get(request_url(base, x, y, z, quality), 60.0)
    if status != 200 or b'"status":true' not in body.replace(b" ", b""):
        raise HeadcamError("the touchscreen refused the photo (HTTP %s: %s)"
                           % (status, body[:200].decode("utf-8", "replace")))
    deadline = clock() + timeout
    while True:
        status, jpeg = get(image_url(base), 10.0)
        if status == 200 and jpeg.startswith(JPEG) and jpeg != before:
            return jpeg
        if clock() > deadline:
            raise HeadcamError("no new photo from the touchscreen after %.0f s" % timeout)
        sleep(POLL_S)


def mjpeg_part(jpeg):
    """One part of the MJPEG stream LightBurn holds open (it stops polling a still JPEG URL)."""
    return (b"--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
            % (BOUNDARY.encode("ascii"), len(jpeg))) + jpeg + b"\r\n"
