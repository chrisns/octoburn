"""The 10W head camera, through the Snapmaker touchscreen's HTTP API (port 8080).

The camera in the 10W module sends its picture to the touchscreen over Bluetooth, never to the
controller, so OctoPrint's serial link cannot fetch it. Luban asks the touchscreen for a photo at
X, Y, Z (request_capture_photo, which answers {"status":true} once the photo is taken) and then
downloads it (get_camera_image, the last photo, 1024x1280 JPEG).
"""
import pytest

from lhp_pkg import headcam

JPEG_OLD = b"\xff\xd8old photo\xff\xd9"
JPEG_NEW = b"\xff\xd8new photo\xff\xd9"


class Screen(object):
    """The touchscreen: keeps the last photo; a capture replaces it after `delay` image reads."""

    def __init__(self, status=b'{"status":true}', delay=0, stored=JPEG_OLD, refuse=None):
        self.status, self.delay, self.stored, self.refuse = status, delay, stored, refuse
        self.calls = []
        self.pending = None

    def get(self, url, timeout):
        self.calls.append(url)
        if self.refuse is not None:
            raise headcam.HeadcamError(self.refuse)
        if "/api/request_capture_photo" in url:
            self.pending = self.delay
            return 200, self.status
        if "/api/get_camera_image" in url:
            if self.pending is not None:
                if self.pending <= 0:
                    self.stored, self.pending = JPEG_NEW, None
                else:
                    self.pending -= 1
            if self.stored is None:
                return 404, b""
            return 200, self.stored
        return 404, b""


class Clock(object):
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += s


def test_a_capture_asks_for_the_photo_at_the_position_and_returns_the_new_one():
    screen, clock = Screen(), Clock()
    jpeg = headcam.capture(screen.get, "http://screen.test:8080", 232, 178, 290, quality=10,
                           clock=clock, sleep=clock.sleep)
    assert jpeg == JPEG_NEW
    request = [c for c in screen.calls if "request_capture_photo" in c][0]
    assert request == ("http://screen.test:8080/api/request_capture_photo?index=0&x=232.0&y=178.0"
                       "&z=290.0&feedRate=3000&photoQuality=10")


def test_a_photo_that_comes_late_is_waited_for_and_the_old_one_is_never_returned():
    screen, clock = Screen(delay=3), Clock()
    assert headcam.capture(screen.get, "http://s:8080", 1, 2, 3, clock=clock, sleep=clock.sleep) == JPEG_NEW
    screen, clock = Screen(stored=None, delay=1), Clock()        # no photo yet: 404 first
    assert headcam.capture(screen.get, "http://s:8080", 1, 2, 3, clock=clock, sleep=clock.sleep) == JPEG_NEW


def test_a_photo_that_never_comes_times_out():
    screen, clock = Screen(delay=10 ** 6), Clock()
    with pytest.raises(headcam.HeadcamError, match="no new photo"):
        headcam.capture(screen.get, "http://s:8080", 1, 2, 3, timeout=5.0, clock=clock, sleep=clock.sleep)


def test_a_refused_capture_and_an_unreachable_screen_say_so():
    screen, clock = Screen(status=b'{"status":false}'), Clock()
    with pytest.raises(headcam.HeadcamError, match="refused"):
        headcam.capture(screen.get, "http://s:8080", 1, 2, 3, clock=clock, sleep=clock.sleep)
    screen = Screen(refuse="the touchscreen at http://s:8080 did not answer: timed out")
    with pytest.raises(headcam.HeadcamError, match="did not answer"):
        headcam.capture(screen.get, "http://s:8080", 1, 2, 3, clock=clock, sleep=clock.sleep)
    assert issubclass(headcam.HeadcamError, ValueError)          # the API answers 400, not 500



def test_each_mjpeg_part_carries_one_whole_photo():
    # LightBurn's network camera stops polling a still JPEG URL (seen 2026-11-20: no request
    # for minutes after an OctoPrint restart), but it holds an MJPEG stream open, as it does
    # the bed camera's. So the plugin streams the last photo as multipart parts.
    for jpeg in (JPEG_OLD, JPEG_NEW):
        head, body = headcam.mjpeg_part(jpeg).split(b"\r\n\r\n", 1)
        assert head.startswith(b"--" + headcam.BOUNDARY.encode() + b"\r\n")
        assert b"Content-Type: image/jpeg" in head
        assert b"Content-Length: %d" % len(jpeg) in head
        assert body == jpeg + b"\r\n"
    assert headcam.MJPEG_TYPE == "multipart/x-mixed-replace; boundary=" + headcam.BOUNDARY


def test_a_caller_that_holds_the_last_photo_skips_the_first_read():
    from lhp_pkg import headcam as hc
    calls = []
    photos = iter([b"\xff\xd8new"])

    def get(url, timeout):
        calls.append(url)
        if "request_capture_photo" in url:
            return 200, b'{"status":true}'
        return 200, next(photos)
    jpeg = hc.capture(get, "http://screen", 1, 2, 3, quality=8, before=b"\xff\xd8old", sleep=lambda s: None)
    assert jpeg == b"\xff\xd8new" and len(calls) == 2           # request + one read, no read first
