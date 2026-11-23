# coding=utf-8
"""numpy and OpenCV for the close-up camera, imported only when a close-up needs them.

The test machine's OctoPrint Python has both (numpy 2.5, OpenCV 5). The plugin installs with --no-deps,
and without them it must still probe and run the LightBurn bridge, so no module imports them at
the top (tests/test_imports.py checks that): they come from here, inside the functions.
"""
from __future__ import absolute_import


class CloseupError(ValueError):
    """A close-up cannot be done; the API answers 400 with the message."""


def np():
    try:
        import numpy
    except ImportError:                                  # pragma: no cover (the test machine has it)
        raise CloseupError("numpy is not installed in OctoPrint's Python: the close-up camera needs it "
                           "(pip install numpy opencv-python-headless)")
    return numpy


def cv():
    try:
        import cv2
    except ImportError:                                  # pragma: no cover (the test machine has it)
        raise CloseupError("OpenCV is not installed in OctoPrint's Python: the close-up camera needs it "
                           "(pip install opencv-python-headless)")
    return cv2


def versions():
    """{'numpy': '2.5.4', 'cv2': '5.0.0'} with None for one that is missing."""
    out = {}
    for name in ("numpy", "cv2"):
        try:
            out[name] = __import__(name).__version__
        except ImportError:
            out[name] = None
    return out
