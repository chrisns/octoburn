# coding=utf-8
"""The red autofocus dot as a height gauge: finding it, fitting it, reading heights."""
import numpy as np
import pytest

from lhp_pkg import redot
from lhp_pkg.closeup_libs import CloseupError

TRUE = dict(a=10.11, b=65707.8, c=3.52)


def photo(dot=None, background=170, clutter=()):
    """A 1024 x 1280 BGR photo: grey paper, optional bright things, optional red dot at (x, y)."""
    img = np.full((1280, 1024, 3), background, np.float32)
    yy, xx = np.mgrid[0:1280, 0:1024]
    for (cx, cy, amp) in clutter:                      # bright marks present with the light off too
        img += amp * np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * 3.0 ** 2))[..., None]
    if dot is not None:
        cx, cy = dot
        halo = np.exp(-((xx - cx - 3) ** 2 + (yy - cy) ** 2) / (2 * 12.0 ** 2))    # lopsided red halo
        img[..., 2] += 60 * halo
        img[..., 1] += 15 * halo
        core = np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * 4.0 ** 2))
        img += 200 * core[..., None]
    return np.clip(img, 0, 255).astype(np.uint8)


def test_finds_the_dot_to_a_fraction_of_a_pixel_despite_clutter():
    clutter = [(175, 648, 120), (900, 640, 90)]       # bright marks in the band (the wood case)
    on, off = photo((662.3, 641.2), clutter=clutter), photo(None, clutter=clutter)
    d = redot.find_dot(on, off)
    assert d["x"] == pytest.approx(662.3, abs=0.25) and d["y"] == pytest.approx(641.2, abs=0.25)
    assert d["unique"] > 3


def test_no_dot_is_refused():
    with pytest.raises(CloseupError, match="no red dot"):
        redot.find_dot(photo(None), photo(None))


def test_two_dots_are_refused():
    on = photo((400, 640))
    on2 = np.maximum(on, photo((800, 650)))
    with pytest.raises(CloseupError, match="not clear"):
        redot.find_dot(on2, photo(None))


def _samples(heads=((175.0, 120.0),), zs=(100, 120, 140, 160, 180), lift=0.0, noise=0.0, seed=1):
    rng = np.random.default_rng(seed)
    out = []
    d = np.array([-0.521, 0.0141, -0.8534]); d /= np.linalg.norm(d)
    p0 = np.array([19.4, -0.4, 0.0])
    for hx, hy in heads:
        for z in zs:
            x = TRUE["a"] + TRUE["b"] / (z + lift + TRUE["c"]) + rng.normal(0, noise)
            s = (-z - p0[2]) / d[2]
            out.append(dict(z=z, x=x, y=640 + 0.01 * x, lift=lift, rel=list(p0 + s * d)))
    return out


def test_fit_recovers_the_law_and_the_ray():
    m = redot.fit(_samples())
    assert m.a == pytest.approx(TRUE["a"], abs=0.5) and m.c == pytest.approx(TRUE["c"], abs=0.05)
    assert m.residual_px < 0.05
    dx, dy = m.offset(100.0, 0.0)
    assert dx == pytest.approx(19.4 - 100 * 0.521 / 0.8534, abs=0.05)
    # a higher surface puts the dot nearer the head
    assert m.offset(100.0, 10.0)[0] > dx
    assert m.off_line(500.0, 645.0) == pytest.approx(0.0, abs=0.1)


def test_heights_come_back_from_the_column():
    m = redot.fit(_samples())
    for h in (0.0, 2.7, 10.0, 30.0):
        x = TRUE["a"] + TRUE["b"] / (100 + TRUE["c"] - h)
        assert m.height(100.0, x) == pytest.approx(h, abs=0.01)
    # the bed map: the firmware lifts the head 0.3 more than the bed under the dot
    x = TRUE["a"] + TRUE["b"] / (100 + 0.3 + TRUE["c"] - 2.7)
    assert m.height(100.0, x, lift=0.3) == pytest.approx(2.7, abs=0.01)


def test_sensitivity_is_finer_lower_down():
    m = redot.fit(_samples())
    assert m.mm_per_px(100.0) < m.mm_per_px(170.0) < 0.6
    assert m.mm_per_px(100.0) == pytest.approx(0.163, abs=0.01)


def test_fit_needs_enough_heights():
    with pytest.raises(CloseupError):
        redot.fit(_samples(zs=(100, 120, 140)))
    with pytest.raises(CloseupError):
        redot.fit(_samples(zs=(100, 105, 110, 115)))


def test_model_round_trips():
    m = redot.fit(_samples(noise=0.1))
    m2 = redot.DotModel.from_dict(m.to_dict())
    assert m2.height(120.0, 600.0) == pytest.approx(m.height(120.0, 600.0))
    assert m2.offset(120.0, 5.0) == pytest.approx(m.offset(120.0, 5.0))


def test_outside_the_range_is_refused():
    m = redot.fit(_samples())
    with pytest.raises(CloseupError):
        m.height(100.0, m.a + 0.5)
