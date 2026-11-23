# coding=utf-8
"""A laser module's beam offset from the probe pin, from two jogs onto the same dot.

1. Probe on the head: jog until the pin is on a dot on a card. The head is at H1, so
   H1 + P = dot (P: the pin from the head's reference).
2. Laser on the head, its pointer on: jog until the light is on the dot, head at H2.
   - A beam pointer (the 10W at low power): H2 + B = dot.
   - A crosshair (the 2W IR): H2 + B + c = dot, with c = crosshair - beam (M2002 T8).

The plugin's dx, dy is pin minus beam (P - B): the head goes that much further so the beam
lands where the pin measured. So dx, dy = H2 - H1 (+ c for a crosshair).
"""
from __future__ import absolute_import, division


def beam_offset(pin_head, beam_head, crosshair=None):
    cx, cy = crosshair or (0.0, 0.0)
    return (round(beam_head[0] - pin_head[0] + cx, 3), round(beam_head[1] - pin_head[1] + cy, 3))
