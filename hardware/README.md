# Hardware

- [`probe/`](probe/): the probe head. One PETG print on a Quick Swap plate profile taken from
  Snapmaker's own 10W laser CAD, a lid, the OpenSCAD source, wiring and build notes.
- [`firmware/`](firmware/): the ESPHome configuration for the ESP32-C3 on the head.

You also need a BLTouch or clone, an ESP32-C3 SuperMini with the 0.42" OLED, two M3 screws and
nuts, and a USB-C lead for power. The red dot, camera, rotary and LightBurn bridge need only the
Snapmaker's own modules.
