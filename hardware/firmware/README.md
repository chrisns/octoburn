# Probe firmware (ESPHome)

`laserprobe.yaml` runs on an ESP32-C3 SuperMini with the 0.42" OLED. It drives a BLTouch-style
probe and serves a small REST API that OctoBurn polls over Wi-Fi. The header of the YAML lists
every endpoint and the wiring.

## Build and flash

1. Install ESPHome (`pip install esphome`).
2. Copy `secrets.example.yaml` to `secrets.yaml` and fill it in. Never commit `secrets.yaml`.
3. Put two fonts next to the YAML (they are not included here, for licence reasons):
   - `pixelmix.ttf`: the "pixelmix" bitmap font by Andrew Tyler.
   - `DejaVuSans.ttf`: from the DejaVu fonts (https://dejavu-fonts.github.io/).
   Any TTF works if you change the file names in the `font:` block.
4. Plug the board in over USB and run `esphome run laserprobe.yaml`. Later flashes go over Wi-Fi.
5. In OctoBurn's settings, set **ESP base URL** to the board's address. The default,
   `http://laserprobe.local`, works where mDNS does.

The web server has no authentication: keep the board on a trusted network.

`laserprobe_latch.h` is the interrupt latch that catches the probe's short trigger pulse.
Pulse widths come from Klipper's `bltouch.py`.

Then follow the build steps in [`../probe/README.md`](../probe/README.md): the open-drain check
must happen before the white wire goes to GPIO3.
