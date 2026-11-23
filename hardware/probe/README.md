# Laser height probe

A probe head for the Snapmaker A350's Quick Swap bay. It measures the height of a delicate
object without touching it with the laser, so LightBurn can be given exact focus heights.

- **Head**: one printed part. It is a Quick Swap toolhead plate, with a probe shelf and an
  ESP32 box on its front face. A screw-on lid with a window for the OLED goes over the box.
- **Probe**: a BLTouch-style clone. The pin sits on the 10W laser's beam axis, so a probe
  XY is the same as a laser XY.
- **Brain**: an ESP32-C3 SuperMini with the 0.42" OLED, running ESPHome
  ([`../firmware/laserprobe.yaml`](../firmware/)).
- **Control**: the OctoBurn OctoPrint plugin (this repository). It steps Z down
  and reads the probe over Wi-Fi.

The laser stays **plugged in** while the probe head is in the bay. That keeps the firmware in
laser mode, so the machine homes and moves, and no power cycle is needed to swap heads. Park the
laser **off the work platform**, beam-down on something that can't burn, within reach of its
cable. The A350's platform moves in Y, so the laser must not sit on it. The plugin never sends a
laser command.

## Files

| File | What |
|---|---|
| `laser-height-probe.scad` | Source. `part = "plate" \| "lid" \| "fit_test" \| "assembly"` |
| `laser-height-probe.stl` | The head (~53 g PETG) |
| `laser-height-probe-lid.stl` | Box lid (~3 g) |
| `laser-height-probe-fit-test.stl` | The plate alone (~35 g), to check the slide and the notch |
| `wiring.svg` / `wiring.png` | Wiring diagram |

## Where the geometry comes from

The plate profile is **Snapmaker's own**: the 10W laser's STEP file
(`Snap-LASER-10W-ASSY.STEP`) includes the genuine Quick Swap toolhead plate. Its features:

- 80 x 7 mm. Calipers on a real plate (on the 10W laser) read 80.02 x 7.03, and 77.10 across
  the machine face from the left edge to the clamp lobe. The STEP says 76.8 there; the model
  uses the real 77.10, with the extra 0.3 on the lobe.
- 35° dovetails: the left edge runs full height, and the right edge has a clamp lobe
  between Y 21.8 and 59.5 that the bracket handle presses on.
- A 46 mm channel, 3 mm deep, in the machine face.
- A locating notch at the bottom left, as seen from the front. It drops onto the bracket's
  pin, and it is the Z datum.

The printable plate on MakerWorld (model 1874872) is the same shape to within 0.7 mm. Its
file is exported upside down, but once rotated properly its notch lands at the same bottom-left
corner. This design follows Snapmaker's CAD. The genuine plate also has two 4 mm holes on the
centreline, at Y 20 and 60, and they're here too as blind holes.

| Parameter | Value | Note |
|---|---|---|
| `plate_w` | 77.10 mm | width across the machine face, left edge to lobe, measured on a real plate |
| `win_lip` | 1.0 mm | lid lip over each long PCB edge. At 2.2 the window printed 16.45 wide and caught the RST and BOOT buttons; at 1.0 it is 18.5 wide and runs 1.5 past the board's far end |
| `fit_cl` | 0 | clearance normal to the LEFT dovetail face (the fixed rail). The open lever gives the slide its play; raise this only if the plate won't slide in with the lever open |
| `lobe_extra` | 0.92 mm | X added to the lobe beyond the real plate. At 0.52 the head read 76.80 on calipers (genuine plate 77.10 the same way), was a little loose and the lever did not lock fully. 0.92 should read about 77.20. Too wide can be filed off the lobe; too narrow cannot be fixed |
| `ch_sag` | 0.5 mm | extra channel depth, so any sag of the channel roof stays clear of the bracket; 3.5 deep puts the roof on a layer line, so the PLA support interface meets it flat |
| `notch_cl` | 0.5 mm | notch width only; the notch top (the datum) is not moved, so the width can be generous |
| `tip_below` | 24 mm | deployed pin tip below the plate's bottom edge (the laser nozzle ends at 19) |
| `probe_hole_pitch` | 18 mm | **assumed** (the BLTouch/3DTouch standard). Measure your clone |
| probe body | 13.2 x 14 x 36, flange 2.35, pin 5.3 / 10.4 | your caliper numbers |

## Print

- PETG, **machine face down** (the model is already in that pose).
- The channel in the machine face is a 44 mm bridge: support it in the slicer. On the dual
  head, PLA as the support interface peels off PETG cleanly. The channel roof is not a mating
  face, so a rough finish there does not matter. (A printed fin comb was tried and dropped: it
  was hard to get out.)
- The lid prints flat.
- Calipers across a print catch it about 0.4 mm above the bed, where the elephant-foot chamfer
  ends: a print true to the model reads 77.46 there. The first head read 76.80 at
  lobe_extra 0.52; aim for about 77.20 (the genuine plate reads 77.10).
- Sliced in OrcaSlicer for the A350 Dual: PETG on T0,
  auto supports with 2 dense PLA interface layers on T1, 2 mm XY gap, no prime tower, bed held
  at 75 C).
- The full plate (or `laser-height-probe-fit-test.stl` to check the slide and the notch).
  Also look at the bracket for anything that enters the channel or the two centreline holes.

## Build

No extra components: the five probe wires go straight to the board (see `wiring.png`).

| Probe wire | Board |
|---|---|
| Red (+5V) | 5V |
| Brown (GND) and black (GND) | GND |
| Orange (control) | GPIO4 |
| White (Z-min out) | GPIO3, internal pull-up. **Only after the check in step 3** |

1. Fix the probe under the shelf with 2 x M3 screws, with nuts in the traps on top of the
   shelf. The cable goes up through the slot.
2. Solder red, brown, black and orange to the board from underneath. **Leave white off for now.**
   Feed the leads into the box through the slot in its right-hand wall. The lid clamps the
   board's edges. Power comes from a USB-C lead, routed with the laser cable.
3. **Prove the white wire never drives 5 V.** The C3's pins are not 5 V tolerant, and with no
   diode in the way that matters. Flash (step 4), power up, and press **probe_check** on the
   ESPHome page. That puts the pin up and the output in its "triggered" state. Then measure
   from white to GND:
   - About 0 V, or floating: the output is open-drain. Solder white to GPIO3.
   - 5 V: press **probe_set_od_mode** (it sets open-drain and stores it in the probe's EEPROM),
     power-cycle and measure again.
   - Still 5 V: this clone can't do open-drain. Put a 10k resistor in series in the white wire.
     That limits the current into the pin's clamp to about 0.15 mA.
4. Flash: see [`../firmware/README.md`](../firmware/README.md) (the first flash is over USB).
   If the OLED text is clipped or shifted, adjust `oled_offset_x/y`: the listing warns that
   this panel's window starts at (13,14). If the text is upside down, set `oled_rotation: "180"`.
5. With white connected, press **probe_check** again: `probe_check_ok` should read ON. Some
   clones lack touch mode; if yours does, press probe_deploy, push the pin up with a finger and
   watch `probe_latched` turn ON.

What the left-out parts did, and what replaces them:

- **Spike filter capacitor.** The firmware only accepts a trigger that stays high for 100 µs.
  Noise from stepper wiring lasts microseconds; a real trigger lasts milliseconds.
- **Bulk capacitor on 5V.** Printer boards run BLTouches without one. If the board resets when
  the pin deploys, add a 470 µF capacitor across 5V/GND at the probe plug.
- **Diode and pull-up.** Replaced by the chip's internal pull-up. This depends on the
  open-drain check in step 3.

The space below the board in the box is now spare.

## Use

1. Loosen the bracket handle, slide the laser out, and park it off the platform, beam-down and
   still plugged in.
   Slide the probe head in and tighten the handle.
2. In OctoPrint's **Laser Height Probe** tab, enter the points (laser XY), choose the laser
   module, and press Run. Each point is approached in coarse 0.5 mm steps, then 0.1, then
   0.02 mm, and the fine pass is repeated until two readings agree.
3. Swap the laser back in. The cable never came out, so there's no power cycle.
4. Read the numbers. **Focus Z** is the absolute Z to put the laser at. **LightBurn Z offset**
   is the difference between each point and the datum point, for per-layer Z offsets (your
   LightBurn device has Enable Z and Relative-Z-only on).

   **Check the sign once before trusting it.** Give a test layer a +1 mm Z offset in LightBurn,
   run it with the laser off (or at 0%), and watch which way Z moves. If LightBurn's sign is the
   opposite of the plugin's, every offset doubles the focus error instead of cancelling it.

## Calibration (once per laser module)

The plugin needs `focus_offset = laser focus Z - probe trigger Z` measured on the same spot:

1. Probe a flat reference (a scrap of the material) with the head.
2. Swap the laser in and bring it to best focus on that same spot. Either use your usual focus
   method, or touch the nozzle off and add the module's focal distance. Press **Use current Z**.

The plugin stores the offset against the module. Do this again if you reprint the head or
change the probe's height.

**Repeatability test, worth doing once.** Probe one spot five times, swapping the head out
and back in between runs. The spread is how well the printed plate seats on the bracket pin.
Expect a few hundredths of a millimetre. If it's more, the notch or the clamp isn't seating.

## Open items

- Measure the probe's mounting-hole pitch. 18 mm is assumed.
- Find where the probe's lead exits. The cable slot now covers the whole top of the body
  (10 x 14 mm), which should cover any exit.
- Tune the OLED offset and rotation on the real board.
- The FreeCAD mesh gauntlet was not run because FreeCAD isn't installed on this Mac. The parts
  passed OpenSCAD's Manifold backend, trimesh (watertight, single body) and manifold3d instead.
