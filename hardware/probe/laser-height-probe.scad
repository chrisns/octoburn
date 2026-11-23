// Laser height probe head for a Snapmaker 2.0 A350 with the Quick Swap Kit.
//
// One printed part: a Quick Swap toolhead plate with a probe shelf and an ESP32 box on its
// toolhead face, plus a separate lid. The head goes in the Quick Swap bay in place of the
// laser (the laser stays plugged in, parked). A BLTouch-style clone probe measures the
// object's height; an ESP32-C3 SuperMini with a 0.42" OLED drives it (see laserprobe.yaml).
//
// Frame (as mounted, looking at the machine from the front):
//   X right, Y up (0 = plate bottom edge), Z forward (0 = machine face, the face against
//   the Quick Swap bracket; plate_t = toolhead face).
// Print orientation: machine face DOWN on the bed (the model is already in that pose).
//
// Plate geometry is the genuine Snapmaker toolhead plate as modelled in Snapmaker's own
// Snap-LASER-10W-ASSY.STEP (solid 76.8 x 80 x 7), measured off the STEP on 2026-11-16.
// Its frame has the hole pattern (+/-20, 12/68) centred on X = 0.
//
// part = "plate" | "lid" | "fit_test" | "assembly"
part = "plate";
$fn = 96;

// ---------------------------------------------------------------- plate (genuine CAD)
plate_h      = 80.0;    // STEP: Y 0..80 (live calipers on the genuine plate: 80.02)
plate_t      = 7.0;     // STEP: Z -37..-30 (live: 7.03)
dt_left_face = -37.9;   // left dovetail edge X at the machine face
dovetail_k   = 0.7;     // dovetail run per mm of thickness: 4.9 over 7 = 35.0 deg
right_edge   = 34.0;    // right edge X outside the lobe (square, not dovetailed)
// LIVE calipers, 2026-11-16, genuine plate on the 10W laser: 77.10 across the machine face,
// left edge to the clamp lobe. The STEP says 76.8; the real plate wins. The extra 0.3 goes
// on the lobe (the left edge locates the plate, the lobe only takes the jaw).
plate_w      = 77.10;
lobe_face    = dt_left_face + plate_w;   // right clamp lobe dovetail X at the machine face
corner_r     = 3.0;     // top-right and bottom-right corners (left corners are square)
fit_cl       = 0.0;     // clearance normal to the LEFT dovetail face (the fixed rail); raise if it won't slide in
// Neither face gets clearance: the open lever's jaw gives the slide its play, and the closed jaw
// must have something to bite on. The first print was 0.48 narrower than the real plate here
// (STEP width less clearance) and wobbled more than 1 mm with the lever unable to grip.
// Fit strip 2 (lobe_extra 0.30, model 76.42 across at Z 0.4) printed 76.0 in PETG.
// Head print 1 (lobe_extra 0.52, model 77.06 at Z 0.4) read 76.80 on calipers, 0.30 under the
// genuine plate's 77.10 measured the same way. It was a little loose and the lever did not
// lock fully. So +0.40: the next head should read about 77.20. Too wide can be filed off the
// lobe; too narrow cannot be fixed.
lobe_extra   = 0.92;   // X added to the lobe beyond the real plate
ef           = 0.4;     // elephant-foot chamfer on the bed edges of both dovetails
// machine-face channel: X +/-23 at the face, quarter rounds (r 3) down to a +/-20 floor
ch_half_floor = 20.0;
ch_r          = 3.0;
ch_depth      = 3.0;
ch_sag        = 0.5;    // extra depth, keeps sag clear of the bracket; 3.5 puts the roof on a layer line (0.3 + 0.2 n)
// locating notch, open at the BOTTOM edge, from the machine face: the Z datum (it sits on
// the bracket's pin). STEP: X -32.25..-27.5, round top at Y 5.6, 4.5 deep, 0.75 chamfers.
notch_x       = -29.875;
notch_w       = 4.75;
notch_top     = 5.6;
notch_depth   = 4.5;
notch_cl      = 0.5;    // total width clearance (free: only the top is the datum); the top is not moved
notch_chamfer = 0.75;

// The channel roof is a 44 mm bridge in the print pose: support it in the slicer (PLA support
// interface on the dual head peels off PETG cleanly). The printed fin comb is gone: it was hard
// to remove.

// ---------------------------------------------------------------- probe (user calipers)
// Clone BLTouch measured 2026-11-16: body 13.2 W x 14 D, 36 high including the 2.35 top
// flange, pin 5.3 below the body retracted and 10.4 deployed.
probe_w        = 13.2;
probe_d        = 14.0;
probe_total    = 36.0;   // flange top to body bottom
probe_flange_t = 2.35;
pin_retracted  = 5.3;
pin_deployed   = 10.4;
probe_hole_pitch = 18.0; // ASSUMED: BLTouch/3DTouch standard M3 x 2 at 18 mm. MEASURE.
m3_clear       = 3.3;
m3_nut_af      = 5.7;    // M3 nut 5.5 AF + 0.2
m3_nut_depth   = 2.6;

// Where the pin goes. The 10W laser's beam axis (STEP) sits at X +17 and 30 mm in front of
// the toolhead face, and its nozzle ends 19 mm below the plate's bottom edge. Putting the pin
// on that axis means probe XY = laser XY; putting the tip a little below the nozzle means the
// probe reaches anything the laser can focus on.
beam_x     = 17.0;
beam_z     = plate_t + 30.0;
tip_below  = 24.0;      // deployed tip, mm below the plate bottom edge (nozzle: 19)
shelf_t    = 5.0;
shelf_under = -tip_below + probe_total + pin_deployed;   // flange top bears on this face
shelf_x0   = 0.0;
shelf_x1   = right_edge - fit_cl - 0.25;                 // detuned off the plate edge
shelf_z1   = beam_z + probe_d / 2 + 3.0;
gusset_w   = 3.5;
gusset_drop = 20.0;     // gusset height below the shelf, on the plate face
gusset_reach = 22.0;    // gusset reach along the shelf (stays behind the probe flange)
cable_slot = [10.0, probe_d];  // X, Z through the shelf, over the whole body top (lead exit unknown)
// The genuine plate has two 4 mm holes on the centreline, from the channel floor to the
// toolhead face. Purpose unknown (dowels?); kept as blind holes so nothing on the bracket
// can bottom on them.
cl_holes   = [[0, 20], [0, 60]];
cl_hole_d  = 4.3;

// ---------------------------------------------------------------- ESP32 box
// ESP32-C3 SuperMini 0.42" OLED: PCB 20.5 x 25.0 (listing drawing), header rows 17.78 apart.
board_w   = 20.5;
board_l   = 25.0;
board_t   = 1.2;        // PCB thickness, generous; the lid clamps the board edges
board_cl  = 0.3;        // per side
usb_w     = 13.0;       // opening for a USB-C plug overmould
box_x0    = dt_left_face + fit_cl / cos(35) + dovetail_k * plate_t + 0.6;  // inside the plate
box_x1    = -2.0;
box_y0    = 3.0;
box_y1    = 77.0;
box_h     = 12.0;       // rim height above the toolhead face
wall      = 2.0;
boss      = 6.0;        // square screw bosses in the inner corners
pilot     = 2.6;        // M3 self-tapping pilot
pilot_depth = 9.0;
lid_t     = 1.6;
win_lip   = 1.0;        // lid lip over each long PCB edge (2.2 caught the buttons)
rim_top   = plate_t + box_h;
board_cx  = (box_x0 + box_x1) / 2;
board_top = box_y1 - wall - boss - 0.4;  // board's USB edge, below the top bosses
board_cy  = board_top - board_l / 2;
wire_slot_y = [14.0, 24.0];              // right wall, towards the probe shelf

eps = 0.05;
sL  = fit_cl / cos(35);                  // clearance as an X shift on a 35 deg face

// ================================================================ plate
function xl(z) = dt_left_face + sL + dovetail_k * z;              // left dovetail
function xr(z, e = lobe_extra) = lobe_face + e - dovetail_k * z;  // right lobe dovetail (clamped)

module plate_section_xz(e = lobe_extra) {   // cross-section, X across, Y here = Z (thickness)
    polygon([[xl(0) + ef, 0], [xr(0, e) - ef, 0], [xr(ef, e), ef],
             [xr(plate_t, e), plate_t], [xl(plate_t), plate_t], [xl(ef), ef]]);
}

function arc(cx, cy, r, a0, a1, n = 16) =
    [for (i = [0:n]) let(a = a0 + i * (a1 - a0) / n) [cx + r * cos(a), cy + r * sin(a)]];

module plate_outline_xy() {             // face outline; dovetails are trimmed by the section
    re = right_edge - fit_cl;
    polygon(concat(
        [[-50, 0]],
        arc(re - corner_r, corner_r, corner_r, -90, 0),
        [[re, 12.16], [re + 0.17, 13.45], [re + 0.67, 14.66],
         [lobe_face + 2, 23.02], [lobe_face + 2, 59.10],   // lobe flat (trimmed by xr())
         [36.5, 60.83], [35.71, 61.39], [34.8, 62.45], [34.2, 63.75], [re, 65.16]],
        arc(re - corner_r, plate_h - corner_r, corner_r, 0, 90),
        [[-50, plate_h]]));
}

module plate_body(e = lobe_extra) {
    intersection() {
        linear_extrude(plate_t) plate_outline_xy();
        translate([0, plate_h + 1, 0]) rotate([90, 0, 0])
            linear_extrude(plate_h + 2) plate_section_xz(e);
    }
}

module channel_cut() {                  // full-height channel in the machine face
    d = ch_depth + ch_sag;
    ell = [for (i = [0:24]) let(a = 180 - i * 90 / 24)          // left round, top-left going up
           [-ch_half_floor + ch_r * cos(a), d * sin(a)]];
    elr = [for (i = [0:24]) let(a = 90 - i * 90 / 24)
           [ch_half_floor + ch_r * cos(a), d * sin(a)]];
    translate([0, plate_h + 1, 0]) rotate([90, 0, 0])
        linear_extrude(plate_h + 2)
            polygon(concat([[-ch_half_floor - ch_r - 0.01, -1]], ell, elr,
                           [[ch_half_floor + ch_r + 0.01, -1]]));
}

module centreline_holes() {             // blind, from the machine face to 0.6 below the top
    for (h = cl_holes) translate([h[0], h[1], -1]) cylinder(d = cl_hole_d, h = plate_t - 0.6 + 1);
}

module notch_cut() {
    w = notch_w + notch_cl;
    yc = notch_top - w / 2;
    translate([notch_x, 0, -1]) linear_extrude(notch_depth + 1)
        polygon(concat(
            [[-w / 2 - notch_chamfer, -1], [-w / 2 - notch_chamfer, 0], [-w / 2, notch_chamfer]],
            arc(0, yc, w / 2, 180, 0, 24),
            [[w / 2, notch_chamfer], [w / 2 + notch_chamfer, 0], [w / 2 + notch_chamfer, -1]]));
}

// ================================================================ probe shelf
module shelf() {
    translate([shelf_x0, shelf_under, plate_t - eps])
        cube([shelf_x1 - shelf_x0, shelf_t, shelf_z1 - plate_t + eps]);
    for (gx = [shelf_x0, shelf_x1 - gusset_w])
        hull() {
            translate([gx, shelf_under - gusset_drop, plate_t - eps])
                cube([gusset_w, gusset_drop + eps, 0.6]);
            translate([gx, shelf_under - 0.6, plate_t - eps])
                cube([gusset_w, 0.6 + eps, gusset_reach]);
        }
}

module teardrop2d(r) {                  // tip points +Y of the 2D shape (= print up)
    hull() { circle(r); translate([0, r * sqrt(2) - 0.01]) square(0.02, center = true); }
}

module shelf_cuts() {
    for (s = [-1, 1]) {
        hx = beam_x + s * probe_hole_pitch / 2;
        // M3 clearance, vertical as mounted (horizontal in print: teardrop, tip up)
        translate([hx, shelf_under + shelf_t + 1, beam_z]) rotate([90, 0, 0])
            linear_extrude(shelf_t + 2) teardrop2d(m3_clear / 2);
        // nut trap from the shelf top
        translate([hx, shelf_under + shelf_t + 1, beam_z]) rotate([90, 0, 0])
            linear_extrude(m3_nut_depth + 1)
                rotate(30) circle(d = m3_nut_af / cos(30), $fn = 6);
    }
    // cable slot over the probe body
    translate([beam_x - cable_slot[0] / 2, shelf_under - 1, beam_z - cable_slot[1] / 2])
        cube([cable_slot[0], shelf_t + 2, cable_slot[1]]);
}

// ================================================================ ESP32 box
module box() {
    difference() {
        translate([box_x0, box_y0, plate_t - eps])
            cube([box_x1 - box_x0, box_y1 - box_y0, box_h + eps]);
        translate([box_x0 + wall, box_y0 + wall, plate_t - 1])
            cube([box_x1 - box_x0 - 2 * wall, box_y1 - box_y0 - 2 * wall, box_h + 2]);
    }
    // corner bosses (lid screws)
    for (bx = [box_x0 + wall - eps, box_x1 - wall - boss + eps],
         by = [box_y0 + wall - eps, box_y1 - wall - boss + eps])
        translate([bx, by, plate_t - eps]) cube([boss, boss, box_h + eps]);
    // board corner posts (under the PCB) and L locators (around it)
    pcb_bottom = rim_top - board_t;
    hw = board_w / 2 + board_cl;
    hl = board_l / 2 + board_cl;
    for (sx = [-1, 1], sy = [-1, 1]) {
        cx = board_cx + sx * (board_w / 2 - 1.25);
        cy = board_cy + sy * (board_l / 2 - 1.25);
        translate([cx - 1.25, cy - 1.25, plate_t - eps]) cube([2.5, 2.5, pcb_bottom - plate_t + eps]);
        // L locator outside the corner, rising to the rim
        lx = board_cx + sx * hw;
        ly = board_cy + sy * hl;
        // both arms include the 1.2 x 1.2 corner square, so they overlap (not edge-touch)
        translate([sx > 0 ? lx : lx - 1.2, sy > 0 ? ly - 3 : ly - 1.2, plate_t - eps])
            cube([1.2, 4.2, box_h + eps]);
        translate([sx > 0 ? lx - 3 : lx - 1.2, sy > 0 ? ly : ly - 1.2, plate_t - eps])
            cube([4.2, 1.2, box_h + eps]);
    }
}

module box_cuts() {
    // lid screw pilots
    for (bx = [box_x0 + wall + boss / 2, box_x1 - wall - boss / 2],
         by = [box_y0 + wall + boss / 2, box_y1 - wall - boss / 2])
        translate([bx, by, rim_top - pilot_depth]) cylinder(d = pilot, h = pilot_depth + 1);
    // USB-C plug opening through the top wall, open to the rim
    translate([board_cx - usb_w / 2, box_y1 - wall - 1, rim_top - board_t - 2.5])
        cube([usb_w, wall + 2, board_t + 2.5 + 1]);
    // wire slot in the right wall, open to the rim
    translate([box_x1 - wall - 1, wire_slot_y[0], plate_t + 2])
        cube([wall + 2, wire_slot_y[1] - wire_slot_y[0], box_h]);
}

// ================================================================ lid
module lid() {
    lw = box_x1 - box_x0;
    ll = box_y1 - box_y0;
    difference() {
        hull() for (x = [1, lw - 1], y = [1, ll - 1]) translate([x, y, 0]) cylinder(r = 1, h = lid_t);
        // window: OLED, buttons and the USB connector. The lid keeps a lip over each long PCB edge.
        // At a 2.2 lip the window printed 16.45 wide and caught the RST and BOOT buttons, so the
        // lip is now 1.0 and the window runs 1.5 past the far end of the board.
        translate([board_cx - box_x0 - (board_w / 2 - win_lip), board_cy - box_y0 - board_l / 2 - 1.5, -1])
            cube([board_w - 2 * win_lip, board_l + 2, lid_t + 2]);
        // USB notch to the top edge
        translate([board_cx - box_x0 - usb_w / 2, board_top - box_y0 - 1, -1])
            cube([usb_w, ll, lid_t + 2]);
        // screw holes
        for (bx = [wall + boss / 2, lw - wall - boss / 2], by = [wall + boss / 2, ll - wall - boss / 2])
            translate([bx, by, -1]) cylinder(d = m3_clear, h = lid_t + 2);
    }
}

// ================================================================ assembly
module head() {
    difference() {
        union() { plate_body(); shelf(); box(); }
        channel_cut();
        notch_cut();
        centreline_holes();
        shelf_cuts();
        box_cuts();
    }
}

module probe_dummy(deployed = true) {   // visual only
    color("orange") {
        translate([beam_x - probe_hole_pitch / 2 - 4, shelf_under - probe_flange_t, beam_z - 5.75])
            cube([probe_hole_pitch + 8, probe_flange_t, 11.5]);
        translate([beam_x - probe_w / 2, shelf_under - probe_total, beam_z - probe_d / 2])
            cube([probe_w, probe_total - probe_flange_t, probe_d]);
    }
    color("silver") translate([beam_x, shelf_under - probe_total - (deployed ? pin_deployed : pin_retracted), beam_z])
        rotate([-90, 0, 0]) cylinder(d = 1.8, h = (deployed ? pin_deployed : pin_retracted) + 1);
}

if (part == "plate") head();
else if (part == "lid") lid();
else if (part == "fit_test") {          // plate only (~35 g): check the slide and the notch first
    difference() { plate_body(); channel_cut(); notch_cut(); centreline_holes(); }
}
else if (part == "assembly") {
    color("dimgray") head();
    color("darkslategray") translate([box_x0, box_y0, rim_top]) lid();
    probe_dummy();
    color("purple") translate([board_cx - board_w / 2, board_cy - board_l / 2, rim_top - board_t])
        cube([board_w, board_l, board_t]);
}
