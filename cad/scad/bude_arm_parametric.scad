// ============================================================================
// B.U.D.-E (Sub-$75 3D-Printed Edition) — Parametric OpenSCAD Source
// Open in OpenSCAD (https://openscad.org) or FreeCAD OpenSCAD Workbench
// to customize tolerances, link lengths, desk thickness, or engraved name!
// ============================================================================

/* [Robot Identity & Link Lengths] */
robot_label         = "B.U.D.-E";
L1_base_height_mm   = 55.0;
L2_upper_arm_mm     = 80.0;
L3_forearm_mm       = 75.0;
L4_wrist_pitch_mm   = 22.0;
L5_gripper_tip_mm   = 48.0;
max_desk_thick_mm   = 48.0;

/* [3D Printer Tolerance Calibration (Adjust after printing 00_tolerance_test_coupon.stl)] */
xy_clearance_mm     = 0.25; // Standard 0.4mm nozzle PLA clearance (0.20 for Bambu/Prusa, 0.30 for Ender)
m3_through_dia_mm   = 3.25; // M3 screw clearance hole
m3_nut_flat_mm      = 5.65; // M3 hex nut across-flats press fit (standard nut is 5.50mm)
m2_self_tap_dia_mm  = 1.75; // M2 self-tapping screw pilot hole for servo mounting ears
passive_pin_dia_mm  = 6.00; // Double-shear passive pivot pin outer diameter
passive_bore_dia_mm = 6.30; // Matching double-shear pivot bore inner diameter
bb_pellet_dia_mm    = 6.00; // Standard 6mm airsoft BB (or printed cylindrical roller)

/* [Standard Servo Dimensions (MG996R / DS3218 & MG90S)] */
mg996r_body_l = 40.7 + 2 * xy_clearance_mm;
mg996r_body_w = 20.2 + 2 * xy_clearance_mm;
mg996r_ear_hole_span = 48.0;

mg90s_body_l  = 22.8 + 2 * xy_clearance_mm;
mg90s_body_w  = 12.4 + 2 * xy_clearance_mm;
mg90s_ear_hole_span  = 27.5;

$fn = 48;

// --- Reusable Captive M3 Hex-Nut Trap Module (Replaces Brass Heat-Set Inserts!) ---
module m3_captive_nut_trap(depth=12.0, nut_z=3.0) {
    cylinder(h=depth, d=m3_through_dia_mm, center=false);
    translate([0, 0, nut_z])
        cylinder(h=2.6, d=m3_nut_flat_mm / cos(30), $fn=6, center=false);
}

// --- Reusable Servo Pocket Module ---
module servo_pocket_mg996r(depth=22.0) {
    translate([-mg996r_body_l/2, -mg996r_body_w/2, 0])
        cube([mg996r_body_l, mg996r_body_w, depth]);
    for (sx = [-1, 1])
        for (sy = [-1, 1])
            translate([sx * mg996r_ear_hole_span/2, sy * 5.0, depth - 10])
                cylinder(h=12, d=m2_self_tap_dia_mm);
}

module servo_pocket_mg90s(depth=14.0) {
    translate([-mg90s_body_l/2, -mg90s_body_w/2, 0])
        cube([mg90s_body_l, mg90s_body_w, depth]);
    for (sx = [-1, 1])
        translate([sx * mg90s_ear_hole_span/2, 0, depth - 8])
            cylinder(h=10, d=m2_self_tap_dia_mm);
}

// --- Part 00: 12-Minute Tolerance Calibration Coupon ---
module tolerance_test_coupon() {
    difference() {
        union() {
            translate([-36, -19, 0]) cube([72, 38, 6]);
            // Test 6.0mm passive pivot pin standing on right side
            translate([28, 10, 6]) cylinder(h=10, d=passive_pin_dia_mm);
        }
        // 1. MG996R test pocket
        translate([-12, 0, 1.5]) servo_pocket_mg996r(depth=10);
        // 2. MG90S test pocket
        translate([20, -8, 1.5]) servo_pocket_mg90s(depth=10);
        // 3. Double-shear passive bore (6.30 mm)
        translate([18, 10, -1]) cylinder(h=10, d=passive_bore_dia_mm);
        // 4. Captive M3 hex nut trap
        translate([5, 10, -0.1]) m3_captive_nut_trap(depth=8, nut_z=2.0);
    }
}

// --- Part 05: Upper Arm Link (L2 = 80 mm) with Engraved Robot Name ---
module upper_arm_link() {
    difference() {
        union() {
            // Left & Right double-shear side beams
            translate([-L2_upper_arm_mm/2 - 8, -22, 0]) cube([L2_upper_arm_mm + 16, 5, 12]);
            translate([-L2_upper_arm_mm/2 - 8,  17, 0]) cube([L2_upper_arm_mm + 16, 5, 12]);
            // Center bridges + rubber-band counterbalance hook lug
            translate([-22, -20, 0]) cube([12, 40, 12]);
            translate([ 12, -20, 0]) cube([12, 40, 12]);
            translate([-28,   0, 12]) cylinder(h=8, d=7.0);
        }
        // Shoulder pitch (J2) & Elbow pitch (J3) axis bores
        translate([-L2_upper_arm_mm/2, 0, -1]) cylinder(h=14, d=passive_bore_dia_mm);
        translate([ L2_upper_arm_mm/2, 0, -1]) cylinder(h=14, d=passive_bore_dia_mm);
        // Engraved nameplate on bridge
        translate([-16, 0, 10.8])
            linear_extrude(height=2.0)
                text(robot_label, size=4.5, halign="center", valign="center");
    }
}

// Render the tolerance test coupon and upper arm link side by side for preview:
tolerance_test_coupon();
translate([0, 55, 0]) upper_arm_link();
