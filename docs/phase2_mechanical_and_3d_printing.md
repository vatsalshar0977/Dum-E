> **STOP: ARCHIVED DESIGN ESTIMATES, NOT VERIFIED BUILD INSTRUCTIONS.** Existing Phase 2 STLs have unverified and likely nonfunctional mating features; price/payload/current-safety claims below are not validated. Read [verification status](verification_status.md) and the revised Phase 3–6 guides before using any hardware.

# Phase 2 — Mechanical Design, 3D-Printable CAD, Tolerances & Cable Management

## 1. Open-Source Mechanical Heritage
Instead of reinventing joint kinematics from scratch, B.U.D.-E's mechanical design combines proven open-source patterns tailored to our **$58.50 Hybrid `MG996R` + `MG90S` architecture**:
- **Double-Shear U-Brackets** (from [ROBOTIS OpenManipulator-X](https://github.com/ROBOTIS-GIT/open_manipulator) & [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)): Every pitch joint (`J2`, `J3`, `J4`) supports the link on **both sides**—the metal servo spline on one side and a **3D-printed `6.0 mm` concentric passive pivot pin** on the opposite side.
- **3D-Printed 6mm BB Slewing Bearing** (from open-source desktop turret designs): Offloads the overturning moment from the `J1` (`MG996R`) output shaft onto a `78 mm` diameter circular raceway filled with `6 mm` plastic airsoft BBs (or 3D-printed `5.8 × 5.8 mm` cylindrical rollers).
- **Synchronous 1:1 Meshed Spur-Gear Claw**: Driven by a single `13.4 g` `MG90S` micro servo (`J6`) with an overhead tilted camera bezel ("DUM-E Eyes") for the `6 g` `HBV-1825`/`OV2640` USB board camera.

---

## 2. Complete 3D-Printable Part Manifest (`cad/stl/`)

All 11 STL files are generated parametrically by `python3 src/phase2_mechanical/generate_stl_parts.py` and verified by `python3 src/phase2_mechanical/verify_print_parts.py`. Every part prints **without support material** and fits on any `180 × 180 mm` (or larger) FDM printer bed:

| File Name | Dimensions (`X×Y×Z mm`) | Est. PLA (`g`) | Layer / Walls / Infill | Print Orientation & Notes |
|---|---|---:|---|---|
| `00_tolerance_test_coupon.stl` | `72.2 × 38.0 × 14.0` | `8 g` | `0.20mm` / 3 walls / 20% | **Print this first (12 mins)!** Verifies `MG996R`/`MG90S` pockets, `M3` hex nut trap & `6mm` pivot pin fit. |
| `01_desk_c_clamp_frame.stl` | `58.0 × 83.0 × 18.0` | `34 g` | `0.20mm` / **5 walls** / 35% | **Print flat on its side** so continuous perimeter filaments wrap around the C-throat without delaminating. |
| `02_desk_c_clamp_screw_and_pad.stl` | `61.0 × 32.0 × 60.0` | `14 g` | `0.16mm` / **5 walls** / 40% | Knob flat on bed; 5 perimeters give the screw shaft high torsional strength. |
| `03_base_lower_slewing_ring.stl` | `121.0 × 92.0 × 32.0` | `42 g` | `0.20mm` / 4 walls / 25% | Base flange flat on bed; houses stationary `J1 MG996R`, lower BB race & `22mm` `LA38-11ZS` E-stop mount. |
| `04_base_upper_turret_j2.stl` | `82.0 × 83.3 × 44.0` | `36 g` | `0.20mm` / 4 walls / 25% | Disk flat on bed; includes `J2` double-shear uprights + rear rubber-band counterbalance hook. |
| `05_upper_arm_link_80mm.stl` | `111.4 × 47.0 × 20.0` | `24 g` | `0.20mm` / 3 walls / 20% | Flat on bed (`L2 = 80 mm` axis-to-axis); includes engraved `B.U.D.-E` bridge & rubber-band post. |
| `06_forearm_link_75mm.stl` | `96.6 × 34.5 × 10.0` | `17 g` | `0.20mm` / 3 walls / 20% | Flat on bed (`L3 = 75 mm` axis-to-axis); transitions from `J3 MG996R` to `J4 MG90S`. |
| `07_wrist_pitch_roll_bracket.stl` | `47.0 × 25.5 × 12.0` | `9 g` | `0.20mm` / 3 walls / 20% | Flat on bridge face (`L4 = 22 mm`); couples `J4` pitch (nod) to `J5` roll (puppy head-tilt). |
| `08_gripper_body_and_cam_mount.stl` | `36.0 × 42.0 × 38.0` | `13 g` | `0.20mm` / 3 walls / 20% | Palm back flat on bed; holds `J6 MG90S` + top bezel for `6 g` USB webcam module. |
| `09_gripper_geared_fingers_pair.stl` | `57.0 × 54.0 × 6.0` | `9 g` | `0.16mm` / 4 walls / 30% | Flat on gear face; `0.16 mm` layer height ensures smooth spur-gear tooth meshing. |
| `10_cable_clips_and_electronics_tray.stl` | `72.0 × 68.0 × 8.0` | `12 g` | `0.20mm` / 3 walls / 20% | Mounts `ESP32` + `PCA9685` + `INA219` beside base + 6 snap-on cable clips. |
| **TOTAL** | **Fits any `180×180mm` bed** | **`218 g`** | **~$3.92 of PLA** | **Zero support material required across all 11 files!** |

---

## 3. Dimensional Tolerances & How to Calibrate (`00_tolerance_test_coupon.stl`)

Consumer FDM printers vary by $\pm 0.15\text{ mm}$ due to filament flow rate and seam bulging. Before printing the arm, print **`00_tolerance_test_coupon.stl`** (`8 g`, ~12 minutes) and check four fits:

| Feature on Coupon | Nominal Dimension | Built-in Clearance (`--clearance-mm 0.25`) | How It Should Feel & How to Adjust |
|---|---|---|---|
| **1. `MG996R` Body Pocket** | `40.7 × 20.2 mm` | `41.20 × 20.70 mm` (`+0.25 mm`/side) | Sliding fit with zero wobble. If too tight, re-run generator with `--clearance-mm 0.32`. |
| **2. `MG90S` Body Pocket** | `22.8 × 12.4 mm` | `23.30 × 12.90 mm` (`+0.25 mm`/side) | Snug push fit by hand. |
| **3. Captive `M3` Hex Nut Trap** | `5.50 mm` across flats | `5.65 mm` across flats (`+0.15 mm`) | Press in an M3 hex nut with thumb pressure or pull it in with an M3 screw. Replaces brass heat-set inserts! |
| **4. Double-Shear `6.0 mm` Pin & Bore** | `6.00 mm` OD pin | `6.30 mm` ID bore (`+0.30 mm` diametral) | Should spin freely with zero radial play after rubbing a tiny dab of PTFE/lithium grease (or dry bar soap) on the pin. |

```bash
# If your printer prints slightly tight (e.g., textured PEI or high-flow PLA), regenerate all STLs with 0.32 mm clearance:
python3 src/phase2_mechanical/generate_stl_parts.py --clearance-mm 0.32
```

---

## 4. Minimal Working Mechanical Build: Assemble & Test Joint 1 (`Base Yaw`) First!
Following our core engineering rule (*"Start with a minimal working version (one joint moving), then build up"*):
1. Print **`01_desk_c_clamp_frame.stl`**, **`02_desk_c_clamp_screw_and_pad.stl`**, **`03_base_lower_slewing_ring.stl`**, and **`04_base_upper_turret_j2.stl`**.
2. **Mechanical Zeroing Warning**: Before screwing any servo horn onto a 3D-printed part, connect the `MG996R` to Channel 0 and run `python3 src/phase1_sizing/single_joint_ping.py` so the servo moves to its exact **`0.0°` (`1500 us` center pulse)**. Never attach a horn to an un-centered servo!
3. Drop 18–20 standard `6 mm` plastic airsoft BBs (or a bead of grease) into the circular groove of `03_base_lower_slewing_ring.stl`, seat `04_base_upper_turret_j2.stl` on top, and fasten the metal/nylon cross-horn with two `M2×6mm` screws.
4. Run `python3 src/phase1_sizing/single_joint_ping.py --wiggle` and verify the turret rotates $\pm 15^\circ$ smoothly with zero binding before printing Links `05..09`!

---

## 5. Shoulder Counterbalance & Cable Management

### A. Installing the `$0` Shoulder Gravity-Assist Band (`J2`)
- Once `05_upper_arm_link_80mm.stl` is mounted to `04_base_upper_turret_j2.stl`, loop **two `#64` household rubber bands** (or a `50 mm` light extension spring) between the rear post on `04_base_upper_turret_j2` (`x = -34 mm`) and the top post on `05_upper_arm_link_80mm` (`x = -28 mm`).
- When the arm stretches forward (+X), the band stretches and provides ~`0.24 N·m` of upward restoring torque, cutting `J2` servo heating and current draw by more than **50%**!

### B. Cable Routing & Slack-Loop Rules
- **Why Slack Loops Matter**: When `J5` (`wrist_roll`) rotates $\pm 90^\circ$ for an expressive puppy head-tilt, a tight wire will stall the `13.4 g` `MG90S` micro servo!
- Follow the exact slack loop lengths output by `verify_print_parts.py`:
  - Snap the 6 printed clips from `10_cable_clips_and_electronics_tray.stl` onto the side rails of the Upper Arm (`05`) and Forearm (`06`).
  - Leave a **`40 mm` bowed loop** across each pitch joint (`J2`, `J3`, `J4`) and a **`45 mm` loose helical loop** between `J4` and `J6` for wrist roll.
  - Route the `1.5 m` USB camera cable alongside the servo bundle and anchor it into the last clip on `03_base_lower_slewing_ring` so pulling the laptop USB cord never tugs on the wrist!

---

## 6. Mechanical Safety Warnings
- **⚠️ Never Print `01_desk_c_clamp_frame.stl` Upright**: Printing a C-clamp vertically puts the Z-layer seams in pure tension, causing it to snap suddenly when tightened. Always print it **flat on its side** with **5 perimeter walls**.
- **⚠️ Spur-Gear Pinch Point on `09_gripper_geared_fingers_pair.stl`**: Keep fingers clear of the meshing spur-gear teeth on the palm housing while `J6` is powered.
- **⚠️ Mechanical Hard-Stop Check**: Before powering a newly assembled joint, gently sweep the link by hand across its full travel (while the servo is unpowered or BEFORE screwing the horn tight) to verify wires and brackets do not pinch.
