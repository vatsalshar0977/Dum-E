# B.U.D.-E (Benchtop Utility & Dexterity Engine) — Sub-$75 3D-Printed Edition

An open-source, 3D-printable, ultra-low-cost (**`$58.50` total!**) desktop robotic arm assistant inspired by **DUM-E** from *Iron Man* and built on proven open-source foundations ([SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), [IKPy](https://github.com/Phylliade/ikpy), [MuJoCo](https://github.com/google-deepmind/mujoco), [PyBullet](https://github.com/bulletphysics/bullet3), [Ultralytics YOLO](https://github.com/ultralytics/ultralytics), and [Whisper](https://github.com/openai/whisper)).

---

## Project Roadmap & Phase Status

- [x] **Phase 1 — Requirements, Naming, Architecture & System Design (Sub-$75 Edition)**
  - 5-DOF arm + 1-DOF geared 2-finger claw (`22.5 cm` compact reach, `100–160 g` payload, only `~218 g` of PLA to print!)
  - **Hybrid Metal-Gear Actuators (`$19.50` for all 6 servos)**: 3× `MG996R`/`DS3218` (Base, Shoulder, Elbow) + 3× `MG90S` 13.4g micro servos (Wrist Pitch, Wrist Roll, Claw)
  - **Smart Current-Sensing Controller (`$10.00` total)**: `ESP32-WROOM-32` (`$4.50`) + `PCA9685` 16-ch PWM driver (`$3.00`) + `INA219` I2C current sensor (`$2.50`) for real-time stall/collision protection and gentle current-limited gripping
  - **Maximal 3D Printing**: 3D-printed Desk C-Clamp, 3D-printed Base Slewing Bearing, 3D-printed Captive Nut Traps, and 3D-printed Shoulder Spring/Rubber-Band Gravity Assist
  - Complete Bill of Materials (`docs/bom_phase1.csv` — **`$58.50` total**) & engineering spec (`docs/phase1_requirements_and_design.md`)
  - Runnable torque/power calculator (`src/phase1_sizing/torque_and_power_calc.py`) & 1-joint bringup script (`src/phase1_sizing/single_joint_ping.py`)
- [x] **Phase 2 — Mechanical Design & 3D Printing** *(Current Phase)*
  - Parametric STL mesh generator (`src/phase2_mechanical/generate_stl_parts.py`) producing all 11 support-free printable `.stl` files in `cad/stl/`
  - Editable OpenSCAD parametric source (`cad/scad/bude_arm_parametric.scad`)
  - 12-minute first-print calibration gauge (`cad/stl/00_tolerance_test_coupon.stl`) + STL/bed/cable verifier (`src/phase2_mechanical/verify_print_parts.py`)
  - Full Mechanical & Slicer Guide (`docs/phase2_mechanical_and_3d_printing.md`)
- [ ] **Phase 3 — Electronics, Wiring, Power & E-Stop Circuit**
- [ ] **Phase 4 — Simulation First (URDF + PyBullet / MuJoCo)**
- [ ] **Phase 5 — Control Software, FK/IK, Trajectories & ROS 2 / Python Bridge**
- [ ] **Phase 6 — Perception (Camera & Hand-Eye Calibration, YOLO Object + Hand Tracking)**
- [ ] **Phase 7 — Voice Commands & Expressive Personality Engine**
- [ ] **Phase 8 — Safety Verification, Testing & Iteration**

---

## Repository Structure

```text
Dum-E/
├── README.md                                              # Project overview & phase status tracker
├── requirements.txt                                       # Cross-platform Python dependencies (macOS/Windows/Linux)
├── cad/
│   ├── scad/
│   │   └── bude_arm_parametric.scad                       # Editable OpenSCAD parametric source (tolerances, links, label)
│   └── stl/                                               # 11 ready-to-slice binary STL files (218 g PLA, zero supports)
│       ├── 00_tolerance_test_coupon.stl
│       ├── 01_desk_c_clamp_frame.stl
│       ├── 02_desk_c_clamp_screw_and_pad.stl
│       ├── 03_base_lower_slewing_ring.stl
│       ├── 04_base_upper_turret_j2.stl
│       ├── 05_upper_arm_link_80mm.stl
│       ├── 06_forearm_link_75mm.stl
│       ├── 07_wrist_pitch_roll_bracket.stl
│       ├── 08_gripper_body_and_cam_mount.stl
│       ├── 09_gripper_geared_fingers_pair.stl
│       └── 10_cable_clips_and_electronics_tray.stl
├── config/
│   └── robot_config.yaml                                  # Master parameters: link lengths, joint limits, PCA9685 channels, INA219 caps
├── docs/
│   ├── phase1_requirements_and_design.md                  # Full Phase 1 Sub-$75 engineering spec & trade-off study
│   ├── bom_phase1.csv                                     # Complete Sub-$75 Bill of Materials with concrete part numbers & prices
│   └── phase2_mechanical_and_3d_printing.md               # Phase 2 CAD, slicer settings, tolerances, cable routing & assembly
├── firmware/
│   └── phase1_single_joint_esp32/
│       └── phase1_single_joint_esp32.ino                  # Minimal ESP32 Arduino sketch: 1-joint 100Hz cosine move + INA219 watchdog
└── src/
    ├── __init__.py
    ├── phase1_sizing/
    │   ├── __init__.py
    │   ├── torque_and_power_calc.py                       # Static gravity torque, safety factor & power budget verifier
    │   ├── workspace_envelope_check.py                    # Planar/3D forward kinematics reach & expressive pose verifier
    │   └── single_joint_ping.py                           # Minimal 1-servo bringup script over ESP32+PCA9685+INA219 (supports --dry-run)
    └── phase2_mechanical/
        ├── __init__.py
        ├── generate_stl_parts.py                          # Parametric Python STL mesh generator for all 11 printable parts
        └── verify_print_parts.py                          # Binary STL integrity, 180mm bed-fit & cable slack-loop verifier
```

## Quick Start (Phase 1 Verification)

```bash
# 1. Run the static torque, payload & electrical power budget calculator:
python3 src/phase1_sizing/torque_and_power_calc.py

# 2. Test custom link lengths or payloads on the fly:
python3 src/phase1_sizing/torque_and_power_calc.py --payload-g 120 --upper-arm-mm 80

# 3. Run the minimal single-joint bringup script in dry-run mode:
python3 src/phase1_sizing/single_joint_ping.py --dry-run
```
