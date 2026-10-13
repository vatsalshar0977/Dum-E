# B.U.D.-E — experimental desktop robotic arm assistant

A compact, Python-first arm concept with original personality, inspired by workshop robots. **Prototype / design study, not an assemble-and-operate kit.** Start with [verification status](docs/verification_status.md) before buying, printing or powering anything. The older Phase 1/2 documents contain unverified claims about price, payload, mechanics and safety. Do not operate near people or edges. Neither camera tracking nor current monitoring is a human safety system.

**All eight phases are now implemented.** One command runs every phase and prints a single
scorecard (`43 / 43 checks PASS` on this revision — with the caveat printed underneath it):

```sh
python3 src/phase8_safety_testing/run_full_acceptance_suite.py
```

## Phases and files

| Phase | Files | Status |
|---|---|---|
| 1 — concept | `config/robot_config.yaml`, `docs/phase1_requirements_and_design.md`, `docs/bom_phase1.csv`, `src/phase1_sizing/` | Budget/geometry **estimates only**. The `$58.50` stationary / `$66.50` mobile totals are dated estimates, not quoted prices. |
| 2 — mechanical | `cad/stl/`, `src/phase2_mechanical/`, `docs/phase2_mechanical_and_3d_printing.md` (+ Appendix B) | 13 generated STLs with **true hollow cavities** (no overlap tricks), datasheet spline offsets giving **0.0000 mm** measured coaxiality, and +0.60 mm telescoping straddle clearance. `verify_assembly_fit.py` measures 206 checks straight from the delivered files: **206 / 206 PASS**. Still not printed; tolerances need the coupon. |
| 3 — electronics | `docs/phase3_electronics_and_wiring.md`, `firmware/phase3_full_6dof_esp32/` | **Full firmware** for 6 arm channels + 2 rover wheels, a **hardware cliff reflex** on GPIO 25/26 (reverse 350 ms, then neutral, target <5 ms), a non-blocking HC-SR04, the ESP32 task watchdog (FMEA #11) and I²C bus recovery (FMEA #12). Its safety logic is **compiled and executed on a host** — 73 + 65 checks — but the sketch has **never been flashed**, and the wheel directions must be proved on the floor before any cliff test. |
| 4 — simulation | `urdf/bude_arm.urdf`, `src/phase4_simulation/` | Box-proxy URDF, a stdlib kinematic demo, an optional PyBullet path, and an interactive schematic viewer. No grasp or collision certification. |
| 5 — controls | `src/phase5_control/`, `ros2_ws/src/bude_moveit_config/` | FK, fail-closed IK with an opt-in pitch ladder, minimum-jerk planning, analytic keep-outs, and MoveIt 2/`ros2_control` config drafts that have **never been loaded**. No hardware driver. |
| 6 — perception/mobile | `src/phase6_perception/` | Calibration tool, design-model camera pipeline (round trip closes to 1e-14 mm *in the model*), PCA grasp-roll estimation, and a fail-closed offline drive policy. No live detection, no autonomous driving, no built rover. |
| 7 — voice & personality | `src/phase7_voice_personality/`, docs, viewer panel | Six **100 Hz minimum-jerk** gestures over 6 joints + 2 wheels, with every segment validated against a declared speed budget. Offline-first speech (Whisper, optional) plus a **deterministic text parser** that needs nothing installed. In-browser panel plays the real command stream. |
| 8 — safety & trials | `src/phase8_safety_testing/`, `docs/phase8_safety_testing_and_bringup_checklist.md` | A supervisor that evaluates six fault classes in a fixed order, a latched e-stop that syncs targets to feedback before release, JSONL/CSV telemetry, six fault injectors, an FMEA table and a bring-up checklist. All of it simulated. |

The mathematical geometry is base height 55 mm, upper 80 mm, forearm 75 mm, wrist and tool 70 mm (nominal max radial reach 225 mm). Actual reach and payload depend on a **redesigned and measured** assembly. No physical component alignment has been verified.

## Run offline checks from the repository root

On Windows PowerShell use `python` where examples say `python3`:

```sh
python3 src/phase8_safety_testing/run_full_acceptance_suite.py    # everything: 8 phases, one scorecard
python3 -m unittest tests.test_foundations tests.test_phase5_phase6 tests.test_phase7_phase8
python3 src/phase2_mechanical/generate_stl_parts.py     # regenerate all 13 STLs
python3 src/phase2_mechanical/verify_print_parts.py     # mesh integrity + measured bolt circle
python3 src/phase2_mechanical/verify_assembly_fit.py    # 206 checks straight from cad/stl/
python3 src/phase5_control/kinematics.py
python3 src/phase5_control/trajectory_and_collision.py
python3 src/phase6_perception/camera_and_hand_eye_calib.py
python3 src/phase6_perception/vision_mobile_navigator.py
python3 src/phase7_voice_personality/personality_engine.py    # validates all six gestures
python3 src/phase7_voice_personality/voice_assistant.py       # offline, no microphone: demo
python3 src/phase7_voice_personality/voice_assistant.py -c "hand me the screwdriver"
python3 src/phase8_safety_testing/safety_supervisor_and_logger.py --echo
python3 -m unittest tests.test_firmware            # compiles + runs the firmware on a host
python3 src/phase2_mechanical/verify_assembly_fit.py  # 206 checks straight from cad/stl/
```

The firmware needs no toolchain to be *checked* — `tests/test_firmware.py` compiles it with
`g++` and drives the real `.ino` against a simulated ESP32 (simulated clock, GPIO, PCA9685
register decoding and serial port). To build it for the board itself, open
`firmware/phase3_full_6dof_esp32/phase3_full_6dof_esp32.ino` in the Arduino IDE with the ESP32
core installed (no libraries beyond the core) or point PlatformIO at the same folder.

```sh

Interactive schematic viewer (standard library only, no third-party JavaScript) — it now has a
**Voice & personality** panel that plays every gesture on the drawn arm at real speed, and a
text box wired to the same parser the voice path uses:

```sh
python3 src/phase4_simulation/web_sim_server.py         # then open http://localhost:8080
```

Optional extras, none required for the checks above: `numpy pybullet` for the box-proxy rigid-body
simulation (`python src/phase4_simulation/sim_runner.py --backend pybullet --gui`); `numpy
opencv-python` plus your own chessboard photos for real camera calibration;
`faster-whisper sounddevice soundfile` (or `openai-whisper`) if you want to talk to it:

```sh
python -m pip install faster-whisper sounddevice soundfile
python3 src/phase7_voice_personality/voice_assistant.py --mic --seconds 4
```

Telemetry lands in `logs/` (git-ignored). The viewer's browser model is cross-checked against the
Python kinematics by the test suite when `node` is available.

## Next physical milestone

Redesign **one joint** in proper Boolean CAD from measured servo dimensions, print a mating test
coupon, inspect cross-sections in a slicer and test-fit **unpowered**. Use a rated metal clamp
rather than the printed clamp for any supported arm. Then follow
[Phase 3](docs/phase3_electronics_and_wiring.md) for a single *unloaded* servo and a physically
verified, DC-rated power cutoff, and only then work through the
[Phase 8 bring-up checklist](docs/phase8_safety_testing_and_bringup_checklist.md) in order — it
ends with the first powered gesture and the first deliberate stall test. Do **not** trust a shared
INA219 to protect people from collisions. If you build the rover, start on the **floor** with the
arm stowed and a supervisor.

Open-source references: [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), [ROBOTIS OpenManipulator](https://github.com/ROBOTIS-GIT/open_manipulator), [PyBullet](https://github.com/bulletphysics/bullet3), [MoveIt 2](https://github.com/moveit/moveit2), [OpenCV](https://github.com/opencv/opencv) and [whisper.cpp](https://github.com/ggerganov/whisper.cpp) (the idea behind the optional offline speech path). No third-party code or CAD is copied into this repository.
