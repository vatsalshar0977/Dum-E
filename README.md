# B.U.D.-E — experimental desktop robotic arm assistant

A compact, Python-first arm concept with original personality, inspired by workshop robots. **Prototype / design study, not an assemble-and-operate kit.** Start with [verification status](docs/verification_status.md) before buying, printing or powering anything. The older Phase 1/2 documents contain unverified claims about price, payload, mechanics and safety; the existing STLs are **not verified to fit or function**. Do not operate near people or edges. Neither camera tracking nor current monitoring is a human safety system.

## Phases and files

| Phase | Files | Status |
|---|---|---|
| 1 — concept | `config/robot_config.yaml`, `docs/phase1_requirements_and_design.md`, `docs/bom_phase1.csv`, `src/phase1_sizing/` | Budget/geometry **estimates only**. Quoted ~$58.50 is not a current verified purchase price. |
| 2 — mechanical | `cad/`, `docs/phase2_mechanical_and_3d_printing.md` | Existing STLs contain critical unverified geometry; **not ready for full assembly**. |
| 3 — electronics | `docs/phase3_electronics_and_wiring.md`, `firmware/phase3_bench_diagnostic/` | One-servo **unloaded diagnostic only**. Firmware has not been flashed/tested on real boards. Old Phase 1 firmware is superseded. |
| 4 — simulation | `urdf/bude_arm.urdf`, `src/phase4_simulation/sim_runner.py`, `docs/phase4_simulation.md` | Box-proxy URDF; stdlib kinematic demo and optional PyBullet physics. No grasp/collision certification. |
| 5 — controls | `src/phase5_control/`, `docs/phase5_control_kinematics_moveit2.md` | FK/IK and sampled path checks. No hardware driver or working ROS 2/MoveIt 2 package. |
| 6 — perception/mobile concept | `src/phase6_perception/`, `docs/phase6_perception_and_mobile_navigation.md` | Camera calibration tool, ray-plane math, offline stop policy. No live detection, autonomous fetch or built rover. |
| 7/8 — voice, personality, safety trials | future | Not built. |

The mathematical geometry is base height 55 mm, upper 80 mm, forearm 75 mm, wrist and tool 70 mm (nominal max radial reach 225 mm). Actual reach and payload depend on a **redesigned and measured** assembly. No physical component alignment has been verified.

## Run offline checks from the repository root

On Windows PowerShell use `python` where examples say `python3`:

```sh
python3 -m unittest discover -s tests -v
python3 src/phase4_simulation/sim_runner.py
python3 src/phase5_control/kinematics.py
python3 src/phase6_perception/vision_mobile_navigator.py
```

For *optional* box-proxy rigid-body simulation, create a virtual environment then install `numpy pybullet` and run `python src/phase4_simulation/sim_runner.py --backend pybullet` (add `--gui` for desktop GUI). See [Phase 4](docs/phase4_simulation.md). No installation is needed for the default offline tests. Camera calibration needs `numpy opencv-python` and **your own chessboard photos**, see [Phase 6](docs/phase6_perception_and_mobile_navigation.md).

## Next physical milestone

Redesign a **single joint** in proper Boolean CAD from measured servo dimensions, print a mating test coupon, inspect cross-sections in a slicer and test-fit **unpowered**. Use a rated metal clamp rather than the existing printed clamp for any supported arm. Then follow [Phase 3](docs/phase3_electronics_and_wiring.md) for a single *unloaded* servo and a physically verified, DC-rated power cutoff. Do **not** trust a shared INA219 to protect people from collisions.

Open-source references: [SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100), [ROBOTIS OpenManipulator](https://github.com/ROBOTIS-GIT/open_manipulator), [PyBullet](https://github.com/bulletphysics/bullet3), [MoveIt 2](https://github.com/moveit/moveit2) and [OpenCV](https://github.com/opencv/opencv). No third-party code or CAD is copied into this repository by the new phases.
