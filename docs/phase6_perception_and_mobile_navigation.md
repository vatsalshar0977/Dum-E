# Phase 6 — perception and a **future** mobile-base option

This document is the requested replacement for the missing Phase 6 guide. It is **new work**, not the missing original file. Camera calibration and a fail-closed *offline* navigation policy are implemented; live YOLO/hand tracking, grasping, hand-eye calibration and autonomous driving are **not** implemented.

## 1. Camera intrinsic calibration, step by step

1. Choose a real UVC USB camera that works on your actual macOS/Windows machine and measure its PCB, lens and cable before designing a mount. A generic OV2640 camera board is **not necessarily UVC USB**; verify the seller's USB interface with your OS. Mount without obstructing the gripper.
2. Print a flat chessboard (e.g. **9×6 inner corners**); measure the actual square size with a ruler (e.g. 20.0 mm, not assumed). Fix the focus/resolution and disable automatic zoom. Collect 15–25 sharp photos across the FOV at varied distance/tilt; save them outside Git as `calibration_photos/*.png`.
3. Install `python -m pip install opencv-python numpy`; run on Windows PowerShell:
   ```powershell
   python src\phase6_perception\camera_and_hand_eye_calib.py calibrate --images 'calibration_photos\*.png' --square-mm 20 --cols 9 --rows 6 --output camera_intrinsics.json
   ```
   The tool rejects fewer than 8 usable photos. The output contains measured fx, fy, cx, cy, lens distortion, RMS reprojection error and image size. Collect independent held-out shots and measure actual localization error with a ruler; do not assert a zero-error calibration on synthetic points. Avoid checking calibration photos into Git.
4. **Hand-eye extrinsics still required:** the 3×3 camera-to-base rotation and translation must be measured for each arm pose using a fixed fiducial/board and recorded joint states. The transform changes when the wrist moves. `intersect_table(u,v,intrinsics,transform,z_mm)` projects an undistorted pixel ray to a plane of **known object height**. Undistort pixels with OpenCV before calling. It validates rotation, rejects rays parallel to/behind the desk, but cannot infer object height from a single view.
5. Test localization at several known desk points using a ruler. Do not grasp until errors, latency, height assumptions and end-effector clearances are verified. Use an operator-approved pick point and low-speed empty gripper trials first.

**Detection plan, not working pipeline:** start with color fiducials/AprilTags (OpenCV), then manually labeled object detection (e.g. Ultralytics model if its license fits your deployment) and a separate hand detector. Keep a person out of the moving arm's workspace; camera-only hand tracking is not collision protection. Candidate must pass confidence threshold and stay stable across multiple frames; unknown object => no motion. No LLM-generated direct motion commands.

## 2. Mobile base: scope and safety

A mobile base is an optional redesign **after** the stationary arm passes tests. The earlier +$8/$66.50 and printed wheel/chassis claims were not backed by files, pricing, torque, traction or tip calculations. A 2WD platform typically needs two continuous-rotation motors **or** geared DC motors + a rated H-bridge, wheels/traction, robust caster, separate motor power/current protection, *actual* wheel odometry if accurate return-to-hand navigation is needed, physical bumper, obstacle sensing and a hard E-stop for all drive power. Two uncalibrated reflectance sensors alone cannot guarantee tabletop cliff protection (lighting, dark/glossy surfaces, sensor failure, latency and braking distance). Never autonomously drive a top-heavy arm on a desk edge. Begin on the **floor** with the arm stowed in a contained area, a supervisor and a physical stop.

`src/phase6_perception/vision_mobile_navigator.py` demonstrates an **offline-only** wheel-command policy: sensor staleness, any detected edge, obstacle, released e-stop or extended arm produces STOP. It does not interface with PCA9685, acquire camera frames, localize on a map, calibrate wheel direction or guarantee a stop in time. Run `python src/phase6_perception/vision_mobile_navigator.py`. Next experiments: measure mass/center of gravity with arm extended, find tipping/braking margin, build a plywood mock-up on the floor, test redundant edge cutoff with one sensor deliberately disconnected. Only then CAD a chassis and source parts with measured prices.

See `docs/verification_status.md` for the claims that have *not* been demonstrated.
