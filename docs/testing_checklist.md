# B.U.D.-E test checklist

Print this, tick as you go. Every command and every pass criterion, nothing else.
The full explanations — what each test proves, how to read a failure — are in
**`docs/testing_guide.md`**. Commands are PowerShell; on macOS/Linux use `python3` and `/`.

**Standing rules, all the way down:** the physical e-stop is the only thing guaranteed to stop
this robot · never leave a stalled servo powered · one change at a time.

---

## Part 1 — laptop only (~30 min, no hardware)

- [ ] **1. Python present** — `python --version` → 3.10 or newer
- [ ] **2. Tools** — `python -m pip install pytest pyserial` · `g++ --version` (optional; without
      it the two firmware tests skip instead of failing)
- [ ] **3. Whole project, one command** — `python src\phase8_safety_testing\run_phase_walkthrough.py`
      → last line **`18 of 18 commands exited 0`**
- [ ] **4. Unit tests** — `python -m pytest -v` → **`125 passed`**
- [ ] **5. Acceptance suite** — `python src\phase8_safety_testing\run_full_acceptance_suite.py`
      → **`50 / 50 checks passed`**, **`VERDICT: PASS`**
- [ ] **6. Rebuild + re-audit the parts** —
      `python src\phase2_mechanical\generate_stl_parts.py` ·
      `python src\phase2_mechanical\verify_print_parts.py` → `PASS: every part` ·
      `python src\phase2_mechanical\verify_assembly_fit.py` → **`TOTAL 206 / 206 checks PASS`**
- [ ] **7. Firmware, compiled and executed** —
      `g++ -std=c++11 -Wall -Wextra -Wpedantic -o reflex.exe firmware\phase3_full_6dof_esp32\test_reflex_logic.cpp` then `.\reflex.exe` → `PASS: 73 / 73`
      `g++ -std=c++11 -Wall -Wextra -I tests\firmware_host -o fw.exe tests\firmware_host\main.cpp` then `.\fw.exe` → `PASS: 68 / 68`
- [ ] **8. Pinout agreement** — `python src\phase3_electronics\power_and_wiring_verifier.py`
      → `VERDICT: PASS — 18 / 18`
- [ ] **9. URDF vs solver** — `python src\phase4_simulation\verify_urdf_model.py`
      → `VERDICT: PASS — 10 / 10`, `worst disagreement 0.00 mm`
- [ ] **10. Phases 5 + 6** — `python src\phase5_control\kinematics.py` ·
      `python src\phase5_control\trajectory_and_collision.py` ·
      `python src\phase5_control\ros2_moveit_bridge.py --dry-run` ·
      `python src\phase6_perception\camera_and_hand_eye_calib.py` ·
      `python src\phase6_perception\vision_mobile_navigator.py`
- [ ] **11. Viewer** — `python src\phase4_simulation\web_sim_server.py`, open
      **http://127.0.0.1:8080/** — drag the sliders, click a target, try the voice box
- [ ] **12. Read** — `docs\phase_walkthrough_transcript.txt` and
      `docs\phase8_acceptance_scorecard.txt`

---

## Part 2 — hardware, in order

> **Do not skip a gate because the one before it passed.** Gates A–C protect your parts.
> Gates D–G protect your floor. Gate G is the one that matters.

### Gate A — power OFF, multimeter
- [ ] Fuse fitted, DC-rated, no bigger than the smallest wire or board in the chain
- [ ] E-stop continuity **closed** released, **open** pressed (NC = fails safe)
- [ ] ~10 kΩ from GPIO 4 to 3V3 (the OE pull-up)
- [ ] **Open circuit** from 6 V rail to PCA9685 VCC and to 3V3
- [ ] Polarity traced: supply + → fuse → e-stop → INA219 VIN+ → VIN− → servo V+ (never reversed)
- [ ] All grounds common (ESP32 ↔ PCA9685 ↔ INA219 ↔ supply −)
- [ ] 470–1000 µF, ≥10 V, correct polarity across servo V+/GND after the switch

### Gate B — USB only, servo rail unpowered
- [ ] Sketch uploads: `firmware\phase3_full_6dof_esp32\phase3_full_6dof_esp32.ino`, board
      **ESP32 Dev Module**, serial monitor **115200**
- [ ] Boot line shows `"pca9685":true`, `"ina219":true`, `"outputs":"off"`
- [ ] `PING` → both addresses · `STATUS` → the live line · `HELP` → the command list
- [ ] `bus_v` stays near 0 with the 6 V rail off (if it reads 6 V: **stop**, something back-feeds)

### Gate C — one servo, unloaded, supply current limit 1 A
- [ ] `OE 0`, then `M 0:1600` / `M 0:1500` — smooth, stops when told, no buzzing, no heat
- [ ] No `BROWNOUT` event, no ESP32 reset when it moves
- [ ] `OE 1` is your panic button — it tri-states all eight channels

### Gate D — cliff sensor polarity (before any driving)
- [ ] Flat on the desk → `STATUS` `"cliff":[0,0]`
- [ ] Sensor looking into air → `[1,1]` (one sensor only → `[1,0]` = that sensor is dead)
- [ ] If yours reads the other way: set `CLIFF_TRIPPED_IS_HIGH = false` in the sketch, reflash
- [ ] Know that an **unfitted** sensor reads as a cliff: forward is refused (`FORWARD_REFUSED`)

### Gate E — rangefinder GPIO 18 / 19, ECHO level-shifted
- [ ] `distance_mm` tracks a tape measure within ~10 mm
- [ ] `sonar_fresh` is `true` while measuring
- [ ] Book removed → within ~0.5 s readings go stale and forward is refused (**fail closed**)

### Gate F — wheels OFF the ground
- [ ] `W 1320 1680` → both wheels turn so the rover would move **away from you**
- [ ] `W 1680 1320` → the other way · `W 1500 1500` → actually stops
- [ ] Send `W 1320 1680`, then type nothing → ~0.5 s later the wheels stop and `WHEEL_TIMEOUT`
- [ ] `W 1320 1680`, slide a card under one sensor → wheels **reverse** + `CLIFF_REFLEX` with
      `latency_us` < 5000
- [ ] Wrong direction? Flip both `wheel_*_forward_above_neutral` flags, reflash. **Do not go to
      the floor until this reads correctly**

### Gate G — the floor test
- [ ] Contained area, arm stowed, 2–3 cm of runway to the edge, foot on the supply switch
- [ ] `W 1320 1680` toward the edge → reverses ~350 ms, stops, `CLIFF_REFLEX` logged
- [ ] Repeat 3–5× including an angled approach so only **one** sensor goes over
- [ ] `reflex_max_us` in `STATUS` stays under 5000

### Gate H — the arm, joint by joint
- [ ] Each of CH0…5 moved alone (`M 0:1400` … `M 5:1500` …) — no buzzing, no heat, right direction
- [ ] Blended move works: `G 1500 0:1200,1:1300`
- [ ] Light load (≤100 g): no sag, no stall
- [ ] Home frame: `G 1500 0:1500,1:1250,2:1944,3:1221,4:1500,5:1393`
- [ ] Watched `current_ma` while holding a load
- [ ] After an e-stop: the firmware latches and **will not re-enable** until the ESP32 reboots

### Gate I — camera
- [ ] 9×6 chessboard printed, square measured with calipers
- [ ] 15–20 photos from varied angles, then
      `python src\phase6_perception\camera_and_hand_eye_calib.py calibrate --images "calibration_photos\*.png" --square-mm <measured> --cols 9 --rows 6 --output camera_intrinsics.json`
- [ ] Reprojection error under ~1 px; `camera_intrinsics.json` written
- [ ] A known object's pixel matches `/api/plan` in the viewer (else adjust
      `CAMERA_MOUNT_OFFSET_MM`, currently `(0, 30, −43)`)

### Gate J — end to end
- [ ] `python src\phase3_electronics\esp32_arm_driver.py --port COM3 --status`
- [ ] `... --port COM3 --home --yes` · `... --port COM3 --open 100 --yes`
- [ ] `... --port COM3 --drive 0.35 0.35 --yes` then `... --stop --yes` (wheels off the ground first)
- [ ] `python src\phase7_voice_personality\voice_assistant.py --demo`
- [ ] Remember `--stop` is a *request*: the real net is the ESP32's 500 ms timeout, and the last
      one is the physical e-stop

---

## What you have proven when the ticks are done

| Tick | Proof |
|---|---|
| Steps 3–10 | the code runs and the parts, model, config, wiring doc and firmware agree with each other |
| Step 6 | the printed parts are geometrically consistent and bed-seated |
| Step 7 | the firmware's safety logic compiles and executes correctly |
| Gate B | the ESP32 talks and both I²C chips answer |
| Gate C | a servo moves under control without browning out the rail |
| Gate D | the cliff sensors read the floor the way the firmware expects |
| Gate E | the rangefinder measures real distance and fails closed |
| Gate F | the wheels move the right way, stop on command, time out, and reverse on a cliff |
| Gate G | the reflex saves the rover at a real edge |
| Gate H | the arm moves all six joints without stalling or overheating |
| Gate I | the camera measures the world |

## What no tick proves

The servo torques are **datasheets**, not your servos. The sketch has **never been flashed by
me**. **< 5 ms is a requirement**, not a promise — Gate F/G is where you measure it. The arm has
never held anything. Nothing has been dropped. The budget is an estimate that excludes shipping,
taxes, fuse holder, capacitor, wire and revised hardware.
