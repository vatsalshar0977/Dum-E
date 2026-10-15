# Phase 8 — Safety Supervision, FMEA, Telemetry & Bring-Up Checklist

This is the last phase and the one that decides whether the other seven are allowed to move.
It has three parts: a supervisor that can refuse a command, a telemetry recorder that writes
down everything it saw, and a checklist that ends with a real servo turning for the first time.

```bash
python3 src/phase8_safety_testing/safety_supervisor_and_logger.py --echo   # six faults
python3 src/phase8_safety_testing/run_full_acceptance_suite.py            # all 8 phases
python3 src/phase8_safety_testing/run_full_acceptance_suite.py --json      # for CI or a log
```

---

## 1. The supervisor

`SafetySupervisor.tick(sensors) -> Command` runs at 100 Hz and evaluates faults in a **fixed
order**. The order is the design:

| # | Fault | Trigger | Response | Recovery |
|---|---|---|---|---|
| 1 | Hardware e-stop | INA219 bus ≤ 0.05 V (the LA38-11ZS has cut the servo rail) | **Latch `ESTOP_HOLD`**, wheels 1500 µs, servo rail off | `release_estop(measured_joints)` then `resume()` — never automatic |
| 2 | Brownout | bus < 4.90 V | Drop torque (`enable_servos = False`), wheels stop, warn that the arm may sag | Automatic only after ≥ 5.20 V held for 1 s, and then `resume()` |
| 3 | Cliff | TCRT5000 GPIO 25 or 26 HIGH | Wheels to 1500 µs **in the same tick**, then `startled_cliff_recoil` (40 mm reverse) | When both sensors see floor again |
| 4 | Obstacle | HC-SR04 < 250 mm | Decelerate proportionally | < 80 mm → `OBSTACLE_HOLD`, stop and stay stopped |
| 5 | Over-current | > 2600 mA held > 150 ms | Stop the wheels, return home, torque still on | When current falls back |
| 6 | Missed grasp | Closed jaw, < 180 mA held ≥ 350 ms | `sad_failed_grasp_droop`, then **one** retry | Second failure → give up and say so |
| — | Stale sensors | frame older than 250 ms | Wheels stop (fail closed) | Fresh data |

### The two details that are easy to get wrong

**The e-stop must not jump on release.** While the e-stop is latched the servos are unpowered
and the arm may have sagged to a position the software never asked for. Commanding the stored
target on release would snap the arm by the full sag distance. So `release_estop()` overwrites
the software target with the *measured* joint angles first, then clears the latch, then waits
for an explicit `resume()`. The injector measures the residual: **0.000°**.

**"Stop in under 5 ms" is a firmware requirement.** The supervisor decides in the same tick it
sees the cliff — zero ticks of software delay — but Python, USB serial and a 100 Hz loop are
not a real-time path. `HARDWARE_CLIFF_DEADLINE_MS = 5.0` names the budget the ESP32's GPIO
25/26 interrupt handler has to meet: write 1500 µs to both wheel channels in the ISR, and let
the Python supervisor be the second line of defence. Any claim that this repository meets 5 ms
through serial is false.

### Telemetry

Every tick is appended as one JSON object to `logs/bude_telemetry.jsonl` (git-ignored), with
joint angles, TCP xyz from the *same* FK the rest of the repo uses, bus volts, milliamps,
wheel pulse widths, both cliff states, the supervisor state and the active gesture.

```bash
python3 src/phase8_safety_testing/safety_supervisor_and_logger.py --fault estop --echo
python3 -c "from pathlib import Path; from src.phase8_safety_testing import \
TelemetryRecorder as T; print(T.jsonl_to_csv(Path('logs/bude_telemetry.jsonl')))"
```

JSONL is written line by line so a crash costs one tick; the CSV is generated from memory at
close, or from a JSONL file after the fact with `jsonl_to_csv`.

## 2. FMEA

Severity × Likelihood × Detection, each 1–5 (5 = worst). **RPN = S × L × D.** The mitigation
column says what this repository actually does about it, and whether that has been tested.

| # | Failure mode | Effect | S | L | D | RPN | Mitigation in this repo | Tested? |
|---|---|---|---|---|---|---|---|---|
| 1 | Servo stalls (arm pinned, object too heavy) | Driver or supply overheats, brownout, possible fire risk | 5 | 3 | 2 | 30 | Supervisor backs off home above 2600 mA for 150 ms; 5 A fused supply; `ina219_arm_trip_current_a 3.20` | No — synthetic only |
| 2 | Gripper closes on air | "Handover" of nothing; user may grab at a moving arm | 3 | 4 | 1 | 12 | < 180 mA on a closed jaw → droop + **one** retry, then report failure | Detector yes, hardware no |
| 3 | Cliff / table edge (rover) | Fall, broken arm, camera destroyed | 5 | 3 | 2 | 30 | TCRT5000 × 2 on GPIO 25/26, ISR stop, 40 mm reverse, priority-0 reflex | Policy yes, sensors no |
| 4 | Ultrasonic misses a thin obstacle | Collision with furniture or a person's ankle | 4 | 4 | 4 | 64 | < 250 mm decelerate, < 80 mm stop; **the HC-SR04 cone misses thin legs** — a physical bumper is still required | No |
| 5 | Brownout on servo inrush | Servo jitter, ESP32 resets, arm moves uncontrolled | 4 | 3 | 3 | 36 | < 4.90 V cuts torque and warns; 6 V 5 A supply; keep servo power off USB | No |
| 6 | E-stop released with the arm sagged | Arm snaps to a stale target | 4 | 3 | 3 | 36 | `release_estop()` syncs targets to measured feedback first | Yes (simulated) |
| 7 | J1 yaw drags the arm through the desk | Broken print, bent horn | 3 | 3 | 2 | 18 | Phase 5 keep-outs (TCP 10 mm, palm/wrist 15 mm, elbow 18 mm) reject the path | Path yes, physical no |
| 8 | Pinch between claw fingers | Flesh injury | 3 | 3 | 2 | 18 | 450 mA jaw stall stop; fingers geared 1:1 and unpowered when the rail is cut | No |
| 9 | Rover run on a desk edge | Fall from height | 5 | 4 | 1 | 20 | **Never run the rover on a desk.** Floor, contained area, arm stowed, supervisor present | Process control |
| 10 | 3D-printed part delaminates under load | Arm drops mid-move | 3 | 3 | 3 | 27 | 3–5 perimeters on load-bearing parts; torque margins in Phase 1; there is no fatigue model | No |
| 11 | Firmware/serial link dies mid-move | Servos hold their last pulse forever; a rover keeps driving | 4 | 2 | 2 | 16 | **Implemented in `phase3_full_6dof_esp32.ino`.** Two guards: a 5 s ESP32 **task watchdog** (`esp_task_wdt_init`, `trigger_panic = true`) that reboots the chip into outputs-off if the loop wedges, and a **500 ms wheel-command timeout** in `WheelSafety` that returns the wheels to 1500 µs on its own when the host goes quiet | Logic: yes (65 host checks). Hardware: **no** |
| 12 | I²C bus lockup (PCA9685/INA219) | No current sensing, no PWM updates, and a driver that holds its last pulse width | 4 | 3 | 2 | 24 | **Implemented in `phase3_full_6dof_esp32.ino`.** `i2c_bus_recovery()` clocks SCL nine times and issues a STOP (open drain, both pins), runs at boot **before** `Wire.begin()` and again on every NACK; the PCA9685 and INA219 are driven from raw `Wire` so a NACK is visible at all; after 3 consecutive failures the OE pin is cut | Logic: yes (host). Hardware: **no** |
| 13 | Voice "stop" not heard / misheard | The robot does not stop | 4 | 3 | 2 | 24 | Voice is explicitly **not** a safety device; physical e-stop is | N/A by design |
| 14 | Whisper model download mid-session | Robot appears to hang | 1 | 3 | 1 | 3 | Download once at the bench; the parser and gestures need no network | Yes |
| 15 | Wheel direction mis-calibrated (mirrored servos wired the other way) | The "reflex" drives the rover **toward** the edge instead of away from it | 5 | 3 | 2 | 30 | The two drive servos face each other, so the same pulse width turns them opposite ways. `WheelSafetyConfig.left/right_forward_is_above_neutral` states the assumption explicitly, and the bring-up checklist makes you prove it on the floor (`W 1320 1680` must move the rover **away** from you) before any cliff test | **No — you must do this one** |
| 16 | A TCRT5000 fails HIGH (dust, black floor, low battery) | The rover refuses to move forward and looks broken | 2 | 3 | 1 | 6 | Fail-safe direction: refusing to move is the safe outcome. `STATUS` reports both cliff states and every refusal is logged as `FORWARD_REFUSED` with a reason | Partly (host) |

Highest RPN: **#4 (64)** and **#5/#6 (36)**. #4 is not solvable in software — an ultrasonic
cone does not see a chair leg — so the honest answer is a mechanical bumper plus a low speed
limit. #1 and #3 are the ones that can destroy hardware; the supervisor helps, but only a fuse
and a physical e-stop *stop* it.

## 3. Pre-power bring-up checklist

Do these in order. Do not skip ahead because a step "obviously works". Stop at the first
surprise; a surprise you keep going past is how hardware dies.

**On the bench, nothing powered**

- [ ] Print `00_tolerance_test_coupon.stl` first. Test-fit a real MG996R, a real MG90S, a real
      M3 hex nut and a real 6 mm pin. If the pockets are tight, regenerate every STL with
      `--clearance 0.32` and reprint the coupon.
- [ ] Confirm the printed part's declared mass against a scale. `verify_assembly_fit.py` says
      the parts are ~588 g of solid PLA; at 3 walls and 20% infill a real set should be far
      lighter. If a single link is much heavier than the table, the Phase 1 torque margin is
      gone: re-check `torque_and_power_calc.py --payload-g 100`.
- [ ] Inspect every printed bore with a 6 mm pin and every hex trap with a real nut, by hand.
- [ ] Wire the servo rail **unpowered**: DC brick → fuse → latching NC e-stop → INA219 VIN+ →
      PCA9685 V+; common ground to the ESP32; PCA9685 VCC from 3V3 only.
- [ ] Fit the **10 kΩ pull-up on PCA9685 OE**. With the ESP32 disconnected, OE must be HIGH.
      This is the one resistor that makes "the microcontroller is dead" a safe state.
- [ ] Check the I²C addresses with a scanner: PCA9685 at **0x40**, INA219 at **0x41**.

**First power, motors disconnected**

- [ ] 6 V brick on, nothing on the servo headers. Measure the rail: expect 6.0–6.3 V.
- [ ] Press the e-stop: the INA219 bus voltage must fall to ~0 V. Release it.
- [ ] Flash `firmware/phase3_bench_diagnostic`. Confirm the I²C scan and the INA219 reading.
- [ ] Record the idle current with no servos attached.

**One servo, no load**

- [ ] J1 only, on PCA9685 CH0, arm removed from the servo. 1500 µs must be still.
- [ ] Flash `firmware/phase1_single_joint_esp32`. Sweep ±20° slowly. Listen, do not watch.
- [ ] Current while moving: a few hundred mA. If it spikes near 2600 mA, stop and find out why.
- [ ] With the servo held, verify the e-stop kills PWM *and* the rail.

**Assembled, empty, on the desk**

- [ ] Fit the arm with no tool and no payload. Rubber-band counterbalance on J2 as per Phase 2.
- [ ] Run the Phase 5 home pose at **half** the gesture speed: `speed_scale = 0.5`.
- [ ] Run every gesture once: `python3 src/phase7_voice_personality/voice_assistant.py --demo`.
      Watch for the claw and the camera hitting the base, and for J3 hitting the J2 straddle.
- [ ] Run the fault injections with the arm attached but stationary:
      `python3 src/phase8_safety_testing/safety_supervisor_and_logger.py --echo`.
- [ ] Stall J2 **briefly and deliberately** against your hand and confirm the supervisor backs
      off home. This is the only way to validate the 2600 mA threshold for your supply.
- [ ] Only now: full speed, then a real tool, then the camera, then the rover on the floor.

**Every session after that**

- [ ] E-stop reachable without looking. Wheels off the ground or the rover on the floor.
- [ ] Nothing breakable within the arm's 225 mm radial reach, and no hands inside it while the
      plan is running.
- [ ] `logs/bude_telemetry.jsonl` growing means at least one of the two processes is alive.

## 4. The master scorecard

`run_full_acceptance_suite.py` executes all eight phases and prints one scorecard. Current
state of this repository: **43 / 43 checks PASS in ~26 s**, with the honest caveat printed
underneath it and repeated here:

> Nothing in this repository has been printed, powered, calibrated or dropped. Every dimension
> is a model, every servo figure is a datasheet, and every latency is simulated. `PASS` means
> the model has no objection — it is not permission to run a real arm near a person.
