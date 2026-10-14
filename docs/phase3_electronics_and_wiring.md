# Phase 3 — electronics (staged, not yet hardware-validated)

**Read first:** `docs/verification_status.md`. Do not assemble or energize the full arm using the existing STL set; servo fit, horn interfaces, clamp strength and clearances are unverified. Start with **one unloaded, restrained servo on the bench**. The example firmware deliberately limits motion to channel 0, ±15° and 700–2300 µs until individual servo calibration is recorded. It is not a safety-rated controller.

## Controller and bill-of-material caveats

Use a laptop + ESP32-WROOM-32 + PCA9685 (50 Hz); this avoids buying an SBC. A nominal 6 V, current-limited DC supply is preferable to a LiPo. Check the **actual** servo voltage rating and start with a current-limited bench supply. The previous $58.50 figure is an unverified *estimate*, excludes shipping/taxes, fuse holder, capacitor, wiring, motor stall measurements and revised mounting hardware. Prices and the 100 g payload are **not guaranteed**. Do not buy a kit on the basis of the previous mass/torque spreadsheet alone.

## One-servo wiring (power off while making changes)

| From | To | Note |
| --- | --- | --- |
| DC brick +6 V | DC-rated fuse, then latching **NC** mushroom E-stop, then INA219 VIN+ | Fuse at supply output, sized **no greater than the smallest wire/switch/board continuous rating**, according to measured load. Do **not** assume a 7.5 A fuse protects a 5 A supply or a small INA219 board. |
| INA219 VIN− | PCA9685 servo V+ | INA219 0.1 Ω shunt is usually unsuitable for all six stalled servos; never assume 5 A through a default breakout is safe. |
| INA219 shunt | **0.01 Ω, 5 A class** recommended | The sketch defaults to `INA219_SHUNT_OHMS = 0.1f` / `INA219_CURRENT_LSB_MA = 0.1f`, matching the stock board. A 0.01 Ω board needs **both** lines changed to `0.01f` / `1.0f`; the boot log prints `INA219_CAL` with the pair it assumed, so a mismatch is visible instead of silent. |
| DC brick − | PCA9685 servo GND, INA219 GND, ESP32 GND | Common reference. Keep motor return separate from thin signal wires until ground junction. |
| USB laptop | ESP32 USB | USB powers *logic only*. No servo power from USB. |
| ESP32 3V3 / GND | PCA9685 VCC / GND, INA219 VCC / GND | Never connect 6 V to PCA9685 VCC or ESP32 3V3. Check I²C pull-ups are to 3.3 V. |
| ESP32 GPIO21 / GPIO22 | PCA9685 & INA219 SDA / SCL | Move INA219 to verified address **0x41** (A0 wiring varies by breakout); PCA9685 normally 0x40. Run I²C scanner first. |
| ESP32 GPIO4 | PCA9685 OE | **10 kΩ pull-up to 3.3 V at OE** so reset/disconnected GPIO disables PWM; OE low enables. Verify your board exposes OE. |
| PCA9685 CH0 signal/V+/GND | MG996R signal/red/brown | Verify the actual connector pin order and rated 6 V supply. **Only CH0** for first bring-up. |
| PCA9685 CH0–CH5 | J1 yaw, J2 shoulder, J3 elbow, J4 wrist pitch, J5 wrist roll, J6 jaw | The full-firmware channel map. Servo V+ comes from the switched rail, never from the ESP32. |
| PCA9685 CH6 / CH7 | LEFT / RIGHT drive servo (continuous rotation) | 1500 µs is stop. The two servos face each other, so the same pulse width turns them opposite ways — see the calibration warning below. |
| ESP32 GPIO25 | LEFT TCRT5000 digital out | HIGH = no floor. Feeds an interrupt **and** the loop. Wire the module's VCC from the **3V3** rail unless your board is a 5 V-tolerant type. |
| ESP32 GPIO26 | RIGHT TCRT5000 digital out | Same, other side. Two sensors is the minimum: one cannot see a corner. |
| ESP32 GPIO18 | HC-SR04 TRIG | 10 µs pulse, issued from a non-blocking state machine. |
| ESP32 GPIO19 | HC-SR04 ECHO | **Level-shift or divide this to 3V3.** Many HC-SR04 boards drive ECHO at 5 V, and 5 V on an ESP32 GPIO is out of spec. A 1 kΩ / 2 kΩ divider is enough. |

Put a suitably rated bulk capacitor (e.g. 470–1000 µF, ≥10 V, correct polarity) across servo V+/GND *after* the switch; its stored energy means power cut is not instantaneous at the servo. Do not open mains wiring or use a bare AC mains PSU for a novice bench build. The E-stop removes drive power but can cause the arm to **fall**: physically support suspended loads. This one-contact example **cannot guarantee that a very brief E-stop press is latched** by the firmware; never twist-release while the servo remains armed. Disarm and verify motor power is off before resetting. A real assembled arm needs a positively monitored, manual-reset safety relay and independently validated stopping behaviour. Check E-stop contact **DC** rating at the measured worst-case current; use a rated contactor/relay if the button is undersized. Never rely on OE or USB software as an E-stop.

**Important limitations:** INA219 reports *aggregate* rail current, cannot distinguish claw contact from shoulder load, cannot enforce individual servo torque and may saturate above its chosen calibration. I²C reads, filtering and loop scheduling have variable latency; previous claims of <5 ms human collision protection, 450 mA gentle gripping and reliable 3.2 A trips were unjustified. Hobby PWM servos expose no trustworthy load/position telemetry. Avoid servo potentiometer solder modifications until output voltage, electrical isolation and calibration are measured; feeding a 5–6 V wiper to a 3.3 V ESP32 can destroy it.

## Firmware bring-up

Install Arduino ESP32 board support, **Adafruit PWM Servo Driver** and **Adafruit INA219** libraries. Flash `firmware/phase3_bench_diagnostic/phase3_bench_diagnostic.ino`. It starts disabled; a current/voltage sample and `ARM` command are required, and loss of serial heartbeat or a telemetry fault disables PWM and latches a fault. This is a *diagnostic*, not certified collision sensing. Serial commands at 115200 baud, newline terminated: `STATUS`, `ARM`, `MOVE 0 5 1000`, `DISARM`, `CLEAR`. The ±15° movement is relative to **assumed 1500 µs center**, not a calibrated absolute robot angle. Unplug the horn for the first center test; attach only after checking for interference. Test E-stop with the servo unloaded, then measure real current and verify it stays off until deliberately re-armed. Hardware tests and firmware compilation have **not** been performed here.

Do not energize channels 1–5 until the mechanics are redesigned and individually tested. A rover adds its own motor driver/current budget, bumper, emergency stop and redundant cliff detection; a single camera or TCRT5000 is not an adequate desk-edge safety system.


---

## Full firmware: `firmware/phase3_full_6dof_esp32/`

`phase3_bench_diagnostic.ino` is the bring-up sketch: it proves the I2C bus, both addresses and
the PCA9685 one channel at a time. `phase1_single_joint_esp32.ino` is a legacy single-joint
demo with a warning at the top. **`phase3_full_6dof_esp32.ino` is the one that drives the
finished robot**: all six arm channels, both rover wheels, the cliff reflex, the front
distance sensor, the watchdog and the I2C recovery.

### What runs where

| Concern | Where it lives | Why there |
|---|---|---|
| Smooth arm motion | ESP32 cosine blend, 100 Hz, `G <ms>` | Serial jitter never reaches a servo |
| **Cliff reflex** | **ESP32**, interrupt + loop | A USB round trip is 5–20 ms and can be absent entirely. The deadline is 5 ms, so it cannot live in Python |
| Host-liveness timeout | ESP32, 500 ms | A host that hangs is indistinguishable from a host that stopped talking |
| Task watchdog | ESP32, 5 s → panic + reboot | The backstop for the *firmware* wedging |
| Over-current, brownout, missed grasp, obstacle policy | Python supervisor (Phase 8) | 10–50 ms decisions; none of them is a reflex |
| Wheel dead-band calibration | **You, with a tape measure** | No model can know it |

### Serial protocol (115200, newline terminated)

```
PING                       identity, addresses, wheel channels, recoil duration
W <left_us> <right_us>     BOTH WHEELS.  1500 1500 = stop
                                         1320 1680 = forward   (mirrored pair)
                                         1680 1320 = reverse
M <ch>:<us>[,<ch>:<us>...]          immediate set, up to 8 channels
G <ms> <ch>:<us>[,<ch>:<us>...]     cosine-blended move over <ms> (arm channels only)
OE <0|1>                   0 enables the outputs, 1 tri-states them (the safe state)
STATUS                     JSON: bus volts, current, cliffs, distance, reflex statistics
HELP
```

Wheel channels inside `M`/`G` are applied **immediately** and are never blended: a blended
wheel command would fight the cliff reflex for ownership of channels 6 and 7, and two writers
on one safety path is how you lose a safety path.

### ⚠ Check the cliff sensor polarity before anything else (Gate D)

TCRT5000 breakouts ship with two different output conventions and the difference is not written
on the board. The firmware declares which one it expects in one place:

```cpp
static const bool CLIFF_TRIPPED_IS_HIGH = true;
```

With that setting: rover flat on the desk → `STATUS "cliff":[0,0]`; sensor looking into air →
`[1,1]`. If your modules read the other way round, change that single line to `false` and reflash
— the reads, the forward gate and the interrupt edge all follow it, and the boot line reports
which convention is compiled in as `"cliff_tripped_is_high"`. Both conventions are covered by
`tests/test_firmware.py`.

Note that the pins are `INPUT_PULLUP`, so an **unplugged** sensor reads as a cliff: the rover will
refuse to drive forward rather than drive blind. That is the safe direction, and it is also the
thing that will confuse you on the bench — see `docs/testing_guide.md`, Gate D.

### ⚠ Three build options, all of which fail closed

These live at the top of `phase3_full_6dof_esp32.ino`, and the boot JSON reports every one of
them, so the serial log always states what the firmware assumed:

| Constant | Ships as | What it protects | The only reason to change it |
| --- | --- | --- | --- |
| `CLIFF_TRIPPED_IS_HIGH` | `true` | A broken sensor wire reads HIGH (INPUT_PULLUP) = "cliff", so it cannot drive forward. | Your TCRT5000 modules invert the output - Gate D tells you which way round yours are. |
| `SONAR_REQUIRED_FOR_FORWARD` | `true` | No **fresh** distance reading means no forward motion, so a dead or unplugged HC-SR04 stops the rover rather than letting it drive blind into what it was meant to see. | A bench build with no sonar fitted at all (stationary arm, Gates C/D). |
| `REQUIRE_INA219_TO_ENABLE` | `true` | Refuses `OE 0` when the INA219 never answered, because the brownout cut and the e-stop-open latch both read through it - without it, a sagging rail is invisible. | You deliberately built with no current sensing; you then lose both protections. |

The first two are exercised by `tests/firmware_host/main.cpp` at runtime (sections 13-14 prove the
sonar one by starving the sensor and checking that forward is refused).

### ⚠ Calibrate the wheel directions before any cliff test

The brief's recoil pair is `1680 / 1320`, which only moves the rover *backward* if the two drive
servos are mounted facing each other. The firmware states the assumption explicitly
(`wheel_left_forward_above_neutral: false`, `wheel_right_forward_above_neutral: true` in
`config/robot_config.yaml`) instead of hiding it in a hard-coded pair.

Prove it on the **floor**, in a contained area, arm stowed, hand on the e-stop:

```sh
W 1320 1680      # the rover must move AWAY from you
W 1680 1320      # ... and this must bring it back
W 1500 1500      # and this must actually stop it, not creep
```

If `W 1320 1680` brings the rover *toward* you, flip both flags and reflash. Until you have done
this, the cliff reflex is pointing in the wrong direction, which is FMEA row #15 and the highest
risk this firmware introduces.

### What has actually been checked

```sh
python3 -m unittest tests.test_firmware        # compiles and runs everything below
g++ -std=c++11 -Wall -Wextra -o /tmp/t firmware/phase3_full_6dof_esp32/test_reflex_logic.cpp && /tmp/t
g++ -std=c++11 -Wall -Wextra -I tests/firmware_host -o /tmp/fw tests/firmware_host/main.cpp && /tmp/fw
```

* `reflex_logic.h` — the reflex, the liveness timeout, the NACK policy, the line parser and the
  distance guard, with **no** Arduino dependency. 73 checks, all passing.
* `tests/firmware_host/main.cpp` — the **real `.ino`**, compiled against stub Arduino / ESP32 /
  Wire APIs and driven with a simulated clock, simulated GPIO and a PCA9685 whose register
  writes are decoded back into pulse widths. 73 checks, all passing: the boot order, a GPIO 25
  edge → both channels reversed inside one loop iteration, the 350 ms recoil, the forward
  refusal, the 500 ms timeout, NACK → recovery → OE cut, a 400-character serial flood, and
  the two fail-closed paths: a wall inside the stop distance, and a sonar that stops answering.

That is a real compiler and a real execution of the firmware's logic — and it is still **not an
ESP32**. The I2C peripheral, the interrupts, the watchdog and the servos are models. This
sketch has never been flashed; treat the first flash as bring-up with the arm unloaded and the
wheels off the ground.
