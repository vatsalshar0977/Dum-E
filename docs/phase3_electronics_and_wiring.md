# Phase 3 — electronics (staged, not yet hardware-validated)

**Read first:** `docs/verification_status.md`. Do not assemble or energize the full arm using the existing STL set; servo fit, horn interfaces, clamp strength and clearances are unverified. Start with **one unloaded, restrained servo on the bench**. The example firmware deliberately limits motion to channel 0, ±15° and 700–2300 µs until individual servo calibration is recorded. It is not a safety-rated controller.

## Controller and bill-of-material caveats

Use a laptop + ESP32-WROOM-32 + PCA9685 (50 Hz); this avoids buying an SBC. A nominal 6 V, current-limited DC supply is preferable to a LiPo. Check the **actual** servo voltage rating and start with a current-limited bench supply. The previous $58.50 figure is an unverified *estimate*, excludes shipping/taxes, fuse holder, capacitor, wiring, motor stall measurements and revised mounting hardware. Prices and the 100 g payload are **not guaranteed**. Do not buy a kit on the basis of the previous mass/torque spreadsheet alone.

## One-servo wiring (power off while making changes)

| From | To | Note |
| --- | --- | --- |
| DC brick +6 V | DC-rated fuse, then latching **NC** mushroom E-stop, then INA219 VIN+ | Fuse at supply output, sized **no greater than the smallest wire/switch/board continuous rating**, according to measured load. Do **not** assume a 7.5 A fuse protects a 5 A supply or a small INA219 board. |
| INA219 VIN− | PCA9685 servo V+ | INA219 0.1 Ω shunt is usually unsuitable for all six stalled servos; never assume 5 A through a default breakout is safe. |
| DC brick − | PCA9685 servo GND, INA219 GND, ESP32 GND | Common reference. Keep motor return separate from thin signal wires until ground junction. |
| USB laptop | ESP32 USB | USB powers *logic only*. No servo power from USB. |
| ESP32 3V3 / GND | PCA9685 VCC / GND, INA219 VCC / GND | Never connect 6 V to PCA9685 VCC or ESP32 3V3. Check I²C pull-ups are to 3.3 V. |
| ESP32 GPIO21 / GPIO22 | PCA9685 & INA219 SDA / SCL | Move INA219 to verified address **0x41** (A0 wiring varies by breakout); PCA9685 normally 0x40. Run I²C scanner first. |
| ESP32 GPIO4 | PCA9685 OE | **10 kΩ pull-up to 3.3 V at OE** so reset/disconnected GPIO disables PWM; OE low enables. Verify your board exposes OE. |
| PCA9685 CH0 signal/V+/GND | MG996R signal/red/brown | Verify the actual connector pin order and rated 6 V supply. **Only CH0** for first bring-up. |

Put a suitably rated bulk capacitor (e.g. 470–1000 µF, ≥10 V, correct polarity) across servo V+/GND *after* the switch; its stored energy means power cut is not instantaneous at the servo. Do not open mains wiring or use a bare AC mains PSU for a novice bench build. The E-stop removes drive power but can cause the arm to **fall**: physically support suspended loads. This one-contact example **cannot guarantee that a very brief E-stop press is latched** by the firmware; never twist-release while the servo remains armed. Disarm and verify motor power is off before resetting. A real assembled arm needs a positively monitored, manual-reset safety relay and independently validated stopping behaviour. Check E-stop contact **DC** rating at the measured worst-case current; use a rated contactor/relay if the button is undersized. Never rely on OE or USB software as an E-stop.

**Important limitations:** INA219 reports *aggregate* rail current, cannot distinguish claw contact from shoulder load, cannot enforce individual servo torque and may saturate above its chosen calibration. I²C reads, filtering and loop scheduling have variable latency; previous claims of <5 ms human collision protection, 450 mA gentle gripping and reliable 3.2 A trips were unjustified. Hobby PWM servos expose no trustworthy load/position telemetry. Avoid servo potentiometer solder modifications until output voltage, electrical isolation and calibration are measured; feeding a 5–6 V wiper to a 3.3 V ESP32 can destroy it.

## Firmware bring-up

Install Arduino ESP32 board support, **Adafruit PWM Servo Driver** and **Adafruit INA219** libraries. Flash `firmware/phase3_bench_diagnostic/phase3_bench_diagnostic.ino`. It starts disabled; a current/voltage sample and `ARM` command are required, and loss of serial heartbeat or a telemetry fault disables PWM and latches a fault. This is a *diagnostic*, not certified collision sensing. Serial commands at 115200 baud, newline terminated: `STATUS`, `ARM`, `MOVE 0 5 1000`, `DISARM`, `CLEAR`. The ±15° movement is relative to **assumed 1500 µs center**, not a calibrated absolute robot angle. Unplug the horn for the first center test; attach only after checking for interference. Test E-stop with the servo unloaded, then measure real current and verify it stays off until deliberately re-armed. Hardware tests and firmware compilation have **not** been performed here.

Do not energize channels 1–5 until the mechanics are redesigned and individually tested. A rover adds its own motor driver/current budget, bumper, emergency stop and redundant cliff detection; a single camera or TCRT5000 is not an adequate desk-edge safety system.
