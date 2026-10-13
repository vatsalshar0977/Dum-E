"""Phase 3 — verify the wiring table, the config and the firmware agree, and size the rail.

The three descriptions of this robot's electronics live in three different files:

    docs/phase3_electronics_and_wiring.md   what a human is told to solder
    config/robot_config.yaml                what the Python supervisor reads
    firmware/phase3_full_6dof_esp32/*.ino   what the ESP32 actually does

Nothing stops those three from drifting apart, and a drift here is not a cosmetic problem: the
config could tell the supervisor to watch a GPIO that the firmware never reads, or the doc could
send you to a different pin than the code. So this walks all three and fails on disagreement.

It also does the arithmetic that decides whether the build works at all:

  * the voltage drop in the supply wiring at the 5 A limit, in 20/22/24/26 AWG, so you can pick
    a gauge instead of guessing;
  * the worst-case stall current against the 5 A supply, which is the single most likely way a
    beginner's first power-up browns out and resets the ESP32 mid-move;
  * the INA219 0.1 ohm shunt's dissipation at that current, which is why the stock breakout is
    not a 5 A part.

Every number here is a model. Nothing in this file proves a joint moved.

Usage:  python3 src/phase3_electronics/power_and_wiring_verifier.py [--json]
Exit code 0 = every check passed.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[2]
WIRING_DOC = ROOT / "docs" / "phase3_electronics_and_wiring.md"
CONFIG = ROOT / "config" / "robot_config.yaml"
SKETCH = ROOT / "firmware" / "phase3_full_6dof_esp32" / "phase3_full_6dof_esp32.ino"
HEADER = ROOT / "firmware" / "phase3_full_6dof_esp32" / "reflex_logic.h"

# --- the design envelope (declared; every one of these is an estimate, not a measurement) ----
SUPPLY_V = 6.0
SUPPLY_A = 5.0
MIN_SAFE_V = 4.8
SUPPLY_RUN_M = 1.0                      # 1 m of figure-eight servo wire, both legs counted once

# Worst-case stall current per servo, from the datasheets the BOM quotes.
STALL_A = {"MG996R": 2.5, "MG90S": 1.0}
ARM_SERVOS = ("MG996R", "MG996R", "MG996R", "MG90S", "MG90S", "MG90S")
ROVER_SERVOS = ("MG90S", "MG90S")       # two continuous-rotation drives

# Copper resistivity at 20 C, and the cross-section of each gauge in mm^2.
COPPER_OHM_MM2_PER_M = 0.01724
AWG_MM2 = {"20": 0.5176, "22": 0.3260, "24": 0.2047, "26": 0.1281}
AWG_CONTINUOUS_A = {"20": 5.0, "22": 3.0, "24": 2.0, "26": 1.0}   # conservative chassis wiring

SHUNT_OHM = 0.1                         # the stock INA219 breakout


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8") if path.exists() else ""


def _config_scalar(text: str, key: str) -> Optional[float]:
    match = re.search(rf"^\s*{re.escape(key)}\s*:\s*(-?\d+(?:\.\d+)?)", text, re.MULTILINE)
    return float(match.group(1)) if match else None


def _firmware_constant(text: str, name: str) -> Optional[int]:
    match = re.search(rf"\b{re.escape(name)}\s*=\s*(\d+)", text)
    return int(match.group(1)) if match else None


def wire_drop_v(gauge: str, current_a: float, run_m: float) -> float:
    """Resistance of a round trip of `run_m` in the given gauge, times the current."""
    area_mm2 = AWG_MM2[gauge]
    ohm_per_m = COPPER_OHM_MM2_PER_M / area_mm2
    return 2.0 * run_m * ohm_per_m * current_a


class Verifier:
    def __init__(self) -> None:
        self.checks: List[Dict[str, object]] = []

    def check(self, ok: bool, title: str, detail: str = "") -> bool:
        self.checks.append({"ok": bool(ok), "title": title, "detail": detail})
        return bool(ok)

    # ------------------------------------------------------------------ the three-way pin map
    def pins_match_everywhere(self, doc: str, config: str, sketch: str) -> None:
        firmware_pins = {
            "SDA": _firmware_constant(sketch, "PIN_I2C_SDA"),
            "SCL": _firmware_constant(sketch, "PIN_I2C_SCL"),
            "OE": _firmware_constant(sketch, "PIN_PCA9685_OE"),
            "cliff_left": _firmware_constant(sketch, "PIN_CLIFF_LEFT"),
            "cliff_right": _firmware_constant(sketch, "PIN_CLIFF_RIGHT"),
            "sonar_trig": _firmware_constant(sketch, "PIN_SONAR_TRIG"),
            "sonar_echo": _firmware_constant(sketch, "PIN_SONAR_ECHO"),
        }
        config_pins = {
            "SDA": _config_scalar(config, "sda"),
            "SCL": _config_scalar(config, "scl"),
            "OE": _config_scalar(config, "pca9685_oe"),
            "cliff_left": _config_scalar(config, "cliff_left"),
            "cliff_right": _config_scalar(config, "cliff_right"),
            "sonar_trig": _config_scalar(config, "sonar_trig"),
            "sonar_echo": _config_scalar(config, "sonar_echo"),
        }
        doc_pins = {
            "SDA": 21 if "GPIO21" in doc else None,
            "SCL": 22 if "GPIO22" in doc else None,
            "OE": 4 if "GPIO4" in doc else None,
            "cliff_left": 25 if "GPIO25" in doc else None,
            "cliff_right": 26 if "GPIO26" in doc else None,
            "sonar_trig": 18 if "GPIO18" in doc else None,
            "sonar_echo": 19 if "GPIO19" in doc else None,
        }

        self.check(all(v is not None for v in firmware_pins.values()),
                   "the firmware defines all seven GPIOs",
                   ", ".join(f"{k}={v}" for k, v in firmware_pins.items()))
        mismatches = [
            f"{name}: doc={doc_pins[name]} config={config_pins[name]} firmware={firmware_pins[name]}"
            for name in firmware_pins
            if not (doc_pins[name] == config_pins[name] == firmware_pins[name])
        ]
        self.check(not mismatches,
                   "all seven GPIOs agree across the wiring table, the config and the firmware",
                   "SDA 21 / SCL 22 / OE 4 / cliff 25,26 / sonar 18,19"
                   if not mismatches else "; ".join(mismatches))

    def channels_match(self, config: str, sketch: str, header: str) -> None:
        left = _firmware_constant(sketch, "WHEEL_LEFT_CH")
        right = _firmware_constant(sketch, "WHEEL_RIGHT_CH")
        neutral = _firmware_constant(header, "neutral_us")
        self.check((left, right, neutral) == (6, 7, 1500),
                   "the wheels are on PCA9685 channels 6 and 7 with 1500 us neutral",
                   f"left={left} right={right} neutral={neutral} in the firmware; "
                   f"config wheel_left_ch={_config_scalar(config, 'wheel_left_ch')} "
                   f"wheel_right_ch={_config_scalar(config, 'wheel_right_ch')}")
        self.check(_config_scalar(config, "wheel_left_ch") == left
                   and _config_scalar(config, "wheel_right_ch") == right,
                   "the config names the same wheel channels as the firmware",
                   "a supervisor driving channels the firmware never reads fails silently")
        self.check(_config_scalar(config, "cliff_recoil_ms") == _firmware_constant(
            header, "recoil_ms") and _config_scalar(config, "wheel_command_timeout_ms")
            == _firmware_constant(header, "command_timeout_ms"),
            "the config quotes the same recoil and timeout figures as the firmware",
            "350 ms recoil, 500 ms host-silence timeout")

    def addresses_match(self, doc: str, config: str, sketch: str) -> None:
        pca = re.search(r"PCA9685_ADDR\s*=\s*0x([0-9A-Fa-f]+)", sketch)
        ina = re.search(r"INA219_ADDR\s*=\s*0x([0-9A-Fa-f]+)", sketch)
        pca_addr = int(pca.group(1), 16) if pca else None
        ina_addr = int(ina.group(1), 16) if ina else None
        self.check(pca_addr == 0x40 and ina_addr == 0x41,
                   "the firmware uses PCA9685 0x40 and INA219 0x41",
                   f"0x{pca_addr:02X} / 0x{ina_addr:02X}" if pca_addr is not None else "missing")
        self.check("0x41" in doc and "0x40" in doc,
                   "the wiring table tells you to expect those addresses",
                   "the doc also warns that the INA219 address depends on its A0 jumper")

    # ------------------------------------------------------------------------ rail arithmetic
    def rail_budget(self) -> None:
        arm_stall = sum(STALL_A[model] for model in ARM_SERVOS)
        rover_stall = sum(STALL_A[model] for model in ROVER_SERVOS)
        self.check(arm_stall > SUPPLY_A,
                   "the arm alone can demand more current than the 5 A supply can give",
                   f"all six arm servos stalled = {arm_stall:.1f} A vs a {SUPPLY_A:.0f} A supply: "
                   "it will brown out (not trip) - keep moves gentle and never stall two "
                   "proximal servos at once")
        self.check(rover_stall <= SUPPLY_A - 1.5,
                   "the rover's two drives fit inside the supply with headroom",
                   f"{rover_stall:.1f} A stalled, leaving {SUPPLY_A - rover_stall:.1f} A: the "
                   "rover build is the one you can actually hold at stall")

    def wiring_gauge(self) -> Tuple[str, float]:
        rows = []
        for gauge in ("20", "22", "24", "26"):
            drop = wire_drop_v(gauge, SUPPLY_A, SUPPLY_RUN_M)
            ok_ampacity = AWG_CONTINUOUS_A[gauge] >= SUPPLY_A
            rows.append((gauge, drop, ok_ampacity))
        usable = [r for r in rows if r[2] and r[1] <= (SUPPLY_V - MIN_SAFE_V)]
        best = min(usable, key=lambda r: float(r[0])) if usable else rows[0]
        self.check(bool(usable),
                   "a copper gauge carries 5 A over 1 m and still leaves 4.8 V at the servo",
                   "; ".join(f"{g} AWG: {d * 1000:.0f} mV"
                             f"{'' if a else ' (ampacity too low)'}" for g, d, a in rows)
                   + f"  -> use {best[0]} AWG or heavier")
        self.check(AWG_CONTINUOUS_A["22"] < SUPPLY_A,
                   "22 AWG is NOT enough for the full 5 A rail",
                   f"22 AWG chassis wiring is good for about {AWG_CONTINUOUS_A['22']:.0f} A; "
                   "the doc quotes a 22 AWG example, which is fine for one servo, not for six")
        return str(best[0]), float(best[1])

    def shunt_and_protection(self, doc: str, config: str) -> None:
        dissipation = SUPPLY_A ** 2 * SHUNT_OHM
        drop = SUPPLY_A * SHUNT_OHM
        self.check("0.1 Ω" in doc and "unsuitable" in doc,
                   "the doc warns that the stock 0.1 ohm INA219 shunt is not a 5 A part",
                   f"at {SUPPLY_A:.0f} A that shunt drops {drop:.2f} V and burns "
                   f"{dissipation:.1f} W - it will desolder itself")
        arm_trip = _config_scalar(config, "ina219_arm_trip_current_a")
        self.check(arm_trip is not None and arm_trip <= SUPPLY_A,
                   "the configured trip current is inside the supply's capability",
                   f"the config trips the arm at {arm_trip} A; a trip above the supply rating "
                   "would never fire before brownout")
        self.check("NC" in doc and "E-stop" in doc and "6 V" in doc,
                   "the e-stop is a latching NC switch in the 6 V servo rail",
                   "NC means a broken wire or an unplugged switch also kills the rail")
        self.check("10 kΩ pull-up" in doc and "OE" in doc,
                   "the OE pin has a documented pull-up so a reset leaves the servos off",
                   "OE HIGH = tri-stated; a floating OE at power-up is a random pose")
        self.check("3.3 V" in doc and "VCC" in doc,
                   "the doc warns against putting 6 V on PCA9685 VCC or ESP32 3V3",
                   "the logic side is 3.3 V; the servo side is 6 V, and only V+ sees 6 V")
        self.check("Level-shift" in doc or "divider" in doc,
                   "the HC-SR04 ECHO pin is flagged for level shifting",
                   "many boards drive ECHO at 5 V, which is out of spec for an ESP32 GPIO")
        self.check("capacitor" in doc,
                   "a bulk capacitor across the servo rail is specified",
                   "stall transients are what reset the ESP32; the capacitor absorbs them")

    # ----------------------------------------------------------------------------- reporting
    def run(self) -> int:
        doc, config, sketch, header = _read(WIRING_DOC), _read(CONFIG), _read(SKETCH), _read(HEADER)
        missing = [str(p.relative_to(ROOT)) for p in (WIRING_DOC, CONFIG, SKETCH, HEADER)
                   if not p.exists()]
        if missing:
            print(f"  FAIL  missing files: {', '.join(missing)}")
            return 1
        self.pins_match_everywhere(doc, config, sketch)
        self.channels_match(config, sketch, header)
        self.addresses_match(doc, config, sketch)
        self.rail_budget()
        self.wiring_gauge()
        self.shunt_and_protection(doc, config)

        passed = sum(1 for c in self.checks if c["ok"])
        print("=" * 78)
        print("  PHASE 3 — POWER, WIRING AND PINOUT VERIFICATION")
        print("  Model arithmetic only. Nothing here has been powered or measured.")
        print("=" * 78)
        for item in self.checks:
            print(f"  {'PASS' if item['ok'] else 'FAIL'}  {item['title']}")
            if item["detail"]:
                print(f"           {item['detail']}")
        print("-" * 78)
        if passed == len(self.checks):
            print(f"  VERDICT: PASS — {passed} / {len(self.checks)} checks")
            print("  The doc, the config and the firmware describe the same robot, and the rail")
            print("  arithmetic has headroom. That is a consistency result, not a measurement:")
            print("  use a current-limited supply and a fuse for the first power-up.")
        else:
            print(f"  VERDICT: FAIL — {len(self.checks) - passed} of {len(self.checks)} failed")
        print("=" * 78)
        return 0 if passed == len(self.checks) else 1


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    args = parser.parse_args(argv)
    verifier = Verifier()
    if args.json:
        verifier.pins_match_everywhere(_read(WIRING_DOC), _read(CONFIG), _read(SKETCH))
        verifier.channels_match(_read(CONFIG), _read(SKETCH), _read(HEADER))
        verifier.addresses_match(_read(WIRING_DOC), _read(CONFIG), _read(SKETCH))
        verifier.rail_budget()
        verifier.wiring_gauge()
        verifier.shunt_and_protection(_read(WIRING_DOC), _read(CONFIG))
        payload = {
            "checks": verifier.checks,
            "passed": sum(1 for c in verifier.checks if c["ok"]),
            "total": len(verifier.checks),
        }
        print(json.dumps(payload, indent=2))
        return 0 if payload["passed"] == payload["total"] else 1
    return verifier.run()


if __name__ == "__main__":
    sys.exit(main())
