#!/usr/bin/env python3
"""
Minimal First-Joint Bringup Script (Sub-$75 ESP32 + PCA9685 + INA219 Edition).

Follows the rule: "Start with a minimal working version (one joint moving), then build up."
Before printing or assembling the full arm, use this script to:
  1. Connect to the ESP32 controller over USB Serial (macOS/Windows/Linux).
  2. Read live 6V bus voltage (V) and servo current (mA) from the $2.50 INA219 sensor.
  3. Set the software over-current cutoff (450 mA for single joint / gripper test).
  4. Command a smooth cosine-interpolated +/- 15 degree "hello wiggle" on Joint 1 (PCA9685 Ch 0).

Can be run WITHOUT hardware right now using `--dry-run`!

Usage:
    python3 src/phase1_sizing/single_joint_ping.py --dry-run
    python3 src/phase1_sizing/single_joint_ping.py --port /dev/tty.usbserial-0001 --channel 0 --wiggle
"""

from __future__ import annotations

import argparse
import json
import sys
import time


def deg_to_pwm_us(angle_deg: float, min_deg: float = -90.0, max_deg: float = 90.0,
                  min_us: int = 500, max_us: int = 2500) -> int:
    """Map joint angle in degrees (-90..+90) to PCA9685 pulse width in microseconds (500..2500 us)."""
    clamped = max(min_deg, min(max_deg, angle_deg))
    ratio = (clamped - min_deg) / (max_deg - min_deg)
    return int(round(min_us + ratio * (max_us - min_us)))


class ESP32ServoBridge:
    """Minimal USB-Serial JSON/ASCII bridge to ESP32 + PCA9685 + INA219 controller."""

    def __init__(self, port: str, baudrate: int = 115200, dry_run: bool = False) -> None:
        self.port = port
        self.baudrate = baudrate
        self.dry_run = dry_run
        self._ser = None
        self._sim_angle_deg = 0.0

        if not dry_run:
            try:
                import serial  # pyserial
            except ImportError as exc:
                raise RuntimeError(
                    "pyserial is required for hardware mode. Install with: pip install pyserial"
                ) from exc
            self._ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.2)
            time.sleep(1.5)  # Wait for ESP32 auto-reset on serial open

    def close(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()

    def send_cmd(self, cmd_dict: dict) -> dict:
        line = json.dumps(cmd_dict, separators=(",", ":"))
        if self.dry_run:
            print(f"  [DRY-RUN USB TX] -> {line}")
            if cmd_dict.get("cmd") == "PING":
                return {"ok": True, "fw": "BUDE-ESP32-v1.0", "pca9685": "0x40", "ina219": "0x41"}
            if cmd_dict.get("cmd") == "MOVE":
                self._sim_angle_deg = float(cmd_dict.get("deg", 0.0))
                return {
                    "ok": True,
                    "ch": cmd_dict["ch"],
                    "deg": self._sim_angle_deg,
                    "pwm_us": cmd_dict["pwm_us"],
                    "bus_v": 5.98,
                    "current_ma": 215,
                }
            return {"ok": True, "bus_v": 6.02, "current_ma": 42}

        assert self._ser is not None
        self._ser.reset_input_buffer()
        self._ser.write((line + "\n").encode("utf-8"))
        self._ser.flush()
        raw = self._ser.readline().decode("utf-8", errors="replace").strip()
        return json.loads(raw) if raw else {"ok": False, "err": "timeout"}


def main() -> int:
    parser = argparse.ArgumentParser(description="Test a single servo on ESP32 + PCA9685 + INA219.")
    parser.add_argument("--port", default="/dev/tty.usbserial-0001", help="ESP32 USB serial port")
    parser.add_argument("--channel", type=int, default=0, help="PCA9685 channel (0..5)")
    parser.add_argument("--wiggle", action="store_true", help="Perform a smooth +/-15 deg greeting wiggle")
    parser.add_argument("--dry-run", action="store_true", help="Simulate USB serial TX/RX without hardware")
    args = parser.parse_args()

    print(f"=== B.U.D.-E Sub-$75 Single-Joint Bringup (PCA9685 Ch={args.channel}, dry_run={args.dry_run}) ===")
    bridge = ESP32ServoBridge(port=args.port, dry_run=args.dry_run)
    try:
        ping_res = bridge.send_cmd({"cmd": "PING"})
        if not ping_res.get("ok"):
            print(f"[ERROR] No response from ESP32 on {args.port}: {ping_res}")
            return 1
        print(f"[OK] Connected: FW={ping_res.get('fw')}, PCA9685={ping_res.get('pca9685')}, INA219={ping_res.get('ina219')}")

        # Set conservative single-joint software current limit (450 mA) via INA219
        print("[SAFETY] Setting INA219 single-servo stall current cutoff to 450 mA...")
        bridge.send_cmd({"cmd": "SET_CURRENT_LIMIT", "max_ma": 450})

        if args.wiggle or args.dry_run:
            print("[MOTION] Commanding smooth cosine-interpolated +/-15 deg wiggle...")
            for target_deg in [0.0, 15.0, -15.0, 0.0]:
                pwm_us = deg_to_pwm_us(target_deg)
                res = bridge.send_cmd(
                    {
                        "cmd": "MOVE",
                        "ch": args.channel,
                        "deg": target_deg,
                        "pwm_us": pwm_us,
                        "duration_ms": 500,
                    }
                )
                print(
                    f"  -> Ch {args.channel}: {target_deg:+5.1f} deg ({pwm_us} us) | "
                    f"Bus={res.get('bus_v', 0):.2f}V | Current={res.get('current_ma', 0)} mA"
                )
                if not args.dry_run:
                    time.sleep(0.55)

            print("[SAFETY] Disabling PWM pulse on channel (freewheel/relax mode)...")
            bridge.send_cmd({"cmd": "DISABLE", "ch": args.channel})

    finally:
        bridge.close()

    print("=== Single-Joint Bringup Complete ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
