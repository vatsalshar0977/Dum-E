"""Phase 3 — host-side serial driver for the B.U.D.-E ESP32.

This is the laptop end of the link that `firmware/phase3_full_6dof_esp32/phase3_full_6dof_esp32.ino`
speaks. It is deliberately dumb: it converts joint angles to pulse widths using the same table
the firmware uses, frames the command, and writes one newline-terminated line. Everything that
needs to be fast - blending, the cliff reflex, the wheel timeout - happens on the ESP32 and is
NOT duplicated here.

    python3 src/phase3_electronics/esp32_arm_driver.py --dry-run --home
    python3 src/phase3_electronics/esp32_arm_driver.py --dry-run --wheel 1320 1680
    python3 src/phase3_electronics/esp32_arm_driver.py --dry-run --grip 100
    python3 src/phase3_electronics/esp32_arm_driver.py --port COM3 --ping

SAFETY, and this file tries hard not to be the thing that hurts you:

  * Without `--port`, nothing is opened and nothing is sent: the frames are printed. `--dry-run`
    is therefore the default, not an option you have to remember.
  * Over a real port, any command that can move the robot (W / M / G) additionally requires
    `--yes`. Reading commands (PING / STATUS / HELP) do not.
  * `E-STOP` is not a serial command. It is a physical switch in the 6 V rail. This driver
    cannot stop a moving arm; it can only ask the firmware to, and it can hang.
  * The firmware refuses forward wheel commands while a cliff is tripped and returns the wheels
    to neutral if this script stops sending for 500 ms. Both are on the ESP32 so that a laptop
    crash, a USB unplug or a hung Python process cannot leave the rover driving.

Usage: `--help`. Exit code 0 = the frames were produced (dry run) or written.
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "config" / "robot_config.yaml"
SKETCH = ROOT / "firmware" / "phase3_full_6dof_esp32" / "phase3_full_6dof_esp32.ino"

BAUD = 115200
NEUTRAL_US = 1500
WHEEL_MIN_US, WHEEL_MAX_US = 1000, 2000
CHANNEL_COUNT = 8


class Joint:
    """One row of the config's joint table, plus the degrees <-> microseconds mapping."""

    def __init__(self, cfg_id: int, channel: int, name: str, min_deg: float, max_deg: float,
                 home_deg: float, pwm_min_us: float, pwm_max_us: float) -> None:
        self.cfg_id = cfg_id
        self.channel = channel
        self.name = name
        self.min_deg = min_deg
        self.max_deg = max_deg
        self.home_deg = home_deg
        self.pwm_min_us = pwm_min_us
        self.pwm_max_us = pwm_max_us

    def us_from_deg(self, deg: float) -> int:
        clamped = max(self.min_deg, min(self.max_deg, deg))
        span = (self.max_deg - self.min_deg) or 1.0
        return int(round(self.pwm_min_us + (clamped - self.min_deg) / span
                         * (self.pwm_max_us - self.pwm_min_us)))

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"<Joint {self.name} ch{self.channel} {self.min_deg}..{self.max_deg} deg>"


def load_joints(path: Path = CONFIG) -> List[Joint]:
    """Parse the joint table out of the YAML without depending on PyYAML."""
    joints: List[Joint] = []
    current: Dict[str, object] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if stripped.startswith("- id:"):
            if current:
                joints.append(_make_joint(current))
            current = {"cfg_id": int(stripped.split(":", 1)[1].strip())}
            continue
        if not current:
            continue
        for key in ("pca9685_channel", "min_deg", "max_deg", "home_deg", "pwm_min_us",
                    "pwm_max_us"):
            if stripped.startswith(f"{key}:"):
                value = stripped.split(":", 1)[1].split("#")[0].strip()
                try:
                    current[key] = float(value)
                except ValueError:
                    pass
            elif stripped.startswith("name:"):
                current["name"] = stripped.split(":", 1)[1].strip().strip('"')
    if current:
        joints.append(_make_joint(current))
    return joints


def _make_joint(raw: Dict[str, object]) -> Joint:
    return Joint(
        cfg_id=int(raw.get("cfg_id", 0)),                       # type: ignore[arg-type]
        channel=int(raw.get("pca9685_channel", 0)),             # type: ignore[arg-type]
        name=str(raw.get("name", "?")),
        min_deg=float(raw.get("min_deg", 0.0)),                 # type: ignore[arg-type]
        max_deg=float(raw.get("max_deg", 180.0)),               # type: ignore[arg-type]
        home_deg=float(raw.get("home_deg", 0.0)),               # type: ignore[arg-type]
        pwm_min_us=float(raw.get("pwm_min_us", 500)),           # type: ignore[arg-type]
        pwm_max_us=float(raw.get("pwm_max_us", 2500)),          # type: ignore[arg-type]
    )


# --------------------------------------------------------------------------------- framing
def frame_ping() -> str:
    return "PING"


def frame_status() -> str:
    return "STATUS"


def frame_help() -> str:
    return "HELP"


def frame_outputs(enabled: bool) -> str:
    return f"OE {0 if enabled else 1}"


def frame_wheels(left_us: int, right_us: int) -> str:
    for value in (left_us, right_us):
        if not WHEEL_MIN_US <= value <= WHEEL_MAX_US:
            raise ValueError(f"wheel pulse {value} us is outside {WHEEL_MIN_US}-{WHEEL_MAX_US}")
    return f"W {int(left_us)} {int(right_us)}"


def frame_channels(pairs: Sequence[Tuple[int, int]], duration_ms: Optional[int] = None) -> str:
    """`M ch:us,...` (immediate) or `G ms ch:us,...` (cosine-blended on the ESP32)."""
    body = ",".join(f"{int(ch)}:{int(us)}" for ch, us in pairs)
    if duration_ms is None:
        return f"M {body}"
    return f"G {int(duration_ms)} {body}"


def wheel_command_from_velocity(left_speed: float, right_speed: float) -> str:
    """Map -1.0 .. +1.0 per wheel to the mirrored forward/reverse pulse pair.

    The two drive servos face each other, so 'forward' is 1320 us on the left and 1680 us on
    the right - the same convention the firmware encodes as
    `left_forward_is_above_neutral: false` / `right_forward_is_above_neutral: true`.
    """
    for speed in (left_speed, right_speed):
        if not -1.0 <= speed <= 1.0:
            raise ValueError("wheel speed must be within -1.0 .. +1.0")
    left = NEUTRAL_US - int(round(left_speed * 180.0))
    right = NEUTRAL_US + int(round(right_speed * 180.0))
    return frame_wheels(left, right)


def home_frames(joints: Sequence[Joint], duration_ms: int = 1500) -> List[str]:
    """Blend every arm joint to its configured home angle in one `G` command."""
    pairs = [(joint.channel, joint.us_from_deg(joint.home_deg)) for joint in joints]
    return [frame_channels(pairs, duration_ms)]


def grip_frame(joints: Sequence[Joint], percent_open: float) -> str:
    """0 % = fully closed, 100 % = fully open, using the gripper's own config row."""
    gripper = next((j for j in joints if j.name == "gripper"), None)
    if gripper is None:
        raise ValueError("no 'gripper' joint in config/robot_config.yaml")
    percent_open = max(0.0, min(100.0, percent_open))
    deg = gripper.min_deg + (gripper.max_deg - gripper.min_deg) * percent_open / 100.0
    return frame_channels([(gripper.channel, gripper.us_from_deg(deg))])


# --------------------------------------------------------------------------- firmware check
def firmware_agreement(sketch: str) -> List[str]:
    """Complain (do not guess) if the sketch disagrees with this driver's assumptions."""
    warnings: List[str] = []
    expectations = {
        "WHEEL_LEFT_CH": 6,
        "WHEEL_RIGHT_CH": 7,
        "CHANNEL_COUNT": CHANNEL_COUNT,
    }
    for name, expected in expectations.items():
        match = re.search(rf"\b{name}\s*=\s*(\d+)", sketch)
        if match is None:
            warnings.append(f"the sketch does not define {name}; this driver assumes {expected}")
        elif int(match.group(1)) != expected:
            warnings.append(f"the sketch has {name}={match.group(1)}, this driver assumes "
                            f"{expected}")
    if "applyWheelPair" not in sketch:
        warnings.append("the sketch has no applyWheelPair(): forward wheel commands may bypass "
                        "the cliff gate")
    return warnings


# ----------------------------------------------------------------------------------- output
class Link:
    """Writes frames either to stdout (dry run) or to a real serial port."""

    def __init__(self, port: Optional[str], baud: int, assume_yes: bool, quiet: bool = False):
        self.port = port
        self.baud = baud
        self.assume_yes = assume_yes
        self.quiet = quiet
        self.serial = None
        self.sent: List[str] = []
        self.dry_run = port is None

    def open(self) -> None:
        if self.dry_run:
            print("  DRY RUN — no serial port opened, no hardware addressed.")
            return
        try:
            import serial  # type: ignore  # pyserial
        except ImportError:
            print("  pyserial is not installed:  python3 -m pip install pyserial")
            print("  (rerun with --dry-run to see the frames without hardware)")
            raise SystemExit(3)
        self.serial = serial.Serial(self.port, self.baud, timeout=1.0)
        time.sleep(2.0)                      # the ESP32 resets when the port opens
        self.serial.reset_input_buffer()
        print(f"  opened {self.port} at {self.baud} baud "
              f"(the board reboots on connect: waited 2 s)")

    def send(self, frame: str, moves_robot: bool) -> None:
        if moves_robot and not self.dry_run and not self.assume_yes:
            print(f"  REFUSED: {frame!r} can move the robot. Use --yes once the area is clear "
                  f"and the e-stop is in reach.")
            raise SystemExit(4)
        self.sent.append(frame)
        if self.dry_run:
            print(f"  TX  {frame}")
            wire_bytes = frame.encode() + b"\n"
            print(f"      {len(wire_bytes)} bytes on the wire -> {wire_bytes!r}")
        else:
            assert self.serial is not None
            self.serial.write(frame.encode() + b"\n")
            self.serial.flush()
            if not self.quiet:
                reply = self.serial.readline().decode("utf-8", "replace").strip()
                print(f"  TX  {frame}\n  RX  {reply or '(no reply within 1 s)'}")

    def close(self) -> None:
        if self.serial is not None:
            self.serial.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Host-side serial driver for the B.U.D.-E ESP32",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="The physical e-stop is the only thing that stops this robot.")
    parser.add_argument("--port", help="serial port, e.g. COM3 or /dev/ttyUSB0. "
                                       "Omit for a dry run.")
    parser.add_argument("--baud", type=int, default=BAUD)
    parser.add_argument("--dry-run", action="store_true",
                        help="print the frames without opening a port (the default)")
    parser.add_argument("--yes", action="store_true",
                        help="allow commands that can move the robot over a real port")
    parser.add_argument("--ping", action="store_true", help="PING: identity and settings")
    parser.add_argument("--status", action="store_true", help="STATUS: the full telemetry line")
    parser.add_argument("--help-cmd", action="store_true", help="ask the firmware for HELP")
    parser.add_argument("--home", action="store_true", help="blend every arm joint to home")
    parser.add_argument("--open", type=float, metavar="PCT",
                        help="gripper opening, 0 = closed, 100 = open")
    parser.add_argument("--wheel", nargs=2, type=int, metavar=("LEFT_US", "RIGHT_US"),
                        help="raw wheel pulses, 1500 1500 = stop, 1320 1680 = forward")
    parser.add_argument("--drive", nargs=2, type=float, metavar=("LEFT", "RIGHT"),
                        help="wheel speed -1.0 .. 1.0 per side (mirrored automatically)")
    parser.add_argument("--stop", action="store_true", help="W 1500 1500")
    parser.add_argument("--move", nargs="+", metavar="CH:DEG",
                        help="immediate joint moves in degrees, e.g. --move 0:15 3:-10")
    parser.add_argument("--move-ms", type=int, metavar="MS",
                        help="blend --move over MS milliseconds instead of jumping")
    parser.add_argument("--outputs", choices=["on", "off"],
                        help="enabling restores pulse output; off tri-states the PCA9685")
    parser.add_argument("--raw", action="append", default=[], metavar="LINE",
                        help="send a raw protocol line (repeatable)")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    joints = load_joints()
    by_channel = {joint.channel: joint for joint in joints}

    print("=" * 78)
    print("  B.U.D.-E ESP32 driver — Phase 3")
    print("=" * 78)
    print(f"  {len(joints)} arm joints loaded from config/robot_config.yaml")
    print("  " + ", ".join(f"ch{j.channel}={j.name}" for j in joints))

    sketch = SKETCH.read_text(encoding="utf-8") if SKETCH.exists() else ""
    warnings = firmware_agreement(sketch) if sketch else ["the firmware sketch was not found"]
    for warning in warnings:
        print(f"  WARNING: {warning}")
    if not warnings:
        print("  firmware agrees with this driver (channels 6/7, 8 channels, guarded wheel path)")

    link = Link(None if args.dry_run else args.port, args.baud, args.yes)
    link.open()

    plan: List[Tuple[str, bool]] = []            # (frame, moves_robot)
    if args.ping:
        plan.append((frame_ping(), False))
    if args.status:
        plan.append((frame_status(), False))
    if args.help_cmd:
        plan.append((frame_help(), False))
    if args.outputs is not None:
        plan.append((frame_outputs(args.outputs == "on"), False))
    if args.wheel is not None:
        plan.append((frame_wheels(*args.wheel), True))
    if args.drive is not None:
        plan.append((wheel_command_from_velocity(*args.drive), True))
    if args.stop:
        plan.append((frame_wheels(NEUTRAL_US, NEUTRAL_US), True))
    if args.open is not None:
        plan.append((grip_frame(joints, args.open), True))
    if args.move:
        pairs: List[Tuple[int, int]] = []
        for item in args.move:
            channel_text, _, deg_text = item.partition(":")
            channel = int(channel_text)
            joint = by_channel.get(channel)
            if joint is None:
                print(f"  REFUSED: channel {channel} is not an arm joint "
                      f"(arm joints are {sorted(by_channel)}; 6 and 7 are the rover wheels)")
                return 5
            pairs.append((channel, joint.us_from_deg(float(deg_text))))
        plan.append((frame_channels(pairs, args.move_ms), True))
    if args.home:
        for frame in home_frames(joints):
            plan.append((frame, True))
    for line in args.raw:
        plan.append((line, True))

    if not plan:
        print("\n  Nothing to do. Examples:")
        print("    --dry-run --ping")
        print("    --dry-run --home")
        print("    --dry-run --drive 0.5 0.5        # forward, mirrored automatically")
        print("    --dry-run --wheel 1500 1500      # stop")
        print("    --dry-run --open 100             # open the claw")
        link.close()
        return 0

    print("-" * 78)
    for frame, moves in plan:
        link.send(frame, moves)

    print("-" * 78)
    if link.dry_run:
        print(f"  {len(link.sent)} frame(s) prepared, nothing sent.")
        print("  What the firmware does with them:")
        print("    W  -> both wheel channels in ONE I2C transaction; forward is refused while a")
        print("          cliff is tripped, and the wheels return to 1500 us if this driver goes")
        print("          quiet for 500 ms")
        print("    M  -> immediate; G -> cosine-blended on the ESP32 over the given duration")
        print("    the cliff reflex runs on the ESP32 (GPIO 25/26, < 5 ms) regardless of what")
        print("    this laptop is doing, including being unplugged")
    else:
        print(f"  {len(link.sent)} frame(s) written to {link.port}.")
    print("  Reminder: the physical e-stop in the 6 V rail is the only thing that is guaranteed")
    print("  to stop this robot. `W 1500 1500` is a request, not a guarantee.")
    link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
