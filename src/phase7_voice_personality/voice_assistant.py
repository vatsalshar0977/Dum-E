"""Phase 7 - offline-first voice control and the intent layer.

THE SHAPE OF THIS FILE
----------------------
    speech -> text        `WhisperTranscriber` (optional) or `--command` (always available)
    text   -> intent      `parse()`, pure stdlib, no network, no randomness
    intent -> behaviour    `Assistant`, which drives the personality engine and Phase 6

The text-to-intent step is deliberately deterministic and dependency-free. A laptop with no
microphone, no model and no internet still gets the whole robot: type or pipe the sentence in.
Speech recognition is the *optional* front end, never the only way in.

WHAT TO WORRY ABOUT
-------------------
* "Stop" is parsed first, on the raw text, before any other pattern is considered, and it
  never needs the STT model to be loaded. That said, voice is not a safety device. The
  physical e-stop (LA38-11ZS on GPIO 4 through the PCA9685 OE pin) is the safety device. A
  voice "stop" that has to survive room noise is a convenience.
* Whisper downloads its model on first use (about 75 MB for `tiny.en`). After that it runs
  offline. `--mic` needs `sounddevice`, which needs PortAudio.
* The microphone is a privacy surface. `--mic` is opt-in, one utterance at a time, and this
  file never opens a recording stream in the background.

Install the optional pieces only if you want speech:

    python -m pip install faster-whisper sounddevice soundfile      # fast, CTranslate2
    python -m pip install openai-whisper sounddevice soundfile      # reference build
"""

from __future__ import annotations

import argparse
import difflib
import json
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.phase6_perception.vision_mobile_navigator import (  # noqa: E402
    MAX_COMMAND, Reading, choose_wheels, cliff_reflex, plan_fetch_sequence,
)
from src.phase7_voice_personality.personality_engine import (  # noqa: E402
    HOME_DEG, SEQUENCES, WHEEL_STOP_US, PersonalityDirector,
)

# ---------------------------------------------------------------------------------------
# What B.U.D.-E can be asked for
# ---------------------------------------------------------------------------------------
# Canonical tool -> the words a person actually says. Keep this short and physical: the arm
# can only hand over something it can see and reach, and every entry here needs a place on
# the desk that the camera can find (see Phase 6's demo targets).
TOOL_SYNONYMS: Dict[str, Tuple[str, ...]] = {
    "screwdriver": ("screwdriver", "screw driver", "phillips", "philips", "driver"),
    "allen key": ("allen key", "allen wrench", "hex key", "hex wrench", "allen"),
    "pliers": ("pliers", "plyers", "side cutters", "wire cutters", "nippers"),
    "tape": ("tape", "tape measure", "measuring tape", "ruler"),
    "pen": ("pen", "pencil", "marker", "biro"),
    "cup": ("cup", "mug", "glass"),
    "screws": ("screws", "screw", "a screw", "bolts", "m3 screws"),
}

# The arm's transport/stow pose: folded up over the base so the rover can drive. Phase 6's
# `choose_wheels` refuses to move unless `arm_stowed` is true, and this is the pose that makes
# it true. It is a declaration, not a measurement: check that nothing fouls at J2 and J3.
HOME_PERCHED_DEG: Tuple[float, ...] = (0.0, -45.0, 75.0, -60.0, 0.0, 0.0)

HALT_PATTERN = re.compile(
    r"\b(stop|freeze|halt|abort|cancel|quit|emergency\s*stop|e[\s-]?stop|hold\s+still|"
    r"don'?t\s+move|stop\s+it)\b")
HOME_PATTERN = re.compile(
    r"\b(go\s+home|home|sleep|sleepy|rest|park|stand\s*by|good\s*night|goodnight|"
    r"perch|fold\s+up)\b")
FOLLOW_PATTERN = re.compile(
    r"\b(follow\s+me|come\s+here|come\s+to\s+me|come\s+along|stay\s+with\s+me|"
    r"chase\s+me|come\s+on)\b")
HAND_PATTERN = re.compile(
    r"\b(look\s+at\s+(my\s+)?(hand|hands|finger|fingers|palm)|watch\s+(my\s+)?hand|"
    r"look\s+here|look\s+at\s+me|see\s+my\s+hand)\b")
WAKE_PATTERN = re.compile(r"\b(hey|hi|hello|yo|ok|okay)?\s*(bud[\s-]?e|buddy|bud)\b")
FETCH_VERB = re.compile(
    r"\b(hand|pass|give|bring|fetch|get|grab|pick\s+up|take|gimme|give\s+me)\b")
PRAISE_PATTERN = re.compile(
    r"\b(thank\s+you|thanks|good\s+(job|work|boy|girl)|well\s+done|nice\s+work|"
    r"great\s+job|clever)\b")
HELP_PATTERN = re.compile(r"\b(help|what\s+can\s+you\s+do|commands|options)\b")


@dataclass(frozen=True)
class Intent:
    """What the sentence meant, plus why we think so."""

    name: str
    response: str
    tool: Optional[str] = None
    confidence: float = 1.0
    evidence: str = ""
    safety_critical: bool = False

    def as_dict(self) -> Dict[str, object]:
        return {"intent": self.name, "tool": self.tool, "confidence": round(self.confidence, 3),
                "response": self.response, "evidence": self.evidence,
                "safety_critical": self.safety_critical}


KNOWN_PHRASES = (
    "hand me the screwdriver", "fetch the allen key", "get me the pliers",
    "look at my hand", "follow me", "stop", "freeze", "go home", "sleep",
    "thank you", "what can you do",
)


def _normalise(text: str) -> str:
    lowered = text.lower().replace("-", " ")
    lowered = re.sub(r"[^a-z0-9\s']", " ", lowered)
    return re.sub(r"\s+", " ", lowered).strip()


def _find_tool(text: str) -> Optional[str]:
    best: Optional[Tuple[int, str]] = None
    for canonical, synonyms in TOOL_SYNONYMS.items():
        for word in synonyms:
            if re.search(rf"\b{re.escape(word)}\b", text):
                if best is None or len(word) > best[0]:
                    best = (len(word), canonical)
    return best[1] if best else None


def parse(text: str) -> Intent:
    """Map one sentence to one intent. Deterministic: same text, same answer, always."""
    if not isinstance(text, str) or not text.strip():
        return Intent("UNKNOWN", "I did not catch that.", confidence=0.0,
                      evidence="empty transcript")
    raw = _normalise(text)
    # 1. Safety first, on the raw words, before anything else can claim the sentence.
    if HALT_PATTERN.search(raw):
        hit = HALT_PATTERN.search(raw)
        return Intent("HALT", "Stopping.", confidence=1.0,
                      evidence=f"matched {hit.group(0)!r}", safety_critical=True)
    # 2. Then the physical commands.
    tool = _find_tool(raw)
    if HAND_PATTERN.search(raw):
        hit = HAND_PATTERN.search(raw)
        return Intent("LOOK_AT_HAND", "Watching your hand.", confidence=0.95,
                      evidence=f"matched {hit.group(0)!r}")
    if FOLLOW_PATTERN.search(raw):
        hit = FOLLOW_PATTERN.search(raw)
        return Intent("FOLLOW_ME", "Coming after you, slowly.", confidence=0.95,
                      evidence=f"matched {hit.group(0)!r}")
    if HOME_PATTERN.search(raw):
        hit = HOME_PATTERN.search(raw)
        return Intent("GO_HOME", "Folding up and going to sleep.", confidence=0.9,
                      evidence=f"matched {hit.group(0)!r}")
    if tool is not None and FETCH_VERB.search(raw):
        return Intent("FETCH_TOOL", f"Getting the {tool}.", tool=tool, confidence=0.9,
                      evidence="fetch verb with a known object")
    if tool is not None:
        return Intent("FETCH_TOOL", f"Getting the {tool}.", tool=tool, confidence=0.6,
                      evidence="bare object name")
    if PRAISE_PATTERN.search(raw):
        return Intent("PRAISE", "Thank you!", confidence=0.9, evidence="praise phrase")
    if HELP_PATTERN.search(raw):
        return Intent("HELP", "I can fetch a tool, watch your hand, follow you, go home, "
                              "or stop. Say or type any of those.", confidence=0.9,
                      evidence="help phrase")
    if WAKE_PATTERN.search(raw):
        return Intent("WAKE", "Yes? I am listening.", confidence=0.8,
                      evidence="wake word")
    near = difflib.get_close_matches(raw, KNOWN_PHRASES, n=1, cutoff=0.6)
    hint = f" Did you mean {near[0]!r}?" if near else ""
    return Intent("UNKNOWN", f"I did not understand {text.strip()!r}.{hint}",
                  confidence=0.0, evidence="no pattern matched")


# ---------------------------------------------------------------------------------------
# Optional speech-to-text
# ---------------------------------------------------------------------------------------
DEFAULT_MODEL = "tiny.en"          # 75 MB, ~1 s per utterance on a laptop CPU
BETTER_MODEL = "base.en"           # 145 MB, noticeably better with background noise


class WhisperTranscriber:
    """faster-whisper, then openai-whisper, then nothing at all.

    `available()` is checked before anything else touches the microphone, so a machine
    without the packages gets a clear message instead of a stack trace.
    """

    def __init__(self, model_size: str = DEFAULT_MODEL) -> None:
        self.model_size = model_size
        self.backend = ""
        self._model = None

    def available(self) -> bool:
        try:
            import faster_whisper          # noqa: F401
            self.backend = "faster-whisper"
            return True
        except ImportError:
            pass
        try:
            import whisper                 # noqa: F401
            self.backend = "openai-whisper"
            return True
        except ImportError:
            return False

    def load(self):
        if self._model is not None:
            return self._model
        if self.backend == "faster-whisper":
            from faster_whisper import WhisperModel
            # int8 on CPU: about 3x faster than float32 with no useful accuracy loss for
            # the short, in-vocabulary sentences this robot needs.
            self._model = WhisperModel(self.model_size, device="cpu", compute_type="int8")
        elif self.backend == "openai-whisper":
            import whisper
            self._model = whisper.load_model(self.model_size)
        else:
            raise RuntimeError("no whisper backend: pip install faster-whisper")
        return self._model

    def transcribe(self, wav_path: Path) -> str:
        if not Path(wav_path).is_file():
            raise FileNotFoundError(wav_path)
        if not self.available():
            raise RuntimeError(
                "no speech backend. Install one with either\n"
                "    python -m pip install faster-whisper sounddevice soundfile\n"
                "    python -m pip install openai-whisper sounddevice soundfile\n"
                "or skip speech entirely and use: python voice_assistant.py --command \"stop\"")
        model = self.load()
        if self.backend == "faster-whisper":
            segments, _info = model.transcribe(str(wav_path), language="en", beam_size=1)
            return " ".join(seg.text for seg in segments).strip()
        return str(model.transcribe(str(wav_path), language="en")["text"]).strip()

    def listen_once(self, seconds: float = 4.0, sample_rate: int = 16000) -> str:
        """Record one utterance and transcribe it. No background stream, ever."""
        try:
            import sounddevice as sd
        except ImportError as exc:
            raise RuntimeError(
                "--mic needs sounddevice (pip install sounddevice soundfile). Without a "
                "microphone use --command.") from exc
        import wave
        out = ROOT / "logs" / "last_utterance.wav"
        out.parent.mkdir(parents=True, exist_ok=True)
        frames = int(seconds * sample_rate)
        print(f"  listening for {seconds:.0f} s ... speak now")
        audio = sd.rec(frames, samplerate=sample_rate, channels=1, dtype="int16")
        sd.wait()
        with wave.open(str(out), "wb") as handle:
            handle.setnchannels(1)
            handle.setsampwidth(2)
            handle.setframerate(sample_rate)
            handle.writeframes(audio.tobytes())
        text = self.transcribe(out)
        print(f"  heard: {text!r}")
        return text


# ---------------------------------------------------------------------------------------
# Turning intents into movement
# ---------------------------------------------------------------------------------------
def wheel_us_from_command(command: float) -> float:
    """Map Phase 6's normalised command (-0.15..0.15) to a servo pulse width.

    Linear and uncalibrated: `MAX_COMMAND` becomes 2000 us, and the sign picks the
    direction. A real rover needs a dead-band calibration, because a continuous-rotation
    servo almost never stops at exactly 1500 us.
    """
    clamped = max(-MAX_COMMAND, min(MAX_COMMAND, command))
    return WHEEL_STOP_US + (clamped / MAX_COMMAND) * 500.0


@dataclass
class AssistantReport:
    intent: Intent
    gestures: List[str] = field(default_factory=list)
    wheel_commands: List[float] = field(default_factory=list)
    notes: List[str] = field(default_factory=list)

    def as_dict(self) -> Dict[str, object]:
        return {"intent": self.intent.as_dict(), "gestures": self.gestures,
                "wheel_commands": [round(c, 4) for c in self.wheel_commands],
                "notes": self.notes}


class SimRobotLink:
    """A stand-in for the hardware. Prints what it would do and runs the Phase 6 policy."""

    def __init__(self, verbose: bool = True) -> None:
        self.verbose = verbose
        self.director = PersonalityDirector(t0=0.0)
        self.t = 0.0
        self.stowed = True
        self.jaw_deg = 0.0
        self.jaw_target_deg = 0.0
        self.log: List[str] = []

    # -- helpers -------------------------------------------------------------------
    def _say(self, message: str) -> None:
        self.log.append(message)
        if self.verbose:
            print(f"    {message}")

    def play(self, sequence_name: str, t: Optional[float] = None) -> None:
        seq = SEQUENCES[sequence_name]
        self.t = self.t if t is None else t
        self.director.state.sequence = seq
        self.director.state.started = self.t
        self.director.state.last_motion_t = self.t
        self._say(f"gesture {sequence_name} for {seq.duration:.2f} s "
                  f"({len(seq.sample())} frames at 100 Hz)")

    def stow(self) -> None:
        self.stowed = True
        self._say(f"arm stowed at home_perched {tuple(round(v, 1) for v in HOME_PERCHED_DEG)}")

    def halt(self) -> None:
        frame = self.director.halt(self.t)
        self._say(f"HALT at t={self.t:.2f} s: wheels {frame.wheels[0]:.0f}/"
                  f"{frame.wheels[1]:.0f} us, arm held at "
                  f"{tuple(round(v, 1) for v in frame.joints[:5])}")

    def close_jaw(self, target_deg: float, current_ma: float, held_s: float) -> None:
        self.jaw_target_deg = target_deg
        self.jaw_deg = 3.0 if target_deg <= 5.0 else target_deg * 0.2
        self._say(f"jaw commanded to {target_deg:.0f} deg, INA219 reads {current_ma:.0f} mA "
                  f"after {held_s:.2f} s")

    def fetch(self, tool: str, object_mm: Tuple[float, float] = (1050.0, 180.0)) -> dict:
        """The full 'hand me the screwdriver' flow: nod, stow, drive, grasp, wiggle."""
        self.play("affirmative_nod")
        self.t += SEQUENCES["affirmative_nod"].duration
        self.stow()
        if not self.stowed:
            self._say("refusing to drive: arm is not stowed")
            return {"arrived": False}
        plan = plan_fetch_sequence(object_mm)
        fx, fy, fh = plan["final_pose"]
        self._say(f"Phase 6 navigator: arrived={plan['arrived']} after {plan['ticks']} ticks, "
                  f"final ({fx:.0f}, {fy:.0f}) mm, heading {math.degrees(fh):+.1f} deg, "
                  f"{plan['distance_to_standoff_mm']:.1f} mm off the standoff")
        self.play("excited_success_wiggle")
        self.t += SEQUENCES["excited_success_wiggle"].duration
        return plan

    def look_at_hand(self) -> None:
        self.play("curious_head_tilt")
        self._say("Phase 6 PCA hand-roll estimator armed; keep the hand in front of the "
                  "camera at 150-400 mm")
        self.t += SEQUENCES["curious_head_tilt"].duration

    def follow(self) -> None:
        """Follow-me runs the cliff reflex whenever a sensor trips, every tick."""
        for trip, left_ok, right_ok in ((False, True, True), (True, True, False)):
            reading = Reading(now_s=self.t, sensor_time_s=self.t, target_bearing_rad=0.2,
                              obstacle_m=1.2, left_floor_ok=left_ok, right_floor_ok=right_ok,
                              estop_ok=True, arm_stowed=self.stowed)
            wheels = cliff_reflex(reading) if trip else choose_wheels(reading)
            self._say(f"tick t={self.t:.2f} s cliff={'TRIPPED' if trip else 'clear'} -> "
                      f"wheels {wheels[0]:+.3f} / {wheels[1]:+.3f} "
                      f"= {wheel_us_from_command(wheels[0]):.0f} / "
                      f"{wheel_us_from_command(wheels[1]):.0f} us")
            if trip:
                self.play("startled_cliff_recoil")
                self.t += SEQUENCES["startled_cliff_recoil"].duration
            else:
                self.t += 0.2
        self.play("idle_breathing_and_look_around")
        self._say("idle loop starts after 10 s of no commands")

    def go_home(self) -> None:
        self.stowed = True
        self.director.state.sequence = None
        self._say(f"home_perched {tuple(round(v, 1) for v in HOME_PERCHED_DEG)}; wheels to "
                  f"{WHEEL_STOP_US:.0f} us; gesture machine idle")


class HardwareRobotLink:
    """Where the real commands go. Left unimplemented on purpose.

    To fill this in you need three things that do not exist yet:
      1. an ESP32 sketch that owns the PCA9685, the INA219 and the two TCRT5000s, and that
         STOPS THE WHEELS IN HARDWARE when either cliff pin goes high - not in Python;
      2. a 1500 us calibration per wheel, because a continuous-rotation servo creeps;
      3. a watchdog: if no command arrives for 300 ms, cut PWM and let the arm sag (or hold,
         if you wired the OE pin to GPIO 4 as the e-stop does).
    """

    def __getattr__(self, name: str):
        raise NotImplementedError(
            f"{name}() needs the ESP32 firmware and a measured wheel calibration; see "
            f"docs/phase3_electronics_and_wiring.md")


class Assistant:
    """Turns one utterance into one action, then reports exactly what it did."""

    def __init__(self, robot: Optional[object] = None) -> None:
        self.robot = robot if robot is not None else SimRobotLink()

    def handle(self, text: str) -> AssistantReport:
        intent = parse(text)
        report = AssistantReport(intent=intent)
        if intent.name == "HALT":
            self.robot.halt()                       # type: ignore[attr-defined]
            report.gestures.append("HALT (immediate, no blend)")
            return report
        if intent.name == "FETCH_TOOL":
            self.robot.fetch(intent.tool)            # type: ignore[attr-defined]
            report.gestures += ["affirmative_nod", "excited_success_wiggle"]
            report.notes.append("the arm must be stowed (Phase 6 policy) before the wheels "
                                "turn at all")
            return report
        if intent.name == "LOOK_AT_HAND":
            self.robot.look_at_hand()                # type: ignore[attr-defined]
            report.gestures.append("curious_head_tilt")
            report.notes.append("keep the hand 150-400 mm from the camera, palm open")
            return report
        if intent.name == "FOLLOW_ME":
            self.robot.follow()                      # type: ignore[attr-defined]
            report.gestures += ["startled_cliff_recoil", "idle_breathing_and_look_around"]
            report.notes.append("follow-me is floor-only: the cliff reflex is the last line "
                                "of defence and it is not a substitute for a bumper")
            return report
        if intent.name == "GO_HOME":
            self.robot.go_home()                     # type: ignore[attr-defined]
            report.notes.append(f"home_perched = {tuple(round(v, 1) for v in HOME_PERCHED_DEG)}")
            return report
        if intent.name == "WAKE":
            self.robot.play("curious_head_tilt")     # type: ignore[attr-defined]
            report.gestures.append("curious_head_tilt")
            return report
        if intent.name == "PRAISE":
            self.robot.play("excited_success_wiggle")   # type: ignore[attr-defined]
            report.gestures.append("excited_success_wiggle")
            return report
        if intent.name == "HELP":
            report.notes.append("known commands: " + ", ".join(KNOWN_PHRASES))
            return report
        report.notes.append("nothing moved: an unknown sentence is not a command")
        return report


# ---------------------------------------------------------------------------------------
# Demo and CLI
# ---------------------------------------------------------------------------------------
DEMO_LINES = (
    "bud-e, hand me the screwdriver",
    "look at my hand",
    "follow me",
    "stop",
    "thank you",
    "go home",
    "make me a sandwich",
)


def run_demo(as_json: bool = False) -> int:
    robot = SimRobotLink(verbose=not as_json)
    assistant = Assistant(robot)
    rows = []
    if not as_json:
        print("B.U.D.-E Phase 7 voice + personality demo (no microphone, no network)")
        print("=" * 96)
    for line in DEMO_LINES:
        if not as_json:
            print(f"\n> {line}")
        report = assistant.handle(line)
        if not as_json:
            print(f"  -> {report.intent.name} ({report.intent.confidence:.2f}): "
                  f"{report.intent.response}")
        rows.append({"said": line, **report.as_dict()})
    if as_json:
        print(json.dumps({"ok": True, "turns": rows}, indent=2))
        return 0
    print("\n" + "=" * 96)
    print(f"  {len(rows)} turns, {len(robot.log)} robot actions, all offline. The microphone "
          f"was never opened.")
    print("  Reality check: everything above ran in a simulator. No servo moved.")
    return 0


def run_commands(commands: Sequence[str], as_json: bool) -> int:
    robot = SimRobotLink(verbose=not as_json)
    assistant = Assistant(robot)
    rows = []
    for command in commands:
        report = assistant.handle(command)
        rows.append({"said": command, **report.as_dict()})
        if not as_json:
            print(f"> {command}\n  -> {report.intent.name} ({report.intent.confidence:.2f}): "
                  f"{report.intent.response}")
            for note in report.notes:
                print(f"     note: {note}")
    if as_json:
        print(json.dumps({"ok": True, "turns": rows}, indent=2))
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="B.U.D.-E Phase 7 voice assistant: offline-first speech, deterministic "
                    "text commands, and the personality engine behind both.")
    parser.add_argument("--command", "-c", action="append", default=[],
                        help="run one typed sentence (repeatable). Needs no microphone, no "
                             "model and no network.")
    parser.add_argument("--demo", action="store_true",
                        help="seven scripted turns, printed action by action (the default)")
    parser.add_argument("--mic", action="store_true",
                        help="record one utterance and transcribe it (needs sounddevice and "
                             "a whisper backend)")
    parser.add_argument("--seconds", type=float, default=4.0,
                        help="microphone window for --mic (default 4 s)")
    parser.add_argument("--transcribe", type=Path,
                        help="transcribe an existing wav file and run it")
    parser.add_argument("--model", default=DEFAULT_MODEL,
                        help=f"whisper model size (default {DEFAULT_MODEL}; {BETTER_MODEL} is "
                             f"better in noise)")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    parser.add_argument("--list", action="store_true",
                        help="list the gestures and the phrases the parser knows")
    args = parser.parse_args(argv)

    if args.list:
        print("Gestures (100 Hz minimum-jerk, see personality_engine.py):")
        for name, seq in SEQUENCES.items():
            trigger = ", ".join(seq.triggers) or "-"
            print(f"  {name:<32} {seq.duration:5.2f} s  priority {seq.priority}  "
                  f"triggers: {trigger}")
            print(f"      {seq.description}")
        print("\nKnown phrases (approximate matching is used as a fallback):")
        for phrase in KNOWN_PHRASES:
            print(f"  {phrase}")
        print("\nTools it will go and get:")
        for canonical, synonyms in TOOL_SYNONYMS.items():
            print(f"  {canonical:<12} <- {', '.join(synonyms)}")
        return 0

    if args.mic or args.transcribe is not None:
        transcriber = WhisperTranscriber(model_size=args.model)
        if not transcriber.available():
            print("No speech backend installed. Install one of:")
            print("    python -m pip install faster-whisper sounddevice soundfile")
            print("    python -m pip install openai-whisper sounddevice soundfile")
            print("Everything else works without speech: try --demo or --command \"stop\".")
            return 3
        try:
            text = (transcriber.transcribe(args.transcribe) if args.transcribe is not None
                    else transcriber.listen_once(args.seconds))
        except (RuntimeError, FileNotFoundError) as exc:
            print(f"speech failed: {exc}")
            return 3
        print(f"transcript: {text!r}  (backend: {transcriber.backend}, "
              f"model: {transcriber.model_size})")
        if not text:
            print("empty transcript: nothing to do")
            return 0
        return run_commands([text], args.json)

    if args.command:
        return run_commands(args.command, args.json)
    return run_demo(args.json)


if __name__ == "__main__":
    sys.exit(main())
