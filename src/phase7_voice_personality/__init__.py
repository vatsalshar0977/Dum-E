"""Phase 7 - voice control and personality for B.U.D.-E.

Two halves that never depend on each other:

    personality_engine.py   the body language: 100 Hz minimum-jerk keyframe sequences over the
                            6 arm joints and the 2 rover wheels, plus the director that picks
                            a gesture from an event and the failure detector that decides when
                            a grasp missed.
    voice_assistant.py      the front end: an optional offline Whisper transcriber, a
                            deterministic text-to-intent parser, and the assistant that turns
                            an intent into gestures, wheel commands and (on the real robot)
                            movement through the Phase 6 navigation policy.

Quick start, no microphone and no network required:

    python3 src/phase7_voice_personality/personality_engine.py        # gesture self test
    python3 src/phase7_voice_personality/voice_assistant.py           # seven-turn demo
    python3 src/phase7_voice_personality/voice_assistant.py -c "stop"
    python3 src/phase7_voice_personality/voice_assistant.py --list

With speech (optional, needs one of the whisper packages):

    python -m pip install faster-whisper sounddevice soundfile
    python3 src/phase7_voice_personality/voice_assistant.py --mic --seconds 4

And in the browser: `python3 src/phase4_simulation/web_sim_server.py`, then the
"Voice & personality" panel plays every gesture on the drawn arm at real speed.

Read docs/phase7_voice_and_personality.md before connecting any of this to servos. A voice
"stop" is a convenience; the LA38-11ZS e-stop is the safety device.
"""

from src.phase7_voice_personality.personality_engine import (    # noqa: F401
    AFFIRMATIVE_NOD, CHANNEL_NAMES, CURIOUS_HEAD_TILT, EXCITED_SUCCESS_WIGGLE, Frame,
    GESTURE_SPEED_DEG_S, HOME_DEG, IDLE_BREATHING_AND_LOOK_AROUND,
    IDLE_BREATHING_AND_LOOK_AROUND as IDLE_LOOK_AROUND, JAW_LIMITS_DEG, JOINT_LIMITS_DEG,
    JOINT_NAMES, Keyframe, N_CHANNELS, PersonalityDirector, SAD_FAILED_GRASP_DROOP,
    SAMPLE_HZ, SEQUENCES, STARTLED_CLIFF_RECOIL, Sequence, WHEEL_MAX_US, WHEEL_MIN_US,
    WHEEL_NAMES, WHEEL_STOP_US, looks_like_failed_grasp, min_jerk_duration, minimum_jerk,
)
from src.phase7_voice_personality.voice_assistant import (      # noqa: F401
    Assistant, AssistantReport, DEFAULT_MODEL, HALT_PATTERN, HOME_PERCHED_DEG, Intent,
    KNOWN_PHRASES, TOOL_SYNONYMS, SimRobotLink, WhisperTranscriber, parse,
    wheel_us_from_command,
)

__all__ = [
    "AFFIRMATIVE_NOD", "Assistant", "AssistantReport", "CHANNEL_NAMES", "CURIOUS_HEAD_TILT",
    "DEFAULT_MODEL", "EXCITED_SUCCESS_WIGGLE", "Frame", "GESTURE_SPEED_DEG_S", "HALT_PATTERN",
    "HOME_DEG", "HOME_PERCHED_DEG", "IDLE_BREATHING_AND_LOOK_AROUND", "IDLE_LOOK_AROUND",
    "Intent", "JAW_LIMITS_DEG", "JOINT_LIMITS_DEG", "JOINT_NAMES", "KNOWN_PHRASES", "Keyframe",
    "N_CHANNELS", "PersonalityDirector", "SAD_FAILED_GRASP_DROOP", "SAMPLE_HZ", "SEQUENCES",
    "STARTLED_CLIFF_RECOIL", "Sequence", "SimRobotLink", "TOOL_SYNONYMS", "WHEEL_MAX_US",
    "WHEEL_MIN_US", "WHEEL_NAMES", "WHEEL_STOP_US", "WhisperTranscriber",
    "looks_like_failed_grasp", "min_jerk_duration", "minimum_jerk", "parse",
    "wheel_us_from_command",
]
