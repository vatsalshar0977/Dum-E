"""Phase 8 - safety supervision, fault injection, telemetry and the master scorecard.

    safety_supervisor_and_logger.py   the supervisor (6 fault classes, latched e-stop,
                                      telemetry JSONL/CSV) and the fault injectors
    run_full_acceptance_suite.py      one command, all eight phases, one scorecard

Run both, in this order:

    python3 src/phase8_safety_testing/safety_supervisor_and_logger.py --echo
    python3 src/phase8_safety_testing/run_full_acceptance_suite.py

Telemetry is written to logs/ (git-ignored, because it is generated). Read
docs/phase8_safety_testing_and_bringup_checklist.md before the first power-up: it has the
FMEA table and the bring-up order.
"""

from src.phase8_safety_testing.safety_supervisor_and_logger import (   # noqa: F401
    BACKOFF_HOME,
    BROWNOUT_HOLD,
    BROWNOUT_RECOVER_V,
    BROWNOUT_V,
    CLIFF_RECOVER,
    Command,
    ESTOP_HOLD,
    ESTOP_MAX_V,
    FAULTS,
    FaultInjector,
    FaultResult,
    GRASP_RETRY,
    HARDWARE_CLIFF_DEADLINE_MS,
    IDLE,
    MISSED_GRASP_MAX_RETRIES,
    MISSED_GRASP_MA,
    OBSTACLE_HOLD,
    OBSTACLE_SLOW_MM,
    OBSTACLE_STOP_MM,
    OVERCURRENT_HOLD_S,
    OVERCURRENT_MA,
    SafetySupervisor,
    Sensors,
    TelemetryRecorder,
    TelemetryRow,
)

__all__ = [
    "BACKOFF_HOME", "BROWNOUT_HOLD", "BROWNOUT_RECOVER_V", "BROWNOUT_V", "CLIFF_RECOVER",
    "Command", "ESTOP_HOLD", "ESTOP_MAX_V", "FAULTS", "FaultInjector", "FaultResult",
    "GRASP_RETRY", "HARDWARE_CLIFF_DEADLINE_MS", "IDLE", "MISSED_GRASP_MAX_RETRIES",
    "MISSED_GRASP_MA", "OBSTACLE_HOLD", "OBSTACLE_SLOW_MM", "OBSTACLE_STOP_MM",
    "OVERCURRENT_HOLD_S", "OVERCURRENT_MA", "SafetySupervisor", "Sensors",
    "TelemetryRecorder", "TelemetryRow",
]
