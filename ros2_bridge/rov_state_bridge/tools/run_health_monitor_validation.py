from __future__ import annotations

"""Run a deterministic advisory health-monitor evaluation without ROS2.

This keeps stage2 health-monitor logic testable on a plain Linux/Python host.
"""

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'src'
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rov_state_bridge.health_monitor import build_health_monitor_status  # noqa: E402
from rov_state_bridge.models import (  # noqa: E402
    CommandResult,
    ControlState,
    HealthMonitorStatus,
    NavStateView,
    SystemState,
    TelemetryFrameV2,
    to_jsonable,
)


def main() -> int:
    telemetry = TelemetryFrameV2(
        seq=12,
        stamp_ns=123456789,
        control=ControlState(failsafe_active=1),
        system=SystemState(
            session_state=3,
            health_state=3,
            fault_state=1,
            last_fault_code=6,
            nav_fault_code=12,
            nav_status_flags=(1 << 10),
            nav_age_ms=240,
            heartbeat_age_ms=15,
            nav_valid=0,
            nav_stale=1,
            nav_degraded=0,
        ),
        last_command_result=CommandResult(status=5, fault_code=12),
    )
    nav_view = NavStateView(age_ms=240, valid=0, stale=1, degraded=0, fault_code=12, status_flags=(1 << 10))
    status = build_health_monitor_status(telemetry, nav_view)
    assert isinstance(status, HealthMonitorStatus)
    print(json.dumps(to_jsonable(status), sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
