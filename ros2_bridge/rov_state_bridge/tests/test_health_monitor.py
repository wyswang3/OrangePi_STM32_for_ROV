from __future__ import annotations

import unittest

from rov_state_bridge.health_monitor import (
    SEVERITY_CRIT,
    SEVERITY_OK,
    SEVERITY_WARN,
    build_health_monitor_status,
)
from rov_state_bridge.models import CommandResult, ControlState, NavStateView, SystemState, TelemetryFrameV2


class HealthMonitorTests(unittest.TestCase):
    def test_health_monitor_flags_failsafe_fault_and_reconnect(self) -> None:
        telemetry = TelemetryFrameV2(
            seq=7,
            stamp_ns=123,
            control=ControlState(failsafe_active=1),
            system=SystemState(
                session_state=3,
                health_state=3,
                fault_state=1,
                last_fault_code=6,
                nav_fault_code=12,
                nav_status_flags=(1 << 10),
                nav_age_ms=90,
                heartbeat_age_ms=12,
                nav_valid=0,
                nav_stale=1,
                nav_degraded=0,
            ),
            last_command_result=CommandResult(status=5, fault_code=12),
        )
        nav_view = NavStateView(age_ms=90, valid=0, stale=1, degraded=0, fault_code=12, status_flags=(1 << 10))

        status = build_health_monitor_status(telemetry, nav_view)

        self.assertEqual(status.severity, SEVERITY_CRIT)
        self.assertEqual(status.summary, 'failsafe_active')
        self.assertEqual(status.imu_reconnecting, 1)
        self.assertEqual(status.nav_stale, 1)
        self.assertEqual(status.command_fault_code, 12)

    def test_health_monitor_flags_device_mismatch(self) -> None:
        telemetry = TelemetryFrameV2(
            seq=9,
            stamp_ns=456,
            system=SystemState(
                session_state=2,
                health_state=2,
                fault_state=0,
                nav_fault_code=14,
                nav_status_flags=(1 << 9),
                nav_age_ms=35,
                heartbeat_age_ms=8,
                nav_valid=0,
                nav_stale=0,
                nav_degraded=0,
            ),
            last_command_result=CommandResult(status=0, fault_code=0),
        )

        status = build_health_monitor_status(telemetry)

        self.assertEqual(status.severity, SEVERITY_WARN)
        self.assertEqual(status.summary, 'device_mismatch')
        self.assertEqual(status.dvl_mismatch, 1)

    def test_health_monitor_reports_ok_when_runtime_is_healthy(self) -> None:
        telemetry = TelemetryFrameV2(
            seq=11,
            stamp_ns=789,
            control=ControlState(estop_latched=0, failsafe_active=0),
            system=SystemState(
                session_state=3,
                health_state=1,
                fault_state=0,
                nav_fault_code=0,
                nav_status_flags=(1 << 6) | (1 << 7),
                nav_age_ms=5,
                heartbeat_age_ms=5,
                nav_valid=1,
                nav_stale=0,
                nav_degraded=0,
            ),
            last_command_result=CommandResult(status=1, fault_code=0),
        )

        status = build_health_monitor_status(telemetry)

        self.assertEqual(status.severity, SEVERITY_OK)
        self.assertEqual(status.summary, 'ok')
        self.assertEqual(status.imu_online, 1)
        self.assertEqual(status.dvl_online, 1)


if __name__ == '__main__':
    unittest.main()
