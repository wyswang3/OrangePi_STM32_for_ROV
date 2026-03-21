from __future__ import annotations

"""Derived outer-plane health summary helpers.

These helpers consume the already-frozen mirror fields and produce an advisory
health summary for ROS2 diagnostics/UI consumers. The output is read-only and
must never be fed back into control or navigation authority paths.
"""

from typing import Any

from .models import HealthMonitorStatus

SEVERITY_UNKNOWN = 0
SEVERITY_OK = 1
SEVERITY_WARN = 2
SEVERITY_CRIT = 3

SESSION_CONNECTED = 2
SESSION_READY = 3

COMMAND_REJECTED = 2
COMMAND_EXPIRED = 4
COMMAND_FAILED = 5

NAV_FLAG_IMU_DEVICE_ONLINE = 1 << 6
NAV_FLAG_DVL_DEVICE_ONLINE = 1 << 7
NAV_FLAG_IMU_BIND_MISMATCH = 1 << 8
NAV_FLAG_DVL_BIND_MISMATCH = 1 << 9
NAV_FLAG_IMU_RECONNECTING = 1 << 10
NAV_FLAG_DVL_RECONNECTING = 1 << 11


def _field(value: Any, path: str, default: Any = 0) -> Any:
    current = value
    marker = object()
    for part in path.split('.'):
        if current is None:
            return default
        if isinstance(current, dict):
            current = current.get(part, marker)
        else:
            current = getattr(current, part, marker)
        if current is marker:
            return default
    return current


def _flag_has(flags: int, bit: int) -> int:
    return 1 if (int(flags) & int(bit)) != 0 else 0


def _summary_and_severity(*, estop_latched: int, failsafe_active: int, fault_state: int,
                          command_status: int, nav_valid: int, nav_stale: int,
                          nav_degraded: int, imu_mismatch: int, dvl_mismatch: int,
                          imu_reconnecting: int, dvl_reconnecting: int,
                          imu_online: int, dvl_online: int) -> tuple[str, int]:
    if estop_latched:
        return 'estop_latched', SEVERITY_CRIT
    if failsafe_active:
        return 'failsafe_active', SEVERITY_CRIT
    if fault_state:
        return 'system_fault_active', SEVERITY_CRIT
    if command_status in (COMMAND_REJECTED, COMMAND_EXPIRED, COMMAND_FAILED):
        return 'command_failed', SEVERITY_WARN
    if imu_mismatch or dvl_mismatch:
        return 'device_mismatch', SEVERITY_WARN
    if imu_reconnecting or dvl_reconnecting:
        return 'device_reconnecting', SEVERITY_WARN
    if not imu_online or not dvl_online:
        return 'device_offline', SEVERITY_WARN
    if nav_stale:
        return 'nav_stale', SEVERITY_WARN
    if not nav_valid:
        return 'nav_invalid', SEVERITY_WARN
    if nav_degraded:
        return 'nav_degraded', SEVERITY_WARN
    return 'ok', SEVERITY_OK


def _recommended_action(summary: str) -> str:
    return {
        'estop_latched': 'confirm surroundings are safe before clearing estop; export telemetry and fault context',
        'failsafe_active': 'keep Manual/Failsafe; inspect nav and link health, then export telemetry and timeline logs',
        'system_fault_active': 'inspect fault code and controller status; export telemetry and incident bundle',
        'command_failed': 'check mode or arm preconditions, then export telemetry and recent command history',
        'device_mismatch': 'check USB binding and device identity, then restart the navigation stack',
        'device_reconnecting': 'wait for reconnect to complete; if it does not recover, inspect USB power and cabling',
        'device_offline': 'check sensor power and cabling, then export nav_timing.bin and telemetry',
        'nav_stale': 'export nav_timing.bin and inspect sensor timing or replay before re-arming',
        'nav_invalid': 'check sensor readiness and alignment state before enabling closed-loop control',
        'nav_degraded': 'continue in Manual/Failsafe and inspect degraded sensors before switching modes',
        'ok': 'no recovery action required',
    }.get(summary, 'inspect telemetry and current diagnostic state before proceeding')


def build_health_monitor_status(telemetry: Any, nav_view: Any | None = None) -> HealthMonitorStatus:
    """Build an advisory health summary from read-only mirror data.

    Inputs:
    - `telemetry`: `/rov/telemetry` mirror payload or equivalent object
    - `nav_view`: optional `/rov/nav_view` mirror payload for fresher nav flags

    Failure semantics:
    - Missing fields fall back conservatively to zero/unknown values.
    - The result is advisory only and must never be written back into the core chain.
    """
    stamp_ns = int(_field(telemetry, 'stamp_ns', _field(nav_view, 'stamp_ns', 0)))
    telemetry_seq = int(_field(telemetry, 'seq', 0))

    session_state = int(_field(telemetry, 'system.session_state', 0))
    health_state = int(_field(telemetry, 'system.health_state', 0))
    fault_state = int(_field(telemetry, 'system.fault_state', 0))
    last_fault_code = int(_field(telemetry, 'system.last_fault_code', 0))
    command_status = int(_field(telemetry, 'last_command_result.status', 0))
    command_fault_code = int(_field(telemetry, 'last_command_result.fault_code', 0))

    nav_fault_code = int(_field(nav_view, 'fault_code', _field(telemetry, 'system.nav_fault_code', 0)))
    nav_status_flags = int(_field(nav_view, 'status_flags', _field(telemetry, 'system.nav_status_flags', 0)))
    nav_age_ms = int(_field(nav_view, 'age_ms', _field(telemetry, 'system.nav_age_ms', 0)))
    heartbeat_age_ms = int(_field(telemetry, 'system.heartbeat_age_ms', 0))
    nav_valid = int(_field(nav_view, 'valid', _field(telemetry, 'system.nav_valid', 0)))
    nav_stale = int(_field(nav_view, 'stale', _field(telemetry, 'system.nav_stale', 0)))
    nav_degraded = int(_field(nav_view, 'degraded', _field(telemetry, 'system.nav_degraded', 0)))

    estop_latched = int(_field(telemetry, 'control.estop_latched', 0))
    failsafe_active = int(_field(telemetry, 'control.failsafe_active', 0))

    imu_online = _flag_has(nav_status_flags, NAV_FLAG_IMU_DEVICE_ONLINE)
    dvl_online = _flag_has(nav_status_flags, NAV_FLAG_DVL_DEVICE_ONLINE)
    imu_mismatch = _flag_has(nav_status_flags, NAV_FLAG_IMU_BIND_MISMATCH)
    dvl_mismatch = _flag_has(nav_status_flags, NAV_FLAG_DVL_BIND_MISMATCH)
    imu_reconnecting = _flag_has(nav_status_flags, NAV_FLAG_IMU_RECONNECTING)
    dvl_reconnecting = _flag_has(nav_status_flags, NAV_FLAG_DVL_RECONNECTING)

    summary, severity = _summary_and_severity(
        estop_latched=estop_latched,
        failsafe_active=failsafe_active,
        fault_state=fault_state,
        command_status=command_status,
        nav_valid=nav_valid,
        nav_stale=nav_stale,
        nav_degraded=nav_degraded,
        imu_mismatch=imu_mismatch,
        dvl_mismatch=dvl_mismatch,
        imu_reconnecting=imu_reconnecting,
        dvl_reconnecting=dvl_reconnecting,
        imu_online=imu_online,
        dvl_online=dvl_online,
    )

    if session_state >= SESSION_READY and severity == SEVERITY_UNKNOWN:
        severity = SEVERITY_OK

    return HealthMonitorStatus(
        stamp_ns=stamp_ns,
        telemetry_seq=telemetry_seq,
        severity=severity,
        session_state=session_state,
        health_state=health_state,
        fault_state=fault_state,
        last_fault_code=last_fault_code,
        command_status=command_status,
        command_fault_code=command_fault_code,
        nav_fault_code=nav_fault_code,
        nav_status_flags=nav_status_flags,
        nav_age_ms=nav_age_ms,
        heartbeat_age_ms=heartbeat_age_ms,
        nav_valid=nav_valid,
        nav_stale=nav_stale,
        nav_degraded=nav_degraded,
        estop_latched=estop_latched,
        failsafe_active=failsafe_active,
        imu_online=imu_online,
        dvl_online=dvl_online,
        imu_reconnecting=imu_reconnecting,
        dvl_reconnecting=dvl_reconnecting,
        imu_mismatch=imu_mismatch,
        dvl_mismatch=dvl_mismatch,
        summary=summary,
        recommended_action=_recommended_action(summary),
    )
