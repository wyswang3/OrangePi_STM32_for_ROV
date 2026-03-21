from __future__ import annotations

"""Pure-Python mirror models used by the stage1 read-only bridge.

These dataclasses intentionally preserve the current shared-contract field names
and integer-encoded semantics. They are bridge-side mirror models only; they do
not become runtime authority and they are never written back into the core chain.
"""

from dataclasses import asdict, dataclass, field, fields, is_dataclass
from typing import Any


@dataclass(slots=True)
class MotorTestState:
    active: int = 0
    motor_id: int = 0
    mode: int = 0
    value: float = 0.0
    remaining_ms: int = 0
    cmd_id: int = 0


@dataclass(slots=True)
class ControlIntentState:
    intent_id: int = 0
    session_id: int = 0
    cmd_seq: int = 0
    stamp_ns: int = 0
    ttl_ms: int = 0
    source: int = 0
    requested_mode: int = 0
    arm_cmd: int = 0
    estop_cmd: int = 0
    valid: int = 0
    dof_cmd: list[float] = field(default_factory=list)
    motor_test: MotorTestState = field(default_factory=MotorTestState)


@dataclass(slots=True)
class ControlState:
    active_mode: int = 0
    armed: int = 0
    estop_latched: int = 0
    failsafe_active: int = 0
    control_source: int = 0
    intent_fresh: int = 0
    controller_status: int = 0
    motor_test_active: int = 0
    active_intent_id: int = 0
    controller_name: str = ""
    desired_controller: str = ""
    dof_cmd_applied: list[float] = field(default_factory=list)
    thruster_cmd: list[float] = field(default_factory=list)
    pwm_duty: list[float] = field(default_factory=list)
    consecutive_failures: int = 0
    auto_fail_limit: int = 0


@dataclass(slots=True)
class SystemState:
    session_state: int = 0
    nav_state: int = 0
    stm32_link_state: int = 0
    pwm_link_state: int = 0
    health_state: int = 0
    degraded: int = 0
    fault_state: int = 0
    last_fault_code: int = 0
    nav_fault_code: int = 0
    nav_status_flags: int = 0
    heartbeat_age_ms: int = 0
    nav_age_ms: int = 0
    nav_valid: int = 0
    nav_health: int = 0
    nav_stale: int = 0
    nav_degraded: int = 0
    session_id: int = 0
    stm32_last_rtt_ms: float = -1.0
    pwm_tx_frames: int = 0
    stm32_hb_tx: int = 0
    stm32_hb_ack: int = 0


@dataclass(slots=True)
class CommandResult:
    intent_id: int = 0
    cmd_seq: int = 0
    stamp_ns: int = 0
    event_code: int = 0
    fault_code: int = 0
    status: int = 0
    source: int = 0


@dataclass(slots=True)
class EventRecord:
    seq: int = 0
    stamp_ns: int = 0
    event_code: int = 0
    fault_code: int = 0
    arg0: int = 0
    arg1: int = 0


@dataclass(slots=True)
class TelemetryFrameV2:
    version: int = 0
    payload_size: int = 0
    seq: int = 0
    stamp_ns: int = 0
    valid: int = 0
    source: int = 0
    intent: ControlIntentState = field(default_factory=ControlIntentState)
    control: ControlState = field(default_factory=ControlState)
    system: SystemState = field(default_factory=SystemState)
    attitude_rpy: list[float] = field(default_factory=list)
    position: list[float] = field(default_factory=list)
    velocity: list[float] = field(default_factory=list)
    depth_m: float = 0.0
    last_command_result: CommandResult = field(default_factory=CommandResult)
    last_event: EventRecord = field(default_factory=EventRecord)
    event_count: int = 0
    event_head: int = 0
    events: list[EventRecord] = field(default_factory=list)


@dataclass(slots=True)
class NavStateView:
    version: int = 0
    flags: int = 0
    stamp_ns: int = 0
    mono_ns: int = 0
    age_ms: int = 0
    valid: int = 0
    stale: int = 0
    degraded: int = 0
    nav_state: int = 0
    health: int = 0
    fault_code: int = 0
    sensor_mask: int = 0
    status_flags: int = 0
    pos: list[float] = field(default_factory=list)
    vel: list[float] = field(default_factory=list)
    rpy: list[float] = field(default_factory=list)
    depth_m: float = 0.0
    omega_b: list[float] = field(default_factory=list)
    acc_b: list[float] = field(default_factory=list)


@dataclass(slots=True)
class NavState:
    t_ns: int = 0
    pos: list[float] = field(default_factory=list)
    vel: list[float] = field(default_factory=list)
    rpy: list[float] = field(default_factory=list)
    depth: float = 0.0
    omega_b: list[float] = field(default_factory=list)
    acc_b: list[float] = field(default_factory=list)
    age_ms: int = 0
    valid: int = 0
    stale: int = 0
    degraded: int = 0
    nav_state: int = 0
    health: int = 0
    fault_code: int = 0
    sensor_mask: int = 0
    status_flags: int = 0


@dataclass(slots=True)
class HealthSummary:
    stamp_ns: int = 0
    telemetry_seq: int = 0
    session_state: int = 0
    health_state: int = 0
    fault_state: int = 0
    last_fault_code: int = 0
    nav_state: int = 0
    nav_valid: int = 0
    nav_stale: int = 0
    nav_degraded: int = 0
    nav_fault_code: int = 0
    nav_status_flags: int = 0
    nav_age_ms: int = 0
    heartbeat_age_ms: int = 0


def to_jsonable(value: Any) -> Any:
    """Convert bridge dataclasses into JSON-serialisable plain values."""
    if is_dataclass(value):
        return {f.name: to_jsonable(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, list):
        return [to_jsonable(item) for item in value]
    return value


def to_dict(value: Any) -> dict[str, Any]:
    if is_dataclass(value):
        return asdict(value)
    raise TypeError(f"Expected dataclass value, got: {type(value)!r}")
