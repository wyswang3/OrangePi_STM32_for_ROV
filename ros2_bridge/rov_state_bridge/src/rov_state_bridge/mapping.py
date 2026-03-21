from __future__ import annotations

"""Mirror mapping helpers.

The bridge is intentionally read-only. These functions preserve the current
shared-contract field names and integer semantics when copying into bridge-side
mirror models. No safety, stale, or age logic is recomputed here.
"""

from . import layouts
from .models import (
    CommandResult,
    ControlIntentState,
    ControlState,
    EventRecord,
    HealthSummary,
    MotorTestState,
    NavState,
    NavStateView,
    SystemState,
    TelemetryFrameV2,
)


def _float_list(values) -> list[float]:
    return [float(v) for v in values]


def _event_record(value: layouts.EventRecord) -> EventRecord:
    return EventRecord(
        seq=int(value.seq),
        stamp_ns=int(value.stamp_ns),
        event_code=int(value.event_code),
        fault_code=int(value.fault_code),
        arg0=int(value.arg0),
        arg1=int(value.arg1),
    )


def _command_result(value: layouts.CommandResult) -> CommandResult:
    return CommandResult(
        intent_id=int(value.intent_id),
        cmd_seq=int(value.cmd_seq),
        stamp_ns=int(value.stamp_ns),
        event_code=int(value.event_code),
        fault_code=int(value.fault_code),
        status=int(value.status),
        source=int(value.source),
    )


def map_telemetry_frame(value: layouts.TelemetryFrameV2) -> TelemetryFrameV2:
    """Map the shared telemetry payload into a read-only mirror model."""
    return TelemetryFrameV2(
        version=int(value.version),
        payload_size=int(value.payload_size),
        seq=int(value.seq),
        stamp_ns=int(value.stamp_ns),
        valid=int(value.valid),
        source=int(value.source),
        intent=ControlIntentState(
            intent_id=int(value.intent.intent_id),
            session_id=int(value.intent.session_id),
            cmd_seq=int(value.intent.cmd_seq),
            stamp_ns=int(value.intent.stamp_ns),
            ttl_ms=int(value.intent.ttl_ms),
            source=int(value.intent.source),
            requested_mode=int(value.intent.requested_mode),
            arm_cmd=int(value.intent.arm_cmd),
            estop_cmd=int(value.intent.estop_cmd),
            valid=int(value.intent.valid),
            dof_cmd=_float_list(value.intent.dof_cmd),
            motor_test=MotorTestState(
                active=int(value.intent.motor_test.active),
                motor_id=int(value.intent.motor_test.motor_id),
                mode=int(value.intent.motor_test.mode),
                value=float(value.intent.motor_test.value),
                remaining_ms=int(value.intent.motor_test.remaining_ms),
                cmd_id=int(value.intent.motor_test.cmd_id),
            ),
        ),
        control=ControlState(
            active_mode=int(value.control.active_mode),
            armed=int(value.control.armed),
            estop_latched=int(value.control.estop_latched),
            failsafe_active=int(value.control.failsafe_active),
            control_source=int(value.control.control_source),
            intent_fresh=int(value.control.intent_fresh),
            controller_status=int(value.control.controller_status),
            motor_test_active=int(value.control.motor_test_active),
            active_intent_id=int(value.control.active_intent_id),
            controller_name=layouts.c_string_to_text(value.control.controller_name),
            desired_controller=layouts.c_string_to_text(value.control.desired_controller),
            dof_cmd_applied=_float_list(value.control.dof_cmd_applied),
            thruster_cmd=_float_list(value.control.thruster_cmd),
            pwm_duty=_float_list(value.control.pwm_duty),
            consecutive_failures=int(value.control.consecutive_failures),
            auto_fail_limit=int(value.control.auto_fail_limit),
        ),
        system=SystemState(
            session_state=int(value.system.session_state),
            nav_state=int(value.system.nav_state),
            stm32_link_state=int(value.system.stm32_link_state),
            pwm_link_state=int(value.system.pwm_link_state),
            health_state=int(value.system.health_state),
            degraded=int(value.system.degraded),
            fault_state=int(value.system.fault_state),
            last_fault_code=int(value.system.last_fault_code),
            nav_fault_code=int(value.system.nav_fault_code),
            nav_status_flags=int(value.system.nav_status_flags),
            heartbeat_age_ms=int(value.system.heartbeat_age_ms),
            nav_age_ms=int(value.system.nav_age_ms),
            nav_valid=int(value.system.nav_valid),
            nav_health=int(value.system.nav_health),
            nav_stale=int(value.system.nav_stale),
            nav_degraded=int(value.system.nav_degraded),
            session_id=int(value.system.session_id),
            stm32_last_rtt_ms=float(value.system.stm32_last_rtt_ms),
            pwm_tx_frames=int(value.system.pwm_tx_frames),
            stm32_hb_tx=int(value.system.stm32_hb_tx),
            stm32_hb_ack=int(value.system.stm32_hb_ack),
        ),
        attitude_rpy=_float_list(value.attitude_rpy),
        position=_float_list(value.position),
        velocity=_float_list(value.velocity),
        depth_m=float(value.depth_m),
        last_command_result=_command_result(value.last_command_result),
        last_event=_event_record(value.last_event),
        event_count=int(value.event_count),
        event_head=int(value.event_head),
        events=[_event_record(evt) for evt in value.events],
    )


def map_nav_state_view(value: layouts.NavStateView) -> NavStateView:
    return NavStateView(
        version=int(value.version),
        flags=int(value.flags),
        stamp_ns=int(value.stamp_ns),
        mono_ns=int(value.mono_ns),
        age_ms=int(value.age_ms),
        valid=int(value.valid),
        stale=int(value.stale),
        degraded=int(value.degraded),
        nav_state=int(value.nav_state),
        health=int(value.health),
        fault_code=int(value.fault_code),
        sensor_mask=int(value.sensor_mask),
        status_flags=int(value.status_flags),
        pos=[float(v) for v in value.pos],
        vel=[float(v) for v in value.vel],
        rpy=[float(v) for v in value.rpy],
        depth_m=float(value.depth_m),
        omega_b=[float(v) for v in value.omega_b],
        acc_b=[float(v) for v in value.acc_b],
    )


def map_nav_state(value: layouts.NavState) -> NavState:
    return NavState(
        t_ns=int(value.t_ns),
        pos=[float(v) for v in value.pos],
        vel=[float(v) for v in value.vel],
        rpy=[float(v) for v in value.rpy],
        depth=float(value.depth),
        omega_b=[float(v) for v in value.omega_b],
        acc_b=[float(v) for v in value.acc_b],
        age_ms=int(value.age_ms),
        valid=int(value.valid),
        stale=int(value.stale),
        degraded=int(value.degraded),
        nav_state=int(value.nav_state),
        health=int(value.health),
        fault_code=int(value.fault_code),
        sensor_mask=int(value.sensor_mask),
        status_flags=int(value.status_flags),
    )


def build_health_summary(telemetry: TelemetryFrameV2) -> HealthSummary:
    """Derive a compact health mirror without changing authority semantics."""
    system = telemetry.system
    return HealthSummary(
        stamp_ns=telemetry.stamp_ns,
        telemetry_seq=telemetry.seq,
        session_state=system.session_state,
        health_state=system.health_state,
        fault_state=system.fault_state,
        last_fault_code=system.last_fault_code,
        nav_state=system.nav_state,
        nav_valid=system.nav_valid,
        nav_stale=system.nav_stale,
        nav_degraded=system.nav_degraded,
        nav_fault_code=system.nav_fault_code,
        nav_status_flags=system.nav_status_flags,
        nav_age_ms=system.nav_age_ms,
        heartbeat_age_ms=system.heartbeat_age_ms,
    )
