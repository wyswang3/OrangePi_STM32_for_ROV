from __future__ import annotations

"""ctypes layouts mirroring the current runtime shared contracts.

These definitions are intentionally local to the bridge so the bridge can read
existing SHM blocks without linking against or modifying the core runtime.
They must stay semantically aligned with `shared/msg/*` and `shared/shm/*`.
"""

import ctypes


TELEMETRY_FRAME_V2_WIRE_VERSION = 2
NAV_STATE_PAYLOAD_VERSION = 2
NAV_STATE_VIEW_WIRE_VERSION = 2

TELEMETRY_MAGIC = (ord("T") << 24) | (ord("L") << 16) | (ord("M") << 8) | ord("2")
NAV_STATE_MAGIC = (ord("N") << 24) | (ord("A") << 16) | (ord("V") << 8) | ord("1")
NAV_VIEW_MAGIC = (ord("N") << 24) | (ord("V") << 16) | (ord("W") << 8) | ord("1")

TELEMETRY_LAYOUT_VERSION = 1
NAV_STATE_LAYOUT_VERSION = 1
NAV_VIEW_LAYOUT_VERSION = 1

TELEMETRY_CONTROLLER_NAME_MAX = 32
TELEMETRY_EVENT_HISTORY = 16

Float3 = ctypes.c_float * 3
Float6 = ctypes.c_float * 6
Float8 = ctypes.c_float * 8
Double3 = ctypes.c_double * 3
Char32 = ctypes.c_char * TELEMETRY_CONTROLLER_NAME_MAX


class MotorTestState(ctypes.Structure):
    _fields_ = [
        ("active", ctypes.c_uint8),
        ("motor_id", ctypes.c_uint8),
        ("mode", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("value", ctypes.c_float),
        ("remaining_ms", ctypes.c_uint32),
        ("cmd_id", ctypes.c_uint32),
    ]


class ControlIntentState(ctypes.Structure):
    _fields_ = [
        ("intent_id", ctypes.c_uint64),
        ("session_id", ctypes.c_uint64),
        ("cmd_seq", ctypes.c_uint64),
        ("stamp_ns", ctypes.c_uint64),
        ("ttl_ms", ctypes.c_uint32),
        ("source", ctypes.c_uint8),
        ("requested_mode", ctypes.c_uint8),
        ("arm_cmd", ctypes.c_uint8),
        ("estop_cmd", ctypes.c_uint8),
        ("valid", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8 * 3),
        ("dof_cmd", Float6),
        ("motor_test", MotorTestState),
    ]


class ControlState(ctypes.Structure):
    _fields_ = [
        ("active_mode", ctypes.c_uint8),
        ("armed", ctypes.c_uint8),
        ("estop_latched", ctypes.c_uint8),
        ("failsafe_active", ctypes.c_uint8),
        ("control_source", ctypes.c_uint8),
        ("intent_fresh", ctypes.c_uint8),
        ("controller_status", ctypes.c_uint8),
        ("motor_test_active", ctypes.c_uint8),
        ("active_intent_id", ctypes.c_uint64),
        ("controller_name", Char32),
        ("desired_controller", Char32),
        ("dof_cmd_applied", Float6),
        ("thruster_cmd", Float8),
        ("pwm_duty", Float8),
        ("consecutive_failures", ctypes.c_uint32),
        ("auto_fail_limit", ctypes.c_uint32),
    ]


class SystemState(ctypes.Structure):
    _fields_ = [
        ("session_state", ctypes.c_uint8),
        ("nav_state", ctypes.c_uint8),
        ("stm32_link_state", ctypes.c_uint8),
        ("pwm_link_state", ctypes.c_uint8),
        ("health_state", ctypes.c_uint8),
        ("degraded", ctypes.c_uint8),
        ("fault_state", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("last_fault_code", ctypes.c_uint16),
        ("nav_fault_code", ctypes.c_uint16),
        ("nav_status_flags", ctypes.c_uint16),
        ("reserved1", ctypes.c_uint16),
        ("heartbeat_age_ms", ctypes.c_uint32),
        ("nav_age_ms", ctypes.c_uint32),
        ("nav_valid", ctypes.c_uint8),
        ("nav_health", ctypes.c_uint8),
        ("nav_stale", ctypes.c_uint8),
        ("nav_degraded", ctypes.c_uint8),
        ("session_id", ctypes.c_uint64),
        ("stm32_last_rtt_ms", ctypes.c_float),
        ("pwm_tx_frames", ctypes.c_uint64),
        ("stm32_hb_tx", ctypes.c_uint64),
        ("stm32_hb_ack", ctypes.c_uint64),
    ]


class CommandResult(ctypes.Structure):
    _fields_ = [
        ("intent_id", ctypes.c_uint64),
        ("cmd_seq", ctypes.c_uint64),
        ("stamp_ns", ctypes.c_uint64),
        ("event_code", ctypes.c_uint16),
        ("fault_code", ctypes.c_uint16),
        ("status", ctypes.c_uint8),
        ("source", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8 * 6),
    ]


class EventRecord(ctypes.Structure):
    _fields_ = [
        ("seq", ctypes.c_uint64),
        ("stamp_ns", ctypes.c_uint64),
        ("event_code", ctypes.c_uint16),
        ("fault_code", ctypes.c_uint16),
        ("arg0", ctypes.c_int32),
        ("arg1", ctypes.c_int32),
    ]


class TelemetryFrameV2(ctypes.Structure):
    _fields_ = [
        ("version", ctypes.c_uint32),
        ("payload_size", ctypes.c_uint32),
        ("seq", ctypes.c_uint64),
        ("stamp_ns", ctypes.c_uint64),
        ("valid", ctypes.c_uint8),
        ("source", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint16),
        ("intent", ControlIntentState),
        ("control", ControlState),
        ("system", SystemState),
        ("attitude_rpy", Float3),
        ("position", Float3),
        ("velocity", Float3),
        ("depth_m", ctypes.c_float),
        ("last_command_result", CommandResult),
        ("last_event", EventRecord),
        ("event_count", ctypes.c_uint32),
        ("event_head", ctypes.c_uint32),
        ("events", EventRecord * TELEMETRY_EVENT_HISTORY),
    ]


class TelemetryFrameV2ShmHeader(ctypes.Structure):
    _fields_ = [
        ("seqlock", ctypes.c_uint64),
        ("mono_ns", ctypes.c_uint64),
        ("wall_ns", ctypes.c_uint64),
        ("magic", ctypes.c_uint32),
        ("layout_ver", ctypes.c_uint32),
        ("payload_ver", ctypes.c_uint32),
        ("payload_size", ctypes.c_uint32),
        ("payload_align", ctypes.c_uint32),
        ("reserved0", ctypes.c_uint32),
    ]


class TelemetryFrameV2ShmLayout(ctypes.Structure):
    _fields_ = [("hdr", TelemetryFrameV2ShmHeader), ("payload", TelemetryFrameV2)]


class NavState(ctypes.Structure):
    _fields_ = [
        ("t_ns", ctypes.c_uint64),
        ("pos", Double3),
        ("vel", Double3),
        ("rpy", Double3),
        ("depth", ctypes.c_double),
        ("omega_b", Double3),
        ("acc_b", Double3),
        ("age_ms", ctypes.c_uint32),
        ("valid", ctypes.c_uint8),
        ("stale", ctypes.c_uint8),
        ("degraded", ctypes.c_uint8),
        ("nav_state", ctypes.c_uint8),
        ("health", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("fault_code", ctypes.c_uint16),
        ("sensor_mask", ctypes.c_uint16),
        ("status_flags", ctypes.c_uint16),
        ("reserved1", ctypes.c_uint16),
    ]


class NavStateShmHeader(ctypes.Structure):
    _fields_ = [
        ("seq", ctypes.c_uint64),
        ("mono_ns", ctypes.c_uint64),
        ("wall_ns", ctypes.c_uint64),
        ("magic", ctypes.c_uint32),
        ("layout_ver", ctypes.c_uint32),
        ("payload_ver", ctypes.c_uint32),
        ("payload_size", ctypes.c_uint32),
        ("payload_align", ctypes.c_uint32),
        ("reserved0", ctypes.c_uint32),
    ]


class NavStateShmLayout(ctypes.Structure):
    _fields_ = [("hdr", NavStateShmHeader), ("payload", NavState)]


class NavStateView(ctypes.Structure):
    _fields_ = [
        ("version", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("stamp_ns", ctypes.c_uint64),
        ("mono_ns", ctypes.c_uint64),
        ("age_ms", ctypes.c_uint32),
        ("valid", ctypes.c_uint8),
        ("stale", ctypes.c_uint8),
        ("degraded", ctypes.c_uint8),
        ("nav_state", ctypes.c_uint8),
        ("health", ctypes.c_uint8),
        ("reserved0", ctypes.c_uint8),
        ("fault_code", ctypes.c_uint16),
        ("sensor_mask", ctypes.c_uint16),
        ("status_flags", ctypes.c_uint16),
        ("reserved1", ctypes.c_uint16),
        ("pos", Double3),
        ("vel", Double3),
        ("rpy", Double3),
        ("depth_m", ctypes.c_double),
        ("omega_b", Double3),
        ("acc_b", Double3),
        ("reserved2", ctypes.c_uint32),
    ]


class NavStateViewShmHeader(ctypes.Structure):
    _fields_ = [
        ("seq", ctypes.c_uint64),
        ("mono_ns", ctypes.c_uint64),
        ("wall_ns", ctypes.c_uint64),
        ("magic", ctypes.c_uint32),
        ("layout_ver", ctypes.c_uint32),
        ("payload_ver", ctypes.c_uint32),
        ("payload_size", ctypes.c_uint32),
        ("payload_align", ctypes.c_uint32),
        ("reserved0", ctypes.c_uint32),
    ]


class NavStateViewShmLayout(ctypes.Structure):
    _fields_ = [("hdr", NavStateViewShmHeader), ("payload", NavStateView)]


def c_string_to_text(raw: ctypes.Array[ctypes.c_char]) -> str:
    return bytes(raw).split(b"\x00", 1)[0].decode("utf-8", errors="ignore")
