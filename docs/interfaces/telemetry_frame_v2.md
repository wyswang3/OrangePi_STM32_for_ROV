# TelemetryFrameV2 Interface

## Purpose

`TelemetryFrameV2` is the V1 authoritative runtime telemetry payload for:

- control-loop observability
- operator diagnostics
- future ROS 2 bridge publication
- gateway-to-legacy-telemetry adaptation

It is published from `pwm_control_program` into SHM:

- SHM name: `/rovctrl_telemetry_v2`
- payload type: `shared::msg::TelemetryFrameV2`
- layout header: `shared::shm::TelemetryFrameV2ShmLayout`

## Top-level Structure

`TelemetryFrameV2` contains:

- wire/version header: `version`, `payload_size`, `seq`, `stamp_ns`, `valid`, `source`
- `ControlIntentState intent`
- `ControlState control`
- `SystemState system`
- navigation snapshot: `attitude_rpy`, `position`, `velocity`, `depth_m`
- `CommandResult last_command_result`
- `EventRecord last_event`
- fixed-size event history ring `events[16]`

## ControlIntentState

V1 captures the currently applied/active intent view:

- `intent_id`
- `session_id`
- `cmd_seq`
- `stamp_ns`
- `ttl_ms`
- `source`
- `requested_mode`
- `arm_cmd`
- `estop_cmd`
- `valid`
- `dof_cmd[6]`
- `motor_test`

Current implementation note:

- `intent_id` defaults to `cmd_seq` in V1
- `session_id` remains `0` until a future intent-wire upgrade carries session ownership end-to-end

## ControlState

Contains the active control/runtime execution result:

- `active_mode`
- `armed`
- `estop_latched`
- `failsafe_active`
- `control_source`
- `active_intent_id`
- `intent_fresh`
- `controller_name`
- `desired_controller`
- `controller_status`
- `motor_test_active`
- `dof_cmd_applied[6]`
- `thruster_cmd[8]`
- `pwm_duty[8]`
- `consecutive_failures`
- `auto_fail_limit`

## SystemState

Contains runtime health and link status:

- `session_state`
- `nav_state`
- `stm32_link_state`
- `pwm_link_state`
- `health_state`
- `degraded`
- `fault_state`
- `last_fault_code`
- `heartbeat_age_ms`
- `nav_age_ms`
- `nav_valid`
- `nav_health`
- `nav_stale`
- `nav_degraded`
- `session_id`
- `stm32_last_rtt_ms`
- `pwm_tx_frames`
- `stm32_hb_tx`
- `stm32_hb_ack`

## Command and Event Tracking

`last_command_result` tracks:

- accepted
- rejected
- executed
- expired
- failed

`events[]` records the last 16 event edges, including:

- intent accepted/executed/expired/failed
- mode changes
- failsafe entered
- estop latched/cleared
- arm changed
- motor-test started/stopped/rejected

## Legacy Gateway Mapping

`gcs_server` subscribes to `TelemetryFrameV2` and converts it to legacy `StatusTelemetry`.

Mapped fields:

- session/link status from gateway session
- estop from `control.estop_latched`
- mode from `control.active_mode`
- active/desired controller names
- consecutive failures
- auto fail limit
- timestamp from `stamp_ns`

Not carried by old `StatusTelemetry`:

- thruster and pwm arrays
- command result status
- event history
- nav degraded/stale details
- heartbeat RTT and age

These remain available in `TelemetryFrameV2` for `telemetry_dump`, future UI, and ROS 2 bridges.
