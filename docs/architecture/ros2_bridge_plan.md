# ROS 2 Bridge Plan

## Boundary

ROS 2 is not the final safety authority.

- final estop arbitration remains in `ControlGuard`
- final thruster/PWM safety remains in `pwm_control_program`
- STM32 transport remains outside ROS 2

ROS 2 is used only for outer-plane functionality:

- state publication
- telemetry fan-out
- health monitoring
- MotorTest orchestration
- logs and replay
- operator UI backend

## Planned Packages

### `rov_msgs`

Message package for:

- `TelemetryFrameV2` mirror
- nav health/status
- system faults/events
- motor-test goal/result/feedback

### `rov_state_bridge`

Reads SHM and publishes:

- control state
- system state
- fault/events

Primary inputs:

- `/rovctrl_telemetry_v2`

### `rov_nav_bridge`

Reads nav SHM and publishes:

- raw/filtered nav state
- nav validity and degradation state

### `rov_telemetry_bridge`

Republishes `TelemetryFrameV2` to ROS 2 topics and rosbag pipeline.

### `rov_health_monitor`

Aggregates:

- session health
- nav stale/degraded
- pwm/stm32 link
- watchdog/fault counters

### `rov_motor_test_server`

Exposes MotorTest as ROS 2 action/service:

- validates request
- converts to control intent
- observes completion/timeout/fault

### `rov_operator_ui_backend`

WebSocket/HTTP bridge for UI.

Consumes:

- ROS 2 topics
- event history
- parameter endpoints

## Topic and Action Shape

V1 bridge priorities:

- `/rov/telemetry`
- `/rov/health`
- `/rov/nav_state`
- `/rov/events`
- `/rov/faults`
- `/rov/motor_test` action

## Deployment Order

1. keep current SHM contracts stable
2. add ROS 2 bridge processes without changing control kernel
3. verify bridge cannot block or stall `pwm_control_program`
4. only after that add UI/backend consumers

## Acceptance Rule

If ROS 2 crashes, stalls, or disconnects:

- control loop continues
- PWM/STM32 path continues under existing safety policy
- telemetry export may degrade, but control authority does not migrate into ROS 2
