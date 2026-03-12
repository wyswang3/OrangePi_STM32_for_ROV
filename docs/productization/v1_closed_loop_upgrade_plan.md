# V1 Closed-Loop Upgrade Plan

## Current Problems

The pre-upgrade stack was a research-oriented multi-process prototype with clear module boundaries but several broken control loops:

- Manual command TTL did not really take effect once GCS/SHM stopped updating. `MultiInputProvider` rewrote `stamp_ns/cmd_seq/ttl_ms` every cycle, so stale detection could never converge correctly.
- `Failsafe` and `Auto` were inconsistent across protocol, gateway, and control core. GCS `Failsafe` did not reach the control core, and `Auto` could appear switched while the controller manager had no auto controller.
- Telemetry returned by `gcs_server` was demo data assembled inside the session process, not authoritative runtime state from `pwm_control_program`.
- `gcs_server` accessed one `GcsSession` instance from both the UDP callback thread and the telemetry thread without synchronization.
- `MotorTest` existed as a partial concept but the protocol/session/control chain was not closed.

## V1 Scope

V1 upgrades the project into an engineering control system with:

- real command freshness and manual stale auto-zero
- real failsafe propagation from GCS to control core
- authoritative runtime telemetry from the control loop
- session concurrency protection in gateway
- a complete MotorTest protocol/session/control override path
- a stable `TelemetryFrameV2` and fault/event model for UI, diagnostics, and future ROS 2 bridges
- a minimal operator-facing diagnostic tool: `telemetry_dump`
- unit/integration smoke coverage for TTL, failsafe mapping, MotorTest dispatch, and telemetry mapping

## Non-Goals

The following are intentionally deferred beyond V1:

- full MPC/RL/autonomy stack
- replacing the PWM/STM32 execution kernel with ROS 2
- converting the control core into a ROS-native node graph
- full web UI or operator station frontend
- full merged gateway+control external telemetry protocol redesign

## Phased Plan

### P0: Closed-loop fundamentals

Implemented in this upgrade:

- preserve upstream intent timing metadata through input arbitration
- make `ControlGuard` zero manual outputs when stale
- propagate `Failsafe` from wire protocol to control mode
- make `ControllerManager::set_mode()` fail atomically
- publish authoritative `TelemetryFrameV2` from the control loop to SHM
- subscribe runtime telemetry in `gcs_server` and map it to legacy `StatusTelemetry`
- add explicit mutex protection around `GcsSession`

### P1: Execution and observability

Implemented in this upgrade:

- add `FaultCode`, `EventCode`, `CommandResultCode`
- add `SystemState`, `ControlState`, `ControlIntentState`, `TelemetryFrameV2`
- add STM32 heartbeat polling and link stats exposure in `PwmClient`
- close the `MotorTest` protocol/session/control path
- add `telemetry_dump` for minimal operator diagnostics

### P2: Controller framework expansion

Planned next:

- register PID-based controllers in `ControllerManager`
- implement `DepthHoldPidController`
- implement `HeadingHoldPidController`
- implement `DepthHeadingPidController`
- keep `VelocityHold` as reserved interface
- move controller selection and parameterization into stable config files

### P3: ROS 2 bridge

Planned next:

- publish `TelemetryFrameV2` and navigation state into ROS 2 topics
- expose MotorTest as ROS 2 action/service
- add health monitor and diagnostics aggregator
- bridge operator UI backend to ROS 2 while keeping PWM/STM32 safety local

## Why Gateway Uses a Mutex in V1

V1 keeps the existing UDP callback thread plus telemetry thread and adds explicit locking around `GcsSession`.

Reason:

- smallest regression surface
- fixes the real race immediately
- avoids re-architecting the gateway event model during the same release that changes safety and telemetry semantics

Single-thread event-loop refactoring remains a valid V2/V3 hardening task, but it is not required to close the current safety holes.

## ROS 2 Boundary

ROS 2 is explicitly outside the final safety decision path.

- `pwm_control_program` remains the deterministic control and actuation kernel
- `gateway` remains protocol/session/SHM bridge logic
- ROS 2 is allowed for telemetry, logging, health monitoring, tooling, MotorTest orchestration, and UI backend integration
- ROS 2 must not own the final estop decision or final PWM output arbitration
