# System State Machine

## Overview

V1 uses a product-oriented state model even though the codebase still spans multiple processes. The control kernel, gateway session, and UI/diagnostics all map into the following state set.

## Product States

- `Boot`
- `Disconnected`
- `Connected`
- `SessionReady`
- `Disarmed`
- `Armed`
- `Manual`
- `Auto`
- `MotorTest`
- `Failsafe`
- `EstopLatched`
- `Fault`

## Ownership

- `gateway/GcsSession` owns `Disconnected -> Connected -> SessionReady`
- `ControlGuard` owns `Disarmed/Armed/EstopLatched/Failsafe`
- `ControllerManager` owns `Manual/Auto`
- MotorTest is represented in V1 as a guarded execution override with explicit telemetry state; it is logically a product state even though legacy wire mode remains `Manual/Auto/Failsafe`

## Core Transition Rules

### Session plane

- `Boot -> Disconnected`: process started, no peer
- `Disconnected -> Connected`: UDP peer observed
- `Connected -> SessionReady`: handshake established
- `SessionReady -> Disconnected`: timeout/reset/session lost

### Safety plane

- any state -> `EstopLatched`: `estop=1`
- `EstopLatched -> Disarmed`: explicit clear-estop hold-to-clear while neutral
- `Disarmed -> Armed`: arm request accepted and no estop latch
- `Armed -> Disarmed`: disarm request or estop latch

### Control plane

- `Armed -> Manual`: default operational state
- `Manual -> Auto`: only when controller switch succeeds
- `Auto -> Failsafe`: stale/nav invalid/controller failure/mode rejection
- `Manual -> Failsafe`: stale while armed or explicit failsafe request
- `Failsafe -> Manual`: explicit mode recovery request after conditions are healthy

### MotorTest plane

- `Armed/Manual -> MotorTest`: valid motor-test command accepted
- `MotorTest -> Manual`: duration timeout, disarm, estop, or command cancel
- `MotorTest` is mutually exclusive with teleop/PID output

## V1 Enforced Safety Rules

### Manual stale auto-zero

- stale intent clears `teleop_dof/ref/ref_delta`
- armed manual stale triggers `FailsafeAction::kZeroOutput`
- an `IntentExpired` event is emitted and `FaultCode::kIntentStale` is recorded

### Estop latch

- estop immediately forces zero output and disarms the control path
- estop remains latched until clear-estop is held while inputs are neutral
- no controller output is allowed while latched

### MotorTest mutual exclusion

- normal thruster command is computed first, then overwritten by MotorTest
- all non-tested thrusters are forced to zero in MotorTest
- MotorTest is amplitude-limited and duration-limited in `ControlGuard`
- disarm/estop immediately aborts MotorTest

## Implementation Notes

V1 intentionally separates:

- product state semantics in telemetry/docs
- legacy wire compatibility in `StatusTelemetry`

That means `TelemetryFrameV2` is the authoritative engineering state model, while the old GCS wire status remains a compressed compatibility view.
