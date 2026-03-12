# V1 Acceptance Checklist

## Unit Tests

- [x] TTL metadata survives input arbitration
- [x] manual stale clears teleop output and triggers zero-output failsafe
- [x] estop latches immediately
- [x] MotorTest latches, times out, and clears
- [x] wire `Failsafe` maps into control-core `Failsafe`
- [x] session dispatch accepts `MOTOR_TEST`
- [x] runtime telemetry maps to legacy `StatusTelemetry`

## Integration Tests

Run with built binaries in `build/bin/`:

- [ ] `gcs_server` + `pwm_control_program` + `telemetry_dump`
  - verify `/rovctrl_telemetry_v2` updates at control-loop rate
- [ ] GCS `SET_DOF_CMD` stream
  - stop command source
  - verify manual output goes to zero after TTL timeout
- [ ] GCS `SET_MODE(Failsafe)`
  - verify control mode becomes failsafe and output is zeroed
- [ ] `SET_MODE(Auto)` when no auto controller exists
  - verify command result becomes failed/rejected
  - verify system does not pretend to be in auto
- [ ] `MOTOR_TEST`
  - verify only one thruster is active
  - verify other thrusters remain zero
  - verify timeout auto-zero works
- [ ] STM32 heartbeat visibility
  - verify `heartbeat_age_ms`, `stm32_hb_tx`, `stm32_hb_ack`, `stm32_last_rtt_ms` update

## Replay / Fault Injection

- [ ] nav stale/degraded injection
  - verify telemetry marks `nav_valid/nav_stale/nav_degraded`
- [ ] gcs session timeout
  - verify session reset
  - verify zero-intent publish
- [ ] shm stale input
  - verify no last-frame reuse
- [ ] pwm/stm32 link interruption
  - verify `LinkState` degrades/faults and events are visible

## Operator Diagnostics

- [ ] `telemetry_dump --once` prints current mode, arm, estop, failsafe, nav, pwm, stm32, command result
- [ ] `telemetry_dump` continuous mode reflects live thruster and PWM changes

## Pre-HIL / Pre-Water Checks

- [ ] alloc.yaml and teleop mixer validated
- [ ] estop latch/clear tested on bench
- [ ] disarm on startup verified
- [ ] all eight thrusters mapped correctly
- [ ] reverse flags verified
- [ ] manual stale timeout measured on bench
- [ ] heartbeat ACK present from STM32

## V1 Exit Criteria

- [ ] commands are traceable as `accepted/rejected/executed/expired/failed`
- [ ] no demo telemetry remains in the gateway telemetry path
- [ ] stale manual input cannot keep non-zero thrust alive
- [ ] `Failsafe` command path is real, not cosmetic
- [ ] `MotorTest` is protocol-complete and safety-bounded
- [ ] gateway session race is eliminated by synchronization
- [ ] control-loop telemetry is authoritative and consumable by future UI/ROS 2 bridges
