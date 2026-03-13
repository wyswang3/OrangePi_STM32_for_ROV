#include "control_core/telemetry_frame_builder.hpp"

#include "shared/msg/control_intent.hpp"

namespace rovctrl::control_core {

namespace {

std::uint8_t telemetry_runtime_nav_state(shared::msg::NavRunState state) noexcept
{
    using shared::msg::RuntimeNavState;

    switch (state) {
    case shared::msg::NavRunState::kOk:
        return static_cast<std::uint8_t>(RuntimeNavState::kOk);
    case shared::msg::NavRunState::kDegraded:
        return static_cast<std::uint8_t>(RuntimeNavState::kDegraded);
    case shared::msg::NavRunState::kUninitialized:
    case shared::msg::NavRunState::kAligning:
    case shared::msg::NavRunState::kInvalid:
    default:
        return static_cast<std::uint8_t>(RuntimeNavState::kInvalid);
    }
}

} // namespace

std::uint8_t telemetry_runtime_mode(ControlMode mode) noexcept
{
    using shared::msg::RuntimeControlMode;

    switch (mode) {
    case ControlMode::kManual:
        return static_cast<std::uint8_t>(RuntimeControlMode::kManual);
    case ControlMode::kAuto:
        return static_cast<std::uint8_t>(RuntimeControlMode::kAuto);
    case ControlMode::kFailsafe:
        return static_cast<std::uint8_t>(RuntimeControlMode::kFailsafe);
    case ControlMode::kNone:
        return static_cast<std::uint8_t>(RuntimeControlMode::kNone);
    case ControlMode::kUnknown:
    default:
        return static_cast<std::uint8_t>(RuntimeControlMode::kUnknown);
    }
}

std::uint8_t telemetry_control_source(std::uint8_t source_id) noexcept
{
    using shared::msg::ControlSource;
    using shared::msg::IntentSource;

    switch (static_cast<IntentSource>(source_id)) {
    case IntentSource::kGcs:
        return static_cast<std::uint8_t>(ControlSource::kGcs);
    case IntentSource::kLocal:
        return static_cast<std::uint8_t>(ControlSource::kLocal);
    case IntentSource::kAuto:
        return static_cast<std::uint8_t>(ControlSource::kAuto);
    case IntentSource::kTest:
        return static_cast<std::uint8_t>(ControlSource::kTest);
    case IntentSource::kUnknown:
    default:
        return static_cast<std::uint8_t>(ControlSource::kUnknown);
    }
}

bool telemetry_intent_has_payload(const ControlIntent& in) noexcept
{
    return in.request_exit ||
           in.has_estop_cmd ||
           in.has_arm_cmd ||
           in.has_mode_request ||
           in.has_teleop_dof ||
           in.has_ref ||
           in.has_ref_delta ||
           in.has_motor_test;
}

void fill_telemetry_frame_v2(shared::msg::TelemetryFrameV2& frame,
                             const TelemetryBuildInput&     input) noexcept
{
    frame.version = shared::msg::kTelemetryFrameV2WireVersion;
    frame.payload_size = static_cast<std::uint32_t>(sizeof(shared::msg::TelemetryFrameV2));
    frame.seq += 1;
    // Telemetry stamp_ns is the control-core publish stamp for this frame.
    // Nav sample time stays inside nav_snapshot->payload().stamp_ns/nav_age_ms.
    frame.stamp_ns = input.stamp_ns;
    frame.valid = 1;

    const auto& intent = input.applied_intent;

    frame.intent.intent_id =
        (intent.intent_id != 0) ? intent.intent_id : intent.cmd_seq;
    frame.intent.session_id = intent.session_id;
    frame.intent.cmd_seq = intent.cmd_seq;
    frame.intent.stamp_ns = intent.stamp_ns;
    frame.intent.ttl_ms = intent.ttl_ms;
    frame.intent.source = telemetry_control_source(intent.source_id);
    frame.intent.requested_mode = telemetry_runtime_mode(intent.mode_request);
    frame.intent.arm_cmd = intent.has_arm_cmd
        ? static_cast<std::uint8_t>(intent.arm ? 1 : 0)
        : 0;
    frame.intent.estop_cmd = intent.has_estop_cmd
        ? static_cast<std::uint8_t>(intent.estop ? 1 : 0)
        : 0;
    frame.intent.valid = intent.valid || telemetry_intent_has_payload(intent);
    frame.intent.dof_cmd[0] = static_cast<float>(intent.teleop_dof_cmd.surge);
    frame.intent.dof_cmd[1] = static_cast<float>(intent.teleop_dof_cmd.sway);
    frame.intent.dof_cmd[2] = static_cast<float>(intent.teleop_dof_cmd.heave);
    frame.intent.dof_cmd[3] = static_cast<float>(intent.teleop_dof_cmd.roll);
    frame.intent.dof_cmd[4] = static_cast<float>(intent.teleop_dof_cmd.pitch);
    frame.intent.dof_cmd[5] = static_cast<float>(intent.teleop_dof_cmd.yaw);
    frame.intent.motor_test.active =
        (intent.has_motor_test && intent.motor_test.enable) ? 1 : 0;
    frame.intent.motor_test.motor_id = intent.motor_test.motor_id;
    frame.intent.motor_test.mode = intent.motor_test.mode;
    frame.intent.motor_test.value = intent.motor_test.value;
    frame.intent.motor_test.remaining_ms = intent.motor_test.duration_ms;
    frame.intent.motor_test.cmd_id = intent.motor_test.cmd_id;

    frame.control.active_mode = telemetry_runtime_mode(input.active_mode);
    frame.control.armed = input.armed ? 1 : 0;
    frame.control.estop_latched = input.estop_latched ? 1 : 0;
    frame.control.failsafe_active = input.failsafe_active ? 1 : 0;
    frame.control.control_source = telemetry_control_source(intent.source_id);
    frame.control.intent_fresh = input.input_stale ? 0 : 1;
    frame.control.controller_status = static_cast<std::uint8_t>(
        input.controller_status.last_compute_ok
            ? shared::msg::ControllerStatus::kRunning
            : shared::msg::ControllerStatus::kFault);
    frame.control.motor_test_active =
        (intent.has_motor_test && intent.motor_test.enable) ? 1 : 0;
    frame.control.active_intent_id = frame.intent.intent_id;
    shared::msg::telemetry_write_cstr(frame.control.controller_name,
                                      shared::msg::kTelemetryControllerNameMax,
                                      input.controller_status.active_controller.c_str());
    shared::msg::telemetry_write_cstr(frame.control.desired_controller,
                                      shared::msg::kTelemetryControllerNameMax,
                                      input.controller_status.desired_controller.c_str());
    frame.control.dof_cmd_applied[0] = static_cast<float>(input.applied_reference.dof_cmd.surge);
    frame.control.dof_cmd_applied[1] = static_cast<float>(input.applied_reference.dof_cmd.sway);
    frame.control.dof_cmd_applied[2] = static_cast<float>(input.applied_reference.dof_cmd.heave);
    frame.control.dof_cmd_applied[3] = static_cast<float>(input.applied_reference.dof_cmd.roll);
    frame.control.dof_cmd_applied[4] = static_cast<float>(input.applied_reference.dof_cmd.pitch);
    frame.control.dof_cmd_applied[5] = static_cast<float>(input.applied_reference.dof_cmd.yaw);
    for (std::size_t i = 0; i < input.thruster_cmd.size(); ++i) {
        frame.control.thruster_cmd[i] = input.thruster_cmd[i];
        frame.control.pwm_duty[i] = input.pwm_duty[i];
    }
    frame.control.consecutive_failures = input.controller_status.consecutive_failures;
    frame.control.auto_fail_limit = input.auto_fail_limit;

    const auto& stats = input.transport_stats;
    frame.system.stm32_link_state = static_cast<std::uint8_t>(
        !input.have_transport_stats ? shared::msg::LinkState::kUnknown
        : (!stats.heartbeat_seen
            ? shared::msg::LinkState::kDown
            : (stats.heartbeat_age_ms > 1500
                ? shared::msg::LinkState::kDegraded
                : shared::msg::LinkState::kAlive)));
    frame.system.pwm_link_state = static_cast<std::uint8_t>(
        input.pwm_ok ? shared::msg::LinkState::kAlive : shared::msg::LinkState::kDown);
    frame.system.heartbeat_age_ms = input.have_transport_stats
        ? static_cast<std::uint32_t>(stats.heartbeat_age_ms)
        : 0;
    frame.system.stm32_last_rtt_ms = input.have_transport_stats
        ? stats.last_rtt_ms
        : -1.0f;
    frame.system.pwm_tx_frames = input.have_transport_stats ? stats.tx_pwm : 0;
    frame.system.stm32_hb_tx = input.have_transport_stats ? stats.tx_hb : 0;
    frame.system.stm32_hb_ack = input.have_transport_stats ? stats.rx_hb_ack : 0;

    bool nav_fault = false;
    if (input.nav_snapshot) {
        const auto& nav = input.nav_snapshot->payload();
        frame.system.nav_valid = nav.valid;
        frame.system.nav_health = static_cast<std::uint8_t>(nav.health);
        // nav_age_ms must already be the cumulative age seen by control after local SHM hop.
        frame.system.nav_age_ms = input.nav_age_ms;
        frame.system.nav_stale = nav.stale;
        frame.system.nav_degraded = nav.degraded;
        frame.system.nav_state = telemetry_runtime_nav_state(nav.nav_state);
        nav_fault = (nav.valid == 0) || (nav.stale != 0) ||
                    (nav.fault_code != shared::msg::NavFaultCode::kNone);
    } else {
        frame.system.nav_valid = 0;
        frame.system.nav_health = 0;
        frame.system.nav_age_ms = 0;
        frame.system.nav_stale = 1;
        frame.system.nav_degraded = 1;
        frame.system.nav_state =
            static_cast<std::uint8_t>(shared::msg::RuntimeNavState::kInvalid);
        nav_fault = true;
    }

    frame.system.session_state =
        static_cast<std::uint8_t>(shared::msg::SessionState::kUnknown);
    frame.system.health_state = static_cast<std::uint8_t>(
        (!input.pwm_ok ||
         frame.system.stm32_link_state == static_cast<std::uint8_t>(shared::msg::LinkState::kDown) ||
         nav_fault)
            ? shared::msg::HealthState::kFault
            : ((frame.system.nav_degraded || frame.control.failsafe_active)
                ? shared::msg::HealthState::kDegraded
                : shared::msg::HealthState::kOk));
    frame.system.degraded =
        (frame.system.health_state == static_cast<std::uint8_t>(shared::msg::HealthState::kDegraded)) ? 1 : 0;
    frame.system.fault_state =
        (frame.system.health_state == static_cast<std::uint8_t>(shared::msg::HealthState::kFault)) ? 1 : 0;
    frame.system.last_fault_code = static_cast<std::uint16_t>(input.last_fault_code);

    frame.attitude_rpy[0] = static_cast<float>(input.current_state.nav_rpy[0]);
    frame.attitude_rpy[1] = static_cast<float>(input.current_state.nav_rpy[1]);
    frame.attitude_rpy[2] = static_cast<float>(input.current_state.nav_rpy[2]);
    frame.position[0] = static_cast<float>(input.current_state.nav_pos_ned[0]);
    frame.position[1] = static_cast<float>(input.current_state.nav_pos_ned[1]);
    frame.position[2] = static_cast<float>(input.current_state.nav_pos_ned[2]);
    frame.velocity[0] = static_cast<float>(input.current_state.nav_vel_ned[0]);
    frame.velocity[1] = static_cast<float>(input.current_state.nav_vel_ned[1]);
    frame.velocity[2] = static_cast<float>(input.current_state.nav_vel_ned[2]);
    frame.depth_m = static_cast<float>(input.current_state.nav_depth);
}

} // namespace rovctrl::control_core
