#pragma once

#include "proto_gcs/gcs_protocol.hpp"
#include "shared/msg/telemetry_frame_v2.hpp"

namespace comm_gcs::telemetry {

inline std::uint8_t to_wire_mode(std::uint8_t runtime_mode) noexcept
{
    using shared::msg::RuntimeControlMode;
    using rovctrl::io::gcs::WireControlMode;

    switch (static_cast<RuntimeControlMode>(runtime_mode)) {
    case RuntimeControlMode::kManual:
        return static_cast<std::uint8_t>(WireControlMode::Manual);
    case RuntimeControlMode::kAuto:
        return static_cast<std::uint8_t>(WireControlMode::Auto);
    case RuntimeControlMode::kFailsafe:
        return static_cast<std::uint8_t>(WireControlMode::Failsafe);
    case RuntimeControlMode::kNone:
    case RuntimeControlMode::kUnknown:
    default:
        return static_cast<std::uint8_t>(WireControlMode::Unknown);
    }
}

inline rovctrl::io::gcs::StatusTelemetry build_status_telemetry(
    const shared::msg::TelemetryFrameV2& frame,
    bool session_established,
    bool link_alive) noexcept
{
    rovctrl::io::gcs::StatusTelemetry out{};
    out.session_established = session_established ? 1 : 0;
    out.link_alive = link_alive ? 1 : 0;
    out.estop = frame.control.estop_latched;
    out.armed = frame.control.armed;
    out.mode = to_wire_mode(frame.control.active_mode);
    out.failsafe_active = frame.control.failsafe_active;
    out.nav_valid = frame.system.nav_valid;
    out.nav_state = frame.system.nav_state;
    out.nav_stale = frame.system.nav_stale;
    out.nav_degraded = frame.system.nav_degraded;
    out.fault_state = frame.system.fault_state;
    out.health_state = frame.system.health_state;
    out.command_status = frame.last_command_result.status;
    out.last_fault_code = frame.system.last_fault_code;
    out.command_fault_code = frame.last_command_result.fault_code;
    out.status_seq = static_cast<std::uint32_t>(frame.seq & 0xFFFFFFFFu);
    out.command_cmd_seq = frame.last_command_result.cmd_seq;
    rovctrl::io::gcs::write_cstr(out.active_controller,
                                 rovctrl::io::gcs::kCtrlNameMaxLen,
                                 frame.control.controller_name);
    rovctrl::io::gcs::write_cstr(out.desired_controller,
                                 rovctrl::io::gcs::kCtrlNameMaxLen,
                                 frame.control.desired_controller);
    out.consecutive_failures = frame.control.consecutive_failures;
    out.auto_fail_limit = frame.control.auto_fail_limit;
    out.t_ns = frame.stamp_ns;
    return out;
}

} // namespace comm_gcs::telemetry
