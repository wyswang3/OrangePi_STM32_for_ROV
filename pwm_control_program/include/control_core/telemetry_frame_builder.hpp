#pragma once

#include <array>
#include <cstdint>

#include "control_core/control_intent.hpp"
#include "control_core/control_mode.hpp"
#include "control_core/control_types.hpp"
#include "controllers/controller_manager.hpp"
#include "io/nav/nav_state_view.hpp"
#include "platform/pwm_client.hpp"
#include "shared/msg/telemetry_frame_v2.hpp"

namespace rovctrl::control_core {

struct TelemetryBuildInput final {
    std::uint64_t stamp_ns = 0;

    ControlIntent applied_intent{};
    ControlReference applied_reference{};
    ControlState current_state{};

    ControlMode active_mode = ControlMode::kUnknown;
    ControllerManagerStatus controller_status{};
    std::uint32_t auto_fail_limit = 0;

    bool armed = false;
    bool estop_latched = false;
    bool failsafe_active = false;
    bool input_stale = false;

    ThrusterArray thruster_cmd{};
    std::array<float, rovctrl::platform::kNumPwmChannels> pwm_duty{};

    bool pwm_ok = false;
    bool have_transport_stats = false;
    rovctrl::platform::PwmTransportStats transport_stats{};

    const rovctrl::io::NavStateView* nav_snapshot = nullptr;
    std::uint32_t nav_age_ms = 0;

    shared::msg::FaultCode last_fault_code = shared::msg::FaultCode::kNone;
};

std::uint8_t telemetry_runtime_mode(ControlMode mode) noexcept;
std::uint8_t telemetry_control_source(std::uint8_t source_id) noexcept;
bool telemetry_intent_has_payload(const ControlIntent& in) noexcept;

void fill_telemetry_frame_v2(shared::msg::TelemetryFrameV2& frame,
                             const TelemetryBuildInput&     input) noexcept;

} // namespace rovctrl::control_core
