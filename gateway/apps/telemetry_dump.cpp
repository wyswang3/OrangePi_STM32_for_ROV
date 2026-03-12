#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "gateway/IPC/state/telemetry_frame_v2_subscriber_shm.hpp"
#include "shared/msg/telemetry_frame_v2.hpp"

namespace {

std::atomic_bool g_stop{false};

void on_sigint(int)
{
    g_stop.store(true);
}

const char* health_str(std::uint8_t v)
{
    using shared::msg::HealthState;
    switch (static_cast<HealthState>(v)) {
    case HealthState::kOk: return "ok";
    case HealthState::kDegraded: return "degraded";
    case HealthState::kFault: return "fault";
    case HealthState::kUnknown:
    default: return "unknown";
    }
}

const char* link_str(std::uint8_t v)
{
    using shared::msg::LinkState;
    switch (static_cast<LinkState>(v)) {
    case LinkState::kAlive: return "alive";
    case LinkState::kDegraded: return "degraded";
    case LinkState::kDown: return "down";
    case LinkState::kUnknown:
    default: return "unknown";
    }
}

const char* mode_str(std::uint8_t v)
{
    using shared::msg::RuntimeControlMode;
    switch (static_cast<RuntimeControlMode>(v)) {
    case RuntimeControlMode::kManual: return "manual";
    case RuntimeControlMode::kAuto: return "auto";
    case RuntimeControlMode::kFailsafe: return "failsafe";
    case RuntimeControlMode::kNone: return "none";
    case RuntimeControlMode::kUnknown:
    default: return "unknown";
    }
}

const char* cmd_result_str(std::uint8_t v)
{
    using shared::msg::CommandResultCode;
    switch (static_cast<CommandResultCode>(v)) {
    case CommandResultCode::kAccepted: return "accepted";
    case CommandResultCode::kRejected: return "rejected";
    case CommandResultCode::kExecuted: return "executed";
    case CommandResultCode::kExpired: return "expired";
    case CommandResultCode::kFailed: return "failed";
    case CommandResultCode::kNone:
    default: return "none";
    }
}

void print_frame(const shared::msg::TelemetryFrameV2& f)
{
    std::cout << "seq=" << f.seq
              << " mode=" << mode_str(f.control.active_mode)
              << " armed=" << int(f.control.armed)
              << " estop=" << int(f.control.estop_latched)
              << " failsafe=" << int(f.control.failsafe_active)
              << " health=" << health_str(f.system.health_state)
              << " nav_valid=" << int(f.system.nav_valid)
              << " nav_age_ms=" << f.system.nav_age_ms
              << " stm32=" << link_str(f.system.stm32_link_state)
              << " pwm=" << link_str(f.system.pwm_link_state)
              << " hb_age_ms=" << f.system.heartbeat_age_ms
              << " controller=" << f.control.controller_name
              << " desired=" << f.control.desired_controller
              << "\n";

    std::cout << "pose_rpy=("
              << f.attitude_rpy[0] << ", "
              << f.attitude_rpy[1] << ", "
              << f.attitude_rpy[2] << ") depth=" << f.depth_m
              << " cmd_result=" << cmd_result_str(f.last_command_result.status)
              << " fault=" << f.last_command_result.fault_code
              << " event=" << f.last_event.event_code
              << "\n";

    std::cout << "thruster:";
    for (float v : f.control.thruster_cmd) {
        std::cout << " " << v;
    }
    std::cout << "\n";

    std::cout << "pwm_duty:";
    for (float v : f.control.pwm_duty) {
        std::cout << " " << v;
    }
    std::cout << "\n";
}

} // namespace

int main(int argc, char** argv)
{
    bool once = false;
    int interval_ms = 500;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--once") {
            once = true;
        } else if (arg == "--interval-ms" && i + 1 < argc) {
            interval_ms = std::stoi(argv[++i]);
        }
    }

    comm_gcs::ipc::state::TelemetryFrameV2SubscriberShm sub;
    comm_gcs::ipc::state::TelemetryFrameV2SubscriberShm::Config cfg{};
    cfg.enable = true;
    cfg.shm_name = "/rovctrl_telemetry_v2";
    cfg.lazy_init = true;

    if (!sub.init(cfg)) {
        std::cerr << "[telemetry_dump] subscriber init failed.\n";
        return 1;
    }

    std::signal(SIGINT, on_sigint);
    while (!g_stop.load()) {
        const auto frame = sub.poll();
        if (frame) {
            print_frame(*frame);
        } else {
            std::cout << "[telemetry_dump] no telemetry snapshot.\n";
        }

        if (once) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    }

    return 0;
}
