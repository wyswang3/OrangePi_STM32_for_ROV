#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <string>
#include <unistd.h>

#include "io/log/telemetry_timeline_logger.hpp"
#include "shared/msg/nav_state.hpp"

namespace {

#define TEST_CHECK(cond)                                                                          \
    do {                                                                                          \
        if (!(cond)) {                                                                            \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " CHECK(" #cond ") failed\n";                                            \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

#define TEST_EQ(a, b)                                                                             \
    do {                                                                                          \
        const auto _va = (a);                                                                     \
        const auto _vb = (b);                                                                     \
        if (!((_va) == (_vb))) {                                                                  \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " EQ(" #a ", " #b ") failed\n";                                          \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

std::size_t count_lines(const std::filesystem::path& path)
{
    std::ifstream in(path);
    std::size_t lines = 0;
    std::string line;
    while (std::getline(in, line)) {
        ++lines;
    }
    return lines;
}

std::string second_line(const std::filesystem::path& path)
{
    std::ifstream in(path);
    std::string line;
    std::getline(in, line);
    std::getline(in, line);
    return line;
}

int test_logger_writes_timeline_and_deduplicated_events()
{
    const auto base_ns = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    const char* keep_root_env = std::getenv("UWSYS_KEEP_TELEM_LOG_ROOT");
    const auto root = keep_root_env != nullptr
        ? std::filesystem::path(keep_root_env)
        : (std::filesystem::temp_directory_path() /
           ("telemetry_timeline_logger_" + std::to_string(static_cast<long long>(::getpid()))));
    std::filesystem::remove_all(root);

    rovctrl::io::TelemetryTimelineLogger logger;
    TEST_CHECK(logger.init(root.string(), "telemetry"));

    shared::msg::TelemetryFrameV2 frame{};
    frame.seq = 1;
    frame.stamp_ns = base_ns + 100'000'000ull;
    frame.control.active_mode = static_cast<std::uint8_t>(shared::msg::RuntimeControlMode::kAuto);
    frame.control.failsafe_active = 1;
    frame.system.nav_valid = 0;
    frame.system.nav_stale = 1;
    frame.system.nav_degraded = 1;
    frame.system.nav_state = static_cast<std::uint8_t>(shared::msg::RuntimeNavState::kInvalid);
    frame.system.nav_fault_code =
        static_cast<std::uint16_t>(shared::msg::NavFaultCode::kImuDisconnected);
    frame.system.nav_status_flags =
        shared::msg::NAV_FLAG_IMU_RECONNECTING |
        shared::msg::NAV_FLAG_DVL_BIND_MISMATCH;
    frame.last_event.seq = 10;
    frame.last_event.stamp_ns = base_ns + 101'000'000ull;
    frame.last_event.event_code = static_cast<std::uint16_t>(shared::msg::EventCode::kModeChanged);
    frame.last_command_result.cmd_seq = 5;
    frame.last_command_result.intent_id = 5;
    frame.last_command_result.stamp_ns = base_ns + 102'000'000ull;
    frame.last_command_result.status =
        static_cast<std::uint8_t>(shared::msg::CommandResultCode::kFailed);
    frame.last_command_result.fault_code =
        static_cast<std::uint16_t>(shared::msg::FaultCode::kNavUntrusted);
    logger.log_frame(frame);

    frame.seq = 2;
    frame.stamp_ns = base_ns + 110'000'000ull;
    logger.log_frame(frame);

    frame.seq = 3;
    frame.stamp_ns = base_ns + 120'000'000ull;
    frame.last_event.seq = 11;
    frame.last_event.stamp_ns = base_ns + 121'000'000ull;
    frame.last_event.event_code = static_cast<std::uint16_t>(shared::msg::EventCode::kIntentExecuted);
    frame.last_command_result.cmd_seq = 6;
    frame.last_command_result.intent_id = 6;
    frame.last_command_result.stamp_ns = base_ns + 122'000'000ull;
    frame.last_command_result.status =
        static_cast<std::uint8_t>(shared::msg::CommandResultCode::kExecuted);
    logger.log_frame(frame);
    logger.close();

    std::filesystem::path timeline_file;
    std::filesystem::path events_file;
    for (const auto& entry : std::filesystem::directory_iterator(root)) {
        const auto name = entry.path().filename().string();
        if (name.find("_timeline_") != std::string::npos) {
            timeline_file = entry.path();
        } else if (name.find("_events_") != std::string::npos) {
            events_file = entry.path();
        }
    }

    TEST_CHECK(!timeline_file.empty());
    TEST_CHECK(!events_file.empty());
    TEST_EQ(count_lines(timeline_file), static_cast<std::size_t>(4));
    TEST_EQ(count_lines(events_file), static_cast<std::size_t>(5));
    const auto row = second_line(timeline_file);
    TEST_CHECK(row.find(",12,1536,") != std::string::npos);

    if (keep_root_env == nullptr) {
        std::filesystem::remove_all(root);
    } else {
        std::cout << "[test_telemetry_timeline_logger] kept logs at " << root << "\n";
    }
    return 0;
}

} // namespace

int main()
{
    const int rc = test_logger_writes_timeline_and_deduplicated_events();
    if (rc != 0) {
        return rc;
    }

    std::cout << "[test_telemetry_timeline_logger] all tests passed.\n";
    return 0;
}
