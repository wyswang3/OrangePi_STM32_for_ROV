#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

#include "io/log/telemetry_timeline_logger.hpp"

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

int test_logger_writes_timeline_and_deduplicated_events()
{
    const auto root =
        std::filesystem::temp_directory_path() /
        ("telemetry_timeline_logger_" + std::to_string(static_cast<long long>(::getpid())));
    std::filesystem::remove_all(root);

    rovctrl::io::TelemetryTimelineLogger logger;
    TEST_CHECK(logger.init(root.string(), "telemetry"));

    shared::msg::TelemetryFrameV2 frame{};
    frame.seq = 1;
    frame.stamp_ns = 100;
    frame.control.active_mode = static_cast<std::uint8_t>(shared::msg::RuntimeControlMode::kAuto);
    frame.system.nav_valid = 1;
    frame.system.nav_state = static_cast<std::uint8_t>(shared::msg::RuntimeNavState::kOk);
    frame.last_event.seq = 10;
    frame.last_event.stamp_ns = 101;
    frame.last_event.event_code = static_cast<std::uint16_t>(shared::msg::EventCode::kModeChanged);
    frame.last_command_result.cmd_seq = 5;
    frame.last_command_result.intent_id = 5;
    frame.last_command_result.stamp_ns = 102;
    frame.last_command_result.status =
        static_cast<std::uint8_t>(shared::msg::CommandResultCode::kAccepted);
    logger.log_frame(frame);

    frame.seq = 2;
    frame.stamp_ns = 110;
    logger.log_frame(frame);

    frame.seq = 3;
    frame.stamp_ns = 120;
    frame.last_event.seq = 11;
    frame.last_event.stamp_ns = 121;
    frame.last_event.event_code = static_cast<std::uint16_t>(shared::msg::EventCode::kIntentExecuted);
    frame.last_command_result.cmd_seq = 6;
    frame.last_command_result.intent_id = 6;
    frame.last_command_result.stamp_ns = 122;
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

    std::filesystem::remove_all(root);
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
