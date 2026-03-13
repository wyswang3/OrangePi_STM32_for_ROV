#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

#include "io/log/control_loop_logger.hpp"

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

int test_control_loop_logger_writes_nav_diagnostics()
{
    const char* keep_root_env = std::getenv("UWSYS_KEEP_CONTROL_LOG_ROOT");
    const auto root = keep_root_env != nullptr
        ? std::filesystem::path(keep_root_env)
        : (std::filesystem::temp_directory_path() /
           ("control_loop_logger_" + std::to_string(static_cast<long long>(::getpid()))));
    std::filesystem::remove_all(root);

    rovctrl::io::ControlLoopLogger logger;
    TEST_CHECK(logger.init(root.string(), "control_loop"));

    rovctrl::io::ControlEffect eff{};
    eff.surge = 0.1f;
    eff.yaw = -0.2f;
    eff.intent_age_ms = 42;

    rovctrl::io::ControlGuardOutput guard{};
    guard.armed = true;
    guard.failsafe = true;
    guard.effective_mode = 4;

    rovctrl::io::NavigationData nav{};
    nav.present = true;
    nav.valid = false;
    nav.stale = true;
    nav.degraded = true;
    nav.age_ms = 250;
    nav.nav_state = 4;
    nav.nav_health = 3;
    nav.fault_code = 8;
    nav.status_flags = 1536;

    logger.log_data(1.0, eff, guard, nav);
    logger.close();

    std::filesystem::path csv_file;
    for (const auto& entry : std::filesystem::directory_iterator(root)) {
        if (entry.path().extension() == ".csv") {
            csv_file = entry.path();
            break;
        }
    }

    TEST_CHECK(!csv_file.empty());
    TEST_EQ(count_lines(csv_file), static_cast<std::size_t>(2));
    const auto row = second_line(csv_file);
    TEST_CHECK(row.find(",1,0,4,0,1,0,") != std::string::npos);
    TEST_CHECK(row.find(",250,4,3,8,0,1536") != std::string::npos);

    if (keep_root_env == nullptr) {
        std::filesystem::remove_all(root);
    } else {
        std::cout << "[test_control_loop_logger] kept logs at " << root << "\n";
    }
    return 0;
}

} // namespace

int main()
{
    const int rc = test_control_loop_logger_writes_nav_diagnostics();
    if (rc != 0) {
        return rc;
    }

    std::cout << "[test_control_loop_logger] all tests passed.\n";
    return 0;
}
