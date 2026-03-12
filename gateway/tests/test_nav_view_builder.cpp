#include <cstdint>
#include <iostream>

#include "gateway/IPC/nav/nav_view_builder.hpp"

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
        auto _va = (a);                                                                           \
        auto _vb = (b);                                                                           \
        if (!((_va) == (_vb))) {                                                                  \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " EQ(" #a ", " #b ") failed. got=" << static_cast<int>(_va)              \
                      << " expect=" << static_cast<int>(_vb) << "\n";                             \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

int test_builder_zeroes_invalid_payload()
{
    shared::msg::NavState in{};
    in.t_ns = 100;
    in.pos[0] = 1.0;
    in.vel[0] = 2.0;
    in.valid = 0;
    in.stale = 1;
    in.nav_state = shared::msg::NavRunState::kInvalid;
    in.health = shared::msg::NavHealth::INVALID;
    in.fault_code = shared::msg::NavFaultCode::kImuStale;
    in.status_flags = shared::msg::NAV_FLAG_NONE;

    const auto out = comm_gcs::ipc::nav::NavViewBuilder::build(in);
    TEST_CHECK(out.valid == 0);
    TEST_CHECK(out.stale == 1);
    TEST_EQ(out.nav_state, shared::msg::NavRunState::kInvalid);
    TEST_EQ(out.fault_code, shared::msg::NavFaultCode::kImuStale);
    TEST_CHECK(out.flags == 0);
    TEST_CHECK(out.pos[0] == 0.0);
    TEST_CHECK(out.vel[0] == 0.0);
    return 0;
}

int test_builder_preserves_degraded_valid_semantics()
{
    shared::msg::NavState in{};
    in.t_ns = 123456789ull;
    in.pos[0] = 1.5;
    in.vel[1] = -0.2;
    in.rpy[2] = 0.3;
    in.depth = 4.2;
    in.valid = 1;
    in.stale = 0;
    in.degraded = 1;
    in.nav_state = shared::msg::NavRunState::kDegraded;
    in.health = shared::msg::NavHealth::DEGRADED;
    in.fault_code = shared::msg::NavFaultCode::kNone;
    in.sensor_mask = shared::msg::NAV_SENSOR_IMU;
    in.status_flags = shared::msg::NAV_FLAG_IMU_OK |
                      shared::msg::NAV_FLAG_ALIGN_DONE |
                      shared::msg::NAV_FLAG_ESKF_OK;

    const auto out = comm_gcs::ipc::nav::NavViewBuilder::build(in);
    TEST_CHECK(out.valid == 1);
    TEST_CHECK(out.degraded == 1);
    TEST_EQ(out.nav_state, shared::msg::NavRunState::kDegraded);
    TEST_EQ(out.health, shared::msg::NavHealth::DEGRADED);
    TEST_EQ(out.fault_code, shared::msg::NavFaultCode::kNone);
    TEST_CHECK(out.pos[0] == 1.5);
    TEST_CHECK(out.vel[1] == -0.2);
    TEST_CHECK(out.rpy[2] == 0.3);
    TEST_CHECK(out.depth_m == 4.2);
    TEST_CHECK(out.status_flags == in.status_flags);
    TEST_CHECK(out.sensor_mask == in.sensor_mask);
    TEST_CHECK(out.flags != 0);
    return 0;
}

} // namespace

int main()
{
    int rc = 0;
    rc = test_builder_zeroes_invalid_payload();
    if (rc != 0) return rc;
    rc = test_builder_preserves_degraded_valid_semantics();
    if (rc != 0) return rc;

    std::cout << "[test_nav_view_builder] all tests passed.\n";
    return 0;
}
