#include <cstdint>
#include <iostream>
#include <limits>

#include "gateway/IPC/nav/nav_view_policy.hpp"

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
                      << " EQ(" #a ", " #b ") failed\n";                                          \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

using comm_gcs::ipc::nav::NavViewDaemonDecision;
using comm_gcs::ipc::nav::NavViewDaemonPolicyConfig;

shared::msg::NavStateView make_valid_view()
{
    shared::msg::NavStateView view{};
    view.version = shared::msg::kNavStateViewWireVersion;
    view.stamp_ns = 1'000'000'000ull;
    view.valid = 1;
    view.stale = 0;
    view.degraded = 0;
    view.nav_state = shared::msg::NavRunState::kOk;
    view.health = shared::msg::NavHealth::OK;
    view.fault_code = shared::msg::NavFaultCode::kNone;
    view.pos[0] = 1.0;
    view.vel[0] = 2.0;
    view.flags = shared::msg::kHasPosition | shared::msg::kHasVelocity;
    view.status_flags = shared::msg::NAV_FLAG_IMU_OK |
                        shared::msg::NAV_FLAG_ALIGN_DONE |
                        shared::msg::NAV_FLAG_ESKF_OK;
    return view;
}

int test_policy_passes_through_fresh_view()
{
    NavViewDaemonPolicyConfig cfg{};
    cfg.max_age_ms = 250;
    cfg.warmup_ms = 100;

    auto view = make_valid_view();
    const NavViewDaemonDecision out = comm_gcs::ipc::nav::evaluate_nav_view_publish(
        &view, 1'180'000'000ull, 0, 1'200'000'000ull, cfg, true);

    TEST_CHECK(out.publish);
    TEST_CHECK(!out.diagnostic_only);
    TEST_CHECK(!out.stale_triggered);
    TEST_EQ(out.out.valid, 1);
    TEST_EQ(out.out.stale, 0);
    TEST_EQ(out.out.mono_ns, 1'200'000'000ull);
    TEST_EQ(out.out.age_ms, 200u);
    TEST_EQ(out.out.pos[0], 1.0);
    return 0;
}

int test_policy_propagates_invalid_upstream_view()
{
    NavViewDaemonPolicyConfig cfg{};
    auto view = make_valid_view();
    view.valid = 0;
    view.stale = 1;
    view.nav_state = shared::msg::NavRunState::kInvalid;
    view.health = shared::msg::NavHealth::INVALID;
    view.fault_code = shared::msg::NavFaultCode::kImuStale;
    view.flags = 0;

    const NavViewDaemonDecision out = comm_gcs::ipc::nav::evaluate_nav_view_publish(
        &view, 1'010'000'000ull, 0, 1'030'000'000ull, cfg, true);

    TEST_CHECK(out.publish);
    TEST_CHECK(!out.diagnostic_only);
    TEST_EQ(out.out.valid, 0);
    TEST_EQ(out.out.stale, 1);
    TEST_EQ(out.out.fault_code, shared::msg::NavFaultCode::kImuStale);
    TEST_EQ(out.out.age_ms, 30u);
    return 0;
}

int test_policy_converts_stale_input_to_diagnostic_frame()
{
    NavViewDaemonPolicyConfig cfg{};
    cfg.max_age_ms = 50;
    cfg.warmup_ms = 0;

    auto view = make_valid_view();
    view.sensor_mask = shared::msg::NAV_SENSOR_IMU;

    const NavViewDaemonDecision out = comm_gcs::ipc::nav::evaluate_nav_view_publish(
        &view, 1'000'000'000ull, 0, 1'100'000'000ull, cfg, true);

    TEST_CHECK(out.publish);
    TEST_CHECK(out.diagnostic_only);
    TEST_CHECK(out.stale_triggered);
    TEST_EQ(out.out.valid, 0);
    TEST_EQ(out.out.stale, 1);
    TEST_EQ(out.out.degraded, 1);
    TEST_EQ(out.out.nav_state, shared::msg::NavRunState::kInvalid);
    TEST_EQ(out.out.health, shared::msg::NavHealth::INVALID);
    TEST_EQ(out.out.fault_code, shared::msg::NavFaultCode::kNavViewStale);
    TEST_EQ(out.out.flags, 0u);
    TEST_EQ(out.out.stamp_ns, view.stamp_ns);
    TEST_EQ(out.out.sensor_mask, shared::msg::NAV_SENSOR_IMU);
    TEST_EQ(out.out.age_ms, 100u);
    return 0;
}

int test_policy_respects_degrade_throttle_slot()
{
    NavViewDaemonPolicyConfig cfg{};
    cfg.max_age_ms = 50;
    cfg.warmup_ms = 0;

    auto view = make_valid_view();
    const NavViewDaemonDecision out = comm_gcs::ipc::nav::evaluate_nav_view_publish(
        &view, 1'000'000'000ull, 0, 1'100'000'000ull, cfg, false);

    TEST_CHECK(!out.publish);
    TEST_CHECK(out.diagnostic_only);
    TEST_CHECK(out.stale_triggered);
    return 0;
}

int test_policy_no_nav_yet_publishes_explicit_invalid()
{
    NavViewDaemonPolicyConfig cfg{};
    cfg.warmup_ms = 0;

    const NavViewDaemonDecision out = comm_gcs::ipc::nav::evaluate_nav_view_publish(
        nullptr, 0, 0, 1'000'000'000ull, cfg, true);

    TEST_CHECK(out.publish);
    TEST_CHECK(out.no_nav_yet);
    TEST_EQ(out.out.valid, 0);
    TEST_EQ(out.out.stale, 1);
    TEST_EQ(out.out.fault_code, shared::msg::NavFaultCode::kNavViewStale);
    TEST_EQ(out.out.age_ms, std::numeric_limits<std::uint32_t>::max());
    return 0;
}

} // namespace

int main()
{
    int rc = 0;
    rc = test_policy_passes_through_fresh_view();
    if (rc != 0) return rc;
    rc = test_policy_propagates_invalid_upstream_view();
    if (rc != 0) return rc;
    rc = test_policy_converts_stale_input_to_diagnostic_frame();
    if (rc != 0) return rc;
    rc = test_policy_respects_degrade_throttle_slot();
    if (rc != 0) return rc;
    rc = test_policy_no_nav_yet_publishes_explicit_invalid();
    if (rc != 0) return rc;

    std::cout << "[test_nav_view_policy] all tests passed.\n";
    return 0;
}
