#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "gateway/IPC/nav/nav_view_publisher_shm.hpp"
#include "gateway/IPC/nav/nav_view_builder.hpp"
#include "io/nav/nav_view_shm_source.hpp"

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

std::uint64_t now_mono_ns()
{
    using Clock = std::chrono::steady_clock;
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch()).count());
}

std::string unique_shm_name()
{
    const auto now_ns = now_mono_ns();
    return "/codex_nav_view_test_" + std::to_string(now_ns);
}

int test_invalid_snapshot_is_not_dropped()
{
    const std::string shm_name = unique_shm_name();

    comm_gcs::ipc::nav::NavViewPublisherShm pub;
    comm_gcs::ipc::nav::NavViewPublisherShm::Config pcfg{};
    pcfg.enable = true;
    pcfg.shm_name = shm_name;
    TEST_CHECK(pub.init(pcfg));

    shared::msg::NavStateView wire{};
    wire.valid = 0;
    wire.stale = 1;
    wire.degraded = 1;
    wire.nav_state = shared::msg::NavRunState::kInvalid;
    wire.health = shared::msg::NavHealth::INVALID;
    wire.fault_code = shared::msg::NavFaultCode::kNavViewStale;
    wire.stamp_ns = now_mono_ns() - 500'000'000ull;
    TEST_CHECK(pub.publish(wire));

    rovctrl::io::nav::NavViewShmSource src;
    rovctrl::io::nav::NavViewShmSource::Config scfg{};
    scfg.enable = true;
    scfg.shm_name = shm_name;
    scfg.max_age_ms = 1000;
    scfg.require_valid = false;
    TEST_CHECK(src.init(scfg));

    rovctrl::io::NavStateView out{};
    TEST_CHECK(src.read_latest(out));
    TEST_CHECK(out.has_snapshot());
    TEST_CHECK(out.payload().valid == 0);
    TEST_CHECK(out.payload().stale == 1);
    TEST_EQ(out.payload().nav_state, shared::msg::NavRunState::kInvalid);
    TEST_EQ(out.payload().fault_code, shared::msg::NavFaultCode::kNavViewStale);
    return 0;
}

int test_source_marks_overage_as_stale()
{
    const std::string shm_name = unique_shm_name();

    comm_gcs::ipc::nav::NavViewPublisherShm pub;
    comm_gcs::ipc::nav::NavViewPublisherShm::Config pcfg{};
    pcfg.enable = true;
    pcfg.shm_name = shm_name;
    TEST_CHECK(pub.init(pcfg));

    shared::msg::NavStateView wire{};
    wire.valid = 1;
    wire.stale = 0;
    wire.degraded = 0;
    wire.nav_state = shared::msg::NavRunState::kOk;
    wire.health = shared::msg::NavHealth::OK;
    wire.fault_code = shared::msg::NavFaultCode::kNone;
    wire.stamp_ns = now_mono_ns() - 500'000'000ull;
    wire.age_ms = 500;
    TEST_CHECK(pub.publish(wire));

    rovctrl::io::nav::NavViewShmSource src;
    rovctrl::io::nav::NavViewShmSource::Config scfg{};
    scfg.enable = true;
    scfg.shm_name = shm_name;
    scfg.max_age_ms = 100;
    TEST_CHECK(src.init(scfg));

    rovctrl::io::NavStateView out{};
    TEST_CHECK(src.read_latest(out));
    TEST_CHECK(out.payload().valid == 0);
    TEST_CHECK(out.payload().stale == 1);
    TEST_EQ(out.payload().nav_state, shared::msg::NavRunState::kInvalid);
    TEST_EQ(out.payload().health, shared::msg::NavHealth::INVALID);
    TEST_EQ(out.payload().fault_code, shared::msg::NavFaultCode::kNavViewStale);
    TEST_CHECK(out.total_age_ms() >= 500);
    return 0;
}

int test_navstate_to_control_preserves_time_contract()
{
    const std::string shm_name = unique_shm_name();

    comm_gcs::ipc::nav::NavViewPublisherShm pub;
    comm_gcs::ipc::nav::NavViewPublisherShm::Config pcfg{};
    pcfg.enable = true;
    pcfg.shm_name = shm_name;
    TEST_CHECK(pub.init(pcfg));

    shared::msg::NavState nav{};
    nav.t_ns = now_mono_ns() - 200'000'000ull;
    nav.age_ms = 200;
    nav.valid = 1;
    nav.stale = 0;
    nav.degraded = 0;
    nav.nav_state = shared::msg::NavRunState::kOk;
    nav.health = shared::msg::NavHealth::OK;
    nav.fault_code = shared::msg::NavFaultCode::kNone;
    nav.pos[0] = 1.0;
    nav.vel[0] = 0.1;

    shared::msg::NavStateView wire = comm_gcs::ipc::nav::NavViewBuilder::build(nav);
    TEST_EQ(wire.stamp_ns, nav.t_ns);
    TEST_EQ(wire.age_ms, nav.age_ms);
    TEST_CHECK(pub.publish(wire));

    rovctrl::io::nav::NavViewShmSource src;
    rovctrl::io::nav::NavViewShmSource::Config scfg{};
    scfg.enable = true;
    scfg.shm_name = shm_name;
    scfg.max_age_ms = 1000;
    TEST_CHECK(src.init(scfg));

    rovctrl::io::NavStateView out{};
    TEST_CHECK(src.read_latest(out));
    TEST_CHECK(out.has_snapshot());
    TEST_EQ(out.payload().stamp_ns, nav.t_ns);
    TEST_EQ(out.payload().mono_ns, out.pub_mono_ns);
    TEST_CHECK(out.payload().age_ms >= nav.age_ms);
    TEST_EQ(out.total_age_ms(), out.payload().age_ms);
    TEST_CHECK(out.payload().valid == 1);
    return 0;
}

} // namespace

int main()
{
    int rc = 0;
    rc = test_invalid_snapshot_is_not_dropped();
    if (rc != 0) return rc;
    rc = test_source_marks_overage_as_stale();
    if (rc != 0) return rc;
    rc = test_navstate_to_control_preserves_time_contract();
    if (rc != 0) return rc;

    std::cout << "[test_nav_view_shm_source] all tests passed.\n";
    return 0;
}
