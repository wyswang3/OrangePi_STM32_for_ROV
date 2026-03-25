#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "controllers/controller_manager.hpp"
#include "controllers/manual_controller.hpp"
#include "controllers/pid_controller.hpp"
#include "control_core/control_guard.hpp"
#include "io/input/control_intent_wire_codec.hpp"
#include "io/input/multi_input_provider.hpp"

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
                      << " EQ(" #a ", " #b ") failed. got=" << _va                                 \
                      << " expect=" << _vb << "\n";                                               \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

#define TEST_NEAR(a, b, eps)                                                                      \
    do {                                                                                          \
        const auto _va = static_cast<double>(a);                                                  \
        const auto _vb = static_cast<double>(b);                                                  \
        const auto _ve = static_cast<double>(eps);                                                \
        if (std::fabs(_va - _vb) > _ve) {                                                         \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " NEAR(" #a ", " #b ", " #eps ") failed. got=" << _va                    \
                      << " expect=" << _vb << " eps=" << _ve << "\n";                             \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

namespace cc = rovctrl::control_core;
namespace io = rovctrl::io;

shared::msg::NavStateView make_auto_ready_nav(bool degraded = false)
{
    shared::msg::NavStateView nav{};
    nav.valid = 1;
    nav.stale = 0;
    nav.degraded = degraded ? 1 : 0;
    nav.nav_state = degraded
        ? shared::msg::NavRunState::kDegraded
        : shared::msg::NavRunState::kOk;
    nav.health = degraded
        ? shared::msg::NavHealth::DEGRADED
        : shared::msg::NavHealth::OK;
    nav.fault_code = shared::msg::NavFaultCode::kNone;
    nav.status_flags = shared::msg::NAV_FLAG_IMU_OK |
                       shared::msg::NAV_FLAG_ALIGN_DONE |
                       shared::msg::NAV_FLAG_ESKF_OK;
    nav.age_ms = 20;
    return nav;
}

class FakeProvider final : public io::IInputProvider {
public:
    explicit FakeProvider(cc::ControlIntent intent)
        : intent_(intent) {}

    bool init() override { return true; }

    bool poll(cc::ControlState&, cc::ControlIntent& out) override
    {
        out = intent_;
        return true;
    }

    void reset() override {}

private:
    cc::ControlIntent intent_{};
};

int test_multi_input_preserves_primary_metadata()
{
    cc::ControlIntent g{};
    g.intent_id = 42;
    g.cmd_seq = 42;
    g.stamp_ns = 123456789ull;
    g.ttl_ms = 180;
    g.source_id = static_cast<std::uint8_t>(shared::msg::IntentSource::kGcs);
    g.valid = true;
    g.has_teleop_dof = true;
    g.teleop_dof_cmd.surge = 0.7;
    g.has_motor_test = true;
    g.motor_test.enable = true;
    g.motor_test.motor_id = 3;
    g.motor_test.duration_ms = 300;

    io::MultiInputProvider provider(
        std::make_shared<FakeProvider>(cc::ControlIntent{}),
        std::make_shared<FakeProvider>(g));

    cc::ControlState state{};
    cc::ControlIntent out{};
    TEST_CHECK(provider.init());
    TEST_CHECK(provider.poll(state, out));

    TEST_EQ(out.intent_id, 42ull);
    TEST_EQ(out.cmd_seq, 42ull);
    TEST_EQ(out.stamp_ns, 123456789ull);
    TEST_EQ(out.ttl_ms, 180u);
    TEST_EQ(out.source_id, static_cast<std::uint8_t>(shared::msg::IntentSource::kGcs));
    TEST_CHECK(out.has_motor_test);
    TEST_EQ(out.motor_test.motor_id, 3);
    return 0;
}

int test_guard_stale_zeroes_manual_output()
{
    cc::ControlGuard guard(cc::ControlGuardConfig{});
    cc::ControlState state{};
    cc::ControlIntent arm{};
    arm.cmd_seq = 1;
    arm.stamp_ns = 1;
    arm.ttl_ms = 100;
    arm.valid = true;
    arm.has_arm_cmd = true;
    arm.arm = true;
    auto arm_res = guard.step(1, state, nullptr, arm);
    TEST_CHECK(arm_res.armed);

    cc::ControlIntent teleop{};
    teleop.intent_id = 2;
    teleop.cmd_seq = 2;
    teleop.stamp_ns = 2;
    teleop.ttl_ms = 100;
    teleop.valid = true;
    teleop.has_teleop_dof = true;
    teleop.teleop_dof_cmd.surge = 0.6;
    auto fresh = guard.step(2, state, nullptr, teleop);
    TEST_CHECK(!fresh.input_stale);
    TEST_CHECK(fresh.effective_intent.has_teleop_dof);

    auto stale = guard.step(250000000ull, state, nullptr, teleop);
    TEST_CHECK(stale.input_stale);
    TEST_CHECK(!stale.effective_intent.has_teleop_dof);
    TEST_EQ(static_cast<int>(stale.failsafe),
            static_cast<int>(cc::FailsafeAction::kZeroOutput));
    return 0;
}

int test_guard_motor_test_latch_and_expire()
{
    cc::ControlGuard guard(cc::ControlGuardConfig{});
    cc::ControlState state{};

    cc::ControlIntent arm{};
    arm.cmd_seq = 1;
    arm.stamp_ns = 1;
    arm.ttl_ms = 200;
    arm.valid = true;
    arm.has_arm_cmd = true;
    arm.arm = true;
    TEST_CHECK(guard.step(1, state, nullptr, arm).armed);

    cc::ControlIntent mt{};
    mt.cmd_seq = 2;
    mt.stamp_ns = 10;
    mt.ttl_ms = 200;
    mt.valid = true;
    mt.has_motor_test = true;
    mt.motor_test.enable = true;
    mt.motor_test.motor_id = 2;
    mt.motor_test.value = 0.5f;
    mt.motor_test.duration_ms = 50;

    auto started = guard.step(10, state, nullptr, mt);
    TEST_CHECK(started.effective_intent.has_motor_test);

    cc::ControlIntent empty{};
    auto latched = guard.step(30000000ull, state, nullptr, empty);
    TEST_CHECK(latched.effective_intent.has_motor_test);

    auto expired = guard.step(70000000ull, state, nullptr, empty);
    TEST_CHECK(!expired.effective_intent.has_motor_test);
    return 0;
}

int test_estop_latches()
{
    cc::ControlGuard guard(cc::ControlGuardConfig{});
    cc::ControlState state{};

    cc::ControlIntent estop{};
    estop.cmd_seq = 1;
    estop.stamp_ns = 1;
    estop.ttl_ms = 100;
    estop.valid = true;
    estop.has_estop_cmd = true;
    estop.estop = true;

    auto out = guard.step(1, state, nullptr, estop);
    TEST_CHECK(out.estop_latched);
    TEST_EQ(static_cast<int>(out.failsafe),
            static_cast<int>(cc::FailsafeAction::kEmergencyStop));
    return 0;
}

int test_guard_rejects_aligning_auto_nav()
{
    cc::ControlGuard guard(cc::ControlGuardConfig{});
    cc::ControlState state{};

    cc::ControlIntent arm{};
    arm.cmd_seq = 1;
    arm.stamp_ns = 1;
    arm.ttl_ms = 100;
    arm.valid = true;
    arm.has_arm_cmd = true;
    arm.arm = true;
    TEST_CHECK(guard.step(1, state, nullptr, arm).armed);

    shared::msg::NavStateView nav{};
    nav.valid = 0;
    nav.stale = 0;
    nav.degraded = 0;
    nav.nav_state = shared::msg::NavRunState::kAligning;
    nav.health = shared::msg::NavHealth::UNINITIALIZED;
    nav.fault_code = shared::msg::NavFaultCode::kAlignmentPending;
    nav.age_ms = 50;

    cc::ControlIntent auto_req{};
    auto_req.cmd_seq = 2;
    auto_req.stamp_ns = 2;
    auto_req.ttl_ms = 100;
    auto_req.valid = true;
    auto_req.has_mode_request = true;
    auto_req.mode_request = cc::ControlMode::kAuto;

    const auto out = guard.step(2, state, &nav, auto_req);
    TEST_EQ(static_cast<int>(out.effective_mode),
            static_cast<int>(cc::ControlMode::kFailsafe));
    TEST_EQ(static_cast<int>(out.failsafe),
            static_cast<int>(cc::FailsafeAction::kZeroOutput));
    return 0;
}

int test_guard_rejects_stale_auto_nav()
{
    cc::ControlGuard guard(cc::ControlGuardConfig{});
    cc::ControlState state{};

    cc::ControlIntent arm{};
    arm.cmd_seq = 1;
    arm.stamp_ns = 1;
    arm.ttl_ms = 100;
    arm.valid = true;
    arm.has_arm_cmd = true;
    arm.arm = true;
    TEST_CHECK(guard.step(1, state, nullptr, arm).armed);

    auto nav = make_auto_ready_nav(true);
    nav.valid = 0;
    nav.stale = 1;
    nav.nav_state = shared::msg::NavRunState::kInvalid;
    nav.health = shared::msg::NavHealth::INVALID;
    nav.fault_code = shared::msg::NavFaultCode::kNavViewStale;

    cc::ControlIntent auto_req{};
    auto_req.cmd_seq = 2;
    auto_req.stamp_ns = 2;
    auto_req.ttl_ms = 100;
    auto_req.valid = true;
    auto_req.has_mode_request = true;
    auto_req.mode_request = cc::ControlMode::kAuto;

    const auto out = guard.step(2, state, &nav, auto_req);
    TEST_EQ(static_cast<int>(out.effective_mode),
            static_cast<int>(cc::ControlMode::kFailsafe));
    TEST_EQ(static_cast<int>(out.failsafe),
            static_cast<int>(cc::FailsafeAction::kZeroOutput));
    return 0;
}

int test_guard_allows_degraded_auto_nav()
{
    cc::ControlGuard guard(cc::ControlGuardConfig{});
    cc::ControlState state{};

    cc::ControlIntent arm{};
    arm.cmd_seq = 1;
    arm.stamp_ns = 1;
    arm.ttl_ms = 100;
    arm.valid = true;
    arm.has_arm_cmd = true;
    arm.arm = true;
    TEST_CHECK(guard.step(1, state, nullptr, arm).armed);

    auto nav = make_auto_ready_nav(true);

    cc::ControlIntent auto_req{};
    auto_req.cmd_seq = 2;
    auto_req.stamp_ns = 2;
    auto_req.ttl_ms = 100;
    auto_req.valid = true;
    auto_req.has_mode_request = true;
    auto_req.mode_request = cc::ControlMode::kAuto;

    auto switched = guard.step(2, state, &nav, auto_req);
    TEST_EQ(static_cast<int>(switched.effective_mode),
            static_cast<int>(cc::ControlMode::kAuto));
    TEST_EQ(static_cast<int>(switched.failsafe),
            static_cast<int>(cc::FailsafeAction::kNone));

    cc::ControlIntent hold{};
    hold.cmd_seq = 3;
    hold.stamp_ns = 3;
    hold.ttl_ms = 100;
    hold.valid = true;
    auto steady = guard.step(3, state, &nav, hold);
    TEST_EQ(static_cast<int>(steady.effective_mode),
            static_cast<int>(cc::ControlMode::kAuto));
    TEST_EQ(static_cast<int>(steady.failsafe),
            static_cast<int>(cc::FailsafeAction::kNone));
    return 0;
}

int test_guard_emits_structured_events()
{
    cc::ControlGuard guard(cc::ControlGuardConfig{});
    cc::ControlState state{};
    std::vector<std::string> events;

    guard.set_event_callback([&](const cc::GuardEvent& event) {
        events.emplace_back(event.event != nullptr ? event.event : "");
    });

    cc::ControlIntent arm{};
    arm.cmd_seq = 1;
    arm.stamp_ns = 1;
    arm.ttl_ms = 100;
    arm.valid = true;
    arm.has_arm_cmd = true;
    arm.arm = true;
    TEST_CHECK(guard.step(1, state, nullptr, arm).armed);

    shared::msg::NavStateView bad_nav{};
    bad_nav.valid = 0;
    bad_nav.stale = 1;
    bad_nav.degraded = 0;
    bad_nav.nav_state = shared::msg::NavRunState::kInvalid;
    bad_nav.health = shared::msg::NavHealth::INVALID;
    bad_nav.fault_code = shared::msg::NavFaultCode::kNavViewStale;

    cc::ControlIntent auto_req{};
    auto_req.cmd_seq = 2;
    auto_req.stamp_ns = 2;
    auto_req.ttl_ms = 100;
    auto_req.valid = true;
    auto_req.has_mode_request = true;
    auto_req.mode_request = cc::ControlMode::kAuto;

    const auto rejected = guard.step(2, state, &bad_nav, auto_req);
    TEST_EQ(static_cast<int>(rejected.effective_mode),
            static_cast<int>(cc::ControlMode::kFailsafe));

    auto has_event = [&](const char* name) {
        for (const auto& event_name : events) {
            if (event_name == name) {
                return true;
            }
        }
        return false;
    };

    TEST_CHECK(has_event("guard_reject"));
    TEST_CHECK(has_event("guard_nav_gating_changed"));
    TEST_CHECK(has_event("guard_failsafe_entered"));

    events.clear();
    auto good_nav = make_auto_ready_nav();

    cc::ControlIntent manual_req{};
    manual_req.cmd_seq = 3;
    manual_req.stamp_ns = 3;
    manual_req.ttl_ms = 100;
    manual_req.valid = true;
    manual_req.has_mode_request = true;
    manual_req.mode_request = cc::ControlMode::kManual;

    const auto recovered = guard.step(3, state, &good_nav, manual_req);
    TEST_EQ(static_cast<int>(recovered.effective_mode),
            static_cast<int>(cc::ControlMode::kManual));
    TEST_EQ(static_cast<int>(recovered.failsafe),
            static_cast<int>(cc::FailsafeAction::kNone));
    TEST_CHECK(has_event("guard_nav_gating_changed"));
    TEST_CHECK(has_event("guard_failsafe_cleared"));
    return 0;
}

int test_wire_codec_maps_failsafe()
{
    shared::msg::ControlIntent wire{};
    wire.version = shared::msg::kControlIntentWireVersion;
    wire.flags = shared::msg::kHasModeRequest;
    wire.mode_request = shared::msg::ControlMode::kFailsafe;
    wire.cmd_seq = 5;
    wire.stamp_ns = 123;
    wire.ttl_ms = 100;

    cc::ControlIntent out{};
    TEST_CHECK(rovctrl::io::input::decode_control_intent(wire, out));
    TEST_EQ(static_cast<int>(out.mode_request),
            static_cast<int>(cc::ControlMode::kFailsafe));
    TEST_EQ(out.source_id, wire.source_id);
    return 0;
}

int test_depth_hold_pid_latches_current_depth()
{
    rovctrl::controllers::DepthHoldPidControllerConfig cfg{};
    cfg.depth.kp = 10.0;
    cfg.depth.ki = 0.0;
    cfg.depth.kd = 0.0;
    cfg.depth.out_min = -100.0;
    cfg.depth.out_max = 100.0;
    cfg.depth.invert_error = true;
    cfg.depth.rate_limit_enabled = false;
    cfg.depth.derivative_filter_enabled = false;

    rovctrl::controllers::DepthHoldPidController ctrl(cfg);
    cc::ControlState state{};
    state.nav_valid = true;
    state.nav_depth = 3.0;

    cc::ControlReference ref{};
    cc::ControlOutput out{};
    TEST_CHECK(ctrl.compute(state, ref, out, 0.1));
    TEST_CHECK(out.has_body_wrench);
    TEST_NEAR(out.body_wrench[2], 0.0, 1e-9);

    state.nav_depth = 4.0;
    TEST_CHECK(ctrl.compute(state, ref, out, 0.1));
    TEST_CHECK(out.body_wrench[2] > 0.0);
    return 0;
}

int test_heading_hold_pid_wraps_short_arc()
{
    constexpr double kPi = 3.14159265358979323846;

    rovctrl::controllers::HeadingHoldPidControllerConfig cfg{};
    cfg.heading.kp = 10.0;
    cfg.heading.ki = 0.0;
    cfg.heading.kd = 0.0;
    cfg.heading.out_min = -100.0;
    cfg.heading.out_max = 100.0;
    cfg.heading.wrap_error = true;
    cfg.heading.wrap_min = -kPi;
    cfg.heading.wrap_max =  kPi;
    cfg.heading.rate_limit_enabled = false;
    cfg.heading.derivative_filter_enabled = false;

    rovctrl::controllers::HeadingHoldPidController ctrl(cfg);
    cc::ControlState state{};
    state.nav_valid = true;
    state.nav_rpy[2] = 3.10;

    cc::ControlReference ref{};
    ref.use_pose_ref = true;
    ref.pose_ref.yaw = -3.10;

    cc::ControlOutput out{};
    TEST_CHECK(ctrl.compute(state, ref, out, 0.1));
    TEST_CHECK(out.has_body_wrench);
    TEST_CHECK(out.body_wrench[5] > 0.0);
    TEST_CHECK(out.body_wrench[5] < 5.0);
    return 0;
}

int test_controller_manager_auto_uses_registered_depth_heading_pid()
{
    rovctrl::controllers::ManualControllerConfig manual_cfg{};
    cc::ControllerManagerOptions mgr_opt{};
    mgr_opt.default_auto_controller = "depth_heading_pid";
    cc::ControllerManager mgr(mgr_opt);

    auto manual =
        cc::ControllerManager::make_controller<rovctrl::controllers::ManualController>(manual_cfg);
    TEST_CHECK(mgr.init_manual_only(std::move(manual)));

    rovctrl::controllers::DepthHeadingPidControllerConfig pid_cfg{};
    pid_cfg.depth.kp = 10.0;
    pid_cfg.depth.ki = 0.0;
    pid_cfg.depth.kd = 0.0;
    pid_cfg.depth.out_min = -100.0;
    pid_cfg.depth.out_max = 100.0;
    pid_cfg.depth.invert_error = true;
    pid_cfg.depth.rate_limit_enabled = false;
    pid_cfg.depth.derivative_filter_enabled = false;
    pid_cfg.heading.kp = 10.0;
    pid_cfg.heading.ki = 0.0;
    pid_cfg.heading.kd = 0.0;
    pid_cfg.heading.out_min = -100.0;
    pid_cfg.heading.out_max = 100.0;
    pid_cfg.heading.wrap_error = true;
    pid_cfg.heading.rate_limit_enabled = false;
    pid_cfg.heading.derivative_filter_enabled = false;

    auto auto_ctrl =
        cc::ControllerManager::make_controller<rovctrl::controllers::DepthHeadingPidController>(pid_cfg);
    TEST_CHECK(mgr.register_controller(std::move(auto_ctrl)));
    TEST_CHECK(mgr.set_mode(cc::ControlMode::kAuto));
    TEST_CHECK(mgr.status().active_controller == "depth_heading_pid");

    cc::ControlState state{};
    state.nav_valid = true;
    state.nav_depth = 2.0;
    state.nav_rpy[2] = 0.1;

    cc::ControlReference ref{};
    cc::ControlOutput out{};
    TEST_CHECK(mgr.compute(state, ref, out, 0.1));
    TEST_CHECK(out.has_body_wrench);
    TEST_NEAR(out.body_wrench[2], 0.0, 1e-9);
    TEST_NEAR(out.body_wrench[5], 0.0, 1e-9);

    state.nav_depth = 2.2;
    state.nav_rpy[2] = 0.2;
    TEST_CHECK(mgr.compute(state, ref, out, 0.1));
    TEST_CHECK(out.body_wrench[2] > 0.0);
    TEST_CHECK(out.body_wrench[5] < 0.0);
    return 0;
}

int test_pid_reset_does_not_override_same_cycle_reference()
{
    rovctrl::controllers::DepthHoldPidControllerConfig cfg{};
    cfg.depth.kp = 10.0;
    cfg.depth.ki = 0.0;
    cfg.depth.kd = 0.0;
    cfg.depth.out_min = -100.0;
    cfg.depth.out_max = 100.0;
    cfg.depth.invert_error = true;
    cfg.depth.rate_limit_enabled = false;
    cfg.depth.derivative_filter_enabled = false;

    rovctrl::controllers::DepthHoldPidController ctrl(cfg);
    cc::ControlState state{};
    state.nav_valid = true;
    state.nav_depth = 3.0;

    cc::ControlReference ref{};
    cc::ControlOutput out{};
    TEST_CHECK(ctrl.compute(state, ref, out, 0.1));
    TEST_NEAR(out.body_wrench[2], 0.0, 1e-9);

    ctrl.reset();
    state.nav_depth = 3.5;
    ref.use_pose_ref = true;
    ref.pose_ref.z = 4.0;

    TEST_CHECK(ctrl.compute(state, ref, out, 0.1));
    TEST_CHECK(out.body_wrench[2] < 0.0);
    return 0;
}

} // namespace

int main()
{
    int rc = 0;
    rc = test_multi_input_preserves_primary_metadata();
    if (rc != 0) return rc;
    rc = test_guard_stale_zeroes_manual_output();
    if (rc != 0) return rc;
    rc = test_guard_motor_test_latch_and_expire();
    if (rc != 0) return rc;
    rc = test_estop_latches();
    if (rc != 0) return rc;
    rc = test_guard_rejects_aligning_auto_nav();
    if (rc != 0) return rc;
    rc = test_guard_rejects_stale_auto_nav();
    if (rc != 0) return rc;
    rc = test_guard_allows_degraded_auto_nav();
    if (rc != 0) return rc;
    rc = test_guard_emits_structured_events();
    if (rc != 0) return rc;
    rc = test_wire_codec_maps_failsafe();
    if (rc != 0) return rc;
    rc = test_depth_hold_pid_latches_current_depth();
    if (rc != 0) return rc;
    rc = test_heading_hold_pid_wraps_short_arc();
    if (rc != 0) return rc;
    rc = test_controller_manager_auto_uses_registered_depth_heading_pid();
    if (rc != 0) return rc;
    rc = test_pid_reset_does_not_override_same_cycle_reference();
    if (rc != 0) return rc;

    std::cout << "[test_v1_closed_loop] all tests passed.\n";
    return 0;
}
