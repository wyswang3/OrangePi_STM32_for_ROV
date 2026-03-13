#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "control_core/app_context.hpp"
#include "control_core/telemetry_frame_builder.hpp"
#include "control_core/thruster_allocation.hpp"
#include "controllers/controller_manager.hpp"
#include "controllers/manual_controller.hpp"
#include "controllers/pid_controller.hpp"
#include "io/nav/nav_state_view.hpp"
#include "shared/msg/telemetry_frame_v2.hpp"

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
namespace ctrl = rovctrl::controllers;

std::string repo_path(const std::string& relative)
{
    return (std::filesystem::path(PID_FRAMEWORK_SOURCE_DIR) / relative).string();
}

std::filesystem::path make_temp_dir(const std::string& name)
{
    const auto stamp = std::to_string(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const auto dir = std::filesystem::temp_directory_path() / (name + "_" + stamp);
    std::filesystem::create_directories(dir);
    return dir;
}

std::filesystem::path write_text_file(const std::filesystem::path& dir,
                                      const std::string&           name,
                                      const std::string&           body)
{
    const auto path = dir / name;
    std::ofstream os(path);
    os << body;
    os.close();
    return path;
}

bool any_abs_gt(const std::array<float, 8>& arr, float threshold)
{
    for (float v : arr) {
        if (std::fabs(v) > threshold) return true;
    }
    return false;
}

bool any_duty_changed_from_mid(const std::array<float, 8>& arr, float mid)
{
    for (float v : arr) {
        if (std::fabs(v - mid) > 1e-3f) return true;
    }
    return false;
}

bool any_abs_gt_raw(const float* arr, std::size_t n, float threshold)
{
    for (std::size_t i = 0; i < n; ++i) {
        if (std::fabs(arr[i]) > threshold) return true;
    }
    return false;
}

bool any_duty_changed_from_mid_raw(const float* arr, std::size_t n, float mid)
{
    for (std::size_t i = 0; i < n; ++i) {
        if (std::fabs(arr[i] - mid) > 1e-3f) return true;
    }
    return false;
}

std::string make_control_config_yaml_sections(const std::string& modes_default_auto_controller,
                                              bool               emit_modes_default_auto_controller,
                                              const std::string& controllers_default_auto_controller,
                                              bool emit_controllers_default_auto_controller,
                                              const std::string& initial_mode,
                                              double             depth_kp,
                                              bool               yaw_enable)
{
    std::string yaml;
    yaml += "controllers:\n";
    if (emit_controllers_default_auto_controller) {
        yaml += "  default_auto_controller: \"" + controllers_default_auto_controller + "\"\n";
    }
    yaml +=
        "  pid:\n"
        "    enabled: true\n"
        "    defaults:\n"
        "      anti_windup:\n"
        "        enabled: true\n"
        "        i_min: -1.0\n"
        "        i_max: 1.0\n"
        "      derivative_filter:\n"
        "        enabled: false\n"
        "        cutoff_hz: 5.0\n"
        "      deadband:\n"
        "        enabled: true\n"
        "        error_deadband: 0.0\n"
        "      rate_limit:\n"
        "        enabled: false\n"
        "        max_abs_du: 50.0\n"
        "    depth:\n"
        "      enable: true\n"
        "      kp: " + std::to_string(depth_kp) + "\n"
        "      ki: 0.0\n"
        "      kd: 0.0\n"
        "      max_output: 100.0\n"
        "      min_output: -100.0\n"
        "    yaw_angle:\n"
        "      enable: " + std::string(yaw_enable ? "true" : "false") + "\n"
        "      kp: 5.0\n"
        "      ki: 0.0\n"
        "      kd: 0.0\n"
        "      max_output: 50.0\n"
        "      min_output: -50.0\n";
    yaml += "modes:\n";
    yaml += "  initial_mode: \"" + initial_mode + "\"\n";
    if (emit_modes_default_auto_controller) {
        yaml += "  default_auto_controller: \"" + modes_default_auto_controller + "\"\n";
    }
    return yaml;
}

std::string make_control_config_yaml(const std::string& default_auto_controller,
                                     const std::string& initial_mode,
                                     double             depth_kp,
                                     bool               yaw_enable)
{
    return make_control_config_yaml_sections(default_auto_controller,
                                             true,
                                             default_auto_controller,
                                             true,
                                             initial_mode,
                                             depth_kp,
                                             yaw_enable);
}

cc::AppBuildOptions make_dummy_build_options(const std::string& control_config)
{
    cc::AppBuildOptions opt{};
    opt.loop_hz = 50.0;
    opt.pwm_ctrl_hz = 50.0;
    opt.max_step_pct = 0.2;
    opt.enable_gcs = false;
    opt.enable_teleop = true;
    opt.pwm_dummy = true;
    opt.pwm_dummy_print = false;
    opt.control_config_cli = control_config;
    opt.pwm_config_cli = repo_path("OrangePi_STM32_for_ROV/pwm_control_program/config/pwm_client.yaml");
    opt.alloc_config_cli = repo_path("OrangePi_STM32_for_ROV/pwm_control_program/config/alloc.yaml");
    opt.teleop_mixer_config_cli = repo_path("OrangePi_STM32_for_ROV/pwm_control_program/config/teleop_mixer.yaml");
    opt.traj_config_cli = repo_path("OrangePi_STM32_for_ROV/pwm_control_program/config/trajectory.yaml");
    return opt;
}

int test_pid_axis_dt_clamp_and_integral_limit()
{
    ctrl::PidAxisConfig cfg{};
    cfg.kp = 0.0;
    cfg.ki = 1.0;
    cfg.kd = 0.0;
    cfg.i_min = -0.5;
    cfg.i_max = 0.5;
    cfg.out_min = -10.0;
    cfg.out_max = 10.0;

    ctrl::PidAxis axis(cfg);
    TEST_NEAR(axis.step(1.0, 0.0, 10.0), 0.2, 1e-9);
    TEST_NEAR(axis.step(1.0, 0.0, 10.0), 0.4, 1e-9);
    TEST_NEAR(axis.step(1.0, 0.0, 10.0), 0.5, 1e-9);
    TEST_NEAR(axis.step(1.0, 0.0, 0.0), 0.5, 1e-9);
    return 0;
}

int test_pid_axis_rate_limit()
{
    ctrl::PidAxisConfig cfg{};
    cfg.kp = 10.0;
    cfg.ki = 0.0;
    cfg.kd = 0.0;
    cfg.out_min = -100.0;
    cfg.out_max = 100.0;
    cfg.rate_limit_enabled = true;
    cfg.max_abs_du = 1.0;

    ctrl::PidAxis axis(cfg);
    TEST_NEAR(axis.step(0.0, 0.0, 0.1), 0.0, 1e-9);
    TEST_NEAR(axis.step(1.0, 0.0, 0.1), 0.1, 1e-9);
    TEST_NEAR(axis.step(1.0, 0.0, 0.1), 0.2, 1e-9);
    return 0;
}

int test_depth_pid_direction_latch_and_reset()
{
    ctrl::DepthHoldPidControllerConfig cfg{};
    cfg.depth.kp = 10.0;
    cfg.depth.ki = 0.0;
    cfg.depth.kd = 0.0;
    cfg.depth.out_min = -100.0;
    cfg.depth.out_max = 100.0;
    cfg.depth.invert_error = true;
    cfg.depth.rate_limit_enabled = false;
    cfg.depth.derivative_filter_enabled = false;

    ctrl::DepthHoldPidController controller(cfg);
    cc::ControlState state{};
    state.nav_valid = true;
    state.nav_depth = 2.0;

    cc::ControlReference ref{};
    cc::ControlOutput out{};
    TEST_CHECK(controller.compute(state, ref, out, 0.1));
    TEST_NEAR(out.body_wrench[2], 0.0, 1e-9);

    state.nav_depth = 2.5;
    TEST_CHECK(controller.compute(state, ref, out, 0.1));
    TEST_CHECK(out.body_wrench[2] > 0.0);

    controller.reset();
    TEST_CHECK(controller.compute(state, ref, out, 0.1));
    TEST_NEAR(out.body_wrench[2], 0.0, 1e-9);
    return 0;
}

int test_reset_and_same_cycle_reference_priority()
{
    ctrl::DepthHoldPidControllerConfig cfg{};
    cfg.depth.kp = 10.0;
    cfg.depth.ki = 0.0;
    cfg.depth.kd = 0.0;
    cfg.depth.out_min = -100.0;
    cfg.depth.out_max = 100.0;
    cfg.depth.invert_error = true;
    cfg.depth.rate_limit_enabled = false;
    cfg.depth.derivative_filter_enabled = false;

    ctrl::DepthHoldPidController controller(cfg);
    cc::ControlState state{};
    state.nav_valid = true;
    state.nav_depth = 3.0;

    cc::ControlReference ref{};
    cc::ControlOutput out{};
    TEST_CHECK(controller.compute(state, ref, out, 0.1));
    controller.reset();

    state.nav_depth = 3.5;
    ref.use_pose_ref = true;
    ref.pose_ref.z = 4.0;
    TEST_CHECK(controller.compute(state, ref, out, 0.1));
    TEST_CHECK(out.body_wrench[2] < 0.0);
    return 0;
}

int test_controller_manager_auto_switch_matrix()
{
    ctrl::ManualControllerConfig manual_cfg{};

    {
        cc::ControllerManagerOptions opt{};
        opt.default_auto_controller = "missing_pid";
        cc::ControllerManager manager(opt);
        auto manual =
            cc::ControllerManager::make_controller<ctrl::ManualController>(manual_cfg);
        TEST_CHECK(manager.init_manual_only(std::move(manual)));
        TEST_CHECK(!manager.set_mode(cc::ControlMode::kAuto));
    }

    {
        cc::ControllerManagerOptions opt{};
        opt.default_auto_controller = "depth_hold_pid";
        cc::ControllerManager manager(opt);
        auto manual =
            cc::ControllerManager::make_controller<ctrl::ManualController>(manual_cfg);
        TEST_CHECK(manager.init_manual_only(std::move(manual)));

        ctrl::DepthHoldPidControllerConfig depth_cfg{};
        depth_cfg.depth.kp = 8.0;
        depth_cfg.depth.ki = 0.0;
        depth_cfg.depth.kd = 0.0;
        depth_cfg.depth.out_min = -100.0;
        depth_cfg.depth.out_max = 100.0;
        TEST_CHECK(manager.register_controller(
            cc::ControllerManager::make_controller<ctrl::DepthHoldPidController>(depth_cfg)));
        TEST_CHECK(manager.set_mode(cc::ControlMode::kAuto));
        TEST_CHECK(manager.status().active_controller == "depth_hold_pid");
    }

    {
        cc::ControllerManagerOptions opt{};
        opt.default_auto_controller = "depth_hold_pid";
        cc::ControllerManager manager(opt);
        auto manual =
            cc::ControllerManager::make_controller<ctrl::ManualController>(manual_cfg);
        TEST_CHECK(manager.init_manual_only(std::move(manual)));

        ctrl::DepthHoldPidControllerConfig depth_cfg{};
        depth_cfg.depth.kp = 8.0;
        depth_cfg.depth.ki = 0.0;
        depth_cfg.depth.kd = 0.0;
        depth_cfg.depth.out_min = -100.0;
        depth_cfg.depth.out_max = 100.0;

        ctrl::HeadingHoldPidControllerConfig heading_cfg{};
        heading_cfg.heading.kp = 4.0;
        heading_cfg.heading.ki = 0.0;
        heading_cfg.heading.kd = 0.0;
        heading_cfg.heading.out_min = -50.0;
        heading_cfg.heading.out_max = 50.0;

        TEST_CHECK(manager.register_controller(
            cc::ControllerManager::make_controller<ctrl::DepthHoldPidController>(depth_cfg)));
        TEST_CHECK(manager.register_controller(
            cc::ControllerManager::make_controller<ctrl::HeadingHoldPidController>(heading_cfg)));
        TEST_CHECK(manager.select_auto_controller("heading_hold_pid"));
        TEST_CHECK(manager.set_mode(cc::ControlMode::kAuto));
        TEST_CHECK(manager.status().active_controller == "heading_hold_pid");
    }

    return 0;
}

int test_app_context_loads_pid_params_and_default_auto_controller()
{
    const auto temp_dir = make_temp_dir("pid_framework_appctx");
    const auto control_path = write_text_file(
        temp_dir,
        "control_params.yaml",
        make_control_config_yaml("depth_hold_pid", "auto", 12.0, false));

    cc::AppContext ctx{};
    std::atomic_bool stop{false};
    const auto opt = make_dummy_build_options(control_path.string());
    const std::string argv0 = repo_path("OrangePi_STM32_for_ROV/pwm_control_program/config/pwm_client.yaml");

    const auto br = cc::build_app_context(opt, argv0.c_str(), &stop, ctx, std::cerr);
    TEST_CHECK(br.ok);
    TEST_EQ(static_cast<int>(ctx.ctrl_mgr.mode()), static_cast<int>(cc::ControlMode::kAuto));
    TEST_CHECK(ctx.ctrl_mgr.status().active_controller == "depth_hold_pid");

    cc::ControlState state{};
    state.nav_valid = true;
    state.nav_depth = 2.0;
    cc::ControlReference ref{};
    cc::ControlOutput out{};
    TEST_CHECK(ctx.ctrl_mgr.compute(state, ref, out, 0.1));
    TEST_NEAR(out.body_wrench[2], 0.0, 1e-9);

    state.nav_depth = 3.0;
    TEST_CHECK(ctx.ctrl_mgr.compute(state, ref, out, 0.1));
    TEST_NEAR(out.body_wrench[2], 12.0, 1e-6);

    ctx.shutdown(std::cerr, 0.1f);
    return 0;
}

int test_app_context_invalid_default_auto_falls_back_to_manual()
{
    const auto temp_dir = make_temp_dir("pid_framework_appctx_invalid");
    const auto control_path = write_text_file(
        temp_dir,
        "control_params.yaml",
        make_control_config_yaml("missing_pid", "auto", 6.0, false));

    cc::AppContext ctx{};
    std::atomic_bool stop{false};
    const auto opt = make_dummy_build_options(control_path.string());
    const std::string argv0 = repo_path("OrangePi_STM32_for_ROV/pwm_control_program/config/pwm_client.yaml");

    const auto br = cc::build_app_context(opt, argv0.c_str(), &stop, ctx, std::cerr);
    TEST_CHECK(br.ok);
    TEST_EQ(static_cast<int>(ctx.ctrl_mgr.mode()), static_cast<int>(cc::ControlMode::kManual));
    TEST_CHECK(ctx.ctrl_mgr.status().active_controller == "manual");

    ctx.shutdown(std::cerr, 0.1f);
    return 0;
}

int test_app_context_loads_modes_default_auto_controller()
{
    const auto temp_dir = make_temp_dir("pid_framework_modes_default");
    const auto control_path = write_text_file(
        temp_dir,
        "control_params.yaml",
        make_control_config_yaml_sections("heading_hold_pid",
                                          true,
                                          "",
                                          false,
                                          "auto",
                                          6.0,
                                          true));

    cc::AppContext ctx{};
    std::atomic_bool stop{false};
    const auto opt = make_dummy_build_options(control_path.string());
    const std::string argv0 = repo_path("OrangePi_STM32_for_ROV/pwm_control_program/config/pwm_client.yaml");

    const auto br = cc::build_app_context(opt, argv0.c_str(), &stop, ctx, std::cerr);
    TEST_CHECK(br.ok);
    TEST_EQ(static_cast<int>(ctx.ctrl_mgr.mode()), static_cast<int>(cc::ControlMode::kAuto));
    TEST_CHECK(ctx.ctrl_mgr.status().active_controller == "heading_hold_pid");

    ctx.shutdown(std::cerr, 0.1f);
    return 0;
}

int test_app_context_controller_default_overrides_modes_default()
{
    const auto temp_dir = make_temp_dir("pid_framework_default_precedence");
    const auto control_path = write_text_file(
        temp_dir,
        "control_params.yaml",
        make_control_config_yaml_sections("depth_hold_pid",
                                          true,
                                          "heading_hold_pid",
                                          true,
                                          "auto",
                                          6.0,
                                          true));

    cc::AppContext ctx{};
    std::atomic_bool stop{false};
    const auto opt = make_dummy_build_options(control_path.string());
    const std::string argv0 = repo_path("OrangePi_STM32_for_ROV/pwm_control_program/config/pwm_client.yaml");

    const auto br = cc::build_app_context(opt, argv0.c_str(), &stop, ctx, std::cerr);
    TEST_CHECK(br.ok);
    TEST_EQ(static_cast<int>(ctx.ctrl_mgr.mode()), static_cast<int>(cc::ControlMode::kAuto));
    TEST_CHECK(ctx.ctrl_mgr.status().active_controller == "heading_hold_pid");

    ctx.shutdown(std::cerr, 0.1f);
    return 0;
}

int test_pid_software_chain_smoke()
{
    const auto temp_dir = make_temp_dir("pid_framework_smoke");
    const auto control_path = write_text_file(
        temp_dir,
        "control_params.yaml",
        make_control_config_yaml("depth_hold_pid", "auto", 15.0, false));

    cc::AppContext ctx{};
    std::atomic_bool stop{false};
    const auto opt = make_dummy_build_options(control_path.string());
    const std::string argv0 = repo_path("OrangePi_STM32_for_ROV/pwm_control_program/config/pwm_client.yaml");
    const auto br = cc::build_app_context(opt, argv0.c_str(), &stop, ctx, std::cerr);
    TEST_CHECK(br.ok);
    TEST_EQ(static_cast<int>(ctx.ctrl_mgr.mode()), static_cast<int>(cc::ControlMode::kAuto));
    TEST_CHECK(ctx.ctrl_mgr.status().active_controller == "depth_hold_pid");

    cc::ControlState state{};
    state.nav_valid = true;
    state.nav_depth = 2.0;

    cc::ControlReference ref{};
    cc::ControlOutput out{};
    TEST_CHECK(ctx.ctrl_mgr.compute(state, ref, out, 0.1));

    state.nav_depth = 2.4;
    TEST_CHECK(ctx.ctrl_mgr.compute(state, ref, out, 0.1));
    TEST_CHECK(out.has_body_wrench);
    TEST_CHECK(out.body_wrench[2] > 0.0);

    cc::ThrusterAllocator allocator{};
    TEST_CHECK(allocator.init(ctx.loop_cfg.thruster_alloc));
    cc::BodyWrench6D wrench{};
    for (std::size_t i = 0; i < wrench.size(); ++i) {
        wrench[i] = out.body_wrench[i];
    }

    cc::ThrusterNormArray thr_cmd{};
    TEST_CHECK(allocator.allocate(wrench, thr_cmd));
    TEST_CHECK(any_abs_gt(thr_cmd, 1e-3f));

    TEST_EQ(ctx.pwm_client.setTargets(thr_cmd), 0);
    TEST_EQ(ctx.pwm_client.step(), 0);

    std::array<float, rovctrl::platform::kNumPwmChannels> applied{};
    TEST_CHECK(ctx.pwm_client.getLastApplied(applied));
    TEST_CHECK(any_duty_changed_from_mid(applied, 7.5f));

    cc::TelemetryBuildInput build_input{};
    build_input.stamp_ns = 123456789u;
    build_input.applied_intent.has_mode_request = true;
    build_input.applied_intent.mode_request = cc::ControlMode::kAuto;
    build_input.applied_intent.valid = true;
    build_input.applied_reference = ref;
    build_input.current_state = state;
    build_input.active_mode = ctx.ctrl_mgr.mode();
    build_input.controller_status = ctx.ctrl_mgr.status();
    build_input.auto_fail_limit = ctx.ctrl_mgr.auto_fail_limit();
    build_input.armed = true;
    build_input.thruster_cmd = thr_cmd;
    build_input.pwm_duty = applied;
    build_input.pwm_ok = ctx.pwm_client.is_ok();

    shared::msg::TelemetryFrameV2 frame{};
    fill_telemetry_frame_v2(frame, build_input);

    TEST_EQ(frame.seq, 1u);
    TEST_EQ(frame.stamp_ns, 123456789u);
    TEST_EQ(frame.source,
            static_cast<std::uint8_t>(shared::msg::TelemetrySource::kControlCore));
    TEST_EQ(frame.control.active_mode,
            static_cast<std::uint8_t>(shared::msg::RuntimeControlMode::kAuto));
    TEST_EQ(frame.control.armed, 1u);
    TEST_EQ(frame.control.controller_status,
            static_cast<std::uint8_t>(shared::msg::ControllerStatus::kRunning));
    TEST_EQ(frame.control.auto_fail_limit, ctx.ctrl_mgr.auto_fail_limit());
    TEST_EQ(frame.control.intent_fresh, 1u);
    TEST_CHECK(std::string(frame.control.controller_name) == "depth_hold_pid");
    TEST_CHECK(any_abs_gt_raw(frame.control.thruster_cmd, 8, 1e-3f));
    TEST_CHECK(any_duty_changed_from_mid_raw(frame.control.pwm_duty, 8, 7.5f));
    TEST_EQ(frame.system.pwm_link_state,
            static_cast<std::uint8_t>(shared::msg::LinkState::kAlive));
    TEST_EQ(frame.system.nav_state,
            static_cast<std::uint8_t>(shared::msg::RuntimeNavState::kInvalid));
    ctx.shutdown(std::cerr, 0.1f);
    return 0;
}

int test_telemetry_preserves_total_nav_age_semantics()
{
    rovctrl::io::NavStateView nav_snapshot{};
    nav_snapshot.wire.valid = 1;
    nav_snapshot.wire.stale = 0;
    nav_snapshot.wire.degraded = 0;
    nav_snapshot.wire.nav_state = shared::msg::NavRunState::kOk;
    nav_snapshot.wire.health = shared::msg::NavHealth::OK;
    nav_snapshot.wire.stamp_ns = 1'000'000'000ull;
    nav_snapshot.wire.mono_ns = 1'120'000'000ull;
    nav_snapshot.wire.age_ms = 150;
    nav_snapshot.pub_mono_ns = nav_snapshot.wire.mono_ns;
    nav_snapshot.age_ms_local = 25;

    cc::TelemetryBuildInput build_input{};
    build_input.stamp_ns = 1'200'000'000ull;
    build_input.nav_snapshot = &nav_snapshot;
    build_input.nav_age_ms = nav_snapshot.total_age_ms();

    shared::msg::TelemetryFrameV2 frame{};
    fill_telemetry_frame_v2(frame, build_input);

    TEST_EQ(frame.stamp_ns, build_input.stamp_ns);
    TEST_EQ(frame.system.nav_valid, 1u);
    TEST_EQ(frame.system.nav_state,
            static_cast<std::uint8_t>(shared::msg::RuntimeNavState::kOk));
    TEST_EQ(frame.system.nav_age_ms, nav_snapshot.total_age_ms());
    TEST_EQ(frame.system.nav_stale, 0u);
    TEST_EQ(frame.system.nav_degraded, 0u);
    return 0;
}

} // namespace

int main()
{
    int rc = 0;
    rc = test_pid_axis_dt_clamp_and_integral_limit();
    if (rc != 0) return rc;
    rc = test_pid_axis_rate_limit();
    if (rc != 0) return rc;
    rc = test_depth_pid_direction_latch_and_reset();
    if (rc != 0) return rc;
    rc = test_reset_and_same_cycle_reference_priority();
    if (rc != 0) return rc;
    rc = test_controller_manager_auto_switch_matrix();
    if (rc != 0) return rc;
    rc = test_app_context_loads_pid_params_and_default_auto_controller();
    if (rc != 0) return rc;
    rc = test_app_context_invalid_default_auto_falls_back_to_manual();
    if (rc != 0) return rc;
    rc = test_app_context_loads_modes_default_auto_controller();
    if (rc != 0) return rc;
    rc = test_app_context_controller_default_overrides_modes_default();
    if (rc != 0) return rc;
    rc = test_pid_software_chain_smoke();
    if (rc != 0) return rc;
    rc = test_telemetry_preserves_total_nav_age_semantics();
    if (rc != 0) return rc;

    std::cout << "[test_pid_framework] all tests passed.\n";
    return 0;
}
