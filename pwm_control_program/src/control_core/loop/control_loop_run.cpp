/**
 * @file   control_loop_run.cpp
 * @brief  ControlLoop::run main loop (Path A) — engineered loop with NavStateView (B2)
 *
 * Key points:
 *  - Nav feedback uses rovctrl::io::NavStateView (from gateway/nav_viewd via SHM).
 *  - nav_view / nav_present are defined per-iteration in the correct scope (no shadowing).
 *  - 控制侧显式区分“拿到快照”和“快照可信”，不再把 invalid/stale/no-data 折叠成一个 false。
 *  - Manual/Teleop do not hard-require nav; Auto can require nav per policy.
 */

#include "control_core/control_loop.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string_view>
#include <thread>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <unistd.h>

#include "control_core/telemetry_frame_builder.hpp"
#include "io/log/control_loop_logger.hpp"
#include "io/log/telemetry_timeline_logger.hpp"
#include "io/nav/nav_state_view.hpp"   // rovctrl::io::NavStateView
#include "io/state/telemetry_publisher_shm.hpp"
#include "shared/msg/control_intent.hpp"
#include "shared/msg/telemetry_frame_v2.hpp"

// ==================== 小仪表输出：推进器活动日志 ====================
namespace {

// ThrusterArray 一般是 std::array<float, 8>，这里用模板支持 N 维兼容。
// 功能：
//  1) 如果所有通道都“非常接近 0”，则认为是静止，不打印；
//  2) 打印节流：至少每 200 ms 才打印一次，避免刷屏；
//  3) 打印格式：
//     [ControlLoop] thrusters active: m1=0.32 m2=0.28 ... m8=0.00
template <std::size_t N>
void log_thruster_activity(const std::array<float, N>& thr_cmd)
{
    // 1) 判断是否“近似静止”
    constexpr float kEps = 1e-3f;  // 小于这个就视为 0
    float max_abs = 0.0f;
    for (float v : thr_cmd) {
        max_abs = std::max(max_abs, std::fabs(v));
    }
    if (max_abs < kEps) {
        // 所有通道都很小，认为静止，不输出
        return;
    }

    // 2) 简单节流：200 ms 以上才打印一次
    using clock = std::chrono::steady_clock;
    static clock::time_point last_print_tp = clock::now();
    const auto now = clock::now();
    if (now - last_print_tp < std::chrono::milliseconds(200)) {
        return;
    }
    last_print_tp = now;

    // 3) 打印一行当前推进器归一化指令（通常 -1..1 或 0..1）
    // std::cout << "[ControlLoop] thrusters active: ";
    // std::cout << std::fixed << std::setprecision(2);
    // for (std::size_t i = 0; i < N; ++i) {
    //     std::cout << "m" << (i + 1) << "=" << thr_cmd[i] << " ";
    // }
    // std::cout << "\n";
}


std::string csv_escape(std::string_view value)
{
    std::string out;
    out.reserve(value.size() + 2);
    out.push_back('"');
    for (const char ch : value) {
        if (ch == '"') {
            out.push_back('"');
        }
        out.push_back(ch);
    }
    out.push_back('"');
    return out;
}

std::string wall_time_now_string()
{
    std::time_t t = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

std::string resolve_run_id(const char* process_name)
{
    if (const char* env = std::getenv("ROV_RUN_ID"); env != nullptr && env[0] != '\0') {
        return env;
    }

    std::time_t t = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream oss;
    oss << process_name << "-" << ::getpid() << "-" << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

const char* guard_failsafe_name(rovctrl::control_core::FailsafeAction action) noexcept
{
    switch (action) {
    case rovctrl::control_core::FailsafeAction::kNone:
        return "none";
    case rovctrl::control_core::FailsafeAction::kHoldOutput:
        return "hold_output";
    case rovctrl::control_core::FailsafeAction::kZeroOutput:
        return "zero_output";
    case rovctrl::control_core::FailsafeAction::kEmergencyStop:
        return "emergency_stop";
    default:
        return "unknown";
    }
}

struct ControlEventCsvLogger {
    bool init(const std::filesystem::path& path)
    {
        std::error_code ec;
        const auto dir = path.parent_path();
        if (!dir.empty() && !std::filesystem::exists(dir, ec) &&
            !std::filesystem::create_directories(dir, ec)) {
            return false;
        }

        const bool need_header = !std::filesystem::exists(path, ec) ||
                                 std::filesystem::file_size(path, ec) == 0;
        ofs_.open(path, std::ios::out | std::ios::app);
        if (!ofs_.is_open()) {
            return false;
        }

        run_id_ = resolve_run_id("pwm_control_program");
        if (need_header) {
            ofs_
                << "mono_ns,wall_time,component,event,level,run_id,process_name,pid"
                << ",fault_code,mode,requested_mode,controller,nav_present,nav_valid"
                << ",nav_stale,nav_degraded,armed,estop_latched,failsafe_action,message\n";
            ofs_.flush();
        }
        return true;
    }

    void log_guard_event(const rovctrl::control_core::GuardEvent& event,
                         std::string_view                         controller)
    {
        if (!ofs_.is_open()) {
            return;
        }

        // Guard 内部只发低频边沿事件，这里同步 append 一行 CSV。
        // 这样可以先把 reject / failsafe 结构化落地，而不把文件 I/O 塞回安全决策对象内部。
        ofs_
            << event.mono_ns
            << "," << csv_escape(wall_time_now_string())
            << "," << csv_escape(event.component != nullptr ? event.component : "")
            << "," << csv_escape(event.event != nullptr ? event.event : "")
            << "," << csv_escape(event.level != nullptr ? event.level : "")
            << "," << csv_escape(run_id_)
            << "," << csv_escape("pwm_control_program")
            << "," << ::getpid()
            << "," << event.fault_code
            << "," << csv_escape(std::string(rovctrl::control_core::to_string(event.mode)))
            << "," << csv_escape(std::string(rovctrl::control_core::to_string(event.requested_mode)))
            << "," << csv_escape(controller)
            << "," << (event.nav_present ? 1 : 0)
            << "," << (event.nav_valid ? 1 : 0)
            << "," << (event.nav_stale ? 1 : 0)
            << "," << (event.nav_degraded ? 1 : 0)
            << "," << (event.armed ? 1 : 0)
            << "," << (event.estop_latched ? 1 : 0)
            << "," << csv_escape(guard_failsafe_name(event.failsafe))
            << "," << csv_escape(event.message != nullptr ? event.message : "")
            << "\n";
        ofs_.flush();
    }

private:
    std::ofstream ofs_;
    std::string run_id_;
};

struct GuardCallbackReset final {
    rovctrl::control_core::ControlGuard& guard;

    ~GuardCallbackReset()
    {
        guard.set_event_callback({});
    }
};

} // namespace
// ================================================================

namespace rovctrl::control_core {

using clock      = std::chrono::steady_clock;
using duration_d = std::chrono::duration<double>;

int ControlLoop::run()
{
    if (!input_) {
        std::cerr << "[ControlLoop] input_ is null, cannot run.\n";
        return -2;
    }

    if (!ctrl_mgr_.has_active_controller()) {
        std::cerr << "[ControlLoop] ControllerManager has no active controller.\n";
        std::cerr << "             status.ok=" << ctrl_mgr_.status().ok
                  << " mode=" << static_cast<int>(ctrl_mgr_.status().mode)
                  << " active='" << ctrl_mgr_.status().active_controller
                  << "' err='" << ctrl_mgr_.status().last_error << "'\n";
        return -1;
    }

    if (!input_->init()) {
        std::cerr << "[ControlLoop] InputProvider::init() failed.\n";
        return -3;
    }

    // PWM log（按开关创建）
    if (cfg_.enable_pwm_log) {
        pwm_logger_ = make_pwm_logger_();
        if (!pwm_logger_ || !pwm_logger_->init("./logs",
                                               ControlLoop::PwmLog::Mode::CmdAndApplied,
                                               "pwm_log")) {
            std::cerr << "[ControlLoop] PwmLogger init failed, continue without logging.\n";
            pwm_logger_.reset();
        }
    }

    // allocator
    if (!allocator_.init(cfg_.thruster_alloc)) {
        std::cerr << "[ControlLoop] ThrusterAllocator::init() failed.\n";
        return -4;
    }
    if (!allocator_.ok()) {
        std::cerr << "[ControlLoop] ThrusterAllocator is not OK.\n";
        return -5;
    }

    double loop_hz = cfg_.loop_hz;
    if (loop_hz <= 0.0) {
        std::cerr << "[ControlLoop] invalid loop_hz=" << loop_hz << ", fallback to 100.\n";
        loop_hz = 100.0;
    }

    const auto   loop_period = duration_d(1.0 / loop_hz);
    const double dt_nom      = 1.0 / loop_hz;

    double dt_max = cfg_.dt_clamp_max_sec;
    if (dt_max <= 0.0) dt_max = 0.2;
    if (dt_max < dt_nom) dt_max = dt_nom;

    auto last_tick = clock::now();
    auto next_tick = last_tick + loop_period;
    start_time_    = last_tick;

    int step_err_count   = 0;
    int nav_miss_counter = 0;

    rovctrl::io::state::TelemetryPublisherShm telemetry_pub;
    shared::msg::TelemetryFrameV2 telemetry{};
    telemetry.source = static_cast<std::uint8_t>(shared::msg::TelemetrySource::kControlCore);

    if (cfg_.enable_telemetry_shm) {
        rovctrl::io::state::TelemetryPublisherShm::Config telemetry_cfg{};
        telemetry_cfg.enable = true;
        telemetry_cfg.shm_name = cfg_.telemetry_shm_name;
        if (!telemetry_pub.init(telemetry_cfg)) {
            std::cerr << "[ControlLoop] TelemetryPublisherShm init failed, continue without SHM telemetry.\n";
        }
    }

    rovctrl::io::ControlLoopLogger control_loop_logger;
    if ((cfg_.enable_pwm_log || cfg_.enable_telemetry_shm) &&
        !control_loop_logger.init("./logs/control", "control_loop")) {
        std::cerr << "[ControlLoop] ControlLoopLogger init failed, continue without control CSV.\n";
    }

    rovctrl::io::TelemetryTimelineLogger telemetry_logger;
    if ((cfg_.enable_pwm_log || cfg_.enable_telemetry_shm) &&
        !telemetry_logger.init("./logs/telemetry", "telemetry")) {
        std::cerr << "[ControlLoop] TelemetryTimelineLogger init failed, continue without telemetry CSV.\n";
    }

    ControlEventCsvLogger control_event_logger;
    if (!control_event_logger.init("./logs/control/control_events.csv")) {
        std::cerr << "[ControlLoop] ControlEventCsvLogger init failed, continue without structured guard events.\n";
    }

    // Guard 只负责判断“为什么拒绝/为什么进入 failsafe”，真正写 CSV 放在进程边界。
    // 这样本轮先把低频关键事件落地，不需要把刷盘逻辑塞回控制安全对象内部。
    guard_.set_event_callback([&](const GuardEvent& event) {
        control_event_logger.log_guard_event(event, ctrl_mgr_.active_controller_name());
    });
    GuardCallbackReset guard_callback_reset{guard_};

    std::cout << "[ControlLoop] starting v20260104-a, loop_hz=" << loop_hz
              << " Hz, active_controller=" << ctrl_mgr_.active_controller_name()
              << ", mode=" << static_cast<int>(ctrl_mgr_.mode()) << "\n";

    // ---------------- helpers (define once, not per-iteration) ----------------

    ThrusterArray last_thr_cmd{};
    last_thr_cmd.fill(0.0f);
    std::array<float, 8> last_pwm_duty{};
    last_pwm_duty.fill(0.0f);
    shared::msg::FaultCode last_fault_code = shared::msg::FaultCode::kNone;
    std::uint64_t telemetry_event_seq = 0;
    std::uint64_t last_accepted_cmd_seq = 0;
    std::uint64_t last_executed_cmd_seq = 0;
    std::uint64_t last_expired_cmd_seq = 0;
    std::uint64_t last_failed_cmd_seq = 0;
    bool last_armed_state = false;
    bool last_estop_state = false;
    bool last_motor_test_state = false;
    ControlMode last_mode_state = ctrl_mgr_.mode();

    auto push_event = [&](shared::msg::EventCode event_code,
                          shared::msg::FaultCode fault_code,
                          int arg0,
                          int arg1) {
        shared::msg::EventRecord rec{};
        rec.seq = ++telemetry_event_seq;
        rec.stamp_ns = now_mono_ns_();
        rec.event_code = static_cast<std::uint16_t>(event_code);
        rec.fault_code = static_cast<std::uint16_t>(fault_code);
        rec.arg0 = arg0;
        rec.arg1 = arg1;
        telemetry.last_event = rec;

        const std::size_t idx =
            static_cast<std::size_t>(telemetry.event_head % shared::msg::kTelemetryEventHistory);
        telemetry.events[idx] = rec;
        telemetry.event_head += 1;
        if (telemetry.event_count < shared::msg::kTelemetryEventHistory) {
            telemetry.event_count += 1;
        }
    };

    auto set_command_result = [&](shared::msg::CommandResultCode status,
                                  const ControlIntent& cmd,
                                  shared::msg::EventCode event_code,
                                  shared::msg::FaultCode fault_code) {
        telemetry.last_command_result.intent_id =
            (cmd.intent_id != 0) ? cmd.intent_id : cmd.cmd_seq;
        telemetry.last_command_result.cmd_seq = cmd.cmd_seq;
        telemetry.last_command_result.stamp_ns = now_mono_ns_();
        telemetry.last_command_result.event_code = static_cast<std::uint16_t>(event_code);
        telemetry.last_command_result.fault_code = static_cast<std::uint16_t>(fault_code);
        telemetry.last_command_result.status = static_cast<std::uint8_t>(status);
        telemetry.last_command_result.source = telemetry_control_source(cmd.source_id);
    };

    auto publish_telemetry = [&](const ControlIntent& applied_intent,
                                 const rovctrl::io::NavStateView* nav_snapshot,
                                 std::uint32_t nav_age_ms_total) {
        rovctrl::platform::PwmTransportStats transport_stats{};
        const bool have_transport_stats = pwm_.getTransportStats(transport_stats);
        const bool pwm_ok = pwm_.is_ok();
        TelemetryBuildInput build_input{};
        build_input.stamp_ns = now_mono_ns_();
        build_input.applied_intent = applied_intent;
        build_input.applied_reference = ref_;
        build_input.current_state = state_;
        build_input.active_mode = ctrl_mgr_.mode();
        build_input.controller_status = ctrl_mgr_.status();
        build_input.auto_fail_limit = ctrl_mgr_.auto_fail_limit();
        build_input.armed = guard_.armed();
        build_input.estop_latched = guard_.estop_latched();
        build_input.failsafe_active =
            (guard_result_.failsafe != FailsafeAction::kNone ||
             ctrl_mgr_.mode() == ControlMode::kFailsafe);
        build_input.input_stale = guard_result_.input_stale;
        build_input.thruster_cmd = last_thr_cmd;
        build_input.pwm_duty = last_pwm_duty;
        build_input.pwm_ok = pwm_ok;
        build_input.have_transport_stats = have_transport_stats;
        build_input.transport_stats = transport_stats;
        build_input.nav_snapshot = nav_snapshot;
        build_input.nav_age_ms = nav_age_ms_total;
        build_input.last_fault_code = last_fault_code;

        fill_telemetry_frame_v2(telemetry, build_input);

        if (control_loop_logger.is_open()) {
            rovctrl::io::ControlEffect eff{};
            eff.surge = static_cast<float>(applied_intent.teleop_dof_cmd.surge);
            eff.sway = static_cast<float>(applied_intent.teleop_dof_cmd.sway);
            eff.heave = static_cast<float>(applied_intent.teleop_dof_cmd.heave);
            eff.roll = static_cast<float>(applied_intent.teleop_dof_cmd.roll);
            eff.pitch = static_cast<float>(applied_intent.teleop_dof_cmd.pitch);
            eff.yaw = static_cast<float>(applied_intent.teleop_dof_cmd.yaw);
            eff.has_ref = applied_intent.has_ref;
            eff.has_ref_delta = applied_intent.has_ref_delta;
            eff.request_exit = applied_intent.request_exit;
            eff.intent_age_ms =
                (applied_intent.stamp_ns > 0 && now_mono_ns_() >= applied_intent.stamp_ns)
                    ? static_cast<std::uint32_t>((now_mono_ns_() - applied_intent.stamp_ns) / 1000000ull)
                    : 0u;

            rovctrl::io::ControlGuardOutput guard_out{};
            guard_out.armed = guard_.armed();
            guard_out.estop_latched = guard_.estop_latched();
            guard_out.effective_mode = static_cast<int>(ctrl_mgr_.mode());
            guard_out.has_nav = (nav_snapshot != nullptr);
            guard_out.failsafe =
                (guard_result_.failsafe != FailsafeAction::kNone ||
                 ctrl_mgr_.mode() == ControlMode::kFailsafe);
            guard_out.input_stale = guard_result_.input_stale;

            rovctrl::io::NavigationData nav_data{};
            nav_data.x = state_.nav_pos_ned[0];
            nav_data.y = state_.nav_pos_ned[1];
            nav_data.z = state_.nav_pos_ned[2];
            nav_data.roll = state_.nav_rpy[0];
            nav_data.pitch = state_.nav_rpy[1];
            nav_data.yaw = state_.nav_rpy[2];
            nav_data.depth_m = state_.nav_depth;
            nav_data.present = state_.nav_present;
            nav_data.valid = state_.nav_valid;
            nav_data.stale = state_.nav_stale;
            nav_data.degraded = state_.nav_degraded;
            nav_data.age_ms = state_.nav_age_ms;
            nav_data.nav_state = static_cast<std::uint8_t>(state_.nav_state);
            nav_data.nav_health = static_cast<std::uint8_t>(state_.nav_health);
            nav_data.fault_code = static_cast<std::uint16_t>(state_.nav_fault_code);
            nav_data.sensor_mask = state_.nav_sensor_mask;
            nav_data.status_flags = state_.nav_status_flags;

            control_loop_logger.log_data(
                duration_d(clock::now() - start_time_).count(), eff, guard_out, nav_data);
        }

        if (telemetry_logger.is_open()) {
            telemetry_logger.log_frame(telemetry);
        }

        if (telemetry_pub.initialized()) {
            (void)telemetry_pub.publish(telemetry);
        }
    };

    auto log_pwm_cmd_applied = [&](double t_s, const ThrusterArray& thr_cmd) {
        if (!pwm_logger_ || !pwm_logger_->is_open()) return;

        std::array<float, 8> cmd{};
        std::array<float, 8> applied{};
        for (std::size_t i = 0; i < 8; ++i) {
            cmd[i]     = thr_cmd[i];
            applied[i] = thr_cmd[i]; // TODO: 若能读到 safety-layer applied，替换
        }
        pwm_logger_->logCmdAndApplied(t_s, cmd, applied);
    };

    auto neutral_and_step = [&](double t_s) {
        ThrusterArray thr_cmd{};
        thr_cmd.fill(0.0f); // 0 -> neutral
        (void)pwm_.setTargets(thr_cmd);
        (void)pwm_.step();
        last_thr_cmd = thr_cmd;
        (void)pwm_.getLastApplied(last_pwm_duty);
        log_pwm_cmd_applied(t_s, thr_cmd);
    };

    auto has_active_command_from_intent = [&](const ControlIntent& in) -> bool {
        // 1) 任何显式“非运动”命令都视为 active，避免被归中逻辑吞掉
        if (in.request_exit) return true;
        if (in.has_estop_cmd) return true;  // estop / clear_estop
        if (in.has_arm_cmd)   return true;  // arm / disarm
        if (in.has_mode_request && in.mode_request != ControlMode::kNone) return true;

        if (in.has_ref)       return true;
        if (in.has_ref_delta) return true;

        // 1.5) 单电机测试：只要有有效 motor_test 也视为 active
        if (in.has_motor_test && in.motor_test.enable != 0) return true;

        // 2) 遥控输入：只有当 DOF 确实“非零”时，才认为 active
        if (in.has_teleop_dof) {
            constexpr double eps = 0.02; // 抑制抖动/浮点噪声
            const auto absd = [](double x) { return (x >= 0.0) ? x : -x; };

            const auto& c = in.teleop_dof_cmd;
            if (absd(c.surge) > eps) return true;
            if (absd(c.sway)  > eps) return true;
            if (absd(c.heave) > eps) return true;
            if (absd(c.roll)  > eps) return true;
            if (absd(c.pitch) > eps) return true;
            if (absd(c.yaw)   > eps) return true;

            // DOF 全部接近 0：视为“无外界运动命令”
            return false;
        }

        // 3) 其他情况：无外界输入/命令
        return false;
    };

    double no_input_sec = 0.0;
    const double no_input_neutral_sec =
        (cfg_.no_input_neutral_ms > 0) ? (cfg_.no_input_neutral_ms / 1000.0) : 0.2;

    // ---------------- main loop ----------------
    while (true) {
        // 外部退出（Ctrl+C / UI quit 等）
        if (external_stop_ && external_stop_->load()) {
            const double t_s = duration_d(clock::now() - start_time_).count();
            std::cout << "[ControlLoop] external stop flag set, exiting loop.\n";
            neutral_and_step(t_s); // 退出前归中一次
            break;
        }

        // 固定周期调度
        auto now = clock::now();
        if (now < next_tick) {
            std::this_thread::sleep_until(next_tick);
            now = clock::now();
        } else {
            const auto lag = duration_d(now - next_tick).count();
            if (lag > 5.0 * dt_nom) {
                next_tick = now; // large lag: reset schedule anchor
            }
        }

        double dt = duration_d(now - last_tick).count();
        last_tick = now;
        next_tick += loop_period;

        if (dt <= 0.0) dt = dt_nom;
        if (dt > dt_max) dt = dt_max;

        const double t_s = duration_d(now - start_time_).count();
        state_.timestamp_sec = t_s;

        // ---------------- nav update (B2: NavStateView) ----------------
        rovctrl::io::NavStateView nav_view{};              // per-cycle snapshot
        const bool nav_present = update_nav_feedback_(nav_view);
        const bool nav_usable = state_.nav_valid;
        const std::uint32_t nav_age_ms_total =
            nav_present ? nav_view.total_age_ms() : 0u;

        // 只有 Auto 模式要求导航（Manual/Teleop 不需要）
        const ControlMode mode_now = ctrl_mgr_.mode();
        const bool nav_required = (mode_now == ControlMode::kAuto);

        if (!nav_present) {
            ++nav_miss_counter;

            if (nav_required) {
                if (!cfg_.allow_run_without_nav) {
                    std::cerr << "[ControlLoop] NavView missing in AUTO mode and allow_run_without_nav=false, entering failsafe.\n";
                    execute_failsafe_(FailsafeAction::kEmergencyStop);
                    last_thr_cmd.fill(0.0f);
                    (void)pwm_.getLastApplied(last_pwm_duty);
                    last_fault_code = shared::msg::FaultCode::kNavUntrusted;
                    publish_telemetry(ControlIntent{}, nullptr, 0);
                    return -8;
                }

                if (nav_miss_counter == 100 ||
                    (cfg_.step_error_log_interval > 0 &&
                     nav_miss_counter % cfg_.step_error_log_interval == 0)) {
                    std::cerr << "[ControlLoop] Warning(AUTO): no stable NavView for "
                              << nav_miss_counter << " cycles.\n";
                }
            } else {
                // Manual / Failsafe：忽略导航缺失，避免干扰遥控
                if (nav_miss_counter == 1) {
                    std::cout << "[ControlLoop] NavView missing (ignored in mode="
                              << static_cast<int>(mode_now) << ").\n";
                }
            }
        } else if (!nav_usable) {
            ++nav_miss_counter;
            if (nav_required &&
                (nav_miss_counter == 1 ||
                 (cfg_.step_error_log_interval > 0 &&
                  nav_miss_counter % cfg_.step_error_log_interval == 0))) {
                std::cerr << "[ControlLoop] NavView present but untrusted in AUTO mode: "
                          << "state=" << static_cast<int>(nav_view.payload().nav_state)
                          << " stale=" << int(nav_view.payload().stale)
                          << " fault=" << static_cast<int>(nav_view.payload().fault_code)
                          << " age_ms=" << nav_age_ms_total
                          << "\n";
            }
        } else {
            nav_miss_counter = 0;
        }

        // ---------------- poll input ----------------
        ControlIntent intent{};
        if (!input_->poll(state_, intent)) {
            std::cerr << "[ControlLoop] InputProvider::poll(state,intent) failed.\n";
            execute_failsafe_(FailsafeAction::kEmergencyStop);
            last_thr_cmd.fill(0.0f);
            (void)pwm_.getLastApplied(last_pwm_duty);
            last_fault_code = shared::msg::FaultCode::kCommFault;
            publish_telemetry(ControlIntent{}, nav_present ? &nav_view : nullptr, nav_age_ms_total);
            return -10;
        }

        if (intent.valid && intent.cmd_seq != 0 && intent.cmd_seq != last_accepted_cmd_seq) {
            last_accepted_cmd_seq = intent.cmd_seq;
            set_command_result(shared::msg::CommandResultCode::kAccepted,
                               intent,
                               shared::msg::EventCode::kIntentAccepted,
                               shared::msg::FaultCode::kNone);
            push_event(shared::msg::EventCode::kIntentAccepted,
                       shared::msg::FaultCode::kNone,
                       static_cast<int>(intent.cmd_seq & 0x7fffffff),
                       0);
        }
        // 调试：低频打印 Intent 的 6DOF（只要有 has_teleop_dof）
        // static int intent_debug_counter = 0;
        // if (intent.has_teleop_dof && (++intent_debug_counter % 50 == 0)) {
        //     const auto& c = intent.teleop_dof_cmd;
        //     std::cout << "[ControlLoop][INTENT] teleop_dof "
        //               << "s="  << c.surge
        //               << " sw=" << c.sway
        //               << " h="  << c.heave
        //               << " r="  << c.roll
        //               << " p="  << c.pitch
        //               << " y="  << c.yaw
        //               << "\n";
        // }

        // input 请求退出
        if (intent.request_exit) {
            std::cout << "[ControlLoop] Input provider requested exit.\n";
            neutral_and_step(t_s);
            publish_telemetry(intent, nav_present ? &nav_view : nullptr, nav_age_ms_total);
            break;
        }

        // 看门狗：无有效输入则归中，否则计时清零。判定是否故障，如果故障，则持续归中。进入故障日志只打印一次，恢复正常也打印一次。
        {
            const bool active = has_active_command_from_intent(intent);

            // 1) 累积 / 清零无输入时间
            static bool s_watchdog_fault = false;  // 当前是否处于“无输入故障状态”

            if (!active) {
                no_input_sec += dt;
            } else {
                // 有有效输入：如果之前在故障状态，这里打印一次“恢复正常”
                if (s_watchdog_fault) {
                    std::cout << "[ControlLoop][WATCHDOG] input_recovered, "
                              << "resume normal control. no_input_sec=" << no_input_sec
                              << "\n";
                }
                no_input_sec      = 0.0;
                s_watchdog_fault  = false;
            }

            // 2) 判定当前是否进入故障区间
            const bool fault_now = (no_input_sec >= no_input_neutral_sec);

            // 2.1 从“正常 → 故障”边沿：只打印一次
            if (fault_now && !s_watchdog_fault) {
                std::cout << "[ControlLoop][WATCHDOG] no_active_cmd for "
                          << no_input_sec << "s (>= "
                          << no_input_neutral_sec
                          << "s), engaging neutral_and_step.\n";
                s_watchdog_fault = true;
            }

            // 3) 只要处于故障状态，就持续执行 neutral_and_step
            if (fault_now) {
                neutral_and_step(t_s);
                publish_telemetry(intent, nav_present ? &nav_view : nullptr, nav_age_ms_total);
                continue;   // 本周期不再做控制器计算
            }

            // 走到这里说明：
            //   - 要么一直有输入（永不进入故障）；
            //   - 要么刚从故障恢复（上面已经打印过恢复日志）。
        }

        // ---------------- guard ----------------
        // 本轮是否有导航数据：在整个循环体内都要用到，所以放在 block 外面
        bool has_nav = false;
        {
            const std::uint64_t now_ns = now_mono_ns_();

            // Path A/B2: Guard expects NavStateView*, not NavState*
            const shared::msg::NavStateView* nav_ptr =
                (nav_present ? &nav_view.payload() : nullptr);
            has_nav = (nav_ptr != nullptr);

            guard_result_ = guard_.step(now_ns, state_, nav_ptr, intent);

            if (guard_result_.input_stale &&
                intent.cmd_seq != 0 &&
                intent.cmd_seq != last_expired_cmd_seq) {
                last_expired_cmd_seq = intent.cmd_seq;
                last_fault_code = shared::msg::FaultCode::kIntentStale;
                set_command_result(shared::msg::CommandResultCode::kExpired,
                                   intent,
                                   shared::msg::EventCode::kIntentExpired,
                                   last_fault_code);
                push_event(shared::msg::EventCode::kIntentExpired,
                           last_fault_code,
                           static_cast<int>(intent.cmd_seq & 0x7fffffff),
                           0);
            }

            if (intent.has_mode_request &&
                intent.mode_request != ControlMode::kNone &&
                !guard_result_.input_stale &&
                guard_result_.effective_mode != intent.mode_request &&
                intent.cmd_seq != last_failed_cmd_seq) {
                last_failed_cmd_seq = intent.cmd_seq;
                last_fault_code = shared::msg::FaultCode::kIllegalStateTransition;
                set_command_result(shared::msg::CommandResultCode::kRejected,
                                   intent,
                                   shared::msg::EventCode::kIntentFailed,
                                   last_fault_code);
                push_event(shared::msg::EventCode::kIntentFailed,
                           last_fault_code,
                           static_cast<int>(intent.cmd_seq & 0x7fffffff),
                           static_cast<int>(guard_result_.effective_mode));
            }

            if (intent.has_motor_test &&
                intent.motor_test.enable &&
                !guard_result_.effective_intent.has_motor_test &&
                intent.cmd_seq != last_failed_cmd_seq) {
                last_failed_cmd_seq = intent.cmd_seq;
                last_fault_code = shared::msg::FaultCode::kMotorTestViolation;
                set_command_result(shared::msg::CommandResultCode::kRejected,
                                   intent,
                                   shared::msg::EventCode::kMotorTestRejected,
                                   last_fault_code);
                push_event(shared::msg::EventCode::kMotorTestRejected,
                           last_fault_code,
                           static_cast<int>(intent.motor_test.motor_id),
                           0);
            }

            // Guard owns the authoritative safety decision. ControlLoop must not
            // silently downgrade FAILSAFE/ZeroOutput decisions based on local demos.

            if (guard_result_.effective_intent.request_exit) {
                std::cout << "[ControlLoop] Guard requested exit.\n";
                neutral_and_step(t_s);
                publish_telemetry(guard_result_.effective_intent,
                                  nav_present ? &nav_view : nullptr,
                                  nav_age_ms_total);
                break;
            }

            if (guard_result_.failsafe != FailsafeAction::kNone) {
                execute_failsafe_(guard_result_.failsafe);
                last_thr_cmd.fill(0.0f);
                (void)pwm_.getLastApplied(last_pwm_duty);
                last_fault_code = (guard_result_.failsafe == FailsafeAction::kEmergencyStop)
                    ? shared::msg::FaultCode::kSessionFault
                    : (guard_result_.input_stale
                        ? shared::msg::FaultCode::kIntentStale
                        : shared::msg::FaultCode::kIllegalStateTransition);
                push_event(shared::msg::EventCode::kFailsafeEntered,
                           last_fault_code,
                           static_cast<int>(guard_result_.failsafe),
                           0);
                publish_telemetry(guard_result_.effective_intent,
                                  nav_present ? &nav_view : nullptr,
                                  nav_age_ms_total);
                continue;
            }

            if (guard_result_.mode_changed) {
                if (!ctrl_mgr_.set_mode(guard_result_.effective_mode)) {
                    last_fault_code = shared::msg::FaultCode::kControllerUnavailable;
                    set_command_result(shared::msg::CommandResultCode::kFailed,
                                       intent,
                                       shared::msg::EventCode::kIntentFailed,
                                       last_fault_code);
                    push_event(shared::msg::EventCode::kIntentFailed,
                               last_fault_code,
                               static_cast<int>(intent.cmd_seq & 0x7fffffff),
                               static_cast<int>(guard_result_.effective_mode));
                    execute_failsafe_(FailsafeAction::kZeroOutput);
                    last_thr_cmd.fill(0.0f);
                    (void)pwm_.getLastApplied(last_pwm_duty);
                    publish_telemetry(guard_result_.effective_intent,
                                      nav_present ? &nav_view : nullptr,
                                      nav_age_ms_total);
                    continue;
                }
                push_event(shared::msg::EventCode::kModeChanged,
                           shared::msg::FaultCode::kNone,
                           static_cast<int>(guard_result_.effective_mode),
                           0);
            }
        }

        if (guard_.armed() != last_armed_state) {
            push_event(shared::msg::EventCode::kArmChanged,
                       shared::msg::FaultCode::kNone,
                       guard_.armed() ? 1 : 0,
                       0);
            last_armed_state = guard_.armed();
        }
        if (guard_.estop_latched() != last_estop_state) {
            push_event(guard_.estop_latched()
                           ? shared::msg::EventCode::kEstopLatched
                           : shared::msg::EventCode::kEstopCleared,
                       shared::msg::FaultCode::kNone,
                       guard_.estop_latched() ? 1 : 0,
                       0);
            last_estop_state = guard_.estop_latched();
        }
        if (guard_result_.effective_intent.has_motor_test != last_motor_test_state) {
            push_event(guard_result_.effective_intent.has_motor_test
                           ? shared::msg::EventCode::kMotorTestStarted
                           : shared::msg::EventCode::kMotorTestStopped,
                       shared::msg::FaultCode::kNone,
                       guard_result_.effective_intent.has_motor_test
                           ? static_cast<int>(guard_result_.effective_intent.motor_test.motor_id)
                           : 0,
                       0);
            last_motor_test_state = guard_result_.effective_intent.has_motor_test;
        }
        if (ctrl_mgr_.mode() != last_mode_state) {
            last_mode_state = ctrl_mgr_.mode();
        }

        build_reference_from_guard_();

        // 调试：低频打印 Effective Intent 的 6DOF（只要有 has_teleop_dof）
        const auto& eff = guard_result_.effective_intent;

        // 调试：打印 guard 之后“生效的 Intent”
        if (eff.has_teleop_dof) {
            std::cout << "[ControlLoop][INTENT_EFF] teleop_dof "
                      << "s=" << eff.teleop_dof_cmd.surge
                      << " sw=" << eff.teleop_dof_cmd.sway
                      << " h=" << eff.teleop_dof_cmd.heave
                      << " r=" << eff.teleop_dof_cmd.roll
                      << " p=" << eff.teleop_dof_cmd.pitch
                      << " y=" << eff.teleop_dof_cmd.yaw
                      << " | has_ref=" << eff.has_ref
                      << " has_ref_delta=" << eff.has_ref_delta
                      << " has_motor_test=" << eff.has_motor_test
                      << " request_exit=" << eff.request_exit
                      << "\n";
        } else {
            std::cout << "[ControlLoop][INTENT_EFF] has_teleop_dof=0"
                      << " | has_ref=" << eff.has_ref
                      << " has_ref_delta=" << eff.has_ref_delta
                      << " has_motor_test=" << eff.has_motor_test
                      << " request_exit=" << eff.request_exit
                      << "\n";
        }       

        // ---------------- controller compute ----------------
        output_ = ControlOutput{};
        if (!ctrl_mgr_.compute(state_, ref_, output_, dt)) {
            std::cerr << "[ControlLoop] ControllerManager::compute() failed: "
                      << ctrl_mgr_.status().last_error << "\n";
            last_fault_code = shared::msg::FaultCode::kControllerComputeFailed;
            if (intent.cmd_seq != 0) {
                set_command_result(shared::msg::CommandResultCode::kFailed,
                                   intent,
                                   shared::msg::EventCode::kIntentFailed,
                                   last_fault_code);
            }

            if (cfg_.enter_failsafe_on_controller_error) {
                execute_failsafe_(FailsafeAction::kZeroOutput);
                last_thr_cmd.fill(0.0f);
                (void)pwm_.getLastApplied(last_pwm_duty);
                push_event(shared::msg::EventCode::kFailsafeEntered,
                           last_fault_code,
                           static_cast<int>(ctrl_mgr_.mode()),
                           0);
                publish_telemetry(guard_result_.effective_intent,
                                  nav_present ? &nav_view : nullptr,
                                  nav_age_ms_total);
                continue;
            }
            publish_telemetry(guard_result_.effective_intent,
                              nav_present ? &nav_view : nullptr,
                              nav_age_ms_total);
            return -20;
        }
        // 调试：只在 teleop 有 DOF 时输出一行
        // if (guard_result_.effective_intent.has_teleop_dof) {
        //     const auto& st = ctrl_mgr_.status();
        //     std::cout << "[ControlLoop][DBG] after compute: "
        //               << "mode=" << static_cast<int>(st.mode)
        //               << " active=" << st.active_controller
        //               << " last_ok=" << st.last_compute_ok
        //               << " has_thr=" << int(output_.has_thruster_command)
        //               << " has_wrench=" << int(output_.has_body_wrench)
        //               << "\n";
        // }
        // 这里就可以放心使用 has_nav 了
        if (guard_result_.effective_intent.has_teleop_dof &&
            !output_.has_body_wrench &&
            !output_.has_thruster_command)
        {
            std::cout << "[ControlLoop][WARN] teleop_dof present but controller produced no output "
                      << "(has_nav=" << (has_nav ? 1 : 0) << ")\n";
        }

        // 调试：查看 controller 输出的 ControlOutput
        // std::cout << "[ControlLoop][OUT] "
        //           << "has_thruster=" << int(output_.has_thruster_command)
        //           << " has_wrench="  << int(output_.has_body_wrench)
        //           << "\n";

        // ---------------- thruster command ----------------
        ThrusterArray thr_cmd{};
        const bool have_thr_cmd = build_thruster_command_(thr_cmd);

        if (!have_thr_cmd) {
            // 没有任何有效的控制输出：统一归零，并低频打印一次告警
            static std::uint64_t last_warn_ns = 0;
            const std::uint64_t now_ns = now_mono_ns_();

            if (last_warn_ns == 0 || (now_ns - last_warn_ns) > 1'000'000'000ull) {
                std::cout << "[ControlLoop][ALLOC] no valid thruster command, fallback to zero.\n";
                last_warn_ns = now_ns;
            }
            thr_cmd.fill(0.0f);
        }

        // 可选：打印最终“将要下发”的 thr_cmd（统一视角）
        // if (cfg_.enable_pwm_log) {
        //     std::cout << "[ControlLoop][OUT] thruster_cmd="
        //               << thr_cmd[0] << ", "
        //               << thr_cmd[1] << ", "
        //               << thr_cmd[2] << ", "
        //               << thr_cmd[3] << ", "
        //               << thr_cmd[4] << ", "
        //               << thr_cmd[5] << ", "
        //               << thr_cmd[6] << ", "
        //               << thr_cmd[7] << "\n";
        // }

        // 单电机测试覆盖逻辑（在所有正常控制输出之后）
        apply_motor_test_override(guard_result_.effective_intent, thr_cmd);

        // 推进器活动小仪表（只在非零且节流满足时打印）
        log_thruster_activity(thr_cmd);

        // 实际下发到 PWM 客户端
        {
            std::cout << "[ControlLoop][THR_CMD] "
                      << "u0=" << thr_cmd[0]
                      << " u1=" << thr_cmd[1]
                      << " u2=" << thr_cmd[2]
                      << " u3=" << thr_cmd[3]
                      << " u4=" << thr_cmd[4]
                      << " u5=" << thr_cmd[5]
                      << " u6=" << thr_cmd[6]
                      << " u7=" << thr_cmd[7]
                      << "\n";

            const int rc = pwm_.setTargets(thr_cmd);
            if (rc < 0) {
                std::cerr << "[ControlLoop] pwm_.setTargets() rc=" << rc
                          << " msg=" << pwm_.status().last_error_msg << "\n";
                last_fault_code = shared::msg::FaultCode::kPwmStepFailed;
            }
        }

        const int step_rc = pwm_.step();
        if (step_rc < 0) {
            ++step_err_count;
            last_fault_code = shared::msg::FaultCode::kPwmStepFailed;

            if (step_err_count <= 3 ||
                (cfg_.step_error_log_interval > 0 &&
                 step_err_count % cfg_.step_error_log_interval == 0)) {
                std::cerr << "[ControlLoop] pwm_.step() rc=" << step_rc
                          << " msg=" << pwm_.status().last_error_msg
                          << " (error count=" << step_err_count << ")\n";
            }

            if (cfg_.max_step_errors > 0 && step_err_count > cfg_.max_step_errors) {
                std::cerr << "[ControlLoop] pwm_.step() errors exceed max_step_errors="
                          << cfg_.max_step_errors << ", entering failsafe and abort.\n";
                execute_failsafe_(FailsafeAction::kEmergencyStop);
                last_thr_cmd.fill(0.0f);
                (void)pwm_.getLastApplied(last_pwm_duty);
                publish_telemetry(guard_result_.effective_intent,
                                  nav_present ? &nav_view : nullptr,
                                  nav_age_ms_total);
                return -30;
            }
            (void)pwm_.getLastApplied(last_pwm_duty);
            publish_telemetry(guard_result_.effective_intent,
                              nav_present ? &nav_view : nullptr,
                              nav_age_ms_total);
            continue;  // 这轮循环失败，跳到下一轮
        }

        step_err_count = 0;
        last_fault_code = shared::msg::FaultCode::kNone;
        last_thr_cmd = thr_cmd;
        (void)pwm_.getLastApplied(last_pwm_duty);
        log_pwm_cmd_applied(t_s, thr_cmd);

        if (intent.cmd_seq != 0 &&
            intent.cmd_seq != last_executed_cmd_seq &&
            telemetry_intent_has_payload(guard_result_.effective_intent))
        {
            last_executed_cmd_seq = intent.cmd_seq;
            set_command_result(shared::msg::CommandResultCode::kExecuted,
                               intent,
                               shared::msg::EventCode::kIntentExecuted,
                               shared::msg::FaultCode::kNone);
            push_event(shared::msg::EventCode::kIntentExecuted,
                       shared::msg::FaultCode::kNone,
                       static_cast<int>(intent.cmd_seq & 0x7fffffff),
                       0);
        }

        publish_telemetry(guard_result_.effective_intent,
                          nav_present ? &nav_view : nullptr,
                          nav_age_ms_total);

        // Optional debug hook: 当前总导航 age 已统一收敛到 nav_age_ms_total。
        (void)nav_age_ms_total;
    } // <-- 这里是 while(...) 主控制循环 的结尾大括号

    // 正常退出：再保险归中一次（比无条件 E-Stop 更符合“退出键结束程序”的语义）
    neutral_and_step(duration_d(clock::now() - start_time_).count());
    publish_telemetry(ControlIntent{}, nullptr, 0);

    std::cout << "[ControlLoop] loop exited normally.\n";
    return 0;
}

} // namespace rovctrl::control_core
