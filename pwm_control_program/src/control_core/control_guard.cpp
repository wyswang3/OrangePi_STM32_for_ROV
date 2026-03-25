#include "control_core/control_guard.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>

// 只在 cpp 里依赖 nav_state / telemetry fault 的完整定义。
#include "shared/msg/nav_state.hpp"
#include "shared/msg/telemetry_frame_v2.hpp"

// 做法 A：ControlIntent/ControlMode 真源在 control_core。
#include "control_core/control_intent.hpp"
#include "control_core/control_mode.hpp"

namespace rovctrl::control_core {

static inline double clampd(double v, double lo, double hi) noexcept
{
    return std::max(lo, std::min(v, hi));
}

static inline double absd(double x) noexcept { return x < 0 ? -x : x; }

static bool nav_ready_for_auto(const shared::msg::NavStateView& nav) noexcept
{
    const bool lifecycle_ok =
        (nav.nav_state == shared::msg::NavRunState::kOk) ||
        (nav.nav_state == shared::msg::NavRunState::kDegraded);
    if (!lifecycle_ok) {
        return false;
    }
    if (nav.valid == 0 || nav.stale != 0) {
        return false;
    }
    if (nav.health == shared::msg::NavHealth::INVALID ||
        nav.health == shared::msg::NavHealth::UNINITIALIZED) {
        return false;
    }
    if (nav.fault_code != shared::msg::NavFaultCode::kNone) {
        return false;
    }

    const std::uint16_t flags = nav.status_flags;
    const bool imu_ok = shared::msg::nav_flag_has(flags, shared::msg::NAV_FLAG_IMU_OK);
    const bool align_done = shared::msg::nav_flag_has(flags, shared::msg::NAV_FLAG_ALIGN_DONE);
    const bool eskf_ok = shared::msg::nav_flag_has(flags, shared::msg::NAV_FLAG_ESKF_OK);
    return imu_ok && align_done && eskf_ok;
}

static inline std::uint16_t control_fault_u16(shared::msg::FaultCode code) noexcept
{
    return static_cast<std::uint16_t>(code);
}

static inline const char* failsafe_name(FailsafeAction action) noexcept
{
    switch (action) {
    case FailsafeAction::kNone:
        return "none";
    case FailsafeAction::kHoldOutput:
        return "hold_output";
    case FailsafeAction::kZeroOutput:
        return "zero_output";
    case FailsafeAction::kEmergencyStop:
        return "emergency_stop";
    default:
        return "unknown";
    }
}

static std::uint16_t guard_fault_code_for_state(const shared::msg::NavStateView* nav,
                                                bool                             input_stale,
                                                bool                             estop_latched,
                                                ControlMode                      mode_eff) noexcept
{
    if (estop_latched) {
        return control_fault_u16(shared::msg::FaultCode::kSessionFault);
    }
    if (input_stale) {
        return control_fault_u16(shared::msg::FaultCode::kIntentStale);
    }
    if (mode_eff == ControlMode::kAuto &&
        (nav == nullptr || nav->valid == 0 || nav->stale != 0 ||
         nav->fault_code != shared::msg::NavFaultCode::kNone)) {
        return control_fault_u16(shared::msg::FaultCode::kNavUntrusted);
    }
    if (mode_eff == ControlMode::kFailsafe) {
        return control_fault_u16(shared::msg::FaultCode::kIllegalStateTransition);
    }
    return control_fault_u16(shared::msg::FaultCode::kNone);
}

ControlGuard::ControlGuard(ControlGuardConfig cfg)
    : cfg_(cfg)
{
}

void ControlGuard::set_event_callback(GuardEventCallback cb)
{
    event_callback_ = std::move(cb);
}

void ControlGuard::reset()
{
    armed_ = false;
    estop_latched_ = false;

    // 做法 A：ControlMode 在 control_core。
    mode_ = ControlMode::kManual;

    last_intent_ns_ = 0;
    last_intent_cmd_seq_ = 0;
    input_age_ms_ = 0;
    clear_hold_start_ns_ = 0;
    clear_hold_ms_ = 0;
    motor_test_active_ = false;
    motor_test_deadline_ns_ = 0;
    motor_test_cmd_seq_ = 0;
    latched_motor_test_ = MotorTestCmd{};
    have_last_nav_gating_state_ = false;
    last_nav_gating_ok_ = false;
    last_failsafe_action_ = FailsafeAction::kNone;
    last_reject_cmd_seq_ = 0;
    last_reject_tag_ = nullptr;
}

// -----------------------------------------------------------------------------
// “输入过期”判定（做法 A 推荐实现）：
//  - 优先使用 intent.stamp_ns + ttl_ms（若 stamp_ns==0，则退化到 now_ns）
//  - ttl_ms==0：使用 intent.ttl_ms=0 表示“让 Guard 用 cfg_.default_ttl_ms”
//  - cfg_.default_ttl_ms==0：禁用超时
// -----------------------------------------------------------------------------
bool ControlGuard::is_intent_stale(std::uint64_t now_ns,
                                   const ControlIntent& intent) const
{
    const std::uint32_t ttl_ms =
        (intent.ttl_ms != 0) ? intent.ttl_ms : cfg_.default_ttl_ms;

    if (ttl_ms == 0) {
        return false;
    }

    const std::uint64_t t0 = (intent.stamp_ns != 0) ? intent.stamp_ns : last_intent_ns_;
    const std::uint64_t t1 = (now_ns != 0) ? now_ns : t0;

    if (t1 < t0) {
        return false;
    }

    const std::uint64_t age_ns = (t1 - t0);
    const std::uint64_t age_ms = age_ns / 1000000ull;
    return age_ms > static_cast<std::uint64_t>(ttl_ms);
}

// -----------------------------------------------------------------------------
// 模式门控（做法 A 收敛）：
//  - Manual 永远允许
//  - Auto 依赖 nav（若 enable_mode_gating==true）
//  - Failsafe 永远允许（安全态）
// -----------------------------------------------------------------------------
bool ControlGuard::nav_ok_for_mode(const shared::msg::NavStateView* nav,
                                   ControlMode requested) const
{
    if (!cfg_.enable_mode_gating) {
        return true;
    }

    switch (requested) {
    case ControlMode::kManual:
        return true;
    case ControlMode::kAuto:
        return nav != nullptr && nav_ready_for_auto(*nav);
    case ControlMode::kFailsafe:
        return true;
    default:
        return nav != nullptr && nav_ready_for_auto(*nav);
    }
}

ControlMode ControlGuard::downgrade_mode(ControlMode requested) const
{
    switch (requested) {
    case ControlMode::kAuto:
        return ControlMode::kFailsafe;
    case ControlMode::kFailsafe:
        return ControlMode::kFailsafe;
    case ControlMode::kManual:
    case ControlMode::kNone:
    case ControlMode::kUnknown:
    default:
        return ControlMode::kManual;
    }
}

void ControlGuard::clamp_teleop(ControlIntent& inout) const
{
    if (!inout.has_teleop_dof) return;

    auto& c = inout.teleop_dof_cmd;
    c.surge = clampd(c.surge, cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.sway = clampd(c.sway, cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.heave = clampd(c.heave, cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.roll = clampd(c.roll, cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.pitch = clampd(c.pitch, cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.yaw = clampd(c.yaw, cfg_.teleop_dof_min, cfg_.teleop_dof_max);
}

void ControlGuard::clamp_ref_delta(ControlIntent& inout) const
{
    if (!inout.has_ref_delta) return;

    // 这里严格依赖你们 ControlReference 的字段布局。
    // 建议你后续把 ref_delta 从 ControlReference 提取为 RefDelta 结构，便于 clamp 和语义约束。
    // 当前保持钩子，不做假设。
    (void)inout;
}

void ControlGuard::emit_event_(const GuardEvent& event) const
{
    if (event_callback_) {
        event_callback_(event);
    }
}

bool ControlGuard::should_emit_reject_(std::uint64_t cmd_seq, const char* reject_tag) noexcept
{
    if (reject_tag == nullptr) {
        return false;
    }
    if (cmd_seq != 0 && cmd_seq == last_reject_cmd_seq_ && reject_tag == last_reject_tag_) {
        return false;
    }
    if (cmd_seq == 0 && reject_tag == last_reject_tag_) {
        return false;
    }
    last_reject_cmd_seq_ = cmd_seq;
    last_reject_tag_ = reject_tag;
    return true;
}

// =========================================================
// ControlGuard helpers (private)
// =========================================================
bool ControlGuard::is_neutral_for_clear_(const ControlIntent& intent) const noexcept
{
    constexpr double kEps = 0.05;

    const auto& d = intent.teleop_dof_cmd;
    return absd(d.surge) < kEps &&
           absd(d.sway) < kEps &&
           absd(d.heave) < kEps &&
           absd(d.roll) < kEps &&
           absd(d.pitch) < kEps &&
           absd(d.yaw) < kEps;
}

GuardResult ControlGuard::step(std::uint64_t now_ns,
                               const ControlState& /*state*/,
                               const shared::msg::NavStateView* nav,
                               const ControlIntent& intent)
{
    GuardResult out{};
    out.last_intent_ns = last_intent_ns_;
    out.effective_intent = intent;
    out.effective_mode = mode_;
    out.mode_changed = false;
    out.input_stale = false;
    out.estop_latched = estop_latched_;
    out.armed = armed_;
    out.failsafe = FailsafeAction::kNone;

    auto& eff = out.effective_intent;
    const bool req_exit = (intent.request_exit != 0);
    const bool nav_present = (nav != nullptr);
    const bool nav_valid = nav_present && (nav->valid != 0);
    const bool nav_stale = nav_present && (nav->stale != 0);
    const bool nav_degraded = nav_present && (nav->degraded != 0);
    const bool nav_ok_auto_now = nav_ok_for_mode(nav, ControlMode::kAuto);

    auto emit_guard_event = [&](const char* event_name,
                                const char* level,
                                std::uint16_t fault_code,
                                ControlMode requested_mode,
                                FailsafeAction failsafe,
                                const char* message) {
        GuardEvent event{};
        event.mono_ns = now_ns;
        event.event = event_name;
        event.level = level;
        event.fault_code = fault_code;
        event.mode = mode_;
        event.requested_mode = requested_mode;
        event.failsafe = failsafe;
        event.armed = armed_;
        event.estop_latched = estop_latched_;
        event.nav_present = nav_present;
        event.nav_valid = nav_valid;
        event.nav_stale = nav_stale;
        event.nav_degraded = nav_degraded;
        event.message = message;
        emit_event_(event);
    };

    // ========= 1) 输入新鲜度（cmd_seq / stamp_ns / teleop_dof 后备） =========
    {
        const bool has_cmd_seq = (intent.cmd_seq != 0);

        if (has_cmd_seq && intent.cmd_seq != last_intent_cmd_seq_) {
            last_intent_cmd_seq_ = intent.cmd_seq;
            last_intent_ns_ = (intent.stamp_ns != 0) ? intent.stamp_ns : now_ns;
            input_age_ms_ = 0;
        } else if (!has_cmd_seq && intent.has_teleop_dof) {
            last_intent_ns_ = now_ns;
            input_age_ms_ = 0;
        }
    }

    // ========= 2) TTL / stale 判定 =========
    const bool input_stale = is_intent_stale(now_ns, intent);
    out.input_stale = input_stale;
    if (last_intent_ns_ != 0 && now_ns >= last_intent_ns_) {
        input_age_ms_ = static_cast<std::uint32_t>((now_ns - last_intent_ns_) / 1000000ull);
    } else {
        input_age_ms_ = 0;
    }

    if (input_stale) {
        // TTL 过期只在 Guard 内裁掉执行载荷；是否落盘交给低频事件回调，
        // 避免在每个控制周期都写文本日志。
        eff.clear_payload();
        eff.request_exit = req_exit ? 1 : 0;
        eff.valid = false;
    }

    // ========= 3) E-STOP 锁存 / 解除 =========
    {
        const bool has_estop_level = (eff.estop != 0);
        const bool has_clear_req = (eff.clear_estop != 0);

        if (has_estop_level) {
            estop_latched_ = true;
            clear_hold_start_ns_ = 0;
            clear_hold_ms_ = 0;
        } else if (estop_latched_ && has_clear_req) {
            if (is_neutral_for_clear_(eff)) {
                if (clear_hold_start_ns_ == 0) {
                    clear_hold_start_ns_ = now_ns;
                    clear_hold_ms_ = 0;
                } else if (now_ns >= clear_hold_start_ns_) {
                    clear_hold_ms_ = static_cast<std::uint32_t>((now_ns - clear_hold_start_ns_) / 1000000ull);
                }

                const std::uint32_t hold_threshold_ms =
                    (cfg_.estop_clear_hold_ms > 0) ? cfg_.estop_clear_hold_ms : 2000;

                if (clear_hold_ms_ >= hold_threshold_ms) {
                    estop_latched_ = false;
                    clear_hold_start_ns_ = 0;
                    clear_hold_ms_ = 0;
                }
            } else {
                clear_hold_start_ns_ = 0;
                clear_hold_ms_ = 0;
                if (should_emit_reject_(intent.cmd_seq, "clear_estop_not_neutral")) {
                    emit_guard_event("guard_reject",
                                     "warn",
                                     control_fault_u16(shared::msg::FaultCode::kIllegalStateTransition),
                                     ControlMode::kNone,
                                     FailsafeAction::kNone,
                                     "clear estop rejected because teleop input is not neutral");
                }
            }
        } else {
            clear_hold_start_ns_ = 0;
            clear_hold_ms_ = 0;
        }

        out.estop_latched = estop_latched_;
    }

    // ========= 4) ARM / DISARM 处理 =========
    {
        const bool has_arm_cmd = eff.has_arm_cmd;
        const bool prev_armed = armed_;

        if (estop_latched_) {
            armed_ = false;
        } else if (has_arm_cmd) {
            const bool req_arm = (eff.arm != 0);
            const bool req_disarm = (eff.disarm != 0);

            std::cout << "[ControlGuard][ARM] has_arm_cmd=1"
                      << " estop_latched=" << int(estop_latched_)
                      << " req_arm=" << int(req_arm)
                      << " req_disarm=" << int(req_disarm)
                      << " prev_armed=" << int(prev_armed)
                      << "\n";

            if (req_disarm) {
                armed_ = false;
            } else if (req_arm) {
                armed_ = true;
            }
        }

        if (armed_ != prev_armed) {
            std::cout << "[ControlGuard][ARM] armed_ changed "
                      << prev_armed << " -> " << armed_ << "\n";
        }

        out.armed = armed_;
    }

    // ========= 5) 未 ARM 时禁止 DOF/Ref 输出 =========
    {
        if (!out.armed) {
            if (eff.has_teleop_dof) {
                eff.teleop_dof_cmd = DofCommand{};
                eff.has_teleop_dof = false;
            }
            if (eff.has_ref) {
                eff.has_ref = false;
            }
            if (eff.has_ref_delta) {
                eff.has_ref_delta = false;
            }
        }
    }

    // ========= 5.5) MotorTest 互斥 / 限时 / 自动回零 =========
    {
        const auto clear_motor_test = [&]() {
            motor_test_active_ = false;
            motor_test_deadline_ns_ = 0;
            motor_test_cmd_seq_ = 0;
            latched_motor_test_ = MotorTestCmd{};
            eff.has_motor_test = false;
            eff.motor_test = MotorTestCmd{};
        };

        const auto clamp_motor_test = [&](MotorTestCmd& mt) {
            if (mt.motor_id < 1) mt.motor_id = 1;
            if (mt.motor_id > 8) mt.motor_id = 8;
            if (mt.mode > 1) mt.mode = 0;
            if (mt.mode == 0) {
                mt.value = static_cast<float>(clampd(mt.value, -1.0, 1.0));
            }
            if (mt.duration_ms == 0) mt.duration_ms = 500;
            if (mt.duration_ms > 1000) mt.duration_ms = 1000;
        };

        const bool can_run_test = out.armed && !estop_latched_;
        const bool can_start_test = can_run_test && !input_stale;

        if (eff.has_motor_test && eff.motor_test.enable) {
            if (!can_start_test) {
                clear_motor_test();
                if (should_emit_reject_(intent.cmd_seq, "motor_test_reject")) {
                    emit_guard_event("guard_reject",
                                     "warn",
                                     control_fault_u16(shared::msg::FaultCode::kMotorTestViolation),
                                     mode_,
                                     FailsafeAction::kNone,
                                     "motor test rejected because guard preconditions are not satisfied");
                }
            } else {
                clamp_motor_test(eff.motor_test);
                latched_motor_test_ = eff.motor_test;
                motor_test_active_ = true;
                motor_test_cmd_seq_ = intent.cmd_seq;
                motor_test_deadline_ns_ =
                    now_ns + static_cast<std::uint64_t>(eff.motor_test.duration_ms) * 1000000ull;
            }
        } else if (motor_test_active_ && can_run_test && now_ns < motor_test_deadline_ns_) {
            eff.has_motor_test = true;
            eff.motor_test = latched_motor_test_;
        } else {
            clear_motor_test();
        }

        if (eff.has_motor_test) {
            eff.has_teleop_dof = false;
            eff.teleop_dof_cmd = DofCommand{};
            eff.has_ref = false;
            eff.has_ref_delta = false;
            eff.has_mode_request = false;
        }
    }

    // ========= 6) 模式门控（mode_request + nav 能力） =========
    {
        if (eff.has_mode_request && eff.mode_request != ControlMode::kNone) {
            ControlMode requested = eff.mode_request;

            if (!nav_ok_for_mode(nav, requested)) {
                if (should_emit_reject_(intent.cmd_seq, "mode_reject")) {
                    emit_guard_event("guard_reject",
                                     "warn",
                                     control_fault_u16(shared::msg::FaultCode::kIllegalStateTransition),
                                     requested,
                                     FailsafeAction::kNone,
                                     "requested mode rejected because nav gating is not satisfied");
                }
                requested = downgrade_mode(requested);
            }

            out.mode_changed = (requested != mode_);
            mode_ = requested;
        }

        if (mode_ == ControlMode::kAuto && !nav_ok_for_mode(nav, ControlMode::kAuto)) {
            const ControlMode downgraded = downgrade_mode(mode_);
            out.mode_changed = out.mode_changed || (downgraded != mode_);
            mode_ = downgraded;
        }

        out.effective_mode = mode_;
    }

    const bool nav_gating_context =
        cfg_.enable_mode_gating &&
        (nav_present || mode_ == ControlMode::kAuto ||
         (eff.has_mode_request && eff.mode_request == ControlMode::kAuto));
    if (nav_gating_context) {
        if (!have_last_nav_gating_state_ || nav_ok_auto_now != last_nav_gating_ok_) {
            emit_guard_event("guard_nav_gating_changed",
                             nav_ok_auto_now ? "info" : "warn",
                             nav_ok_auto_now
                                 ? control_fault_u16(shared::msg::FaultCode::kNone)
                                 : control_fault_u16(shared::msg::FaultCode::kNavUntrusted),
                             ControlMode::kAuto,
                             FailsafeAction::kNone,
                             nav_ok_auto_now
                                 ? "AUTO nav gating recovered"
                                 : "AUTO nav gating blocked because nav is missing or untrusted");
        }
        have_last_nav_gating_state_ = true;
        last_nav_gating_ok_ = nav_ok_auto_now;
    }

    // ========= 7) 限幅（teleop / ref_delta） =========
    clamp_teleop(eff);
    clamp_ref_delta(eff);

    // ========= 8) failsafe 决策 =========
    {
        FailsafeAction fs = FailsafeAction::kNone;

        if (estop_latched_) {
            fs = FailsafeAction::kEmergencyStop;
        } else {
            const auto mode_eff = out.effective_mode;
            const bool nav_untrusted =
                (mode_eff == ControlMode::kAuto) && !nav_ok_for_mode(nav, ControlMode::kAuto);
            const bool not_armed = !out.armed;

            if (mode_eff == ControlMode::kAuto) {
                if (input_stale || nav_untrusted || not_armed) {
                    fs = FailsafeAction::kZeroOutput;
                }
            } else if (mode_eff == ControlMode::kFailsafe) {
                fs = FailsafeAction::kZeroOutput;
            } else if (input_stale && out.armed) {
                fs = FailsafeAction::kZeroOutput;
            }
        }

        out.failsafe = fs;
    }

    if (out.failsafe != last_failsafe_action_) {
        const std::uint16_t fault_code = guard_fault_code_for_state(
            nav, input_stale, estop_latched_, out.effective_mode);
        if (out.failsafe != FailsafeAction::kNone) {
            emit_guard_event("guard_failsafe_entered",
                             "warn",
                             fault_code,
                             out.effective_mode,
                             out.failsafe,
                             estop_latched_
                                 ? "failsafe entered because estop is latched"
                                 : (input_stale
                                     ? "failsafe entered because control intent timed out"
                                     : "failsafe entered because guard conditions are not satisfied"));
        } else if (last_failsafe_action_ != FailsafeAction::kNone) {
            emit_guard_event("guard_failsafe_cleared",
                             "info",
                             control_fault_u16(shared::msg::FaultCode::kNone),
                             out.effective_mode,
                             last_failsafe_action_,
                             "failsafe cleared after guard conditions recovered");
        }
        last_failsafe_action_ = out.failsafe;
    }

    // ========= 9) 调试输出（状态快照变化时打印） =========
    std::uint32_t intent_age_ms = 0;
    if (intent.stamp_ns != 0 && now_ns >= intent.stamp_ns) {
        intent_age_ms = static_cast<std::uint32_t>((now_ns - intent.stamp_ns) / 1000000ull);
    }

    struct GuardDebugSnapshot {
        std::uint8_t armed;
        std::uint8_t estop_latched;
        std::uint8_t mode;
        std::uint8_t has_nav;
        std::uint8_t failsafe;
    };

    GuardDebugSnapshot cur{
        static_cast<std::uint8_t>(out.armed ? 1 : 0),
        static_cast<std::uint8_t>(out.estop_latched ? 1 : 0),
        static_cast<std::uint8_t>(static_cast<int>(out.effective_mode)),
        static_cast<std::uint8_t>(nav ? 1 : 0),
        static_cast<std::uint8_t>(static_cast<int>(out.failsafe)),
    };

    static GuardDebugSnapshot s_last{};
    static bool s_have_last = false;

    const bool changed = !s_have_last || std::memcmp(&cur, &s_last, sizeof(GuardDebugSnapshot)) != 0;
    if (changed) {
        std::cout << "[ControlGuard][STATE] "
                  << "armed=" << int(out.armed)
                  << " estop_latched=" << int(out.estop_latched)
                  << " mode=" << static_cast<int>(out.effective_mode)
                  << " has_nav=" << (nav ? 1 : 0)
                  << " intent_has_dof=" << int(intent.has_teleop_dof)
                  << " eff_has_dof=" << int(eff.has_teleop_dof)
                  << " eff_motor_test=" << int(eff.has_motor_test)
                  << " intent_ttl_ms=" << intent.ttl_ms
                  << " intent_age_ms=" << intent_age_ms
                  << " stale=" << int(input_stale)
                  << " failsafe=" << static_cast<int>(out.failsafe)
                  << " failsafe_name=" << failsafe_name(out.failsafe)
                  << "\n";
        s_last = cur;
        s_have_last = true;
    }

    return out;
}

} // namespace rovctrl::control_core
