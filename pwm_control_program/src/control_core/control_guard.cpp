#include "control_core/control_guard.hpp"

#include <algorithm>
#include <cstdint>
#include <iostream>   // <<< 新增
#include <cstring>


// 只在 cpp 里依赖 nav_state 的完整定义
#include "shared/msg/nav_state.hpp"

// 做法 A：ControlIntent/ControlMode 真源在 control_core
#include "control_core/control_intent.hpp"
#include "control_core/control_mode.hpp"

namespace rovctrl::control_core {

static inline double clampd(double v, double lo, double hi) {
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

ControlGuard::ControlGuard(ControlGuardConfig cfg)
    : cfg_(cfg)
{
}

void ControlGuard::reset()
{
    armed_          = false;
    estop_latched_  = false;

    // 做法 A：ControlMode 在 control_core
    mode_           = ControlMode::kManual;

    last_intent_ns_  = 0;
    last_intent_cmd_seq_ = 0;
    input_age_ms_    = 0;
    clear_hold_start_ns_ = 0;
    clear_hold_ms_ = 0;
    motor_test_active_ = false;
    motor_test_deadline_ns_ = 0;
    motor_test_cmd_seq_ = 0;
    latched_motor_test_ = MotorTestCmd{};
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
    // 1) ttl 决策
    const std::uint32_t ttl_ms =
        (intent.ttl_ms != 0) ? intent.ttl_ms : cfg_.default_ttl_ms;

    if (ttl_ms == 0) {
        return false; // ttl disabled
    }

    // 2) 起始时间
    const std::uint64_t t0 = (intent.stamp_ns != 0) ? intent.stamp_ns : last_intent_ns_;
    const std::uint64_t t1 = (now_ns != 0) ? now_ns : t0;

    if (t1 < t0) {
        // steady ns 理论不应倒退；保守处理为“不过期”
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
                                  rovctrl::control_core::ControlMode requested) const
{
    if (!cfg_.enable_mode_gating) {
        return true;
    }

    switch (requested) {
    case ControlMode::kManual:
        // 手动模式一般不强依赖导航
        return true;

    case ControlMode::kAuto:
        // 自动/闭环必须同时通过 valid/stale/fault/status_flags 生命周期检查。
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
    c.sway  = clampd(c.sway,  cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.heave = clampd(c.heave, cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.roll  = clampd(c.roll,  cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.pitch = clampd(c.pitch, cfg_.teleop_dof_min, cfg_.teleop_dof_max);
    c.yaw   = clampd(c.yaw,   cfg_.teleop_dof_min, cfg_.teleop_dof_max);
}

void ControlGuard::clamp_ref_delta(ControlIntent& inout) const
{
    if (!inout.has_ref_delta) return;

    // 这里严格依赖你们 ControlReference 的字段布局。
    // 建议你后续把 ref_delta 从 ControlReference 提取为 RefDelta 结构，便于 clamp 和语义约束。
    // 当前保持钩子，不做假设。
}

// =========================================================
// ControlGuard helpers (private)
// =========================================================
bool ControlGuard::is_neutral_for_clear_(const ControlIntent& intent) const noexcept
{
    // 你可以把 eps 做成 cfg 参数；先给一个工程上相对保守的默认值
    constexpr double kEps = 0.05; // “基本回中立”的阈值，按你的摇杆映射调整

    const auto& d = intent.teleop_dof_cmd;

    return absd(d.surge) < kEps &&
           absd(d.sway)  < kEps &&
           absd(d.heave) < kEps &&
           absd(d.roll)  < kEps &&
           absd(d.pitch) < kEps &&
           absd(d.yaw)   < kEps;
}

GuardResult ControlGuard::step(std::uint64_t now_ns,
                               const ControlState& /*state*/,
                               const shared::msg::NavStateView* nav,
                               const ControlIntent& intent)
{
    GuardResult out{};
    out.last_intent_ns  = last_intent_ns_;
    out.effective_intent = intent;    // 可修改副本
    out.effective_mode   = mode_;
    out.mode_changed     = false;
    out.input_stale      = false;
    out.estop_latched    = estop_latched_;
    out.armed            = armed_;    // 真正结果稍后更新
    out.failsafe         = FailsafeAction::kNone;

    auto& eff = out.effective_intent;
    const bool req_exit = (intent.request_exit != 0);

    // ========= 1) 输入新鲜度（cmd_seq / stamp_ns / teleop_dof 后备） =========
    {
        const bool has_cmd_seq = (intent.cmd_seq != 0);

        if (has_cmd_seq && intent.cmd_seq != last_intent_cmd_seq_) {
            // 正常路径：依赖 cmd_seq 递增
            last_intent_cmd_seq_ = intent.cmd_seq;
            last_intent_ns_      = (intent.stamp_ns != 0) ? intent.stamp_ns : now_ns;
            input_age_ms_        = 0;
        } else if (!has_cmd_seq && intent.has_teleop_dof) {
            // 防御式兜底：某些实现若忘记填 cmd_seq，但 teleop_dof 在更新
            last_intent_ns_ = now_ns;
            input_age_ms_   = 0;
        }
    }

    // ========= 2) TTL / stale 判定 =========
    const bool input_stale = is_intent_stale(now_ns, intent);
    out.input_stale  = input_stale;
    if (last_intent_ns_ != 0 && now_ns >= last_intent_ns_) {
        input_age_ms_ = static_cast<std::uint32_t>((now_ns - last_intent_ns_) / 1000000ull);
    } else {
        input_age_ms_ = 0;
    }

    if (input_stale) {
        // TTL 过期：清除所有执行载荷，只保留退出语义。
        eff.clear_payload();
        eff.request_exit = req_exit ? 1 : 0;
        eff.valid = false;
    }

    // ========= 3) E-STOP 锁存 / 解除 =========
    {
        const bool has_estop_level = (eff.estop != 0);
        const bool has_clear_req   = (eff.clear_estop != 0);

        if (has_estop_level) {
            // (A) 任何时刻收到 estop=1：立即锁存，并重置“解除计时”
            estop_latched_       = true;
            clear_hold_start_ns_ = 0;
            clear_hold_ms_       = 0;
        } else if (estop_latched_ && has_clear_req) {
            // (B) 只有在已锁存时，才考虑解除
            if (is_neutral_for_clear_(eff)) {
                if (clear_hold_start_ns_ == 0) {
                    clear_hold_start_ns_ = now_ns;
                    clear_hold_ms_       = 0;
                } else if (now_ns >= clear_hold_start_ns_) {
                    clear_hold_ms_ = static_cast<std::uint32_t>(
                        (now_ns - clear_hold_start_ns_) / 1000000ull
                    );
                }

                const std::uint32_t hold_threshold_ms =
                    (cfg_.estop_clear_hold_ms > 0) ? cfg_.estop_clear_hold_ms : 2000;

                if (clear_hold_ms_ >= hold_threshold_ms) {
                    estop_latched_       = false;
                    clear_hold_start_ns_ = 0;
                    clear_hold_ms_       = 0;
                }
            } else {
                // 不在中立态：重置计时，防止误解锁
                clear_hold_start_ns_ = 0;
                clear_hold_ms_       = 0;
            }
        } else {
            // (C) 没有 clear 请求或未锁存：计时清零
            clear_hold_start_ns_ = 0;
            clear_hold_ms_       = 0;
        }

        out.estop_latched = estop_latched_;
    }

    // ========= 4) ARM / DISARM 处理 =========
    {
        const bool has_arm_cmd = eff.has_arm_cmd;
        const bool prev_armed  = armed_;

        if (estop_latched_) {
            // 急停锁存：无条件上锁
            armed_ = false;
        } else if (has_arm_cmd) {
            const bool req_arm    = (eff.arm    != 0);
            const bool req_disarm = (eff.disarm != 0);

            // 调试：仅在带 ARM 命令的帧打印
            std::cout << "[ControlGuard][ARM] has_arm_cmd=1"
                      << " estop_latched=" << int(estop_latched_)
                      << " req_arm="      << int(req_arm)
                      << " req_disarm="   << int(req_disarm)
                      << " prev_armed="   << int(prev_armed)
                      << "\n";

            if (req_disarm) {
                armed_ = false;               // DISARM 优先
            } else if (req_arm) {
                armed_ = true;                // 无急停锁存前提下允许 ARM
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
                eff.teleop_dof_cmd = rovctrl::control_core::DofCommand{};
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
                requested = downgrade_mode(requested);
            }

            out.mode_changed = (requested != mode_);
            mode_            = requested;
        }

        // 当前处于 AUTO 时也要持续检查导航，而不是只在 mode_request 边沿检查一次。
        if (mode_ == ControlMode::kAuto && !nav_ok_for_mode(nav, ControlMode::kAuto)) {
            const ControlMode downgraded = downgrade_mode(mode_);
            out.mode_changed = out.mode_changed || (downgraded != mode_);
            mode_ = downgraded;
        }

        out.effective_mode = mode_;
    }

    // ========= 7) 限幅（teleop / ref_delta） =========
    clamp_teleop(eff);
    clamp_ref_delta(eff);

    // ========= 8) failsafe 决策 =========
    {
        FailsafeAction fs = FailsafeAction::kNone;

        if (estop_latched_) {
            // 急停锁存：跨模式紧急停机
            fs = FailsafeAction::kEmergencyStop;
        } else {
            const auto mode_eff    = out.effective_mode;
            const bool nav_untrusted =
                (mode_eff == ControlMode::kAuto) &&
                !nav_ok_for_mode(nav, ControlMode::kAuto);
            const bool not_armed   = !out.armed;

            if (mode_eff == ControlMode::kAuto) {
                // Auto 模式：对输入 stale / 导航不可信 / 未 ARM 严格防护。
                if (input_stale || nav_untrusted || not_armed) {
                    fs = FailsafeAction::kZeroOutput;
                }
            } else if (mode_eff == ControlMode::kFailsafe) {
                fs = FailsafeAction::kZeroOutput;
            } else if (input_stale && out.armed) {
                // Manual stale 不再复用最后一帧，直接归零。
                fs = FailsafeAction::kZeroOutput;
            }
        }

        out.failsafe = fs;
    }

    // ========= 9) 调试输出（状态快照变化时打印） =========
    std::uint32_t intent_age_ms = 0;
    if (intent.stamp_ns != 0 && now_ns >= intent.stamp_ns) {
        intent_age_ms = static_cast<std::uint32_t>(
            (now_ns - intent.stamp_ns) / 1000000ull
        );
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

    const bool changed =
        !s_have_last ||
        std::memcmp(&cur, &s_last, sizeof(GuardDebugSnapshot)) != 0;

    if (changed) {
        std::cout << "[ControlGuard][STATE] "
                  << "armed="           << int(out.armed)
                  << " estop_latched="  << int(out.estop_latched)
                  << " mode="           << static_cast<int>(out.effective_mode)
                  << " has_nav="        << (nav ? 1 : 0)
                  << " intent_has_dof=" << int(intent.has_teleop_dof)
                  << " eff_has_dof="    << int(eff.has_teleop_dof)
                  << " eff_motor_test=" << int(eff.has_motor_test)
                  << " intent_ttl_ms="  << intent.ttl_ms
                  << " intent_age_ms="  << intent_age_ms
                  << " stale="          << int(input_stale)
                  << " failsafe="       << static_cast<int>(out.failsafe)
                  << "\n";

        s_last      = cur;
        s_have_last = true;
    }

    return out;
}

} // namespace rovctrl::control_core
