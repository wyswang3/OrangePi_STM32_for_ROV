/**
 * @file   control_loop_nav.cpp
 * @brief  NavSub PIMPL + navigation feedback update (B2: NavStateView)
 */

#include "control_core/control_loop.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "io/nav/nav_state_view.hpp"
#include "io/nav/nav_view_shm_source.hpp"   // rovctrl::io::nav::NavViewShmSource

namespace rovctrl::control_core {

namespace {

inline void clear_nav_feedback_state(ControlState& state) noexcept
{
    state.nav_present = false;
    state.nav_valid = false;
    state.nav_stale = true;
    state.nav_degraded = false;
    state.nav_t_ns = 0;
    state.nav_age_ms = 0;
    state.nav_state = shared::msg::NavRunState::kUninitialized;
    state.nav_health = shared::msg::NavHealth::UNINITIALIZED;
    state.nav_fault_code = shared::msg::NavFaultCode::kNoData;
    state.nav_sensor_mask = shared::msg::NAV_SENSOR_NONE;
    state.nav_status_flags = shared::msg::NAV_FLAG_NONE;
    state.nav_depth = 0.0;
    state.nav_pos_ned.fill(0.0);
    state.nav_vel_ned.fill(0.0);
    state.nav_rpy.fill(0.0);
    state.nav_omega_b.fill(0.0);
    state.nav_acc_b.fill(0.0);
}

inline bool nav_snapshot_trusted(const shared::msg::NavStateView& v) noexcept
{
    const bool state_ok =
        (v.nav_state == shared::msg::NavRunState::kOk) ||
        (v.nav_state == shared::msg::NavRunState::kDegraded);
    return (v.valid != 0) &&
           (v.stale == 0) &&
           (v.fault_code == shared::msg::NavFaultCode::kNone) &&
           state_ok;
}

} // namespace

// ============================================================================
// NavSub (PIMPL) —— only depends on nav shm source in this TU
// ============================================================================

struct ControlLoop::NavSub {
    rovctrl::io::nav::NavViewShmSource src{};
    rovctrl::io::nav::NavViewShmSource::Config cfg{};
    bool inited{false};

    bool ensure_init() {
        if (inited) return src.ok();
        inited = src.init(cfg);
        return inited && src.ok();
    }

    bool read_latest(rovctrl::io::NavStateView& out) {
        if (!ensure_init()) return false;
        return src.read_latest(out);
    }

    void shutdown() noexcept { src.shutdown(); inited = false; }
};


// custom deleter for unique_ptr<NavSub, NavSubDeleter>
void ControlLoop::NavSubDeleter::operator()(NavSub* p) noexcept { delete p; }

// ============================================================================
// Navigation feedback update (B2: NavStateView)
// ============================================================================

bool ControlLoop::update_nav_feedback_(rovctrl::io::NavStateView& nav_view_out)
{
    clear_nav_feedback_state(state_);

    if (!nav_sub_) {
        nav_sub_.reset(new NavSub());
        // 可在此用 cfg_ 覆盖 nav_sub_->cfg.*
    }

    if (!nav_sub_->read_latest(nav_view_out)) {
        last_nav_valid_ = false;
        return false;
    }

    last_nav_valid_    = true;
    state_.nav_present = true;

    // readability: use accessor
    const auto& v = nav_view_out.payload();

    // 统一时间语义：
    //   - stamp_ns: 导航状态真正对应的估计时间
    //   - age_ms  : control 读取该快照时的总 age（gateway hop + local hop）
    state_.nav_t_ns = v.stamp_ns;
    state_.nav_age_ms = v.age_ms;
    state_.nav_valid = nav_snapshot_trusted(v);
    state_.nav_stale = (v.stale != 0);
    state_.nav_degraded = (v.degraded != 0);
    state_.nav_state = v.nav_state;
    state_.nav_health = v.health;
    state_.nav_fault_code = v.fault_code;
    state_.nav_sensor_mask = v.sensor_mask;
    state_.nav_status_flags = v.status_flags;

    for (std::size_t i = 0; i < 3; ++i) {
        state_.nav_pos_ned[i] = v.pos[i];
        state_.nav_vel_ned[i] = v.vel[i];
        state_.nav_rpy[i]     = v.rpy[i];
        state_.nav_omega_b[i] = v.omega_b[i];
        state_.nav_acc_b[i]   = v.acc_b[i];
    }

    state_.nav_depth = v.depth_m;

    // （可选）如果 state_ 有诊断字段，再写入：
    // state_.nav_pub_mono_ns = nav_view_out.pub_mono_ns;
    // state_.nav_pub_wall_ns = nav_view_out.pub_wall_ns;
    // state_.nav_age_ms      = nav_view_out.age_ms_local;

    return true;
}


// IMPORTANT: define ~ControlLoop() here (NavSub complete in this TU)
ControlLoop::~ControlLoop() noexcept = default;

} // namespace rovctrl::control_core
