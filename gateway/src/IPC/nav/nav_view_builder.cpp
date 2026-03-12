#include "gateway/IPC/nav/nav_view_builder.hpp"

#include <cmath>
#include <cstdint>

namespace comm_gcs::ipc::nav {

namespace {

inline bool is_finite3(const double v[3]) noexcept
{
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

} // namespace

shared::msg::NavStateView NavViewBuilder::build(const shared::msg::NavState& s) noexcept
{
    shared::msg::NavStateView v{};

    // ---- wire header ----
    v.version  = shared::msg::kNavStateViewWireVersion;

    // NavState 的 t_ns 是 steady_clock 的 ns（你在 nav_state.hpp 注释里已经明确）
    // NavStateView 里我们放在 stamp_ns（“导航时间戳”）
    v.stamp_ns = s.t_ns;
    v.age_ms = s.age_ms;
    v.valid = s.valid;
    v.stale = s.stale;
    v.degraded = s.degraded;
    v.nav_state = s.nav_state;
    v.health = s.health;
    v.fault_code = s.fault_code;
    v.sensor_mask = s.sensor_mask;
    v.status_flags = s.status_flags;

    const bool finite_ok =
        (s.t_ns != 0) &&
        is_finite3(s.pos) && is_finite3(s.vel) && is_finite3(s.rpy) &&
        is_finite3(s.omega_b) && is_finite3(s.acc_b) &&
        std::isfinite(s.depth);

    // 控制面只在 upstream 明确有效、且数值有限时暴露运动学量。
    if (s.valid != 0 && s.stale == 0 && finite_ok) {
        for (int i = 0; i < 3; ++i) {
            v.pos[i]     = s.pos[i];
            v.vel[i]     = s.vel[i];
            v.rpy[i]     = s.rpy[i];
            v.omega_b[i] = s.omega_b[i];
            v.acc_b[i]   = s.acc_b[i];
        }
        v.depth_m = s.depth;
        v.flags = shared::msg::kHasPosition |
                  shared::msg::kHasVelocity |
                  shared::msg::kHasRPY |
                  shared::msg::kHasDepth |
                  shared::msg::kHasOmegaBody |
                  shared::msg::kHasAccBody;
        return v;
    }

    v.flags = 0;
    v.valid = 0;
    if (!finite_ok && v.fault_code == shared::msg::NavFaultCode::kNone) {
        v.nav_state = shared::msg::NavRunState::kInvalid;
        v.health = shared::msg::NavHealth::INVALID;
        v.fault_code = shared::msg::NavFaultCode::kEstimatorNumericInvalid;
    }

    return v;
}

} // namespace comm_gcs::ipc::nav
