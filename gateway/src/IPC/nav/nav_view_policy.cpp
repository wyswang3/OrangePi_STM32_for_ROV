#include "gateway/IPC/nav/nav_view_policy.hpp"

#include <limits>

namespace comm_gcs::ipc::nav {

void fill_nav_view_publish_fields(shared::msg::NavStateView& out,
                                  std::uint64_t              pub_mono_ns) noexcept
{
    out.mono_ns = pub_mono_ns;

    if (out.stamp_ns != 0 && pub_mono_ns >= out.stamp_ns) {
        const std::uint64_t age_ms64 = (pub_mono_ns - out.stamp_ns) / 1000000ull;
        out.age_ms = (age_ms64 > std::numeric_limits<std::uint32_t>::max())
            ? std::numeric_limits<std::uint32_t>::max()
            : static_cast<std::uint32_t>(age_ms64);
    } else {
        out.age_ms = std::numeric_limits<std::uint32_t>::max();
    }
}

shared::msg::NavStateView make_stale_nav_view(
    const shared::msg::NavStateView* last_view) noexcept
{
    shared::msg::NavStateView out{};
    out.version = shared::msg::kNavStateViewWireVersion;
    out.valid = 0;
    out.stale = 1;
    out.degraded = 1;
    out.nav_state = shared::msg::NavRunState::kInvalid;
    out.health = shared::msg::NavHealth::INVALID;
    out.fault_code = shared::msg::NavFaultCode::kNavViewStale;
    out.flags = 0;

    if (last_view != nullptr) {
        out.stamp_ns = last_view->stamp_ns;
        out.sensor_mask = last_view->sensor_mask;
        out.status_flags = last_view->status_flags;
    }

    return out;
}

NavViewDaemonDecision evaluate_nav_view_publish(
    const shared::msg::NavStateView* last_view,
    std::uint64_t                    last_nav_pub_mono_ns,
    std::uint64_t                    daemon_start_mono_ns,
    std::uint64_t                    now_mono_ns,
    const NavViewDaemonPolicyConfig& cfg,
    bool                             diagnostic_slot_ready) noexcept
{
    NavViewDaemonDecision decision{};
    decision.no_nav_yet = (last_view == nullptr);
    decision.age_ms_from_nav_pub = (last_nav_pub_mono_ns == 0 || now_mono_ns < last_nav_pub_mono_ns)
        ? UINT64_MAX
        : (now_mono_ns - last_nav_pub_mono_ns) / 1000000ull;

    const std::uint64_t elapsed_ms = (now_mono_ns < daemon_start_mono_ns)
        ? 0
        : (now_mono_ns - daemon_start_mono_ns) / 1000000ull;
    const bool in_warmup = elapsed_ms < cfg.warmup_ms;

    decision.stale_triggered = (!in_warmup) &&
                               (decision.age_ms_from_nav_pub != UINT64_MAX) &&
                               (decision.age_ms_from_nav_pub > cfg.max_age_ms);
    decision.diagnostic_only = decision.stale_triggered || decision.no_nav_yet;

    if (!decision.diagnostic_only) {
        decision.publish = true;
        decision.out = *last_view;
        decision.degraded_publish = (decision.out.valid == 0) || (decision.out.degraded != 0);
    } else if (cfg.publish_when_stale && diagnostic_slot_ready) {
        decision.publish = true;
        decision.degraded_publish = true;
        decision.out = make_stale_nav_view(last_view);
        decision.out.valid = 0;
        decision.out.stale = 1;
        decision.out.degraded = 1;
        decision.out.nav_state = shared::msg::NavRunState::kInvalid;
        decision.out.health = shared::msg::NavHealth::INVALID;
        if (decision.out.fault_code == shared::msg::NavFaultCode::kNone) {
            decision.out.fault_code = shared::msg::NavFaultCode::kNavViewStale;
        }
    }

    if (decision.publish) {
        fill_nav_view_publish_fields(decision.out, now_mono_ns);
    }

    return decision;
}

} // namespace comm_gcs::ipc::nav
