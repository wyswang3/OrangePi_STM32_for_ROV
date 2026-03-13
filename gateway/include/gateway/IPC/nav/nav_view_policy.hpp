#pragma once

#include <cstdint>

#include "shared/msg/nav_state_view.hpp"

namespace comm_gcs::ipc::nav {

/**
 * @brief nav_viewd 当前 hop 的 stale/no-data 发布策略配置。
 *
 * 语义：
 *  - `max_age_ms` 基于 NavState SHM 发布时刻判断 bridge 输入是否 stale；
 *  - `warmup_ms` 在 daemon 冷启动窗口内抑制 stale 诊断；
 *  - `publish_when_stale` 控制是否发布显式诊断帧，而不是静默停更。
 */
struct NavViewDaemonPolicyConfig {
    std::uint32_t max_age_ms{250};
    std::uint32_t warmup_ms{1500};
    bool publish_when_stale{true};
};

struct NavViewDaemonDecision {
    bool publish{false};
    bool diagnostic_only{false};
    bool stale_triggered{false};
    bool no_nav_yet{false};
    bool degraded_publish{false};
    std::uint64_t age_ms_from_nav_pub{UINT64_MAX};
    shared::msg::NavStateView out{};
};

void fill_nav_view_publish_fields(shared::msg::NavStateView& out,
                                  std::uint64_t              pub_mono_ns) noexcept;

shared::msg::NavStateView make_stale_nav_view(
    const shared::msg::NavStateView* last_view) noexcept;

NavViewDaemonDecision evaluate_nav_view_publish(
    const shared::msg::NavStateView* last_view,
    std::uint64_t                    last_nav_pub_mono_ns,
    std::uint64_t                    daemon_start_mono_ns,
    std::uint64_t                    now_mono_ns,
    const NavViewDaemonPolicyConfig& cfg,
    bool                             diagnostic_slot_ready) noexcept;

} // namespace comm_gcs::ipc::nav
