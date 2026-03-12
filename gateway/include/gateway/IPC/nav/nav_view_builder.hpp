#pragma once
#ifndef COMM_GCS_IPC_NAV_NAV_VIEW_BUILDER_HPP
#define COMM_GCS_IPC_NAV_NAV_VIEW_BUILDER_HPP

#include <cstdint>
#include <cstring>

#include "shared/msg/nav_state.hpp"
#include "shared/msg/nav_state_view.hpp"

namespace comm_gcs::ipc::nav {

/**
 * @brief Convert shared::msg::NavState -> shared::msg::NavStateView
 *
 * Builder policy:
 *   - 直接透传 upstream 已经显式计算好的 valid/stale/degraded/fault 语义；
 *   - 只有 upstream 明确 valid=1 且数值有限时，才把运动学字段暴露给控制侧；
 *   - upstream invalid/stale 时，保留诊断字段，但清空运动学 payload，避免旧快照复用。
 *
 * Timebase:
 *   - NavState::t_ns is steady_clock ns (publisher side)
 *   - NavStateView::stamp_ns keeps the same semantic
 *   - age_ms / mono_ns are filled by the hop that publishes NavStateView.
 */
struct NavViewBuilder final {
    static shared::msg::NavStateView build(const shared::msg::NavState& s) noexcept;
};

} // namespace comm_gcs::ipc::nav

#endif // COMM_GCS_IPC_NAV_NAV_VIEW_BUILDER_HPP
