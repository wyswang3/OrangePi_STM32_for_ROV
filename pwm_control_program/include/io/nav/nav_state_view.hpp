#pragma once
#ifndef ROVCTRL_IO_NAV_STATE_VIEW_HPP
#define ROVCTRL_IO_NAV_STATE_VIEW_HPP

#include <cstdint>
#include "shared/msg/nav_state_view.hpp"

namespace rovctrl::io {

/**
 * @brief Control-side navigation view.
 *
 * Design:
 *  - wire: the exact payload published by gateway (ABI-stable)
 *  - pub_*: publisher timestamps from SHM header (not part of wire payload)
 *  - age_ms_local: 本地读取这一 hop SHM 之后额外增加的延迟
 *
 * Rule of thumb:
 *  - business/controls should read from this type, not from shared::msg directly.
 */
struct NavStateView final {
    // ---- wire payload (gateway->control ABI) ----
    shared::msg::NavStateView wire{};

    // ---- publisher meta (from SHM header) ----
    std::uint64_t pub_mono_ns = 0;
    std::uint64_t pub_wall_ns = 0;

    // ---- control-side computed meta ----
    std::uint32_t age_ms_local = 0;

    const shared::msg::NavStateView& payload() const noexcept { return wire; }
    shared::msg::NavStateView&       payload() noexcept { return wire; }

    [[nodiscard]] bool has_snapshot() const noexcept
    {
        return (pub_mono_ns != 0) || (wire.stamp_ns != 0);
    }

    [[nodiscard]] std::uint32_t total_age_ms() const noexcept
    {
        return wire.age_ms;
    }
};

} // namespace rovctrl::io

// Optional compatibility alias for old namespace usage
namespace rovctrl::io::nav {
using NavStateView = rovctrl::io::NavStateView;
}

#endif // ROVCTRL_IO_NAV_STATE_VIEW_HPP
/**
 * @file    control_loop_nav.cpp
 * @brief   NavSub PIMPL + navigation feedback update (B2: NavStateView)
 */
