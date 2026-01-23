#ifndef ROVCTRL_PLATFORM_TIMEBASE_HPP
#define ROVCTRL_PLATFORM_TIMEBASE_HPP

#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace rovctrl::platform::timebase {

//-------------------------
// 基础类型
//-------------------------

// 单调时钟：作为全系统统一“时间轴底座”
using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration  = Clock::duration;

// -------------------------
// 1) Mono 时间接口（香橙派 steady_clock）
// -------------------------
//
// 约定：
// - MonoNS/MonoS 基于 steady_clock，单调递增，不受系统校时影响
// - now_ns() 为兼容旧代码：等价于 now_mono_ns()
int64_t   now_mono_ns() noexcept;
TimePoint now_mono() noexcept;

// Backward compatible aliases
inline int64_t now_ns() noexcept { return now_mono_ns(); }
inline TimePoint now() noexcept { return now_mono(); }

// -------------------------
// 2) Est 时间接口（可对齐/可校准的估计时间轴）
// -------------------------
//
// 最小实现：EstNS = MonoNS + est_offset_ns
// 后续可扩展为 EstNS = a*MonoNS + b（漂移 + 偏置），但先把 offset 做扎实。
int64_t now_est_ns() noexcept;

// Est 轴 offset（纳秒）
// - set_est_offset_ns: 允许上层在启动时/同步时设置
// - est_offset_ns:     当前 offset 值
int64_t est_offset_ns() noexcept;
void    set_est_offset_ns(int64_t offset_ns) noexcept;

// -------------------------
// 3) 控制层的“事件类型”（精简版）
// -------------------------
enum class SensorKind {
    IMU,          // IMU 传感器数据
    TELEOP,       // 人工遥操作输入
    CONTROL_LOOP, // 控制循环内部事件（如控制周期触发）
    OTHER
};

// -------------------------
// 4) 默认延迟参数（单位：纳秒）
// -------------------------
struct LatencyDefaults {
    int64_t imu_ns          = 2'000'000;   // 2 ms
    int64_t teleop_ns       = 10'000'000;  // 10 ms
    int64_t control_loop_ns = 0;           // 控制循环内部事件通常认为无额外延迟
    int64_t other_ns        = 0;
};

LatencyDefaults& latency_defaults() noexcept;

// -------------------------
// 5) 统一时间戳结构 Stamp
// -------------------------
//
// 你要的四列可由 Stamp 直接导出：
// - MonoNS = mono_ns
// - EstNS  = corrected_est_ns（推荐用于融合/对齐）或 raw_est_ns（诊断用）
// - MonoS  = mono_ns * 1e-9
// - EstS   = corrected_est_ns * 1e-9
//
// 语义约定：
// - mono_ns:      本机（OrangePi）单调时钟时间戳
// - raw_est_ns:   估计时间轴上的“采到/生成”时刻（= mono_ns + est_offset_ns）
// - sensor_time_ns: 设备自带时间（若有；未来可做 sensor->est 对齐）
// - latency_ns:   采用的延迟估计（默认或覆写）
// - corrected_est_ns: 估计时间轴上的“事件发生时刻”近似（raw_est_ns - latency_ns，或未来基于 sensor_time 对齐后再扣延迟）
struct Stamp {
    std::string            sensor_id;  // "imu0", "teleop0", "ctrl_loop0", ...
    SensorKind             kind{SensorKind::OTHER};

    // --- primary timebases ---
    int64_t                mono_ns{0};      // MonoNS
    int64_t                raw_est_ns{0};   // EstNS (uncorrected)

    // --- optional upstream/device time ---
    std::optional<int64_t> sensor_time_ns; // 可为空（设备自带时钟）

    // --- latency correction ---
    int64_t                latency_ns{0};
    int64_t                corrected_est_ns{0}; // EstNS after latency correction (recommended)

    // --- convenience helpers (seconds) ---
    static constexpr double kNsToS = 1e-9;

    double mono_s() const noexcept { return static_cast<double>(mono_ns) * kNsToS; }
    double raw_est_s() const noexcept { return static_cast<double>(raw_est_ns) * kNsToS; }
    double corrected_est_s() const noexcept { return static_cast<double>(corrected_est_ns) * kNsToS; }
};

// -------------------------
// 6) 统一打时间戳的工具函数 stamp()
// -------------------------
//
// 行为（建议实现语义）：
// - mono_ns      = now_mono_ns()
// - raw_est_ns   = now_est_ns()（或 mono_ns + offset）
// - latency_ns   = 参数优先，否则使用默认
// - base_est_ns  = raw_est_ns（最小实现；未来可扩展用 sensor_time_ns 对齐后作为 base）
// - corrected_est_ns = base_est_ns - latency_ns
Stamp stamp(
    const std::string&     sensor_id,
    SensorKind             kind,
    std::optional<int64_t> sensor_time_ns = std::nullopt,
    std::optional<int64_t> latency_ns     = std::nullopt
);

} // namespace rovctrl::platform::timebase

#endif /* ROVCTRL_PLATFORM_TIMEBASE_HPP */
