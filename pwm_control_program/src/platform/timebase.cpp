// src/platform/timebase.cpp

#include "platform/timebase.hpp"

#include <atomic>
#include <chrono>

namespace rovctrl::platform::timebase {

// -------------------------
// Est offset（全局）
// -------------------------
//
// 最小实现：EstNS = MonoNS + offset
// 注意：offset 允许在运行时被设置。若你担心并发读写，使用 atomic。
static std::atomic<int64_t> g_est_offset_ns{0};

// -------------------------
// 1) 基础时间接口：Mono（steady_clock）
// -------------------------

int64_t now_mono_ns() noexcept
{
    const auto tp = Clock::now().time_since_epoch();
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(tp);
    return ns.count();
}

TimePoint now_mono() noexcept
{
    return Clock::now();
}

// -------------------------
// 2) Est 时间接口
// -------------------------

int64_t est_offset_ns() noexcept
{
    return g_est_offset_ns.load(std::memory_order_relaxed);
}

void set_est_offset_ns(int64_t offset_ns) noexcept
{
    g_est_offset_ns.store(offset_ns, std::memory_order_relaxed);
}

int64_t now_est_ns() noexcept
{
    // 用 mono + offset 构造 Est 时间轴
    return now_mono_ns() + est_offset_ns();
}

// -------------------------
// 3) 默认延迟配置（全局静态）
// -------------------------

static LatencyDefaults g_latency_defaults{};

LatencyDefaults& latency_defaults() noexcept
{
    return g_latency_defaults;
}

static int64_t default_latency_for(SensorKind kind) noexcept
{
    const auto& d = g_latency_defaults;
    switch (kind) {
    case SensorKind::IMU:
        return d.imu_ns;
    case SensorKind::TELEOP:
        return d.teleop_ns;
    case SensorKind::CONTROL_LOOP:
        return d.control_loop_ns;
    case SensorKind::OTHER:
    default:
        return d.other_ns;
    }
}

// -------------------------
// 4) stamp(): 统一时间戳核心方法
// -------------------------

Stamp stamp(const std::string&        sensor_id,
            SensorKind                kind,
            std::optional<int64_t>    sensor_time_ns,
            std::optional<int64_t>    latency_ns)
{
    Stamp s;
    s.sensor_id = sensor_id;
    s.kind      = kind;

    // 1) 采样时刻（Mono + Est）
    s.mono_ns    = now_mono_ns();                // MonoNS
    s.raw_est_ns = s.mono_ns + est_offset_ns();  // EstNS (uncorrected)

    // 2) 延迟选择：参数优先，否则默认
    const int64_t used_latency_ns =
        latency_ns.has_value() ? *latency_ns : default_latency_for(kind);
    s.latency_ns = used_latency_ns;

    // 3) 保存设备时间（如有）
    s.sensor_time_ns = sensor_time_ns;

    // 4) 选择 base_est_ns 并做延迟补偿
    //
    // 最小实现：base_est_ns = raw_est_ns
    // 未来扩展点：若 sensor_time_ns 有值，可在此做 sensor->est 对齐映射：
    //   aligned_est_ns = align(sensor_id, *sensor_time_ns, s.raw_est_ns, ...)
    //   base_est_ns = aligned_est_ns
    const int64_t base_est_ns = s.raw_est_ns;

    s.corrected_est_ns = base_est_ns - used_latency_ns;

    return s;
}

} // namespace rovctrl::platform::timebase
