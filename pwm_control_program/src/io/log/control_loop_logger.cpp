#include "io/log/control_loop_logger.hpp"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <ctime>
#include <filesystem>
#include <sstream>

#include "platform/timebase.hpp"

namespace rovctrl::io {

struct ControlLoopLogger::Impl {
    std::ofstream ofs;

    // 是否把 legacy 入参 t_s 写进 CSV（仅用于对照旧链路）
    // 若打开，会在时间戳四列后追加一列 t_s_legacy
    static constexpr bool kWriteLegacyTs = false;

    static bool ensure_dir_(const std::filesystem::path& p)
    {
        std::error_code ec;
        if (std::filesystem::exists(p, ec)) return true;
        return std::filesystem::create_directories(p, ec);
    }

    static std::string make_timestamp_()
    {
        std::time_t t = std::time(nullptr);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
        return oss.str();
    }

    static void write_timebase_(std::ofstream& ofs) noexcept
    {
        using rovctrl::platform::timebase::SensorKind;
        using rovctrl::platform::timebase::stamp;

        // 控制回路日志：归为 CONTROL_LOOP
        const auto st = stamp("control_loop", SensorKind::CONTROL_LOOP);

        // EstNS/EstS 使用 corrected_est（默认 control_loop_ns=0 则等价 raw_est）
        ofs << st.mono_ns
            << "," << st.corrected_est_ns
            << "," << st.mono_s()
            << "," << st.corrected_est_s();
    }

    void write_header_()
    {
        if (!ofs.is_open()) return;

        // 统一时间戳（与 PWM 日志保持一致）
        ofs << "MonoNS,EstNS,MonoS,EstS";
        if constexpr (kWriteLegacyTs) {
            ofs << ",t_s_legacy";
        }

        // ControlEffect（严格按头文件字段）
        ofs << ",eff_surge,eff_sway,eff_heave,eff_roll,eff_pitch,eff_yaw"
            << ",eff_has_ref,eff_has_ref_delta,eff_request_exit,eff_intent_age_ms";

        // ControlGuardOutput（严格按头文件字段）
        ofs << ",armed,estop_latched,effective_mode,has_nav,failsafe";

        // NavigationData（严格按头文件字段）
        ofs << ",nav_x,nav_y,nav_z,nav_roll,nav_pitch,nav_yaw";

        ofs << "\n";
        ofs.flush();
    }

    void write_data_(double t_s,
                     const ControlEffect& eff,
                     const ControlGuardOutput& guard_out,
                     const NavigationData& nav_data)
    {
        if (!ofs.is_open()) return;

        // 1) 时间戳
        write_timebase_(ofs);
        if constexpr (kWriteLegacyTs) {
            ofs << "," << t_s;
        }

        // 2) ControlEffect
        ofs << "," << eff.surge
            << "," << eff.sway
            << "," << eff.heave
            << "," << eff.roll
            << "," << eff.pitch
            << "," << eff.yaw
            << "," << (eff.has_ref ? 1 : 0)
            << "," << (eff.has_ref_delta ? 1 : 0)
            << "," << (eff.request_exit ? 1 : 0)
            << "," << static_cast<unsigned>(eff.intent_age_ms);

        // 3) ControlGuardOutput
        ofs << "," << (guard_out.armed ? 1 : 0)
            << "," << (guard_out.estop_latched ? 1 : 0)
            << "," << static_cast<int>(guard_out.effective_mode)
            << "," << (guard_out.has_nav ? 1 : 0)
            << "," << (guard_out.failsafe ? 1 : 0);

        // 4) NavigationData
        ofs << "," << nav_data.x
            << "," << nav_data.y
            << "," << nav_data.z
            << "," << nav_data.roll
            << "," << nav_data.pitch
            << "," << nav_data.yaw;

        ofs << "\n";
        // 不强制每行 flush，避免 IO 开销过大；必要时可按周期 flush
        // ofs.flush();
    }
};

ControlLoopLogger::ControlLoopLogger() : impl_(std::make_unique<Impl>()) {}
ControlLoopLogger::~ControlLoopLogger() { close(); }

bool ControlLoopLogger::init(const std::string& root_dir, const std::string& prefix)
{
    if (!impl_) impl_ = std::make_unique<Impl>();
    close();

    std::filesystem::path root(root_dir);
    if (!Impl::ensure_dir_(root)) {
        std::cerr << "[ControlLoopLogger] ensure_dir failed: " << root << "\n";
        return false;
    }

    const std::string ts = Impl::make_timestamp_();
    std::filesystem::path file = root / (prefix + "_" + ts + ".csv");

    impl_->ofs.open(file, std::ios::out | std::ios::trunc);
    if (!impl_->ofs.is_open()) {
        std::cerr << "[ControlLoopLogger] open failed: " << file << "\n";
        return false;
    }

    // 数值格式：便于后处理（可按需调整）
    impl_->ofs.setf(std::ios::fixed);
    impl_->ofs << std::setprecision(6);

    impl_->write_header_();
    return true;
}

void ControlLoopLogger::close() noexcept
{
    if (impl_ && impl_->ofs.is_open()) {
        impl_->ofs.flush();
        impl_->ofs.close();
    }
}

bool ControlLoopLogger::is_open() const noexcept
{
    return impl_ && impl_->ofs.is_open();
}

void ControlLoopLogger::log_data(double t_s,
                                 const ControlEffect& eff,
                                 const ControlGuardOutput& guard_out,
                                 const NavigationData& nav_data)
{
    if (!is_open()) return;
    impl_->write_data_(t_s, eff, guard_out, nav_data);
}

} // namespace rovctrl::io
