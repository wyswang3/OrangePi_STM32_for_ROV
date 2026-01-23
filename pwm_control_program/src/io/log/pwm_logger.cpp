// pwm_logger.cpp

#include <fstream>      // 必须放第一：避免被项目头文件宏污染
#include <cmath>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "platform/timebase.hpp"
#include "io/log/pwm_logger.hpp"

namespace fs = std::filesystem;

namespace rovctrl::io {

struct PwmLogger::Impl {
    std::ofstream ofs;
    Mode mode{Mode::CmdAndApplied};

    // 是否保留 legacy t_s（调用方传入的秒时间）用于对照
    // 若打开，需要同时修改 header 与每行写入逻辑
    static constexpr bool kWriteLegacyTs = false;

    // 将归一化指令 u ∈ [-1, 1] 映射到占空比百分制 duty ∈ [5, 10]
    // 与 PwmClient::setTargets 里的映射保持一致：
    //   kDutyMid = 7.5, kDutySpan = 2.5 -> duty = 7.5 + 2.5 * u
    static double norm_to_duty_pct_(float u) noexcept
    {
        constexpr double kDutyMin  = 5.0;
        constexpr double kDutyMid  = 7.5;
        constexpr double kDutyMax  = 10.0;
        constexpr double kDutySpan = 2.5;

        // 先裁剪到 [-1, 1]
        if (u < -1.0f) u = -1.0f;
        if (u >  1.0f) u =  1.0f;

        double duty = kDutyMid + kDutySpan * static_cast<double>(u);
        if (duty < kDutyMin) duty = kDutyMin;
        if (duty > kDutyMax) duty = kDutyMax;
        return duty;  // 单位：%
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

    static bool ensure_dir_(const fs::path& p)
    {
        std::error_code ec;
        if (fs::exists(p, ec)) return true;
        return fs::create_directories(p, ec);
    }

    void write_header_()
    {
        if (!ofs.is_open()) return;

        // 统一时间戳列：Mono/Est 两条时间轴 + NS/S 两种表示
        ofs << "MonoNS,EstNS,MonoS,EstS";
        if constexpr (kWriteLegacyTs) {
            ofs << ",t_s_legacy";
        }

        if (mode == Mode::AppliedOnly) {
            for (int i = 1; i <= 8; ++i) ofs << ",ch" << i;  // duty[%]
        } else {
            for (int i = 1; i <= 8; ++i) ofs << ",ch" << i << "_cmd";
            for (int i = 1; i <= 8; ++i) ofs << ",ch" << i << "_applied";
        }
        ofs << "\n";
        ofs.flush();
    }

    static void write_nan_(std::ofstream& ofs, int n)
    {
        for (int i = 0; i < n; ++i) ofs << ",nan";
    }

    static void write_timebase_(std::ofstream& ofs) noexcept
    {
        using rovctrl::platform::timebase::SensorKind;
        using rovctrl::platform::timebase::stamp;

        // PWM 记录建议归为 CONTROL_LOOP（便于统一配置默认延迟）
        const auto st = stamp("pwm", SensorKind::CONTROL_LOOP);

        // EstNS/EstS 采用 corrected_est（默认 control_loop_ns=0 时等价 raw_est）
        ofs << st.mono_ns
            << "," << st.corrected_est_ns
            << "," << st.mono_s()
            << "," << st.corrected_est_s();
    }
};

PwmLogger::PwmLogger() : impl_(std::make_unique<Impl>()) {}
PwmLogger::~PwmLogger() { close(); }

PwmLogger::PwmLogger(PwmLogger&&) noexcept = default;
PwmLogger& PwmLogger::operator=(PwmLogger&&) noexcept = default;

bool PwmLogger::init(const std::string& root_dir, Mode mode, const std::string& prefix)
{
    if (!impl_) impl_ = std::make_unique<Impl>();
    close();

    impl_->mode = mode;

    fs::path root(root_dir);
    if (!Impl::ensure_dir_(root)) {
        std::cerr << "[PwmLogger] ensure_dir failed: " << root << "\n";
        return false;
    }

    const std::string ts = Impl::make_timestamp_();
    fs::path file = root / (prefix + "_" + ts + ".csv");

    impl_->ofs.open(file, std::ios::out | std::ios::trunc);
    if (!impl_->ofs.is_open()) {
        std::cerr << "[PwmLogger] open failed: " << file << "\n";
        return false;
    }

    // 固定格式：例如 7.5000、5.7800，便于后处理
    impl_->ofs.setf(std::ios::fixed);
    impl_->ofs << std::setprecision(4);

    impl_->write_header_();
    return true;
}

void PwmLogger::close() noexcept
{
    if (impl_ && impl_->ofs.is_open()) {
        impl_->ofs.flush();
        impl_->ofs.close();
    }
}

bool PwmLogger::is_open() const noexcept
{
    return impl_ && impl_->ofs.is_open();
}

PwmLogger::Mode PwmLogger::mode() const noexcept
{
    return impl_ ? impl_->mode : Mode::CmdAndApplied;
}

// 只记录“实际下发”的占空比（单位：%），输入为 [-1,1] 归一化 duty
void PwmLogger::logApplied(double t_s, const std::array<float, 8>& applied)
{
    if (!is_open()) return;

    auto& ofs = impl_->ofs;

    // 写统一时间戳
    Impl::write_timebase_(ofs);
    if constexpr (Impl::kWriteLegacyTs) {
        ofs << "," << t_s;
    }

    if (impl_->mode == Mode::AppliedOnly) {
        for (float v : applied) {
            ofs << "," << Impl::norm_to_duty_pct_(v);
        }
        ofs << "\n";
        return;
    }

    // CmdAndApplied: cmd 用 NaN 填充，applied 记录 duty[%]
    Impl::write_nan_(ofs, 8);
    for (float v : applied) {
        ofs << "," << Impl::norm_to_duty_pct_(v);
    }
    ofs << "\n";
}

// 同时记录“指令”和“实际下发”的占空比（单位：%），两者都假设是 [-1,1] 归一化
void PwmLogger::logCmdAndApplied(double t_s,
                                 const std::array<float, 8>& cmd,
                                 const std::array<float, 8>& applied)
{
    if (!is_open()) return;

    if (impl_->mode == Mode::AppliedOnly) {
        logApplied(t_s, applied);
        return;
    }

    auto& ofs = impl_->ofs;

    // 写统一时间戳
    Impl::write_timebase_(ofs);
    if constexpr (Impl::kWriteLegacyTs) {
        ofs << "," << t_s;
    }

    // cmd: 归一化 [-1,1] → duty[%]
    for (float v : cmd) {
        ofs << "," << Impl::norm_to_duty_pct_(v);
    }

    // applied: 同样按 [-1,1] → duty[%] 处理
    for (float v : applied) {
        ofs << "," << Impl::norm_to_duty_pct_(v);
    }
    ofs << "\n";
}

} // namespace rovctrl::io
