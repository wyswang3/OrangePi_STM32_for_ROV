/**
 * @file   control_loop_log.cpp
 * @brief  PwmLog PIMPL implementation + separate ControlLoopLogger (no mixing).
 */

#include "control_core/control_loop.hpp"

#include <array>
#include <filesystem>
#include <memory>
#include <string>

#include "io/log/pwm_logger.hpp"
#include "io/log/control_loop_logger.hpp"

namespace rovctrl::control_core {

namespace {

class PwmLogImpl final : public ControlLoop::PwmLog {
public:
    bool init(const std::string& root_dir,
              Mode               mode,
              const std::string& prefix) override
    {
        namespace fs = std::filesystem;

        const rovctrl::io::PwmLogger::Mode m =
            (mode == Mode::AppliedOnly) ? rovctrl::io::PwmLogger::Mode::AppliedOnly
                                        : rovctrl::io::PwmLogger::Mode::CmdAndApplied;

        // 强烈建议：分目录，彻底避免误覆盖/混写
        const fs::path root(root_dir);
        const fs::path pwm_dir     = root / "pwm";
        const fs::path control_dir = root / "control";

        // 1) PWM 日志：prefix 用调用方传入（例如 "pwm"），但落到 pwm/ 子目录
        if (!pwm_logger_.init(pwm_dir.string(), m, prefix)) {
            return false;
        }

        // 2) 控制日志：固定前缀，落到 control/ 子目录
        //    绝对不能用与 PWM 相同的 prefix
        if (!control_loop_logger_.init(control_dir.string(), "control_loop")) {
            // PWM 已开，控制日志没开：按你策略决定是否失败
            // 我建议直接 return false，保证两份日志同时可用
            pwm_logger_.close();
            return false;
        }

        return true;
    }

    bool is_open() const noexcept override {
        return pwm_logger_.is_open();
        // 若你希望两份都必须 open 才算 open：
        // return pwm_logger_.is_open() && control_loop_logger_.is_open();
    }

    void logApplied(double t_s,
                    const std::array<float, 8>& applied) override
    {
        // 只记录 PWM
        pwm_logger_.logApplied(t_s, applied);
    }

    void logCmdAndApplied(double t_s,
                          const std::array<float, 8>& cmd,
                          const std::array<float, 8>& applied) override
    {
        // 只记录 PWM
        pwm_logger_.logCmdAndApplied(t_s, cmd, applied);
    }

    void close() noexcept override
    {
        pwm_logger_.close();
        control_loop_logger_.close();
    }

    // 说明：控制日志应在 ControlLoop 的正确时刻写入（拿到 intent/guard/nav 的地方）
    // 你可以在 ControlLoop 内部持有一个指向本实现的指针并调用这个函数，
    // 或者把该能力通过 ControlLoop::PwmLog 接口显式暴露出来（更正统）。
    void logControlLoop(double t_s,
                        const rovctrl::io::ControlEffect& eff,
                        const rovctrl::io::ControlGuardOutput& guard_out,
                        const rovctrl::io::NavigationData& nav) noexcept
    {
        if (!control_loop_logger_.is_open()) return;
        control_loop_logger_.log_data(t_s, eff, guard_out, nav);
    }

private:
    rovctrl::io::PwmLogger         pwm_logger_;
    rovctrl::io::ControlLoopLogger control_loop_logger_;
};

} // namespace

std::unique_ptr<ControlLoop::PwmLog> ControlLoop::make_pwm_logger_()
{
    return std::make_unique<PwmLogImpl>();
}

} // namespace rovctrl::control_core
