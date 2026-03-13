/**
 * @file   control_loop_log.cpp
 * @brief  PwmLog PIMPL implementation.
 *
 * 控制状态/telemetry 时间线日志在 control_loop_run.cpp 中单独管理，
 * 这里仅保留 PWM 记录，避免生成空的 control CSV。
 */

#include "control_core/control_loop.hpp"

#include <array>
#include <filesystem>
#include <memory>
#include <string>

#include "io/log/pwm_logger.hpp"
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
        const fs::path pwm_dir = root / "pwm";

        // 1) PWM 日志：prefix 用调用方传入（例如 "pwm"），但落到 pwm/ 子目录
        if (!pwm_logger_.init(pwm_dir.string(), m, prefix)) {
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
    }

private:
    rovctrl::io::PwmLogger pwm_logger_;
};

} // namespace

std::unique_ptr<ControlLoop::PwmLog> ControlLoop::make_pwm_logger_()
{
    return std::make_unique<PwmLogImpl>();
}

} // namespace rovctrl::control_core
