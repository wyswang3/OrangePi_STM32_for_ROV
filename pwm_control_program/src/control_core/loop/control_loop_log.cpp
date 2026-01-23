/**
 * @file   control_loop_log.cpp
 * @brief  PwmLog PIMPL implementation, now includes ControlLoopLogger integration.
 */

#include "control_core/control_loop.hpp"

#include <array>
#include <string>
#include <memory>   // +++

// 日志实现（cpp 内部使用，不透传到头文件）
#include "io/log/pwm_logger.hpp"
#include "io/log/control_loop_logger.hpp"  // 引入 ControlLoopLogger

namespace rovctrl::control_core {

namespace {

// PwmLogImpl 现在不仅仅管理 PwmLogger，还管理 ControlLoopLogger
class PwmLogImpl final : public ControlLoop::PwmLog {
public:
    bool init(const std::string& root_dir,
              Mode               mode,
              const std::string& prefix) override
    {
        const rovctrl::io::PwmLogger::Mode m =
            (mode == Mode::AppliedOnly) ? rovctrl::io::PwmLogger::Mode::AppliedOnly
                                        : rovctrl::io::PwmLogger::Mode::CmdAndApplied;

        // 初始化 PwmLogger
        if (!logger_.init(root_dir, m, prefix)) {
            return false;
        }

        // 初始化 ControlLoopLogger
        control_loop_logger_.init(root_dir, prefix);  // 使用相同的路径和前缀

        return true;
    }

    bool is_open() const noexcept override {
        return logger_.is_open() && control_loop_logger_.is_open();
    }

    void logApplied(double t_s,
                    const std::array<float, 8>& applied) override
    {
        logger_.logApplied(t_s, applied);
        // PwmLogger 完成之后记录控制回路数据
        log_control_loop_data(t_s, applied, {}, {});  // 控制指令和状态可以稍后补充
    }

    void logCmdAndApplied(double t_s,
                          const std::array<float, 8>& cmd,
                          const std::array<float, 8>& applied) override
    {
        logger_.logCmdAndApplied(t_s, cmd, applied);
        // 同时记录控制回路数据
        log_control_loop_data(t_s, applied, cmd, {});  // 控制指令、实际下发值和导航数据稍后补充
    }

    void close() noexcept override
    {
        // 关闭 PwmLogger
        logger_.close();
        // 关闭 ControlLoopLogger
        control_loop_logger_.close();
    }

private:
    // PwmLogger 仍然负责 PWM 日志记录
    rovctrl::io::PwmLogger logger_;
    // 新增 ControlLoopLogger 负责控制回路日志记录
    rovctrl::io::ControlLoopLogger control_loop_logger_;

    // 记录控制回路的数据（控制指令、控制状态、导航数据）
    void log_control_loop_data(double t_s, const std::array<float, 8>& applied, 
                               const std::array<float, 8>& cmd, const std::array<float, 8>& nav_data)
    {
        rovctrl::io::ControlEffect eff;
        eff.surge = applied[0];
        eff.sway = applied[1];
        eff.heave = applied[2];
        eff.roll = applied[3];
        eff.pitch = applied[4];
        eff.yaw = applied[5];

        rovctrl::io::ControlGuardOutput guard_out;
        // 假设你有函数获取控制守护状态，填充 guard_out

        rovctrl::io::NavigationData nav_data_struct;
        // 假设你有方法填充导航数据 nav_data_struct

        control_loop_logger_.log_data(t_s, eff, guard_out, nav_data_struct);
    }
};

} // namespace

// 工厂：在 run() 中创建（保持你原来的“按开关创建”的行为）
std::unique_ptr<ControlLoop::PwmLog> ControlLoop::make_pwm_logger_()
{
    return std::make_unique<PwmLogImpl>();
}

} // namespace rovctrl::control_core
