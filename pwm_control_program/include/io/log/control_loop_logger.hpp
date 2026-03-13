#ifndef ROVCTRL_IO_CONTROL_LOOP_LOGGER_HPP
#define ROVCTRL_IO_CONTROL_LOOP_LOGGER_HPP

#include <fstream>
#include <array>
#include <cstdint>
#include <string>
#include <ctime>
#include <filesystem>

namespace rovctrl::io {

/**
 * @file control_loop_logger.hpp
 * 
 * @brief 负责记录控制回路中的控制指令、控制状态和来自导航系统的共享内存数据。
 * 
 * 该日志模块的功能包括：
 * - 记录每个控制周期的控制指令，包括前进（surge）、横向（sway）、上下（heave）以及角速度指令（roll, pitch, yaw）。
 * - 记录控制回路的状态，包括是否激活（armed）、是否紧急停车（estop_latched）、当前模式（effective_mode）以及是否进入故障安全（failsafe）。
 * - 记录导航系统发布的共享内存数据，例如位姿（x, y, z）和姿态（roll, pitch, yaw）。
 * 
 * 目前日志输出为 CSV 格式，包含时间戳（t_s）和各类数据字段，便于后续分析与回溯。
 * 
 * 未来可能的扩展：
 * - 支持更多的控制状态字段和复杂的控制指令（如速度限幅、增量控制等）。
 * - 增加对不同控制模式（手动、自动、故障模式等）下的数据记录。
 * - 增强对多传感器数据的支持，例如集成 IMU 数据、深度传感器、声呐数据等。
 * - 支持多种日志格式（例如 JSONL、数据库存储）以及定制化的输出策略（如按需保存日志、历史数据压缩等）。
 * - 对日志内容进行基于事件的筛选，仅在关键状态变化时记录，以避免日志过大。
 * 
 * 本模块与 `PwmLogger` 完全分离，职责明确：`PwmLogger` 只记录 PWM 控制信号相关数据，`ControlLoopLogger` 负责记录控制回路状态和控制指令等。
 */

struct NavigationData {
    double x;   ///< X 位置
    double y;   ///< Y 位置
    double z;   ///< Z 位置
    double roll;  ///< 滚转角
    double pitch; ///< 俯仰角
    double yaw;   ///< 偏航角
    double depth_m; ///< 深度

    bool present;   ///< 本周期是否拿到了导航快照
    bool valid;     ///< 控制是否认为当前导航可信
    bool stale;     ///< 当前导航是否 stale
    bool degraded;  ///< 当前导航是否 degraded

    std::uint32_t age_ms;       ///< control 看到的累计导航年龄
    std::uint8_t nav_state;     ///< shared::msg::NavRunState
    std::uint8_t nav_health;    ///< shared::msg::NavHealth
    std::uint16_t fault_code;   ///< shared::msg::NavFaultCode
    std::uint16_t sensor_mask;  ///< 当前 fresh 传感器
    std::uint16_t status_flags; ///< 设备/估计器状态位
};

struct ControlEffect {
    float surge;  ///< 前进速度指令
    float sway;   ///< 横向速度指令
    float heave;  ///< 上下速度指令
    float roll;   ///< 滚转角速度指令
    float pitch;  ///< 俯仰角速度指令
    float yaw;    ///< 偏航角速度指令
    bool has_ref;         ///< 是否有参考目标
    bool has_ref_delta;   ///< 是否有参考增量
    bool request_exit;    ///< 是否请求退出
    uint32_t intent_age_ms; ///< 控制指令的过期时间
};

struct ControlGuardOutput {
    bool armed;              ///< 控制器是否已激活
    bool estop_latched;      ///< 紧急停车是否已激活
    int effective_mode;      ///< 当前控制模式（例如：手动、自动）
    bool has_nav;            ///< 是否有导航数据
    bool failsafe;           ///< 是否进入故障安全模式
    bool input_stale;        ///< 当前输入是否已过期
};

/**
 * @class ControlLoopLogger
 * 
 * @brief 控制回路日志记录类
 * 
 * 该类负责记录每个控制周期的控制指令、控制状态以及来自导航系统的共享内存数据。日志内容包括：
 * - 控制指令（例如速度指令、角速度指令等）
 * - 控制状态（例如是否激活、是否处于故障安全模式等）
 * - 导航数据（例如当前位置、姿态等）
 * 
 * 日志输出格式为 CSV 文件，每条记录包含时间戳以及相关控制数据字段，便于后续分析与回溯。
 * 
 * 当前只记录控制状态和控制指令，导航数据来自共享内存。控制指令数据和控制状态不会与 PWM 日志冲突。
 */
class ControlLoopLogger {
public:
    /**
     * @brief 初始化日志记录器
     * 
     * 初始化日志记录器并设置文件保存路径和文件前缀，生成日志文件。
     * 
     * @param root_dir 日志文件根目录路径
     * @param prefix 日志文件前缀
     * 
     * @return 如果初始化成功，返回 true；否则返回 false。
     */
    bool init(const std::string& root_dir, const std::string& prefix);

    /**
     * @brief 关闭日志文件
     * 
     * 确保日志文件被正确关闭并刷新所有内容。
     */
    void close() noexcept;

    /**
     * @brief 判断日志文件是否已成功打开
     * 
     * @return 如果日志文件已打开，返回 true；否则返回 false。
     */
    bool is_open() const noexcept;

    /**
     * @brief 记录控制数据
     * 
     * 记录每个控制周期的控制数据、状态以及导航数据。每次调用会将当前的控制指令和控制状态记录到日志文件。
     * 
     * @param t_s 时间戳（控制周期时间）
     * @param eff 控制指令
     * @param guard_out 控制状态
     * @param nav_data 导航数据（共享内存中的位姿）
     */
    void log_data(double t_s, const ControlEffect& eff, const ControlGuardOutput& guard_out, const NavigationData& nav_data);

    /**
     * @brief 构造函数
     * 
     * 创建一个 ControlLoopLogger 实例。
     */
    ControlLoopLogger();

    /**
     * @brief 析构函数
     * 
     * 在析构时关闭日志文件。
     */
    ~ControlLoopLogger();

private:
    struct Impl;  ///< 私有实现结构体
    std::unique_ptr<Impl> impl_;  ///< 控制循环日志实现

    /**
     * @brief 写入日志文件头部
     * 
     * 在日志文件中写入字段头部，定义日志记录的字段格式。
     */
    void write_header_();

    /**
     * @brief 写入日志数据
     * 
     * 将控制指令、控制状态和导航数据写入日志文件。
     * 
     * @param t_s 时间戳
     * @param eff 控制指令
     * @param guard_out 控制状态
     * @param nav_data 导航数据
     */
    void write_data_(double t_s, const ControlEffect& eff, const ControlGuardOutput& guard_out, const NavigationData& nav_data);
};

} // namespace rovctrl::io

#endif // ROVCTRL_IO_CONTROL_LOOP_LOGGER_HPP
