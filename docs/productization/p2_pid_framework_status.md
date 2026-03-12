# P2 PID Framework Status

## Scope

本文件用于收口当前 P2 阶段的 PID 控制器框架，不推进新的轨迹跟踪功能，只确认当前已经具备的能力、边界和进入下一阶段的前置条件。

## 已实现内容

### PID 控制器族

- 已落地 `DepthHoldPidController`
- 已落地 `HeadingHoldPidController`
- 已落地 `DepthHeadingPidController`
- 已保留 `pid` 兼容别名，当前指向 `DepthHeadingPidController`

### PID 轴行为

- `dt` 被钳位到安全范围，避免调度抖动导致积分或导数暴涨
- 已支持积分限幅
- 已支持输出速率限制
- `reset()` 会清空积分、导数状态和已锁存 setpoint
- Auto 首周期在没有显式 reference 时，会锁存当前测量值作为 hold setpoint
- `reset` 与同周期新 reference 冲突时，显式新 reference 优先

### ControllerManager

- 已新增显式 `register_controller()`，允许渐进式扩展 Auto 控制器
- 已修复 `init_manual_only()` 下默认 Auto 控制器名未初始化的问题
- `set_mode(kAuto)` 现在不会再出现“表面进入 Auto、实际没有切换控制器”的假状态

### app_context 配置装配

- 已支持从 `control_params.yaml` 装配 PID 参数
- 已支持从 `controllers.default_auto_controller` 装配默认 Auto 控制器
- 已支持从 `modes.default_auto_controller` 装配默认 Auto 控制器
- 当前优先级为：
  1. `controllers.default_auto_controller`
  2. `modes.default_auto_controller`
  3. 内建默认值 `depth_heading_pid`
- 当 `initial_mode=auto` 但默认 Auto 控制器不可用时，系统会显式回退到 `manual`

### Telemetry/状态反映

- 本阶段新增了 `TelemetryFrameV2` 的公共组装函数 `fill_telemetry_frame_v2()`
- `ControlLoop` 与测试现在复用同一套 telemetry 组装逻辑，避免“测试手工拼状态、运行时走另一条逻辑”的分叉
- smoke test 已验证 `active_mode`、`controller_name`、`controller_status`、`thruster_cmd`、`pwm_duty` 能从 PID 输出链正确反映到 telemetry frame

## 当前 Auto 的真实语义

当前 Auto 不是 trajectory tracking。

当前 Auto 更接近 hold controller：

- `DepthHoldPidController`：锁存当前深度并保持
- `HeadingHoldPidController`：锁存当前航向并保持
- `DepthHeadingPidController`：同时锁存当前深度和航向并保持

只有在上层显式提供 `pose_ref` 时，PID 控制器才会对该 setpoint 工作。当前 `trajectory_tracking` 配置虽然能加载，但还没有接入控制闭环，因此不能把当前 Auto 描述为任务轨迹跟踪。

## 未实现内容

- 未把 `trajectory_tracking` 深接入 `ControlLoop`
- 未实现 `VelocityHold`
- 未实现 HIL/实机级 PID 闭环验收
- 未实现 Auto 模式下完整的任务/航点调度
- 未实现 PID 参数在线热更新与整定界面

## 并发与时序结论

### 当前成立的前提

- `ControllerManager` 的注册、默认控制器选择和 `app_context` 装配发生在主循环启动前，不与运行期 `compute()` 并发
- PID 控制器计算仍在单线程控制循环内完成，没有新增共享可变状态跨线程访问
- `TelemetryFrameV2` 组装函数是纯数据映射，不依赖线程执行顺序，不依赖 SHM、新旧帧到达先后

### 本阶段重点确认的问题

- `dt` 不再依赖“调度通常很稳定”的假设，超过边界时会被钳位
- `reset()` 与“同周期新 reference”冲突已经明确成稳定语义：显式 reference 优先
- `Auto` 进入条件不再依赖偶然注册顺序，默认控制器名在初始化阶段即固定
- smoke test 不再依赖 POSIX SHM 权限或外部进程，避免“测试偶然通过、环境一变就失效”

### 仍需注意的时序风险

- 当前 hold controller 的 setpoint 锁存仍依赖“进入 Auto 后至少运行一次 compute()”；这是设计语义，不是 bug，但后续接 trajectory 时必须避免把旧锁存值误当成轨迹参考
- `app_context` 仍在每次测试构建时完整初始化 PWM/allocator/config 依赖，后续若要扩大测试规模，建议继续拆出更细粒度装配层

## 本阶段新增/修正代码

- `pwm_control_program/include/control_core/telemetry_frame_builder.hpp`
- `pwm_control_program/src/control_core/telemetry_frame_builder.cpp`
- `pwm_control_program/src/control_core/loop/control_loop_run.cpp`
- `pwm_control_program/src/control_core/app_context.cpp`
- `pwm_control_program/tests/test_pid_framework.cpp`
- `pwm_control_program/CMakeLists.txt`

## 进入下一阶段前的前置条件

进入 `trajectory_tracking` 深接入前，建议满足以下条件：

1. 明确 Auto 的 reference 来源契约：是 `pose_ref`、轨迹采样点，还是速度参考
2. 明确进入/退出 Auto 时的 bumpless transfer 策略，避免 hold setpoint 与轨迹首点冲突
3. 为 `DepthHeadingPidController` 补充 `ControlLoop` 级回放测试，覆盖参考切换、nav 抖动和 stale
4. 明确 telemetry 中“hold setpoint”和“外部参考 setpoint”的展示字段，避免 UI 把锁存值误判成上位机下发值

## 当前判断

当前 P2 阶段已经具备进入下一步设计工作的基础，但还不适合直接宣称“已具备 trajectory tracking”。如果下一阶段要做轨迹接入，应该先围绕 reference 契约、切换语义和回放测试继续收口，再扩展功能。
