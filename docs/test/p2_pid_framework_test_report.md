# P2 PID Framework Test Report

## Summary

本轮验收目标是对当前 P2 PID 框架做阶段收口，不新增轨迹跟踪功能，只验证以下内容：

- PID 控制器基础行为
- `ControllerManager` 的 Auto 切换语义
- `app_context` 的 PID 配置装配与默认控制器选择
- 最小软件链路 `Auto/PID -> Allocator -> PWM -> Telemetry`

结果：本轮新增测试和现有回归测试全部通过。

## 新增测试

测试入口：

- `pwmctrl_test_pid_framework`

覆盖内容：

- `test_pid_axis_dt_clamp_and_integral_limit`
  - 验证 `dt` 钳位
  - 验证积分限幅
- `test_pid_axis_rate_limit`
  - 验证输出速率限制
- `test_depth_pid_direction_latch_and_reset`
  - 验证输出方向
  - 验证首周期 setpoint 锁存
  - 验证 `reset()` 后重新锁存
- `test_reset_and_same_cycle_reference_priority`
  - 验证 reset 与同周期新 reference 的优先级
- `test_controller_manager_auto_switch_matrix`
  - 验证无 Auto 控制器时进入 Auto 失败
  - 验证注册默认控制器后 Auto 切换成功
  - 验证显式选择控制器后 Auto 切换命中目标控制器
- `test_app_context_loads_pid_params_and_default_auto_controller`
  - 验证 PID 参数加载
  - 验证默认 Auto 控制器加载
- `test_app_context_invalid_default_auto_falls_back_to_manual`
  - 验证无效默认 Auto 控制器时回退到 Manual
- `test_app_context_loads_modes_default_auto_controller`
  - 验证 `modes.default_auto_controller` 生效
- `test_app_context_controller_default_overrides_modes_default`
  - 验证 `controllers.default_auto_controller` 覆盖 `modes.default_auto_controller`
- `test_pid_software_chain_smoke`
  - 验证进入 Auto 后是否真的选中 PID 控制器
  - 验证 PID 输出是否进入 allocator
  - 验证 thruster 输出和 PWM duty 是否发生变化
  - 验证 telemetry frame 是否反映 `mode/controller/status/output`

## 代码收口调整

本轮验收中顺带完成了一项必要重构：

- 抽出 `fill_telemetry_frame_v2()` 作为公共 telemetry 组装函数

原因：

- 原 smoke test 只能手工拼 `TelemetryFrameV2`
- 这种方式无法证明测试和生产路径一致
- 抽成公共函数后，`ControlLoop` 与测试共用同一逻辑，减少“测试过了但运行态不一致”的风险

## 构建验证

每次有意义修改后都做了即时构建检查。

已执行的构建：

- `cmake --build OrangePi_STM32_for_ROV/build --target pwm_control_program -j4`
- `cmake --build OrangePi_STM32_for_ROV/build --target test_pid_framework -j4`
- `cmake --build OrangePi_STM32_for_ROV/build -j4`

## 测试验证

专项测试：

- `./OrangePi_STM32_for_ROV/build/bin/test_pid_framework`

阶段回归：

- `ctest --test-dir OrangePi_STM32_for_ROV/build --output-on-failure`

通过结果：

- `gateway_test_codec`
- `gateway_test_session`
- `pwmctrl_test_v1_closed_loop`
- `pwmctrl_test_pid_framework`

## 本轮发现的问题与修复

### 1. 公共 telemetry builder 初次构建失败

问题：

- `telemetry_frame_builder.cpp` 初次编译时缺少 `shared/msg/control_intent.hpp`

修复：

- 补充显式 include，消除对间接头文件的偶然依赖

### 2. smoke test 依赖 POSIX SHM 权限

问题：

- 早期版本的 smoke test 通过 `TelemetryPublisherShm` 和 `TelemetryFrameV2SubscriberShm` 验证 telemetry
- 在当前沙箱环境下 `shm_open` 会报 `Permission denied`

修复：

- 不再让 smoke test 依赖 OS SHM 权限
- 改为调用生产态 `fill_telemetry_frame_v2()` 验证 telemetry 映射

### 3. 生产态 telemetry 与测试逻辑分叉

问题：

- 原测试手工拼 `TelemetryFrameV2`
- 无法证明生产态 `ControlLoop` 与测试走的是同一条状态映射逻辑

修复：

- 抽出公共 builder
- `ControlLoop` 与 `test_pid_framework` 共用相同映射实现

## 并发/时序检查结论

- 本轮新增 PID 测试没有引入新的跨线程共享状态
- `ControllerManager::register_controller()` 与 `app_context` 装配仍在启动期完成，不与运行期 `compute()` 并发
- `dt` 钳位、积分限幅、速率限制已经覆盖了 CPU 调度抖动带来的主要数值风险
- `reset` 与“同周期新 reference”优先级已固定，不依赖线程先后或偶然时序
- smoke test 已验证闭环软件链路，但没有覆盖实际 `ControlLoop::run()` 的周期调度行为

## 未覆盖项

- 未覆盖 HIL/实机行为
- 未覆盖 trajectory tracking 接入
- 未覆盖 nav 抖动下的长时 Auto 保持稳定性
- 未覆盖 PID 在真实多周期 `ControlLoop` 运行中的统计性时序表现
- 未覆盖参数热更新

## 验收结论

当前 P2 PID 框架满足阶段验收目标：

- 可以稳定描述为 hold-controller 型 Auto
- 可以稳定进入指定 PID 控制器
- 可以贯通到 allocator、PWM 与 telemetry 表达层
- 当前不应描述为 trajectory tracking

是否适合进入下一阶段：

- 适合进入“trajectory tracking 接入前的设计与接口收口阶段”
- 还不适合直接进入“大规模轨迹功能开发”或“对外宣称 Auto 任务控制完成”
