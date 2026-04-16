# OrangePi_STM32_for_ROV

`OrangePi_STM32_for_ROV` 是当前机器人侧 control / gateway / telemetry / PWM execution 仓库。

如果你想理解“外部输入怎样进入控制栈、怎样经过安全门控、怎样变成最终 PWM 输出”，这里是主入口。

## 当前阶段 / Current Status

当前这个仓已经完成了一轮 P0 authority / telemetry / mode baseline 收口，并进入：

- P1 bring-up / reconnect / diagnostics 持续收口
- P2 controller framework 受控推进

当前稳定下来的事实包括：

- `Manual` 是当前主用模式
- `Auto` 需要导航可信，且仍更接近 hold-controller 语义
- `ControlGuard` 是最终安全裁决点
- `TelemetryFrameV2` 已成为当前权威状态输出

当前仍在推进的重点包括：

- controller framework 扩展
- 更完整的 nav reconnect 负路径验证
- 更清晰的 telemetry / command result 诊断语义

## 这个仓当前负责什么

负责：

- GCS 命令桥接
- nav view 投影
- control loop / guard / mode / allocator
- telemetry 输出
- OrangePi -> STM32 PWM backend

不负责：

- GCS UI
- navigation estimator 本体
- shared contract 真源治理以外的系统级文档

## 当前主链路

远程控制主线：

```text
GCS(TUI/GUI)
  -> gateway/gcs_server
  -> /rovctrl_gcs_intent_v1
  -> pwm_control_program
  -> PwmClient
  -> orangepi_send / STM32 / ESC / Thrusters
```

导航输入主线：

```text
NavState
  -> gateway/nav_viewd
  -> /rovctrl_nav_view_v1
  -> pwm_control_program
```

## 当前技术思路 / Current Engineering Thinking

这个仓的工程重点是：

- authority boundary 要清楚
- final safety gating 只能在车端完成
- GCS request、nav confidence、mode semantics、PWM output 必须分层处理

所以不要把：

- session bridge
- control loop
- backend transport

混成一个模块去理解。

## 目录结构

- `pwm_control_program/`
  - 控制主程序
- `gateway/`
  - UDP / SHM / session / nav view bridge
- `orangepi_send/`
  - PWM backend 与传输安全层
- `ros2_bridge/`
  - ROS2 只读 mirror / diagnostics / UI backend preview
- `docs/`
  - 当前中文基线文档

## 推荐阅读顺序

建议先按这个顺序理解：

1. `docs/文档总览.md`
2. `docs/控制系统总览.md`
3. `pwm_control_program/README.md`
4. `gateway/README.md`
5. `orangepi_send/README.md`
6. `ros2_bridge/README.md`

## Build

```bash
cd <OrangePi_STM32_for_ROV repo root>
cmake -S . -B build
cmake --build build -j4
```

常见目标：

- `build/bin/pwm_control_program`
- `build/bin/gcs_server`
- `build/bin/nav_viewd`
- `build/bin/telemetry_dump`

## 当前文档入口

当前最重要的仓内基线文档：

- `docs/文档总览.md`
- `docs/控制系统总览.md`
- `docs/控制安全与运行语义.md`
- `docs/通信协议与遥测说明.md`
- `docs/控制器调参与测试指南.md`
- `docs/演进记录与边界约束.md`
