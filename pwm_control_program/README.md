# pwm_control_program

`pwm_control_program` 是当前机器人侧 control main loop。

如果只看一个目录来理解“intent / nav view / mode / safety / controller / PWM backend”怎样串成完整运行链，这里最关键。

## 当前进度 / Current Status

当前版本已经不再以“本地终端键盘直驱”为主，而是：

- 远程 GCS intent 成为主用输入路径
- `ControlGuard` 成为最终安全裁决点
- `Manual` 成为当前成熟主路径
- `Auto` 保持 gated、保守、依赖导航可信
- telemetry 输出已与当前系统级 UI / protocol 口径对齐

## 模块用途 / What This Module Owns

负责：

- 输入整合
- 模式管理
- guard / stale / estop / arm gating
- controller 调用
- thruster allocation / teleop mixer
- PwmClient 驱动
- telemetry 和控制日志

不负责：

- GCS UDP session
- navigation estimator
- STM32 固件

## 当前运行链路

```text
GCS / local intent / nav view
  -> InputProvider chain
  -> ControlGuard
  -> ControllerManager
  -> Thruster allocation
  -> PwmClient
  -> telemetry + logs
```

## 当前思路 / Current Engineering Thinking

这里最重要的不是某个 controller 公式，而是：

- 输入从哪里来
- 安全门控在哪里生效
- 何时允许把命令落到 PWM backend

所以推荐先理解：

- `GcsShmInputProvider`
- `MultiInputProvider`
- `ControlGuard`
- `PwmClient`

再去看具体 controller。

## 目录结构

- `include/control_core/`
- `include/controllers/`
- `include/io/`
- `include/platform/`
- `src/control_core/`
- `src/controllers/`
- `src/io/`
- `src/platform/`
- `config/`
- `tests/`

## 推荐阅读顺序

主线优先：

1. `src/control_core/app_main.cpp`
2. `src/control_core/app_context.cpp`
3. `src/control_core/loop/control_loop_run.cpp`
4. `src/control_core/control_guard.cpp`
5. `src/controllers/manual_controller.cpp`
6. `src/platform/pwm_client.cpp`

输入优先：

1. `src/io/input/gcs_shm_input_provider.cpp`
2. `src/io/input/teleop_input.cpp`
3. `src/io/input/multi_input_provider.cpp`
4. `src/io/input/control_intent_wire_codec.cpp`

## Build

```bash
cd <OrangePi_STM32_for_ROV repo root>
cmake -S . -B build
cmake --build build -j4
```

## 常用联调方式

dummy backend：

```bash
cd <OrangePi_STM32_for_ROV repo root>/build/bin
./pwm_control_program --no-teleop --pwm-dummy --pwm-dummy-print
```

它证明的是 control path 能跑通，不等于真实 STM32 输出已经放行。

## 关键配置

`config/` 里当前最常用的文件包括：

- `pwm_client.yaml`
- `alloc.yaml`
- `control_params.yaml`
- `trajectory.yaml`
- `teleop_mixer.yaml`
- `config_reference.md`

## 相关文档

- `../docs/控制系统总览.md`
- `../docs/控制安全与运行语义.md`
- `../docs/通信协议与遥测说明.md`
- `../docs/控制器调参与测试指南.md`
