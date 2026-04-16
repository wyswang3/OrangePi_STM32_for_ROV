# OrangePi_STM32_for_ROV

`OrangePi_STM32_for_ROV` 是当前机器人侧控制、会话桥接、telemetry 和 PWM 执行链仓库。

这个仓库面向后续开发者时，最重要的不是“怎么启动”，而是三条技术主线：

1. 外部输入怎样进入控制栈
2. 安全和模式裁决在哪里生效
3. 归一化控制量怎样变成最终 PWM 输出

## 1. Main Runtime Pipelines

Current remote-control path:

```text
GCS(TUI/GUI)
  -> gateway/gcs_server
  -> /rovctrl_gcs_intent_v1
  -> pwm_control_program
  -> PwmClient
  -> orangepi_send / STM32 / ESC / Thrusters
```

Navigation-to-control path:

```text
nav_core/NavState
  -> gateway/nav_viewd
  -> /rovctrl_nav_view_v1
  -> pwm_control_program
```

Important authority boundaries:

- `Manual` 模式不强依赖导航
- `Auto` 模式要求导航可信
- 最终安全裁决点在 `ControlGuard`

## 2. Documentation Policy

This repo's docs should primarily explain:

- technical design
- runtime contracts
- module boundaries
- safety semantics
- low-level implementation choices

Field startup, cross-machine bring-up, and operator sequencing should live in the system doc repo instead of being the primary entry point here.

Start with:

1. `docs/文档总览.md`
2. `docs/控制系统总览.md`
3. `pwm_control_program/README.md`
4. `docs/控制安全与运行语义.md`
5. `docs/通信协议与遥测说明.md`
6. `docs/演进记录与边界约束.md`

## 3. Repository Layout

- `pwm_control_program/`
  - 机器人侧控制核心
- `gateway/`
  - UDP、SHM、会话和视图桥接
- `orangepi_send/`
  - PWM 传输与安全层后端
- `proto_gcs/`
  - 协议相关的共享定义
- `docs/`
  - 架构、接口、测试、产品化文档

## 4. What Each Module Owns

### `pwm_control_program`

负责：

- 输入整合
- 模式管理
- ARM / E-STOP / stale 处理
- 控制器调用
- 推力分配
- telemetry 生成
- PwmClient 驱动

### `gateway`

负责：

- GCS UDP 会话与命令桥接
- NavState -> NavView 投影
- intent / telemetry 相关 SHM 辅助工具

### `orangepi_send`

负责：

- OrangePi 到 STM32 的 PWM 传输后端
- PWM 安全层
- 协议打包和校验

## 5. Recommended Reading Order

如果你要先理解“这个仓为什么会长成现在这样”，建议顺序是：

1. `docs/文档总览.md`
2. `docs/控制系统总览.md`
3. `pwm_control_program/README.md`
4. `gateway/README.md`
5. `orangepi_send/README.md`

然后再进入源码。

## 6. Recommended Code Reading Order

If you want the control path first:

1. `docs/控制系统总览.md`
2. `pwm_control_program/src/control_core/app_main.cpp`
3. `pwm_control_program/src/control_core/loop/control_loop_run.cpp`
4. `pwm_control_program/src/control_core/control_guard.cpp`
5. `pwm_control_program/src/controllers/manual_controller.cpp`
6. `gateway/apps/gcs_server.cpp`
7. `gateway/apps/gcs_client.cpp`

If you want the data-contract path first:

1. `shared/msg/control_intent.hpp`
2. `docs/通信协议与遥测说明.md`
3. `docs/控制安全与运行语义.md`
4. `pwm_control_program/src/io/input/control_intent_wire_codec.cpp`

## 7. Build

At repo root:

```bash
cd <OrangePi_STM32_for_ROV repo root>
cmake -S . -B build
cmake --build build -j4
```

Common binaries in `build/bin/`:

- `pwm_control_program`
- `gcs_server`
- `nav_viewd`
- `telemetry_dump`
- `intentd`
- `teleop_local`

## 8. What To Read When Debugging Motion Mapping

For "why did that key press produce this motion?" questions, inspect in this order:

1. `UnderWaterRobotGCS/src/urogcs/control/keyboard_mapper.py`
2. `gateway/apps/gcs_client.cpp`
3. `/rovctrl_gcs_intent_v1`
4. `pwm_control_program/src/io/input/gcs_shm_input_provider.cpp`
5. `pwm_control_program/src/control_core/control_guard.cpp`
6. `pwm_control_program/src/control_core/loop/control_loop_helpers.cpp`
7. `pwm_control_program/src/controllers/manual_controller.cpp`
8. `pwm_control_program/config/teleop_mixer.yaml`
9. `orangepi_send/`

## 9. Minimal Dummy-Backend Bring-Up For Developers

When you only need the control path and do not want real thrust:

OrangePi side:

```bash
./build/bin/gcs_server
./build/bin/pwm_control_program --no-teleop --pwm-dummy --pwm-dummy-print
```

GCS side:

```bash
cd <UnderWaterRobotGCS repo root>
UROGCS_ROV_IP=<OrangePi_IP> PYTHONPATH=src python -m urogcs.app.tui.tui_main
```

This proves the end-to-end control path, but not real STM32 output.

## 10. Documentation Cleanup Rule

这个仓库里仍然保留了一些阶段性计划、测试报告和操作说明，但它们不再是开发者主入口。

阅读顺序遵循下面的规则：

- 先读当前技术基线和升级路线
- 再读模块 README 和接口文档
- 历史计划、阶段复盘、操作手册只作为证据和背景

## 11. Common Misunderstandings

- `pwm_control_program` is not the session server
- `gateway` is not the final safety authority
- `orangepi_send` does not decide mode logic
- GCS key mapping is not the same thing as thruster allocation
- local keyboard bench flow and upper-computer GCS flow are not identical paths

This repo is easiest to understand as a runtime chain, not as isolated modules.
