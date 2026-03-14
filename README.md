# OrangePi_STM32_for_ROV

`OrangePi_STM32_for_ROV` 是当前项目的控制与执行链仓库。

它覆盖了从“GCS 或本地输入意图”到“PWM 输出后端”的主要闭环，是机器人侧控制主线的核心仓库。

## 1. 当前主线是什么

当前已经验证的控制主线是：

```text
GCS(TUI/GUI)
  -> gateway/gcs_server
  -> /rovctrl_gcs_intent_v1
  -> pwm_control_program
  -> PwmClient
  -> orangepi_send / STM32 / ESC / Thrusters
```

导航进入控制的主线是：

```text
nav_core/NavState
  -> gateway/nav_viewd
  -> /rovctrl_nav_view_v1
  -> pwm_control_program
```

当前语义边界很重要：

- `Manual` 模式不强依赖导航
- `Auto` 模式要求导航可信
- 最终安全裁决点在 `ControlGuard`

## 2. 仓库结构

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

## 3. 哪个目录负责什么

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

## 4. 当前推荐的开发阅读顺序

如果你是第一次看这个仓库，建议按下面顺序：

1. 本 README
2. [pwm_control_program/README.md](/home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/pwm_control_program/README.md)
3. [gateway/README.md](/home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/gateway/README.md)
4. [orangepi_send/README.md](/home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/orangepi_send/README.md)

如果你更偏代码学习：

1. 先理解 `ControlIntent` 和 `NavView`
2. 再看 `ControlGuard`
3. 再看 `PwmClient`
4. 最后再回来看 `gateway` 和 UDP/session

## 5. 快速构建

在仓库根目录：

```bash
cd /home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV
cmake -S . -B build
cmake --build build -j4
```

常见二进制位于 `build/bin/`，包括：

- `pwm_control_program`
- `gcs_server`
- `nav_viewd`
- `telemetry_dump`
- `intentd`
- `teleop_local`

## 6. 当前最实用的联调方式

在没有真实推进器输出需求时，先用 dummy 后端做联调：

香橙派侧：

```bash
./build/bin/gcs_server
./build/bin/pwm_control_program --no-teleop --pwm-dummy --pwm-dummy-print
```

GCS 侧：

```bash
cd /home/wys/orangepi/UnderWaterRobotGCS
UROGCS_ROV_IP=<OrangePi_IP> PYTHONPATH=src python -m urogcs.app.tui.tui_main
```

这是当前已经验证能看到 PWM duty 变化的最小路径。

## 7. 当前仓库里最容易误解的点

有几个边界必须说清楚。

- `pwm_control_program` 当前不再把“本地终端键盘输入”当作主路径
- GCS 远程路径是当前更稳定、更常用的键盘控制方式
- `gateway` 不负责最终控制安全
- `orangepi_send` 不负责模式和任务逻辑

## 8. 对技术团队和学习者的建议

这个仓库跨了 C、C++、共享内存、UDP 和硬件后端。

最容易学懂的顺序不是“从最底层开始”，而是：

1. 从控制链顶层入口看整体数据流
2. 再看安全裁决
3. 再看后端输出
4. 最后再看桥接和协议

这样你读到的不是零件，而是一条能跑起来的链路。
