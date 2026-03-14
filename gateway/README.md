# gateway

`gateway` 是机器人侧的桥接层。

它不做最终控制决策，也不直接做 PWM 输出；它负责把不同来源的数据流变成控制主线可以消费的 SHM 或状态视图。

## 1. gateway 的职责

当前 `gateway` 主要做三类事情：

- GCS UDP 会话与命令桥接
- 导航状态到控制视图的转换
- 调试/观测工具

可以把它理解成“运行时数据桥”和“边界适配层”。

## 2. 当前最重要的两条链路

### GCS 命令链

```text
UDP packets
  -> gcs_server
  -> /rovctrl_gcs_intent_v1
  -> pwm_control_program
```

`gcs_server` 是当前远程键盘/会话控制的主入口。

### 导航视图链

```text
NavState SHM
  -> nav_viewd
  -> /rovctrl_nav_view_v1
  -> pwm_control_program
```

`nav_viewd` 负责 stale 策略、视图裁剪和控制侧可消费的导航投影。

## 3. 目录结构

- `apps/`
  - 可执行程序入口
- `include/gateway/`
  - 桥接层接口、协议、SHM、会话、视图构建
- `src/`
  - 实现
- `tests/`
  - codec、session、nav view 相关测试
- `docs/`
  - 说明文档

## 4. 当前主要二进制

常见目标都位于 `build/bin/`：

- `gcs_server`
  - 接收 GCS UDP，发布 GCS intent SHM
- `nav_viewd`
  - 订阅 `NavState`，发布控制可消费的 `NavView`
- `telemetry_dump`
  - 订阅并打印 telemetry SHM
- `intent_dump`
  - 打印 intent SHM 内容
- `nav_view_dump`
  - 打印 nav view SHM 内容
- `teleop_local`
  - 本地键盘事件采集工具
- `intentd`
  - 本地/远程/自动意图仲裁实验入口

## 5. 当前推荐使用方式

对远程控制和联调，优先使用：

- `gcs_server`
- `nav_viewd`
- `telemetry_dump`

`teleop_local + intentd` 这条本地键盘路径当前更适合 bench 或实验用途，不建议把它当成首选操作链。

原因很简单：

- 远程 GCS 链路是当前更常用、更充分验证的路径
- 本地键盘链路涉及额外的 intent 合成和 SHM 命名约定，开发时更容易踩边界问题

## 6. 推荐阅读顺序

如果你想快速看懂 `gateway`，建议按下面顺序：

1. `apps/gcs_server.cpp`
2. `apps/nav_viewd.cpp`
3. `include/gateway/session/gcs_session.hpp`
4. `include/gateway/IPC/nav/`
5. `include/gateway/IPC/intent/`
6. `tests/test_session.cpp`
7. `tests/test_nav_view_policy.cpp`

## 7. 当前最重要的边界

`gateway` 负责桥接，不负责：

- 模式门控最终判定
- ARM / E-STOP 的最终安全策略
- PWM 限斜率或驱动保护
- 导航滤波本体

这些分别属于：

- `pwm_control_program`
- `orangepi_send`
- `nav_core`

## 8. 开发时最值得记住的原则

如果你准备改 `gateway`，先问自己：

- 我是在改“桥接语义”，还是在偷偷把控制逻辑塞进桥接层？

如果是后者，通常方向就错了。

`gateway` 应该保持“把数据运过去、格式理清楚、边界理明白”，而不是成为新的业务中心。
