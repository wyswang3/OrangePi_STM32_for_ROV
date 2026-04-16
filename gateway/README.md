# gateway

`gateway` 是机器人侧的 runtime bridge layer。

它的定位不是“再做一套控制逻辑”，而是把外部输入和导航状态整理成控制主线可消费的 SHM / view / session abstraction。

## 当前进度 / Current Status

当前 `gateway` 已经稳定承担两条关键链路：

- `gcs_server`：GCS UDP session -> intent SHM
- `nav_viewd`：NavState -> control-consumable NavView

当前它已经属于实际运行主链，不再只是实验辅助模块。

## 这个目录负责什么

负责：

- GCS UDP session 与命令桥接
- 导航状态到控制视图的投影
- intent / telemetry / nav view 的调试观测工具

不负责：

- 最终 mode gating
- 最终 ARM / E-STOP 安全策略
- PWM backend 保护
- navigation estimator 本体

## 当前两条关键链路

GCS command path:

```text
UDP packets
  -> gcs_server
  -> /rovctrl_gcs_intent_v1
  -> pwm_control_program
```

Nav view path:

```text
NavState SHM
  -> nav_viewd
  -> /rovctrl_nav_view_v1
  -> pwm_control_program
```

## 当前思路 / Current Engineering Thinking

`gateway` 应该保持：

- bridge semantics 清楚
- contract boundary 清楚
- view projection 清楚

而不应该：

- 偷偷吸收 control logic
- 代替 `ControlGuard`
- 成为新的 authority center

如果你改这里，先确认自己改的是桥接语义，不是业务语义。

## 目录结构

- `apps/`
  - `gcs_server`、`nav_viewd` 等入口
- `include/gateway/`
  - session、IPC、nav view、intent 等接口
- `src/`
  - 实现
- `tests/`
  - session / codec / nav view 相关测试

## 推荐阅读顺序

1. `apps/gcs_server.cpp`
2. `apps/nav_viewd.cpp`
3. `include/gateway/session/gcs_session.hpp`
4. `include/gateway/IPC/nav/`
5. `include/gateway/IPC/intent/`
6. `tests/test_session.cpp`
7. `tests/test_nav_view_policy.cpp`

## 相关文档

当前仓级文档入口：

- `../docs/控制系统总览.md`
- `../docs/控制安全与运行语义.md`
- `../docs/通信协议与遥测说明.md`
