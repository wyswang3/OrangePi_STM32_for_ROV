# pwm_control_program

`pwm_control_program` 是当前机器人侧控制主程序。

它把外部输入意图、导航状态、模式约束、安全逻辑、控制器输出和 PWM 后端串成一条完整的运行时链路。

如果你只看一个目录来理解“机器人到底怎么从 GCS 指令走到 PWM 输出”，这个目录最重要。

## 1. 当前运行链路

当前主线可以概括为：

```text
GCS / local intent / nav view
  -> InputProvider chain
  -> ControlGuard
  -> ControllerManager
  -> Thruster allocation / teleop mixer
  -> PwmClient
  -> telemetry + logs
```

更具体一点：

```text
/rovctrl_gcs_intent_v1
  + /rovctrl_intent_mux_v1
  + /rovctrl_nav_view_v1
    -> pwm_control_program
    -> Guard / mode / failsafe
    -> thruster command
    -> PWM backend
```

## 2. 现在最容易误解的地方

当前版本和历史版本最大的不同之一是：

- `pwm_control_program` 不再把“直接读本地终端键盘”当作主输入路径
- 远程 GCS 或上游 intent SHM 才是当前主用路径

其中：

- `GcsShmInputProvider` 读 `/rovctrl_gcs_intent_v1`
- `TeleopInputProvider` 读最终 intent SHM `/rovctrl_intent_mux_v1`
- `MultiInputProvider` 负责合并它们，当前默认 GCS 优先

## 3. 当前语义基线

在当前控制语义下：

- 程序启动时默认进入 `Manual`
- `Manual` 模式不强依赖导航
- `Auto` 模式要求导航可信
- `ControlGuard` 是最终安全裁决点
- 输入 TTL 过期、急停、未解锁、导航不可信等都会在 guard 层收口

## 4. 目录结构

- `include/control_core/`
  - 主循环、模式、Guard、类型、allocation、telemetry
- `include/controllers/`
  - 控制器接口和具体控制器
- `include/io/`
  - 输入、导航、日志、状态发布
- `include/platform/`
  - `PwmClient`、时间基
- `src/control_core/`
  - 控制主程序和主循环实现
- `src/controllers/`
  - 手动/控制器实现
- `src/io/`
  - SHM 输入、日志、telemetry、导航订阅
- `src/platform/`
  - PWM 客户端与时间
- `config/`
  - 运行参数
- `tests/`
  - 当前最重要的控制侧回归
- `docs/`
  - 开发文档、操作说明和测试说明

## 5. 推荐阅读顺序

如果你想跟主线代码：

1. `src/control_core/app_main.cpp`
2. `src/control_core/app_context.cpp`
3. `src/control_core/loop/control_loop_run.cpp`
4. `src/control_core/control_guard.cpp`
5. `src/controllers/manual_controller.cpp`
6. `src/platform/pwm_client.cpp`

如果你更关心输入：

1. `src/io/input/gcs_shm_input_provider.cpp`
2. `src/io/input/teleop_input.cpp`
3. `src/io/input/multi_input_provider.cpp`
4. `src/io/input/control_intent_wire_codec.cpp`

## 6. 快速构建

在仓库根目录构建：

```bash
cd /home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV
cmake -S . -B build
cmake --build build -j4
```

运行文件一般在：

```text
build/bin/pwm_control_program
```

## 7. 当前最实用的联调命令

不接真实推进器时，推荐先跑 dummy：

```bash
cd /home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/build/bin
./pwm_control_program --no-teleop --pwm-dummy --pwm-dummy-print
```

这条命令的作用是：

- 保留完整控制链
- 计算真实的 PWM duty
- 只打印，不驱动真实推进器

如果你要确认真实 STM32 下发链，请不要只停在这条命令。

至少要满足：

- `pwm_control_program` 没有带 `--pwm-dummy`
- supervisor/status 明确显示 `pwm_backend=stm32`
- 当前联调流程已经显式放行真实 PWM

## 8. 日志与可观测性

当前程序会在运行目录下生成：

- `./logs/control/`
  - 控制循环快照
- `./logs/pwm/`
  - PWM 指令和 applied duty
- `./logs/telemetry/`
  - telemetry timeline 和 command/event 记录

这些日志是 incident bundle 和 replay compare 的关键输入之一。

## 9. 关键配置

常用配置位于 `config/`：

- `pwm_client.yaml`
- `alloc.yaml`
- `control_params.yaml`
- `trajectory.yaml`
- `teleop_mixer.yaml`
- `config_reference.md`

阅读代码之前，先知道这些配置影响什么，会省很多时间。

## 10. 当前最重要的测试

如果你改控制主线，最值得优先看的测试是：

- `tests/test_v1_closed_loop.cpp`
- `tests/test_nav_reconnect_pipeline.cpp`
- `tests/test_nav_view_shm_source.cpp`
- `tests/test_control_loop_logger.cpp`
- `tests/test_telemetry_timeline_logger.cpp`

## 11. 当前边界

`pwm_control_program` 负责控制主循环，但不负责：

- GCS UDP 会话本体
- 导航滤波本体
- STM32 低级执行固件

这些分别由：

- `gateway`
- `nav_core`
- `orangepi_send`

负责。

## 12. 给代码学习者的建议

不要先去读某个控制器公式实现。

更好的路径是：

1. 先看输入怎么进来
2. 再看 Guard 怎么裁决
3. 再看输出怎么变成 duty
4. 最后再看具体控制器细节

这样你学到的是“控制系统”，而不是只学到一个函数。
