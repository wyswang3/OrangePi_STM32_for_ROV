# orangepi_send

`orangepi_send` 是当前控制仓里的 low-level PWM backend。

它负责把上层已经归一化、已经过 guard 的推进器命令，转换成可发送、可保护、可落到 STM32 的 PWM 数据。

## 当前进度 / Current Status

这个目录当前的定位已经比较稳定：

- 作为 `PwmClient` 的后端依赖存在
- 承担协议打包、UDP 发送和安全保护
- 不再承载高层 mode / session / navigation 语义

## 模块用途

负责：

- protocol packing
- UDP sending
- PWM safety protection
- heartbeat / transport health

不负责：

- Manual / Auto mode decision
- GCS session
- nav confidence judgement
- controller computation

## 当前运行位置

```text
pwm_control_program
  -> PwmClient
  -> orangepi_send
  -> STM32
```

## 当前思路 / Current Engineering Thinking

这里最关键的不是“发 UDP”这件事本身，而是：

- duty change 怎样被平滑和保护
- estop / timeout / neutralize 怎样在后端继续生效

也就是说，这里是 backend safety layer，不是业务中心。

## 当前最值得先看的代码

1. `include/pwm_control.h`
2. `src/pwm_control.c`
3. `include/libpwm_host.h`
4. `src/libpwm_host.c`
5. `src/protocol_pack.c`
6. `src/main.cpp`

## Build

通常通过控制仓顶层一起构建：

```bash
cd /home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV
cmake -S . -B build
cmake --build build -j4
```

## 相关文档

当前仓级基线说明已收口到：

- `../docs/控制系统总览.md`
- `../docs/控制安全与运行语义.md`
- `../docs/通信协议与遥测说明.md`
- `../docs/控制器调参与测试指南.md`
