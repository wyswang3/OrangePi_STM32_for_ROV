# orangepi_send

`orangepi_send` 是控制仓里的低级 PWM 后端与传输模块。

它提供的是“如何安全地把归一化推进器指令变成可发送、可保护、可落到 STM32 的 PWM 数据”，而不是高层控制逻辑。

## 1. 模块定位

在系统里，`orangepi_send` 位于：

```text
pwm_control_program
  -> PwmClient
  -> orangepi_send
  -> STM32
```

它解决的是下面这些问题：

- 如何把目标推进器命令打包成协议帧
- 如何通过 UDP 向下位机发送
- 如何做限斜率、急停、回中和反向保护
- 如何维持心跳与基础传输健康

## 2. 当前主要文件

### 头文件

- `include/libpwm_host.h`
  - OrangePi 侧 host API
- `include/pwm_control.h`
  - PWM 安全层 API
- `include/protocol_pack.h` / `.hpp`
  - 协议打包接口
- `include/UdpSender.h`
  - UDP 发送器
- `include/PwmFrameBuilder.h`
  - PWM 帧构造
- `include/crc16_ccitt.h`
  - CRC 校验

### 源文件

- `src/libpwm_host.c`
- `src/pwm_control.c`
- `src/protocol_pack.c`
- `src/UdpSender.cpp`
- `src/PwmFrameBuilder.cpp`
- `src/crc16_ccitt.cpp`
- `src/main.cpp`

## 3. 它负责什么，不负责什么

负责：

- 协议帧
- UDP 发送
- PWM 安全保护
- 心跳与基础传输状态

不负责：

- 手动模式 / 自动模式切换
- GCS 会话
- 导航可信度
- 控制器计算

这些都属于上层。

## 4. 当前最值得先看的代码

建议按这个顺序看：

1. `include/pwm_control.h`
2. `src/pwm_control.c`
3. `include/libpwm_host.h`
4. `src/libpwm_host.c`
5. `src/protocol_pack.c`
6. `src/main.cpp`

这条路径最能帮助你理解“PwmClient 依赖了什么”。

## 5. 当前安全语义

这里最关键的不是协议，而是安全层：

- 限斜率
- 回中
- 心跳超时
- 急停
- 反向保护

上层就算给出一组不平滑的目标，真正落到底层之前也要经过这里。

所以如果你在 `pwm_control_program` 看到的是归一化命令，而在底层看到的是 duty 变化，中间的语义桥就在这里。

## 6. 构建

当前一般通过控制仓顶层一起构建：

```bash
cd /home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV
cmake -S . -B build
cmake --build build -j4
```

如果你单独进这个目录，也可以基于本目录的 `CMakeLists.txt` 做局部构建。

## 7. 与上层的关系

对上层来说，最重要的事实只有一个：

`orangepi_send` 是后端，不是业务中心。

如果你准备改这里，最好先确认你改的是：

- 传输
- 协议
- 安全层

而不是偷偷把控制模式、GCS 或导航逻辑塞进来。

## 8. 文档入口

这个目录相关的仓级基线说明已经并入：

- `../docs/控制系统总览.md`
- `../docs/控制安全与运行语义.md`
- `../docs/通信协议与遥测说明.md`
- `../docs/控制器调参与测试指南.md`

如果你要做的是协议、执行保护或 bench 回归，优先读这些基线文档，再回到本目录源码。
