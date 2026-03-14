# pwm_control_program/docs

这个目录是 `pwm_control_program` 的文档索引。

这份 README 面向：

- 技术开发团队
- 新加入项目的代码学习者
- 需要在“操作说明”和“源码入口”之间切换的人

## 1. 先按角色找文档

### 现场操作和联调人员

优先看：

1. [操作说明.md](/home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/pwm_control_program/docs/操作说明.md)
2. `pwm/pwm_test_procedures.md`
3. `pwm/pwm_teleop_user_manual.md`

### 控制开发者

优先看：

1. [../README.md](/home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/pwm_control_program/README.md)
2. `pwm/pwm_control_architecture.md`
3. `control_loop_sequence.svg`
4. `test/control_algorithm_development_guide.md`

### 代码学习者

优先看：

1. [../README.md](/home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/pwm_control_program/README.md)
2. `pwm/pwm_control_architecture.md`
3. `gcs/gcs_control_and_telemetry_protocol.md`
4. [../../gateway/README.md](/home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/gateway/README.md)

## 2. 文档分区说明

- `pwm/`
  - PWM、安全层、teleop、测试流程
- `gcs/`
  - GCS 控制和 telemetry 协议说明
- `test/`
  - 控制算法和测试指南
- 根目录下的 `.svg`
  - 架构图和时序图

## 3. 推荐阅读顺序

如果你准备改控制主线，建议：

1. 先看 `../README.md`
2. 再看 `pwm/pwm_control_architecture.md`
3. 再看 `control_loop_sequence.svg`
4. 最后再看操作说明和测试计划

如果你准备带新人：

1. 先让他读 `../README.md`
2. 再让他读这份 docs 索引
3. 然后根据角色分流

## 4. 当前最重要的几份文档

- [操作说明.md](/home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/pwm_control_program/docs/操作说明.md)
  - 当前版本给现场使用的实际步骤
- `pwm/pwm_control_architecture.md`
  - 控制和 PWM 安全层的设计说明
- `pwm/pwm_test_procedures.md`
  - 安全测试流程
- `gcs/gcs_control_and_telemetry_protocol.md`
  - GCS 控制与状态字段约定
- [../config/config_reference.md](/home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/pwm_control_program/config/config_reference.md)
  - 配置项索引

## 5. 当前文档边界

这个目录里的文档主要围绕 `pwm_control_program` 自身。

如果你要看：

- GCS 会话与 UDP 桥接，请转到 [gateway/README.md](/home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/gateway/README.md)
- 底层 PWM 传输与安全后端，请转到 [orangepi_send/README.md](/home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/orangepi_send/README.md)
- 导航链路和 replay，请转到 [Underwater-robot-navigation/README.md](/home/wys/orangepi/UnderwaterRobotSystem/Underwater-robot-navigation/README.md)
