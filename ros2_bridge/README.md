# ROS2 Bridge Stage1

这个目录承载 UnderwaterRobotSystem 的 ROS2 外围桥接第一阶段实现。

边界原则：

- 这里只做 mirror / diagnostics / UI-backend / logging 入口
- 不回灌 `ControlIntent`
- 不替换 `nav_viewd`
- 不让 `ControlLoop` / `ControlGuard` / PWM/STM32 依赖 ROS2 graph

当前目录结构：

- `rov_msgs/msg/`
  - 第一批 ROS2 mirror 消息定义
- `rov_state_bridge/`
  - 只读 SHM reader、mirror mapping、publisher backend 和最小验证

本阶段默认假设：

- 权威状态源仍是 `shared/msg/*` + `shared/shm/*`
- bridge 在 Linux 上从 `/dev/shm` 只读映射现有状态
- 若本机缺少 ROS2 工具链，可先运行 mapping / reader / stdout backend 验证
