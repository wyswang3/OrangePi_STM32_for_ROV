# ROS2 Bridge Stage1

这个目录承载当前项目的 ROS2 peripheral bridge stage1。

它的边界必须非常清楚：

- 只做 read-only mirror / diagnostics / UI-backend / logging entry
- 不回灌 `ControlIntent`
- 不替换 `nav_viewd`
- 不让 `ControlLoop` / `ControlGuard` / PWM / STM32 依赖 ROS2 graph

## 当前进度 / Current Status

当前这个目录已经具备：

- `rov_msgs/` mirror message package
- `rov_state_bridge/` 只读 bridge、health monitor、validation tooling
- graph validation / rosbag validation 基本链路

当前仍然不是：

- control authority layer
- final estop / arm layer
- nav / control runtime owner

## 目录结构

- `rov_msgs/`
  - ROS2 mirror messages
- `rov_state_bridge/`
  - SHM reader、mapping、publisher backend、health monitor、launch、validation tools

## 当前思路 / Current Engineering Thinking

这个目录的价值在于：

- 给 ROS2 外围消费者提供稳定 mirror
- 不破坏核心 nav / control 主链
- 让 health monitor、UI backend、rosbag2 等外围功能可以先发展

换句话说，ROS2 在这里是 peripheral layer，不是 authority layer。

## Build

```bash
cd /home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/ros2_bridge
PATH=/usr/bin:/bin:$PATH . /opt/ros/humble/setup.bash
colcon build --event-handlers console_direct+ \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
. install/setup.bash
```

说明：

- 当前更稳定的构建路径仍是系统 Python：`/usr/bin/python3`
- 直接用 conda Python 3.11 时，`rosidl` 生成阶段可能因为依赖缺失失败

## 当前已验证的检查

```bash
/usr/bin/python3 rov_state_bridge/tools/run_ros2_graph_validation.py
/usr/bin/python3 rov_state_bridge/tools/run_rosbag_validation.py
```

它们当前会覆盖：

- generated `rov_msgs` 字段与 bridge model 对齐
- `/rov/telemetry`
- `/rov/health`
- `/rov/nav_view`
- `/rov/nav_state_raw`
- `/rov/health_monitor`
- rosbag2 record / replay

## 当前前提

- authority state real source 仍是 `shared/msg/*` + `shared/shm/*`
- bridge 只读映射 `/dev/shm` 或 file-backed source
- 即使 ROS2 graph 故障，核心 nav / control 主链也必须可独立运行

## 相关文档

更完整的系统级路线说明在：

- `../docs/文档总览.md`
- `../docs/通信协议与遥测说明.md`
- `../../UnderwaterRobotSystem/docs/ros2_route/ros2_bridge_stage1_plan.md`
