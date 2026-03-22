# ROS2 Bridge Stage1

这个目录承载 UnderwaterRobotSystem 的 ROS2 外围桥接第一阶段实现。

边界原则：

- 这里只做 mirror / diagnostics / UI-backend / logging 入口
- 不回灌 `ControlIntent`
- 不替换 `nav_viewd`
- 不让 `ControlLoop` / `ControlGuard` / PWM/STM32 依赖 ROS2 graph

当前目录结构：

- `rov_msgs/`
  - ROS2 mirror 消息包
  - 通过 `colcon build` 生成 `rov_msgs.msg.*`
- `rov_state_bridge/`
  - 只读 SHM reader、mirror mapping、publisher backend、health monitor、launch 与验证工具

当前 workspace 用法：

```bash
cd /home/wys/orangepi/UnderwaterRobotSystem/OrangePi_STM32_for_ROV/ros2_bridge
PATH=/usr/bin:/bin:$PATH . /opt/ros/humble/setup.bash
colcon build --event-handlers console_direct+   --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
. install/setup.bash
```

说明：

- 这台机器若直接使用 conda Python 3.11，`rosidl` 生成阶段可能因为缺少 `lark` 而失败。
- 当前已验证通过的 build / graph / rosbag 路径使用系统 Python：`/usr/bin/python3`。

本阶段已验证的本机检查：

```bash
/usr/bin/python3 rov_state_bridge/tools/run_ros2_graph_validation.py
/usr/bin/python3 rov_state_bridge/tools/run_rosbag_validation.py
```

它们会验证：

- generated `rov_msgs` 字段与 bridge-side model 对齐
- `/rov/telemetry`
- `/rov/health`
- `/rov/nav_view`
- `/rov/nav_state_raw`
- `/rov/health_monitor`
- rosbag2 录包与回放

本阶段默认假设：

- 权威状态源仍是 `shared/msg/*` + `shared/shm/*`
- bridge 在 Linux 上从 `/dev/shm` 或 file-backed source 只读映射现有状态
- 即使 ROS2 graph 故障，核心 nav/control 主链也必须保持可独立运行