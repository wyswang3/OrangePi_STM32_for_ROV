"""ROS2 launch entry for the stage1 read-only preview bridge.

作用：
- 在 ROS2 环境里同时拉起只读 bridge 与 advisory health monitor；
- 把常用 SHM 源和轮询频率暴露成 launch 参数，便于现场预览和验证。

实现思路：
- launch 文件只负责节点编排和参数透传；
- 具体 bridge/health 语义仍由各自 Python 节点内部实现，避免 launch 层承载业务逻辑。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    backend = LaunchConfiguration('backend')
    telemetry_source = LaunchConfiguration('telemetry_source')
    nav_view_source = LaunchConfiguration('nav_view_source')
    nav_state_source = LaunchConfiguration('nav_state_source')
    poll_hz = LaunchConfiguration('poll_hz')

    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='ros2'),
        DeclareLaunchArgument('telemetry_source', default_value='/rovctrl_telemetry_v2'),
        DeclareLaunchArgument('nav_view_source', default_value='/rovctrl_nav_view_v1'),
        DeclareLaunchArgument('nav_state_source', default_value='/rov_nav_state_v1'),
        DeclareLaunchArgument('poll_hz', default_value='10.0'),
        Node(
            package='rov_state_bridge',
            executable='rov_state_bridge',
            output='screen',
            arguments=[
                '--backend', backend,
                '--telemetry-source', telemetry_source,
                '--nav-view-source', nav_view_source,
                '--nav-state-source', nav_state_source,
                '--poll-hz', poll_hz,
            ],
        ),
        Node(
            package='rov_state_bridge',
            executable='rov_health_monitor',
            output='screen',
        ),
    ])
