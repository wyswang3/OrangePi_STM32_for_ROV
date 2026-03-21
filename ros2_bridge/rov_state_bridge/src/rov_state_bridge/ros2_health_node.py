from __future__ import annotations

"""Optional ROS2 runtime wrapper for the advisory health monitor.

This node is strictly outer-plane: it subscribes to read-only mirror topics and
publishes a derived advisory summary. No callback here writes back into SHM,
ControlIntent, or any control/navigation authority path.
"""

from dataclasses import fields, is_dataclass
from typing import Any

from .health_monitor import build_health_monitor_status


class Ros2HealthMonitorNode:
    def __init__(self, node_name: str = 'rov_health_monitor', qos_depth: int = 5) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from rov_msgs.msg import HealthMonitorStatus as HealthMonitorStatusMsg
            from rov_msgs.msg import NavStateView, TelemetryFrameV2
        except ModuleNotFoundError as exc:  # pragma: no cover - depends on ROS2 runtime
            raise RuntimeError(
                'ROS2 health monitor requires rclpy and generated rov_msgs Python modules'
            ) from exc

        self._rclpy = rclpy
        self._initialised_here = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._initialised_here = True

        self._node = Node(node_name)
        qos = QoSProfile(depth=qos_depth)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        qos.durability = DurabilityPolicy.VOLATILE

        self._nav_view = None
        self._publisher = self._node.create_publisher(HealthMonitorStatusMsg, '/rov/health_monitor', qos)
        self._node.create_subscription(TelemetryFrameV2, '/rov/telemetry', self._on_telemetry, qos)
        self._node.create_subscription(NavStateView, '/rov/nav_view', self._on_nav_view, qos)
        self._ros_cls = HealthMonitorStatusMsg

    def spin_once(self, timeout_sec: float = 0.1) -> None:  # pragma: no cover - depends on ROS2 runtime
        self._rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    def run_forever(self) -> None:  # pragma: no cover - depends on ROS2 runtime
        while self._rclpy.ok():
            self.spin_once(timeout_sec=0.1)

    def close(self) -> None:  # pragma: no cover - depends on ROS2 runtime
        self._node.destroy_node()
        if self._initialised_here:
            self._rclpy.shutdown()

    def _on_nav_view(self, msg: Any) -> None:  # pragma: no cover - depends on ROS2 runtime
        self._nav_view = msg

    def _on_telemetry(self, msg: Any) -> None:  # pragma: no cover - depends on ROS2 runtime
        status = build_health_monitor_status(msg, self._nav_view)
        self._publisher.publish(self._to_ros_message(status))

    def _to_ros_message(self, payload: Any) -> Any:  # pragma: no cover - depends on ROS2 runtime
        msg = self._ros_cls()
        for field in fields(payload):
            setattr(msg, field.name, self._convert_value(getattr(payload, field.name)))
        return msg

    def _convert_value(self, value: Any) -> Any:  # pragma: no cover - depends on ROS2 runtime
        if is_dataclass(value):
            raise TypeError('Health monitor payload must be flat dataclass values')
        if isinstance(value, list):
            return list(value)
        return value


def main() -> int:  # pragma: no cover - depends on ROS2 runtime
    node = Ros2HealthMonitorNode()
    try:
        node.run_forever()
        return 0
    finally:
        node.close()


if __name__ == '__main__':  # pragma: no cover - depends on ROS2 runtime
    raise SystemExit(main())
