from __future__ import annotations

"""Publisher backends for the stage1 bridge.

Backends are intentionally separate from SHM reading and field mapping so the
bridge can be validated without a ROS2 runtime. The default local validation
path uses `StdoutPublisherBackend` or `RecordingPublisherBackend`.
"""

from dataclasses import dataclass, fields, is_dataclass
import json
from typing import Any, Optional

from .models import to_jsonable


@dataclass(slots=True)
class PublishedRecord:
    topic: str
    payload: Any


class PublisherBackend:
    def publish(self, topic: str, payload: Any) -> None:  # pragma: no cover - interface
        raise NotImplementedError

    def close(self) -> None:
        return None


class RecordingPublisherBackend(PublisherBackend):
    """In-memory backend used by unit tests and dry-run validation."""

    def __init__(self) -> None:
        self.records: list[PublishedRecord] = []

    def publish(self, topic: str, payload: Any) -> None:
        self.records.append(PublishedRecord(topic=topic, payload=payload))


class StdoutPublisherBackend(PublisherBackend):
    """JSON-line backend for local validation without ROS2 installed."""

    def publish(self, topic: str, payload: Any) -> None:
        print(json.dumps({"topic": topic, "payload": to_jsonable(payload)}, sort_keys=True))


class Ros2PublisherBackend(PublisherBackend):
    """Optional runtime backend that publishes generated `rov_msgs` messages.

    Failure semantics:
    - Import failure means the local machine is not ready for ROS2 validation.
    - Publish failure only affects this outer-plane bridge process.
    - No exception here can change the core SHM producers because there is no
      write path back into the runtime chain.
    """

    def __init__(self, node_name: str = "rov_state_bridge", qos_depth: int = 5) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from rov_msgs import msg as rov_msgs_msg
        except ModuleNotFoundError as exc:  # pragma: no cover - depends on external ROS2 env
            raise RuntimeError(
                "ROS2 backend requires rclpy and generated rov_msgs Python modules"
            ) from exc

        self._rclpy = rclpy
        self._rov_msgs_msg = rov_msgs_msg
        self._initialised_here = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._initialised_here = True
        self._node = Node(node_name)
        qos = QoSProfile(depth=qos_depth)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        qos.durability = DurabilityPolicy.VOLATILE
        self._qos = qos
        self._publishers: dict[str, Any] = {}
        self._model_to_ros: dict[type[Any], type[Any]] = {}

    def publish(self, topic: str, payload: Any) -> None:  # pragma: no cover - depends on external ROS2 env
        msg_cls = self._ros_cls_for_model(type(payload))
        pub = self._publishers.get(topic)
        if pub is None:
            pub = self._node.create_publisher(msg_cls, topic, self._qos)
            self._publishers[topic] = pub
        pub.publish(self._to_ros_message(payload))
        self._rclpy.spin_once(self._node, timeout_sec=0.0)

    def close(self) -> None:  # pragma: no cover - depends on external ROS2 env
        self._node.destroy_node()
        if self._initialised_here:
            self._rclpy.shutdown()

    def _ros_cls_for_model(self, model_cls: type[Any]) -> type[Any]:
        cached = self._model_to_ros.get(model_cls)
        if cached is not None:
            return cached
        msg_cls = getattr(self._rov_msgs_msg, model_cls.__name__)
        self._model_to_ros[model_cls] = msg_cls
        return msg_cls

    def _to_ros_message(self, payload: Any) -> Any:
        msg_cls = self._ros_cls_for_model(type(payload))
        msg = msg_cls()
        for field in fields(payload):
            setattr(msg, field.name, self._convert_value(getattr(payload, field.name)))
        return msg

    def _convert_value(self, value: Any) -> Any:
        if is_dataclass(value):
            return self._to_ros_message(value)
        if isinstance(value, list):
            return [self._convert_value(item) for item in value]
        return value
