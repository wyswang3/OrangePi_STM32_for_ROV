"""CLI entry for the stage1 read-only ROS2 bridge.

作用：
- 解析 bridge 启动参数并选择输出 backend；
- 组装 BridgeConfig 后启动一次性轮询或持续桥接循环。

实现思路：
- CLI 层只做参数到配置的映射与 backend 选择；
- 真正的 SHM 读取、字段映射和发布逻辑全部交给 `bridge.py` 与 backend 模块。
"""

from __future__ import annotations

import argparse
import json
from typing import Optional

from .bridge import BridgeConfig, ReadOnlyStateBridge
from .publisher_backend import RecordingPublisherBackend, Ros2PublisherBackend, StdoutPublisherBackend


def build_backend(name: str):
    if name == "stdout":
        return StdoutPublisherBackend()
    if name == "recording":
        return RecordingPublisherBackend()
    if name == "ros2":
        return Ros2PublisherBackend()
    raise ValueError(f"Unknown backend: {name}")


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Stage1 read-only ROS2 bridge")
    parser.add_argument("--backend", choices=["stdout", "recording", "ros2"], default="stdout")
    parser.add_argument("--once", action="store_true", help="run one poll/publish iteration")
    parser.add_argument("--telemetry-source", default=None, help="SHM name or file path")
    parser.add_argument("--nav-view-source", default=None, help="SHM name or file path")
    parser.add_argument("--nav-state-source", default=None, help="SHM name or file path")
    parser.add_argument("--poll-hz", type=float, default=None, help="loop frequency for continuous mode")
    parser.add_argument("--disable-health", action="store_true", help="skip derived health summary topic")
    args = parser.parse_args(argv)

    cfg = BridgeConfig()
    if args.telemetry_source is not None:
        cfg.telemetry_source = args.telemetry_source
    if args.nav_view_source is not None:
        cfg.nav_view_source = args.nav_view_source
    if args.nav_state_source is not None:
        cfg.nav_state_source = args.nav_state_source
    if args.poll_hz is not None:
        cfg.poll_hz = args.poll_hz
    if args.disable_health:
        cfg.publish_health = False

    backend = build_backend(args.backend)
    bridge = ReadOnlyStateBridge(backend=backend, cfg=cfg)
    try:
        if args.once:
            published = bridge.poll_once()
            if args.backend == "recording":
                records = backend.records
                print(json.dumps({"published": published, "records": len(records)}, sort_keys=True))
            return 0
        bridge.run_forever()
        return 0
    finally:
        bridge.close()
