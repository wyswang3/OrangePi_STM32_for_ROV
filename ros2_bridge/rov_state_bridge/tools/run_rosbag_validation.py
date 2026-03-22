from __future__ import annotations

import ctypes
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = ROOT.parent
SRC = ROOT / 'src'
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rov_state_bridge import layouts  # noqa: E402

BRIDGE_EXEC = WORKSPACE_ROOT / 'install' / 'rov_state_bridge' / 'lib' / 'rov_state_bridge' / 'rov_state_bridge'
HEALTH_EXEC = WORKSPACE_ROOT / 'install' / 'rov_state_bridge' / 'lib' / 'rov_state_bridge' / 'rov_health_monitor'
TOPICS = ['/rov/telemetry', '/rov/health', '/rov/nav_view', '/rov/nav_state_raw', '/rov/health_monitor']


def _write_layout(path: Path, layout: ctypes.Structure) -> None:
    blob = ctypes.string_at(ctypes.addressof(layout), ctypes.sizeof(layout.__class__))
    path.write_bytes(blob)
    os.chmod(path, 0o444)


def _build_sample_sources(tmpdir: Path) -> tuple[str, str, str]:
    telemetry = layouts.TelemetryFrameV2ShmLayout()
    telemetry.hdr.seqlock = 2
    telemetry.hdr.mono_ns = 111
    telemetry.hdr.wall_ns = 222
    telemetry.hdr.magic = layouts.TELEMETRY_MAGIC
    telemetry.hdr.layout_ver = layouts.TELEMETRY_LAYOUT_VERSION
    telemetry.hdr.payload_ver = layouts.TELEMETRY_FRAME_V2_WIRE_VERSION
    telemetry.hdr.payload_size = ctypes.sizeof(layouts.TelemetryFrameV2)
    telemetry.hdr.payload_align = ctypes.alignment(layouts.TelemetryFrameV2)
    telemetry.payload.version = layouts.TELEMETRY_FRAME_V2_WIRE_VERSION
    telemetry.payload.payload_size = ctypes.sizeof(layouts.TelemetryFrameV2)
    telemetry.payload.seq = 42
    telemetry.payload.stamp_ns = 555666777
    telemetry.payload.control.active_mode = 2
    telemetry.payload.control.armed = 1
    telemetry.payload.system.session_state = 3
    telemetry.payload.system.nav_valid = 1
    telemetry.payload.system.nav_stale = 0
    telemetry.payload.system.nav_degraded = 0
    telemetry.payload.system.nav_status_flags = (1 << 6) | (1 << 7) | (1 << 10)
    telemetry.payload.system.nav_age_ms = 5
    telemetry_path = tmpdir / 'telemetry.bin'
    _write_layout(telemetry_path, telemetry)

    nav_view = layouts.NavStateViewShmLayout()
    nav_view.hdr.seq = 2
    nav_view.hdr.magic = layouts.NAV_VIEW_MAGIC
    nav_view.hdr.layout_ver = layouts.NAV_VIEW_LAYOUT_VERSION
    nav_view.hdr.payload_ver = layouts.NAV_STATE_VIEW_WIRE_VERSION
    nav_view.hdr.payload_size = ctypes.sizeof(layouts.NavStateView)
    nav_view.hdr.payload_align = ctypes.alignment(layouts.NavStateView)
    nav_view.payload.version = layouts.NAV_STATE_VIEW_WIRE_VERSION
    nav_view.payload.stamp_ns = 888
    nav_view.payload.mono_ns = 999
    nav_view.payload.age_ms = 6
    nav_view.payload.valid = 1
    nav_view.payload.stale = 0
    nav_view.payload.degraded = 0
    nav_view.payload.nav_state = 3
    nav_view.payload.health = 1
    nav_view.payload.status_flags = (1 << 6) | (1 << 7) | (1 << 10)
    nav_view_path = tmpdir / 'nav_view.bin'
    _write_layout(nav_view_path, nav_view)

    nav_state = layouts.NavStateShmLayout()
    nav_state.hdr.seq = 2
    nav_state.hdr.magic = layouts.NAV_STATE_MAGIC
    nav_state.hdr.layout_ver = layouts.NAV_STATE_LAYOUT_VERSION
    nav_state.hdr.payload_ver = layouts.NAV_STATE_PAYLOAD_VERSION
    nav_state.hdr.payload_size = ctypes.sizeof(layouts.NavState)
    nav_state.hdr.payload_align = ctypes.alignment(layouts.NavState)
    nav_state.payload.t_ns = 777
    nav_state.payload.age_ms = 7
    nav_state.payload.valid = 1
    nav_state.payload.stale = 0
    nav_state.payload.degraded = 0
    nav_state.payload.nav_state = 3
    nav_state.payload.health = 1
    nav_state.payload.status_flags = (1 << 6) | (1 << 7) | (1 << 10)
    nav_state_path = tmpdir / 'nav_state.bin'
    _write_layout(nav_state_path, nav_state)

    return str(telemetry_path), str(nav_view_path), str(nav_state_path)


class _Collector:
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rov_msgs.msg import HealthMonitorStatus, HealthSummary, NavState, NavStateView, TelemetryFrameV2

        self._rclpy = rclpy
        self.node = Node('rov_rosbag_validation_collector')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        qos.durability = DurabilityPolicy.VOLATILE
        self.counts = {topic: 0 for topic in TOPICS}
        self.node.create_subscription(TelemetryFrameV2, '/rov/telemetry', lambda msg: self._mark('/rov/telemetry'), qos)
        self.node.create_subscription(HealthSummary, '/rov/health', lambda msg: self._mark('/rov/health'), qos)
        self.node.create_subscription(NavStateView, '/rov/nav_view', lambda msg: self._mark('/rov/nav_view'), qos)
        self.node.create_subscription(NavState, '/rov/nav_state_raw', lambda msg: self._mark('/rov/nav_state_raw'), qos)
        self.node.create_subscription(HealthMonitorStatus, '/rov/health_monitor', lambda msg: self._mark('/rov/health_monitor'), qos)

    def _mark(self, topic: str) -> None:
        self.counts[topic] += 1

    def spin_once(self, timeout_sec: float = 0.1) -> None:
        self._rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def close(self) -> None:
        self.node.destroy_node()


def _start_process(args: list[str], *, cwd: str | None = None, quiet: bool = False) -> subprocess.Popen[str]:
    stdout = subprocess.DEVNULL if quiet else subprocess.PIPE
    return subprocess.Popen(args, cwd=cwd, stdout=stdout, stderr=subprocess.STDOUT, text=True)


def _stop_process(proc: subprocess.Popen[str]) -> str:
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
    output = ''
    if proc.stdout is not None:
        output = proc.stdout.read()
    return output


def main() -> int:
    import rclpy

    initialised_here = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialised_here = True

    with tempfile.TemporaryDirectory() as tmpdir_str:
        tmpdir = Path(tmpdir_str)
        bag_dir = tmpdir / 'rosbag_validation'
        telemetry_source, nav_view_source, nav_state_source = _build_sample_sources(tmpdir)
        bridge_proc = _start_process([
            str(BRIDGE_EXEC),
            '--backend', 'ros2',
            '--telemetry-source', telemetry_source,
            '--nav-view-source', nav_view_source,
            '--nav-state-source', nav_state_source,
            '--poll-hz', '20',
        ], quiet=True)
        health_proc = _start_process([str(HEALTH_EXEC)], quiet=True)
        collector = _Collector()
        record_proc = None
        play_proc = None
        try:
            time.sleep(2.0)
            record_proc = _start_process(['ros2', 'bag', 'record', '-o', str(bag_dir), *TOPICS], cwd=str(tmpdir), quiet=True)
            time.sleep(3.0)
            _stop_process(record_proc)

            info = subprocess.check_output(['ros2', 'bag', 'info', str(bag_dir)], text=True)
            for topic in TOPICS:
                if topic not in info:
                    raise RuntimeError(f'missing topic in rosbag info: {topic}')

            play_proc = _start_process(['ros2', 'bag', 'play', str(bag_dir)], quiet=True)
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                collector.spin_once(timeout_sec=0.1)
                if all(count > 0 for count in collector.counts.values()):
                    break
                time.sleep(0.05)
            if not all(count > 0 for count in collector.counts.values()):
                raise RuntimeError(f'rosbag replay missing topics: {collector.counts}')

            print(json.dumps({
                'bag_dir': str(bag_dir),
                'bag_info_excerpt': [line for line in info.splitlines() if '/rov/' in line or 'Files:' in line or 'Duration:' in line],
                'replay_counts': collector.counts,
            }, sort_keys=True))
            return 0
        finally:
            if play_proc is not None:
                _stop_process(play_proc)
            _stop_process(bridge_proc)
            _stop_process(health_proc)
            collector.close()
            if initialised_here and rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
