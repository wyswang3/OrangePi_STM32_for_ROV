from __future__ import annotations

import ctypes
from dataclasses import fields
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = ROOT.parent
SRC = ROOT / 'src'
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rov_state_bridge import layouts, models  # noqa: E402

BRIDGE_EXEC = WORKSPACE_ROOT / 'install' / 'rov_state_bridge' / 'lib' / 'rov_state_bridge' / 'rov_state_bridge'
HEALTH_EXEC = WORKSPACE_ROOT / 'install' / 'rov_state_bridge' / 'lib' / 'rov_state_bridge' / 'rov_health_monitor'
TOPICS = {
    '/rov/telemetry': 'TelemetryFrameV2',
    '/rov/health': 'HealthSummary',
    '/rov/nav_view': 'NavStateView',
    '/rov/nav_state_raw': 'NavState',
    '/rov/health_monitor': 'HealthMonitorStatus',
}


def _write_layout(path: Path, layout: ctypes.Structure) -> None:
    blob = ctypes.string_at(ctypes.addressof(layout), ctypes.sizeof(layout.__class__))
    path.write_bytes(blob)
    os.chmod(path, 0o444)


def _build_sample_sources(tmpdir: Path) -> tuple[str, str, str]:
    telemetry = layouts.TelemetryFrameV2ShmLayout()
    telemetry.hdr.seqlock = 2
    telemetry.hdr.mono_ns = 1001
    telemetry.hdr.wall_ns = 2002
    telemetry.hdr.magic = layouts.TELEMETRY_MAGIC
    telemetry.hdr.layout_ver = layouts.TELEMETRY_LAYOUT_VERSION
    telemetry.hdr.payload_ver = layouts.TELEMETRY_FRAME_V2_WIRE_VERSION
    telemetry.hdr.payload_size = ctypes.sizeof(layouts.TelemetryFrameV2)
    telemetry.hdr.payload_align = ctypes.alignment(layouts.TelemetryFrameV2)
    telemetry.payload.version = layouts.TELEMETRY_FRAME_V2_WIRE_VERSION
    telemetry.payload.payload_size = ctypes.sizeof(layouts.TelemetryFrameV2)
    telemetry.payload.seq = 7
    telemetry.payload.stamp_ns = 123456789
    telemetry.payload.control.active_mode = 2
    telemetry.payload.control.armed = 1
    telemetry.payload.control.estop_latched = 0
    telemetry.payload.control.failsafe_active = 0
    telemetry.payload.system.session_state = 3
    telemetry.payload.system.nav_valid = 1
    telemetry.payload.system.nav_stale = 0
    telemetry.payload.system.nav_degraded = 0
    telemetry.payload.system.nav_fault_code = 0
    telemetry.payload.system.nav_status_flags = (1 << 6) | (1 << 7) | (1 << 10)
    telemetry.payload.system.nav_age_ms = 88
    telemetry.payload.last_command_result.status = 3
    telemetry.payload.last_command_result.fault_code = 0
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
    nav_view.payload.stamp_ns = 333
    nav_view.payload.mono_ns = 444
    nav_view.payload.age_ms = 12
    nav_view.payload.valid = 1
    nav_view.payload.stale = 0
    nav_view.payload.degraded = 0
    nav_view.payload.nav_state = 3
    nav_view.payload.health = 1
    nav_view.payload.fault_code = 0
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
    nav_state.payload.t_ns = 999
    nav_state.payload.age_ms = 8
    nav_state.payload.valid = 1
    nav_state.payload.stale = 0
    nav_state.payload.degraded = 0
    nav_state.payload.nav_state = 3
    nav_state.payload.health = 1
    nav_state.payload.fault_code = 0
    nav_state.payload.status_flags = (1 << 6) | (1 << 7) | (1 << 10)
    nav_state_path = tmpdir / 'nav_state.bin'
    _write_layout(nav_state_path, nav_state)

    return str(telemetry_path), str(nav_view_path), str(nav_state_path)


def _validate_generated_messages() -> dict[str, int]:
    from rov_msgs import msg as rov_msgs_msg

    checks = [
        models.CommandResult,
        models.ControlIntentState,
        models.ControlState,
        models.EventRecord,
        models.HealthMonitorStatus,
        models.HealthSummary,
        models.MotorTestState,
        models.NavState,
        models.NavStateView,
        models.SystemState,
        models.TelemetryFrameV2,
    ]
    summary: dict[str, int] = {}
    for model_cls in checks:
        msg_cls = getattr(rov_msgs_msg, model_cls.__name__)
        expected = [field.name for field in fields(model_cls)]
        actual = list(msg_cls.get_fields_and_field_types().keys())
        if expected != actual:
            raise AssertionError(f'{model_cls.__name__} fields mismatch: expected {expected}, got {actual}')
        summary[model_cls.__name__] = len(expected)
    return summary


class _Collector:
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rov_msgs.msg import HealthMonitorStatus, HealthSummary, NavState, NavStateView, TelemetryFrameV2

        self._rclpy = rclpy
        self.node = Node('rov_graph_validation_collector')
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST
        qos.durability = DurabilityPolicy.VOLATILE
        self.counts = {topic: 0 for topic in TOPICS}
        self.samples: dict[str, dict[str, Any]] = {}
        self.node.create_subscription(TelemetryFrameV2, '/rov/telemetry', self._on_telemetry, qos)
        self.node.create_subscription(HealthSummary, '/rov/health', self._on_health, qos)
        self.node.create_subscription(NavStateView, '/rov/nav_view', self._on_nav_view, qos)
        self.node.create_subscription(NavState, '/rov/nav_state_raw', self._on_nav_state, qos)
        self.node.create_subscription(HealthMonitorStatus, '/rov/health_monitor', self._on_health_monitor, qos)

    def spin_once(self, timeout_sec: float = 0.1) -> None:
        self._rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def close(self) -> None:
        self.node.destroy_node()

    def _on_telemetry(self, msg: Any) -> None:
        self.counts['/rov/telemetry'] += 1
        self.samples['/rov/telemetry'] = {'seq': int(msg.seq), 'stamp_ns': int(msg.stamp_ns)}

    def _on_health(self, msg: Any) -> None:
        self.counts['/rov/health'] += 1
        self.samples['/rov/health'] = {'telemetry_seq': int(msg.telemetry_seq), 'nav_age_ms': int(msg.nav_age_ms)}

    def _on_nav_view(self, msg: Any) -> None:
        self.counts['/rov/nav_view'] += 1
        self.samples['/rov/nav_view'] = {'age_ms': int(msg.age_ms), 'status_flags': int(msg.status_flags)}

    def _on_nav_state(self, msg: Any) -> None:
        self.counts['/rov/nav_state_raw'] += 1
        self.samples['/rov/nav_state_raw'] = {'t_ns': int(msg.t_ns), 'status_flags': int(msg.status_flags)}

    def _on_health_monitor(self, msg: Any) -> None:
        self.counts['/rov/health_monitor'] += 1
        self.samples['/rov/health_monitor'] = {
            'severity': int(msg.severity),
            'summary': str(msg.summary),
            'recommended_action': str(msg.recommended_action),
        }


def _start_process(args: list[str]) -> subprocess.Popen[str]:
    return subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, text=True)


def _stop_process(proc: subprocess.Popen[str]) -> None:
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)


def main() -> int:
    field_summary = _validate_generated_messages()

    import rclpy

    initialised_here = False
    if not rclpy.ok():
        rclpy.init(args=None)
        initialised_here = True

    collector = _Collector()
    with tempfile.TemporaryDirectory() as tmpdir_str:
        tmpdir = Path(tmpdir_str)
        telemetry_source, nav_view_source, nav_state_source = _build_sample_sources(tmpdir)
        bridge_proc = _start_process([
            str(BRIDGE_EXEC),
            '--backend', 'ros2',
            '--telemetry-source', telemetry_source,
            '--nav-view-source', nav_view_source,
            '--nav-state-source', nav_state_source,
            '--poll-hz', '20',
        ])
        health_proc = _start_process([str(HEALTH_EXEC)])
        try:
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                collector.spin_once(timeout_sec=0.1)
                if all(count > 0 for count in collector.counts.values()):
                    break
                time.sleep(0.05)

            if not all(count > 0 for count in collector.counts.values()):
                raise RuntimeError(f'missing topics: {collector.counts}')

            health_monitor = collector.samples.get('/rov/health_monitor', {})
            if health_monitor.get('summary') != 'device_reconnecting':
                raise RuntimeError(f'unexpected health monitor summary: {health_monitor}')

            print(json.dumps({
                'generated_fields': field_summary,
                'topic_counts': collector.counts,
                'samples': collector.samples,
            }, sort_keys=True))
            return 0
        finally:
            _stop_process(bridge_proc)
            _stop_process(health_proc)
            collector.close()
            if initialised_here and rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
