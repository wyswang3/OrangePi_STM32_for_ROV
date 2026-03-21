from __future__ import annotations

"""Run a deterministic stage1 bridge dry-run without ROS2 installed.

This tool creates temporary file-backed SHM layouts, feeds them through the
read-only bridge, and prints a compact JSON summary. It exists so stage1 can be
validated on a Linux host before a full ROS2 runtime is available.
"""

import ctypes
import json
import os
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from rov_state_bridge import layouts  # noqa: E402
from rov_state_bridge.bridge import BridgeConfig, ReadOnlyStateBridge  # noqa: E402
from rov_state_bridge.models import to_jsonable  # noqa: E402
from rov_state_bridge.publisher_backend import RecordingPublisherBackend  # noqa: E402


def _write_layout(path: Path, layout: ctypes.Structure) -> None:
    """Write one ctypes layout blob as a read-only file-backed mmap source."""
    blob = ctypes.string_at(ctypes.addressof(layout), ctypes.sizeof(layout.__class__))
    path.write_bytes(blob)
    os.chmod(path, 0o444)


def _build_sample_sources(tmpdir: Path) -> tuple[str, str, str]:
    """Create deterministic sample snapshots for telemetry/nav mirrors."""
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
    telemetry.payload.system.nav_valid = 1
    telemetry.payload.system.nav_stale = 0
    telemetry.payload.system.nav_degraded = 1
    telemetry.payload.system.nav_fault_code = 42
    telemetry.payload.system.nav_status_flags = 0x12
    telemetry.payload.system.nav_age_ms = 88
    telemetry.payload.last_command_result.status = 4
    telemetry.payload.last_command_result.fault_code = 6
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
    nav_view.payload.age_ms = 55
    nav_view.payload.valid = 1
    nav_view.payload.stale = 1
    nav_view.payload.degraded = 1
    nav_view.payload.fault_code = 9
    nav_view.payload.status_flags = 0x34
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
    nav_state.payload.age_ms = 66
    nav_state.payload.valid = 1
    nav_state.payload.stale = 0
    nav_state.payload.degraded = 1
    nav_state.payload.fault_code = 17
    nav_state.payload.status_flags = 0x56
    nav_state_path = tmpdir / 'nav_state.bin'
    _write_layout(nav_state_path, nav_state)

    return str(telemetry_path), str(nav_view_path), str(nav_state_path)


def main() -> int:
    with tempfile.TemporaryDirectory() as tmpdir_str:
        tmpdir = Path(tmpdir_str)
        telemetry_source, nav_view_source, nav_state_source = _build_sample_sources(tmpdir)
        backend = RecordingPublisherBackend()
        bridge = ReadOnlyStateBridge(
            backend=backend,
            cfg=BridgeConfig(
                telemetry_source=telemetry_source,
                nav_view_source=nav_view_source,
                nav_state_source=nav_state_source,
                publish_health=True,
                lazy_init=False,
            ),
        )
        try:
            published = bridge.poll_once()
            summary = {
                'published': published,
                'topics': [record.topic for record in backend.records],
                'payloads': {record.topic: to_jsonable(record.payload) for record in backend.records},
                'errors': list(bridge.last_errors),
            }
            print(json.dumps(summary, sort_keys=True))
            return 0
        finally:
            bridge.close()


if __name__ == '__main__':
    raise SystemExit(main())
