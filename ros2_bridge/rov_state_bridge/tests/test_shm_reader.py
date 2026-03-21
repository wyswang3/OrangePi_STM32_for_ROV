from __future__ import annotations

import ctypes
import os
from pathlib import Path
import tempfile
import unittest

from rov_state_bridge import layouts
from rov_state_bridge.shm_reader import ReadOnlyShmReader, ReaderConfig, resolve_source_path


def _write_layout(path: Path, layout: ctypes.Structure) -> None:
    blob = ctypes.string_at(ctypes.addressof(layout), ctypes.sizeof(layout.__class__))
    path.write_bytes(blob)


class ShmReaderTest(unittest.TestCase):
    def test_resolve_source_path_keeps_paths_and_maps_shm_names(self) -> None:
        self.assertEqual(resolve_source_path('/rovctrl_telemetry_v2'), '/dev/shm/rovctrl_telemetry_v2')
        self.assertEqual(resolve_source_path('/tmp/test.bin'), '/tmp/test.bin')

    def test_reader_reads_read_only_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / 'telemetry.bin'
            layout = layouts.TelemetryFrameV2ShmLayout()
            layout.hdr.seqlock = 2
            layout.hdr.mono_ns = 101
            layout.hdr.wall_ns = 202
            layout.hdr.magic = layouts.TELEMETRY_MAGIC
            layout.hdr.layout_ver = layouts.TELEMETRY_LAYOUT_VERSION
            layout.hdr.payload_ver = layouts.TELEMETRY_FRAME_V2_WIRE_VERSION
            layout.hdr.payload_size = ctypes.sizeof(layouts.TelemetryFrameV2)
            layout.hdr.payload_align = ctypes.alignment(layouts.TelemetryFrameV2)
            layout.payload.seq = 303
            layout.payload.stamp_ns = 404
            layout.payload.system.nav_fault_code = 505
            _write_layout(path, layout)
            os.chmod(path, 0o444)

            reader = ReadOnlyShmReader(
                ReaderConfig(
                    source=str(path),
                    layout_cls=layouts.TelemetryFrameV2ShmLayout,
                    header_cls=layouts.TelemetryFrameV2ShmHeader,
                    payload_cls=layouts.TelemetryFrameV2,
                    seq_attr='seqlock',
                    magic=layouts.TELEMETRY_MAGIC,
                    layout_ver=layouts.TELEMETRY_LAYOUT_VERSION,
                    payload_ver=layouts.TELEMETRY_FRAME_V2_WIRE_VERSION,
                    payload_size=ctypes.sizeof(layouts.TelemetryFrameV2),
                    payload_align=ctypes.alignment(layouts.TelemetryFrameV2),
                    lazy_init=False,
                )
            )
            try:
                snap = reader.poll()
                self.assertIsNotNone(snap)
                assert snap is not None
                self.assertEqual(snap.pub_mono_ns, 101)
                self.assertEqual(snap.pub_wall_ns, 202)
                self.assertEqual(snap.payload.seq, 303)
                self.assertEqual(snap.payload.system.nav_fault_code, 505)
            finally:
                reader.close()

    def test_reader_rejects_header_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / 'nav_view.bin'
            layout = layouts.NavStateViewShmLayout()
            layout.hdr.seq = 2
            layout.hdr.magic = 0
            layout.hdr.layout_ver = layouts.NAV_VIEW_LAYOUT_VERSION
            layout.hdr.payload_ver = layouts.NAV_STATE_VIEW_WIRE_VERSION
            layout.hdr.payload_size = ctypes.sizeof(layouts.NavStateView)
            layout.hdr.payload_align = ctypes.alignment(layouts.NavStateView)
            _write_layout(path, layout)

            reader = ReadOnlyShmReader(
                ReaderConfig(
                    source=str(path),
                    layout_cls=layouts.NavStateViewShmLayout,
                    header_cls=layouts.NavStateViewShmHeader,
                    payload_cls=layouts.NavStateView,
                    seq_attr='seq',
                    magic=layouts.NAV_VIEW_MAGIC,
                    layout_ver=layouts.NAV_VIEW_LAYOUT_VERSION,
                    payload_ver=layouts.NAV_STATE_VIEW_WIRE_VERSION,
                    payload_size=ctypes.sizeof(layouts.NavStateView),
                    payload_align=ctypes.alignment(layouts.NavStateView),
                    lazy_init=False,
                )
            )
            try:
                snap = reader.poll()
                self.assertIsNone(snap)
                self.assertIn('bad magic', reader.last_error)
            finally:
                reader.close()


if __name__ == '__main__':
    unittest.main()
