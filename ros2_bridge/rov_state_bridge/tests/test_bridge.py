from __future__ import annotations

import unittest

from rov_state_bridge import layouts
from rov_state_bridge.bridge import BridgeConfig, ReadOnlyStateBridge
from rov_state_bridge.publisher_backend import PublisherBackend, RecordingPublisherBackend
from rov_state_bridge.shm_reader import SnapshotEnvelope


class _StaticReader:
    def __init__(self, envelope):
        self.envelope = envelope
        self.closed = False

    def poll(self):
        return self.envelope

    def close(self):
        self.closed = True


class _FailOnceBackend(PublisherBackend):
    def __init__(self, fail_topic: str):
        self.fail_topic = fail_topic
        self.published: list[str] = []

    def publish(self, topic: str, payload):
        if topic == self.fail_topic:
            raise RuntimeError('planned failure')
        self.published.append(topic)


class BridgeTest(unittest.TestCase):
    def test_bridge_publishes_all_read_only_topics(self) -> None:
        telemetry = layouts.TelemetryFrameV2()
        telemetry.seq = 1
        telemetry.stamp_ns = 2
        telemetry.system.nav_age_ms = 3

        nav_view = layouts.NavStateView()
        nav_view.age_ms = 4
        nav_view.status_flags = 5

        nav_state = layouts.NavState()
        nav_state.t_ns = 6
        nav_state.age_ms = 7

        backend = RecordingPublisherBackend()
        bridge = ReadOnlyStateBridge(
            backend=backend,
            cfg=BridgeConfig(),
            telemetry_reader=_StaticReader(SnapshotEnvelope(telemetry, 10, 11, 2)),
            nav_view_reader=_StaticReader(SnapshotEnvelope(nav_view, 12, 13, 2)),
            nav_state_reader=_StaticReader(SnapshotEnvelope(nav_state, 14, 15, 2)),
        )
        try:
            published = bridge.poll_once()
        finally:
            bridge.close()

        topics = [record.topic for record in backend.records]
        self.assertEqual(
            topics,
            ['/rov/telemetry', '/rov/health', '/rov/nav_view', '/rov/nav_state_raw'],
        )
        self.assertTrue(published['/rov/telemetry'])
        self.assertTrue(published['/rov/health'])
        self.assertTrue(published['/rov/nav_view'])
        self.assertTrue(published['/rov/nav_state_raw'])

    def test_publish_failure_does_not_block_other_topics(self) -> None:
        telemetry = layouts.TelemetryFrameV2()
        nav_view = layouts.NavStateView()
        nav_state = layouts.NavState()
        backend = _FailOnceBackend('/rov/telemetry')
        bridge = ReadOnlyStateBridge(
            backend=backend,
            cfg=BridgeConfig(),
            telemetry_reader=_StaticReader(SnapshotEnvelope(telemetry, 1, 2, 2)),
            nav_view_reader=_StaticReader(SnapshotEnvelope(nav_view, 1, 2, 2)),
            nav_state_reader=_StaticReader(SnapshotEnvelope(nav_state, 1, 2, 2)),
        )
        try:
            published = bridge.poll_once()
        finally:
            bridge.close()

        self.assertFalse(published['/rov/telemetry'])
        self.assertIn('/rov/health', backend.published)
        self.assertIn('/rov/nav_view', backend.published)
        self.assertIn('/rov/nav_state_raw', backend.published)
        self.assertTrue(any('planned failure' in err for err in bridge.last_errors))


if __name__ == '__main__':
    unittest.main()
