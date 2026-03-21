from __future__ import annotations

import unittest

from rov_state_bridge import layouts, mapping


class MappingTest(unittest.TestCase):
    def test_telemetry_mapping_preserves_authority_fields(self) -> None:
        frame = layouts.TelemetryFrameV2()
        frame.version = layouts.TELEMETRY_FRAME_V2_WIRE_VERSION
        frame.payload_size = 1234
        frame.seq = 77
        frame.stamp_ns = 123456789
        frame.valid = 1
        frame.source = 2
        frame.control.active_mode = 3
        frame.control.armed = 1
        frame.control.controller_name = b"hold"
        frame.control.desired_controller = b"pid_depth"
        frame.system.nav_state = 3
        frame.system.nav_valid = 1
        frame.system.nav_stale = 0
        frame.system.nav_degraded = 1
        frame.system.nav_fault_code = 15
        frame.system.nav_status_flags = 0x34
        frame.system.nav_age_ms = 210
        frame.system.heartbeat_age_ms = 45
        frame.last_command_result.status = 5
        frame.last_command_result.fault_code = 13
        frame.last_event.event_code = 14
        frame.events[0].seq = 99
        frame.events[0].fault_code = 7
        frame.position[:] = [1.5, 2.5, 3.5]

        msg = mapping.map_telemetry_frame(frame)

        self.assertEqual(msg.seq, 77)
        self.assertEqual(msg.stamp_ns, 123456789)
        self.assertEqual(msg.control.controller_name, "hold")
        self.assertEqual(msg.control.desired_controller, "pid_depth")
        self.assertEqual(msg.system.nav_fault_code, 15)
        self.assertEqual(msg.system.nav_status_flags, 0x34)
        self.assertEqual(msg.system.nav_age_ms, 210)
        self.assertEqual(msg.last_command_result.status, 5)
        self.assertEqual(msg.last_command_result.fault_code, 13)
        self.assertEqual(msg.last_event.event_code, 14)
        self.assertEqual(msg.events[0].seq, 99)
        self.assertEqual(msg.events[0].fault_code, 7)
        self.assertEqual(msg.position, [1.5, 2.5, 3.5])

    def test_nav_view_mapping_preserves_age_fault_and_flags(self) -> None:
        view = layouts.NavStateView()
        view.version = layouts.NAV_STATE_VIEW_WIRE_VERSION
        view.flags = 0x21
        view.stamp_ns = 111
        view.mono_ns = 222
        view.age_ms = 333
        view.valid = 1
        view.stale = 1
        view.degraded = 1
        view.nav_state = 4
        view.health = 2
        view.fault_code = 8
        view.sensor_mask = 0x7
        view.status_flags = 0xAA
        view.pos[:] = [0.1, 0.2, 0.3]

        msg = mapping.map_nav_state_view(view)

        self.assertEqual(msg.flags, 0x21)
        self.assertEqual(msg.stamp_ns, 111)
        self.assertEqual(msg.mono_ns, 222)
        self.assertEqual(msg.age_ms, 333)
        self.assertEqual(msg.valid, 1)
        self.assertEqual(msg.stale, 1)
        self.assertEqual(msg.degraded, 1)
        self.assertEqual(msg.fault_code, 8)
        self.assertEqual(msg.sensor_mask, 0x7)
        self.assertEqual(msg.status_flags, 0xAA)
        self.assertEqual(msg.pos, [0.1, 0.2, 0.3])

    def test_health_summary_is_derived_without_reinterpreting_time(self) -> None:
        frame = layouts.TelemetryFrameV2()
        frame.seq = 5
        frame.stamp_ns = 8001
        frame.system.session_state = 3
        frame.system.health_state = 2
        frame.system.fault_state = 1
        frame.system.last_fault_code = 9
        frame.system.nav_state = 4
        frame.system.nav_valid = 0
        frame.system.nav_stale = 1
        frame.system.nav_degraded = 0
        frame.system.nav_fault_code = 14
        frame.system.nav_status_flags = 0x55
        frame.system.nav_age_ms = 901
        frame.system.heartbeat_age_ms = 71

        telemetry = mapping.map_telemetry_frame(frame)
        health = mapping.build_health_summary(telemetry)

        self.assertEqual(health.stamp_ns, 8001)
        self.assertEqual(health.telemetry_seq, 5)
        self.assertEqual(health.last_fault_code, 9)
        self.assertEqual(health.nav_fault_code, 14)
        self.assertEqual(health.nav_status_flags, 0x55)
        self.assertEqual(health.nav_age_ms, 901)
        self.assertEqual(health.heartbeat_age_ms, 71)


if __name__ == "__main__":
    unittest.main()
