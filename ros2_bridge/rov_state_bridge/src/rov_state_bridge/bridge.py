from __future__ import annotations

"""Stage1 read-only bridge.

Responsibilities:
- read existing authority snapshots from SHM
- map them into mirror models without changing semantics
- publish them to an outer-plane backend

Thread boundary:
- intended to run in its own process or thread
- must never be embedded into `ControlLoop` / `nav_daemon`

Failure semantics:
- a failed read or failed publish only affects this bridge iteration
- no error path here can block or backpressure the core producers because all
  inputs are read-only SHM snapshots
"""

from dataclasses import dataclass
import ctypes
import time
from typing import Any, Optional

from . import layouts, mapping
from .constants import (
    DEFAULT_POLL_HZ,
    HEALTH_TOPIC,
    NAV_STATE_SHM_NAME,
    NAV_STATE_TOPIC,
    NAV_VIEW_SHM_NAME,
    NAV_VIEW_TOPIC,
    TELEMETRY_SHM_NAME,
    TELEMETRY_TOPIC,
)
from .publisher_backend import PublisherBackend
from .shm_reader import ReaderConfig, ReadOnlyShmReader


@dataclass(slots=True)
class BridgeTopics:
    telemetry: str = TELEMETRY_TOPIC
    nav_view: str = NAV_VIEW_TOPIC
    nav_state_raw: str = NAV_STATE_TOPIC
    health: str = HEALTH_TOPIC


@dataclass(slots=True)
class BridgeConfig:
    telemetry_source: str = TELEMETRY_SHM_NAME
    nav_view_source: str = NAV_VIEW_SHM_NAME
    nav_state_source: str = NAV_STATE_SHM_NAME
    poll_hz: float = DEFAULT_POLL_HZ
    publish_health: bool = True
    lazy_init: bool = True


class ReadOnlyStateBridge:
    """Read authority state from SHM and mirror it into outer-plane topics."""

    def __init__(
        self,
        backend: PublisherBackend,
        cfg: BridgeConfig | None = None,
        topics: BridgeTopics | None = None,
        telemetry_reader: Optional[ReadOnlyShmReader[layouts.TelemetryFrameV2]] = None,
        nav_view_reader: Optional[ReadOnlyShmReader[layouts.NavStateView]] = None,
        nav_state_reader: Optional[ReadOnlyShmReader[layouts.NavState]] = None,
    ) -> None:
        self.backend = backend
        self.cfg = cfg or BridgeConfig()
        self.topics = topics or BridgeTopics()
        self.telemetry_reader = telemetry_reader or self._make_telemetry_reader()
        self.nav_view_reader = nav_view_reader or self._make_nav_view_reader()
        self.nav_state_reader = nav_state_reader or self._make_nav_state_reader()
        self.last_errors: list[str] = []

    def poll_once(self) -> dict[str, bool]:
        """Read and publish at most one snapshot per source.

        Returns a per-topic success map for the current iteration.
        """
        self.last_errors = []
        published: dict[str, bool] = {}

        telemetry_env = self._safe_poll(self.telemetry_reader, "telemetry")
        if telemetry_env is not None:
            telemetry_msg = mapping.map_telemetry_frame(telemetry_env.payload)
            self._safe_publish(self.topics.telemetry, telemetry_msg, published)
            if self.cfg.publish_health:
                health_msg = mapping.build_health_summary(telemetry_msg)
                self._safe_publish(self.topics.health, health_msg, published)

        nav_view_env = self._safe_poll(self.nav_view_reader, "nav_view")
        if nav_view_env is not None:
            nav_view_msg = mapping.map_nav_state_view(nav_view_env.payload)
            self._safe_publish(self.topics.nav_view, nav_view_msg, published)

        nav_state_env = self._safe_poll(self.nav_state_reader, "nav_state")
        if nav_state_env is not None:
            nav_state_msg = mapping.map_nav_state(nav_state_env.payload)
            self._safe_publish(self.topics.nav_state_raw, nav_state_msg, published)

        return published

    def run_forever(self) -> None:
        period_s = 1.0 / max(self.cfg.poll_hz, 0.1)
        while True:
            self.poll_once()
            time.sleep(period_s)

    def close(self) -> None:
        self.telemetry_reader.close()
        self.nav_view_reader.close()
        self.nav_state_reader.close()
        self.backend.close()

    def _safe_poll(self, reader: ReadOnlyShmReader[Any], label: str):
        try:
            return reader.poll()
        except Exception as exc:  # noqa: BLE001
            self.last_errors.append(f"{label} poll failed: {exc}")
            return None

    def _safe_publish(self, topic: str, payload: Any, published: dict[str, bool]) -> None:
        try:
            self.backend.publish(topic, payload)
            published[topic] = True
        except Exception as exc:  # noqa: BLE001
            self.last_errors.append(f"publish failed for {topic}: {exc}")
            published[topic] = False

    def _make_telemetry_reader(self) -> ReadOnlyShmReader[layouts.TelemetryFrameV2]:
        return ReadOnlyShmReader(
            ReaderConfig(
                source=self.cfg.telemetry_source,
                layout_cls=layouts.TelemetryFrameV2ShmLayout,
                header_cls=layouts.TelemetryFrameV2ShmHeader,
                payload_cls=layouts.TelemetryFrameV2,
                seq_attr="seqlock",
                magic=layouts.TELEMETRY_MAGIC,
                layout_ver=layouts.TELEMETRY_LAYOUT_VERSION,
                payload_ver=layouts.TELEMETRY_FRAME_V2_WIRE_VERSION,
                payload_size=ctypes.sizeof(layouts.TelemetryFrameV2),
                payload_align=ctypes.alignment(layouts.TelemetryFrameV2),
                lazy_init=self.cfg.lazy_init,
            )
        )

    def _make_nav_view_reader(self) -> ReadOnlyShmReader[layouts.NavStateView]:
        return ReadOnlyShmReader(
            ReaderConfig(
                source=self.cfg.nav_view_source,
                layout_cls=layouts.NavStateViewShmLayout,
                header_cls=layouts.NavStateViewShmHeader,
                payload_cls=layouts.NavStateView,
                seq_attr="seq",
                magic=layouts.NAV_VIEW_MAGIC,
                layout_ver=layouts.NAV_VIEW_LAYOUT_VERSION,
                payload_ver=layouts.NAV_STATE_VIEW_WIRE_VERSION,
                payload_size=ctypes.sizeof(layouts.NavStateView),
                payload_align=ctypes.alignment(layouts.NavStateView),
                lazy_init=self.cfg.lazy_init,
            )
        )

    def _make_nav_state_reader(self) -> ReadOnlyShmReader[layouts.NavState]:
        return ReadOnlyShmReader(
            ReaderConfig(
                source=self.cfg.nav_state_source,
                layout_cls=layouts.NavStateShmLayout,
                header_cls=layouts.NavStateShmHeader,
                payload_cls=layouts.NavState,
                seq_attr="seq",
                magic=layouts.NAV_STATE_MAGIC,
                layout_ver=layouts.NAV_STATE_LAYOUT_VERSION,
                payload_ver=layouts.NAV_STATE_PAYLOAD_VERSION,
                payload_size=ctypes.sizeof(layouts.NavState),
                payload_align=ctypes.alignment(layouts.NavState),
                lazy_init=self.cfg.lazy_init,
            )
        )
