from __future__ import annotations

"""Read-only SHM readers for the stage1 bridge.

Thread boundary:
- The reader is intended for the bridge process/thread only.
- It opens existing SHM blocks read-only and never creates or mutates them.

Failure semantics:
- Missing SHM in lazy-init mode is treated as "not ready yet", not as a hard error.
- ABI/layout mismatch is reported locally and the reader returns no snapshot.
- Reader failure must never affect the core publisher process because this code only
  opens `/dev/shm` objects read-only from a separate process.
"""

from dataclasses import dataclass
import ctypes
import mmap
import os
from typing import Any, Generic, Optional, TypeVar

from .constants import DEFAULT_MAX_SEQLOCK_RETRIES


PayloadT = TypeVar("PayloadT")


@dataclass(slots=True)
class SnapshotEnvelope(Generic[PayloadT]):
    payload: PayloadT
    pub_mono_ns: int
    pub_wall_ns: int
    stable_seq: int


@dataclass(slots=True)
class ReaderConfig(Generic[PayloadT]):
    source: str
    layout_cls: type[ctypes.Structure]
    header_cls: type[ctypes.Structure]
    payload_cls: type[PayloadT]
    seq_attr: str
    magic: int
    layout_ver: int
    payload_ver: int
    payload_size: int
    payload_align: int
    lazy_init: bool = True
    max_retries: int = DEFAULT_MAX_SEQLOCK_RETRIES


class ReadOnlyShmReader(Generic[PayloadT]):
    """Read snapshots from an existing SHM object or file-backed mmap.

    The bridge uses this class to mirror authority state out of the core chain.
    It must remain read-only: open mode is `O_RDONLY`, mapping is `ACCESS_READ`,
    and there is no write path in this class.
    """

    def __init__(self, cfg: ReaderConfig[PayloadT]) -> None:
        self.cfg = cfg
        self._path = resolve_source_path(cfg.source)
        self._fd: Optional[int] = None
        self._map: Optional[mmap.mmap] = None
        self._map_size: int = 0
        self._error_flag = False
        self.last_error: str = ""

    @property
    def ok(self) -> bool:
        return not self._error_flag

    def close(self) -> None:
        if self._map is not None:
            self._map.close()
            self._map = None
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None
        self._map_size = 0
        self._error_flag = False
        self.last_error = ""

    def poll(self) -> Optional[SnapshotEnvelope[PayloadT]]:
        """Read one stable snapshot using the producer's existing seqlock discipline.

        Returns `None` when the source is missing, unstable, or ABI-incompatible.
        The caller must treat that as bridge-local unavailability only; this
        reader never retries by writing to or recreating the authority SHM.
        """
        if self._map is None and not self._try_open():
            return None
        assert self._map is not None

        layout_size = ctypes.sizeof(self.cfg.layout_cls)
        for _ in range(self.cfg.max_retries):
            raw1 = self._map[:layout_size]
            layout1 = self.cfg.layout_cls.from_buffer_copy(raw1)
            hdr1 = layout1.hdr
            s1 = int(getattr(hdr1, self.cfg.seq_attr))
            if s1 & 1:
                continue
            if not self._check_header(hdr1):
                return None
            raw2 = self._map[:layout_size]
            layout2 = self.cfg.layout_cls.from_buffer_copy(raw2)
            hdr2 = layout2.hdr
            s2 = int(getattr(hdr2, self.cfg.seq_attr))
            if s1 == s2 and (s2 & 1) == 0:
                return SnapshotEnvelope(
                    payload=layout1.payload,
                    pub_mono_ns=int(hdr1.mono_ns),
                    pub_wall_ns=int(hdr1.wall_ns),
                    stable_seq=s2,
                )
        self.last_error = "seqlock snapshot unstable"
        return None

    def _try_open(self) -> bool:
        if self._map is not None:
            return True
        try:
            fd = os.open(self._path, os.O_RDONLY)
        except FileNotFoundError:
            if self.cfg.lazy_init:
                self.last_error = f"source not ready: {self._path}"
                return False
            self._error_flag = True
            self.last_error = f"source missing: {self._path}"
            return False
        except OSError as exc:
            self._error_flag = True
            self.last_error = f"open failed: {exc}"
            return False

        try:
            size = os.fstat(fd).st_size
            min_size = ctypes.sizeof(self.cfg.layout_cls)
            if size < min_size:
                raise RuntimeError(f"mapping too small: {size} < {min_size}")
            mapping = mmap.mmap(fd, size, access=mmap.ACCESS_READ)
        except Exception as exc:  # noqa: BLE001
            os.close(fd)
            if self.cfg.lazy_init and isinstance(exc, RuntimeError) and "mapping too small" not in str(exc):
                self.last_error = str(exc)
                return False
            self._error_flag = True
            self.last_error = f"mmap failed: {exc}"
            return False

        self._fd = fd
        self._map = mapping
        self._map_size = size
        self.last_error = ""
        return True

    def _check_header(self, hdr: Any) -> bool:
        if int(hdr.magic) != self.cfg.magic:
            self._error_flag = True
            self.last_error = f"bad magic: {int(hdr.magic)}"
            return False
        if int(hdr.layout_ver) != self.cfg.layout_ver:
            self._error_flag = True
            self.last_error = f"bad layout_ver: {int(hdr.layout_ver)}"
            return False
        if int(hdr.payload_ver) != self.cfg.payload_ver:
            self._error_flag = True
            self.last_error = f"bad payload_ver: {int(hdr.payload_ver)}"
            return False
        if int(hdr.payload_size) != self.cfg.payload_size:
            self._error_flag = True
            self.last_error = f"bad payload_size: {int(hdr.payload_size)}"
            return False
        if int(hdr.payload_align) != self.cfg.payload_align:
            self._error_flag = True
            self.last_error = f"bad payload_align: {int(hdr.payload_align)}"
            return False
        return True


def resolve_source_path(source: str) -> str:
    """Resolve a SHM name into `/dev/shm/...` while keeping test file paths usable."""
    if source.startswith("/dev/shm/"):
        return source
    if source.startswith("/") and source.count("/") == 1:
        return os.path.join("/dev/shm", source.lstrip("/"))
    return source
