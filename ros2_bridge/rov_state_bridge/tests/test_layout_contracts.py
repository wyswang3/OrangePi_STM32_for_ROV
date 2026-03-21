from __future__ import annotations

import ctypes
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from rov_state_bridge import layouts


def _compile_and_run(workspace_root: Path, source_code: str) -> dict[str, int]:
    with tempfile.TemporaryDirectory() as tmpdir:
        src = Path(tmpdir) / "layout_probe.cpp"
        exe = Path(tmpdir) / "layout_probe"
        src.write_text(source_code)
        subprocess.run(
            ["c++", "-std=c++17", "-I", str(workspace_root), str(src), "-o", str(exe)],
            check=True,
        )
        output = subprocess.check_output([str(exe)], text=True)
    return {key: int(value) for key, value in (line.strip().split("=") for line in output.strip().splitlines())}


class LayoutContractTest(unittest.TestCase):
    @unittest.skipUnless(shutil.which("c++"), "c++ compiler required")
    def test_python_layout_sizes_match_cpp_contracts(self) -> None:
        workspace_root = Path("/home/wys/orangepi/UnderwaterRobotSystem")
        telemetry_and_nav = _compile_and_run(
            workspace_root,
            r"""#include <iostream>
#include "shared/msg/telemetry_frame_v2.hpp"
#include "shared/msg/nav_state.hpp"
#include "shared/shm/telemetry_frame_v2_shm.hpp"
#include "shared/shm/nav_state_shm.hpp"
int main() {
  std::cout << "TelemetryFrameV2=" << sizeof(shared::msg::TelemetryFrameV2) << "\n";
  std::cout << "TelemetryFrameV2ShmLayout=" << sizeof(shared::shm::TelemetryFrameV2ShmLayout) << "\n";
  std::cout << "NavState=" << sizeof(shared::msg::NavState) << "\n";
  std::cout << "NavStateShmLayout=" << sizeof(shared::shm::ShmLayout) << "\n";
  return 0;
}
""",
        )
        nav_view = _compile_and_run(
            workspace_root,
            r"""#include <iostream>
#include "shared/msg/nav_state_view.hpp"
#include "shared/shm/nav_state_view_shm.hpp"
int main() {
  std::cout << "NavStateView=" << sizeof(shared::msg::NavStateView) << "\n";
  std::cout << "NavStateViewShmLayout=" << sizeof(shared::shm::ShmLayout) << "\n";
  return 0;
}
""",
        )

        self.assertEqual(telemetry_and_nav["TelemetryFrameV2"], ctypes.sizeof(layouts.TelemetryFrameV2))
        self.assertEqual(telemetry_and_nav["TelemetryFrameV2ShmLayout"], ctypes.sizeof(layouts.TelemetryFrameV2ShmLayout))
        self.assertEqual(telemetry_and_nav["NavState"], ctypes.sizeof(layouts.NavState))
        self.assertEqual(telemetry_and_nav["NavStateShmLayout"], ctypes.sizeof(layouts.NavStateShmLayout))
        self.assertEqual(nav_view["NavStateView"], ctypes.sizeof(layouts.NavStateView))
        self.assertEqual(nav_view["NavStateViewShmLayout"], ctypes.sizeof(layouts.NavStateViewShmLayout))


if __name__ == "__main__":
    unittest.main()
