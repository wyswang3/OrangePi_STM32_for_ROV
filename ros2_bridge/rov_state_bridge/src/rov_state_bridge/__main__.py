"""Module entrypoint for `python -m rov_state_bridge`.

作用：
- 复用 CLI `main()` 作为包级可执行入口；
- 保持包安装后的 console script 与模块直跑语义一致。
"""

from .cli import main

if __name__ == "__main__":
    raise SystemExit(main())
