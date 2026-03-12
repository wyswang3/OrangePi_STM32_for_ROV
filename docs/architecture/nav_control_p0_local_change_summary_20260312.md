# 本地改动说明（2026-03-12）

## 仓库

- 路径：`UnderwaterRobotSystem/OrangePi_STM32_for_ROV`
- 本轮功能提交：`50e3d71 统一导航视图语义并收紧控制侧保护`

## 本轮改动目标

本仓库本轮只聚焦导航消费侧 P0 整改：

- `NavState -> NavStateView -> ControlLoop` 语义统一
- stale/invalid/no-data/degraded/fault 不再折叠成一个布尔值
- Auto 模式按显式状态语义决定是否允许闭环
- stale 时不再复用旧运动学 payload 继续控制

## 关键改动

### 1. gateway 侧 NavView 语义统一

涉及：

- `gateway/apps/nav_viewd.cpp`
- `gateway/include/gateway/IPC/nav/nav_state_subscriber_shm.hpp`
- `gateway/include/gateway/IPC/nav/nav_view_builder.hpp`
- `gateway/src/IPC/nav/nav_state_subscriber_shm.cpp`
- `gateway/src/IPC/nav/nav_view_builder.cpp`

核心变化：

- `NavState` 默认输入 SHM 名称统一到 `/rov_nav_state_v1`
- `NavViewBuilder` 透传 upstream 的 `valid/stale/degraded/fault` 语义
- stale/no-data 时发布显式 invalid 诊断帧
- stale 诊断帧保留时间戳和诊断位，但清空控制面运动学字段

### 2. 控制侧不再把导航异常折叠成 no-data

涉及：

- `pwm_control_program/include/control_core/control_types.hpp`
- `pwm_control_program/include/io/nav/nav_state_view.hpp`
- `pwm_control_program/include/io/nav/nav_view_shm_source.hpp`
- `pwm_control_program/src/control_core/loop/control_loop_nav.cpp`
- `pwm_control_program/src/io/nav/nav_view_shm_source.cpp`

核心变化：

- `ControlState` 新增 `nav_present/nav_stale/nav_degraded/nav_state/nav_health/nav_fault_code/nav_age_ms/nav_sensor_mask`
- `NavViewShmSource` 总是返回最新快照，由上层显式消费 `valid/stale/fault`
- 控制侧 age 采用逐 hop 累积，不再各算各的

### 3. Auto 模式保护收紧

涉及：

- `pwm_control_program/include/control_core/control_guard.hpp`
- `pwm_control_program/src/control_core/control_guard.cpp`

核心策略：

- `UNINITIALIZED/ALIGNING/INVALID` 禁止进入 Auto
- `valid=0` 或 `stale=1` 禁止闭环
- `fault_code!=kNone` 禁止闭环
- Auto 需要 `IMU_OK + ALIGN_DONE + ESKF_OK`
- 已在 Auto 中运行时也会持续检查导航，不再只在切模态瞬间检查一次

### 4. 测试接入

本轮新增并接入：

- `gateway/tests/test_nav_view_builder.cpp`
- `pwm_control_program/tests/test_v1_closed_loop.cpp`
- `pwm_control_program/tests/test_nav_view_shm_source.cpp`

覆盖点包括：

- invalid NavState 不再向控制面暴露旧 payload
- degraded NavState 语义透传
- Guard 拒绝 `ALIGNING` Auto
- Guard 允许健康的 `DEGRADED` Auto
- SHM source 不再把 invalid/stale 折叠成 no-data

## 本轮编译与测试

已验证：

- 编译：`nav_viewd`
- 编译：`pwm_control_program`
- 测试：`gateway_test_nav_view_builder`
- 测试：`pwmctrl_test_v1_closed_loop`
- 测试：`pwmctrl_test_nav_view_shm_source`

说明：

- `pwmctrl_test_nav_view_shm_source` 依赖 POSIX SHM，默认沙箱内权限受限，已在非沙箱环境重跑通过。

## 当前残余

- 本仓库仍存在大量与本轮无关的未提交改动，包括 telemetry、session、controller、产品化文档等其他功能线。
- 这些残余改动没有纳入本轮 commit，后续应按功能单独拆分。
- 根目录 `build/` 下仍有大量构建产物和日志噪声，说明根级 `.gitignore` 与已跟踪产物治理仍需单独处理。

## 远程提交前建议

- 先只按本轮导航 P0 commit 审查，不要把其他功能线一并推送。
- 推送前建议再拆一轮仓库内剩余改动，至少区分 telemetry、GCS/session、controller framework 三条线。
