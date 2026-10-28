> DIR: app/

# Overview

可执行示例与 SDK 桥接/标定工具。由 `CMakeLists.txt` 中 `add_peripheral_app()` 构建并 install 到 `${CMAKE_INSTALL_BINDIR}`。`app_peripherals_bridge`、`app_peripherals_pico_bridge` 与 `app_peripherals_ft_sensor_calib` 仅在 `SMR_PERIPHERAL_WITH_SDK=ON` 时编译。构建与运行说明见 `README.md` / `README.en.md`。

# Contents

- app_peripherals_probe.cpp: 打印 SpaceMouse/F/T 管理面状态（`GetInfo()`，不 `Start()`）
- app_peripherals_read_spacemouse.cpp: 定频循环打印归一化 `SpaceMouseSample`
- app_peripherals_read_spacemouse_raw.cpp: 定频循环打印 `SpaceMouseRawSample`（原始轴+按键）
- app_peripherals_read_ft_sensor.cpp: `FtSensorOptions` 初始化 → `WaitForFirstSample()` → 打印六维力/力矩
  - CLI: `--serial-port` `--sensor-type` `--baud-rate` `--sample-rate` `--stale-timeout-ms` `--read-buffer-size`
- app_peripherals_read_pico.cpp: 启动 `Pico` localhost WebXR 服务并打印头显/双手柄 WebXR XYZ（米，4 位小数；未追踪显示 N/A）及按键采样；仅有 `--port`，不连接机器人
- app_peripherals_bridge.cpp: 单一 `Robot` SDK 会话内按 CLI 启动 SpaceMouse 和/或 F/T reader；默认两路都启用于 FDCC，`--spacemouse` / `--ft-sensor` 可显式选择单路或两路；连接成功后打印“正在发送外设消息”状态；需 `./scripts/download.sh` 后以 `--with-sdk ON` 构建
- app_peripherals_ft_sensor_calib.cpp: MoveJ 静态六维力标定（bias/重力/质心/yaw；SDK `GetFtSensorRawState` + Preview/Save）；仅 `--with-sdk ON` 时编译
- app_peripherals_pico_bridge.cpp: PICO 双臂遥操 CLI，调用共享 `smrcore_pico_teleop` 核心；内置 B3 支架变换（对应 B3 双臂安装构型，非通用配置），左右 `world_from_*_arm_base`，无 mount 文件/环境变量依赖，切换并确认 base reference 后以同一刚体链处理平移和旋转，无独立轴/yaw 补偿。`--gripper-force/--gripper-speed` 配置末端板 DH 夹爪，Trigger 按下沿切换开关；按住 Grip 时摇杆 Y 轴以 `--j6-speed` 控制对应机械臂 J6。要求 --enable-motion；Grip 离合、奇异点保护与停止逻辑见 `../teleop/_index.md`。

本仓库以 `--with-sdk OFF` 构建时不编译 bridge 与 F/T 标定应用。

- PICO guarded ServoP: 靠近奇异点时 30..60 二次减速到零，离开时使用双倍门槛恢复，到达奇异边界冻结目标但保持会话；非实时线程先 IK + FK 数值雅可比检查候选，B3 q3/q5 距 nπ ≥10°，候选边界只保持最后批准目标。未验证/超过 200ms 的目标不发送；Jacobian 短暂超过 250ms 时仅冻结并制动受影响臂，PICO 输入超过 250ms 时冻结双臂并取消待发夹爪命令，两者恢复后均从实测状态重新离合；陈旧 RobotState、参考系切换仍触发双臂 StopMotion 后退出。松 Grip 取消旧 guard generation；J6 摇杆及松手停止均走 ServoJ 规划，保持命令连续，回中后从当前 TCP 重新离合。后端关节限位、速度保护与急停保留。

- `app_peripherals_pico_bridge.cpp` is CLI-only; delegates to `smrcore_pico_teleop`, retains motion flags and exposes the single-arm base-Z mapping offset. Core behavior is documented in `../teleop/_index.md`.

- app_peripherals_pico_bridge 仅在 SMR_PERIPHERAL_WITH_PICO_TELEOP=ON 时构建，原有 bridge/calib 仅依赖 WITH_SDK。

- README.md / README.en.md: 保留原桥接/标定说明，Pico 使用说明统一指向 docs/site/pico，移除过时的减速和 J6 描述。

- Pico bridge: `--single-arm` selects standalone i3 with physical right hand; `--no-gripper` skips DH IO. Default remains B3 dual-arm.

- `--control-mode servop|fdcc|impedance` defaults to ServoP; force modes require `--single-arm`,
  reuses the right hand/base-yaw mapping and disables J6 thumbstick. Mode
  transition/hold/cleanup live in `teleop/pico_fdcc.hpp`.
  `--fdcc-max-speed/max-accel/follow-gain/target-scale` adjust translation
  shaping/extrapolation; finite/range/mode checks run before any connection.
  `--fdcc-target-feedforward` selects [0,1] target velocity feedforward, default
  0.5; 0 restores old shaping, with unchanged linear/angular rate limits.
  Normal `--help` shows connection/mode/mapping/operation options;
  `--help-advanced` adds tuning. Existing tuning flags remain compatible.
  FDCC startup defaults to the built-in 0.30/1.2/8/0.5 profile; no tuning flags needed.

- `--impedance-stiffness/damping` accept exactly six finite comma-separated
  values (XYZ/RX/RY/RZ); `--impedance-max-speed/max-accel/follow-gain` configure
  equilibrium shaping. Mode 5 uses the same clutch/yaw/guard path; the SDK
  performs CST torque control, while target writes use DDS without RPC ack.

- `--single-mapping-yaw DEG`: finite [-180,180], default 0; single-arm
  base-Z mapping offset for translation and rotation, -90 clockwise viewed
  from +Z. Does not change head-yaw clutching, cabinet frames or URDF.
