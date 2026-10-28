<!-- release-category: pico -->

## 中文

- 增加独立单臂 i3 的右手柄遥操入口，支持 base Z 映射角和可选夹爪控制；双臂默认沿用原 ServoP 链路。
- 单臂可选择 ServoP、FDCC 或笛卡尔阻抗模式。松开 Grip 后退出柔顺模式持位，再次握紧从实测 TCP 重新对齐。
- 增加无效数值和目标预检、速度与加速度受限的目标整形，以及输入、检查和跟随误差诊断。输入仍新鲜但预检暂慢时，冻结最后发送的平衡点并保留模式；检查通过后自动续跟。输入断流、故障和候选拒绝保留停止处理。
- Pico 程序已通过官方 C++ SDK 0.2.0 编译验证；需显式启用 Pico bridge 构建并使用支持对应接口的柜端程序，当前预编译 SDK bridge 制品仍提供 SpaceMouse 和六维力接入。阻抗实机跟随效果需按具体构型验证。

## English

- Add right-controller teleoperation for a standalone i3 arm, with base-Z mapping yaw and optional gripper control. Dual-arm operation retains the existing ServoP default.
- Select ServoP, FDCC or Cartesian impedance for a single arm. Releasing Grip exits compliance mode to hold; re-engaging Grip anchors the target to the measured TCP.
- Add invalid-value and target checks, velocity- and acceleration-bounded target shaping, and input, validation and tracking diagnostics. When input is fresh but validation is delayed, retain the mode and freeze the last sent equilibrium; tracking resumes after validation succeeds. Input loss, faults and rejected candidates retain stop handling.
- The Pico application builds against official C++ SDK 0.2.0. Enable the Pico bridge explicitly and use cabinet software supporting the selected interfaces. Current prebuilt SDK bridge assets still provide SpaceMouse and force/torque integration. Hardware impedance tracking requires validation for the target configuration.
