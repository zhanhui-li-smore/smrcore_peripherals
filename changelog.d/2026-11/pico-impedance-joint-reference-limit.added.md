<!-- release-category: pico -->

## 中文

- 增加 `--impedance-joint-max-speed`（°/s，最大 180）用于单/双臂阻抗遥操：对已批准 IK 解做关节参考变化率限制，再经 FK 转为平衡点；与 direct-target 互斥。首帧锚定、过期保持、松手退出和柜端保护保留，长停顿不积累大跳变。需配套更新主机 CLI/库；额外 FK 往返可能降低更新率，不保证实际阻抗运动不超速，实机尚未验证。

## English

- Add `--impedance-joint-max-speed` (degrees/s, up to 180) for single/dual-arm impedance teleoperation. Rate-limit approved IK joint references and map them to equilibrium poses through FK; mutually exclusive with direct-target. Preserve activation seeding, stale-target holds, release exit and cabinet guards, and cap elapsed-time catch-up. Update the host CLI/library together. Extra FK calls may reduce update rate; actual impedance motion is not guaranteed to stay below the reference limit. Hardware behavior remains unverified.
