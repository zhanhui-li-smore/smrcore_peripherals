<!-- release-category: pico -->

## 中文

- PICO 双臂遥操新增 `--control-mode fdcc`，左右 Grip 独立启停，沿用 B3 坐标映射与 Trigger 夹爪操作。两臂分别以最高 50 Hz 更新目标，并保留各自柜端力源配置。
- 单侧跟踪丢失或目标拒绝时暂停该臂并要求松开再握紧；断流暂停双臂，任一臂控制故障结束双臂会话。退出时先取消两侧目标，再退出力控和下使能；FDCC 不使用摇杆 J6。
- 补充双语启动说明和离线双臂故障注入测试。默认仍为 ServoP。

## English

- Add `--control-mode fdcc` to PICO dual-arm teleoperation with independent Grip control, existing B3 mapping and Trigger gripper operation. Each arm updates targets at up to 50 Hz and preserves its cabinet force source.
- Per-arm tracking loss or target rejection pauses that arm until release/regrip. Stream loss pauses both; a control fault on either arm ends both. Exit cancels both target streams before leaving FDCC and disabling motors; J6 thumbstick motion is disabled in FDCC.
- Add bilingual instructions and offline dual-arm fault-injection tests. ServoP remains the default.
