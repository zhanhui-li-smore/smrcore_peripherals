<!-- release-category: pico -->

## 中文

- PICO 力控退出或模式确认失败时尝试急停，再尝试下使能，避免只执行会被 CST 拒绝的普通下使能而遗留运动。新增离线故障注入回归；未进行实机故障测试，RPC 失败仍需操作者确认硬件停止状态。

## English

- On compliance exit or mode acknowledgement failure, PICO requests emergency stop before Disable. Offline fault injection covers CST Disable rejection; no live fault test was performed. Failed RPCs still require physical stop verification.
