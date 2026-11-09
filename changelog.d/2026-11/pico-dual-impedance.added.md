<!-- release-category: pico -->

## 中文

- 双臂 PICO 新增 `--control-mode impedance`，沿用 B3 / face-to-face 映射，两臂各自进入 CST 笛卡尔阻抗并以最高 200 Hz 更新平衡点。共用可配置 SpaceMouse/hold K/D，各自 Grip 松开、重新离合及目标校验；一侧跟踪丢失仅暂停该侧，任一臂故障或 SDK 控制权丢失结束整个会话。退出先取消两侧待发目标再等待停稳并下使能，不使用 FDCC 目标放大或位置控制回退。已通过编译及离线故障注入测试，实机未验证。

## English

- Add dual-arm PICO `--control-mode impedance` with existing B3 / face-to-face mapping, independent CST Cartesian impedance workers and up to 200 Hz equilibrium updates per arm. Both arms share configurable SpaceMouse/hold gains but keep independent Grip release/reclutch and target guards. Tracking loss pauses only its arm; either arm's fault or SDK ownership loss ends the session. Teardown cancels both unsent streams before waiting and disabling, without FDCC extrapolation or position-control fallback. Build and offline fault-injection tests passed; hardware validation is pending.
