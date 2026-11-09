<!-- release-category: pico -->

## 中文

- 增加 `--impedance-direct-target`，供单/双臂 Pico 阻抗遥操跳过主机平移和旋转目标整形（限速、限加速度及追赶平滑），直接发送最新已批准目标。默认仍采用原整形模式；首次目标保持实测 TCP，候选检查、输入过期、松手退出与柜端保护保留。需配套更新主机 CLI 与库，无需修改柜端；实机响应尚未验证。

## English

- Add `--impedance-direct-target` for single/dual-arm Pico impedance teleoperation to bypass host translation/rotation target shaping and send the latest approved target directly. Shaping remains the default. Measured activation seeding, candidate checks, expiry handling, release exit and cabinet protections remain active. Update the host CLI and library together; no cabinet update is required. Hardware response remains unverified.
