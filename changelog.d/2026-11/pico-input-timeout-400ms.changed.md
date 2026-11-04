<!-- release-category: pico -->

## 中文

- 按现场无线遥操需求，将 PICO 输入超时由 250ms 调整为 400ms，适用于单/双臂输入新鲜度及柔顺控制输入截止时间。真实断流触发停止的等待窗口相应增加 150ms；追踪丢失、机器人/Jacobian 状态过期、200ms 目标校验有效期及 Grip 恢复规则保持原样。

## English

- Increase the PICO input timeout from 250ms to 400ms for single/dual-arm freshness and compliance input deadlines. A true stream loss may take an additional 150ms to trigger stopping; tracking loss, robot/Jacobian freshness, 200ms target approval validity and Grip recovery rules remain unchanged.
