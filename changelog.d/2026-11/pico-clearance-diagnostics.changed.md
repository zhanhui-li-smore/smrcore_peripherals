<!-- release-category: pico -->

## 中文

- PICO 肘部/腕部目标校验拒绝时记录 J3/J5 实际与逆解角度、实际/目标禁区及跨边界标志，并保留目标位姿、输入序号和状态时间戳，便于定位反复暂停。仅增加诊断，不改变保护条件或恢复流程。

## English

- Include measured/IK J3 and J5 angles, actual/target exclusion flags, boundary-crossing flags, target pose, input sequence and feedback timestamp in PICO elbow/wrist rejection reasons. Diagnostics only; protection conditions and recovery behavior are unchanged.
