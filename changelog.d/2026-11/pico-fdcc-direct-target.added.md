<!-- release-category: pico -->

## 中文

- 增加可选 `--fdcc-direct-target`，在实测位置首帧后直接下发批准目标，用于对比主机目标平滑造成的滞后；绕过主机平滑、目标限速/限加速度和速度前馈，保留校验、超时、暂停及模式保护，默认行为不变。单臂和双臂均支持，要求 FDCC 且目标外推倍率为 1。

## English

- Add optional `--fdcc-direct-target` for direct approved pose writes after the measured activation seed, enabling comparisons against host shaping lag. It bypasses host shaping, target speed/acceleration limits and velocity feedforward while retaining checks, expiry, pause and mode guards. Defaults remain unchanged; single and dual arms require FDCC with target extrapolation scale 1.
