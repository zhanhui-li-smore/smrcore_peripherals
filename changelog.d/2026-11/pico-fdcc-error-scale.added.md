<!-- release-category: pico -->

## 中文

- PICO 单/双臂 FDCC 新增 `--fdcc-error-scale`，设置原生 FDCC 位姿/外力 PD 输出倍率，默认 1.0，本地测试入口范围 [0.1,4]；范围不是机器人硬件限值。增大倍率可能改善响应，也会放大外力反馈并降低接触稳定裕度。默认参数不变，CLI 与遥操库须配套重新编译。

## English

- PICO single/dual-arm FDCC exposes `--fdcc-error-scale` for the native pose/wrench PD-output multiplier, default 1.0, local test range [0.1,4]. Higher gain may improve response but also amplifies force feedback and can destabilize contact. Rebuild the paired CLI/library; defaults remain unchanged.
