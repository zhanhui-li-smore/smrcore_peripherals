<!-- release-category: pico -->

## 中文

- PICO 单/双臂 FDCC 新增 `--fdcc-damping-factor`，可设置虚拟模型每积分子步的速度保留比例，默认 0.9，范围 [0,1]；减小表示更强衰减，不能等同于任务空间阻尼或已验证的闭链稳定参数。非法值在连接前拒绝，其他 FDCC 参数保持原默认。PicoTeleopOptions 增加字段，CLI 与遥操库需配套重新编译。

## English

- PICO single/dual-arm FDCC adds `--fdcc-damping-factor` for per-substep virtual velocity retention, default 0.9, range [0,1]. Lower values increase decay; this is not task-space damping or a validated closed-chain stability profile. Rebuild the CLI and teleoperation library together after the Options layout extension.
