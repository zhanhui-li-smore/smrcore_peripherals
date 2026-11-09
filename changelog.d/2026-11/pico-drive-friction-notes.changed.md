<!-- release-category: docs -->

## 中文

- Pico 快速说明区分驱动器摩擦补偿与 FDCC 阻尼参数：Pico 不额外设置摩擦补偿系数，沿用配套 RCore 默认控制流程；需要调整时通过控制柜支持的接口手动配置。FDCC 的默认 `damping_factor=0.9` 保持不变。

## English

- Distinguish drive friction compensation from FDCC damping in the Pico quickstart. Pico does not set an additional friction coefficient and follows the paired RCore default control flow; use the cabinet's supported interface for manual adjustments. Keep the default FDCC `damping_factor=0.9`.
