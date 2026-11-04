<!-- release-category: pico -->

## 中文

- 单臂 PICO 阻抗默认采用 SDK SpaceMouse 遥操参数（平移 K/D=400/30，转动 K/D=30/2），提高相对原定点保持档的恢复力。增加 `--impedance-profile hold` 保留原柔软档；显式 K/D 分别覆盖档位且不受参数顺序影响。启动显示实际参数，继续保留目标限速、松 Grip 阻尼退出和故障保护。已做离线验证，尚未验证实机跟随改善。

## English

- Default single-arm PICO impedance to the SDK SpaceMouse gains (translation K/D=400/30, rotation K/D=30/2), increasing restoring force over the old hold profile. Add `--impedance-profile hold` for the original soft gains; explicit K/D override independently of argument order. Print resolved startup parameters while retaining target limits, Grip-release damping exit and fault handling. Verified offline; hardware tracking improvement remains unverified.
