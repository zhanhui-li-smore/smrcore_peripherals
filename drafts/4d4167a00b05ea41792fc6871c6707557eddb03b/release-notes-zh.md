# 版本说明草稿

Codex 根据代码差异生成，尚未人工审核。

v0.1.0 → `4d4167a00b05ea41792fc6871c6707557eddb03b` · SDK `0.2.0`

## Pico

- 新增 `--single-arm`，通过 SDK `Single` 实例用物理右手柄控制独立单臂。启动不要求左手柄就绪。映射使用 `base` 坐标系；`face-to-face` 不交换控制手柄。默认仍为双臂 ServoP。 ([f6e46d1](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/f6e46d1afa1ea5240ad1a5d45b391ac2a308bb96))
- 新增 `--single-mapping-yaw`，在单臂模式下绕 base Z 旋转平移和旋转增量，范围为 −180° 至 180°，默认 0°。新增 `--no-gripper`，可跳过单臂或双臂的 DH 夹爪初始化和 Trigger 控制。 ([f6e46d1](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/f6e46d1afa1ea5240ad1a5d45b391ac2a308bb96))
- 单臂新增 `--control-mode fdcc|impedance`，对应 `PicoControlMode::Fdcc` 和 `PicoControlMode::CartesianImpedance`。Grip 松开后请求退出柔顺模式并持位，重新握紧从实测 TCP 对齐。两种模式均禁用摇杆 J6，保留夹爪和方向映射。 ([f6e46d1](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/f6e46d1afa1ea5240ad1a5d45b391ac2a308bb96))
- FDCC 使用 `SetFdCartesianAdmittancePoseTarget`，发送上限为 50 Hz。默认平移目标整形为 0.30 m/s、1.2 m/s² 和增益 8/s，速度前馈为 0.5。`--fdcc-target-scale` 可启用额外平移超前，最大增加 30 mm；这些参数约束目标，不代表实测运动上限。 ([f6e46d1](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/f6e46d1afa1ea5240ad1a5d45b391ac2a308bb96))
- 阻抗模式使用 `SetCartesianImpedanceTarget` 写入平衡位姿，发送上限为 200 Hz。`--impedance-stiffness` 和 `--impedance-damping` 接受 XYZ/RX/RY/RZ 六个数值；目标整形参数与刚度、阻尼分开设置。DDS 写入没有送达确认。 ([f6e46d1](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/f6e46d1afa1ea5240ad1a5d45b391ac2a308bb96))
- 柔顺模式在输入新鲜但目标预检超过 200 ms 未更新时，保留模式并冻结最后发送的平衡点；新检查通过后自动续跟。输入或雅可比过期、候选被拒绝时暂停，需新鲜的松 Grip 输入及模式 0 确认后重新离合。状态过期、模式丢失或写入失败会结束会话。 ([f6e46d1](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/f6e46d1afa1ea5240ad1a5d45b391ac2a308bb96))
- 新增非有限输入、无效四元数及发送目标检查。单臂忽略未使用的左手数据。柔顺模式使用实测雅可比减速，不再因候选端点条件数达到 60 而拒绝；仍检查 IK 可达性、关节余量及 FK 一致性。ServoP 保留原候选边界策略。 ([f6e46d1](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/f6e46d1afa1ea5240ad1a5d45b391ac2a308bb96))
- 新增输入包龄、检查耗时、目标整形差距及跟随误差诊断。`--help-advanced` 展示调参选项，普通帮助聚焦操作参数。新增选项通过 `PicoTeleopOptions` 提供；参数范围和模式组合在连接前校验。 ([f6e46d1](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/f6e46d1afa1ea5240ad1a5d45b391ac2a308bb96))

## SDK 与构建发布

- 修复通过 `find_package(smrcore_peripherals CONFIG)` 使用共享库时额外要求私有开发依赖的问题。共享库配置不再查找 `Threads` 和 `nlohmann_json`；静态库继续解析这些依赖，导出目标仍为 `smrcore::peripherals`。 ([abe9328](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/abe93282b2a87a92ddf010986665f6be9263c656))
- 网站、手册及 Release 说明生成器新增按不可变 Git 版本范围读取双语变更片段。已发布记录不受后续编辑影响；未提交内容及 `misc` 片段不发布。保留审核摘要优先级和提交说明回退，并校验语言及分类。网站工作流提供 Markdown 预览制品。 ([4d4167a](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/4d4167a00b05ea41792fc6871c6707557eddb03b))

## 文档

- 补充中英文单臂 Pico 操作、柔顺模式、调参及暂停/退出说明。现有 Python 启动器可透传这些参数到 C++ bridge。Pico 遥操仍需显式启用 `--with-sdk ON --with-pico ON` 构建；当前预编译 SDK bridge 工作流仍关闭 Pico，仅构建 SpaceMouse 和六维力接入。 ([f6e46d1](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/f6e46d1afa1ea5240ad1a5d45b391ac2a308bb96))

## 待核实

- 源码包含 FDCC 和阻抗接口调用，但本次只读审查不能确认已发布 SDK 二进制及目标柜端提供这些接口，也不能确认实机跟随、接触或 CST 模式切换效果。文档中的编译及离线验证记录不等于硬件验收。
