# 版本说明草稿

Codex 根据代码差异生成，尚未人工审核。

v0.1.1 → `f254614929a8cce9bfee8a9c17399f69272eb243` · SDK `0.2.0`

## Pico

- C++ Pico bridge 的 `--control-mode fdcc` 和 `--control-mode impedance` 扩展到双臂。左右 Grip 独立控制各臂进入和退出；任一工作线程故障会取消双臂目标并结束会话。默认仍为双臂 ServoP，柔顺模式不使用摇杆 J6 控制。FDCC 和阻抗的主机目标发送周期分别配置为每臂 20ms 和 5ms，不代表实测更新率。 ([869d31b](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/869d31b834a3c0333d9172eac6c6d9a0d647a2d5))
- 新增 `--fdcc-damping-factor`，范围为 \[0,1\]，默认 0.9；新增 `--fdcc-error-scale`，测试入口范围为 \[0.1,4\]，默认 1。前者设置虚拟积分速度保留比例，后者同时缩放位姿和外力 PD 输出。它们不是驱动器摩擦补偿参数；Pico 沿用配套 RCore 的默认摩擦处理流程。 ([869d31b](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/869d31b834a3c0333d9172eac6c6d9a0d647a2d5))
- 阻抗默认刚度改为 `400,400,400,30,30,30`，阻尼改为 `30,30,30,2,2,2`，会改变旧命令的控制参数。新增 `--impedance-profile hold` 恢复原刚度和阻尼。显式 `--impedance-stiffness`、`--impedance-damping` 分别覆盖预设，不受参数顺序影响。 ([869d31b](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/869d31b834a3c0333d9172eac6c6d9a0d647a2d5))
- 新增 `--fdcc-direct-target` 和 `--impedance-direct-target`，直接发送已批准目标，跳过主机目标平滑及速度、加速度整形。每次激活仍先发送实测 TCP，保留目标校验、有效期和退出检查。FDCC 直接模式要求 `--fdcc-target-scale 1`。 ([869d31b](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/869d31b834a3c0333d9172eac6c6d9a0d647a2d5))
- 新增 `--impedance-joint-max-speed`，显式范围为 (0,180\] 度/秒，与阻抗直接模式互斥。它限制已批准 IK 关节参考的推进量，再通过 FK 生成平衡点；长调度间隔最多计入 20ms。该参数限制参考轨迹，不保证实测关节速度。 ([869d31b](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/869d31b834a3c0333d9172eac6c6d9a0d647a2d5))
- 阻抗退出遇到 `DisableCartesianImpedance()` 返回 4311 时，最多调用三次继续等待停稳，期间不发送新目标或重新激活。持续失败、其他退出错误或模式退出确认失败时，FDCC 和阻抗退出路径会请求 `EStop()`，再尝试 `Disable()`。 ([869d31b](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/869d31b834a3c0333d9172eac6c6d9a0d647a2d5))
- Pico 输入过期阈值从 250ms 改为 400ms；机器人状态和目标校验的时效检查仍独立执行。柔顺模式输入过期后需要松开 Grip 才能重新激活。双臂日志新增输入年龄、接收间隔、序号和跟踪状态；关节候选拒绝日志区分 J3/J5 实测接近边界、IK 接近边界及分支跨越。 ([869d31b](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/869d31b834a3c0333d9172eac6c6d9a0d647a2d5))
- 双臂启动新增 SDK 连接、命令源获取和参考系解析进度。初始化后及后续阶段之间检查退出请求，取消时清理连接，避免继续启动。 ([869d31b](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/869d31b834a3c0333d9172eac6c6d9a0d647a2d5))

## 公共接口与其他更新

- 新增 Linux C++ 应用 `app_peripherals_keyboard_bridge`，支持单臂和双臂独立按键组。笛卡尔模式调用 `Motion().ServoRelativeP()`，关节模式调用 `Motion().ServoJ()`。支持 `--frame base|tool`、无机器人连接的 `--dry-run`，实际连接需显式 `--enable-motion`。 ([869d31b](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/869d31b834a3c0333d9172eac6c6d9a0d647a2d5))
- 键盘在本地 X11 终端支持连续按键和组合键；SSH、Wayland 或 `--terminal` 使用每次按键 1mm 或 0.5° 的有限点动。Space 切换输入启用状态；Tab 等待双臂停稳后切换笛卡尔/关节空间，并保持暂停。焦点丢失或输入失步会暂停；退出会保持 Servo 目标、请求下使能并检查反馈。 ([869d31b](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/869d31b834a3c0333d9172eac6c6d9a0d647a2d5))

## SDK 与构建发布

- Linux SDK 构建新增键盘应用安装目标，发布工作流将其纳入 SDK bridge 包，并配置 `--help` 加载检查。X11 连续按键支持通过可选 XCB 构建，需 `libxcb1` 运行库；未找到 XCB 时保留终端点动。Pico 遥操仍需匹配 SDK 的源码构建，默认预编译包不包含该程序。 ([869d31b](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/869d31b834a3c0333d9172eac6c6d9a0d647a2d5))

## 文档

- 新增中英键盘操作指南，并纳入站点导航和双语手册。补充 Pico 双臂柔顺模式参数、恢复和退出说明，以及中文遥操速查与 FDCC 流程图；进一步精简 Unreleased 的 Pico 和键盘说明。 ([869d31b](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/869d31b834a3c0333d9172eac6c6d9a0d647a2d5), [f254614](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/f254614929a8cce9bfee8a9c17399f69272eb243))

## 待核实

- 源码和工作流不能证明这些变更已通过实机验证、达到配置更新率，或所需 API 已包含在发布 SDK 二进制中。本次仅进行了只读审阅，未运行构建或测试。
- 新增中文速查称 FDCC 外力低通从 10Hz 改为 5Hz，但本范围没有对应控制器实现变更，无法确认该行为或其交付版本。
