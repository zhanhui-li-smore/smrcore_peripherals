## 1. 两种控制方式的基本原理和参数

**FDCC：** 将手柄目标位姿与估算外力输入虚拟动力学模型，积分生成关节位置目标，再由位置伺服执行（CSP）。接触外物时，外力改变虚拟模型的运动，使机械臂让位。

**笛卡尔阻抗：** 根据目标与实际末端的位姿误差，通过刚度、阻尼计算恢复力，经雅可比转置转换为关节力矩，并叠加动力学补偿，由力矩伺服执行（CST）。相当于末端与目标之间连接虚拟弹簧和阻尼器。

| 项目 | FDCC | 笛卡尔阻抗（默认 `spacemouse`） |
|---|---|---|
| 平移 / 转动刚度 | 1000 N/m / 80 N·m/rad | 400 N/m / 30 N·m/rad |
| 阻尼设置 | `damping_factor` 默认 0.9：每次虚拟积分保留的速度比例 | 平移 30 N·s/m；转动 2 N·m·s/rad |
| 驱动调节 | `error_scale` 默认 1.0：内部位姿与力误差驱动倍率 | 通过刚度、阻尼调节恢复作用 |
| 目标力 / 力矩 | 六维均为 0 | 六维均为 0 |
| 主机每臂目标更新上限 | 50 Hz，RPC | 200 Hz，DDS |
| 柜端控制循环 | 1 kHz | 1 kHz |

FDCC 当前使用关节力矩结合模型估算外力；进入控制器的外力低通滤波已由 **10 Hz 改为 5 Hz**，两臂均生效。零目标力不代表实际接触力为零。

`damping_factor` 越小，虚拟速度衰减越强，通常跟随越慢；提高 `error_scale` 可增强驱动，但可能降低稳定裕度。阻抗提高刚度可增强恢复作用，接触手感也会变硬。

阻抗另有 `hold` 预设：平移 / 转动刚度为 **120 N/m / 20 N·m/rad**，平移 / 转动阻尼为 **20 N·s/m / 1 N·m·s/rad**。

驱动器摩擦补偿与上述阻尼不同：`0x2205` 使用驱动器原始量程 0～100，
Pico 不额外设置摩擦补偿系数，沿用配套 RCore 的默认控制流程。
需要调整时通过控制柜支持的参数接口手动配置；FDCC 的 `damping_factor=0.9`
是虚拟模型阻尼参数，不是摩擦补偿系数。

## 2. 脚本可配置参数

对应本机测试程序：`build_dual_fdcc_response/bin/app_peripherals_pico_bridge`。下表列出主要调节项；速度、加速度限制作用于输入目标整形，不代表机械臂硬件极限。

| 类别 | 参数 | 默认值 | 作用 |
|---|---|---|---|
| 公共 | `--control-mode` | 本文选择 `fdcc` 或 `impedance` | 控制方式 |
| 公共 | `--operator-frame` | `same-side` | `face-to-face` 为面对面映射：右手柄控制左臂，左手柄控制右臂 |
| 公共 | `--translation-scale` / `--rotation-scale` | 0.8 / 1.0 | 平移 / 转动映射倍率 |
| 公共 | `--translation-only` | 关闭 | 仅跟随平移 |
| 公共 | `--no-gripper` | 未设置 | 跳过夹爪初始化与控制；默认启动会请求打开夹爪 |
| FDCC | `--fdcc-max-speed` / `--fdcc-max-accel` | 0.30 m/s / 1.2 m/s² | 输入目标速度、加速度限制 |
| FDCC | `--fdcc-follow-gain` | 8 s⁻¹ | 输入目标追赶增益，不是柜端刚度 |
| FDCC | `--fdcc-target-feedforward` | 0.5 | 输入目标速度前馈比例 |
| FDCC | `--fdcc-damping-factor` | 0.9 | 虚拟速度保留比例，范围 0～1 |
| FDCC | `--fdcc-error-scale` | 1.0 | 内部驱动倍率，当前测试入口范围 0.1～4 |
| 阻抗 | `--impedance-joint-max-speed` | 0（关闭） | 关节参考速度上限，单位 °/s，显式取值 (0,180]；与 direct-target 互斥 |
| 阻抗 | `--impedance-direct-target` | 关闭 | 直接发送已批准平衡点，跳过主机平移/旋转限速、限加速度和追赶平滑 |
| 阻抗 | `--impedance-profile` | `spacemouse` | 刚度、阻尼预设，可选 `hold` |
| 阻抗 | `--impedance-stiffness` / `--impedance-damping` | 见第 1 节 | 分别设置六维刚度、阻尼，顺序为 X、Y、Z、Rx、Ry、Rz |
| 阻抗 | `--impedance-max-speed` / `--impedance-max-accel` | 0.15 m/s / 0.6 m/s² | 平衡点目标速度、加速度限制 |
| 阻抗 | `--impedance-follow-gain` | 8 s⁻¹ | 输入目标追赶增益 |

## 3. 视频展示

| 展示内容 | 视频 |
|---|---|
| FDCC 单臂悬空跟随、接触让位 | 待补充 |
| FDCC 双臂共同夹持刚性物体，比较振荡与减振后的响应 | 待补充 |
| 笛卡尔阻抗跟随与接触柔顺 | 待补充 |

## 4. 问题

| 实测现象 | 当前判断与进展 |
|---|---|
| FDCC 双臂共同夹持刚性物体时抖动 | 存在闭链耦合振荡现象。当前两臂独立控制，缺少内力协调；减小虚拟速度保留比例可以消除抖动，但是跟随性显著变差，可以虚拟位置补偿 |
| 阻抗模式低刚度跟随慢 | 摩擦、负载及动力学补偿可能影响响应，尚未量化；受摩擦影响大，第一次下发跟随延时显著。直接提高刚度会降低柔顺性 |

## 5. 总结

当前主要矛盾是**快速跟随与柔顺稳定之间的取舍**。
FDCC模式下，fdcc-damping-factor如果太高，会引起双臂耦合震荡，过小会显著降低速度不跟手。
笛卡尔阻抗在当前测试中未出现振荡，但精密操作时臂的启动受摩擦影响较大，响应慢且响应大。已有驱动器摩擦补偿，控制器未额外叠加软件摩擦补偿；摩擦参数可通过控制柜支持的接口手动调整，Pico 沿用默认控制流程。


pico做一个按钮？切换位控制和力控制，位做粗定位，力做柔性任务。
 进一步方向：fdcc双臂控制，统一调节内力，减少互相打架。阻抗低速迟滞的问题，在线估算补偿摩擦？

## 阻抗直接跟随（跳过主机目标整形）

本机已配套编译的入口：

```bash
cd /home/smore/rcore/peripheral
./build_impedance_direct/bin/app_peripherals_pico_bridge \
  --robot-ip 192.168.3.232 --port 8765 \
  --control-mode impedance --impedance-profile spacemouse \
  --impedance-direct-target --operator-frame face-to-face \
  --no-gripper --enable-motion
```

DIRECT 模式不使用 `--impedance-max-speed/max-accel/follow-gain` 的主机整形值。
每次进入的首帧仍锚定实测 TCP，之后直接发送最新已批准目标；候选位姿检查、
奇异位形保护、断流保护、Grip 松手退出以及柜端保护仍然生效。
这只改变主机平衡点跟随方式，不保证机械臂达到手柄速度，也不修改柜端 K/D。
已做离线验证，实机跟随和接触表现尚未验证。只需重启新版主机 bridge，不需要再次部署控制柜。

## 按关节参考速度 180°/s 限制（突变目标场景）

退出旧 bridge 后，在开发机启动此配套构建，不带 `--impedance-direct-target`：

```bash
cd /home/smore/rcore/peripheral
./build_impedance_joint180/bin/app_peripherals_pico_bridge \
  --robot-ip 192.168.3.232 --port 8765 \
  --control-mode impedance --impedance-profile spacemouse \
  --impedance-joint-max-speed 180 --operator-frame face-to-face \
  --no-gripper --enable-motion
```

启动显示 `JOINT-LIMITED joint_max_speed=180 deg/s`。每轴 IK 参考变化率最多 π rad/s，
长时间停顿只允许最多 20ms 的参考推进量；再用 FK 计算阻抗平衡点。
这一档不使用固定末端 0.15m/s 的整形上限，但需要额外 FK RPC，实际更新率受往返耗时影响。
它限制的是关节**参考轨迹**，不保证 CST 阻抗下实测关节速度永远小于 180°/s。
柜端软限制仍取实际配置（max_velocity × velocity_percentage），3311/3312 等保护不被修改；
若仍触发，需结合实测速度、跟随误差和阻抗参数排查，不能直接提高保护阈值。
仅更新主机 CLI 与库，控制柜无需再次部署；目前完成离线验证，尚未实机确认。


### 松开 Grip 时的阻尼停稳修复（本地主机版）

使用 `build_impedance_release/bin/app_peripherals_pico_bridge`，参数沿用关节参考限速 180°/s。只更新开发机上的配套程序和库，不需要重新部署控制柜。
松手后由控制柜继续阻尼减速；遇到 4311 停稳超时最多三轮退出调用（每轮后端等待 2 秒），期间不能重新进入遥操。其他退出错误或持续无法停稳仍急停。
该修复针对 4311 被当作立即急停的情况；3311/3312 等实际关节速度保护仍有效，不能视为已解决实机超速。
