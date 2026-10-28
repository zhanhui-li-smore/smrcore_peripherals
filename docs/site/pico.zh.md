# Pico 读取与遥操

## 两条链路

| 用途 | 程序 | 机器人 SDK |
| --- | --- | --- |
| 查看头显、双手柄位置和按键 | C++ `app_peripherals_read_pico` 或 Python `app_peripherals_read_pico.py` | 不需要 |
| B3 双臂遥操及 DH 夹爪 | C++ `app_peripherals_pico_bridge` | 需要兼容双臂 SDK |

SpaceMouse bridge 上传采样给柜端消费；Pico bridge 在应用层将手柄位姿映射为各臂目标，再调用 SDK。两者都保持读取层与控制层分离，但不是同一个柜端输入接口。

## 1. 安装与只读验证

```bash
./scripts/build.sh --with-sdk OFF --tests ON
./build_Release/bin/app_peripherals_read_pico --port 8765
```

或者安装 Python wheel 后运行：

```bash
python python/app/app_peripherals_read_pico.py --port 8765
```

两个读取器二选一，不能同时占用同一个端口。USB 连接并授权 adb 后，在另一终端执行：

```bash
adb devices
adb reverse tcp:8765 tcp:8765
```

在头显浏览器打开 `http://localhost:8765/`，进入 VR 并确认手柄正在追踪。

输出包含 `seq`、`ref`、头显及左右手柄 `xyz[m]`、`grip` 和 `trigger`。位置属于 WebXR 参考空间，单位米、保留四位小数，**不是机械臂 Base/TCP 坐标**。未追踪时为 `N/A`。没有新样本时 `GetSample()` 返回空，不应当作新的有效位置。

普通 Wi-Fi 中 localhost 指头显自身；需使用头显能访问且受信任的 HTTPS 入口，同时转发 WebSocket。不能直接把上述地址改成电脑 HTTP IP 就假定 WebXR 可用。上述命令使用 adb 转发。

## 2. 编译双臂程序

```bash
./scripts/build.sh --with-sdk ON --with-pico ON \
  --sdk-root /path/to/compatible-sdk
./build_Release/bin/app_peripherals_pico_bridge --help
```

SDK 需要 `sdk/dual_arm_robot.hpp` 及匹配动态库。发布使用的 SDK 0.2.0 包不包含该头文件，因此预编译包不包含 Pico 遥操程序。请使用提供此接口及匹配运行库的 SDK 从源码构建。

示例**内置 B3 支架变换**，只适用于对应安装构型，没有通用支架配置参数。连接使用 `DualArmRobot::Initialize(robot_ip)`，当前 CLI 没有 domain/interface 覆盖参数。

## 3. 操作方式与启动

下述命令会进入实机控制流程，不是只读命令。先完成只读验证，确认两臂、安装构型、工具和工作区适配，并保持急停可触达。

```bash
./build_Release/bin/app_peripherals_pico_bridge \
  --enable-motion --robot-ip <robot-ip> --port 8765
```

先停止只读程序，避免端口冲突，再启动 bridge 并连接头显。

| 输入 | 行为 |
| --- | --- |
| 按住对应手柄 Grip | 离合后按手柄增量控制对应机械臂 |
| 松开 Grip | 取消旧目标，执行规划的 ServoJ 停止流程 |
| Grip 按住时摇杆 Y | ServoJ 控制对应臂 J6；回中后从实测 TCP 重新接入 ServoP |
| Trigger 按下沿 | 切换对应 DH 夹爪开/闭；按下阈值 0.75，释放阈值 0.25 |
| Ctrl+C | 请求退出并执行停止清理 |

**启动会初始化两侧 DH 夹爪、写入力度/速度并请求打开。** 夹爪命令成功仅说明写入被接受，当前不回读实际位置或夹持状态。

## 4. CLI 参数

| 参数 | 默认值 / 范围 | 作用 |
| --- | --- | --- |
| `--enable-motion` | 必须显式指定 | 确认进入实机控制 |
| `--robot-ip` | 目标柜 IP | 双臂 SDK 连接地址 |
| `--port` | 8765；1–65535 | WebXR 服务端口 |
| `--translation-only` | 默认关闭 | 仅平移，不映射旋转 |
| `--room-fixed` | 默认关闭 | 使用房间固定方向，替代头部 yaw 离合方向 |
| `--operator-frame` | same-side / face-to-face；默认 same-side | 操作者朝向映射 |
| `--control-mode` | servop；可选 fdcc / impedance（仅单臂） | 遥操控制模式 |
| `--single-mapping-yaw` | 0°；−180°～180°；仅单臂 | 映射增量绕 base Z 偏转；从 +Z 看向原点，−90° 为顺时针 |
| `--translation-scale` | 0.8；0.1–3.0 | 平移映射比例 |
| `--rotation-scale` | 1.0；0.1–3.0 | 旋转映射比例 |
| `--motion-scale` | 不指定时保留上述默认值 | 同时设置平移与旋转比例，0.1–3.0 |
| `--j6-speed` | 0.8 rad/s；0.1–3.0 | 摇杆 J6 请求速度，仍受柜端限速约束 |
| `--gripper-force` | 80；20–100 | DH 夹爪力度参数，不是牛顿 |
| `--gripper-speed` | 100；1–100 | DH 夹爪速度参数 |

参数按命令行顺序解析；`--motion-scale` 会覆盖此前的两项比例，后续单项参数可再次覆盖。

## 5. 暂停、恢复及限制

接近奇异区时，条件数 30–60 使用二次减速；离开时使用加倍门槛。短暂 Jacobian 超时会冻结并制动受影响的单臂；Pico 输入超过 250 ms 未更新时会冻结并制动双臂，同时取消未发送夹爪目标，输入恢复后从实测 TCP 重新离合。机器人状态过期、控制器故障和检查基础设施故障仍会停止会话。

候选目标检查尚未完成或不通过时保持最后获批目标，不能理解为强行越过限位。J6 不使用笛卡尔奇异点减速比例，但仍受关节控制约束。

Python 启动入口复用同一 C++ bridge。运动示例需在目标构型上完成现场验证。

## Python 启动 Pico 双臂遥操

Python 入口复用 C++ SDK bridge；B3 支架变换、Grip 离合、J6 摇杆、Trigger 夹爪、奇异位形减速和断流停止与 C++ 程序一致。无需安装外设 wheel 或机器人 Python SDK。

先用配套的双臂 C++ SDK 构建：

```bash
./scripts/build.sh --with-sdk ON --with-pico ON --sdk-root /path/to/compatible-sdk
python3 python/app/app_peripherals_pico_bridge.py --help
```

确认机械臂工作空间和急停可用后启动：

```bash
python3 python/app/app_peripherals_pico_bridge.py \
  --robot-ip 192.168.3.232 --port 8765 --enable-motion \
  --gripper-force 80 --gripper-speed 100
```

使用已安装的程序时增加 `--bridge-executable /path/to/app_peripherals_pico_bridge`。
默认查找仓库 `build_Release/bin`、`build_Release/install/bin`，然后查找 PATH。
其他参数原样交给 C++ bridge 校验；不会自动添加 `--enable-motion`。
Python 进程直接替换为 C++ bridge，Ctrl+C 和退出码沿用原程序，不额外留下后台遥操进程。
不要同时运行 Pico 只读程序占用同一端口。Pico 网络连接步骤见 Pico 指南。

## 独立 i3 单臂：Pico 右手柄

在同一 bridge 中增加 `--single-arm`，连接 SDK 的 `Single` 实例。
物理右手柄控制单臂，不需要左手柄就绪；以机器人 `base` 为参考系，
不使用 B3 支架变换。`--operator-frame face-to-face` 只改变映射方向，
单臂模式仍选择物理右手柄。

需要修正基座与操作者方向时，加 `--single-mapping-yaw -90`，将平移和旋转
增量一起顺时针偏转 90°，初始 TCP 保持不动。此参数不修改柜子的坐标树或
URDF，也不改变每次 Grip 对齐头显朝向的行为；恢复原映射使用值 `0`。

```bash
./build_Release/bin/app_peripherals_pico_bridge \
  --single-arm --robot-ip <单臂IP> --enable-motion --no-gripper
```

未安装 DH 夹爪时使用 `--no-gripper`；有夹爪时去掉该参数，
右 Trigger 按下沿切换开合。右 Grip 控制离合，按住 Grip 时摇杆 Y 控制 J6。
松开 Grip、右手柄断流或丢失跟踪时制动；重连后从当前 TCP 重新离合。
保留奇异位形减速、目标预检、状态过期/锁存错误停止及退出下使能。
默认使用 ServoP；单臂可通过 `--control-mode fdcc` 或 `--control-mode impedance` 选择力控模式。默认不加 `--single-arm` 时仍运行双臂 ServoP。

Python 启动入口原样透传参数：

```bash
python3 python/app/app_peripherals_pico_bridge.py \
  --single-arm --robot-ip <单臂IP> --enable-motion --no-gripper
```

### 单臂 FDCC 遥操

在单臂命令中增加 `--control-mode fdcc`：

```bash
./build_Release/bin/app_peripherals_pico_bridge \
  --single-arm --robot-ip <单臂IP> --enable-motion --no-gripper \
  --control-mode fdcc --single-mapping-yaw -90
```

右手柄 Grip 握紧后，程序等待 FDCC 模式反馈再发送位姿；显示
`FDCC active: mode=6` 表示已进入。松开 Grip 退出 FDCC 并持位，保持连接和
使能；重新握紧以当前 TCP 为新起点继续。Ctrl+C 退出力控并请求下使能。
FDCC 不使用摇杆 J6；方向、旋转比例和夹爪选项沿用原入口。
`--single-mapping-yaw -90` 仅在需要绕基座 Z 顺时针修正 90° 时指定，默认是 0。

FDCC 已内置跟随配置，日常无需填写速度前馈、平滑增益等工程参数。
需要手柄与机械臂位移 1:1 时加 `--motion-scale 1.0`。

<details markdown="1">
<summary>工程调试与诊断（可选）</summary>

FDCC 使用 SDK 默认参数（平移刚度 1000 N/m、kp=0.0005，转动刚度
80 N·m/rad、kp=0.005，逐关节跟踪保护 0.1 rad），沿用柜子的力源配置。
目标力设为零不代表接触力恒为零；位置误差仍产生弹簧恢复力。
目标最多以 50 Hz 发送。默认平移限速 0.30 m/s、加速度 1.2 m/s²，
入口跟随增益 8/s；转动仍限制为 0.8 rad/s、3 rad/s²。
手柄映射后的速度超过平移上限时会积累跟随误差，`--motion-scale 1.0`
只代表位移映射 1:1，并不解除此上限。这些参数约束发送目标，不保证实际运动速度。

下列参数已有默认值，仅调试时覆盖。完整帮助：

```bash
./build_Release/bin/app_peripherals_pico_bridge --help-advanced
```

| 参数 | 默认 | 范围 / 单位 |
| --- | --- | --- |
| `--fdcc-max-speed` | 0.30 | 0.01–0.5 m/s |
| `--fdcc-max-accel` | 1.2 | 0.05–2 m/s² |
| `--fdcc-follow-gain` | 8 | 1–40 /s，入口平滑增益，不是控制器 kp/kd |
| `--fdcc-target-scale` | 1 | 1–3，1 关闭虚拟目标补偿 |
| `--fdcc-target-feedforward` | 0.5 | 0–1，目标速度前馈比例，0 恢复原平滑方式 |

速度前馈依据新获批目标的位移/转动与检查完成时间间隔，经过 40 ms 滤波后
加入入口平滑器，仍受原线/角速度和加速度限制。重复批准帧不会产生新速度，
断续更新与重新握紧时会消退或重置前馈。它不修改 FDCC 的刚度、kp/kd，
也不放大目标位置；跟随更快仍可能增加突然停手的制动超调和接触峰值，
应比较实际跟随误差与接触力后选择比例，而不是直接用满量 1。

FDCC 沿用实测 Jacobian 的渐进减速，不再因候选目标条件数超过 60 而暂停。
实测姿态接近奇异时仍减慢靠近方向，允许退离；有限值、IK 可达性、
关节间隙、FK 一致性与柜端检查继续生效。阻抗使用同一策略，ServoP 保持原行为。

可另加 `--fdcc-target-scale 2` 试验平移虚拟目标：
`virtual = measured + scale * (raw - measured)`。XYZ 误差在 15 mm 内时
恢复原始目标，15–30 mm 间平滑加入补偿；额外前推最多 30 mm。
姿态保持原目标，补偿后的位姿先做 IK/FK/关节间隙预检，再限速发送。
到位后继续遥操，不退出或重复发送 RPC。此补偿可能增大接触力，
应先在自由空间比较跟随效果；它不改变 FDCC 刚度或保护阈值。

每秒的 `FDCC` 输出中，`raw_err_mm` 是原始目标与实测位置差，
`lead_mm` 是虚拟目标额外前推量；`shape_gap_mm` 是批准目标到发送目标的距离，
`follow_err_mm` 是发送目标与发送前实测位置的距离，`v_target` 是目标速度。
`shape_deg`、`follow_deg` 对应转动整形/跟随误差，`w_target` 是目标角速度。
`shape_gap_mm` 持续较大说明入口整形仍在限速/平滑；
它较小而 `follow_err_mm` 较大时，应进一步检查 FDCC 与驱动器跟随。
`guard_age_ms` 是检查完成后经过的时间，`rpc_ms` 是最近目标 RPC 耗时，
均不是物理运动延迟测量。持位时这些值保留最后一次发送快照。
`check=WAIT` 表示等待新的检查结果；`ik_ms`、`fk_ms`、`check_queue_ms`
分别记录最近完成候选的 IK、FK 和提交至开始检查的耗时。

</details>

候选检查暂时变慢时保持当前力控模式。检查完成后超过 200 ms 没有新批准结果，
发送线程冻结最后发送的平衡点；新检查通过后自动继续，无需重新握 Grip。
这一等待不会发送未经检查的新目标，也不会延长手柄输入的存活时间。
输入/Jacobian 断流或候选明确被拒绝仍退出力控持位，需松开再握紧；
此时迟到的结果不能自动恢复运动。此规则也适用于下述阻抗模式。
机器人反馈过期、控制器故障、模式意外退出或 RPC 失败会结束会话。
单次目标写入 RPC 耗时超过 200 ms 仍按通信异常结束会话。
RPC 在独立线程中执行，不阻塞手柄读取；已发出的请求需等待 SDK 返回或超时后完成清理。
需要 SDK 和柜子同时支持 FDCC 接口；本次用 SDK 0.2.0 编译。更新时同时替换
可执行文件与 `libsmrcore_pico_teleop.so`。

### 单臂笛卡尔阻抗遥操

使用 `--control-mode impedance`，右手柄位姿映射为末端平衡点，
控制柜在笛卡尔阻抗模式（`mode=5`）下执行 CST 力矩控制：

```bash
./build_Release/bin/app_peripherals_pico_bridge \
  --single-arm --robot-ip <单臂IP> --control-mode impedance \
  --motion-scale 1.0 --enable-motion --no-gripper
```

方向映射与 ServoP/FDCC 一致，需要基座方向修正时仍可加
`--single-mapping-yaw -90`。Python 启动入口也支持这些参数。

阻抗与 FDCC 一样按实测 Jacobian 渐进减速，不再因候选 IK 终点的条件数
达到 60 而暂停。候选目标仍检查有限值、IK 可达性、关节间隙和 FK 一致性，
省去计算候选 Jacobian 的额外六次 FK 调用；检查暂慢时冻结最后平衡点，
保留阻抗模式，新结果通过后自动续跟。

| 参数 | 默认值 | 单位 / 范围 |
| --- | --- | --- |
| `--impedance-stiffness` | `120,120,120,20,20,20` | XYZ：0–2000 N/m；RX/RY/RZ：0–150 N·m/rad |
| `--impedance-damping` | `20,20,20,1,1,1` | XYZ：0–200 N·s/m；RX/RY/RZ：0–60 N·m·s/rad |
| `--impedance-max-speed` | `0.15` | 0.01–0.5 m/s，平衡点速度上限 |
| `--impedance-max-accel` | `0.6` | 0.05–2 m/s²，平衡点加速度上限 |
| `--impedance-follow-gain` | `8` | 1–40 /s，入口整形增益 |

K/D 参数必须是六个逗号分隔的数，顺序为 X、Y、Z、RX、RY、RZ。
默认沿用 SDK 的 hold 基线，其他阻抗参数保留 SDK 默认值：任务空间阻尼
开启，位置/姿态误差饱和分别为 0.03 m、0.25 rad，前馈 wrench 为零。
工程范围表示参数合法区间，不代表适用于所有负载的调参范围。

握紧右 Grip 并移动后，显示 `Impedance active: mode=5` 才表示模式已确认。
首个平衡点从实测 TCP 开始，再按限速/加速度接近手柄目标。
松开 Grip 由后端阻尼停稳、退出阻抗并回到持位；等待
`Impedance paused: mode=0` 后可以重新握紧继续。松手不退出程序。
Ctrl+C 退出阻抗并请求、回读下使能。阻抗模式不使用摇杆 J6，
夹爪 Trigger 和其他坐标映射选项继续可用。

目标更新走 `SetCartesianImpedanceTarget`，DDS 发送上限 200 Hz；
模式切换走 RPC。`send_ms` 只是目标写入 SDK 的耗时，不是送达确认或
实际跟随延迟；误差字段与 FDCC 的含义一致。输入限速参数不等于刚度/阻尼，
提高发送频率也不保证机械臂跟随更快。阻抗模式不使用 FDCC 虚拟目标放大。

进入时要求机械臂停稳、控制柜具备 CST 和有效动力学模型；模型与负载
会影响力矩补偿。接触时平衡点误差仍会产生弹簧力，前馈 wrench 为零
并不表示接触力为零。输入/Jacobian 过期或候选受阻时退出阻抗持位；
模式丢失、锁存故障、SDK 命令源丢失或状态过期时结束会话。
不自动清错，也不切换成位置遥操继续执行。

官方 SDK 0.2.0 编译和离线生命周期测试已覆盖此入口，实机跟随、接触和
CST 切换效果需在对应柜子验证。更新时同时替换可执行文件与遥操共享库。
