# C++ 应用



本目录包含外设调试与 SDK 桥接可执行工具，使用 `smrcore::peripherals` C++ API。

平台权限与设备节点配置见 [平台配置](https://github.com/zhanhui-li-smore/smrcore_peripherals/blob/17ea56f/docs/platform_setup.md)。

## 构建

在仓库根目录执行：

```bash
# 不需要机器人 SDK，只构建外设库、读数 app 和测试
./scripts/build.sh --with-sdk OFF --tests ON

# 需要机器人 bridge / F/T 标定时（仅此时编译 SDK 相关目标）
./scripts/download.sh
./scripts/build.sh --with-sdk ON --tests ON
```

可执行文件位于 `build_Release/bin/`：

| 可执行文件 | 说明 |
|---|---|
| `app_peripherals_probe` | 探测 SpaceMouse 与 F/T 传感器型号、采样率与连接状态 |
| `app_peripherals_read_spacemouse` | 解码后的 SpaceMouse 6-DOF + 夹爪目标采样 |
| `app_peripherals_read_spacemouse_raw` | 原始 Linux input 轴/按键采样 |
| `app_peripherals_read_ft_sensor` | 串口六维力矩流，含 `WaitForFirstSample` 与完整串口 CLI |
| `app_peripherals_read_pico` | PICO WebXR 头显/双手柄读数，不连接机器人 |
| `app_peripherals_bridge` | 通过 `robot.Peripheral().Update*Sample()` 推送 SpaceMouse 和/或 F/T 数据（需 `--with-sdk ON`） |
| `app_peripherals_keyboard_bridge` | 终端键盘笛卡尔 / 关节遥操，单臂 / 双臂同时控制（`--with-sdk ON`） |
| `app_peripherals_ft_sensor_calib` | MoveJ 静态六维力标定（需 `--with-sdk ON`；先跑 bridge 注入 raw） |
| `app_peripherals_pico_bridge` | PICO 双手柄到双臂 SDK ServoP；按住 Grip 后 thumbstick Y 轴直接控制对应手臂 J6（ServoJ，默认 `0.8 rad/s`，可用 `--j6-speed` 调整）；Trigger 按 0/1 映射左右夹爪开/关；需 `--with-sdk ON --with-pico ON` 和兼容双臂 SDK |

## 仅读外设

```bash
./build_Release/bin/app_peripherals_probe
./build_Release/bin/app_peripherals_probe --serial-port /dev/ttyUSB0

./build_Release/bin/app_peripherals_read_spacemouse --device /dev/input/event5 --sample-rate 125
./build_Release/bin/app_peripherals_read_spacemouse_raw --device /dev/input/event5
./build_Release/bin/app_peripherals_read_ft_sensor \
  --serial-port /dev/ttyUSB0 --sensor-type xjc_serial --baud-rate 460800

# PICO 读数不需要机器人 IP
./build_Release/bin/app_peripherals_read_pico --port 8765
```

## 机器人 SDK 桥接

`app_peripherals_bridge` 需要 `smrcore_sdk`、可连通的机器人，以及本机已连接的外设。

```bash
# 默认同时启用 SpaceMouse 和 F/T，适合 FDCC 双外设输入
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip>

# 仅遥操作或仅力传感器
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> --spacemouse
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> --ft-sensor

# 覆盖设备参数
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> \
  --spacemouse-device /dev/input/event5 \
  --ft-serial-port /dev/ttyUSB0 \
  --ft-sensor-type xjc_serial
```

`--robot-ip <robot-ip>` 可省略；省略时向 SDK 传入空 IP，由 SDK 使用默认连接行为。bridge 应在启用遥操作或 FDCC 之前启动，确保控制器能持续收到外设采样。

## 六维力静态标定

> **安全：** 将外置六维力用于力控（例如
> [smrcore_sdk](https://github.com/smore-robotics/smrcore_sdk) 的 FDCC
> `--wrench-source ft-sensor`）前，**必须先完成一次静态标定并 `--save`**。
> 未标定的外力无法准确反映真实接触力，可能导致大幅非预期运动，十分危险。
> 标定只需成功保存一次；之后日常运行只需 bridge 持续向 SDK/控制器推送采样。

先启动 bridge 注入 `ft_sensor_state`，再运行标定：

```bash
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> --ft-sensor
./build_Release/bin/app_peripherals_ft_sensor_calib --robot-ip <robot-ip>
# 确认 Preview 合格后再持久化：
./build_Release/bin/app_peripherals_ft_sensor_calib --robot-ip <robot-ip> --save
```

标定保存后，日常与 SDK FDCC 联调：

```bash
# 1) 持续推送外置力采样（以及可选 SpaceMouse）
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> --ft-sensor
# 或默认双外设：
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip>

# 2) 在 smrcore_sdk 中启用外置力源 / SpaceMouse
./compliance_fd_cartesian_admittance --robot-ip <ip> --wrench-source ft-sensor
./compliance_fd_cartesian_admittance --robot-ip <ip> --mode spacemouse
```

SpaceMouse 遥操同样**不会**由 SDK 示例直接打开 `/dev/input/*`，必须由本仓库
`app_peripherals_bridge --spacemouse`（或默认双外设）通过
`Robot::Peripheral().UpdateSpaceMouseSample()` 注入。


## Pico

[Pico 读取与双臂遥操完整说明](pico.md)。

## 键盘

[单臂 / 双臂同时控制、按键及权限说明](keyboard.md)。
