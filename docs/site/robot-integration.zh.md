# 机器人接入

## SpaceMouse / 六维力传感器

该 bridge 将外设样本通过 SDK 的 Peripheral 接口交给机器人；实际运动行为取决于柜端控制模式及配置。

```bash
./scripts/download.sh
./scripts/build.sh --with-sdk ON --with-pico OFF
./build_Release/bin/app_peripherals_bridge --help
```

查看并配置设备参数后，才连接目标柜：

```bash
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip>
```

默认 SDK 版本由仓库 `.sdk-version` 固定。预编译 `sdk-bridge` 包带有配套 SDK 运行库，不需要另外安装读取器包。

## Pico

Pico 读取器属于基础库；Pico 双臂遥操属于独立的可选 C++ 程序。

```bash
./scripts/build.sh --with-sdk ON --with-pico ON \
  --sdk-root /path/to/compatible-sdk
./build_Release/bin/app_peripherals_pico_bridge --help
```

!!! warning "SDK 与构型要求"
    SDK 必须同时包含 `sdk/dual_arm_robot.hpp` 和匹配运行库。此前官方 v0.2.0 原包漏装此头文件，不能直接用于该示例。示例内置 B3 支架变换，仅适用于对应 B3 双臂安装构型，不能直接用于任意双臂或 i12。

当前 CI 默认关闭 Pico 遥操，因此基础构建通过不代表该示例已经通过双臂实机验收。默认发布包不包含 `app_peripherals_pico_bridge`。

夹爪通过各机械臂 SDK 的 EndBoard / DH 接口操作，需要柜端、末端板及夹爪协议配套。完整参数见[源码应用说明](https://github.com/smore-robotics/smrcore_peripherals/blob/main/app/README.md)。

执行运动示例前确认目标、工具、负载与工作空间匹配，急停可触达。

详见 [Pico 完整用法](pico.md)和[平台权限配置](platform.md)。

## 外设接入

| 外设 | 输入 | 机器人接入 |
|---|---|---|
| [SpaceMouse](spacemouse.md) | 六自由度输入与按键 | SDK Peripheral 样本接口 |
| [六维力传感器](ft-sensor.md) | 力与力矩采样 | SDK Peripheral 样本接口 |
| [Pico](pico.md) | 头显、手柄位姿与按键 | 双臂 SDK ServoP 与末端夹爪 |
| [主从臂](zhonglin.md) | 主臂关节位置 | SDK ServoJ 从臂跟随 |
| [键盘](keyboard.md) | 终端键盘输入 | SDK ServoRelativeP / ServoJ，单臂 / 双臂同时控制 |

统一按“读取输入 → 机器人接入 → 配置与适用范围”使用；各链路的协议和控制接口按实际实现说明。
