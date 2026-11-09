# Smartmore 外设 SDK

## 文档导航

- [快速开始](getting-started.md)
- [版本说明](release-notes.md)
- [C++ 示例](cpp.md)
- [Python 示例](python.md)
- [API 参考](api.md)

接入 **SpaceMouse、六维力/力矩传感器、Pico、主从臂和键盘**，构建机器人输入应用。

本仓库提供 C++ 源码库、Python 外设读取 wheel，以及可选的机器人 SDK 桥接示例。

| 你想做什么 | 从这里开始 |
| --- | --- |
| 安装和编译 | [快速开始](getting-started.md) |
| 查看位置、按键或力数据 | [外设读取](readers.md) |
| 将外设接入机器人 | [机器人接入](robot-integration.md) |
| 下载编译包和说明书 | [下载与版本](downloads.md) |

## 两种使用方式

**只读外设：** 外设 → 外设库 → 你的 C++ / Python 应用。无需机器人 SDK。

**机器人接入：** 外设 → 独立 bridge → Smartmore Robot SDK → 控制柜。

Pico 双臂遥操示例内置 B3 支架变换，仅适用于对应安装构型；当前默认下载包不包含此程序。

[机器人 SDK 文档](https://smore-robotics.github.io/smrcore_sdk/zh/getting-started/) · [源码仓库](https://github.com/smore-robotics/smrcore_peripherals)

## 外设接入

| 外设 | 输入 | 机器人接入 |
|---|---|---|
| [SpaceMouse](spacemouse.md) | 六自由度输入与按键 | SDK Peripheral 样本接口 |
| [六维力传感器](ft-sensor.md) | 力与力矩采样 | SDK Peripheral 样本接口 |
| [Pico](pico.md) | 头显、手柄位姿与按键 | 双臂 SDK ServoP 与末端夹爪 |
| [主从臂](zhonglin.md) | 主臂关节位置 | SDK ServoJ 从臂跟随 |
| [键盘](keyboard.md) | 终端键盘输入 | SDK ServoRelativeP / ServoJ，单臂 / 双臂同时控制 |

统一按“读取输入 → 机器人接入 → 配置与适用范围”使用；各链路的协议和控制接口按实际实现说明。
