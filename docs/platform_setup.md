# 平台配置

[English](platform_setup.en.md) · **简体中文**

## 支持平台

V1 standalone 构建只支持 Linux。Windows 下载和构建路径会直接报错。

## SpaceMouse

运行进程的用户必须能读取所选 `/dev/input/event*` 设备节点。可通过 udev 规则配置，或将用户加入对该设备有读权限的组。

常用命令：

```bash
ls -l /dev/input/event*
sudo usermod -aG input "$USER"
```

重新登录后组权限才会生效。`--device` 可指定 event 节点；省略时使用源码内的默认发现路径。`--sample-rate` 设置输出频率，解码 sample 默认 125 Hz。

## F/T 传感器

串口六维力传感器需要用户对串口设备具备读写权限，例如 `/dev/ttyUSB0`。示例默认传感器类型为 `xjc_serial`、波特率 `460800`，可通过 CLI 覆盖。

常用命令：

```bash
ls -l /dev/ttyUSB*
sudo usermod -aG dialout "$USER"
```

`app_peripherals_probe` 与 `app_peripherals_read_ft_sensor` 支持 `--serial-port`。读数和 bridge app 会调用 `WaitForFirstSample()`；若首帧有效数据超时，会打印 frame error 信息并退出或继续按对应 app 逻辑处理。

## 机器人连接

`app_peripherals_bridge` 需要可连通的机器人控制器和匹配版本的 `smrcore_sdk`。standalone 构建时先执行：

```bash
sudo apt-get install libxcb1-dev pkg-config
./scripts/download.sh
./scripts/build.sh --with-sdk ON
```

在启用遥操作或 FDCC 之前启动 bridge，确保控制器能持续收到外设采样：

```bash
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip>
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> --spacemouse
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> --ft-sensor
```
