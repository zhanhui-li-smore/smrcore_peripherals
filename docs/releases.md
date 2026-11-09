# 版本与下载

[下载正式版本](https://github.com/zhanhui-li-smore/smrcore_peripherals/releases) · [查看开发构建](https://github.com/zhanhui-li-smore/smrcore_peripherals/actions)

本仓库提供五类外设接入，统一采用“外设输入 → 应用 → 机器人 SDK”的分层。只读采样不连接机器人；需要运动或夹爪控制时运行对应应用。

## 支持的链路

| 外设 | 输入读取 | 机器人接入 | 交付形式 |
|---|---|---|---|
| SpaceMouse | C++ / Python | SDK Peripheral 样本接口 | C++ 预编译 bridge / Python 示例 |
| 六维力传感器 | C++ / Python | SDK Peripheral 样本接口与静态标定 | C++ 预编译 bridge / Python 示例 |
| Pico | C++ / Python | 双臂 ServoP、末端夹爪 | C++ 源码示例 / Python 启动入口 |
| 主从臂 | Python | ServoJ 从臂跟随 | Python 源码示例 |
| 键盘 | Terminal / dry-run | ServoRelativeP / ServoJ 单臂 / 双臂同时控制 | C++ SDK bridge |

Pico 的 Python 入口启动同一 C++ 程序；主从臂的 Python 示例直接调用机器人 Python SDK。两者都沿用外设读取与机器人控制分离的结构。

## 选择下载包

发布平台：**Ubuntu 22.04 / Linux x86_64**；Python wheel 支持 **CPython 3.10、3.11、3.12**。

| 文件 | 用途与包含内容 |
|---|---|
| `smrcore_peripherals-cpp-linux-x86_64-v<version>.tar.gz` | C++ 外设库、头文件、读取程序和 Python 示例；不带机器人 SDK |
| `smrcore_peripherals-sdk-bridge-linux-x86_64-v<version>.tar.gz` | 上述外设组件，加 SpaceMouse / 六维力 bridge、键盘遥操、标定程序和配套 C++ SDK 运行库 |
| `rcore_peripherals_py-<version>-cp3xx-cp3xx-linux_x86_64.whl` | Python 外设读取库；按解释器选择一个 wheel，不含机器人 SDK |
| `smrcore_peripherals-manual-zh-v<version>.pdf/html` | 中文说明书 |
| `smrcore_peripherals-manual-en-v<version>.pdf/html` | 英文说明书 |
| `SHA256SUMS`、`*.tar.gz.sha256` | 完整下载集校验和、单个压缩包校验和 |

两个 C++ 包按用途选一个，不需要叠加安装。Python wheel 是另一种语言的使用入口；示例脚本位于压缩包或同版本源码的 `python/app/`。GitHub 自动附带的 Source code 文件是源码，不是编译包。当前流水线不生成 Windows、ARM64 或模拟器制品。

## 安装与运行

解压所选 C++ 包并进入解压目录，通过 `run.sh` 加载包内动态库：

```bash
./run.sh app_peripherals_probe
```

Python 以 CPython 3.10 为例，安装对应版本的单个 wheel：

```bash
python3 -m pip install ./rcore_peripherals_py-<version>-cp310-cp310-linux_x86_64.whl
```

主从臂读取还需 `numpy`、`pyserial`（可通过 wheel 的 `zhonglin` extra 安装）。直接调用机器人 SDK 的 Python 示例另装配套机器人 Python SDK。Pico Python 启动入口只需要构建好的 C++ bridge。

下载全部制品后运行 `sha256sum -c SHA256SUMS`；只下载一个压缩包时，下载其 `.sha256` 并使用 `sha256sum -c <包名>.sha256`。设备权限和串口参数见同版本说明书。

## SDK 与构型

- 预编译 SpaceMouse / 六维力 / 键盘 bridge 配套 SDK **0.2.0**，由 `.sdk-version` 固定。压缩包内 `build-info.json` 记录源码提交和 SDK 版本。
- Pico 需要带 `sdk/dual_arm_robot.hpp` 的配套 C++ SDK。当前固定的官方 SDK 包缺少此头文件，因此 **Pico 遥操程序不在预编译包中**，需使用包含此接口的配套 SDK 从源码构建。示例内置 B3 支架变换。
- 主从臂示例保留当前主臂与单臂 i3 的标定映射；具体硬件与标定见主从臂说明。
- 库、头文件与运行库须配套更新。构建检查不代表实机验收；使用运动应用前确认构型、工具、工作区和急停。


键盘程序在当前终端运行，需要系统库 `libxcb1`；本地 X11 桌面支持连续按键，无桌面时自动使用有限步长点动，也可显式加 `--terminal`。

## 版本识别

正式 Release 对应 `vX.Y.Z` tag，版本号同时写入 C++ 包和 wheel。开发制品在 Actions 保留 14 天：压缩包名带提交哈希，wheel 版本为 `0.0.0`，通过运行记录确认提交。

网站随开发分支更新；使用某个版本时，以该 tag 的源码和随附说明书为准。
