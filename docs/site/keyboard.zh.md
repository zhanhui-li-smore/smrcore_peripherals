# 键盘遥操

在当前终端中控制独立单臂，或用两组按键同时控制双臂，支持笛卡尔和关节空间。**不新建窗口，不需要 `/dev/input`、sudo 或键盘设备权限。** 本地 X11 桌面自动读取真实按下 / 松开状态，实现连续控制；纯 SSH、Wayland 或 `--terminal` 使用有限步长点动。

## 构建与试用

```bash
# Ubuntu：SDK 工具构建依赖
sudo apt-get install libxcb1-dev pkg-config
./scripts/download.sh
./scripts/build.sh --with-sdk ON --with-pico OFF --tests OFF

# 不连接机器人，检查按键和空间切换
./build_Release/bin/app_peripherals_keyboard_bridge --dry-run --dual-arm
```

SDK bridge 制品包含该程序。使用解压后的制品时，将 `./build_Release/bin/` 换成 `./run.sh `，例如 `./run.sh app_peripherals_keyboard_bridge --dry-run`。带本地连续按键支持的程序需要系统库 `libxcb1`；构建时缺少 XCB 则仅保留终端点动。

## 连接机器人

在 HMI 选择 SDK 命令源、位置控制模式及 `base` 参考系，确保机械臂已停稳。检查运动空间、工具配置并准备好急停，然后运行：

```bash
# 独立单臂
./build_Release/bin/app_peripherals_keyboard_bridge \
  --robot-ip ROBOT_IP --enable-motion

# 双臂
./build_Release/bin/app_peripherals_keyboard_bridge \
  --robot-ip ROBOT_IP --dual-arm --enable-motion
```

将 `ROBOT_IP` 替换成柜子 IP。本地模拟器已启动时，用 `--local` 替代 `--robot-ip ROBOT_IP`，SDK 的 DDS domain 需与模拟器一致。无图形桌面的 SSH 会话加 `--terminal`，并使用带终端的连接（`ssh -t`）。

## 操作

保持运行程序的终端在前台。启动时打印完整按键表，状态行显示当前空间与启用状态。本地连续模式会打印 `Input: continuous held keys in this terminal (local X11)`。

- **空格按一下：启用方向键输入；再按一下：暂停两臂。** 无需一直按住。空格本身不会生成位移。
- **Tab：在笛卡尔 / 关节空间间切换。** 双臂一起切换，停止增加旧目标，等两臂停稳后以当前位置建立新目标。切换后保持暂停，再按空格继续。
- **Ctrl+C 或 Esc：退出并下使能。** 退出时恢复终端设置。
- 状态行显示 `space=cartesian` / `space=joint`、`ENABLED` / `PAUSED`。终端失焦会暂停两臂，返回后松开并重新按一下空格继续。

| 笛卡尔方向 / 关节（正 / 负） | 单臂 / 左臂 | 右臂 |
|---|---|---|
| X / J1 | W / S | ↑ / ↓ |
| Y / J2 | A / D | ← / → |
| Z / J3 | R / F | PageUp / PageDown |
| 绕 X / J4 | Z / X | I / K |
| 绕 Y / J5 | C / V | J / L |
| 绕 Z / J6 | Q / E | U / O |

**按住方向键连续移动，松开后停止增加目标。** 支持组合键、斜向运动和双臂同时按键，不依赖操作系统的按键重复。终端输入和输出与机器人发送循环独立；不会因终端刷屏等待而打断目标生成。

### 终端回退

```bash
./build_Release/bin/app_peripherals_keyboard_bridge --terminal --dry-run --dual-arm
```

终端模式每次方向键输入产生平移 **1 mm**、姿态或关节 **0.5°** 的有限点动。长按依赖系统重复输入，可能有间隔；普通终端没有松键事件，不能保证同时长按多键。需要连续控制时在本地 X11 桌面的终端运行，不加 `--terminal`。

## 速度与坐标系

- 默认平移速度 **0.03 m/s**，姿态和关节速度 **0.15 rad/s**。`--speed` 的范围为 `(0, 0.1]` m/s，`--angular-speed` 为 `(0, 0.5]` rad/s。笛卡尔斜向输入不会放大合速度。
- 默认 `--frame base`，对应各臂自己的基坐标系；`--frame tool` 使用各自工具坐标系。此选项只影响笛卡尔空间，关节空间直接控制 J1～J6。
- 笛卡尔使用 SDK `Motion().ServoRelativeP()`，关节使用 `Motion().ServoJ()`。柜端负责轨迹规划；关节目标还会按启动时读取的柜端位置限位裁剪。
- 双臂按各自坐标系运动，不包含 B3 支架变换、共同物体约束或硬同步规划。
- 暂停后停止累加目标，柜端按轨迹约束减速。终端失焦、输入更新中断会暂停；终端回退遇到粘贴或输入溢出同样暂停。断连、机器人状态失效或命令源/模式改变时退出两臂。后端限位、碰撞及急停仍有效。

此入口使用位置控制，不切换 FDCC、阻抗或操作夹爪。

退出使用 Servo 持位增量后下使能，并确认 `enabled=false`；不调用只针对 Move 任务的 `StopMotion()`，避免纯 Servo 会话出现 `3218` 的误用提示。
