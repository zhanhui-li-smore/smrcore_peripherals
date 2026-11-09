<!-- release-category: other -->

## 中文

- 新增终端键盘遥操，支持独立单臂与双臂同时控制，无需设备权限或额外窗口。本地 X11 桌面按住方向键连续移动、松开停止增加目标，空格单击启用/暂停，Tab 切换笛卡尔与关节空间，Ctrl+C 或 Esc 退出；终端失焦暂停。纯 SSH、Wayland 或 `--terminal` 使用有限步长点动。
- 笛卡尔使用 SDK ServoRelativeP，关节使用 ServoJ；切换等待两臂停稳后重新对齐目标，并保持暂停。退出保持 Servo 目标后下使能并确认反馈，不再调用不适用的 Move 任务停止接口。SDK bridge、网站与双语说明书包含该入口和按键说明，支持无机器人 dry-run。
- 本地连续按键构建需安装 `libxcb1-dev`、`pkg-config`，运行需要 `libxcb1`；无该依赖的 Linux 构建仍保留终端点动。

## English

- Added terminal keyboard teleoperation for standalone and simultaneous dual-arm control without device permissions or an extra window. Local X11 desktops support continuous held keys and releases. Tap Space to enable/pause, Tab to switch Cartesian/joint space, and Ctrl+C or Esc to exit; focus loss pauses. SSH, Wayland or `--terminal` use bounded jogs.
- Cartesian input uses SDK ServoRelativeP and joint input uses ServoJ. Switching waits for both arms to settle, reanchors targets and remains paused. Exit holds Servo targets, disables motors and verifies feedback without calling the inapplicable Move-task stop API. SDK bridge builds, the website and bilingual manuals include key mappings and a no-connection dry-run.
- Local continuous-key builds use `libxcb1-dev` and `pkg-config`, with `libxcb1` at runtime. Linux builds without this dependency retain terminal jogs.
