<!-- release-category: pico -->

## 中文

- 双臂 PICO 启动时分别显示 SDK 初始化、命令源获取、参考系解析和使能阶段，便于区分头显连接与机器人 RPC 等待。初始化和使能前预检返回后检查退出请求，避免已按 Ctrl+C 仍继续进入使能步骤；RPC 超时仍由配套 SDK 提供。
- 记录 SDK 0.2.0 对 RCore `0.2.1-alpha.20261016` 的配置读取兼容性问题；本机按后端 IDL 重建 SDK 后，左右臂只读配置查询通过，未验证机器人运动。

## English

- Report SDK initialization, command-source acquisition, reference setup and enable as separate dual-arm PICO startup stages. Check cancellation after initialization and pre-enable checks so a pending Ctrl+C does not proceed into enable; RPC timeouts still depend on the matching SDK.
- Document the SDK 0.2.0 configuration-query incompatibility with RCore `0.2.1-alpha.20261016`. A local SDK build using the backend IDL passed read-only config queries on both arms; robot motion was not tested.
