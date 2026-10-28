Pico 遥操增加独立单臂右手柄入口、base Z 映射角及可选夹爪控制，支持
ServoP、FDCC 和笛卡尔阻抗模式；双臂仍默认使用原 ServoP 链路。
配套机器人 SDK 版本为 0.2.0，需显式启用 Pico bridge 构建，并使用支持
对应控制接口的柜端程序。

新增有限值与目标预检、受速度/加速度约束的目标整形和分阶段诊断。
FDCC/阻抗检查暂慢时冻结最后发送的平衡点，保留控制模式；新检查通过后
自动续跟。真实输入断流、故障和候选拒绝保留停止处理；松开 Grip 后
退出柔顺模式持位，再次握紧从实测 TCP 重新对齐。阻抗实机跟随表现仍需
按具体构型验证，离线测试不代表接触力或实机跟随性能验收。

中英文用法和 AI 索引同步更新；CI 纳入单臂映射、无效输入及柔顺模式
离线回归，并修复 pytest 的测试包导入。

来源：`teleop/`、`app/app_peripherals_pico_bridge.cpp`、
`src/peripherals/pico/teleop.hpp`、`unittests/`、`.github/workflows/ci.yml`。
