<!-- release-category: pico -->

## 中文

- 双臂 PICO 阻抗/FDCC 在暂停触发及每秒状态中增加输入健康诊断，记录连接、按机械臂映射的手柄追踪状态、输入年龄、主循环和已观察到的接收/头显时间间隔，辅助区分追踪丢失、上游输入停顿及主循环卡顿。最新样本槽的序号跳跃不等同于网络丢包；保留 400ms 超时及恢复保护。

## English

- Add dual-arm PICO impedance/FDCC input-health snapshots on pause transitions and in periodic status: connection, arm-mapped controller tracking, input age, main-loop and observed receive/headset gaps. Latest-slot sequence jumps do not prove network packet loss. The 400ms input timeout and recovery protections are retained.
