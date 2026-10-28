> SRC: pico_compliance.hpp, pico_fdcc.hpp, pico_impedance.hpp
> REF: fdcc_target_limiter.hpp
> REF: ../src/peripherals/pico/teleop.md

# 简介

Pico 单臂柔顺模式的非 RT worker；最新目标槽，无轨迹积压。

# 关键成员

- PicoCompliance: generation 取消未发送目标；进入/退出均等模式反馈。
  首帧从实测 TCP 开始；调用完成才推进整形状态。
  UpdateInputDeadline 使用原始收包时间；不因重复读包延长存活期。
  输入过期记录 deadline 超出时间；主循环暂停日志记录连接、跟踪、
  当前包龄、序号和循环间隔，区分断流与消费线程停顿。
  主循环分开续约输入与批准目标；批准有效期从检查完成起算。
  批准过期且输入新鲜时冻结最后发送平衡点、清整形速度，保留模式；
  新结果自动恢复。没有独立输入期限的直接调用仍按批准期限停止。
  真正断流/候选拒绝后 rearm_required 阻止迟到批准重启；
  新鲜松 Grip 且 mode=0 确认后才能清除。
  陈旧反馈、模式丢失、写入异常仍结束会话；退出失败尝试 StopMotion/Disable。
  SDK 已开始的调用不能取消，停止会等待返回。
- PicoFdcc: mode 6，50Hz RPC，Manual、零 wrench、SDK 默认参数。
  入口默认 0.30m/s、1.2m/s²、gain 8/s、半量速度前馈；0 恢复原平滑器。
- PicoImpedance: mode 5，200Hz 上限，Manual、零前馈 wrench；
  仅调用 SetCartesianImpedanceTarget，检查 SDK command_source。
  K/D 在 SDK 工程范围内校验，其他 DTO 默认保持；退出由后端阻尼停稳
  并回 CSP，期间拒绝重新激活。无 FDCC/ServoP/ServoJ 回退。
- Status: 同拍 raw/lead/shape/follow 误差、转动 shape/follow 误差与线/角速度；
  waiting_for_check 区分等检查与模式退出；call_ms 是调用耗时，
  阻抗 DDS 无送达确认；Holding 保留最后写入快照，非物理延迟测量。
