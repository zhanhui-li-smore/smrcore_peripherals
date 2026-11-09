> SRC: pico_compliance.hpp, pico_fdcc.hpp, pico_impedance.hpp
> REF: fdcc_target_limiter.hpp
> REF: ../src/peripherals/pico/teleop.md

- FDCC 可选 direct_target：首帧仍锚定实测 TCP，之后直接发送最新批准位姿，绕过主机平移/转动整形、限速/限加速度及前馈。批准过期保持最后目标、断流/拒绝暂停和退出保护保留；诊断速度改为相邻下发位姿差分。

# 简介

Pico 单/双臂柔顺模式的非 RT worker；最新目标槽，无轨迹积压。

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
  陈旧反馈、模式丢失、写入异常仍结束会话。
  力控退出 RPC 或 mode=0 确认失败时，立即尝试 EStop 后 Disable；
  急停请求成功不替代调用方最终状态回读，不自动清错或重新进入力控。
  SDK 已开始的调用不能取消，停止会等待返回。
- PicoFdcc: mode 6，50Hz RPC，Manual、零 wrench、SDK 默认参数。
  可覆盖 SDK damping_factor（默认 0.9，有限 [0,1]）和 error_scale
  （默认 1.0，本地测试入口有限 [0.1,4]）；其余控制参数保持 SDK 默认。
  入口默认 0.30m/s、1.2m/s²、gain 8/s、半量速度前馈；0 恢复原平滑器。
- PicoImpedance: mode 5，200Hz 上限，Manual、零前馈 wrench；
  仅调用 SetCartesianImpedanceTarget，检查 SDK command_source。
  K/D 在 SDK 工程范围内校验，其他 DTO 默认保持；退出由后端阻尼停稳
  并回 CSP，期间拒绝重新激活。4311 停稳超时最多三轮退出调用（后端每轮 2s）；仅连接、mode5、SDK 归属正常且无锁存错误时继续等待。其他错误或三轮耗尽仍急停兜底。无 FDCC/ServoP/ServoJ 回退。
- Status: 同拍 raw/lead/shape/follow 误差、转动 shape/follow 误差与线/角速度；
  waiting_for_check 区分等检查与模式退出；call_ms 是调用耗时，
  阻抗 DDS 无送达确认；Holding 保留最后写入快照，非物理延迟测量。

- PicoImpedance 可传 `direct_target=true` 复用直发路径；首帧实测锚定，后续平移/旋转批准目标不经主机整形，过期保持/断流暂停/松手退出逻辑不变。

- 关节参考限速：Submit 可携带已批准 IK joints；首帧使用实测关节/TCP，之后限制各轴参考变化率并通过 FK 转为平衡点。写入成功才提交参考状态；过期保持、退出清参考，重入重新锚定。FK 失败/非有限输出进入原故障退出链路；FK 往返可能降低更新率，实际关节仍可能因阻抗动态超速。
