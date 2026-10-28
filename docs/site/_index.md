> DIR: docs/site/

- pico.md/pico.zh.md: 慢候选检查保留力控模式，冻结最后平衡点并自动续跟；
  区分检查完成时间和原始输入存活期，说明 IK/FK/等待耗时字段。

- Pico FDCC guide: configurable translation shaping and bounded virtual target;
  diagnostic gaps distinguish input shaping from measured follow error.
  Half target-velocity feedforward balances shaping lag and stop overshoot;
  measured slowdown replaces the FDCC endpoint-condition veto. Angular gaps
  distinguish rotation limiting from controller tracking; impedance gains unchanged.
  FDCC starts with the built-in balanced profile; user instructions omit tuning
  flags, engineering detail is collapsed and available via `--help-advanced`.

- Pico single-arm guide documents `--single-mapping-yaw`: base-Z rotation
  of mapped increments, preserving the TCP anchor and cabinet configuration.

- *.md / *.zh.md: 中英文成对页面，suffix i18n 保持 /zh/ 路径。
- assets/: SDK 文档站品牌样式、标识及许可证。

- api.md / api.zh.md: 公开头文件索引、生命周期和 Python 入口；首页提供五个文档入口。

- cpp / python / platform: 整理原应用、桥接、标定及权限文档；pico: 只读到双臂的完整流程、参数与恢复语义。

- Python 中文说明统一按当前 wheel 用法描述，USB 转发独立列为连接步骤。

- zhonglin.md / zhonglin.zh.md: original master/i3 pairing, reader setup and ServoJ example.

- pico/python pages: bilingual Python launcher setup and unchanged C++ control semantics.

- spacemouse/ft-sensor/pico/zhonglin: four peer device guides under Peripheral Integration; public name 主从臂 / Master–follower arms, hardware details only in scope.

- downloads.md / downloads.zh.md: include canonical bilingual release guides instead of duplicating support claims.

- release-notes(.zh).md: generated Git version history, separate from downloads and SDK compatibility.

- Device guides describe current operation and configuration; obsolete integration report removed.

- pico.md / pico.zh.md: dual-arm and standalone i3 right-hand mode, no-gripper option and frame semantics.

- pico.md / pico.zh.md: single-arm `--control-mode fdcc`, mode acknowledgement, Grip pause/reclutch, target limits and preserved cabinet force source.
  Approved-target expiry consistently pauses FDCC/impedance; fresh released Grip
  and confirmed hold are required before re-entry, independent of thread order.

- pico.md / pico.zh.md: Cartesian impedance right-hand mode, K/D and equilibrium
  shaping flags, CST damping exit, DDS send diagnostics and motion-test scope.
  Impedance shares FDCC measured-Jacobian slowdown, omits the endpoint-condition
  veto and six FK calls; finite/IK/clearance/FK validation and expiry remain.
