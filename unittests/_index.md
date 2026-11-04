> DIR: unittests/

- test_pico_fdcc.py also verifies direct XYZ/orientation target writes despite low shaping limits, measured activation seed, stale-approval hold/resume, and rejected-candidate release latch. No hardware validation.

- FDCC lifecycle/dual-arm fixtures verify damping_factor propagation, 0.9
  compatibility, 0.7 on both arms and invalid values rejected before activation.
  Impedance exit failure reproduces ordinary Disable rejection in mode 5 and
  requires EStop fallback to leave motors disabled. No hardware is exercised.
  FDCC error_scale tests cover default 1.0, selected multipliers on both arms,
  unchanged SDK kp/tracking limits and pre-activation rejection of invalid values.

- test_pico_dual_fdcc.py: production dual-arm orchestration and FDCC/impedance workers
  with the production input-timeout constant: 300ms-old input remains valid,
  >400ms-old input latches release-required recovery independently of approval age;
  with injected SDK feedback/RPC; independent Grip/reanchor, tracking/Jacobian/
  candidate pauses, shared input loss and release-required recovery, slow RPC
  independence, either-arm faults, cancel-both-before-join and exception cleanup.
  The fake exposes no ServoP/ServoJ; no robot motion or plant simulation.

- test_pico_fdcc_balance.py: production mode-policy selector + guard + workers,
  high-condition candidate keeps FDCC/mode-6 and impedance/mode-5 active with
  one FK per candidate; ServoP still rejects with seven FK calls.
  IK/FK/finite/clearance errors retained in both compliance modes.
  Rejection diagnostics distinguish actual/IK clearance and branch crossing
  independently for J3 and J5.
  Injected 300ms IK delay retains the mode with a fixed last-sent target;
  completion resumes without release. Stale input still latches an exit,
  and late approvals cannot restart it. IK/queue timing uses separate clocks.
  Ramp/sine comparisons record lag and stopping overshoot; rotation wrap,
  repeated/delayed/reversed timestamps, reset and unchanged velocity/accel limits.

- test_pico_impedance.py: production adapter/shared worker with impedance-only
  fake SDK; K/D propagation and bounds, mode/source acknowledgement, measured
  seed/reclutch, slow damping exit, ownership loss, faults/NaN, expiry hold and
  release acknowledgement, in-flight
  cancellation and DDS write failure. Full CLI parser tested without network.
  CLI also verifies the short FDCC command's built-in profile and normal/advanced
  help separation, SpaceMouse/hold impedance profiles, mode restrictions and
  order-independent per-field explicit K/D overrides.
- test_pico_fdcc.py: production worker + mock SDK: pause/reclutch, delayed
  acknowledgements, canceled in-flight activation, failures/mode loss/stale
  feedback, bounded target rate, expiry and no ServoP/ServoJ calls. Limiter
  regression exercises guard-first/worker-first expiry, late approvals, release
  during mode exit, reanchor after acknowledgement and expiry during activation.
  covers reversal, rotation wrap, scheduler stalls, convergence and NaN.
  Configured response tests compare ramp/sine target lag, preserve rate limits
  and settings on reset; virtual targets cover sign, dead zone, continuity,
  lead cap, unchanged orientation and invalid inputs. No robot motion.

- test_pico_base_yaw.py: production mapping/parser, clockwise sign, Z invariance,
  unchanged nonzero TCP anchor and conjugated rotation; no SDK or hardware.

- test_pico_legacy_release.py: offline comparison-stop tests; 100 fixed-target
  frames, separate TCP/J6 paths, stale/fault/invalid feedback rejection.

# Overview

无硬件语义单测，由 `CMakeLists.txt` 中 `add_peripheral_test()` 注册 CTest。

# Contents

- __init__.py: package marker so shared fake-SDK fixtures import under pytest
  and unittest; CI collects the complete offline test directory.
- ut_spacemouse_sample_model.cpp: 轴映射/符号/死区、夹爪锁存（含双键冲突保持）、定频 consume-on-read
- ut_kunwei_parser.cpp: 坤维 `KunweiParser` 帧解析与 `DrainLatest`
- ut_ft_sensor_options.cpp: `FtSensor::Initialize(FtSensorOptions)` 后 `GetInfo()` 字段（路径/型号/采样率）
- ut_xjc_parser.cpp: 实测 XJC 16 字节 int16 / 28 字节 float 帧解析，分片/重同步/CRC 拒绝，以及启停命令

- ut_pico_singularity_scale.cpp: 硬边界不受离开方向放宽；覆盖 30/45/60、非有限输入、整圈跳解、关节跨度和实测肘部危险姿态。

- `test_pico_stop_worker.py`: 编译生产停止目标/worker，验证有方向的目标估计、越限/无效参数拒绝、J6 比例限速、ServoJ 固定目标、延迟发送与停稳门禁；fake 不提供透传/StopMotion，并禁止遥操源码重新引入透传。

- test_zhonglin.py: mapping, invalid/stale samples, bridge rejection and cleanup without hardware.

- test_pico_python_launcher.py: no-hardware argument forwarding, exit status, missing binary and help tests.

- test_weekly_release.py: temporary Git repo checks for initial/patch tags, skip/retry and rollback refusal.

- test_release_notes.py: release identity, four-chain scope, invalid tags and SDK-doc consistency.

- test_version_history.py: version boundaries, bilingual trailers, unreleased commits and unrelated-tag exclusion.

- test_codex_release_notes.py: published baseline, evidence and bilingual output validation, failure fallback.

- test_local_codex_notes.py: subscription auth environment and retry/deduplication gates.

- test_version_history.py: reviewed range deduplication and historical target isolation.
- test_version_history.py: retained fragment version ranges, edits/deletions,
  dirty-worktree isolation, bilingual/category validation, misc exclusion,
  reviewed-summary precedence and trailer deduplication/coexistence.

- test_package_release.py: user guide allowlist excludes maintenance, templates and AI indices. test_version_history.py checks public baseline and historical tag isolation.

- test_pico_single_hand.py: runs actual startup gates with right-only, absent/stale right and held-Grip inputs; no hardware. Python launcher tests include single-arm/gripper flags.

- test_pico_nonfinite.py: injects NaN/Inf and invalid quaternions into actual preflight/send helpers; asserts invalid targets never call the mock SDK.





- test_pico_dual_fdcc.py now runs the shared production orchestration against
  both actual adapters: independent seed/release/reclutch/rearm, slow writes,
  pair fault cleanup, impedance SDK ownership loss, K/D propagation and no FDCC
  calls/extrapolation on impedance. Hardware is never connected.
