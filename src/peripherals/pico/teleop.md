> SRC: teleop.hpp
> REF: ../../../../teleop/_index.md

- `fdcc_direct_target=false` preserves default shaping. Opt-in direct FDCC targets bypass host shaping after measured seed, retain approval/lifecycle guards, and require `fdcc_target_scale=1`; use matching rebuilt CLI/library after options layout changes.

PicoTeleop is a single-use, blocking session; RequestStop then join Run.
Options require explicit IP/motion consent. Default is B3 dual-arm mapping.
`single_arm` selects SDK Single + physical right hand; mapping is identity at
`single_mapping_yaw_deg=0`, otherwise rotates increments around base +Z;
GetStatus uses right slots. `enable_gripper=false` skips gripper IO.
Grip clutches motion, thumbstick Y controls J6; Trigger toggles DH grippers.
`control_mode=Fdcc` selects single/dual-arm FDCC; default ServoP/B3 is unchanged.
FDCC disables J6 thumbstick, preserves gripper/yaw mapping, and uses a dedicated
RPC worker per arm. Grip release exits only that arm to hold; reclutch reanchors
from measured TCP. Dual-arm FDCC preserves B3/face-to-face mapping, requires
both arms in mode 0 at startup, and uses independent 50Hz workers/guards.
Input loss latches release/regrip; any controller/worker fault ends both arms.
Shutdown cancels both streams before joining either worker, then disables each arm.
FDCC uses the built-in 0.30m/s, 1.2m/s², gain 8/s, feedforward 0.5 profile.
`fdcc_damping_factor` is passed to the native FDCC enable request for each arm;
default 0.9, finite [0,1]. Lower means stronger virtual velocity decay per
integration substep. Zero suppresses virtual velocity, not a hardware stop.
It is distinct from task-space damping and does not add closed-chain coordination.
`fdcc_error_scale` scales the native FDCC pose/wrench PD output, default 1.0.
The test entry accepts finite [0.1,4], a local engineering bound, not a robot
hardware limit. Increasing it also amplifies external-wrench feedback and may
reduce coupled-contact stability. It is distinct from Pico follow_gain.
CLI/library must be rebuilt together after this Options layout extension.
`fdcc_max_speed/accel/follow_gain` are advanced input tuning, not CM gains;
`fdcc_target_scale` optionally extrapolates before guard validation (15mm dead
zone, smooth onset, 30mm extra lead cap).
`fdcc_target_feedforward` defaults to 0.5: filtered approved-target velocity
feedforward, bounded by existing speed/acceleration limits; 0 restores the old
shaper. FDCC and impedance use measured singularity slowdown without the
endpoint condition 60 veto; other preflight checks and ServoP remain unchanged.
Delayed checks retain the mode and freeze the last sent equilibrium; new
approvals resume automatically. Input freshness uses original receive times,
approval validity starts at completion. Input loss/rejected targets still pause.

`CartesianImpedance` supports single/dual-arm mode 5; `impedance_stiffness/damping`
configure XYZ/RX/RY/RZ K/D, SDK SpaceMouse defaults K=400/30 and D=30/2.
CLI `--impedance-profile hold` restores K=120/20, D=20/1; explicit K/D
override the corresponding profile field regardless of argument order.
`impedance_max_speed/accel/follow_gain` shape equilibrium targets (not K/D).
200Hz DDS ceiling, no extrapolation; Grip release waits for backend damping
exit to mode 0. Faults/SDK ownership loss terminate; see pico_compliance.md.

Dual impedance shares DualPicoCompliance orchestration with FDCC, selects
PicoImpedance per arm (200Hz DDS), and disables virtual target extrapolation.
Both arms use the same configured K/D/shaping; independent Grip/rearm and B3
operator mapping remain. Either arm losing SDK ownership/faulting exits both.
