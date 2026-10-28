> SRC: teleop.hpp
> REF: ../../../../teleop/_index.md

PicoTeleop is a single-use, blocking session; RequestStop then join Run.
Options require explicit IP/motion consent. Default is B3 dual-arm mapping.
`single_arm` selects SDK Single + physical right hand; mapping is identity at
`single_mapping_yaw_deg=0`, otherwise rotates increments around base +Z;
GetStatus uses right slots. `enable_gripper=false` skips gripper IO.
Grip clutches motion, thumbstick Y controls J6; Trigger toggles DH grippers.
`control_mode=Fdcc` selects single-arm FDCC; default ServoP/B3 is unchanged.
FDCC disables J6 thumbstick, preserves gripper/yaw mapping, and uses a dedicated
RPC worker. Grip release exits FDCC to hold; reclutch reanchors from measured TCP.
FDCC uses the built-in 0.30m/s, 1.2m/s², gain 8/s, feedforward 0.5 profile.
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

`CartesianImpedance` is single-arm mode 5; `impedance_stiffness/damping`
configure XYZ/RX/RY/RZ K/D, SDK hold defaults 120/20 and 20/1.
`impedance_max_speed/accel/follow_gain` shape equilibrium targets (not K/D).
200Hz DDS ceiling, no extrapolation; Grip release waits for backend damping
exit to mode 0. Faults/SDK ownership loss terminate; see pico_compliance.md.
