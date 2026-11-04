# Pico Reading and Dual-arm Teleoperation

## Architecture

C++ `app_peripherals_read_pico` and Python `app_peripherals_read_pico.py` read headset/controller poses and buttons without a robot SDK. The optional C++ `app_peripherals_pico_bridge` maps poses and controls B3 arms through the dual-arm SDK.

SpaceMouse uploads samples for the cabinet to consume; Pico maps targets in the application and calls SDK motion APIs. Both separate reading from control, but use different control paths.

## Read-only check

```bash
./scripts/build.sh --with-sdk OFF --tests ON
./build_Release/bin/app_peripherals_read_pico --port 8765
```

Or, after installing the reader wheel:

```bash
python python/app/app_peripherals_read_pico.py --port 8765
```

Run only one server on a port. With USB debugging authorized, use another terminal:

```bash
adb devices
adb reverse tcp:8765 tcp:8765
```

Open `http://localhost:8765/` in the headset and enter VR. Output includes sequence, reference space, headset/left/right XYZ in meters, grip and trigger values. These are WebXR reference-space positions, not robot Base/TCP coordinates. Untracked positions are N/A; `GetSample()` is empty when no new sample exists.

For ordinary Wi-Fi, localhost means the headset. Use an accessible trusted HTTPS endpoint with WebSocket forwarding; replacing localhost with an HTTP PC address does not establish a WebXR secure context. The reproducible commands above use adb and do not alter robot network routes.

## Build and start the optional bridge

```bash
./scripts/build.sh --with-sdk ON --with-pico ON --sdk-root /path/to/compatible-sdk
./build_Release/bin/app_peripherals_pico_bridge --help
```

The SDK must contain `sdk/dual_arm_robot.hpp` and matching libraries. The original official v0.2.0 package omitted this header. Default CI/packages exclude Pico teleoperation. The example embeds B3 mounting-bracket transforms and is not a generic dual-arm configuration. Connection uses `DualArmRobot::Initialize(robot_ip)` without CLI domain/interface overrides.

The following enters real robot control. Verify read-only input, mounting, tool and workspace first and keep E-stop accessible. Stop the reader before starting the bridge:

```bash
./build_Release/bin/app_peripherals_pico_bridge --enable-motion --robot-ip <robot-ip> --port 8765
```

The table below describes default ServoP operation; see dual-arm FDCC below.

| Input | Behavior |
| --- | --- |
| Hold Grip | Clutch and control the corresponding arm from pose increments |
| Release Grip | Cancel old targets and perform a planned ServoJ stop |
| Thumbstick Y while gripping | Control J6 with ServoJ; neutral re-clutches ServoP from measured TCP |
| Trigger rising edge | Toggle corresponding DH gripper; press 0.75, release 0.25 hysteresis |
| Ctrl+C | Request stop and cleanup |

**Startup initializes both DH grippers, sets force/speed and requests opening.** Write success is not physical position or grasp confirmation; no readback is implemented.

## CLI options

| Option | Default / range | Meaning |
| --- | --- | --- |
| `--enable-motion` | Required explicitly | Real-control confirmation |
| `--robot-ip` | Cabinet address | Dual-arm SDK connection |
| `--port` | 8765 / 1–65535 | WebXR port |
| `--translation-only` | Off | Disable rotation mapping |
| `--room-fixed` | Off | Room-fixed instead of head-yaw-clutched direction |
| `--operator-frame` | same-side; also face-to-face | Operator direction |
| `--control-mode` | servop; fdcc/impedance for single or dual arms | Teleoperation control mode |
| `--single-mapping-yaw` | 0° / −180° to 180°; single arm only | Rotate mapped increments around base Z; −90° is clockwise viewed from +Z toward the origin |
| `--translation-scale` | 0.8 / 0.1–3.0 | Translation mapping |
| `--rotation-scale` | 1.0 / 0.1–3.0 | Rotation mapping |
| `--motion-scale` | Unset / 0.1–3.0 | Set both scales |
| `--j6-speed` | 0.8 rad/s / 0.1–3.0 | Requested J6 speed, subject to controller limits |
| `--gripper-force` | 80 / 20–100 | DH force parameter, not newtons |
| `--gripper-speed` | 100 / 1–100 | DH speed parameter |

Arguments are processed in order; later options override earlier scale settings.

## Holds and recovery

This section describes default ServoP. FDCC release and rearm behavior is documented below.

Approaching singularities uses quadratic slowdown between condition numbers 30 and 60; leaving uses doubled thresholds. A transient stale Jacobian freezes/brakes the affected arm. Pico input older than 400 ms freezes/brakes both arms and cancels unsent gripper commands; recovered input re-clutches from measured TCP. Stale robot state, controller faults and guard infrastructure failures terminate the session.

Pending/rejected candidate checks hold the last approved target. J6 bypasses Cartesian singularity scaling but retains joint constraints. The Python launcher reuses this C++ bridge. Validate motion examples on the target robot configuration before deployment.

## Start Pico dual-arm teleoperation from Python

The Python entry point reuses the C++ SDK bridge, including the B3 mounting transform, Grip clutch, J6 thumbstick, Trigger grippers, singularity slowdown and stale-input stop. Neither the peripheral wheel nor the robot Python SDK is required.

Build with the compatible dual-arm C++ SDK:

```bash
./scripts/build.sh --with-sdk ON --with-pico ON --sdk-root /path/to/compatible-sdk
python3 python/app/app_peripherals_pico_bridge.py --help
```

After checking the workspace and emergency stop:

```bash
python3 python/app/app_peripherals_pico_bridge.py \
  --robot-ip 192.168.3.232 --port 8765 --enable-motion \
  --gripper-force 80 --gripper-speed 100
```

Use `--bridge-executable /path/to/app_peripherals_pico_bridge` for an installed binary. Otherwise the launcher checks repository `build_Release/bin`, `build_Release/install/bin`, then PATH. Other arguments pass unchanged to the C++ parser; motion opt-in is never added automatically.
The Python process is replaced by the bridge, preserving Ctrl+C handling and exit status without leaving an extra background motion process. Do not run the Pico reader on the same port simultaneously; use the Pico guide for headset connection steps.

## Dual-arm FDCC teleoperation

Add `--control-mode fdcc` to the dual-arm command, without `--single-arm`:

```bash
./build_Release/bin/app_peripherals_pico_bridge \
  --robot-ip <dual-arm-cabinet-ip> --enable-motion --control-mode fdcc
```

Both arms must start in kinematic hold (`mode=0`), with both controllers tracked
and Grips released. Add `--no-gripper` if DH grippers are absent; by default,
startup initializes and opens both grippers. The Python launcher forwards the
same flags. ServoP remains the default control mode.

- Each Grip independently enters its arm's FDCC, seeded from that arm's measured
  TCP. Releasing one Grip exits that arm to hold while the other can keep following.
- B3 mounting, head-yaw and operator mapping are preserved; `face-to-face` still
  swaps hand-to-arm assignments. Triggers toggle the corresponding grippers;
  J6 thumbstick motion is disabled in FDCC.
- Each arm has its own worker with a **50 Hz** target-update ceiling. A slow RPC
  on one arm does not serialize the other worker. This is not 1000 Hz ServoP.
  Both arms share FDCC tuning options and preserve their own cabinet force source.
  Zero target wrench does not guarantee zero contact force.
- Tracking loss, stale Jacobian or rejected targets pause the affected arm and
  require release/regrip. PICO stream loss pauses both with the same rearm rule.
  Delayed checks instead freeze the last equilibrium and resume on fresh approval.
- A controller fault, stale feedback, mode loss or RPC failure on either arm ends
  the whole session. Ctrl+C cancels both streams before exiting FDCC, waiting for
  standstill and requesting motor disable on each arm.

Default translation target limits are 0.30 m/s and 1.2 m/s²; see
[FDCC engineering details](#single-arm-fdcc-teleoperation) for tuning.
`L FDCC` / `R FDCC` logs report each worker's state, send count, RPC duration,
following error and rearm requirement.

Compatibility: FDCC support does not guarantee SDK configuration compatibility.
Against backend `0.2.1-alpha.20261016`, SDK `0.2.0` received state and force-source
replies but stalled on `GetControllerConfig()` after controller-config fields
changed. Rebuilding the SDK with the backend's IDL restored read-only config
queries on both arms. If controller tracking is ready but SDK/reference setup
stalls, check the SDK protocol version before changing PICO Wi-Fi settings.
Startup logs distinguish initialization, command source, reference setup and enable.

## Dual-arm Cartesian impedance teleoperation

Omit `--single-arm` and select impedance:

```bash
./build_Release/bin/app_peripherals_pico_bridge \
  --robot-ip 192.168.3.232 --enable-motion \
  --control-mode impedance --operator-frame face-to-face
```

The local alpha.20261016-compatible build is
`./build_dual_fdcc_1016/bin/app_peripherals_pico_bridge` with the same options.
B3 mounting transforms and operator mapping are preserved. Face-to-face maps
physical right hand to robot left arm, and physical left hand to robot right arm.
Each arm owns its Grip, guard and worker, enters mode 5 and sends equilibrium
poses over DDS at up to 200 Hz. The cabinet runs CST impedance; update rate is
not measured motion bandwidth.

Both arms share configured K/D and shaping limits. Defaults are the SpaceMouse
profile: K=`400,400,400,30,30,30`, D=`30,30,30,2,2,2`, target speed 0.15 m/s,
acceleration 0.6 m/s². `--impedance-profile hold` selects the original soft gains
for both; explicit `--impedance-stiffness/damping` overrides are also supported.

- Both arms must start in mode 0. Each first target is its own measured TCP.
- Releasing one Grip requests that arm's damped exit; the other keeps following.
- Tracking/Jacobian loss or candidate rejection pauses only the affected arm,
  requiring fresh release/re-grip. Shared input loss pauses both.
- Either arm's fault, SDK ownership loss or send failure ends the whole session.
- Ctrl+C cancels both unsent streams before joining workers, then requests and
  reads back motor disable on both arms. There is no ServoP/ServoJ fallback.
- Trigger grippers remain available; use `--no-gripper` when absent. J6 thumbstick
  and FDCC virtual extrapolation are disabled in impedance.

Per-arm `shape_gap_mm`, `follow_err_mm`, `check` and `rearm` distinguish shaping,
following and validation delays. Offline tests and compatible-SDK build passed;
hardware CST transitions and tracking remain unverified. The cabinet must support
the impedance APIs and have valid dynamics configuration.

## Standalone i3: physical right Pico controller

Pass `--single-arm` to connect one SDK `Single` instance. Only the physical
right controller is required. Mapping uses the robot `base` frame with no B3
mount transform. `--operator-frame face-to-face` changes mapping direction,
but never swaps the selected hand in single-arm mode.

Add `--single-mapping-yaw -90` to rotate translation and rotation increments
clockwise around base Z (viewed from +Z). The measured TCP anchor is unchanged.
This does not write cabinet frames/URDF or change head-yaw clutching. Use `0`
to restore the original mapping.

```bash
./build_Release/bin/app_peripherals_pico_bridge \
  --single-arm --robot-ip <single-arm-ip> --enable-motion --no-gripper
```

Use `--no-gripper` when no DH gripper is installed; otherwise omit it to retain
right Trigger gripper toggling. Right Grip clutches motion and thumbstick Y
controls J6 while Grip is held. Release, stale input or right tracking loss
brakes motion. Existing singularity slowdown, target preflight, state/fault
checks and disable-on-exit remain active. ServoP is the default;
`--control-mode fdcc` or `--control-mode impedance` selects single-arm FDCC
or Cartesian impedance. FDCC also supports dual-arm operation; Cartesian
impedance also supports dual arms.
The default mode remains dual-arm. The Python launcher forwards these same flags.

### Single-arm FDCC teleoperation

Add `--control-mode fdcc` to the single-arm command:

```bash
./build_Release/bin/app_peripherals_pico_bridge \
  --single-arm --robot-ip <single-arm-ip> --enable-motion --no-gripper \
  --control-mode fdcc --single-mapping-yaw -90
```

Squeeze the physical right Grip. Pose writes start after FDCC mode acknowledgement;
`FDCC active: mode=6` confirms entry. Release exits FDCC to a position hold while
keeping the connection and motors enabled. Squeeze again to reanchor from measured
TCP. Ctrl+C exits FDCC and requests motor disable. J6 thumbstick is ignored;
operator direction, rotation, gripper and base-yaw options remain available.
Use `--single-mapping-yaw -90` only when a clockwise 90-degree base-Z correction
is needed; its default is zero.

FDCC starts with a built-in following profile; everyday use does not require
feedforward or smoothing-gain options. Add `--motion-scale 1.0` for 1:1 motion.

<details markdown="1">
<summary>Engineering tuning and diagnostics (optional)</summary>

Uses SDK default FDCC parameters (translation stiffness 1000 N/m, kp=0.0005;
rotation stiffness 80 N·m/rad, kp=0.005; joint tracking protection 0.1 rad).
The cabinet's configured force source is preserved. Zero target wrench does not
mean zero contact force: pose error still generates a spring restoring force.
Targets are sent at up to 50 Hz. Defaults are 0.30 m/s, 1.2 m/s² and a
translation shaping gain of 8/s; rotation remains bounded to 0.8 rad/s and 3 rad/s².
Mapped hand motion faster than the translation limit accumulates tracking error.
`--motion-scale 1.0` means a 1:1 displacement mapping, not unlimited target speed.
These settings bound targets, not measured motion.

These settings have built-in defaults. Override them only for tuning; view all options with:

```bash
./build_Release/bin/app_peripherals_pico_bridge --help-advanced
```

| Option | Default | Range / units |
| --- | --- | --- |
| `--fdcc-max-speed` | 0.30 | 0.01–0.5 m/s |
| `--fdcc-max-accel` | 1.2 | 0.05–2 m/s² |
| `--fdcc-follow-gain` | 8 | 1–40 /s, input shaping gain, not controller kp/kd |
| `--fdcc-target-scale` | 1 | 1–3, 1 disables virtual target compensation |
| `--fdcc-target-feedforward` | 0.5 | 0–1, target velocity feedforward; 0 restores the previous shaper |

Feedforward uses newly approved translation/rotation deltas and their check-completion
timestamps, filtered over 40 ms and bounded by the existing linear/angular
speed and acceleration limits. Repeated approvals do not generate new velocity;
interrupted updates decay/reset it and reclutch clears it. It changes neither
FDCC stiffness/kp/kd nor the requested pose. Faster tracking can still increase
abrupt-stop overshoot and contact peaks: compare tracking and contact force
before increasing the fraction to 1.

FDCC retains measured-Jacobian slowdown without pausing at candidate condition
60. Approaching singularities still slows and retreat remains possible. Finite
values, IK reachability, joint clearance, FK consistency and cabinet checks stay
active. Impedance shares this policy; ServoP retains the original candidate checks.

Optionally try `--fdcc-target-scale 2` for translation extrapolation:
`virtual = measured + scale * (raw - measured)`. Compensation is disabled
within 15 mm XYZ error, smoothly introduced over 15–30 mm, and capped at
30 mm additional lead. Orientation is unchanged. The virtual pose passes the
IK/FK/joint-clearance guard before rate limiting and sending. Reaching the target
does not end streaming or duplicate RPCs. This can increase contact force:
compare tracking in free space first. Controller stiffness and protection
thresholds are unchanged.

The once-per-second `FDCC` line separates `raw_err_mm` (raw-to-measured),
`lead_mm` (virtual lead), `shape_gap_mm` (approved-to-sent) and `follow_err_mm`
(sent-to-measured before that RPC). `v_target` reports target speed.
`shape_deg`/`follow_deg` report angular shaping/tracking error; `w_target` is
the commanded angular speed.
A persistent shaping gap indicates input limiting/smoothing; a small shaping
gap but large follow error calls for checking FDCC/drive tracking.
`guard_age_ms` is time since check completion and `rpc_ms` the latest RPC duration;
neither measures physical motion latency. Holding retains the last send snapshot.
`check=WAIT` indicates a pending validation result. `ik_ms`, `fk_ms` and
`check_queue_ms` report the last completed candidate's IK, FK and queue durations.

</details>

Slow candidate checks retain the active control mode. When no new approval has
completed for 200 ms, the sender freezes the last sent equilibrium point;
a new successful check resumes tracking without releasing Grip. Unchecked
targets are never sent, and waiting does not extend the input receive deadline.
Input/Jacobian loss or rejected candidates still pause to hold and require
fresh released Grip input and confirmed mode 0 before reclutch. Late approvals
cannot restart that pause. This also applies to impedance mode below.
Stale robot state, faults, mode loss or RPC failure ends the
session. A dedicated RPC thread keeps input reading responsive; an in-flight
request cannot be recalled and cleanup waits for SDK return/timeout.
A target-write RPC taking more than 200 ms still ends the session as a communication failure.
Both SDK and cabinet must provide FDCC APIs; this build uses SDK 0.2.0. Update
the executable and `libsmrcore_pico_teleop.so` together.

### Single-arm Cartesian impedance teleoperation

Select `--control-mode impedance` to map the physical right controller to a
TCP equilibrium pose. The cabinet runs Cartesian impedance (`mode=5`) in CST
torque control:

```bash
./build_Release/bin/app_peripherals_pico_bridge \
  --single-arm --robot-ip <single-arm-ip> --control-mode impedance \
  --motion-scale 1.0 --enable-motion --no-gripper
```

Mapping matches ServoP/FDCC. Add `--single-mapping-yaw -90` when that base
orientation correction is required. The Python launcher forwards the same options.

Like FDCC, impedance gradually slows using the measured Jacobian rather than
pausing when a candidate IK endpoint has condition number >=60. Each candidate
still passes finite-value, IK reachability, joint-clearance and FK-consistency
checks; the six extra FK calls for its numerical Jacobian are omitted.
Delayed checks freeze the last equilibrium without leaving impedance mode;
tracking resumes automatically after a new successful check.

| Option | Default | Units / range |
| --- | --- | --- |
| `--impedance-profile` | `spacemouse` | `spacemouse` teleop / original soft `hold` |
| `--impedance-stiffness` | `400,400,400,30,30,30` | XYZ: 0–2000 N/m; RX/RY/RZ: 0–150 N·m/rad |
| `--impedance-damping` | `30,30,30,2,2,2` | XYZ: 0–200 N·s/m; RX/RY/RZ: 0–60 N·m·s/rad |
| `--impedance-max-speed` | `0.15` | 0.01–0.5 m/s, equilibrium target limit |
| `--impedance-max-accel` | `0.6` | 0.05–2 m/s², equilibrium target limit |
| `--impedance-follow-gain` | `8` | 1–40 /s, input shaping gain |

K/D take exactly six comma-separated numbers in X/Y/Z/RX/RY/RZ order.
Defaults use the SDK SpaceMouse K/D profile, increasing restoring force and
contact stiffness compared with hold. `--impedance-profile hold` restores
K=`120,120,120,20,20,20`, D=`20,20,20,1,1,1`. Explicit K/D independently
override the profile regardless of argument order. Profiles apply to single- and dual-arm
impedance. Target limits remain 0.15 m/s and 0.6 m/s². Other SDK defaults: task damping
on, position/orientation error saturation 0.03 m / 0.25 rad, zero feedforward
wrench. Engineering bounds describe valid inputs, not a tuning envelope for
every payload.

Hold right Grip and move; `Impedance active: mode=5` confirms activation.
The first equilibrium target starts at measured TCP, then approaches the hand
target with speed/acceleration limits. Release Grip to request the backend's
damped stop and return to position hold. After `Impedance paused: mode=0`,
re-grip to continue from the new measured TCP. Release does not exit the app.
Ctrl+C exits impedance and requests/verifies motor disable. Thumbstick J6 is
inactive; Trigger gripper control and frame options remain available.

Targets use `SetCartesianImpedanceTarget` over DDS at up to 200 Hz; mode
transitions use RPC. `send_ms` measures the SDK write call, not delivery
acknowledgement or physical latency. Error fields have the same meanings as in
FDCC. Input shaping is separate from K/D, and a higher target update rate does
not guarantee faster robot tracking. Impedance does not use FDCC virtual lead.

Activation requires a stationary robot, CST support and a valid dynamics model;
model/payload accuracy affects torque compensation. Position error produces
spring force during contact; zero feedforward wrench does not mean zero contact
force. Stale input/Jacobian or blocked candidates exit to hold. Lost mode, latched
faults, lost SDK ownership or stale state end the session, without automatic
fault recovery or fallback position teleoperation.

The entry builds against official SDK 0.2.0 and has offline lifecycle coverage.
Hardware tracking, contact and CST transition behavior still require validation
on the target cabinet. Update the executable and teleop shared library together.

Startup prints resolved impedance K/D and shaping parameters. Large
`shape_gap_mm` indicates input shaping lag; large `follow_err_mm` indicates actual
TCP lag relative to the sent equilibrium; `check=WAIT` indicates expired target
validation. Matching SpaceMouse gains does not make the whole chain identical
or establish that the reported hardware tracking issue has been fixed.

### Direct-target comparison

Add `--fdcc-direct-target` to bypass host translation/rotation shaping, target speed/acceleration limits and velocity feedforward. Each activation still seeds from measured TCP; subsequent writes use the latest approved target. Native FDCC dynamics, candidate checks, approval expiry holds, input deadlines and release-required pauses remain active. This option requires FDCC and `--fdcc-target-scale 1`; it defaults off. Startup prints `DIRECT` or `SHAPED`. Compare with identical native damping/gain settings; direct targets do not guarantee low physical tracking lag or closed-chain stability.
