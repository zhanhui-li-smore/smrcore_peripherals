> DIR: teleop/
> SRC: pico_teleop.cpp
> SRC: legacy_release_stop.hpp
> SRC: pico_fdcc.hpp, fdcc_target_limiter.hpp
> REF: pico_compliance.md

# PICO teleoperation core

- Single-arm `CartesianImpedance` shares the compliance worker/clutch/guard
  lifecycle; mode 5, Manual equilibrium targets through the SDK DDS API at
  up to 200Hz. Configurable six-axis K/D and translation shaping, no virtual
  extrapolation. Shares measured-Jacobian slowdown with FDCC; reachable
  high-condition candidate endpoints no longer latch a pause, avoiding six
  extra FK RPCs per candidate. Finite/IK/clearance/FK and expiry checks remain.
  Release waits for backend damping exit to mode 0; faults,
  ownership loss and stale feedback end the session. Details: pico_compliance.md.

- Single-arm `control_mode=Fdcc` uses `PicoFdcc`: latest approved target only,
  a dedicated non-RT RPC worker (50 Hz maximum), generation cancellation on
  release, mode-6 acknowledgement before pose writes, and mode-0 acknowledgement
  before reclutch. No ServoP/ServoJ fallback or persistent force-source write.
  SDK default FDCC params and zero target wrench; actual contact force is not
  constrained to zero. First target after each activation is measured TCP.
  FDCC's built-in input profile is 0.30m/s, 1.2m/s², gain 8/s; configurable
  translation speed/accel/gain, fixed rotation 0.8rad/s, 3rad/s². Elapsed
  integration capped at 20ms; Reset retains config and reanchors at measured TCP.
  FDCC adds target velocity feedforward (default 0.5, range 0..1; impedance 0).
  Uses check-completion timestamp deltas, 40ms filtering, same output rate/accel
  bounds; repeat samples decay, >100ms gaps discard derivative, reset clears it.
  Reduces moving-target shaping lag; faster stopping can increase overshoot.
  Generic limiter/impedance defaults remain 0.15m/s, 0.6m/s². FDCC-only option
  checks compare against `PicoTeleopOptions` defaults to keep other modes valid.
  FDCC guard uses measured-Jacobian slowdown instead of endpoint condition >=60
  rejection (six candidate FK probes omitted). IK/finite/clearance/FK consistency
  still required. ServoP keeps candidate condition checks.
  Optional `MakeFdccVirtualTarget` extrapolates translation before guard, once
  per input frame; 15mm dead zone, smooth 15..30mm onset, extra lead <=30mm.
  Virtual lead leaves the raw target/clutch anchor and orientation unchanged. No approval-age refresh
  from feedback-only changes. Worker snapshots include raw/lead/shaping/follow
  errors, angular shaping/follow errors and commanded linear/angular speed;
  no claim of measured plant response/latency.
  Activation consumes a fresh approved slot before the first pose write.
  Rejected candidates or input/Jacobian loss pause to hold and require release.
  Approval expiry freezes the last sent equilibrium without leaving the mode
  while original packet receive deadlines remain fresh. New approvals resume
  automatically; validity starts at check completion, not request submission.
  IK/FK/queue timing and check=WAIT expose delayed validation. A shared rearm
  latch after a real pause rejects late approvals until fresh released Grip and mode-0 confirmation;
  the main loop clears its mapping anchor before accepting the next press.
  Input pauses distinguish WebSocket disconnect, tracking loss and >250ms
  sample age; log sequence/age/loop gap and worker deadline overrun separately.
  mode loss, stale robot feedback, RPC failure or target RPC >200ms exits.
  In-flight RPC cancellation is unavailable; cleanup waits for SDK completion.
  Ctrl+C exits FDCC, waits for standstill, then requests/reads back Disable.
  J6 thumbstick is ignored in FDCC. Single-arm yaw/gripper options still apply.

Comparison builds define `SMRCORE_PICO_LEGACY_RELEASE_STOP`: release sends 100
fixed measured-TCP ServoP frames (J6 release: measured-joint JointPassthrough),
then waits for fresh standstill. Active J6 remains ServoJ; this is not a full old
wheel rollback. Default build retains the forward-endpoint stop. Both variants
retain stop ownership, fault/stale checks and a 3s deadline.

`pico_teleop.cpp` owns the SDK-backed B3 dual-arm bridge formerly in the CLI.
`PicoTeleop::Run` is one-shot and blocking, with per-session atomic cancellation,
process-local exclusive session ownership and no process signal handlers. SDK
exceptions attempt StopMotion/Disable cleanup. The independent C++ bridge calls Run; the Python wheel only reads peripherals.
Connection uses upstream DualArmRobot.Initialize(robot_ip); no domain/interface override is exposed.

Preserves mapping, gripper, clutch, Jacobian monitors and candidate guards.
The B3 mounting-bracket transforms define the coordinate chain:
`world_from_left_arm_base` / `world_from_right_arm_base` matrices are
embedded in the core for the corresponding B3 mounting configuration, with no mount file or CLI dependency. Switch
both controllers to `base` and confirm fresh base-frame state, calculate
translation and rotation targets in the shared world, then transform each
complete target back to its arm base before ServoP.
The B3 chain has no independent axis correction, manual yaw correction, GetBasePose substitution
or URDF fallback is used. Invalid options, mount matrices and reference switches
fail closed. Public GetStatus exposes
latest joint positions, condition numbers and scales via atomics; not a coherent
RT snapshot. RequestStop requires waiting for Run to return; RPC calls can delay
shutdown. Header: `src/peripherals/pico/teleop.hpp`.

> FLOW: WebXR delta → head-yaw/operator mapping → world target → inverse(world_from_base) → base ServoP

Grip release invalidates the guard generation and clears checked/pending
candidates. Late candidate results cannot relatch the canceled generation; release uses a separate planned ServoJ stop
worker, and stop failure terminates the session;
actual Jacobian condition uses a quadratic scale from 1 at 30 to 0 at 60 while
approaching; leaving uses doubled thresholds with linear, filtered recovery so
the arm can retreat from the boundary. While approaching, scaling applies to the whole
measured-TCP-to-target tracking error, not only the latest hand increment, so
scale zero removes queued ServoP target lag and holds the measured pose.
Candidate IK/clearance/condition boundaries hold the last approved target
without latching. Candidate checks pending beyond 200 ms also hold the last
approved target and resume when a result arrives; they do not terminate the
session. A transient Jacobian sample older than 250 ms freezes and briefly
brakes only the affected arm, then re-clutches from measured TCP after recovery.
PICO input older than 250 ms freezes and brakes both arms, cancels pending
gripper targets, and likewise resumes from measured TCP when WebXR recovers.
Stale robot state, guard infrastructure failures and controller faults remain
fail-closed.

Both end-board DH grippers send initialization unconditionally, wait 1 second,
then write force/speed before asynchronously opening. Initialization performs
no register reads or readback verification. Each initialization write retries only
the explicit previous-transaction-busy rejection for up to 5 seconds, with
20 ms retry/write spacing; other failures abort immediately. Persistent busy
reports hardware recovery guidance without automatically clearing safety stops. Trigger uses
smorebot hysteresis (press 0.75, release 0.25) and each rising edge queues the
latest position 0/1000 to a dedicated gripper worker, so RAW485 never blocks the
1 kHz ServoP loop. Stale input cancels unsent gripper targets.

While Grip is held, thumbstick Y sends J6 targets through ServoJ, preserving
controller planner continuity with ServoP and release stopping. Requested J6
speed is capped to 80% of cached max_velocity*velocity_percentage.
Release cancels pending targets, estimates a forward stopping endpoint using
measured position/velocity and cached acceleration/jerk limits, then repeatedly
sends that fixed endpoint as ServoJ. Ruckig owns output continuity and limits;
the peripheral does not send raw joint passthrough or time-sampled brake frames.
The controller checks adjacent commanded position deltas, not the supplied
velocity field; feedback-seeded passthrough and sender timing gaps could trigger
J6 soft-speed protection (3316), even for a nominally bounded velocity curve.
The endpoint is an estimate: measured feedback is not the current command state,
so physical rebound and stopping distance still need hardware validation.
Limits are read before enable. Invalid limits/endpoint reject startup or stop.
Re-clutch requires fresh feedback within 0.001 rad of the endpoint and below
0.01 rad/s for 50 ms. Feedback stale for 100 ms, faults, invalid states or a 3 s
timeout terminate the session. No core/SDK/IDL or protection setting changes.

> SRC: latency_trace.hpp

Optional `RCORE_PICO_LATENCY_SOCKET` enables nonblocking local datagrams (200 Hz
ceiling/arm), carrying receiver monotonic time/sequence through candidate guard,
approved target and measured TCP. No disk I/O in streaming thread. Dropped
records counted; no retry or motion mutation. Python observer handles arrival
statistics; timestamps describe host receiver/feedback observation, not device
capture or exact physical arrival. Default is disabled.

Trace schema v2 adds per-candidate take/map/submit/check timestamps and exact
first ServoP call boundaries (cached before diagnostic throttling), current call
boundaries and cached-feedback read/first-observation times. No remote clock
subtraction or assertion of DDS send/receive acknowledgement. Python v1 reader
compatibility retained in the diagnostic scripts; old binary package unchanged.

Standalone `single_arm` connects only RobotOptions::Single, consumes physical
right input even face-to-face, resolves base with `SingleWorldFromBase`,
and reports values in the right status slots (left slots remain zero).
Startup ignores left tracking/Grip. Input/right tracking loss, Jacobian expiry,
guard failure and robot faults use the same freeze/brake/preflight helpers.
`enable_gripper=false` skips all gripper initialization and target writes in
both single and dual modes; default dual behavior remains enabled.

`single_mapping_yaw_deg` (default 0, finite [-180,180], single-arm only) rotates
mapped translation and orientation increments about base +Z. -90 is clockwise
viewed from +Z toward the origin. The virtual world-from-base uses the inverse
angle; the measured TCP anchor stays unchanged. No cabinet frame/URDF write.

Pico numeric preflight rejects nonfinite controller pose/buttons/axes, zero or
overflowing quaternion norms and invalid receive timestamps before mapping or
gripper processing. Headset values are checked when head-yaw mapping is enabled;
unused left input is ignored in single-arm mode. Invalid numeric packets terminate
the session through existing StopMotion/Disable cleanup. Candidate targets are
checked before IK; all six J6 joint targets and velocity are checked before
ServoJ, with existing final ServoP finite checks retained. Backend-generated
nonfinite commands still require CM/HW protection and diagnosis.
