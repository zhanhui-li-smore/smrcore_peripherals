# Keyboard teleoperation

Control a standalone arm or both arms through independent key groups in the current terminal, in Cartesian or joint space. **No extra window, `/dev/input`, sudo or keyboard-device permissions.** Local X11 desktops automatically use physical key-down/up state for continuous control. SSH, Wayland and explicit `--terminal` use bounded jogs.

## Build and try input

```bash
# Ubuntu: SDK application build dependencies
sudo apt-get install libxcb1-dev pkg-config
./scripts/download.sh
./scripts/build.sh --with-sdk ON --with-pico OFF --tests OFF

# Check keys and space switching without a robot connection
./build_Release/bin/app_peripherals_keyboard_bridge --dry-run --dual-arm
```

The SDK bridge artifact includes this executable. With an extracted artifact, replace `./build_Release/bin/` with `./run.sh `, for example `./run.sh app_peripherals_keyboard_bridge --dry-run`. Builds with local held-key support require `libxcb1` at runtime; builds without XCB retain terminal jogs.

## Connect a robot

Select SDK command source, position control and the `base` reference frame in the HMI, with the arms stationary. Check the workspace and tool configuration and keep the emergency stop accessible, then run:

```bash
# Standalone arm
./build_Release/bin/app_peripherals_keyboard_bridge \
  --robot-ip ROBOT_IP --enable-motion

# Dual arms
./build_Release/bin/app_peripherals_keyboard_bridge \
  --robot-ip ROBOT_IP --dual-arm --enable-motion
```

Replace `ROBOT_IP` with the cabinet address. For an already running local simulator, replace `--robot-ip ROBOT_IP` with `--local`; the SDK DDS domain must match. For SSH without a desktop, add `--terminal` and allocate a terminal (`ssh -t`).

## Operation

Keep this terminal in focus. Startup prints the key table; a status line shows space and enable state. Continuous mode reports `Input: continuous held keys in this terminal (local X11)`.

- **Tap Space to enable direction input; tap again to pause both arms.** Holding it is unnecessary. Space alone does not generate motion.
- **Tab switches Cartesian / joint space for both arms.** Target accumulation stops, both arms must settle, then current positions seed the new space. Switching leaves input paused; press Space to continue.
- **Ctrl+C or Esc exits and disables the motors.** Terminal settings are restored on exit.
- Status shows `space=cartesian` / `space=joint` and `ENABLED` / `PAUSED`. Losing terminal focus pauses both arms; refocus, release Space and press it again to resume.

| Cartesian direction / joint (positive / negative) | Single / left arm | Right arm |
|---|---|---|
| X / J1 | W / S | Up / Down |
| Y / J2 | A / D | Left / Right |
| Z / J3 | R / F | PageUp / PageDown |
| Rotation about X / J4 | Z / X | I / K |
| Rotation about Y / J5 | C / V | J / L |
| Rotation about Z / J6 | Q / E | U / O |

**Hold direction keys for continuous movement; releasing stops target increments.** Key combinations, diagonal movement and both arms are supported independently of OS autorepeat. Terminal input/output and robot command generation run in separate threads.

### Terminal fallback

```bash
./build_Release/bin/app_peripherals_keyboard_bridge --terminal --dry-run --dual-arm
```

Terminal mode jogs **1 mm** in translation or **0.5°** in orientation/joint space per keypress. Holding uses OS autorepeat and can have gaps; terminals do not supply key-release events or reliably repeat multiple held keys. For continuous control, run in a local X11 desktop terminal without `--terminal`.

## Speed and frames

- Defaults: **0.03 m/s** translation and **0.15 rad/s** orientation/joint speed. `--speed` accepts `(0, 0.1]` m/s; `--angular-speed` accepts `(0, 0.5]` rad/s. Cartesian diagonal combinations do not increase vector speed.
- Default `--frame base` uses each arm's own base; `--frame tool` uses each tool frame. This option only affects Cartesian space; joint space controls J1–J6 directly.
- Cartesian mode uses SDK `Motion().ServoRelativeP()`; joint mode uses `Motion().ServoJ()`. The cabinet handles trajectory planning. Joint targets are also clamped to cabinet position limits read at startup.
- Dual-arm motion uses independent frames, without B3 mounting transforms, object constraints or hardware-synchronized trajectories.
- Pausing stops target accumulation; the cabinet decelerates within its trajectory limits. Terminal focus loss or missing input updates pauses input. Terminal fallback also pauses on paste or input overflow. Disconnection, invalid robot feedback or changed source/mode ends both sessions. Backend limits, collision and emergency-stop handling remain active.

This entry uses position control; it does not select FDCC/impedance or operate grippers.

Exit holds the Servo target, disables the motors and checks `enabled=false`. It does not call the Move-task-only `StopMotion()` on a Servo session, avoiding an inapplicable `3218` rejection.
