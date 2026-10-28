# C++ Applications

**English** · [简体中文](README.md)

This directory contains peripheral diagnostic tools and SDK bridge/calibration executables using the `smrcore::peripherals` C++ API.

For platform permissions and device setup, see [Platform setup](../docs/platform_setup.en.md).

## Build

From the repository root:

```bash
# No robot SDK required: build the peripherals library, read apps, and tests
./scripts/build.sh --with-sdk OFF --tests ON

# Required for robot bridge apps / F/T calibration (SDK targets are only built then)
./scripts/download.sh
./scripts/build.sh --with-sdk ON --tests ON
```

Binaries are built under `build_Release/bin/`:

| Binary | Description |
|---|---|
| `app_peripherals_probe` | Probe SpaceMouse and F/T sensor model, rate, and connection flags |
| `app_peripherals_read_spacemouse` | Decoded SpaceMouse 6-DOF + gripper target samples |
| `app_peripherals_read_spacemouse_raw` | Raw Linux input axis/button samples |
| `app_peripherals_read_ft_sensor` | Serial wrench stream with `WaitForFirstSample` and full serial CLI |
| `app_peripherals_read_pico` | PICO WebXR headset/controller samples without a robot connection |
| `app_peripherals_bridge` | Push SpaceMouse and/or F/T data via `robot.Peripheral().Update*Sample()` (`--with-sdk ON`) |
| `app_peripherals_ft_sensor_calib` | MoveJ static F/T calibration (`--with-sdk ON`; run bridge first for raw) |
| `app_peripherals_pico_bridge` | PICO dual controllers to dual-arm SDK ServoP; this app owns the robot connection (`--with-sdk ON`) |

## Read Peripherals Only

```bash
./build_Release/bin/app_peripherals_probe
./build_Release/bin/app_peripherals_probe --serial-port /dev/ttyUSB0

./build_Release/bin/app_peripherals_read_spacemouse --device /dev/input/event5 --sample-rate 125
./build_Release/bin/app_peripherals_read_spacemouse_raw --device /dev/input/event5
./build_Release/bin/app_peripherals_read_ft_sensor \
  --serial-port /dev/ttyUSB0 --sensor-type xjc_serial --baud-rate 460800

# No robot IP is needed for PICO input inspection
./build_Release/bin/app_peripherals_read_pico --port 8765
```

## Robot SDK Bridge

`app_peripherals_bridge` requires `smrcore_sdk`, a reachable robot, and locally connected peripherals.

```bash
# Both SpaceMouse and F/T sensor are enabled by default, useful for FDCC
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip>

# Teleop only or F/T only
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> --spacemouse
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> --ft-sensor

# Override device options
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> \
  --spacemouse-device /dev/input/event5 \
  --ft-serial-port /dev/ttyUSB0 \
  --ft-sensor-type xjc_serial
```

`--robot-ip <robot-ip>` is optional. When omitted, the app passes an empty IP and lets the SDK apply its default connection behavior. Start the bridge before enabling Teleoperation or FDCC so the controller receives fresh peripheral samples.

## F/T Static Calibration

> **Safety:** Before using an external six-axis F/T sensor for force control
> (for example
> [smrcore_sdk](https://github.com/smore-robotics/smrcore_sdk) FDCC with
> `--wrench-source ft-sensor`), you **must** complete a one-time static
> calibration and `--save` it. An uncalibrated external wrench does not reflect
> true contact forces and can cause large unintended motion — this is dangerous.
> Calibration only needs to succeed once; afterwards, keep the bridge streaming
> samples into the SDK/controller for daily use.

Start the bridge to inject `ft_sensor_state`, then run calibration:

```bash
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> --ft-sensor
./build_Release/bin/app_peripherals_ft_sensor_calib --robot-ip <robot-ip>
# Persist after a good Preview:
./build_Release/bin/app_peripherals_ft_sensor_calib --robot-ip <robot-ip> --save
```

After calibration is saved, daily use with the SDK FDCC example:

```bash
# 1) Keep streaming external F/T samples (and optionally SpaceMouse)
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> --ft-sensor
# or default dual peripherals:
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip>

# 2) In smrcore_sdk, enable the external wrench source / SpaceMouse
./compliance_fd_cartesian_admittance --robot-ip <ip> --wrench-source ft-sensor
./compliance_fd_cartesian_admittance --robot-ip <ip> --mode spacemouse
```

SpaceMouse teleop is also **not** opened by the SDK example itself. Samples must
be injected by this repository's `app_peripherals_bridge --spacemouse` (or the
default dual-peripheral bridge) via
`Robot::Peripheral().UpdateSpaceMouseSample()`.


## Pico

[Complete Pico reading, B3 teleoperation, gripper and CLI guide](../docs/site/pico.md). Requires `--with-sdk ON --with-pico ON` and matching dual-arm SDK. Default packages exclude this executable.

## Standalone i3: physical right Pico controller

Pass `--single-arm` to connect one SDK `Single` instance. Only the physical
right controller is required. Mapping uses the robot `base` frame with no B3
mount transform. `--operator-frame face-to-face` changes mapping direction,
but never swaps the selected hand in single-arm mode.

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
or Cartesian impedance.
The default mode remains dual-arm. The Python launcher forwards these same flags.

See [FDCC usage](../docs/site/pico.md#single-arm-fdcc-teleoperation) and
[Cartesian impedance usage](../docs/site/pico.md#single-arm-cartesian-impedance-teleoperation)
for parameters and pause/exit behavior.
