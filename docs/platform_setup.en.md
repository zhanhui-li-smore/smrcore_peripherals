# Platform Setup

**English** · [简体中文](platform_setup.md)

## Supported Platforms

V1 standalone builds support Linux only. Windows download and build paths fail fast.

## SpaceMouse

The user running the process must be able to read the selected `/dev/input/event*` node. Use a udev rule or add the user to a group with read access to the device.

Common commands:

```bash
ls -l /dev/input/event*
sudo usermod -aG input "$USER"
```

Log in again for group changes to take effect. `--device` selects a specific event node; omit it to use the built-in discovery path. `--sample-rate` controls the output rate, and decoded samples default to 125 Hz.

## F/T Sensor

Serial F/T sensors require read/write access to the serial device, for example `/dev/ttyUSB0`. Examples default to sensor type `xjc_serial` and baud rate `460800`, both overridable from the CLI.

Common commands:

```bash
ls -l /dev/ttyUSB*
sudo usermod -aG dialout "$USER"
```

`app_peripherals_probe` and `app_peripherals_read_ft_sensor` accept `--serial-port`. Read and bridge apps call `WaitForFirstSample()`; if the first valid frame times out, they print frame error details and either exit or continue according to the app logic.

## Robot Connection

`app_peripherals_bridge` requires a reachable robot controller and a matching `smrcore_sdk` version. For standalone builds, first run:

```bash
sudo apt-get install libxcb1-dev pkg-config
./scripts/download.sh
./scripts/build.sh --with-sdk ON
```

Start the bridge before enabling Teleoperation or FDCC so the controller receives fresh peripheral samples:

```bash
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip>
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> --spacemouse
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip> --ft-sensor
```
