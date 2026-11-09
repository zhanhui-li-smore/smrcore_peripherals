# Robot Integration

## SpaceMouse / F/T bridge

The bridge forwards samples through the SDK Peripheral interface. Robot behavior depends on the controller mode and configuration.

```bash
./scripts/download.sh
./scripts/build.sh --with-sdk ON --with-pico OFF
./build_Release/bin/app_peripherals_bridge --help
```

Configure the actual devices before connecting with `--robot-ip <robot-ip>`. The version is pinned in `.sdk-version`; the prebuilt bridge package bundles matching SDK runtime libraries.

## Optional Pico dual-arm example

```bash
./scripts/build.sh --with-sdk ON --with-pico ON --sdk-root /path/to/compatible-sdk
./build_Release/bin/app_peripherals_pico_bridge --help
```

!!! warning "SDK and mounting configuration"
    The SDK must provide `sdk/dual_arm_robot.hpp` and matching libraries. The original official v0.2.0 package omitted this header. The example embeds B3 mounting-bracket transforms; it is not a generic dual-arm configuration.

Default CI and release packages exclude this optional executable. A passing reader build does not establish hardware acceptance. Grippers use each robot's EndBoard DH interface and require compatible hardware and firmware.

Before running motion, verify the robot, tool, payload and workspace, and keep the emergency stop accessible.

See the [complete Pico guide](pico.md) and [platform permissions](platform.md).

## Peripheral integration

| Peripheral | Input | Robot integration |
|---|---|---|
| [SpaceMouse](spacemouse.md) | Six-axis input and buttons | SDK Peripheral sample API |
| [Force/torque sensor](ft-sensor.md) | Force and torque | SDK Peripheral sample API |
| [Pico](pico.md) | Headset/controller poses and buttons | Dual-arm SDK ServoP and grippers |
| [Master–follower arms](zhonglin.md) | Master joint positions | SDK ServoJ follower control |
| [Keyboard](keyboard.md) | Terminal keyboard input | SDK ServoRelativeP / ServoJ, single / simultaneous dual arms |

Each chain covers input reading, robot integration and configuration. Protocols and robot interfaces reflect the actual implementation.
