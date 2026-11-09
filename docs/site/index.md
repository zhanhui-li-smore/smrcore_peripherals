# Smartmore Peripherals SDK

## Documentation

- [Getting Started](getting-started.md)
- [Release Notes](release-notes.md)
- [C++ Examples](cpp.md)
- [Python Examples](python.md)
- [API Reference](api.md)

<div class="smr-hero" markdown>

**Integrate SpaceMouse, force/torque sensors, Pico, master–follower arms and keyboards.**

Build C++ and Python applications with the peripheral library, and connect robot applications through an optional Smartmore Robot SDK bridge.

</div>

<div class="grid cards" markdown>

- **Getting started**

    ---

    Build readers and install the Python wheel.

    [Installation →](getting-started.md)

- **Robot integration**

    ---

    Understand the SDK bridge and B3 Pico requirements.

    [Integration →](robot-integration.md)

</div>

Readers do not require a robot SDK. Default release packages do not include the optional Pico dual-arm teleoperation executable.

## Peripheral integration

| Peripheral | Input | Robot integration |
|---|---|---|
| [SpaceMouse](spacemouse.md) | Six-axis input and buttons | SDK Peripheral sample API |
| [Force/torque sensor](ft-sensor.md) | Force and torque | SDK Peripheral sample API |
| [Pico](pico.md) | Headset/controller poses and buttons | Dual-arm SDK ServoP and grippers |
| [Master–follower arms](zhonglin.md) | Master joint positions | SDK ServoJ follower control |
| [Keyboard](keyboard.md) | Terminal keyboard input | SDK ServoRelativeP / ServoJ, single / simultaneous dual arms |

Each chain covers input reading, robot integration and configuration. Protocols and robot interfaces reflect the actual implementation.
