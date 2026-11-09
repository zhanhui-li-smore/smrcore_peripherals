<!-- project-header:start -->
<p align="center">
  <img src="docs/site/assets/logo.png" alt="Smartmore Robotics" width="120">
</p>
<h1 align="center">smrcore_peripherals</h1>
<p align="center"><strong>Peripheral input and robot integration examples for SpaceMouse, force/torque sensors, Pico, master–follower arms and keyboards.</strong></p>
<p align="center">
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-Apache_2.0-blue" alt="License: Apache 2.0"></a>
  <a href="https://zhanhui-li-smore.github.io/smrcore_peripherals/"><img src="https://img.shields.io/badge/Docs-online-brightgreen" alt="Documentation"></a>
  <a href="https://github.com/zhanhui-li-smore/smrcore_peripherals/actions/workflows/ci.yml"><img src="https://github.com/zhanhui-li-smore/smrcore_peripherals/actions/workflows/ci.yml/badge.svg?branch=feat%2Fpico-readers-and-release" alt="Build and Release"></a>
  <a href="https://github.com/zhanhui-li-smore/smrcore_peripherals/releases/latest"><img src="https://img.shields.io/github/v/release/zhanhui-li-smore/smrcore_peripherals?label=Release" alt="Latest release"></a>
</p>
<p align="center"><strong>English</strong> · <a href="README.md">简体中文</a></p>

---
<!-- project-header:end -->

The repository provides readers, protocols and application examples. Readers are independent of the robot SDK; applications combine them with the matching SDK for robot control.

## Integration chains

| Peripheral | Reader | Robot integration |
|---|---|---|
| [SpaceMouse](docs/site/spacemouse.md) | C++ / Python | SDK Peripheral sample API |
| [Force/torque sensor](docs/site/ft-sensor.md) | C++ / Python | SDK Peripheral sample API and calibration |
| [Pico](docs/site/pico.md) | C++ / Python | Dual-arm ServoP and grippers; Python can launch the C++ bridge |
| [Master–follower arms](docs/site/zhonglin.md) | Python | SDK ServoJ follower control |
| [Keyboard](docs/site/keyboard.md) | Terminal keyboard input | SDK ServoRelativeP / ServoJ, single / simultaneous dual arms |

## Downloads and documentation

- [Releases](https://github.com/zhanhui-li-smore/smrcore_peripherals/releases): compiled packages, wheels, bilingual manuals and checksums.
- [Asset selection and SDK compatibility](docs/releases.en.md): C++ package choices, Python dependencies and delivery scope.
- [Documentation](https://zhanhui-li-smore.github.io/smrcore_peripherals/) · [C++ examples](app/README.en.md) · [Python examples](python/app/README.en.md).

## Build from source

```bash
git clone https://github.com/zhanhui-li-smore/smrcore_peripherals.git
cd smrcore_peripherals
python3 -m pip install 'conan>=2,<3'
./scripts/build.sh --with-sdk OFF --tests ON
./scripts/run_tests.sh -t Release
./build_Release/bin/app_peripherals_probe
```

Build and install the Python reader wheel after C++:

```bash
./scripts/build_py.sh --no-install
python3 -m pip install --force-reinstall python/dist/*.whl
```

Prebuilt packages are also available. For robot applications follow the relevant chain guide to install the SDK and verify the mounting configuration and device settings.

## License and operation

Repository code uses [Apache License 2.0](LICENSE). SDK runtimes and document templates retain their own attribution and licenses. Readers need device permissions; motion applications require a checked tool, workspace and emergency stop. See [release information](docs/releases.en.md) for platforms, SDK versions and validation scope.
