# Versions and downloads

[Release downloads](https://github.com/zhanhui-li-smore/smrcore_peripherals/releases) · [Development builds](https://github.com/zhanhui-li-smore/smrcore_peripherals/actions)

Five peripheral chains share the structure “input → application → robot SDK”. Readers do not connect to robots; run the corresponding application for motion or gripper control.

## Supported chains

| Peripheral | Reader | Robot integration | Delivery |
|---|---|---|---|
| SpaceMouse | C++ / Python | SDK Peripheral sample API | Prebuilt C++ bridge / Python example |
| Force/torque sensor | C++ / Python | SDK Peripheral sample API and static calibration | Prebuilt C++ bridge / Python example |
| Pico | C++ / Python | Dual-arm ServoP and grippers | C++ source example / Python launcher |
| Master–follower arms | Python | ServoJ follower control | Python source example |
| Keyboard | Terminal / dry-run | ServoRelativeP / ServoJ single / simultaneous dual arms | C++ SDK bridge |

The Pico Python launcher runs the C++ bridge. The master–follower Python example calls the robot Python SDK directly. Both keep peripheral input separate from robot control.

## Choose an asset

Platform: **Ubuntu 22.04 / Linux x86_64**. Wheels support **CPython 3.10, 3.11 and 3.12**.

| File | Contents and purpose |
|---|---|
| `smrcore_peripherals-cpp-linux-x86_64-v<version>.tar.gz` | C++ library, headers, readers and Python examples; no robot SDK |
| `smrcore_peripherals-sdk-bridge-linux-x86_64-v<version>.tar.gz` | Peripheral components plus SpaceMouse / F/T bridges, keyboard teleoperation, calibration and the matching C++ SDK runtime |
| `rcore_peripherals_py-<version>-cp3xx-cp3xx-linux_x86_64.whl` | Python peripheral readers; choose one matching interpreter; no robot SDK |
| `smrcore_peripherals-manual-zh-v<version>.pdf/html` | Chinese manual |
| `smrcore_peripherals-manual-en-v<version>.pdf/html` | English manual |
| `SHA256SUMS`, `*.tar.gz.sha256` | Checksums for all assets and individual archives |

Choose one C++ archive; they need not be installed together. Wheels provide the Python interface. Example scripts are in the archive or same-tag source under `python/app/`. GitHub's automatic Source code assets are source archives, not compiled packages. No Windows, ARM64 or simulator assets are built by this pipeline.

## Install and run

Extract a C++ archive, enter its directory and use the runtime wrapper:

```bash
./run.sh app_peripherals_probe
```

For Python, install one matching wheel; CPython 3.10 example:

```bash
python3 -m pip install ./rcore_peripherals_py-<version>-cp310-cp310-linux_x86_64.whl
```

Master input needs `numpy` and `pyserial`, also available through the wheel's `zhonglin` extra. Python examples that call robot APIs need the matching robot Python SDK. The Pico launcher requires the built C++ bridge instead.

For a complete download use `sha256sum -c SHA256SUMS`. For one archive download its `.sha256` and run `sha256sum -c <archive>.sha256`. Device permissions and serial settings are described in the versioned manual.

## SDK and robot configuration

- Prebuilt SpaceMouse / F/T / keyboard bridges use SDK **0.2.0**, pinned in `.sdk-version`. Each C++ archive records the commit and SDK in `build-info.json`.
- Pico needs a matching C++ SDK including `sdk/dual_arm_robot.hpp`. The currently pinned official package omits this header, so **prebuilt archives do not include the Pico teleoperation binary**. Build it from source with a matching SDK that provides this interface. The example embeds B3 mounting transforms.
- The master–follower example retains its calibrated master/single-i3 mapping. See that guide for hardware and calibration details.
- Update headers and runtime together. Build validation is not hardware acceptance; verify the robot configuration, tool, workspace and emergency stop before motion.


The keyboard executable runs in the current terminal and requires `libxcb1`. Local X11 desktops support continuous held keys; text-only sessions or explicit `--terminal` use bounded jogs.

## Version identity

Releases correspond to `vX.Y.Z` tags; the version is applied to both C++ packages and wheels. Development artifacts remain in Actions for 14 days: archive names include a commit hash, while wheels use `0.0.0`; identify the commit through the run record.

The website follows development. For an installed version use its tag and bundled manual.
