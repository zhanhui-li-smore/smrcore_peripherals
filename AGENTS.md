# AGENTS.md - smrcore_peripherals

Public source repository for SMRCore peripheral integration. It contains the C++/Python peripheral reader implementation, diagnostic apps, and the SDK bridge/calibration entry points.

## Layout

| Path | What |
|---|---|
| `src/` | Public C++ API, protocol parsers, and device IO |
| `app/` | C++ tools: probe, read, bridge, F/T calib (see `app/README.md`) |
| `python/` | pybind11 wheel, Python app scripts, and tests |
| `docs/` | Platform setup notes |
| `scripts/` | Download, build, test, and package helpers |
| `.sdk-version` | Robot SDK version used by standalone bridge builds |

## Common Tasks

```bash
./scripts/build.sh --with-sdk OFF --tests ON
./scripts/run_tests.sh -t Release

./scripts/download.sh
./scripts/build.sh --with-sdk ON --tests ON
./build_Release/bin/app_peripherals_bridge --robot-ip <robot-ip>
./build_Release/bin/app_peripherals_ft_sensor_calib --robot-ip <robot-ip>
```

## C++ Conventions

- C++17.
- Function and method names use PascalCase.
- Local variables and parameters use snake_case.
- High-rate robot integration goes through `robot.Peripheral().UpdateSpaceMouseSample()` and `robot.Peripheral().UpdateFtSensorSample()`.

## Release Notes

- User-facing changes include a retained bilingual fragment in
  `changelog.d/YYYY-MM/<name>.<kind>.md`; see `docs/maintainers/releasing.md`
  for category and language headings. Internal `misc` fragments are excluded.
- Site, manual and Release builds aggregate fragments from immutable Git ranges.
  After committing, preview with `python3 scripts/version_history.py` before
  pushing; confirm new entries appear under Unreleased. The generator reads
  committed files, not worktree edits. Published version notes remain unchanged.
