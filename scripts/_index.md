> DIR: scripts/

# Overview

仓库根目录构建与测试脚本。Python wheel 细节脚本在 `python/scripts/`。

# Contents

- download.sh: 读取 `.sdk-version`（或 `SDK_VERSION`/`VERSION`），按当前 Linux 架构下载 `smrcore_sdk-cpp-linux-<arch>-v<version>.tar.gz` 到 `third_party/smrcore_sdk`；未设置 `SDK_RELEASE_TAG` 时先查 `v<version>` release，不存在再回退 `prerelease`；不接受 CLI 参数，Windows 直接报错
- build.sh: C++ SDK 编译/安装（`build_<Type>/install`）；默认 `-a x86`、`--with-sdk ON`、`--tests ON`；`--sdk-root` 指向 standalone `smrcore_sdk` 安装树
- build_py.sh: 在 C++ 已安装前提下构建 Python wheel（调用 `python/scripts/build_wheel.sh`）
- run_tests.sh: C++ CTest（`unittests/`）
- run_test_py.sh: Python pytest（需先 `build_py.sh`；调用 `python/scripts/verify_wheel.sh`）
- check_python_examples.sh: 递归 `py_compile` `python/app/`（CI main-checks / PR smoke 共用）
- package.sh: 打包 C++ install 目录为 `rcore_peripherals.tar.gz`
- ci/resolve_version: 从环境变量/Git 解析 `SMRCORE_PERIPHERALS_VERSION`（C++ 与 Python 共用）

- build.sh: --with-pico ON 显式构建独立 C++ 双臂 bridge，要求匹配双臂 SDK；默认 OFF。build_py.sh 保持原基础 wheel 流程。

- package_release.py: 打包 Linux x86_64 安装树、SDK 运行库/归属、元信息及校验和，不运行设备。
- build_manual.sh: 使用 Pandoc/XeLaTeX 合成中英文 HTML/PDF。

- build_manual.sh: include Zhonglin reader/ServoJ instructions in both language manuals.

- weekly_release.py: plan immutable weekly patch tags, skip published unchanged commits, retry unpublished tags; reject divergent history.

- assemble_manual.py / build_manual.sh: named chapters, resolved links and template license; branded Eisvogel PDF plus self-contained HTML.

- build_manual.py: validates version and builds bilingual HTML/PDF; shell entry delegates unchanged arguments.

- assemble_manual.py: one Peripheral Integration chapter with four peer subsections; C++ and Python remain language entry points.

- release_notes.py: render versioned bilingual release notes from user guides; enforce SDK pin alignment.

- version_history.py: reachable immutable Git tag ranges, path categories and bilingual release-note trailers shared by site/manual/Release.

- docs_history.py: MkDocs pre-build hook generates bilingual version pages, including clean checkouts.

- assemble_manual.py: omit GitHub README branding/badges from print chapters to avoid remote image dependencies.

- codex_release_notes.py: published-release diff context, evidence validation and bilingual draft/fallback renderer.

- local_codex_notes.py / install_local_codex_notes.py: local ChatGPT-auth polling, failure backoff, isolated checkout and immutable draft archive branch; systemd user timer.

- version_history.py: consume reviewed bilingual summaries from target Git trees, validate range/evidence and retain uncovered commit fallback.

- version_history.py: v0.1.0 user baseline replaces development history; later releases show bilingual fragments and approved notes/trailers. package_release.py copies only user documentation.
- version_history.py: new retained bilingual fragments are read from each
  immutable target tree relative to its prior tag; misc/deleted/previously
  released fragments omitted, reviewed notes and trailers deduplicated.
  Missing translations/category fail generation; --target/--output-dir preview
  the same input consumed by site/manual/Release builds.
