> DIR: smrcore_peripherals/

# Overview

公开外设 SDK 源码仓库：SpaceMouse、六维力/力矩传感器与 PICO WebXR
的本地读取、协议解析、采样及调试/桥接工具。外设 library 本身不连接
机器人；需要控制时由独立 SDK bridge app 组合。

GitHub `smore-robotics/smrcore_peripherals` 是源码真源。

# Contents

- changelog.d/: monthly retained change fragments; current Pico single-arm
  entry covers ServoP, FDCC/impedance and input/check lifecycle diagnostics.
  Bilingual fragments feed site/manual/Release version notes automatically;
  AGENTS.md and docs/maintainers/releasing.md define the submission format.
- .gitignore: exclude generated dependencies and build output; keep changelog.d/
  available for retained release fragments despite the dependency-file *.d rule.
- README.md / README.en.md: 中英文仓库入口；按源码仓库语义说明构建、SDK 下载、app、Python 和安全要求
- CMakeLists.txt: 顶层构建/安装/package config；`file(GLOB_RECURSE .../src/*.cpp CONFIGURE_DEPENDS)` 自动收集库源；`smrcore_peripherals` 设置 `EXPORT_NAME peripherals`，安装导出目标为 `smrcore::peripherals`；`install(DIRECTORY src/peripherals/ ... *.hpp)` 安装公开头（排除内部 `sample_model.hpp`）；PICO 协议私有依赖 nlohmann_json；`--with-sdk ON` 时构建原 SDK bridge 与 F/T 标定应用；Pico bridge 另需 `--with-pico ON`；产物 `build_<Type>/bin|lib|install`
- conanfile.py: Conan 2 包 `rcore-peripherals`；options `shared`/`build_tests`；版本来自 `SMRCORE_PERIPHERALS_VERSION`；nlohmann_json 供 PICO 协议层使用，Eigen 供可选 ServoP bridge app 使用
- .sdk-version: standalone `scripts/download.sh` 使用的 smrcore_sdk 默认版本
- docs/: 中英文平台配置，覆盖 SpaceMouse `/dev/input/event*`、F/T 串口权限和 robot bridge 连接
- app/README.md / README.en.md: 中英文 C++ 应用构建与运行说明
- cmake/: `find_package(smrcore_peripherals)` 模板，见 `cmake/_index.md`
- infra/conan/: Conan profile（x86/armv8 等）
- src/: 公开 API、协议层与内部工具，见 `src/_index.md`
- app/: 探测、读数、SDK 桥接与 F/T 标定工具，见 `app/_index.md` 与 `app/README.md`
- python/: pybind11 wheel 绑定，见 `python/_index.md`；`python/app/README.md` 为 Python 应用说明；由 `scripts/build_py.sh` / `scripts/run_test_py.sh` 驱动
- scripts/: SDK 下载、C++ 构建、Python 构建、测试与本地打包，见 `scripts/_index.md`
- unittests/: 无硬件语义单测，见 `unittests/_index.md`

> FLOW: `*Options` 经 `Initialize()` 配置 → `Start()` 启后台线程 → `GetSample()` consume-on-read → 可选 SDK bridge 推送

- teleop/: optional SDK-backed PICO core used by the independent C++ CLI; see `teleop/_index.md`.


- Pico 读取保留在基础库；SMR_PERIPHERAL_WITH_PICO_TELEOP 默认 OFF，显式启用并依赖 SDK，配置阶段检查 standalone 双臂头文件。

- .github/workflows/: 每次提交构建 Linux 制品，版本标签发布 Release；见 .github/workflows/_index.md。
- scripts/package_release.py / build_manual.sh: 安装树打包、双语说明书；发布范围见 docs/releases.md。

- mkdocs.yml / docs/site/: SDK 同款双语 Material 文档站，严格构建，Pages 发布入口。

- Python Zhonglin reader and single-i3 ServoJ bridge preserve the original master-arm pairing; see docs/site/zhonglin.md.

- docs/releases.md / docs/releases.en.md: weekly patch release cadence and manual retry instructions.

- docs/manual/: pinned Eisvogel template, bilingual PDF/HTML branding and attribution; see docs/manual/_index.md.

- mkdocs.yml and docs/site: unified peripheral integration navigation for SpaceMouse, F/T, Pico and master–follower arms.

- docs/releases.md / docs/releases.en.md: canonical user release/asset guides; docs/maintainers/releasing.md: maintainer workflow; website and release notes consume the guides.

- scripts/version_history.py generates categorized version history for website, manuals and Releases.

- README.md / README.en.md: centered project logo, bilingual identity and live license/docs/CI/release badges.

- Codex release-note drafts: docs/maintainers/codex-release-notes.md; isolated default-branch workflow, no automatic publication.

- Local Codex subscription automation is the default draft path; API workflow remains manual-only.

- docs/reviewed-release-notes.json: approved Codex summaries with immutable Git range and commit evidence.

- User releases start at v0.1.0 capability baseline; maintainer guides live in docs/maintainers and are excluded from binary archives.
