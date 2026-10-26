# 版本说明草稿

Codex 根据代码差异生成，尚未人工审核。

v0.1.0 → `abe93282b2a87a92ddf010986665f6be9263c656` · SDK `0.2.0`

## SDK 与构建发布

- 本次仅修复 CMake 包集成。通过 `find_package(smrcore_peripherals CONFIG)` 引入共享库时，不再查找私有依赖 `Threads` 和 `nlohmann_json`，避免因缺少 `nlohmann_json` 开发包而配置失败。静态库使用方仍会解析这两项依赖；从源码构建库仍需要它们。链接目标仍为 `smrcore::peripherals`。 ([abe9328](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/abe93282b2a87a92ddf010986665f6be9263c656))

## 待核实

- 以上依据配置模板和源码核对，未运行构建或下游集成测试；尚未验证预编译发布包是否包含此修复。
