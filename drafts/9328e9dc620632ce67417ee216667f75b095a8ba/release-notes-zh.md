# 版本说明草稿

Codex 根据代码差异生成，尚未人工审核。

v0.0.3 → `9328e9dc620632ce67417ee216667f75b095a8ba` · SDK `0.2.0`

## SDK 与构建发布

- 本范围仅涉及版本说明自动化、文档和相关测试维护。SpaceMouse、力/力矩、Pico、主从臂的实现，以及 Python 读取器、启动器和 SDK 版本均未改动。 ([2d6f647](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/2d6f647a6002c5f2dc56c39131af8d42455783cc), [9328e9d](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/9328e9dc620632ce67417ee216667f75b095a8ba))
- 新增双语草稿生成流程，以目标提交的已发布正式祖先版本为基线，固定比较 SHA。校验输出结构和条目引用的提交范围。目标版本的 Actions 流程仅支持默认分支手动触发；模型生成需要 `OPENAI_API_KEY`。准备成功后，缺少 Key、生成失败或输出无效时保留带状态的 Git 摘要。草稿制品保留 90 天，不自动纳入正式 Release。 ([2d6f647](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/2d6f647a6002c5f2dc56c39131af8d42455783cc), [9328e9d](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/9328e9dc620632ce67417ee216667f75b095a8ba))
- 新增本机 ChatGPT 登录方式的草稿脚本和 systemd 用户定时器安装器。安装后每十分钟检查默认分支，按目标 SHA 将双语草稿归档并推送至 `automation/release-notes`。默认跳过已完成的 SHA，失败后等待一小时重试；`--force` 可跳过这些限制。构建工作流排除该归档分支的 push，并将 tag push 限定为 `v*`。 ([9328e9d](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/9328e9dc620632ce67417ee216667f75b095a8ba))

## 文档

- 新增本机定时任务与手动 API 工作流的配置说明，涵盖认证要求、草稿位置、状态检查、停用和失败恢复。维护者流程明确要求审核草稿后再纳入正式版本说明。 ([2d6f647](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/2d6f647a6002c5f2dc56c39131af8d42455783cc), [9328e9d](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/9328e9dc620632ce67417ee216667f75b095a8ba))

## 待核实

- `docs/codex-release-notes.md` 仍有“push/手动触发”的旧描述，与目标工作流仅配置 `workflow_dispatch` 不一致；触发方式以工作流代码为准，文档仍需统一。
- 本次仅静态检查代码和文档，未运行脚本或测试。定时器是否已安装、认证是否可用，以及草稿生成和归档推送是否成功，均未确认。
