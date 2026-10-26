# 版本说明草稿

Codex 根据代码差异生成，尚未人工审核。

v0.0.3 → `2d6f647a6002c5f2dc56c39131af8d42455783cc` · SDK `0.2.0`

## SDK 与构建发布

- 本范围仅包含发布维护变更。新增双语发布说明草稿工作流，以已发布正式版本的祖先提交为基线，固定比较提交范围。需要配置 `OPENAI_API_KEY`。草稿作为 Actions 制品保留 90 天，不自动发布。准备成功后，缺少密钥、生成失败或输出校验不通过时保留带状态的 Git 摘要。SpaceMouse、力/力矩、Pico、主从臂及 Python 读写接口和交付形式均无变更。 ([2d6f647](https://github.com/zhanhui-li-smore/smrcore_peripherals/commit/2d6f647a6002c5f2dc56c39131af8d42455783cc))

## 待核实

- 未验证密钥配置或工作流实际运行结果。文档称默认分支每次 push 均触发，但配置仅监听 `main` 和 `feat/pico-readers-and-release`，且要求运行分支为默认分支；其他默认分支名称需调整配置。
