<!-- release-category: docs -->

## 中文

- 修复代码已推送、版本说明却漏掉新功能的问题。流水线自动按版本范围汇总变更片段，中英文在线版本页、PDF/HTML 说明书和 Release 共用同一份内容。
- 未打 tag 的变化显示在“未发布”；正式版本保持各自提交时的说明。每条可追溯到对应版本的变更记录。

## English

- Fix missing feature entries after a code push. The pipeline aggregates change fragments by version range, sharing the result across bilingual online release notes, PDF/HTML manuals and Releases.
- Changes without a tag appear under Unreleased; each released version retains the notes from its own commit. Entries link to the corresponding version's change record.
