import importlib.util
from pathlib import Path
import subprocess
import json
import pytest

spec = importlib.util.spec_from_file_location('history', Path(__file__).parents[1] / 'scripts/version_history.py')
h = importlib.util.module_from_spec(spec)
spec.loader.exec_module(h)


def test_version_ranges_trailers_and_unreleased(tmp_path):
    def git(*args):
        return subprocess.check_output(['git', *args], cwd=tmp_path, text=True).strip()
    git('init', '-q')
    git('config', 'user.name', 'Test')
    git('config', 'user.email', 'test@example.com')
    def commit(path, message):
        file = tmp_path / path
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(message)
        git('add', '.')
        git('commit', '-qm', message)
    commit('README.md', 'Initial')
    git('tag', 'v0.0.1')
    commit('python/pico.py', 'feat: reader\n\nRelease-note-zh: Pico 支持读取姿态\nRelease-note-en: Read Pico poses')
    git('tag', 'v0.0.2')
    commit('docs/guide.md', 'docs: explain install')
    text = h.history('zh', root=tmp_path)
    assert text.index('## 未发布') < text.index('## 0.0.2') < text.index('## 0.0.1')
    latest = h.history('zh', target='v0.0.2', root=tmp_path, latest_only=True)
    assert 'Pico 支持读取姿态' in latest
    assert 'Initial' not in latest and 'explain install' not in latest
    assert '相对 v0.0.1' in latest and '### Pico' in latest
    assert 'Read Pico poses' in h.history('en', target='v0.0.2', root=tmp_path)
    # Unrelated branch tags must not become a release baseline.
    git('checkout', '-qb', 'other', 'v0.0.1')
    commit('x', 'unrelated')
    git('tag', 'v9.0.0')
    assert '9.0.0' not in h.history('zh', target='v0.0.2', root=tmp_path)


def test_categories():
    assert h.category(['python/app/zhonglin.py']) == 'master'
    assert h.category(['src/peripherals/ft_sensor/reader.cpp']) == 'ft'
    assert h.category(['.sdk-version']) == 'sdk'


def test_reviewed_notes_are_versioned_and_deduplicated(tmp_path):
    import json
    def git(*args):
        return subprocess.check_output(['git', *args], cwd=tmp_path, text=True).strip()
    git('init', '-q'); git('config', 'user.name', 'Test'); git('config', 'user.email', 'test@example.com')
    def commit(message):
        git('add', '.'); git('commit', '-qm', message)
        return git('rev-parse', 'HEAD')
    (tmp_path / 'x').write_text('one')
    base = commit('Initial'); git('tag', 'v0.0.1')
    (tmp_path / 'x').write_text('two')
    source = commit('raw source title')
    record = dict(base_sha=base, target_sha=source, commits=[source], groups=[dict(category='docs', entries=[dict(zh='审核摘要', en='Reviewed summary', commits=[source])])])
    docs = tmp_path / 'docs'; docs.mkdir()
    path = docs / 'reviewed-release-notes.json'
    path.write_text(json.dumps([record]))
    commit('docs: integrate notes')
    text = h.history('zh', root=tmp_path, latest_only=True)
    assert '审核摘要' in text and 'raw source title' not in text
    assert 'integrate notes' in text
    old = h.history('zh', target=source, root=tmp_path, latest_only=True)
    assert '审核摘要' not in old and 'raw source title' in old
    record['commits'] = [base]
    path.write_text(json.dumps([record])); commit('bad evidence')
    import pytest
    with pytest.raises(ValueError, match='Git range'):
        h.history('zh', root=tmp_path)


def test_user_baseline_hides_development_history(tmp_path):
    import json
    def git(*args):
        return subprocess.check_output(['git', *args], cwd=tmp_path, text=True).strip()
    git('init', '-q'); git('config', 'user.name', 'Test'); git('config', 'user.email', 'test@example.com')
    (tmp_path / 'x').write_text('old')
    git('add', '.'); git('commit', '-qm', 'experimental diary'); git('tag', 'v0.0.1')
    (tmp_path / 'docs').mkdir()
    (tmp_path / 'docs/release-baseline.json').write_text(json.dumps(dict(tag='v0.1.0', zh='四类外设', en='Four peripherals')))
    git('add', '.'); git('commit', '-qm', 'baseline'); git('tag', 'v0.1.0')
    assert '四类外设' in h.history(root=tmp_path)
    assert 'experimental diary' not in h.history(root=tmp_path)
    assert 'experimental diary' in h.history(target='v0.0.1', root=tmp_path)
    (tmp_path / 'x').write_text('next')
    git('add', '.'); git('commit', '-qm', 'internal cleanup')
    (tmp_path / 'x').write_text('new')
    git('add', '.'); git('commit', '-qm', 'feature\n\nRelease-note-zh: 新增读取\nRelease-note-en: New reader')
    git('tag', 'v0.1.1')
    text = h.history(root=tmp_path)
    assert '新增读取' in text and '四类外设' in text
    assert 'internal cleanup' not in text
    assert '四类外设' not in h.history(root=tmp_path, latest_only=True)


@pytest.fixture
def fragment_repo(tmp_path):
    def git(*args):
        return subprocess.check_output(['git', *args], cwd=tmp_path, text=True).strip()
    def commit(path, text, message='feat: raw implementation title'):
        file = tmp_path / path
        file.parent.mkdir(parents=True, exist_ok=True)
        file.write_text(text)
        git('add', '.')
        git('commit', '-qm', message)
        return git('rev-parse', 'HEAD')
    git('init', '-q')
    git('config', 'user.name', 'Test')
    git('config', 'user.email', 'test@example.com')
    base = commit('docs/release-baseline.json', json.dumps(dict(tag='v0.1.0', zh='基线', en='Baseline')))
    git('tag', 'v0.1.0')
    return tmp_path, git, commit, base


def fragment(zh='单臂右手柄', en='Single right controller', category='pico'):
    return f'<!-- release-category: {category} -->\n\n## 中文\n\n- {zh}\n\n## English\n\n- {en}\n'


def test_fragments_publish_by_version_and_ignore_worktree(fragment_repo):
    root, git, commit, _ = fragment_repo
    path = 'changelog.d/2026-10/right-controller.added.md'
    commit(path, fragment(), 'feat: raw title\n\nRelease-note-zh: 重复摘要\nRelease-note-en: Duplicate note')
    source = commit(path, fragment('右手柄与映射角', 'Right controller and yaw'), 'docs: refine unreleased note')
    git('tag', 'v0.1.1')
    old = h.history('zh', root=root, target='v0.1.1')
    assert '右手柄与映射角' in old and '### Pico' in old
    assert '单臂右手柄' not in old and '重复摘要' not in old
    assert f'/blob/{source}/{path}' in old
    assert 'Right controller and yaw' in h.history('en', root=root, target='v0.1.1')
    commit('changelog.d/2026-10/next.fixed.md', fragment('后续修复', 'Next fix'))
    # Neither a later committed edit nor a dirty worktree may rewrite old notes.
    commit(path, fragment('不应再次发布', 'Do not republish'), 'docs: later edit')
    (root / 'changelog.d/2026-10/next.fixed.md').write_text(fragment('未提交', 'Uncommitted'))
    latest = h.history('zh', root=root, latest_only=True)
    assert '## 未发布' in latest and '后续修复' in latest
    assert '右手柄与映射角' not in latest and '不应再次发布' not in latest
    assert '未提交' not in latest
    assert h.history('zh', root=root, target='v0.1.1') == old


def test_deleted_and_misc_fragments_do_not_publish(fragment_repo):
    root, git, commit, _ = fragment_repo
    path = 'changelog.d/2026-10/withdrawn.added.md'
    commit(path, fragment('已撤回', 'Withdrawn'))
    git('rm', path)
    commit('changelog.d/2026-10/ci-maintenance.misc.md', 'Internal engineering record')
    commit('changelog.d/README.md', 'Fragment format instructions')
    commit('changelog.d/2026-10/_index.md', 'AI index')
    for lang, forbidden in [('zh', '已撤回'), ('en', 'Withdrawn')]:
        text = h.history(lang, root=root, latest_only=True)
        assert forbidden not in text
        assert 'Internal engineering record' not in text
        assert 'Fragment format instructions' not in text and 'AI index' not in text


@pytest.mark.parametrize('suffix,text,error', [
    ('added', '<!-- release-category: pico -->\n## 中文\n- 只有中文', 'nonempty notes'),
    ('added', '<!-- release-category: pico -->\n## 中文\n- 中文\n## English\n  ', 'nonempty notes'),
    ('added', fragment(category='invalid'), 'unknown release category'),
    ('unknown', fragment(), 'unknown release fragment kind'),
])
def test_invalid_fragments_fail_build(fragment_repo, suffix, text, error):
    root, _, commit, _ = fragment_repo
    commit(f'changelog.d/2026-10/invalid.{suffix}.md', text)
    with pytest.raises(ValueError, match=error):
        h.history(root=root)


def test_reviewed_summary_takes_precedence_over_fragment(fragment_repo):
    root, _, commit, base = fragment_repo
    source = commit('changelog.d/2026-10/covered.added.md', fragment())
    record = dict(base_sha=base, target_sha=source, commits=[source], groups=[dict(
        category='pico', entries=[dict(zh='已审核完整说明', en='Reviewed full summary', commits=[source])])])
    commit('docs/reviewed-release-notes.json', json.dumps([record]))
    text = h.history(root=root, latest_only=True)
    assert '已审核完整说明' in text and '单臂右手柄' not in text


def test_fragment_and_trailer_entries_coexist(fragment_repo):
    root, _, commit, _ = fragment_repo
    commit('changelog.d/2026-10/new.added.md', fragment())
    commit('CMakeLists.txt', '# example', 'fix: build\n\nRelease-note-zh: 修复构建\nRelease-note-en: Fix build')
    text = h.history(root=root, latest_only=True)
    assert '单臂右手柄' in text and '修复构建' in text
    assert '### SDK 与构建发布' in text
