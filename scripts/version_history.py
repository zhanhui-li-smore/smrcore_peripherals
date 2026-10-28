#!/usr/bin/env python3
"""Build version history from reachable tags and auditable Git changes."""
import argparse
import json
from collections import defaultdict
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[1]
GROUPS = {
    'spacemouse': ('SpaceMouse', 'SpaceMouse'),
    'ft': ('六维力传感器', 'Force/torque sensor'),
    'pico': ('Pico', 'Pico'),
    'master': ('主从臂', 'Master–follower arms'),
    'python': ('Python 接口', 'Python interfaces'),
    'sdk': ('SDK 与构建发布', 'SDK, build and release'),
    'docs': ('文档', 'Documentation'),
    'other': ('公共接口与其他更新', 'Common interfaces and other changes'),
}
FRAGMENT_KINDS = {'added', 'changed', 'fixed', 'deprecated', 'removed', 'security', 'misc'}


def git(*args, root=ROOT):
    return subprocess.check_output(['git', *args], cwd=root, text=True).strip()


def category(paths):
    text = '\n'.join(paths).lower()
    for needle, key in [('zhonglin', 'master'), ('pico', 'pico'), ('spacemouse', 'spacemouse'), ('ft_sensor', 'ft')]:
        if needle in text:
            return key
    if any(p.startswith('python/') for p in paths):
        return 'python'
    if any(p.startswith(('scripts/', '.github/', 'cmake/')) or p in ('.sdk-version', 'CMakeLists.txt', 'conanfile.py') for p in paths):
        return 'sdk'
    if paths and all(p.endswith(('.md', '.css', '.yml', '.yaml', '.latex', '.tex')) for p in paths):
        return 'docs'
    return 'other'


def boundaries(target='HEAD', root=ROOT):
    if git('rev-parse', '--is-shallow-repository', root=root) == 'true':
        raise RuntimeError('Version history requires full Git history: git fetch --unshallow --tags')
    tip = git('rev-parse', target, root=root)
    tags = []
    for tag in git('tag', '--merged', tip, root=root).splitlines():
        if re.fullmatch(r'v(?:0|[1-9]\d*)\.(?:0|[1-9]\d*)\.(?:0|[1-9]\d*)', tag):
            sha = git('rev-list', '-n', '1', tag, root=root)
            distance = int(git('rev-list', '--count', f'{sha}..{tip}', root=root))
            tags.append((distance, tag, sha))
    tags.sort(key=lambda t: (t[0], t[1]))
    points = [] if tags and tags[0][2] == tip else [('Unreleased', tip)]
    # Skip tags on diverging ancestors; each baseline must precede its target.
    for _, tag, sha in tags:
        if points:
            result = subprocess.run(['git', 'merge-base', '--is-ancestor', sha, points[-1][1]], cwd=root)
            if result.returncode == 1:
                continue
            if result.returncode:
                raise RuntimeError('Unable to determine release ancestry')
            if sha == points[-1][1]:
                continue
        points.append((tag, sha))
    return points


def reviewed_entries(tip, base, commits, root=ROOT):
    """Read approved notes from the target tree, never from a newer checkout."""
    path = 'docs/reviewed-release-notes.json'
    if path not in git('ls-tree', '--name-only', tip, '--', path, root=root).splitlines():
        return [], set()
    records = json.loads(git('show', f'{tip}:{path}', root=root))
    entries, covered = [], set()
    for record in records:
        if not base or record['base_sha'] != base[1]:
            continue
        source = record['target_sha']
        if source not in commits:
            continue
        expected = set(git('rev-list', '--no-merges', f'{base[1]}..{source}', root=root).splitlines())
        if set(record['commits']) != expected or not expected <= set(commits):
            raise ValueError('Reviewed notes do not match their Git range')
        if covered & expected:
            raise ValueError('Overlapping reviewed release notes')
        for group in record['groups']:
            if group['category'] not in GROUPS:
                raise ValueError('Unknown reviewed category')
            for entry in group['entries']:
                if not entry['commits'] or not set(entry['commits']) <= expected:
                    raise ValueError('Reviewed evidence is outside its Git range')
                if not all(isinstance(entry[lang], str) and entry[lang].strip() for lang in ('zh', 'en')):
                    raise ValueError('Reviewed notes require both languages')
                entries.append((group['category'], entry))
        if not entries:
            raise ValueError('Reviewed notes must contain entries')
        covered.update(expected)
    return entries, covered


def fragment_entries(tip, base, reviewed, root=ROOT):
    """Read new retained fragments from the version's Git tree, not the worktree."""
    def paths(revision):
        return set(git('ls-tree', '-r', '--name-only', revision, '--', 'changelog.d', root=root).splitlines())
    previous = paths(base[1]) if base else set()
    revision = f'{base[1]}..{tip}' if base else tip
    entries, covered = [], set()
    for path in sorted(paths(tip) - previous):
        match = re.fullmatch(r'changelog\.d/\d{4}-(?:0[1-9]|1[0-2])/[a-z0-9][a-z0-9-]*\.([a-z]+)\.md', path)
        if not match:
            continue  # README and AI indices are not release fragments.
        kind = match.group(1)
        if kind not in FRAGMENT_KINDS:
            raise ValueError(f'{path}: unknown release fragment kind: {kind}')
        evidence = git('rev-list', '--reverse', '--no-merges', revision, '--', path, root=root).splitlines()
        if evidence and evidence[0] in reviewed:
            continue  # The approved summary already describes this change.
        covered.update(evidence)
        if kind == 'misc':
            continue
        text = git('show', f'{tip}:{path}', root=root)
        parts = re.fullmatch(
            r'\s*<!-- release-category: ([a-z]+) -->\s*## 中文\s*\n(.+?)\n## English\s*\n(.+?)\s*',
            text, re.DOTALL)
        if not parts or not all(parts.group(i).strip() for i in (2, 3)):
            raise ValueError(f'{path}: expected release-category, ## 中文 and ## English with nonempty notes')
        key = parts.group(1)
        if key not in GROUPS:
            raise ValueError(f'{path}: unknown release category: {key}')
        entries.append((key, {'zh': parts.group(2).strip(), 'en': parts.group(3).strip()}, path))
    return entries, covered


def section(label, tip, base, language, repository, root=ROOT, user_only=False):
    zh = language == 'zh'
    title = ('未发布' if zh else 'Unreleased') if label == 'Unreleased' else label.removeprefix('v')
    lines = [f'## {title}', '']
    if base:
        baseline, sha = base
        url = f'https://github.com/{repository}/compare/{sha}...{tip}'
        lines += [f'相对 {baseline} 的累计变化。[完整差异]({url})。' if zh else f'Cumulative changes since {baseline}. [Full diff]({url}).', '']
        revision = f'{sha}..{tip}'
    else:
        revision = tip
        lines += ['首个版本的提交记录。' if zh else 'Commit history for the first release.', '']
    grouped = defaultdict(list)
    commits = git('rev-list', '--reverse', '--no-merges', revision, root=root).splitlines()
    reviewed, covered = reviewed_entries(tip, base, commits, root)
    for key, entry in reviewed:
        text = entry[language].replace('<', '&lt;').replace('>', '&gt;')
        links = ', '.join(f'[{sha[:7]}](https://github.com/{repository}/commit/{sha})' for sha in entry['commits'])
        grouped[key].append(f'- {text} ({links})')
    fragments, fragment_commits = fragment_entries(tip, base, covered, root)
    for key, entry, path in fragments:
        text = entry[language].replace('<', '&lt;').replace('>', '&gt;')
        source = '变更记录' if zh else 'Change record'
        grouped[key].append(f'{text}\n\n[{source}](https://github.com/{repository}/blob/{tip}/{path})\n')
    covered.update(fragment_commits)
    for commit in commits:
        if commit in covered:
            continue
        message = git('show', '-s', '--format=%B', commit, root=root)
        subject = message.splitlines()[0]
        note = re.search(r'^Release-note-' + language + r':\s*(.+)$', message, re.MULTILINE | re.IGNORECASE)
        if user_only and not note:
            continue
        text = note.group(1) if note else subject
        paths = git('diff-tree', '--root', '--no-commit-id', '--name-only', '-r', commit, root=root).splitlines()
        text = text.replace('<', '&lt;').replace('>', '&gt;').replace('[', '\\[').replace(']', '\\]')
        grouped['docs' if subject.startswith('docs:') else category(paths)].append(f'- {text} ([{commit[:7]}](https://github.com/{repository}/commit/{commit}))')
    if not grouped:
        lines += [('无面向用户的新增变更。' if user_only else '无新增提交。') if zh else ('No additional user-facing changes.' if user_only else 'No new commits.'), '']
    for key, titles in GROUPS.items():
        if key in grouped:
            lines += [f'### {titles[0 if zh else 1]}', '', *grouped[key], '']
    return '\n'.join(lines)


def history(language='zh', target='HEAD', repository='zhanhui-li-smore/smrcore_peripherals', root=ROOT, latest_only=False):
    points = boundaries(target, root)
    tip = git('rev-parse', target, root=root)
    baseline_path = 'docs/release-baseline.json'
    if baseline_path in git('ls-tree', '--name-only', tip, '--', baseline_path, root=root).splitlines():
        baseline = json.loads(git('show', f'{tip}:{baseline_path}', root=root))
        if not re.fullmatch(r'v[0-9]+\.[0-9]+\.[0-9]+', baseline['tag']):
            raise ValueError('Invalid public release baseline')
        lines = ['# 版本说明' if language == 'zh' else '# Release notes', '']
        # Before the first baseline tag, show its release candidate content.
        stop = next((i for i, point in enumerate(points) if point[0] == baseline['tag']), 0)
        for i, (label, sha) in enumerate(points[:stop]):
            lines.append(section(label, sha, points[i+1], language, repository, root, user_only=True))
            if latest_only:
                return '\n'.join(lines) + '\n'
        lines += ['## ' + baseline['tag'][1:], '', baseline[language]]
        return '\n'.join(lines) + '\n'
    lines = ['# 版本说明' if language == 'zh' else '# Release notes', '',
             '按 Git 版本范围整理，优先收录审核后的双语变更摘要；其余提交使用发布说明或原始标题。每条保留代码依据。' if language == 'zh' else
             'Generated from Git version ranges, preferring reviewed bilingual summaries. Remaining commits use release-note trailers or original titles. Entries retain links to their implementation evidence.', '']
    for i, (label, tip) in enumerate(points):
        lines.append(section(label, tip, points[i+1] if i+1 < len(points) else None, language, repository, root))
        if latest_only:
            break
    return '\n'.join(lines) + '\n'


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--repository', default='zhanhui-li-smore/smrcore_peripherals')
    parser.add_argument('--target', default='HEAD')
    parser.add_argument('--output-dir', type=Path, default=ROOT / 'docs/site')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for lang, filename in [('zh', 'release-notes.zh.md'), ('en', 'release-notes.md')]:
        output = args.output_dir / filename
        output.write_text(history(lang, target=args.target, repository=args.repository))
        print(f'Generated {output}')
