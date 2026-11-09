import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('notes', Path(__file__).parents[1] / 'scripts/release_notes.py')
notes = importlib.util.module_from_spec(spec)
spec.loader.exec_module(notes)


def test_release_identity_and_supported_chains():
    text = notes.render('v0.0.3', 'example/peripherals', '0.2.0', 'a'*40)
    assert '<version>' not in text
    assert 'zhanhui-li-smore' not in text
    assert '0.0.3-cp310' in text
    for name in ['SpaceMouse', '六维力传感器', 'Pico', '主从臂', 'Master–follower', '键盘', 'Keyboard']:
        assert name in text
    assert 'not include the Pico teleoperation binary' in text
    assert '每周' not in text


def test_sdk_documentation_must_match_pin():
    with pytest.raises(ValueError, match='pinned SDK'):
        notes.render('v0.0.3', 'example/peripherals', '9.9.9', 'a'*40)


@pytest.mark.parametrize('tag', ['v1.0', 'v01.0.0', 'main', 'v1.0.0;echo'])
def test_invalid_tags(tag):
    with pytest.raises(ValueError):
        notes.render(tag, 'example/peripherals', '0.2.0', 'a'*40)
