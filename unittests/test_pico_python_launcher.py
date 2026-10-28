"""Exercise process replacement using a fake bridge, without hardware."""
import json
from pathlib import Path
import subprocess
import sys

LAUNCHER = Path(__file__).resolve().parents[1] / 'python/app/app_peripherals_pico_bridge.py'


def test_arguments_and_exit_status(tmp_path):
    bridge = tmp_path / 'bridge with spaces'
    output = tmp_path / 'args.json'
    bridge.write_text('#!' + sys.executable + '\nimport json, sys\n'
                      'from pathlib import Path\n'
                      f'Path({str(output)!r}).write_text(json.dumps(sys.argv[1:]))\n'
                      'sys.exit(7)\n')
    bridge.chmod(0o755)
    args = ['--robot-ip', 'literal;$(echo no)', '--gripper-force', '80',
            '--single-arm', '--no-gripper']
    result = subprocess.run([sys.executable, str(LAUNCHER), '--bridge-executable',
                             str(bridge), *args], capture_output=True, text=True)
    assert result.returncode == 7
    assert json.loads(output.read_text()) == args
    assert '--enable-motion' not in json.loads(output.read_text())


def test_explicit_missing_binary_does_not_fall_back(tmp_path):
    result = subprocess.run([sys.executable, str(LAUNCHER), '--bridge-executable',
                             str(tmp_path / 'missing')], capture_output=True, text=True)
    assert result.returncode == 2
    assert 'not found' in result.stderr


def test_help_without_sdk(tmp_path):
    result = subprocess.run([sys.executable, str(LAUNCHER), '--bridge-executable',
                             str(tmp_path / 'missing'), '--help'], capture_output=True, text=True)
    assert result.returncode == 0
    assert '--with-pico ON' in result.stdout
