"""Run the production CLI/input in a private PTY; no keyboard or robot access."""
import fcntl
import os
from pathlib import Path
import pty
import select
import subprocess
import tempfile
import termios
import time
import unittest

from unittests.test_keyboard_teleop import SDK, desktop_build_flags


class Terminal:
    def __init__(self, binary):
        self.master, self.slave = pty.openpty()
        self.saved = termios.tcgetattr(self.slave)
        self.output = b''

        def controlling_terminal():
            os.setsid()
            fcntl.ioctl(0, termios.TIOCSCTTY, 0)

        self.process = subprocess.Popen(
            [str(binary), '--dry-run', '--dual-arm', '--terminal'], stdin=self.slave,
            stdout=self.slave, stderr=self.slave, preexec_fn=controlling_terminal)

    def send(self, data):
        os.write(self.master, data)

    def until(self, text, timeout=3):
        deadline = time.monotonic() + timeout
        collected = b''
        while time.monotonic() < deadline:
            if select.select([self.master], [], [], .05)[0]:
                collected += os.read(self.master, 65536)
                if text in collected:
                    self.output += collected
                    return collected
        self.output += collected
        raise AssertionError(f'Missing {text!r}: {self.output.decode(errors="replace")}')

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        os.close(self.master)
        os.close(self.slave)


class KeyboardTerminalTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        path = Path(cls.tmp.name)
        (path / 'sdk').mkdir()
        (path / 'sdk/robot.hpp').write_text(SDK)
        root = Path(__file__).resolve().parents[1]
        cls.binary = path / 'keyboard'
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pthread',
                        '-I' + str(path), str(root / 'app/app_peripherals_keyboard_bridge.cpp'),
                        str(root / 'app/keyboard_input.cpp'), '-o', str(cls.binary), *desktop_build_flags(root)], check=True)

    def terminal(self):
        terminal = Terminal(self.binary)
        self.addCleanup(terminal.close)
        terminal.until(b'space=cartesian PAUSED')
        return terminal

    def test_space_dual_input_tab_and_ctrl_c_restore_terminal(self):
        t = self.terminal()
        t.send(b' ')
        t.until(b'space=cartesian ENABLED')
        t.send(b'w\x1b[A')
        text = t.until(b'R delta=')
        self.assertIn(b'L/single delta=[0.00003', text)
        self.assertIn(b'R delta=[0.00003', text)
        t.send(b' ')
        t.until(b'space=cartesian PAUSED')
        t.send(b'\t')
        t.until(b'Space changed: joint; paused')
        t.send(b' ')
        t.until(b'space=joint ENABLED')
        t.send(b'w\x1b[A')
        text = t.until(b'R delta=')
        self.assertIn(b'L/single delta=[0.00015', text)
        self.assertIn(b'R delta=[0.00015', text)
        t.send(b'\x03')
        self.assertEqual(t.process.wait(timeout=3), 0)
        self.assertEqual(termios.tcgetattr(t.slave), t.saved)

    def test_arrow_fragments_and_esc_exit(self):
        t = self.terminal()
        t.send(b' ')
        t.until(b'ENABLED')
        t.send(b'\x1b[')
        time.sleep(.01)
        t.send(b'5~')
        text = t.until(b'R delta=')
        self.assertIn(b'R delta=[0.00000,0.00000,0.00003', text)
        t.send(b'\x1b')
        self.assertEqual(t.process.wait(timeout=3), 0)
        self.assertEqual(termios.tcgetattr(t.slave), t.saved)

    def test_paste_and_focus_loss_pause_without_replaying_keys(self):
        t = self.terminal()
        t.send(b' ')
        t.until(b'ENABLED')
        t.send(b'\x1b[200~ \twwwww\x1b[201~')
        text = t.until(b'space=cartesian PAUSED')
        self.assertNotIn(b'switching', text)
        t.send(b' ')
        t.until(b'ENABLED')
        t.send(b'\x1b[O')
        t.until(b'PAUSED')
        t.send(b'\x03')
        self.assertEqual(t.process.wait(timeout=3), 0)

    def test_requires_terminal(self):
        result = subprocess.run([str(self.binary), '--dry-run', '--terminal'], stdin=subprocess.DEVNULL,
                                capture_output=True, text=True, timeout=3)
        self.assertEqual(result.returncode, 1)
        self.assertIn('Interactive terminal required', result.stderr)


if __name__ == '__main__':
    unittest.main()
