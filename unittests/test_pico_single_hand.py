"""Exercise actual startup gates: independent right hand must not need a left hand."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class SingleHandStartupTest(unittest.TestCase):
    def test_right_only_readiness_and_release(self):
        source = (Path(__file__).resolve().parents[1] / 'teleop/pico_teleop.cpp').read_text()
        begin = source.index('bool WaitForControllers(')
        end = source.index('bool ValidateHeadYawMapping()', begin)
        harness = r'''
#include <atomic>
#include <cassert>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
constexpr double kInputStaleSeconds = .25;
constexpr double kGripReleaseThreshold = .25;
std::atomic<bool> running{true};
std::atomic<bool>* session_running = &running;
struct Hand { bool tracked = false; double grip = 0; };
struct PicoSample { Hand left, right; double timestamp_sec = 0; std::string reference_space; };
struct Pico { PicoSample sample; int calls = 0; int limit = 40; };
bool TakePicoSample(Pico& pico, PicoSample& sample) {
    sample = pico.sample;
    if (++pico.calls >= pico.limit) { running.store(false); }
    return true;
}
double SecondsSince(double age) { return age; }
double SecondsSince(std::chrono::steady_clock::time_point since) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now()-since).count();
}
''' + source[begin:end] + r'''
int main() {
    Pico pico;
    PicoSample sample;
    pico.sample.right.tracked = true;
    pico.sample.left.grip = 1;
    assert(WaitForControllers(pico, sample, true));
    pico.calls = 0;
    assert(WaitForReleasedGrips(pico, true));
    // The same packet cannot start the existing dual-arm mode.
    pico.calls = 0; pico.limit = 3; running = true;
    assert(!WaitForControllers(pico, sample));
    // Left tracking never substitutes for missing right tracking.
    pico.sample.left.tracked = true; pico.sample.right.tracked = false;
    pico.calls = 0; running = true;
    assert(!WaitForControllers(pico, sample, true));
    // Held right Grip cannot pass the startup release gate.
    pico.sample.right.tracked = true; pico.sample.right.grip = 1;
    pico.calls = 0; running = true;
    assert(!WaitForReleasedGrips(pico, true));
    // Stale right-only input is never accepted.
    pico.sample.right.grip = 0; pico.sample.timestamp_sec = 1;
    pico.calls = 0; running = true;
    assert(!WaitForControllers(pico, sample, true));
    pico.calls = 0; running = true;
    assert(!WaitForReleasedGrips(pico, true));
}
'''
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / 'test.cpp'
            binary = Path(directory) / 'test'
            cpp.write_text(harness)
            subprocess.run(['g++', '-std=c++17', '-pthread', str(cpp), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
