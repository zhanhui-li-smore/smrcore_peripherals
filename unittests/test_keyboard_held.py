"""No OS keyboard access: replay held keys/focus into the production policy."""
from pathlib import Path
import subprocess
import tempfile
import unittest

CHECK = r'''
#include "app/keyboard_desktop.hpp"
#include "app/keyboard_channel.hpp"
#include <cassert>
using namespace smrcore::peripherals::keyboard;
int main() {
    HeldInput source; Control control(true,.03,.15);
    Keys keys; keys.set(KEY_W); keys.set(KEY_UP);
    Snapshot terminal;
    assert(!source.Apply(terminal,keys,42,false).synchronized); // unbound
    terminal.presses={KEY_SPACE}; keys.set(KEY_SPACE);
    auto input=source.Apply(terminal,keys,42,false);
    auto command=control.UpdateHeld(input,.001,1);
    assert(control.Ready() && command[0][0]==.00003 && command[1][0]==.00003);
    double distance=0;
    for (int tick=0;tick<1000;++tick) {
        // Even repeated Space bytes must not disable/re-enable.
        input=source.Apply(terminal,keys,42,false);
        assert(input.presses.empty());
        command=control.UpdateHeld(input,.001,1+tick*.001);
        distance+=command[0][0]; assert(command[0][0]==command[1][0]);
    }
    assert(std::abs(distance-.03)<1e-10);
    terminal.presses.clear(); keys.reset(KEY_W); keys.reset(KEY_UP); keys.reset(KEY_SPACE);
    assert(control.UpdateHeld(source.Apply(terminal,keys,42,false),.001,2)==Commands{});
    keys.set(KEY_W); keys.set(KEY_S); keys.set(KEY_A);
    command=control.UpdateHeld(source.Apply(terminal,keys,42,false),.001,2.01);
    assert(command[0][0]==0 && command[0][1]==.00003);
    keys.reset(KEY_S);
    command=control.UpdateHeld(source.Apply(terminal,keys,42,false),.001,2.02);
    assert(std::abs(std::hypot(command[0][0],command[0][1])-.00003)<1e-10);
    // Other window/tab cannot supply movement, including held Space repeats.
    keys.set(KEY_SPACE); terminal.presses={KEY_SPACE};
    input=source.Apply(terminal,keys,43,false); assert(!input.synchronized);
    control.UpdateHeld(input,.001,2.03); assert(!control.Ready());
    input=source.Apply(terminal,keys,42,false);
    control.UpdateHeld(input,.001,2.04); assert(!control.Ready());
    keys.reset(KEY_SPACE); terminal.presses.clear(); source.Apply(terminal,keys,42,false);
    keys.set(KEY_SPACE); terminal.presses={KEY_SPACE};
    control.UpdateHeld(source.Apply(terminal,keys,42,false),.001,2.05); assert(control.Ready());
    terminal.synchronized=false;
    control.UpdateHeld(source.Apply(terminal,keys,42,false),.001,2.06); assert(!control.Ready());
    terminal.synchronized=true;
    control.UpdateHeld(source.Apply(terminal,keys,42,false),.001,2.07); assert(!control.Ready());
    keys.reset(); terminal.presses.clear(); source.Apply(terminal,keys,42,false);
    keys.set(KEY_SPACE); terminal.presses={KEY_SPACE};
    control.UpdateHeld(source.Apply(terminal,keys,42,false),.001,2.08); assert(control.Ready());
    terminal.presses={KEY_TAB}; keys.reset(KEY_SPACE); keys.set(KEY_TAB);
    control.UpdateHeld(source.Apply(terminal,keys,42,false),.001,2.09); assert(control.Switching());
    std::array<Feedback,2> states{};
    states[0].timestamp=states[1].timestamp=1; assert(!control.TrySwitch(states,2.1));
    states[0].timestamp=states[1].timestamp=2; assert(control.TrySwitch(states,2.21));
    keys.reset(KEY_TAB); keys.set(KEY_SPACE); keys.set(KEY_W); keys.set(KEY_UP); terminal.presses={KEY_SPACE};
    command=control.UpdateHeld(source.Apply(terminal,keys,42,false),.001,2.22);
    assert(command[0][0]==.00015 && command[1][0]==.00015);
    input=source.Apply(terminal,keys,42,true); // desktop shortcuts pause
    assert(!input.synchronized);
    terminal.presses={KEY_ESC};
    assert(source.Apply(terminal,keys,43,true).presses==std::vector<int>{KEY_ESC});
    Channel channel;
    input={}; input.continuous=true; input.held.set(KEY_W); input.presses={KEY_SPACE};
    channel.Push(input,3); assert(channel.Take(3.01).presses.size()==1);
    assert(channel.Take(3.02).presses.empty());
    input=channel.Take(3.2); assert(!input.synchronized && input.held.none());
    input.synchronized=false; channel.Push(input,4);
    input.synchronized=true; channel.Push(input,4.01); assert(!channel.Take(4.01).synchronized);
    channel.DiscardStartupInput(); assert(!channel.Take(4.02).synchronized);
}
'''


class KeyboardHeldTest(unittest.TestCase):
    def test_continuous_keys_focus_repeat_switch_and_stale_input(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as temp:
            source = Path(temp) / 'check.cpp'
            source.write_text(CHECK)
            binary = Path(temp) / 'check'
            subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pthread',
                            '-I' + str(root), str(source), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, timeout=5)


if __name__ == '__main__':
    unittest.main()
