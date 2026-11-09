"""Exercise the real keyboard policy and session with an offline SDK double."""
from pathlib import Path
import os
import fcntl
import pty
import select
import termios
import time
import subprocess
import shlex
import tempfile
import unittest


def desktop_build_flags(root):
    flags = subprocess.check_output(['pkg-config', '--cflags', '--libs', 'xcb'], text=True)
    return ['-DSMR_KEYBOARD_HAS_XCB=1', str(root / 'app/keyboard_desktop.cpp'), *shlex.split(flags)]


SDK = r'''
#pragma once
#include <array>
#include <stdexcept>
#include <string>
#include <vector>
namespace rcore {
enum class ControlMode { Kinematics, Other };
enum class CartesianJogFrame { Reference, Tool };
struct CartesianPoseDelta { double dx=0,dy=0,dz=0,droll=0,dpitch=0,dyaw=0; };
namespace sdk {
enum class RobotArmInstance { Single, Left, Right };
enum class CommandSource { Sdk, Unclaimed, TeachPendant };
struct RobotOptions { RobotArmInstance arm_instance=RobotArmInstance::Single; };
struct Result {
    bool ok=true;
    bool IsSuccess() const { return ok; }
    int GetErrorCode() const { return ok?0:1; }
    std::string GetErrorMsg() const { return "injected"; }
};
struct Errors { int GetLatchedErrorCount() const { return 0; } };
struct State {
    double timestamp=1;
    bool cartesian_valid=true;
    std::array<double,6> positions{}, velocities{}, tcp_offset_from_nominal{};
    std::array<double,3> cartesian_position{}, cartesian_orientation{};
    CommandSource command_source=CommandSource::Sdk;
    ControlMode control_mode=ControlMode::Kinematics;
    Errors errors;
    std::string reference_frame="base";
};
struct Motor { bool enabled=false, operational=true, estop=false, error=false; };
using JointPositions = std::array<double,6>;
struct Limits {
    JointPositions position_limit_min{{-3,-3,-3,-3,-3,-3}};
    JointPositions position_limit_max{{3,3,3,3,3,3}};
};
struct Record {
    RobotArmInstance instance=RobotArmInstance::Single;
    std::vector<CartesianPoseDelta> writes;
    std::vector<JointPositions> joint_writes;
    CartesianJogFrame frame=CartesianJogFrame::Reference;
    State state;
    Motor motor;
    int enables=0, disables=0, stops=0, shutdowns=0;
};
inline std::array<Record,2> records;
inline int created=0, fail_enable=-1, fail_send=-1, fail_disable=-1, stuck_enabled=-1;
inline CommandSource initial_source=CommandSource::Sdk;
class Robot {
    int id_=created++;
public:
    bool Initialize(const std::string &, const RobotOptions &o) {
        records[id_].instance=o.arm_instance;
        records[id_].state.command_source=initial_source;
        return true;
    }
    bool IsConnected() const { return true; }
    State GetState() { records[id_].state.timestamp += .001; return records[id_].state; }
    Motor GetMotorStatus() const { return records[id_].motor; }
    Robot &Control() { return *this; }
    Robot &Config() { return *this; }
    Limits GetControllerConfig() { return {}; }
    Robot &Motion() { return *this; }
    CommandSource GetCommandSource() const { return records[id_].state.command_source; }
    Result SetCommandSource(CommandSource source) { records[id_].state.command_source=source; return {}; }
    Result Enable() {
        ++records[id_].enables;
        records[id_].motor.enabled=true;
        return {id_!=fail_enable};
    }
    Result Disable() { ++records[id_].disables; if (id_!=stuck_enabled && id_!=fail_disable) { records[id_].motor.enabled=false; } return {id_!=fail_disable}; }
    Result StopMotion() { ++records[id_].stops; return {false}; }
    void Shutdown() { ++records[id_].shutdowns; }
    void ServoJ(const JointPositions &q) {
        if (id_==fail_send) { throw std::runtime_error("send failed"); }
        records[id_].joint_writes.push_back(q);
    }
    void ServoRelativeP(CartesianPoseDelta d, CartesianJogFrame frame) {
        if (id_==fail_send) { throw std::runtime_error("send failed"); }
        records[id_].writes.push_back(d); records[id_].frame=frame;
    }
};
} }
'''

CHECK = r'''
#define main KeyboardMain
#include "app/app_peripherals_keyboard_bridge.cpp"
#undef main
#include <cassert>
#include <limits>
using namespace rcore::sdk;
using namespace smrcore::peripherals::keyboard;
bool Zero(const Commands &commands) { return commands==Commands{}; }
bool ZeroPose(const rcore::CartesianPoseDelta &d) {
    return d.dx==0 && d.dy==0 && d.dz==0 && d.droll==0 && d.dpitch==0 && d.dyaw==0;
}
Feedback Valid() {
    Feedback s; s.timestamp=1000000; s.connected=s.valid=s.sdk_source=s.kinematics=true;
    s.reference="base"; return s;
}
void Controls() {
    Control control(true,.03,.15);
    assert(Zero(control.Update({KEY_W,KEY_UP},.001,1))); // starts paused
    assert(Zero(control.Update({KEY_SPACE},.001,1.01))); // enable alone never moves
    auto d=control.Update({KEY_W,KEY_UP},.001,1.02);
    assert(std::abs(d[0][0]-.00003)<1e-12 && d[0]==d[1]);
    Commands total=d;
    for (int i=0;i<100;++i) {
        d=control.Update({},.001,1.021+i*.001);
        for (int arm=0;arm<2;++arm) { total[arm][0]+=d[arm][0]; }
    }
    assert(std::abs(total[0][0]-.001)<1e-12 && total[0]==total[1]);
    assert(Zero(d)); // finite movement without relying on a key release
    assert(Zero(control.Update({KEY_W,KEY_S},.001,2)));
    d=control.Update({KEY_W,KEY_A,KEY_R,KEY_Q},.001,2.01);
    double norm=std::sqrt(d[0][0]*d[0][0]+d[0][1]*d[0][1]+d[0][2]*d[0][2]);
    assert(std::abs(norm-.00003)<1e-12 && d[0][5]==.00015);
    assert(Zero(control.Update({KEY_SPACE},.001,2.02)) && !control.Ready());
    assert(Zero(control.Update({KEY_SPACE},.001,2.03)) && control.Ready());
    assert(Zero(control.Update({},.001,2.04))); // no replay after pause
    std::vector<int> burst(100,KEY_W);
    total=control.Update(burst,.001,3);
    for (int i=0;i<100;++i) { total[0][0]+=control.Update({},.001,3.001+i*.001)[0][0]; }
    assert(std::abs(total[0][0]-.001)<1e-12); // queue never exceeds one step
    control.Update({KEY_W},.001,4);
    assert(Zero(control.Update({},.06,4.06)) && !control.Ready());
    assert(Zero(control.Update({KEY_SPACE,KEY_W},.001,4.07,false)));
    control.Update({KEY_SPACE},.001,5);
    assert(Zero(control.Update({KEY_ESC},.001,5.01)));
    Control single(false,.03,.15);
    d=single.Update({KEY_SPACE,KEY_W,KEY_UP},.001,1);
    assert(d[0][0]>0 && d[1]==Delta{});
    for (double bad : {0.,-.1,.11,std::numeric_limits<double>::infinity()}) {
        bool failed=false; try { Control invalid(true,bad,.15); } catch (...) { failed=true; }
        assert(failed);
    }
}
void Switches() {
    Control control(true,.03,.15);
    auto states=std::array<Feedback,2>{Valid(),Valid()};
    control.Update({KEY_SPACE,KEY_W,KEY_UP},.001,1);
    assert(Zero(control.Update({KEY_TAB},.001,1.01)) && control.Switching() && !control.Ready());
    states[1].velocities[2]=.1;
    assert(!control.TrySwitch(states,1.02)); // other arm must stop too
    states[1].velocities[2]=0;
    assert(!control.TrySwitch(states,1.03));
    assert(!control.TrySwitch(states,1.15)); // repeated sample is not fresh
    states[0].timestamp+=1; states[1].timestamp+=1;
    assert(control.TrySwitch(states,1.16));
    assert(control.CurrentSpace()==Space::Joint && !control.Ready());
    assert(Zero(control.Update({KEY_W},.001,1.17)));
    auto d=control.Update({KEY_SPACE,KEY_W,KEY_UP},.001,1.18);
    assert(d[0][0]==.00015 && d[1][0]==.00015);
    control.Update({KEY_TAB},.001,2.1);
    assert(!control.TrySwitch(states,2.11));
    states[0].timestamp+=1; states[1].timestamp+=1;
    assert(!control.TrySwitch(states,2.12));
    states[0].timestamp+=1; states[1].timestamp+=1;
    assert(control.TrySwitch(states,2.24));
    assert(control.CurrentSpace()==Space::Cartesian && !control.Ready());
    assert(Zero(control.Update({KEY_SPACE},.001,2.25))); // old joint increment discarded
}
void Guards() {
    std::array<Feedback,2> states{Valid(),Valid()};
    Control control(true,.03,.15);
    assert(control.Check(states,1).empty()); // robot/local clocks intentionally far apart
    states[0].timestamp+=1;
    assert(control.Check(states,1.2).empty());
    states[0].timestamp+=1;
    assert(control.Check(states,1.26).find("arm 2")!=std::string::npos);
    for (int condition=0;condition<8;++condition) {
        Control gate(true,.03,.15); states={Valid(),Valid()};
        assert(gate.Check(states,1).empty());
        auto &s=states[1];
        switch(condition) {
        case 0: s.fault=true; break;
        case 1: s.sdk_source=false; break;
        case 2: s.kinematics=false; break;
        case 3: s.connected=false; break;
        case 4: s.valid=false; break;
        case 5: s.reference="other"; break;
        case 6: s.tool_offset[2]=.1; break;
        case 7: s.tool_offset[2]=std::numeric_limits<double>::quiet_NaN(); break;
        }
        assert(!gate.Check(states,1.001).empty());
    }
}
void SessionCheck(const std::string &scenario) {
    Options options; options.dual=scenario!="single"; options.tool=true;
    if (scenario=="partial_enable") { fail_enable=1; }
    if (scenario=="occupied") { initial_source=CommandSource::TeachPendant; }
    bool failed=false;
    {
        Session session(options);
        try {
            session.Prepare(options);
            assert(scenario!="partial_enable" && scenario!="occupied");
            assert(created==(options.dual?2:1));
            assert(records[0].instance==(options.dual?RobotArmInstance::Left:RobotArmInstance::Single));
            if (options.dual) { assert(records[1].instance==RobotArmInstance::Right); }
            Control control(options.dual,.03,.15);
            if (scenario=="send_failure") { fail_send=1; }
            session.Send(control.Update({KEY_SPACE,KEY_W,KEY_UP},.001,1));
            for (int arm=0;arm<created;++arm) {
                assert(records[arm].writes.back().dx>0);
                assert(records[arm].frame==rcore::CartesianJogFrame::Tool);
            }
            session.Send(control.Update({KEY_SPACE},.001,1.01));
            for (int arm=0;arm<created;++arm) { assert(ZeroPose(records[arm].writes.back())); }
            if (scenario=="joint") {
                auto states=session.States();
                states[0].positions.fill(.4); states[1].positions.fill(-.6);
                session.SetSpace(Space::Joint,states);
                session.Send({});
                assert(records[0].joint_writes.back()[0]==.4 && records[1].joint_writes.back()[0]==-.6);
                Commands inc{}; inc[0][0]=.001; inc[1][1]=-.001;
                session.Send(inc);
                auto held=records[0].joint_writes.back(); session.Send({});
                assert(records[0].joint_writes.back()==held);
                assert(records[1].joint_writes.back()[1]==-.601);
                inc[0][0]=10; session.Send(inc);
                assert(records[0].joint_writes.back()[0]==3); // cabinet limit snapshot
                inc[1][1]=std::numeric_limits<double>::quiet_NaN();
                auto before=records[0].joint_writes.size();
                bool rejected=false; try { session.Send(inc); } catch (...) { rejected=true; }
                assert(rejected && records[0].joint_writes.size()==before);
                before=records[0].writes.size();
                session.Close();
                assert(records[0].writes.size()==before); // cleanup keeps ServoJ fixed hold
                assert(records[0].joint_writes.back()[0]==3);
            }
            if (scenario=="disable_failure") { fail_disable=0; }
            if (scenario=="disable_unconfirmed") { stuck_enabled=0; }
            assert(session.Close()==(scenario!="disable_failure" && scenario!="disable_unconfirmed"));
        } catch (...) { failed=true; }
    }
    assert(failed==(scenario=="partial_enable" || scenario=="send_failure" || scenario=="occupied"));
    for (int arm=0;arm<created;++arm) {
        assert(records[arm].shutdowns==1);
        if (records[arm].enables) {
            assert(records[arm].disables==1 && records[arm].stops==0);
            if (arm!=fail_send) { assert(!records[arm].writes.empty() && ZeroPose(records[arm].writes.back())); }
        }
    }
    if (scenario=="occupied") { assert(records[0].enables==0); }
}
void Parser() {
    for (const auto &args : std::vector<std::vector<std::string>>{
         {"app"}, {"app","--enable-motion"},
         {"app","--enable-motion","--local","--robot-ip","x"},
         {"app","--dry-run","--robot-ip","x"}, {"app","--list-keyboards"},
         {"app","--speed","nan"},{"app","--angular-speed","1"},{"app","--frame","world"}}) {
        std::vector<char*> argv;
        for (const auto &arg:args) { argv.push_back(const_cast<char*>(arg.c_str())); }
        bool failed=false; try { Parse(argv.size(),argv.data()); } catch (...) { failed=true; }
        assert(failed && created==0);
    }
    std::vector<std::string> args={"app","--enable-motion","--local","--dual-arm"};
    std::vector<char*> argv;
    for (auto &arg:args) { argv.push_back(arg.data()); }
    auto options=Parse(argv.size(),argv.data());
    assert(options.dual && options.local && options.motion && options.speed==.03);
}
int main(int argc,char **argv) {
    assert(argc==2); std::string test=argv[1];
    if (test=="controls") { Controls(); }
    else if (test=="switches") { Switches(); }
    else if (test=="feedback") { Guards(); }
    else if (test=="parser") { Parser(); }
    else { SessionCheck(test); }
}
'''


class KeyboardTeleopTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.tmp.cleanup)
        path = Path(cls.tmp.name)
        (path / 'sdk').mkdir()
        (path / 'sdk/robot.hpp').write_text(SDK)
        (path / 'check.cpp').write_text(CHECK)
        cls.binary = path / 'check'
        root = Path(__file__).resolve().parents[1]
        subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pthread',
                        '-I' + str(path), '-I' + str(root), str(path / 'check.cpp'), str(root / 'app/keyboard_input.cpp'),
                        '-o', str(cls.binary), *desktop_build_flags(root)], check=True)

    def test_space_switch(self):
        self.run_case('switches')

    def test_key_combinations_and_release(self):
        self.run_case('controls')

    def test_feedback_guards(self):
        self.run_case('feedback')

    def test_arguments_before_connection(self):
        self.run_case('parser')

    def test_single_and_dual_sessions(self):
        for case in ('single', 'dual', 'joint'):
            with self.subTest(case=case):
                self.run_case(case)

    def test_partial_failures_cleanup_both_arms(self):
        for case in ('partial_enable', 'send_failure', 'disable_failure', 'disable_unconfirmed', 'occupied'):
            with self.subTest(case=case):
                self.run_case(case)

    def run_case(self, name):
        subprocess.run([str(self.binary), name], check=True, timeout=10,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE)


if __name__ == '__main__':
    unittest.main()
