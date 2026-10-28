"""Exercise the real impedance adapter against a fault-injectable SDK, no DDS."""
from pathlib import Path
import subprocess
import tempfile
import unittest


FAKE_SDK = r'''
#pragma once
#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
namespace rcore {
namespace sdk { enum class CommandSource { Unclaimed, Sdk, TeachPendant }; }
enum class TeleopInputSource { Manual, SpaceMouse };
struct Errors { int count=0; int GetLatchedErrorCount() const { return count; } };
struct RobotState {
 double timestamp=1; int control_mode=0; bool cartesian_valid=true;
 std::string reference_frame="base"; Errors errors;
 sdk::CommandSource command_source=sdk::CommandSource::Sdk;
 std::array<double,3> cartesian_position{},cartesian_orientation{};
};
namespace sdk {
struct Pose { std::array<double,3> t{},r{}; auto& tvec(){return t;} auto& rvec(){return r;} };
struct Result {
 bool ok=true; bool IsSuccess()const{return ok;} int GetErrorCode()const{return ok?0:4311;}
 std::string GetErrorMsg()const{return "injected impedance failure";}
};
struct CartesianImpedanceParams {
 std::array<double,6> stiffness{120,120,120,20,20,20}, damping{20,20,20,1,1,1};
 std::array<double,6> desired_wrench{};
 bool enable_task_damping=true;
 double position_error_saturation=.03, orientation_error_saturation=.25;
};
struct Robot {
 std::atomic<int> mode{0},enabled{0},sent{0},exited{0},disabled{0},stops{0},manual{0};
 std::atomic<int> reported_mode{-1},enable_delay_ms{0},exit_delay_ms{0},send_delay_ms{0};
 std::atomic<bool> connected{true},fault{false},freeze_stamp{false},bad_tcp{false},owned{true};
 std::atomic<bool> fail_enable{false},fail_send{false},fail_exit{false},fail_manual{false};
 std::atomic<bool> in_send{false};
 std::atomic<double> x{.3};
 std::mutex mutex; std::vector<Pose> poses;
 std::vector<std::chrono::steady_clock::time_point> writes;
 CartesianImpedanceParams received;
 Robot& Control(){return *this;} Robot& Motion(){return *this;}
 bool IsConnected(){return connected;}
 RobotState GetState(){
  RobotState s; s.control_mode=reported_mode>=0?reported_mode.load():mode.load();
  s.errors.count=fault?1:0;
  s.command_source=owned?CommandSource::Sdk:CommandSource::TeachPendant;
  s.timestamp=freeze_stamp?1:std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  s.cartesian_position={bad_tcp?NAN:x.load(),-.2,.1}; return s;
 }
 Result EnableCartesianImpedance(CartesianImpedanceParams p){
  { std::lock_guard<std::mutex> lock(mutex); received=p; }
  ++enabled; std::this_thread::sleep_for(std::chrono::milliseconds(enable_delay_ms.load()));
  mode=5; return {!fail_enable};
 }
 Result SetCartesianImpedanceTeleopSource(TeleopInputSource source){
  assert(mode==5 && source==TeleopInputSource::Manual); ++manual; return {!fail_manual};
 }
 void SetCartesianImpedanceTarget(Pose p){
  assert(mode==5 && manual>0); in_send=true;
  std::this_thread::sleep_for(std::chrono::milliseconds(send_delay_ms.load()));
  if(fail_send){in_send=false;throw std::runtime_error("DDS writer failed");}
  { std::lock_guard<std::mutex> lock(mutex); poses.push_back(p); writes.push_back(std::chrono::steady_clock::now()); }
  ++sent; in_send=false;
 }
 Result DisableCartesianImpedance(){
  ++exited; std::this_thread::sleep_for(std::chrono::milliseconds(exit_delay_ms.load()));
  if(!fail_exit){mode=0;} return {!fail_exit};
 }
 Result Disable(){++disabled;return {};}
 void StopMotion(){++stops;}
 // No FDCC APIs, ServoP, ServoJ, gains/force-source persistence or safety setters.
};
}}
'''

HARNESS = r'''
#include "pico_impedance.hpp"
#include <limits>
using namespace smrcore::peripherals::detail;
using Clock=std::chrono::steady_clock;
using Phase=PicoCompliance::Phase;
const std::array<double,6> k{150,160,170,15,16,17}, d{21,22,23,1,2,3};
auto Target(double x=1.) {Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.translation().x()=x;return t;}
template<class F> void Until(F f){
 auto end=Clock::now()+std::chrono::seconds(3);
 while(!f()){assert(Clock::now()<end);std::this_thread::sleep_for(std::chrono::milliseconds(2));}
}
void Submit(PicoCompliance& w,double x=1.){w.Submit(Target(x),Clock::now()+std::chrono::seconds(2));}
int main(){
 // Actual mode-5 target API, exact K/D, zero feedforward wrench and measured seed.
 {
  rcore::sdk::Robot r; PicoImpedance w(r,k,d,.3,1.2,24); Submit(w);
  Until([&]{return w.Snapshot().phase==Phase::Active;});
  { std::lock_guard<std::mutex> lock(r.mutex);
    assert(r.received.stiffness==k && r.received.damping==d);
    for(double v:r.received.desired_wrench){assert(v==0);}
    assert(r.received.enable_task_damping && r.received.position_error_saturation==.03);
    assert(r.poses.front().t[0]==.3 && r.poses.front().t[1]==-.2);
  }
  auto end=Clock::now()+std::chrono::milliseconds(230);
  while(Clock::now()<end){Submit(w);std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  assert(w.Snapshot().target_speed_mps>.15 && w.Snapshot().target_speed_mps<=.3+1e-9);
  w.Pause();Until([&]{return w.Snapshot().phase==Phase::Holding;});
  assert(r.mode==0 && r.exited==1 && r.disabled==0);
  const auto count=r.sent.load();std::this_thread::sleep_for(std::chrono::milliseconds(40));assert(r.sent==count);
  r.x=.6;Submit(w);Until([&]{return w.Snapshot().phase==Phase::Active;});w.Stop();
  assert(r.exited==2 && r.mode==0 && w.Snapshot().error.empty());
  { std::lock_guard<std::mutex> lock(r.mutex);
    assert(r.poses.at(count).t[0]==.6);
    for(size_t i=1;i<r.writes.size();++i){assert(r.writes[i]-r.writes[i-1]>=std::chrono::milliseconds(5));}
  }
 }
 // Release cancels in-flight Enable; re-grip cannot skip damping exit.
 {
  rcore::sdk::Robot r;r.enable_delay_ms=80;r.exit_delay_ms=100;PicoImpedance w(r,k,d);Submit(w);
  Until([&]{return r.enabled==1;});w.Pause();Submit(w);
  Until([&]{return r.exited==1;});assert(w.Snapshot().phase==Phase::Stopping);Submit(w);
  Until([&]{return w.Snapshot().phase==Phase::Holding;});
  assert(r.sent==0 && r.enabled==1);w.Stop();
 }
 // Never publish a target before the mode acknowledgement, or after lost ownership.
 {
  rcore::sdk::Robot r;r.reported_mode=0;PicoImpedance w(r,k,d);Submit(w);
  Until([&]{return r.enabled==1;});std::this_thread::sleep_for(std::chrono::milliseconds(60));assert(r.sent==0);
  r.reported_mode=-1;Until([&]{return r.sent>0;});r.owned=false;
  Until([&]{return !w.Snapshot().error.empty();});w.Stop();assert(r.enabled==1);
 }
 {
  rcore::sdk::Robot r;r.reported_mode=0;PicoImpedance w(r,k,d);Submit(w);
  Until([&]{return !w.Snapshot().error.empty();});w.Stop();assert(r.sent==0 && r.exited>=1);
 }
 // Failure may follow successful activation; cleanup is already armed.
 for(int failure:{0,1,2,3}){
  rcore::sdk::Robot r;r.fail_enable=failure==0;r.fail_manual=failure==1;r.fail_send=failure==2;
  PicoImpedance w(r,k,d);Submit(w);
  if(failure==3){Until([&]{return r.sent>0;});r.fail_exit=true;w.Pause();}
  Until([&]{return !w.Snapshot().error.empty();});w.Stop();assert(r.exited>0);
  if(failure==3){assert(r.disabled>0 && r.stops>0);}else{assert(r.mode==0);}
 }
 // Mode loss, disconnection, latched fault, nonfinite and frozen feedback end session.
 for(int failure:{0,1,2,3,4}){
  rcore::sdk::Robot r;PicoImpedance w(r,k,d);Submit(w);Until([&]{return r.sent>0;});
  if(failure==0){r.mode=0;}
  else if(failure==1){r.connected=false;}
  else if(failure==2){r.fault=true;}
  else if(failure==3){r.bad_tcp=true;}
  else{r.freeze_stamp=true;}
  Until([&]{return !w.Snapshot().error.empty();});w.Stop();assert(r.enabled==1);
 }
 // No implicit takeover of another mode, even though SDK exposes mode setters.
 {
  rcore::sdk::Robot r;r.mode=6;PicoImpedance w(r,k,d);Submit(w);
  Until([&]{return !w.Snapshot().error.empty();});w.Stop();assert(r.enabled==0 && r.exited==0);
 }
 // Approval expiry pauses with a release latch, exactly like FDCC. It does not
 // disable motors or terminate; a fresh target alone cannot restart motion.
 {
  rcore::sdk::Robot r;PicoImpedance w(r,k,d);
  w.Submit(Target(),Clock::now()+std::chrono::milliseconds(60));
  Until([&]{return w.Snapshot().rearm_required && w.Snapshot().phase==Phase::Holding;});
  const auto count=r.sent.load();
  assert(w.Snapshot().error.empty() && r.mode==0 && r.disabled==0);
  Submit(w);std::this_thread::sleep_for(std::chrono::milliseconds(30));
  assert(r.enabled==1 && r.sent==count);
  assert(w.AcknowledgeGripRelease());r.x=.6;Submit(w);
  Until([&]{return w.Snapshot().phase==Phase::Active;});
  {std::lock_guard<std::mutex> lock(r.mutex);assert(r.poses.at(count).t[0]==.6);}
  w.Stop();assert(w.Snapshot().error.empty());
 }
 // NaN rejects before activation; there is no fallback motion command.
 {
  rcore::sdk::Robot r;PicoImpedance w(r,k,d);auto t=Target();t.translation()[0]=NAN;
  bool rejected=false;try{w.Submit(t,Clock::now());}catch(...){rejected=true;}
  assert(rejected && r.enabled==0);w.Stop();
 }
 // An already-entered write can finish; release cancels all unsent replacements.
 {
  rcore::sdk::Robot r;r.send_delay_ms=80;PicoImpedance w(r,k,d);Submit(w);
  Until([&]{return r.in_send.load();});w.Pause();
  Until([&]{return w.Snapshot().phase==Phase::Holding;});assert(r.sent==1);w.Stop();
 }
 {
  rcore::sdk::Robot r;r.send_delay_ms=240;PicoImpedance w(r,k,d);Submit(w);
  Until([&]{return !w.Snapshot().error.empty();});w.Stop();assert(r.sent==1);
 }
 // K/D engineering bounds and nonfinite rejection before a worker can activate.
 for(size_t axis=0;axis<6;++axis){
  for(double invalid:{-1.0,std::numeric_limits<double>::infinity(),std::nan("")}){
   auto bad=k;bad[axis]=invalid;assert(!ValidImpedanceGains(bad,d));
   bad=d;bad[axis]=invalid;assert(!ValidImpedanceGains(k,bad));
  }
  auto bad=k;bad[axis]=axis<3?2001:151;assert(!ValidImpedanceGains(bad,d));
  bool rejected=false;rcore::sdk::Robot r;
  try{PicoImpedance w(r,bad,d);}catch(...){rejected=true;}
  assert(rejected && r.enabled==0);
 }
}
'''


class PicoImpedanceTest(unittest.TestCase):
    def test_worker_lifecycle(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / 'sdk').mkdir()
            (path / 'sdk/robot.hpp').write_text(FAKE_SDK)
            (path / 'check.cpp').write_text(HARNESS)
            subprocess.run(['g++', '-std=c++17', '-pthread', '-Wall', '-Wextra',
                            '-I' + str(path), '-I' + str(root / 'teleop'),
                            '-I/usr/include/eigen3', str(path / 'check.cpp'),
                            '-o', str(path / 'check')], check=True)
            subprocess.run([str(path / 'check')], check=True, timeout=25)

    def test_cli(self):
        root = Path(__file__).resolve().parents[1]
        # Compile the full CLI with a non-networking Run stub. Check parsed values
        # directly, including the default mode and existing FDCC option path.
        source = ('#define main pico_cli_main\n#include "' +
                  str(root / 'app/app_peripherals_pico_bridge.cpp') + '"\n#undef main\n' + r'''
#include <cassert>
#include <sstream>
namespace smrcore::peripherals { int PicoTeleop::Run(const PicoTeleopOptions &){return 0;} }
int main(){
 Options o;
 auto parse=[&](std::initializer_list<const char*> args){
  std::vector<char*> argv{const_cast<char*>("bridge")};
  for(auto a:args){argv.push_back(const_cast<char*>(a));}
  return ParseOptions(static_cast<int>(argv.size()),argv.data(),o);
 };
 assert(o.control_mode==smrcore::peripherals::PicoControlMode::ServoP);
 assert(o.fdcc_max_speed==.30 && o.fdcc_max_accel==1.2 && o.fdcc_follow_gain==8);
 assert(o.fdcc_target_feedforward==.5 && o.fdcc_target_scale==1);
 assert(o.impedance_max_speed==.15 && o.impedance_max_accel==.6);
 assert(parse({"--single-arm","--robot-ip","192.0.2.1","--control-mode","fdcc",
  "--single-mapping-yaw","-90","--motion-scale","1","--enable-motion","--no-gripper"}));
 assert(o.fdcc_max_speed==.30 && o.fdcc_max_accel==1.2 && o.fdcc_follow_gain==8);
 assert(o.fdcc_target_feedforward==.5 && o.fdcc_target_scale==1);
 assert(o.single_arm && o.enable_motion && !o.enable_gripper);
 assert(o.translation_scale==1 && o.rotation_scale==1 && o.single_mapping_yaw_deg==-90);
 std::ostringstream help;
 auto* saved=std::cout.rdbuf(help.rdbuf());
 PrintUsage("bridge");std::cout.rdbuf(saved);
 assert(help.str().find("--fdcc-follow-gain")==std::string::npos);
 assert(help.str().find("--impedance-stiffness")==std::string::npos);
 assert(help.str().find("--help-advanced")!=std::string::npos);
 help.str("");help.clear();saved=std::cout.rdbuf(help.rdbuf());
 PrintAdvancedUsage("bridge");std::cout.rdbuf(saved);
 assert(help.str().find("--fdcc-follow-gain")!=std::string::npos);
 assert(help.str().find("--impedance-stiffness")!=std::string::npos);
 o=Options{};
 assert(parse({"--single-arm","--control-mode","impedance","--single-mapping-yaw","-90",
  "--impedance-stiffness","100,110,120,10,11,12","--impedance-damping","20,21,22,1,2,3",
  "--impedance-max-speed","0.3","--impedance-max-accel","1.2","--impedance-follow-gain","24"}));
 assert(o.control_mode==smrcore::peripherals::PicoControlMode::CartesianImpedance && o.single_arm);
 assert(o.impedance_stiffness[5]==12 && o.impedance_damping[2]==22 && o.impedance_follow_gain==24);
 assert(o.single_mapping_yaw_deg==-90 && o.impedance_max_speed==.3);
 for(const char* bad:{"1,2,3,4,5","1,2,3,4,5,6,7","1,2,3,4,5,nan","1,2,3,4,5,inf",
                     "1,2,3,4,5,6x","1,,3,4,5,6","1,2,3,4,5,",""}){
  assert(!parse({"--impedance-stiffness",bad}));
 }
 assert(!parse({"--impedance-damping"}));
 assert(!parse({"--impedance-follow-gain","nan"}));
 assert(parse({"--control-mode","fdcc","--fdcc-max-speed",".4"}));
 assert(o.control_mode==smrcore::peripherals::PicoControlMode::Fdcc && o.fdcc_max_speed==.4);
 assert(o.fdcc_target_feedforward==.5);
 assert(parse({"--control-mode","fdcc","--fdcc-target-feedforward","0"}));
 assert(o.fdcc_target_feedforward==0);
 assert(!parse({"--fdcc-target-feedforward","nan"}));
 assert(!parse({"--fdcc-target-feedforward"}));
}
''')
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / 'check.cpp').write_text('#include <vector>\n' + source)
            subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra',
                            '-I' + str(root / 'src'), str(path / 'check.cpp'),
                            '-o', str(path / 'check')], check=True)
            subprocess.run([str(path / 'check')], check=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
