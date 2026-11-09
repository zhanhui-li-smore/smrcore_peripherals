"""Exercise production dual-arm orchestration with real FDCC/impedance workers, no DDS."""
from pathlib import Path
import subprocess
import tempfile
import unittest
import re

from .test_pico_fdcc import FAKE_SDK as FDCC_SDK


# Combined mock exposes both APIs and counts them to detect wrong-mode writes.
FAKE_SDK = FDCC_SDK.replace('namespace rcore {', 'namespace rcore { namespace sdk { enum class CommandSource { Sdk, TeachPendant }; }')
FAKE_SDK = FAKE_SDK.replace('double timestamp=1;', 'sdk::CommandSource command_source=sdk::CommandSource::Sdk; double timestamp=1;')
FAKE_SDK = FAKE_SDK.replace('struct Robot {', r'''
struct CartesianImpedanceParams {
 std::array<double,6> stiffness{}, damping{}, desired_wrench{};
};
struct Robot {
 std::atomic<int> fdcc_calls{0},impedance_calls{0};
 std::atomic<bool> owned{true};
 CartesianImpedanceParams received;
 Result EnableCartesianImpedance(CartesianImpedanceParams p){
  received=p;++impedance_calls;++enabled;mode=5;return {!fail_enable};
 }
 Result SetCartesianImpedanceTeleopSource(TeleopInputSource source){
  assert(mode==5 && source==TeleopInputSource::Manual);++manual;return {};
 }
 void SetCartesianImpedanceTarget(Pose p){
  assert(mode==5 && manual>0);++impedance_calls;in_pose=true;
  std::this_thread::sleep_for(std::chrono::milliseconds(pose_delay_ms.load()));
  if(fail_pose){in_pose=false;throw std::runtime_error("DDS failed");}
  {std::lock_guard<std::mutex> l(mutex);poses.push_back(p);}++sent;in_pose=false;
 }
 Result DisableCartesianImpedance(){
  ++exited;std::this_thread::sleep_for(std::chrono::milliseconds(exit_delay_ms.load()));
  if(!fail_exit){mode=0;}return {!fail_exit};
 }
''')
FAKE_SDK = FAKE_SDK.replace('RobotState s;', 'RobotState s; s.command_source=owned?CommandSource::Sdk:CommandSource::TeachPendant;')
FAKE_SDK = FAKE_SDK.replace('mode=6; return', '++fdcc_calls;mode=6; return')


PREAMBLE = r'''
#include <cassert>
#include <cmath>
#include <iostream>
#include "pico_fdcc.hpp"
#include "pico_impedance.hpp"
#include <memory>
#include "peripherals/pico/teleop.hpp"
using smrcore::peripherals::PicoControlMode;
using smrcore::peripherals::detail::PicoImpedance;
using Options = smrcore::peripherals::PicoTeleopOptions;
using smrcore::peripherals::detail::PicoFdcc;
using smrcore::peripherals::detail::PicoCompliance;
using Clock = PicoCompliance::Clock;
#ifdef TEST_IMPEDANCE
constexpr auto test_mode = PicoControlMode::CartesianImpedance;
constexpr int active_mode=5;
#else
constexpr auto test_mode = PicoControlMode::Fdcc;
constexpr int active_mode=6;
#endif
Options TestOptions(){Options o;o.control_mode=test_mode;if(test_mode==PicoControlMode::Fdcc){o.fdcc_damping_factor=.7;o.fdcc_error_scale=2;}return o;}

constexpr double kGripReleaseThreshold=.25, kInputStaleSeconds=.40;
double Now(){return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
struct PicoHeadsetPose {};
struct PicoControllerPose {
 bool present=true,tracked=true; double grip=1;
 std::array<double,2> thumbstick{1,1};
};
struct PicoSample { PicoHeadsetPose headset; double timestamp_sec=Now(); };
struct ArmReference {};
struct SingularitySlowdownStatus { bool fresh=true; };
struct HandServoState {
 bool active=false,servop_stream_started=false,stop_requested=false,require_release=false;
 Eigen::Isometry3d target_tcp_reference=Eigen::Isometry3d::Identity();
};
void FreezeHand(const rcore::RobotState&,HandServoState& h,bool release) {
 h.active=false;h.servop_stream_started=false;h.require_release|=release;
}
bool GetTcpInReference(const rcore::RobotState& s,Eigen::Isometry3d& t){
 t=Eigen::Isometry3d::Identity();
 t.translation()=Eigen::Vector3d(s.cartesian_position[0],s.cartesian_position[1],s.cartesian_position[2]);
 return t.matrix().allFinite();
}
// Mapping is tested separately. This seam checks that the production dual FDCC
// path suppresses J6 and never maps while waiting for mode exit or rearm.
void UpdateHandTarget(const PicoControllerPose& p,const PicoHeadsetPose&,
 const rcore::RobotState&,const ArmReference&,bool,bool,double,double,
 double j6,bool,const SingularitySlowdownStatus&,HandServoState& h){
 assert(p.thumbstick[0]==0 && p.thumbstick[1]==0 && j6==0);
 if(p.grip<=kGripReleaseThreshold){h.stop_requested=h.active;h.active=false;h.require_release=false;return;}
 if(!h.require_release){h.active=true;h.servop_stream_started=true;}
}
struct PicoTargetGuard {
 struct Checked {
  std::array<double,6> joints{};
  bool ready=true,pending=false;std::string error,blocked_reason;
  long long completed_ns=0;
  Eigen::Isometry3d target=Eigen::Isometry3d::Identity(),raw_target=target;
 } checked;
 bool reject=false,delayed=false;
 void Cancel(){checked={};checked.ready=false;}
 void Submit(const HandServoState& h,const Eigen::Isometry3d* raw){
  checked.target=h.target_tcp_reference;checked.raw_target=*raw;
  checked.ready=!delayed;checked.pending=delayed;
  checked.blocked_reason=reject?"candidate IK rejected":"";
  checked.completed_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
 }
 Checked Snapshot(){return checked;}
};
'''

SCENARIOS = r'''
template<class F> void Until(F f){
 auto end=Clock::now()+std::chrono::seconds(3);
 while(!f()){assert(Clock::now()<end);std::this_thread::sleep_for(std::chrono::milliseconds(2));}
}
struct Rig {
 Options options=TestOptions();rcore::sdk::Robot l,r;DualPicoCompliance pair{l,r,options};
 PicoSample packet;PicoControllerPose lc,rc;ArmReference reference;
 SingularitySlowdownStatus ls,rs;HandServoState lh,rh;PicoTargetGuard lg,rg;
 Rig(){lh.target_tcp_reference.translation().x()=.4;rh.target_tcp_reference.translation().x()=.8;r.x=.7;}
 void Tick(bool fresh=true){
  packet.timestamp_sec=Now();
  assert(TickDualComplianceArm(options,packet,lc,fresh,true,l.GetState(),reference,ls,lh,lg,pair.left));
  assert(TickDualComplianceArm(options,packet,rc,fresh,true,r.GetState(),reference,rs,rh,rg,pair.right));
 }
 void Active(){Until([&]{Tick();return l.sent>0 && r.sent>0;});}
};
int main(){
 // Production input deadline accepts a 300ms-old packet but rejects >400ms.
 // Target approval stays fresh in this test, independently of input age.
 {
  Rig x;x.Active();
  x.packet.timestamp_sec=Now()-.30;
  assert(TickDualComplianceArm(x.options,x.packet,x.lc,true,true,x.l.GetState(),x.reference,x.ls,x.lh,x.lg,x.pair.left));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  assert(!x.pair.left.Snapshot().rearm_required && x.l.mode==active_mode);
  x.packet.timestamp_sec=Now()-.45;
  assert(TickDualComplianceArm(x.options,x.packet,x.lc,true,true,x.l.GetState(),x.reference,x.ls,x.lh,x.lg,x.pair.left));
  Until([&]{return x.pair.left.Snapshot().rearm_required;});
  assert(x.pair.left.Snapshot().pause_reason.find("Pico input deadline expired")!=std::string::npos);
  x.pair.Stop();
 }
 // Both workers initialize from their own measured TCP and independent slots.
 // Releasing the left Grip must leave right-hand following active.
 {
  Rig x;x.Active();
  if(test_mode==PicoControlMode::CartesianImpedance){
   assert(x.l.fdcc_calls==0 && x.r.fdcc_calls==0 && x.l.zero==0 && x.r.zero==0);
   assert(x.l.received.stiffness==x.options.impedance_stiffness);
   assert(x.r.received.damping==x.options.impedance_damping);
   for(double v:x.l.received.desired_wrench){assert(v==0);}
  }else{
   assert(x.l.impedance_calls==0 && x.r.impedance_calls==0);
   assert(x.l.received_damping_factor==.7 && x.r.received_damping_factor==.7);
   assert(x.l.received_error_scale==2 && x.r.received_error_scale==2);
  }
  {std::lock_guard<std::mutex> lock(x.l.mutex);assert(x.l.poses.front().t[0]==.3);}
  {std::lock_guard<std::mutex> lock(x.r.mutex);assert(x.r.poses.front().t[0]==.7);}
  x.lc.grip=0;
  Until([&]{x.Tick();return x.pair.left.Snapshot().phase==PicoCompliance::Phase::Holding;});
  auto lsent=x.l.sent.load(),rsent=x.r.sent.load();
  Until([&]{x.Tick();return x.r.sent>rsent+2;});assert(x.l.sent==lsent && x.r.mode==active_mode);
  x.l.x=.5;x.lc.grip=1;
  Until([&]{x.Tick();return x.l.sent>lsent;});
  {std::lock_guard<std::mutex> lock(x.l.mutex);assert(x.l.poses.at(lsent).t[0]==.5);}
  x.pair.Stop();assert(x.l.mode==0 && x.r.mode==0 && x.pair.Healthy());
 }
 // Tracking loss/Jacobian expiry/candidate rejection affects only that arm.
 // Recovery with Grip held never re-enables it; a fresh release is required.
 for(int reason=0;reason<3;++reason){
  Rig x;x.Active();
  if(reason==0){x.lc.tracked=false;}else if(reason==1){x.ls.fresh=false;}else{x.lg.reject=true;}
  Until([&]{x.Tick();return x.pair.left.Snapshot().phase==PicoCompliance::Phase::Holding && x.pair.left.Snapshot().rearm_required;});
  x.lc.tracked=true;x.ls.fresh=true;x.lg.reject=false;
  auto rsent=x.r.sent.load();
  Until([&]{x.Tick();return x.r.sent>rsent+2;});assert(x.l.enabled==1 && x.l.mode==0);
  x.lc.grip=0;x.Tick();assert(!x.pair.left.Snapshot().rearm_required);
  x.lc.grip=1;Until([&]{x.Tick();return x.l.enabled==2 && x.l.mode==active_mode;});
  x.pair.Stop();assert(x.pair.Healthy());
 }
 // Shared packet loss cancels both arms and keeps held Grip from resuming.
 {
  Rig x;x.Active();x.Tick(false);
  Until([&]{return x.l.mode==0 && x.r.mode==0;});
  for(int i=0;i<20;++i){x.Tick();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  assert(x.l.enabled==1 && x.r.enabled==1);
  x.lc.grip=0;x.rc.grip=0;
  Until([&]{x.Tick();return !x.pair.left.Snapshot().rearm_required && !x.pair.right.Snapshot().rearm_required;});
  x.lc.grip=1;x.rc.grip=1;
  Until([&]{x.Tick();return x.l.enabled==2 && x.r.enabled==2;});
 }
 // Slow left RPC does not serialize right updates. Stop invalidates BOTH
 // streams before joining the slower one (at most one in-flight write remains).
 {
  Rig x;x.l.pose_delay_ms=150;x.Active();
  Until([&]{x.Tick();return x.l.in_pose.load();});
  auto before=x.r.sent.load();
  Until([&]{x.Tick();return x.r.sent>=before+2;});
  before=x.r.sent.load();x.pair.Stop();assert(x.r.sent<=before+1);
  assert(x.l.mode==0 && x.r.mode==0);
 }
 // Either arm's RPC failure stops the whole session, without Servo fallback.
 for(bool left:{false,true}){
  Rig x;x.Active();(left?x.l:x.r).fail_pose=true;
  Until([&]{return !x.pair.Healthy();});
  x.pair.Stop();assert(x.l.mode==0 && x.r.mode==0);
 }
 // SDK ownership loss is terminal for either impedance arm.
 if(test_mode==PicoControlMode::CartesianImpedance){
  for(bool left:{false,true}){
   Rig x;x.Active();(left?x.l:x.r).owned=false;
   Until([&]{return !x.pair.Healthy();});x.pair.Stop();
   // Ownership loss also blocks feedback acknowledgement, so cleanup may
   // require StopMotion/Disable; it must never reactivate either mode.
   assert(x.l.enabled==1 && x.r.enabled==1);
  }
  // Impedance targets never use FDCC virtual extrapolation.
  Rig x;x.options.fdcc_target_scale=3;x.Active();
  assert(x.lg.checked.target.matrix().isApprox(x.lg.checked.raw_target.matrix()));
  assert(x.rg.checked.target.matrix().isApprox(x.rg.checked.raw_target.matrix()));
 }
 // Teardown on exception still exits both modes.
 rcore::sdk::Robot l,r;Options opts=TestOptions();
 try {
  DualPicoCompliance pair(l,r,opts);
  auto target=Eigen::Isometry3d::Identity();
  pair.left.Submit(target,Clock::now()+std::chrono::seconds(2));
  pair.right.Submit(target,Clock::now()+std::chrono::seconds(2));
  Until([&]{return l.sent>0 && r.sent>0;});throw 1;
 }catch(int){}
 assert(l.mode==0 && r.mode==0);
}
'''


class DualFdccTest(unittest.TestCase):
    def test_dual_fdcc(self):
        self.run_orchestration(False)

    def test_dual_impedance(self):
        self.run_orchestration(True)

    def run_orchestration(self, impedance):
        root = Path(__file__).resolve().parents[1]
        source = (root / 'teleop/pico_teleop.cpp').read_text()
        input_timeout = re.search(r'constexpr double kInputStaleSeconds = ([0-9.]+);', source).group(1)
        preamble = PREAMBLE.replace('kInputStaleSeconds=.40', 'kInputStaleSeconds=' + input_timeout)
        begin = source.index('class DualPicoCompliance\n')
        end = source.index('int RunSession(', begin)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / 'sdk').mkdir()
            (path / 'sdk/robot.hpp').write_text(FAKE_SDK)
            (path / 'check.cpp').write_text(preamble + source[begin:end] + SCENARIOS)
            subprocess.run(['g++', '-std=c++17', '-pthread', '-Wall', '-Wextra',
                            *(['-DTEST_IMPEDANCE'] if impedance else []),
                            '-I' + str(path), '-I' + str(root / 'teleop'),
                            '-I' + str(root / 'src'), '-I/usr/include/eigen3',
                            str(path / 'check.cpp'), '-o', str(path / 'check')], check=True)
            subprocess.run([str(path / 'check')], check=True, timeout=20)


if __name__ == '__main__':
    unittest.main()
