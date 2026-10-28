"""Run the production FDCC worker against a fault-injectable SDK, without DDS."""
from pathlib import Path
import subprocess
import tempfile
import unittest

FAKE_SDK = r'''
#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
namespace rcore {
enum class TeleopInputSource { Manual, SpaceMouse };
struct Errors { int count=0; int GetLatchedErrorCount() const { return count; } };
struct RobotState {
 double timestamp=1; int control_mode=0; bool cartesian_valid=true;
 std::string reference_frame="base"; Errors errors;
 std::array<double,3> cartesian_position{},cartesian_orientation{};
};
namespace sdk {
struct Pose { std::array<double,3> t{},r{}; auto& tvec(){return t;} auto& rvec(){return r;} };
struct Result {
 bool ok=true; bool IsSuccess()const{return ok;} int GetErrorCode()const{return ok?0:4339;}
 std::string GetErrorMsg()const{return "injected failure";}
};
struct FdCartesianAdmittanceParams {
 std::array<double,6> kp={.0005,.0005,.0005,.005,.005,.005};
 std::array<double,6> tracking_error_limit={.1,.1,.1,.1,.1,.1};
 double error_scale=1;
};
struct Robot {
 std::atomic<int> mode{0},enabled{0},sent{0},exited{0},disabled{0},stops{0},manual{0},zero{0};
 std::atomic<bool> connected{true},freeze_stamp{false},fault{false},bad_tcp{false};
 std::atomic<bool> fail_enable{false},fail_pose{false},fail_exit{false};
 std::atomic<int> enable_delay_ms{0},pose_delay_ms{0},exit_delay_ms{0},reported_mode{-1};
 std::atomic<bool> in_pose{false};
 std::atomic<double> x{.3};
 std::mutex mutex; std::vector<Pose> poses;
 Robot& Control(){return *this;} Robot& Motion(){return *this;}
 bool IsConnected(){return connected;}
 RobotState GetState(){
  RobotState s; s.control_mode=reported_mode>=0?reported_mode.load():mode.load(); s.errors.count=fault?1:0;
  s.timestamp=freeze_stamp?1:std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  s.cartesian_position={x.load(),-.2,.1};
  if(bad_tcp){s.cartesian_position[0]=NAN;}
  return s;
 }
 Result EnableFdCartesianAdmittance(FdCartesianAdmittanceParams p){
  assert(p.kp[0]==.0005 && p.error_scale==1 && p.tracking_error_limit[0]==.1);
  ++enabled; std::this_thread::sleep_for(std::chrono::milliseconds(enable_delay_ms.load()));
  mode=6; return {!fail_enable}; // failure may follow successful server activation
 }
 Result SetFdCartesianAdmittanceTeleopSource(TeleopInputSource s){assert(s==TeleopInputSource::Manual);++manual;return {};}
 Result SetFdCartesianAdmittanceWrenchTarget(std::array<double,6> w){for(double x:w){assert(x==0);}++zero;return {};}
 Result SetFdCartesianAdmittancePoseTarget(Pose p){
  assert(mode==6); in_pose=true;
  std::this_thread::sleep_for(std::chrono::milliseconds(pose_delay_ms.load()));
  {std::lock_guard<std::mutex> l(mutex);poses.push_back(p);} ++sent; in_pose=false;
  return {!fail_pose};
 }
 Result DisableFdCartesianAdmittance(){
  ++exited;std::this_thread::sleep_for(std::chrono::milliseconds(exit_delay_ms.load()));
  if(!fail_exit){mode=0;}return {!fail_exit};
 }
 Result Disable(){++disabled;return {};}
 void StopMotion(){++stops;}
 // No ServoP, ServoJ or persistent force-source setter: using one fails to build.
};
}}
'''

HARNESS = r'''
#include <cassert>
#include <cmath>
#include "pico_fdcc.hpp"
using smrcore::peripherals::detail::PicoFdcc;
using Clock=std::chrono::steady_clock;
using Phase=PicoFdcc::Phase;
auto Target(double x=1.) {Eigen::Isometry3d t=Eigen::Isometry3d::Identity();t.translation().x()=x;return t;}
template<class F> void Until(F f){
 auto end=Clock::now()+std::chrono::seconds(3);
 while(!f()){assert(Clock::now()<end);std::this_thread::sleep_for(std::chrono::milliseconds(2));}
}
void Submit(PicoFdcc& d,double x=1.){d.Submit(Target(x),Clock::now()+std::chrono::seconds(2));}
int main(){
 // First target starts at measured TCP. Release holds; re-grip reanchors.
 {
  rcore::sdk::Robot r; PicoFdcc d(r); Submit(d);
  Until([&]{return d.Snapshot().phase==Phase::Active;});
  {std::lock_guard<std::mutex> l(r.mutex);assert(r.poses.front().t[0]==.3);}
  assert(r.manual==1 && r.zero==1);
  d.Pause(); Until([&]{return d.Snapshot().phase==Phase::Holding;});
  assert(r.exited==1 && r.disabled==0);
  auto count=r.sent.load();std::this_thread::sleep_for(std::chrono::milliseconds(40));assert(r.sent==count);
  r.x=.6;Submit(d);Until([&]{return d.Snapshot().phase==Phase::Active;});
  {std::lock_guard<std::mutex> l(r.mutex);assert(r.poses.at(count).t[0]==.6);}
  d.Stop();assert(r.exited==2 && r.mode==0 && d.Snapshot().error.empty());
 }
 // Release while Enable is in flight: no late target or second activation.
 {
  rcore::sdk::Robot r;r.enable_delay_ms=100;PicoFdcc d(r);Submit(d);
  Until([&]{return r.enabled==1;});d.Pause();
  Submit(d); // Stopping rejects new requests until hold feedback arrives.
  Until([&]{return d.Snapshot().phase==Phase::Holding;});
  assert(r.sent==0 && r.enabled==1 && r.exited==1);d.Stop();
 }
 // Mode loss is terminal, not an automatic FDCC re-enable.
 {
  rcore::sdk::Robot r;r.reported_mode=0;PicoFdcc d(r);Submit(d);
  Until([&]{return r.enabled==1;});std::this_thread::sleep_for(std::chrono::milliseconds(60));
  assert(r.sent==0);r.reported_mode=-1;Until([&]{return r.sent>0;});d.Stop();
 }
 // Absent mode acknowledgement times out without writing any pose.
 {
  rcore::sdk::Robot r;r.reported_mode=0;PicoFdcc d(r);Submit(d);
  Until([&]{return !d.Snapshot().error.empty();});d.Stop();assert(r.sent==0 && r.exited>=1);
 }
 {
  rcore::sdk::Robot r;PicoFdcc d(r);Submit(d);Until([&]{return r.sent>0;});r.mode=0;
  Until([&]{return !d.Snapshot().error.empty();});d.Stop();assert(r.enabled==1);
 }
 // Invalid target is rejected before any activation.
 {
  rcore::sdk::Robot r;PicoFdcc d(r);auto t=Target();t.translation()[0]=NAN;
  bool rejected=false;try{d.Submit(t,Clock::now());}catch(...){rejected=true;}
  assert(rejected && r.enabled==0);d.Stop();
 }
 // Failed activation may already have switched the controller: clean it up.
 {
  rcore::sdk::Robot r;r.fail_enable=true;PicoFdcc d(r);Submit(d);
  Until([&]{return !d.Snapshot().error.empty();});d.Stop();assert(r.exited>=1 && r.mode==0 && r.sent==0);
 }
 // Pose RPC failure and pause failure are surfaced; cleanup tries motor disable.
 for(bool exit_failure:{false,true}){
  rcore::sdk::Robot r;r.fail_pose=!exit_failure;PicoFdcc d(r);Submit(d);
  if(exit_failure){Until([&]{return r.sent>0;});r.fail_exit=true;d.Pause();}
  Until([&]{return !d.Snapshot().error.empty();});d.Stop();
  if(exit_failure){assert(r.disabled>0 && r.stops>0);}
 }
 // One in-flight RPC may complete after release; unsent replacements are canceled.
 {
  rcore::sdk::Robot r;r.pose_delay_ms=80;PicoFdcc d(r);Submit(d);
  Until([&]{return r.in_pose.load();});
  auto start=Clock::now();d.Pause();assert(Clock::now()-start<std::chrono::milliseconds(30));
  Until([&]{return d.Snapshot().phase==Phase::Holding;});assert(r.sent==1);d.Stop();
 }
 // No growing target queue; completion slower than 200ms is a failure.
 {
  rcore::sdk::Robot r;r.pose_delay_ms=240;PicoFdcc d(r);Submit(d);
  Until([&]{return !d.Snapshot().error.empty();});d.Stop();assert(r.sent==1);
 }
 // Both scheduling orders converge to hold, with no motor-disable or session
 // failure. Late approvals / still-held Grip cannot re-enable the controller.
 for(bool guard_first:{false,true}){
  rcore::sdk::Robot r;r.exit_delay_ms=80;PicoFdcc d(r);
  const auto expiry=Clock::now()+std::chrono::milliseconds(100);
  d.Submit(Target(),expiry);Until([&]{return r.sent>0;});
  if(guard_first){d.PauseUntilRelease("candidate check pending (>200ms)");}
  Until([&]{return d.Snapshot().rearm_required;});
  assert(!d.AcknowledgeGripRelease()); // Mode-0 acknowledgement is still pending.
  Until([&]{return d.Snapshot().phase==Phase::Holding;});
  const auto count=r.sent.load();
  assert(d.Snapshot().error.empty() && r.mode==0 && r.disabled==0 && r.stops==0);
  assert(!d.Snapshot().pause_reason.empty());
  for(int i=0;i<30;++i){Submit(d);std::this_thread::sleep_for(std::chrono::milliseconds(2));}
  d.Pause(); // Ordinary Pause is not a release acknowledgement.
  assert(r.enabled==1 && r.sent==count && d.Snapshot().rearm_required);
  assert(d.AcknowledgeGripRelease());
  assert(!d.Snapshot().rearm_required && d.Snapshot().pause_reason.empty());
  std::this_thread::sleep_for(std::chrono::milliseconds(30));assert(r.enabled==1);
  r.x=.6;Submit(d);Until([&]{return d.Snapshot().phase==Phase::Active;});
  {std::lock_guard<std::mutex> l(r.mutex);assert(r.poses.at(count).t[0]==.6);}
  d.Stop();assert(r.enabled==2 && r.mode==0 && d.Snapshot().error.empty());
 }
 // Already-expired submissions never activate. Expiry during Enable cleans up
 // without sending a target or letting a late new approval re-enter.
 for(bool before_enable:{false,true}){
  rcore::sdk::Robot r;r.enable_delay_ms=100;PicoFdcc d(r);
  const auto expiry=Clock::now()+(before_enable?std::chrono::milliseconds(-1):std::chrono::milliseconds(40));
  d.Submit(Target(),expiry);
  Until([&]{return d.Snapshot().rearm_required && d.Snapshot().phase==Phase::Holding;});
  Submit(d);assert(r.sent==0 && r.mode==0 && d.Snapshot().error.empty());
  assert(r.enabled==(before_enable?0:1));d.Stop();
 }
 for(int failure:{0,1,2}){
  rcore::sdk::Robot r;PicoFdcc d(r);Submit(d);Until([&]{return r.sent>0;});
  if(failure==0){r.connected=false;}else if(failure==1){r.fault=true;}else{r.bad_tcp=true;}
  Until([&]{return !d.Snapshot().error.empty();});d.Stop();
 }
 // Frozen feedback must terminate even if its values are otherwise plausible.
 {
  rcore::sdk::Robot r;r.freeze_stamp=true;PicoFdcc d(r);Submit(d);
  Until([&]{return !d.Snapshot().error.empty();});d.Stop();assert(r.enabled==1);
 }
 // Wrong starting mode is never overwritten by EnableFdcc.
 {
  rcore::sdk::Robot r;r.mode=5;PicoFdcc d(r);Submit(d);
  Until([&]{return !d.Snapshot().error.empty();});d.Stop();assert(r.enabled==0 && r.exited==0);
 }
 // Diagnostics separate raw/virtual, shaping and measured tracking error;
 // custom speed settings survive pause/reclutch rather than resetting to .15.
 {
  rcore::sdk::Robot r;PicoFdcc d(r,.30,1.2,24);
  d.Submit(Target(1.),Clock::now()+std::chrono::milliseconds(200),Target(.97));
  Until([&]{return r.sent>0;});
  auto s=d.Snapshot();
  assert(std::abs(s.lead_mm-30)<1e-6 && s.follow_error_mm<1e-6);
  assert(s.shape_gap_mm>700 && s.raw_error_mm>600 && s.guard_age_ms>=0);
  for(int round=0;round<2;++round){
   auto end=Clock::now()+std::chrono::milliseconds(230);
   while(Clock::now()<end){Submit(d);std::this_thread::sleep_for(std::chrono::milliseconds(5));}
   assert(d.Snapshot().target_speed_mps>.15 && d.Snapshot().target_speed_mps<=.30+1e-9);
   d.Pause();Until([&]{return d.Snapshot().phase==Phase::Holding;});
   if(round==0){r.x=.5;Submit(d);Until([&]{return d.Snapshot().phase==Phase::Active;});}
  }
  d.Stop();
 }
 // After slow activation, consume the newest checked target, not the first slot.
 {
  rcore::sdk::Robot r;r.enable_delay_ms=80;PicoFdcc d(r);Submit(d,1.);
  Until([&]{return r.enabled==1;});Submit(d,.31);
  Until([&]{return r.sent>0;});
  assert(d.Snapshot().shape_gap_mm<230);d.Stop();
 }
}
'''

LIMITER_HARNESS = r'''
#include <cassert>
#include <limits>
#include "fdcc_target_limiter.hpp"
using smrcore::peripherals::detail::FdccTargetLimiter;
int main(){
 FdccTargetLimiter limiter;
 Eigen::Isometry3d measured=Eigen::Isometry3d::Identity(), wanted=measured;
 wanted.translation().x()=.1;
 auto previous=limiter.Update(wanted,measured,0.0);assert(previous.translation().norm()==0);
 for(int i=0;i<1000;++i){
  if(i==200){wanted.translation()=Eigen::Vector3d(-.1,.1,0);wanted.linear()=Eigen::AngleAxisd(-3.13,Eigen::Vector3d::UnitZ()).toRotationMatrix();}
  if(i==600){wanted.linear()=Eigen::AngleAxisd(3.13,Eigen::Vector3d::UnitZ()).toRotationMatrix();}
  double dt=i==150?10.0:(i%2==0?.02:.013),effective=std::min(dt,.02);
  Eigen::Vector3d v=limiter.Velocity(),w=limiter.AngularVelocity();
  auto current=limiter.Update(wanted,measured,dt);
  assert(limiter.Velocity().norm()<=.15+1e-9);
  assert(limiter.AngularVelocity().norm()<=.8+1e-9);
  assert((limiter.Velocity()-v).norm()<=.6*effective+1e-9);
  assert((limiter.AngularVelocity()-w).norm()<=3.0*effective+1e-9);
  assert((current.translation()-previous.translation()).norm()<=.15*effective+1e-9);
  assert(Eigen::Quaterniond(previous.linear()).angularDistance(Eigen::Quaterniond(current.linear()))<=.8*effective+1e-9);
  previous=current;
 }
 assert((previous.translation()-wanted.translation()).norm()<1e-8);
 assert(Eigen::Quaterniond(previous.linear()).angularDistance(Eigen::Quaterniond(wanted.linear()))<1e-8);
 wanted.translation().x()=NAN;
 bool rejected=false;try{limiter.Update(wanted,measured,.02);}catch(...){rejected=true;}assert(rejected);
}
'''

RESPONSE_HARNESS = r'''
#include <cassert>
#include <iostream>
#include <limits>
#include "fdcc_target_limiter.hpp"
using namespace smrcore::peripherals::detail;
double Compare(bool fast, bool ramp){
 FdccTargetLimiter limiter(fast?.30:.15,fast?1.2:.6,fast?24:8);
 Eigen::Isometry3d measured=Eigen::Isometry3d::Identity(),raw=measured;
 measured.translation()=Eigen::Vector3d(.4,-.2,.3);raw=measured;
 limiter.Update(raw,measured,0);
 double squared=0,peak_speed=0;
 int count=0;
 for(int i=1;i<=100;++i){
  const double t=i*.02;
  raw.translation().x()=.4+(ramp?.25*std::min(t,1.):.03*std::sin(3.141592653589793*t));
  Eigen::Vector3d v=limiter.Velocity();
  auto sent=limiter.Update(raw,measured,.02);
  assert(limiter.Velocity().norm()<=(fast?.30:.15)+1e-9);
  assert((limiter.Velocity()-v).norm()<=(fast?1.2:.6)*.02+1e-9);
  peak_speed=std::max(peak_speed,limiter.Velocity().norm());
  if(t>=.5 && t<=1.){squared+=(raw.translation()-sent.translation()).squaredNorm();++count;}
 }
 const double rms=1000*std::sqrt(squared/count);
 std::cout<<(fast?"fast":"default")<<(ramp?" ramp":" sine")
          <<" target_shaping_rms_mm="<<rms<<" peak_target_speed_mps="<<peak_speed<<'\n';
 if(fast && ramp){assert(peak_speed>.2);}
 return rms;
}
int main(){
 for(bool ramp:{false,true}){const double old_rms=Compare(false,ramp);assert(Compare(true,ramp)<old_rms*.7);}
 // Extrapolation preserves orientation and anchor, vanishes near target,
 // remains bounded far away and is continuous at the onset/cap.
 Eigen::Isometry3d measured=Eigen::Isometry3d::Identity(),raw=measured;
 measured.translation()=Eigen::Vector3d(.4,-.2,.3);raw=measured;
 raw.linear()=Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitX()).toRotationMatrix();
 raw.translation().x()+=.01;
 assert(MakeFdccVirtualTarget(raw,measured,2).matrix().isApprox(raw.matrix()));
 raw.translation().x()=.44;
 assert(MakeFdccVirtualTarget(raw,measured,1).matrix().isApprox(raw.matrix()));
 auto virtual_pose=MakeFdccVirtualTarget(raw,measured,1.5);
 assert(std::abs(virtual_pose.translation().x()-.46)<1e-12);
 assert(virtual_pose.linear().isApprox(raw.linear()));
 auto previous=measured;
 for(int i=0;i<=10000;++i){
  double error=i*.00001;
  raw.translation()=measured.translation()+Eigen::Vector3d(-error,0,0);
  auto target=MakeFdccVirtualTarget(raw,measured,2);
  assert((target.translation()-raw.translation()).norm()<=.03+1e-12);
  assert(target.translation().x()<=raw.translation().x()+1e-12);
  if(i>0){assert((target.translation()-previous.translation()).norm()<.00005);}
  previous=target;
 }
 // Reset must discard old velocity, maintain the configured response, and
 // never jump to a far target on activation. Long scheduler stalls stay capped.
 FdccTargetLimiter fast(.30,1.2,24);
 fast.Update(raw,measured,0);fast.Update(raw,measured,.02);fast.Reset();
 assert(fast.Velocity().isZero());assert(fast.Update(raw,measured,1).matrix().isApprox(measured.matrix()));
 const auto next=fast.Update(raw,measured,10);
 assert((next.translation()-measured.translation()).norm()<=.30*.02);
 for(double value:{double(NAN),double(INFINITY),double(-INFINITY),-1.,0.}){
  assert(!ValidFdccShaping(value,.6,8,1));assert(!ValidFdccShaping(.15,value,8,1));
  assert(!ValidFdccShaping(.15,.6,value,1));assert(!ValidFdccShaping(.15,.6,8,value));
 }
 bool rejected=false;try{FdccTargetLimiter bad(.51,1,24);}catch(...){rejected=true;}assert(rejected);
 for(double scale:{.9,3.1,double(NAN)}){
  rejected=false;try{MakeFdccVirtualTarget(raw,measured,scale);}catch(...){rejected=true;}assert(rejected);
 }
 raw.translation().x()=NAN;rejected=false;
 try{MakeFdccVirtualTarget(raw,measured,1);}catch(...){rejected=true;}assert(rejected);
}
'''


class PicoFdccTest(unittest.TestCase):
    def test_follow_response_and_virtual_target(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / 'check.cpp').write_text(RESPONSE_HARNESS)
            subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra',
                            '-I' + str(root / 'teleop'), '-I/usr/include/eigen3',
                            str(path / 'check.cpp'), '-o', str(path / 'check')], check=True)
            subprocess.run([str(path / 'check')], check=True, timeout=10)

    def test_worker(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / 'sdk').mkdir()
            (path / 'sdk/robot.hpp').write_text(FAKE_SDK)
            (path / 'check.cpp').write_text(HARNESS)
            subprocess.run(['g++', '-std=c++17', '-pthread', '-Wall', '-Wextra',
                            '-I' + str(path), '-I' + str(root / 'teleop'),
                            '-I/usr/include/eigen3', str(path / 'check.cpp'),
                            '-o', str(path / 'check')], check=True)
            subprocess.run([str(path / 'check')], check=True, timeout=20)

    def test_target_limiter(self):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            (path / 'check.cpp').write_text(LIMITER_HARNESS)
            subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra',
                            '-I' + str(root / 'teleop'), '-I/usr/include/eigen3',
                            str(path / 'check.cpp'), '-o', str(path / 'check')], check=True)
            subprocess.run([str(path / 'check')], check=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
