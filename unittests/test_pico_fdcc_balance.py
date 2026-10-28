"""Offline regression: real guard/worker and target shaping, no DDS/hardware."""
from pathlib import Path
import subprocess
import tempfile
import unittest

from .test_pico_fdcc import FAKE_SDK
from .test_pico_impedance import FAKE_SDK as IMPEDANCE_SDK


RESPONSE = r'''
#include <cassert>
#include <iostream>
#include "fdcc_target_limiter.hpp"
using namespace smrcore::peripherals::detail;
double Compare(double ff, bool rotation, bool sine, double gain) {
    FdccTargetLimiter limiter(.3, 1.2, gain, ff);
    auto measured = Eigen::Isometry3d::Identity();
    auto raw = measured;
    limiter.Update(raw, measured, 0, 1);
    double squared = 0, overshoot = 0;
    int count = 0;
    for (int i = 1; i <= 300; ++i) {
        const double time = i * .02;
        const double position = sine ? .03 * std::sin(3.141592653589793 * time) :
            .2 * std::min(time, 2.);
        if (rotation) {
            raw.linear() = Eigen::AngleAxisd(2 * position, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        } else {
            raw.translation().x() = position;
        }
        const auto velocity = limiter.Velocity();
        const auto angular_velocity = limiter.AngularVelocity();
        const auto sent = limiter.Update(raw, measured, .02, 1 + time);
        assert(limiter.Velocity().norm() <= .3 + 1e-9);
        assert((limiter.Velocity() - velocity).norm() <= 1.2 * .02 + 1e-9);
        assert(limiter.AngularVelocity().norm() <= .8 + 1e-9);
        assert((limiter.AngularVelocity() - angular_velocity).norm() <= 3 * .02 + 1e-9);
        const double error = rotation ? Eigen::Quaterniond(raw.linear()).angularDistance(
            Eigen::Quaterniond(sent.linear())) : (raw.translation() - sent.translation()).norm();
        if (time >= .5 && time <= 2.) {
            squared += error * error;
            ++count;
        }
        if (!sine && time > 2.) {
            const double ahead = rotation ? sent.linear().eulerAngles(2, 1, 0)[0] - .8 :
                sent.translation().x() - .4;
            overshoot = std::max(overshoot, ahead);
        }
        if (!sine && i == 300) {
            assert(error < 1e-8);
        }
    }
    const double rms = std::sqrt(squared / count);
    std::cout << "gain=" << gain << " rotation=" << rotation << " sine=" << sine
              << " ff=" << ff << " rms_m_or_rad=" << rms
              << " stop_overshoot_m_or_rad=" << overshoot << '\n';
    // A target abruptly stopping from 0.2m/s must still brake. Record that
    // tradeoff; never describe these target metrics as actual contact force.
    assert(overshoot <= (rotation ? .005 : gain == 8 ? .006 : .014));
    return rms;
}
int main() {
    for (double gain : {8., 24.}) {
        for (bool rotation : {false, true}) {
            for (bool sine : {false, true}) {
                const double baseline = Compare(0, rotation, sine, gain);
                assert(Compare(.5, rotation, sine, gain) < baseline * .6);
            }
        }
    }
    FdccTargetLimiter limiter(.3, 1.2, 8, .5);
    auto measured = Eigen::Isometry3d::Identity();
    measured.linear() = Eigen::AngleAxisd(3.1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    auto target = measured;
    limiter.Update(target, measured, 0, 10);
    for (int i = 1; i <= 30; ++i) {
        target.translation().x() = i * .004;
        target.linear() = Eigen::AngleAxisd(3.1 + i * .008,
            Eigen::Vector3d::UnitZ()).toRotationMatrix();
        limiter.Update(target, measured, .02, 10 + i * .02);
    }
    // Reusing one approved frame decays its feedforward; no continuing drift.
    for (int i = 0; i < 250; ++i) {
        const auto sent = limiter.Update(target, measured, .02, 10.6);
        if (i == 249) {
            assert((sent.translation() - target.translation()).norm() < 1e-8);
            assert(Eigen::Quaterniond(sent.linear()).angularDistance(Eigen::Quaterniond(target.linear())) < 1e-8);
        }
    }
    // Reversed, delayed and sub-ms stamps, long scheduling gap and step goals
    // must retain acceleration bounds and finite output (no inverse-dt spike).
    for (double stamp : {9., 9.0001, 9.5, 9.52}) {
        target.translation().x() = -target.translation().x();
        const auto before = limiter.Velocity();
        const auto sent = limiter.Update(target, measured, .5, stamp);
        assert(sent.matrix().allFinite());
        assert((limiter.Velocity() - before).norm() <= 1.2 * .02 + 1e-9);
    }
    limiter.Reset();
    assert(limiter.Update(target, measured, .02, 20).matrix().isApprox(measured.matrix()));
    for (double bad : {-1., 1.01, double(NAN), double(INFINITY)}) {
        assert(!ValidTargetFeedforward(bad));
    }
    bool rejected = false;
    try { limiter.Update(target, measured, .02, NAN); }
    catch (const std::runtime_error&) { rejected = true; }
    assert(rejected);
}
'''

PRELUDE = r'''
#include <cassert>
#include <Eigen/Geometry>
#include "pico_fdcc.hpp"
#include "peripherals/pico/teleop.hpp"
#include "peripherals/pico/singularity_scale.hpp"
using namespace smrcore::peripherals;
int64_t SteadyNowNanoseconds() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
Eigen::Matrix3d EulerZyxToRotation(const Eigen::Vector3d &rpy) {
    return (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
}
struct HandServoState {
    uint64_t command_frames = 1, trace_sequence = 0;
    double trace_taken = 0, trace_mapped = 0, trace_received = 0;
    Eigen::Isometry3d target_tcp_reference = Eigen::Isometry3d::Identity();
    Eigen::Vector3d target_rpy{.4, .4, .4};
    HandServoState() {
        target_tcp_reference.translation() = Eigen::Vector3d(.4, .4, .004);
        target_tcp_reference.linear() = EulerZyxToRotation(target_rpy);
    }
};
template<class F> void Until(F f) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!f()) {
        assert(std::chrono::steady_clock::now() < deadline);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
'''

GUARD_MAIN = r'''
int main() {
    using Policy = PicoTargetGuard::SingularityPolicy;
    using Worker = smrcore::peripherals::detail::PicoFdcc;
    HandServoState hand;
    // The same reachable endpoint has condition >60. Compliance modes use
    // measured slowdown; ServoP retains the endpoint condition boundary.
    assert(PicoTargetGuard::PolicyForControlMode(PicoControlMode::ServoP) == Policy::CandidateBoundary);
    assert(PicoTargetGuard::PolicyForControlMode(PicoControlMode::Fdcc) == Policy::MeasuredSlowdown);
    assert(PicoTargetGuard::PolicyForControlMode(PicoControlMode::CartesianImpedance) == Policy::MeasuredSlowdown);
    {
        rcore::sdk::Robot robot;
        PicoTargetGuard guard(robot);
        guard.Submit(hand);
        Until([&] { return guard.Snapshot().ready; });
        assert(guard.Snapshot().condition > 60);
        assert(guard.Snapshot().blocked_reason.find("condition >=60") != std::string::npos);
        assert(robot.fk_calls == 7);
    }
    {
        rcore::sdk::Robot robot;
        PicoTargetGuard guard(robot, PicoTargetGuard::PolicyForControlMode(PicoControlMode::Fdcc));
        Worker worker(robot);
        for (int i = 0; i < 8; ++i) {
            guard.Cancel();
            hand.command_frames++;
            hand.target_tcp_reference.translation().x() += .001;
            guard.Submit(hand);
            Until([&] { return guard.Snapshot().ready; });
            const auto checked = guard.Snapshot();
            assert(checked.error.empty() && checked.blocked_reason.empty());
            assert(checked.target.matrix().isApprox(hand.target_tcp_reference.matrix()));
            worker.Submit(checked.target, Worker::Clock::time_point(
                std::chrono::nanoseconds(checked.completed_ns)) + std::chrono::milliseconds(200));
            Until([&] { return worker.Snapshot().phase == Worker::Phase::Active; });
            assert(robot.mode == 6 && robot.exited == 0);
        }
        assert(robot.fk_calls == 8 && robot.enabled == 1);
        const auto stamp = guard.Snapshot().requested_ns;
        guard.Submit(hand);
        assert(guard.Snapshot().requested_ns == stamp);
        worker.Stop();
    }
    // Slow IK/FK is not lost input. Freeze the last sent equilibrium without
    // leaving the mode; a completed check resumes without release/re-enable.
    {
        rcore::sdk::Robot robot;
        PicoTargetGuard guard(robot, PicoTargetGuard::PolicyForControlMode(PicoControlMode::Fdcc));
        Worker worker(robot);
        using Clock = Worker::Clock;
        guard.Submit(hand);
        Until([&] { return guard.Snapshot().ready; });
        auto checked = guard.Snapshot();
        worker.UpdateInputDeadline(Clock::now() + std::chrono::seconds(2));
        worker.Submit(checked.target, Clock::now() + std::chrono::milliseconds(60));
        Until([&] { return robot.sent > 0; });
        robot.ik_delay_ms = 300;
        ++hand.command_frames;
        hand.target_tcp_reference.translation().x() += .05;
        guard.Submit(hand);
        Until([&] { return worker.Snapshot().waiting_for_check; });
        rcore::sdk::Pose frozen;
        size_t frozen_index;
        {
            std::lock_guard<std::mutex> lock(robot.mutex);
            frozen = robot.poses.back();
            frozen_index = robot.poses.size();
        }
        Until([&] { return guard.Snapshot().pending; });
        assert(guard.Snapshot().blocked_reason.empty());
        Until([&] { return guard.Snapshot().requested_ns != checked.requested_ns; });
        checked = guard.Snapshot();
        assert(checked.ik_ms >= 250 && checked.completed_ns > checked.requested_ns);
        assert(!checked.pending && checked.blocked_reason.empty());
        assert(robot.enabled == 1 && robot.exited == 0 && robot.mode == 6);
        {
            std::lock_guard<std::mutex> lock(robot.mutex);
            assert(robot.poses.size() > frozen_index);
            for (size_t i = frozen_index; i < robot.poses.size(); ++i) {
                assert(robot.poses[i].t == frozen.t && robot.poses[i].r == frozen.r);
            }
        }
        worker.Submit(checked.target, Clock::time_point(
            std::chrono::nanoseconds(checked.completed_ns)) + std::chrono::milliseconds(200));
        Until([&] { return !worker.Snapshot().waiting_for_check; });
        Until([&] {
            std::lock_guard<std::mutex> lock(robot.mutex);
            return robot.poses.back().t != frozen.t;
        });
        assert(robot.enabled == 1 && robot.exited == 0 && !worker.Snapshot().rearm_required);
        // Reusing an old receive deadline cannot keep a lost input stream alive.
        const auto dead_input = Clock::now() - std::chrono::milliseconds(1);
        worker.UpdateInputDeadline(dead_input);
        Until([&] { return worker.Snapshot().phase == Worker::Phase::Holding; });
        assert(worker.Snapshot().rearm_required && robot.mode == 0);
        worker.UpdateInputDeadline(Clock::now() + std::chrono::seconds(2));
        worker.Submit(checked.target, Clock::now() + std::chrono::seconds(1));
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        assert(robot.enabled == 1);
        worker.Stop();
    }
    for (int failure = 0; failure < 5; ++failure) {
        rcore::sdk::Robot robot;
        robot.ik_failure = failure == 0;
        robot.fk_failure = failure == 1;
        robot.fk_mismatch = failure == 2;
        robot.bad_joint = failure == 3;
        robot.bad_clearance = failure == 4;
        PicoTargetGuard guard(robot, Policy::MeasuredSlowdown);
        guard.Submit(hand);
        Until([&] { auto c = guard.Snapshot(); return c.ready || !c.error.empty(); });
        const auto c = guard.Snapshot();
        assert(!c.error.empty() || !c.blocked_reason.empty());
    }
    {
        rcore::sdk::Robot robot;
        PicoTargetGuard guard(robot, Policy::MeasuredSlowdown);
        hand.target_tcp_reference.translation().x() = NAN;
        guard.Submit(hand);
        Until([&] { return !guard.Snapshot().error.empty(); });
        assert(robot.ik_calls == 0);
    }
    // Measured slowdown still eases toward a singularity and permits retreat.
    assert(ConditionNumberToSpeedScale(15, true) == 1);
    assert(ConditionNumberToSpeedScale(45, true) == .25);
    assert(ConditionNumberToSpeedScale(60, true) == 0);
    assert(ConditionNumberToSpeedScale(60, false) == 1);
}
'''


class PicoFdccBalanceTest(unittest.TestCase):
    def compile_run(self, code, sdk=None):
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            if sdk:
                (path / 'sdk').mkdir()
                (path / 'sdk/robot.hpp').write_text(sdk)
            (path / 'check.cpp').write_text(code)
            subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-pthread',
                            '-I' + str(path), '-I' + str(root / 'teleop'),
                            '-I' + str(root / 'src'), '-I/usr/include/eigen3',
                            str(path / 'check.cpp'), '-o', str(path / 'check')], check=True)
            subprocess.run([str(path / 'check')], check=True, timeout=20)

    def test_balanced_response(self):
        self.compile_run(RESPONSE)

    def check_guard_policy_and_worker(self, impedance=False):
        root = Path(__file__).resolve().parents[1]
        source = (root / 'teleop/pico_teleop.cpp').read_text()
        guard = source[source.index('class PicoTargetGuard'):source.index('double FilterAlpha(')]
        sdk = (IMPEDANCE_SDK if impedance else FAKE_SDK).replace(
            'namespace sdk {', 'namespace sdk {\nusing JointPositions = std::array<double,6>;')
        sdk = sdk.replace('double timestamp=1;', 'std::array<double,6> positions{.4,.4,.4,.4,.4,.4}; double timestamp=1;')
        sdk = sdk.replace('Robot& Control()', r'''
 std::atomic<int> fk_calls{0}, ik_calls{0}, ik_delay_ms{0};
 bool ik_failure=false, fk_failure=false, fk_mismatch=false, bad_joint=false, bad_clearance=false;
 Result InverseKinematics(Pose p, JointPositions& q) {
  ++ik_calls;
  std::this_thread::sleep_for(std::chrono::milliseconds(ik_delay_ms.load()));
  q={p.t[0],p.t[1],p.t[2]/.01,p.r[0],p.r[1],p.r[2]};
  if (bad_joint) {q[0]=NAN;}
  if (bad_clearance) {q[2]=0;}
  return {!ik_failure};
 }
 Result ForwardKinematics(JointPositions q, Pose& p) {
  ++fk_calls;
  p.t={q[0]+(fk_mismatch?.01:0),q[1],q[2]*.01};p.r={q[3],q[4],q[5]};
  return {!fk_failure};
 }
 Robot& Control()''')
        prelude, main = PRELUDE, GUARD_MAIN
        if impedance:
            prelude = prelude.replace('"pico_fdcc.hpp"', '"pico_impedance.hpp"')
            main = main.replace('detail::PicoFdcc', 'detail::PicoImpedance')
            main = main.replace('Worker worker(robot);',
                                'Worker worker(robot, {240,240,240,20,20,20}, {20,20,20,1,1,1});')
            main = main.replace('guard(robot, PicoTargetGuard::PolicyForControlMode(PicoControlMode::Fdcc))',
                                'guard(robot, PicoTargetGuard::PolicyForControlMode(PicoControlMode::CartesianImpedance))')
            main = main.replace('robot.mode == 6', 'robot.mode == 5')
        self.compile_run(prelude + guard + main, sdk)

    def test_fdcc_guard_policy_and_worker(self):
        self.check_guard_policy_and_worker()

    def test_impedance_guard_policy_and_worker(self):
        self.check_guard_policy_and_worker(impedance=True)


if __name__ == '__main__':
    unittest.main()
