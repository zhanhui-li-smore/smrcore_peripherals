"""Fault injection against the actual Pico preflight and SDK send functions."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class PicoNonfiniteTest(unittest.TestCase):
    def test_reject_before_sdk_send(self):
        root = Path(__file__).resolve().parents[1]
        source = (root / 'teleop/pico_teleop.cpp').read_text()
        helpers = source[source.index('template <typename Values>'):source.index('Eigen::Matrix3d WebXrFromGrip(')]
        sender = source[source.index('bool SendSdkServoPTick('):source.index('void PrintArmDiagnostics(')]
        harness = r'''
#include <array>
#include <cmath>
#include <chrono>
#include <stdexcept>
#include <cassert>
#include <limits>
#include <Eigen/Geometry>
#include "peripherals/types.hpp"
using namespace smrcore::peripherals;
constexpr double kInputStaleSeconds = .25;
namespace rcore::sdk {
struct Pose {
    std::array<double, 3> t{}, r{};
    auto& tvec() { return t; }
    auto& rvec() { return r; }
};
struct Robot {
    int sends = 0;
    Robot& Motion() { return *this; }
    void ServoJ(const std::array<double, 6>&) { ++sends; }
    void ServoP(const Pose&) { ++sends; }
};
}
struct HandServoState {
    bool joint6_active = false, joint6_target_valid = true;
    bool active = true, servop_stream_started = true, target_valid = true;
    std::array<double, 6> joint_target{};
    double joint6_velocity = 0;
    std::chrono::steady_clock::time_point last_joint6_update = std::chrono::steady_clock::now();
    Eigen::Isometry3d target_tcp_reference = Eigen::Isometry3d::Identity();
    Eigen::Vector3d target_rpy = Eigen::Vector3d::Zero();
};
''' + helpers + sender + r'''
int main() {
    const double bad_values[] = {NAN, INFINITY, -INFINITY};
    PicoSample packet;
    ValidatePicoPacket(packet, false, true);
    auto rejects = [](const PicoSample& p) {
        try { ValidatePicoPacket(p, false, true); }
        catch (const std::runtime_error&) { return true; }
        return false;
    };
    for (double bad : bad_values) {
        for (int i = 0; i < 3; ++i) {
            auto p = packet; p.right.position[i] = bad; assert(rejects(p));
        }
        for (int i = 0; i < 4; ++i) {
            auto p = packet; p.left.orientation_xyzw[i] = bad; assert(rejects(p));
        }
        auto p = packet; p.right.trigger = bad; assert(rejects(p));
        p = packet; p.right.grip = bad; assert(rejects(p));
        for (int i = 0; i < 2; ++i) {
            p = packet; p.right.thumbstick[i] = bad; assert(rejects(p));
        }
        p = packet; p.timestamp_sec = bad; assert(rejects(p));
        p = packet; p.headset.position[0] = bad; assert(rejects(p));
        for (int i = 0; i < 6; ++i) {
            rcore::sdk::Robot robot; HandServoState hand;
            hand.joint6_active = true; hand.joint_target[i] = bad;
            assert(!SendSdkServoPTick(robot, hand)); assert(robot.sends == 0);
        }
        rcore::sdk::Robot robot; HandServoState hand;
        hand.target_rpy[0] = bad;
        assert(!SendSdkServoPTick(robot, hand)); assert(robot.sends == 0);
        hand = HandServoState{}; hand.target_tcp_reference.translation()[2] = bad;
        assert(!SendSdkServoPTick(robot, hand)); assert(robot.sends == 0);
        hand = HandServoState{}; hand.joint6_active = true; hand.joint6_velocity = bad;
        assert(!SendSdkServoPTick(robot, hand)); assert(robot.sends == 0);
    }
    auto p = packet; p.right.orientation_xyzw.fill(0); assert(rejects(p));
    p.right.orientation_xyzw.fill(1e308); assert(rejects(p));
    p = packet; p.left.position[0] = NAN;
    ValidatePicoPacket(p, true, false); // Unused left hand must not block single arm.
    rcore::sdk::Robot robot; HandServoState hand;
    assert(SendSdkServoPTick(robot, hand));
    hand.joint6_active = true; assert(SendSdkServoPTick(robot, hand));
    assert(robot.sends == 2);
}
'''
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / 'test.cpp'
            binary = Path(directory) / 'test'
            cpp.write_text(harness)
            subprocess.run(['g++', '-std=c++17', '-I/usr/include/eigen3',
                            '-I' + str(root / 'src'), str(cpp), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, timeout=10)
