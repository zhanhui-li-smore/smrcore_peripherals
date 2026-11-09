#pragma once
#include "pico_compliance.hpp"
#include <array>

namespace smrcore::peripherals::detail
{
inline bool ValidImpedanceGains(const std::array<double, 6> &stiffness,
                               const std::array<double, 6> &damping)
{
    for (size_t i = 0; i < stiffness.size(); ++i)
    {
        // SDK engineering ranges; backend remains authoritative.
        if (!std::isfinite(stiffness[i]) || !std::isfinite(damping[i]) ||
            stiffness[i] < 0.0 || stiffness[i] > (i < 3 ? 2000.0 : 150.0) ||
            damping[i] < 0.0 || damping[i] > (i < 3 ? 200.0 : 60.0))
        {
            return false;
        }
    }
    return true;
}

class PicoImpedance final : public PicoCompliance
{
public:
    PicoImpedance(rcore::sdk::Robot &robot,
                  const std::array<double, 6> &stiffness,
                  const std::array<double, 6> &damping,
                  double max_speed = 0.15, double max_accel = 0.6,
                  double follow_gain = 8.0, bool direct_target = false, double joint_max_speed_deg = 0.0)
        : PicoCompliance(robot, MakeOperations(robot, stiffness, damping, joint_max_speed_deg, direct_target),
                         max_speed, max_accel, follow_gain, 0.0, direct_target, joint_max_speed_deg * 3.14159265358979323846 / 180.0) {}

private:
    static Operations MakeOperations(rcore::sdk::Robot &robot,
                                     const std::array<double, 6> &stiffness,
                                     const std::array<double, 6> &damping,
                                     double joint_max_speed_deg, bool direct_target)
    {
        if (!std::isfinite(joint_max_speed_deg) || joint_max_speed_deg < 0.0 ||
            joint_max_speed_deg > 180.0 || (joint_max_speed_deg > 0.0 && direct_target) ||
            !ValidImpedanceGains(stiffness, damping))
        {
            throw std::runtime_error("invalid CartesianImpedance stiffness/damping");
        }
        rcore::sdk::CartesianImpedanceParams params;
        params.stiffness = stiffness;
        params.damping = damping;
        params.desired_wrench = {0, 0, 0, 0, 0, 0};
        return {"Impedance", 5, std::chrono::milliseconds(5),
            [&robot, params] {
                Check(robot.Control().EnableCartesianImpedance(params), "EnableCartesianImpedance");
            },
            [&robot] {
                Check(robot.Control().SetCartesianImpedanceTeleopSource(
                    rcore::TeleopInputSource::Manual), "Impedance Manual source");
            },
            [&robot] {
                // 4311 acknowledges that CST damping is still active after a
                // two-second settling window. Continue that stop, never send
                // another target or re-enable. Other errors (including RPC
                // timeouts with unknown outcome) keep the EStop fallback.
                constexpr int kStopAttempts = 3;
                for (int attempt = 0; attempt < kStopAttempts; ++attempt)
                {
                    auto result = robot.Control().DisableCartesianImpedance();
                    if (result.IsSuccess())
                    {
                        return;
                    }
                    if (result.GetErrorCode() != 4311 || attempt + 1 == kStopAttempts)
                    {
                        Check(std::move(result), "DisableCartesianImpedance");
                    }
                    const auto state = robot.GetState();
                    if (!robot.IsConnected() || state.errors.GetLatchedErrorCount() > 0 ||
                        static_cast<int>(state.control_mode) != 5 ||
                        state.command_source != rcore::sdk::CommandSource::Sdk)
                    {
                        throw std::runtime_error("Impedance damping exit lost connection/mode/ownership or faulted");
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            },
            [&robot](const rcore::sdk::Pose &pose) {
                // Dedicated equilibrium-point API; void DDS write, no RPC ack.
                robot.Control().SetCartesianImpedanceTarget(pose);
            },
            [](const rcore::RobotState &state) {
                if (state.command_source != rcore::sdk::CommandSource::Sdk)
                {
                    throw std::runtime_error("Impedance SDK command ownership lost");
                }
            }, {},
            [&robot](const std::array<double, 6> &joints) {
                rcore::sdk::JointPositions q;
                for (size_t i = 0; i < joints.size(); ++i) { q[i] = joints[i]; }
                rcore::sdk::Pose pose;
                Check(robot.Motion().ForwardKinematics(q, pose), "joint-limited FK");
                Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
                for (int i = 0; i < 3; ++i) { result.translation()[i] = pose.tvec()[i]; }
                result.linear() = (Eigen::AngleAxisd(pose.rvec()[2], Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(pose.rvec()[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(pose.rvec()[0], Eigen::Vector3d::UnitX())).toRotationMatrix();
                return result;
            }};
    }
};
} // namespace smrcore::peripherals::detail
