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
                  double follow_gain = 8.0)
        : PicoCompliance(robot, MakeOperations(robot, stiffness, damping),
                         max_speed, max_accel, follow_gain) {}

private:
    static Operations MakeOperations(rcore::sdk::Robot &robot,
                                     const std::array<double, 6> &stiffness,
                                     const std::array<double, 6> &damping)
    {
        if (!ValidImpedanceGains(stiffness, damping))
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
                // Backend owns CST damping stop and transition back to CSP.
                Check(robot.Control().DisableCartesianImpedance(), "DisableCartesianImpedance");
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
            }, {}};
    }
};
} // namespace smrcore::peripherals::detail
