#pragma once
#include "pico_compliance.hpp"
#include <cmath>

namespace smrcore::peripherals::detail
{
inline bool ValidFdccDampingFactor(double value)
{
    return std::isfinite(value) && value >= 0.0 && value <= 1.0;
}
inline bool ValidFdccErrorScale(double value)
{
    return std::isfinite(value) && value >= 0.1 && value <= 4.0;
}
class PicoFdcc final : public PicoCompliance
{
public:
    explicit PicoFdcc(rcore::sdk::Robot &robot, double max_speed = 0.30,
                      double max_accel = 1.2, double follow_gain = 8.0,
                      double target_feedforward = 0.5, double damping_factor = 0.9,
                      double error_scale = 1.0, bool direct_target = false)
        : PicoCompliance(robot, MakeOperations(robot, damping_factor, error_scale), max_speed, max_accel,
                         follow_gain, target_feedforward, direct_target) {}

private:
    static Operations MakeOperations(rcore::sdk::Robot &robot, double damping_factor,
                                     double error_scale)
    {
        if (!ValidFdccDampingFactor(damping_factor))
        {
            throw std::runtime_error("invalid FDCC damping_factor; expected [0,1]");
        }
        if (!ValidFdccErrorScale(error_scale))
        {
            throw std::runtime_error("invalid FDCC error_scale; test entry expects [0.1,4]");
        }
        rcore::sdk::FdCartesianAdmittanceParams params;
        params.damping_factor = damping_factor;
        params.error_scale = error_scale;
        return {"FDCC", 6, std::chrono::milliseconds(20),
            [&robot, params] {
                Check(robot.Control().EnableFdCartesianAdmittance(
                    params), "EnableFdcc");
            },
            [&robot] {
                Check(robot.Control().SetFdCartesianAdmittanceTeleopSource(
                    rcore::TeleopInputSource::Manual), "Fdcc Manual source");
            },
            [&robot] { Check(robot.Control().DisableFdCartesianAdmittance(), "DisableFdcc"); },
            [&robot](const rcore::sdk::Pose &pose) {
                Check(robot.Control().SetFdCartesianAdmittancePoseTarget(pose), "Fdcc pose target");
            }, {},
            [&robot] {
                Check(robot.Control().SetFdCartesianAdmittanceWrenchTarget(
                    {0, 0, 0, 0, 0, 0}), "Fdcc zero wrench target");
            }};
    }
};
} // namespace smrcore::peripherals::detail
