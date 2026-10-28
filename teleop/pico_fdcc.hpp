#pragma once
#include "pico_compliance.hpp"

namespace smrcore::peripherals::detail
{
class PicoFdcc final : public PicoCompliance
{
public:
    explicit PicoFdcc(rcore::sdk::Robot &robot, double max_speed = 0.30,
                      double max_accel = 1.2, double follow_gain = 8.0,
                      double target_feedforward = 0.5)
        : PicoCompliance(robot, MakeOperations(robot), max_speed, max_accel,
                         follow_gain, target_feedforward) {}

private:
    static Operations MakeOperations(rcore::sdk::Robot &robot)
    {
        return {"FDCC", 6, std::chrono::milliseconds(20),
            [&robot] {
                Check(robot.Control().EnableFdCartesianAdmittance(
                    rcore::sdk::FdCartesianAdmittanceParams{}), "EnableFdcc");
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
