"""Check the production base-yaw transform and CLI parser without robot IO."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class PicoBaseYawTest(unittest.TestCase):
    def test_frame_rotation_and_parser(self):
        root = Path(__file__).resolve().parents[1]
        core = (root / 'teleop/pico_teleop.cpp').read_text()
        mapping = core[core.index('Eigen::Isometry3d SingleWorldFromBase('):core.index('struct ArmReference')]
        cli = (root / 'app/app_peripherals_pico_bridge.cpp').read_text()
        parser = cli[cli.index('bool ParseMappingYaw('):cli.index('bool ParseOperatorFrame(')]
        source = r'''
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>
#include <string>
constexpr double kPi = 3.14159265358979323846;
''' + mapping + parser + r'''
int main()
{
    double yaw = 0;
    assert(ParseMappingYaw("-90", yaw) && yaw == -90);
    assert(ParseMappingYaw("180", yaw) && yaw == 180);
    assert(ParseMappingYaw("-180", yaw) && yaw == -180);
    for (const char *bad : {"nan", "inf", "-inf", "90junk", "181", "-181", ""})
    {
        assert(!ParseMappingYaw(bad, yaw));
    }
    assert(SingleWorldFromBase(0).matrix().isApprox(Eigen::Matrix4d::Identity()));
    const auto world_from_base = SingleWorldFromBase(-90);
    const auto base_from_world = world_from_base.inverse();
    assert((base_from_world.linear() * Eigen::Vector3d::UnitX()).isApprox(-Eigen::Vector3d::UnitY()));
    assert((base_from_world.linear() * Eigen::Vector3d::UnitY()).isApprox(Eigen::Vector3d::UnitX()));
    assert((base_from_world.linear() * Eigen::Vector3d::UnitZ()).isApprox(Eigen::Vector3d::UnitZ()));
    Eigen::Isometry3d initial = Eigen::Isometry3d::Identity();
    initial.translation() = Eigen::Vector3d(.4, -.2, .3);
    initial.linear() = Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
    auto target = world_from_base * initial;
    assert((base_from_world * target).matrix().isApprox(initial.matrix()));
    target.translation() += Eigen::Vector3d(.01, 0, 0);
    target.linear() = Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitX()).toRotationMatrix() * target.linear();
    const auto actual = base_from_world * target;
    assert(actual.translation().isApprox(initial.translation() + Eigen::Vector3d(0, -.01, 0)));
    const Eigen::Matrix3d expected_rotation = Eigen::AngleAxisd(-.1, Eigen::Vector3d::UnitY()).toRotationMatrix() * initial.linear();
    assert(actual.linear().isApprox(expected_rotation));
}
'''
        with tempfile.TemporaryDirectory() as directory:
            cpp = Path(directory) / 'check.cpp'
            binary = Path(directory) / 'check'
            cpp.write_text(source)
            subprocess.run(['g++', '-std=c++17', '-I/usr/include/eigen3', str(cpp), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
