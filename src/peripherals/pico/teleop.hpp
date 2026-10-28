#pragma once
#include <atomic>
#include <array>
#include <cstdint>
#include <string>

namespace smrcore::peripherals
{
enum class PicoControlMode
{
    ServoP,
    Fdcc,
    CartesianImpedance,
};

struct PicoTeleopOptions
{
    uint16_t port = 8765;
    bool enable_motion = false;
    bool single_arm = false; // Independent Single SDK instance, physical right hand.
    bool enable_gripper = true;
    bool enable_rotation = true;
    bool head_yaw_clutched = true;
    bool operator_facing_robot = false;
    double translation_scale = 0.8;
    double rotation_scale = 1.0;

    double j6_thumbstick_speed = 0.8;
    uint16_t gripper_force = 80;
    uint16_t gripper_speed = 100;
    std::string robot_ip;
    // Rotate single-arm mapped increments in base: degrees, +Z right-hand rule.
    double single_mapping_yaw_deg = 0.0;
    PicoControlMode control_mode = PicoControlMode::ServoP;
    // FDCC balanced input profile. Advanced tuning, not controller kp/kd.
    double fdcc_max_speed = 0.30; // m/s
    double fdcc_max_accel = 1.2; // m/s^2
    double fdcc_follow_gain = 8.0; // 1/s
    double fdcc_target_scale = 1.0; // Optional measured-to-target extrapolation.
    double fdcc_target_feedforward = 0.5; // Bounded target velocity feedforward [0,1].
    // Cartesian impedance K/D: XYZ then RX/RY/RZ, SDK hold baseline.
    std::array<double, 6> impedance_stiffness{120, 120, 120, 20, 20, 20};
    std::array<double, 6> impedance_damping{20, 20, 20, 1, 1, 1};
    double impedance_max_speed = 0.15; // Equilibrium-point shaping, m/s.
    double impedance_max_accel = 0.6; // m/s^2
    double impedance_follow_gain = 8.0; // 1/s, not controller stiffness.
};

struct PicoTeleopStatus
{
    bool running = false;
    int exit_code = -1;
    // Latest observed values; zero before the control loop, not a coherent RT
    // snapshot.
    std::array<double, 6> left_positions{};
    std::array<double, 6> right_positions{};
    double left_condition = 0, right_condition = 0;
    double left_scale = 0, right_scale = 0;
};
// Single-use session. Run blocks; RequestStop is safe from another thread.
// Join the Run caller before destroying this object. No Python callbacks on RT
// path.
class PicoTeleop
{
public:
    PicoTeleop()
    {
        for (auto &v : diagnostics_)
            v.store(0);
    }
    PicoTeleop(const PicoTeleop &) = delete;
    PicoTeleop &operator=(const PicoTeleop &) = delete;
    PicoTeleopStatus GetStatus() const noexcept;
    int Run(const PicoTeleopOptions &options);
    void RequestStop() noexcept { running_.store(false); }
    bool IsRunning() const noexcept { return active_.load(); }
    int GetExitCode() const noexcept { return exit_code_.load(); }

private:
    std::array<std::atomic<double>, 16> diagnostics_;
    std::atomic<bool> started_{false};
    std::atomic<bool> running_{true};
    std::atomic<bool> active_{false};
    std::atomic<int> exit_code_{-1};
};
} // namespace smrcore::peripherals
