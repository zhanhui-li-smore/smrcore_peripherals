/**
 * @file app_peripherals_pico_bridge.cpp
 * @brief PICO SDK 遥操作：双臂/单臂 ServoP，单臂可选 FDCC。
 *
 * WebXR 页面通过 adb reverse + localhost WebSocket 发送头显和左右手柄位姿。
 * 本外设应用只维护 Grip 离合、坐标映射和最新 TCP 目标；机器人侧继续使用
 * ControllerManager 的运动学规划或 FDCC 控制律与安全检查。
 */

#include "peripherals/pico/pico.hpp"
#include "peripherals/pico/singularity_scale.hpp"

#include "sdk/dual_arm_robot.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include "peripherals/pico/teleop.hpp"
#include <cstdlib>
#include <exception>
#include <iomanip>
#include <iostream>
#include <future>
#include <limits>
#include <mutex>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "latency_trace.hpp"
#include "pico_fdcc.hpp"
#include "pico_impedance.hpp"

namespace
{

using smrcore::peripherals::AlignSingularDirection;
using smrcore::peripherals::AnalyzeJacobian;
using smrcore::peripherals::ClassifySingularityTravel;
using smrcore::peripherals::ConditionNumberToSpeedScale;
using smrcore::peripherals::FilterSingularityRecoveryScale;
using smrcore::peripherals::IsApproachingSingularity;
using smrcore::peripherals::kSingularityConditionDeltaEpsilon;
using smrcore::peripherals::kSingularityHardConditionNumber;
using smrcore::peripherals::kSingularityRecoveryFilterTimeConstantSeconds;
using smrcore::peripherals::kSingularitySoftConditionNumber;
using smrcore::peripherals::kSingularityTwistAlignEpsilon;
using smrcore::peripherals::Pico;
using smrcore::peripherals::PicoControllerPose;
using smrcore::peripherals::PicoHeadsetPose;
using smrcore::peripherals::PicoOptions;
using smrcore::peripherals::PicoSample;
using smrcore::peripherals::ResolveTowardFromObservedMotion;
using smrcore::peripherals::SingularityTravel;
using smrcore::peripherals::SingularityTravelToString;

constexpr double kDefaultTranslationScale = 0.80;
constexpr double kDefaultRotationScale = 1.00;
constexpr double kDefaultJ6ThumbstickSpeedRadiansPerSecond = 0.80;
constexpr double kMinimumMappingScale = 0.10;
constexpr double kMaximumMappingScale = 3.00;
// WebXR over adb reverse can hitch past 100ms; freeze+require_release at
// 100ms looks like "PICO moves, arm does not" until the user fully releases
// Grip. 250ms still stops a dropped stream without latching the clutch.
constexpr double kInputStaleSeconds = 0.25;
constexpr double kRobotStateStaleSeconds = 0.25;
constexpr double kGripPressThreshold = 0.90;
constexpr double kGripReleaseThreshold = 0.75;
constexpr double kTriggerPressThreshold = 0.75;
constexpr double kTriggerReleaseThreshold = 0.25;
constexpr double kJ6ThumbstickDeadzone = 0.18;
constexpr double kJ6ThumbstickAxisSign = -1.0;
constexpr double kPi = 3.14159265358979323846;
constexpr double kHandPositionFilterTimeConstant = 0.0;
constexpr double kHandOrientationFilterTimeConstant = 0.0;
constexpr double kMaxTranslationDeltaMeters = 0.35;
constexpr double kMaxOrientationTargetRateRadiansPerSecond = 3.5;
constexpr double kEulerGimbalLockCosineThreshold = 1e-6;
constexpr auto kServoPeriod = std::chrono::microseconds(1000);
constexpr double kServoPStartTranslationMeters = 1e-6;
constexpr double kServoPStartRotationRadians = 1e-5;
constexpr double kSingularityMonitorPeriodSeconds = 0.05;
constexpr double kSingularityMonitorStaleSeconds = 0.25;

std::mutex session_mutex;
thread_local std::atomic<bool> *session_running = nullptr;
using Options = smrcore::peripherals::PicoTeleopOptions;
using smrcore::peripherals::PicoControlMode;
using smrcore::peripherals::detail::PicoFdcc;
using smrcore::peripherals::detail::PicoCompliance;
using smrcore::peripherals::detail::PicoImpedance;

bool TriggerIsPressed(double trigger, bool was_pressed)
{
    if (!std::isfinite(trigger))
    {
        return was_pressed;
    }
    return was_pressed ? trigger >= kTriggerReleaseThreshold
                       : trigger >= kTriggerPressThreshold;
}

class EndBoardDhGripper
{
public:
    EndBoardDhGripper(rcore::sdk::Robot &robot, std::string name)
        : robot_(robot), name_(std::move(name))
    {
    }

    ~EndBoardDhGripper() { Stop(); }

    EndBoardDhGripper(const EndBoardDhGripper &) = delete;
    EndBoardDhGripper &operator=(const EndBoardDhGripper &) = delete;

    bool Initialize(uint16_t force, uint16_t speed)
    {
        if (!WriteInitialization([&] { return robot_.EndBoard().RTU_DH_Init(); },
                                 "初始化"))
        {
            return false;
        }
        // 与末端板 DH 初始化等待一致；不读取状态或回读配置寄存器。
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (!WriteInitialization(
                [&] { return robot_.EndBoard().RTU_DH_SetForce(force); },
                "设置力") ||
            !WriteInitialization(
                [&] { return robot_.EndBoard().RTU_DH_SetSpeed(speed); },
                "设置速度"))
        {
            return false;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = true;
        }
        worker_ = std::thread(&EndBoardDhGripper::Run, this);
        Move(false);
        std::cout << name_ << "初始化命令已发送: force=" << force << "% speed=" << speed
                  << "%" << std::endl;
        return true;
    }

    bool Move(bool closed)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_)
        {
            return false;
        }
        pending_position_ = closed ? uint16_t{0} : uint16_t{1000};
        condition_.notify_one();
        std::cout << name_ << ": " << (closed ? "闭合" : "打开") << std::endl;
        return true;
    }

    void CancelPending()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_position_.reset();
    }

    void Stop()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            running_ = false;
            pending_position_.reset();
            condition_.notify_one();
        }
        if (worker_.joinable())
        {
            worker_.join();
        }
    }

private:
    bool Check(rcore::sdk::Result result, const std::string &action)
    {
        if (result.IsSuccess())
        {
            return true;
        }
        std::cerr << name_ << action << "失败: [" << result.GetErrorCode()
                  << "] " << result.GetErrorMsg() << std::endl;
        return false;
    }

    template <typename Writer>
    bool WriteInitialization(Writer &&writer, const std::string &action)
    {
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::seconds(5);
        bool busy_reported = false;
        while (true)
        {
            auto result = writer();
            if (result.IsSuccess())
            {
                // RPC 成功只表示入队；留出 RT 下发与板端处理的帧间隔。
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                return true;
            }
            // 仅重试明确未入队的 busy；其他失败不能假设命令没有执行。
            if (result.GetErrorCode() != 5403 ||
                result.GetErrorMsg() !=
                    "Previous end board RAW485 transaction busy")
            {
                return Check(std::move(result), action);
            }
            if (std::chrono::steady_clock::now() >= deadline)
            {
                Check(std::move(result), action);
                std::cerr << name_
                          << " RAW485 持续忙超过 5s；请停止其他末端板控制程序，"
                             "检查板端通信，并在确认可恢复后执行硬件 Recover。"
                             "重启遥操程序不会清除板端旧事务。"
                          << std::endl;
                return false;
            }
            if (!busy_reported)
            {
                std::cerr << name_ << action
                          << "等待上一条 RAW485 事务完成（最多 5s）"
                          << std::endl;
                busy_reported = true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void Run()
    {
        while (true)
        {
            uint16_t position = 0;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock,
                                [&] { return !running_ || pending_position_; });
                if (!running_)
                {
                    return;
                }
                position = *pending_position_;
                pending_position_.reset();
            }
            (void)Check(robot_.EndBoard().RTU_DH_SetPosition(position),
                        "设置位置 " + std::to_string(position));
        }
    }

    rcore::sdk::Robot &robot_;
    std::string name_;
    std::mutex mutex_;
    std::condition_variable condition_;
    bool running_ = false;
    std::optional<uint16_t> pending_position_;
    std::thread worker_;
};

Eigen::Matrix3d EulerZyxToRotation(const Eigen::Vector3d &rpy)
{
    return (Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

double UnwrapNear(double angle, double reference)
{
    while (angle - reference > kPi)
    {
        angle -= 2.0 * kPi;
    }
    while (angle - reference < -kPi)
    {
        angle += 2.0 * kPi;
    }
    return angle;
}

Eigen::Vector3d UnwrapRpyNear(Eigen::Vector3d rpy,
                              const Eigen::Vector3d &reference)
{
    for (Eigen::Index axis = 0; axis < 3; ++axis)
    {
        rpy[axis] = UnwrapNear(rpy[axis], reference[axis]);
    }
    return rpy;
}

Eigen::Vector3d RotationToContinuousEulerZyx(const Eigen::Matrix3d &rotation,
                                             const Eigen::Vector3d &reference)
{
    const double sine_pitch = std::clamp(-rotation(2, 0), -1.0, 1.0);
    const double pitch = std::asin(sine_pitch);
    const double cosine_pitch = std::cos(pitch);
    if (std::abs(cosine_pitch) <= kEulerGimbalLockCosineThreshold)
    {
        // At pitch = +/-pi/2 only a roll/yaw combination is observable.
        // Preserve the previous roll and solve the coupled yaw instead of
        // resetting roll to zero, which would create a ServoP branch jump.
        const double roll = reference[0];
        const double coupled_yaw = std::atan2(-rotation(0, 1), rotation(1, 1));
        const double yaw =
            sine_pitch >= 0.0 ? roll + coupled_yaw : coupled_yaw - roll;
        return UnwrapRpyNear({roll, std::copysign(kPi / 2.0, sine_pitch), yaw},
                             reference);
    }

    const Eigen::Vector3d primary{
        std::atan2(rotation(2, 1), rotation(2, 2)),
        pitch,
        std::atan2(rotation(1, 0), rotation(0, 0)),
    };
    // ZYX has two equivalent branches away from gimbal lock. Select the one
    // nearest the last command so crossing +/-90 deg pitch stays continuous.
    const Eigen::Vector3d alternate{
        primary[0] + kPi,
        kPi - primary[1],
        primary[2] + kPi,
    };
    const Eigen::Vector3d primary_near = UnwrapRpyNear(primary, reference);
    const Eigen::Vector3d alternate_near = UnwrapRpyNear(alternate, reference);
    return (primary_near - reference).squaredNorm() <=
                   (alternate_near - reference).squaredNorm()
               ? primary_near
               : alternate_near;
}

Eigen::Isometry3d MakeIsometry(const Eigen::Vector3d &xyz,
                               const Eigen::Vector3d &rpy)
{
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = xyz;
    transform.linear() = EulerZyxToRotation(rpy);
    return transform;
}

void LoadEmbeddedMountTransforms(Eigen::Isometry3d &left,
                                 Eigen::Isometry3d &right)
{
    left.matrix() << -0.935112469, -0.242025472, 0.258821059, 0.028988,
        0.354351054, -0.638691585, 0.683014194, 0.1515, 0.0, 0.730408605,
        0.683010447, 0.2765, 0.0, 0.0, 0.0, 1.0;
    right.matrix() << -0.935112469, 0.242025472, 0.258821059, 0.028988,
        -0.354351054, -0.638691585, -0.683014194, -0.1515, 0.0, -0.730408605,
        0.683010447, 0.2765, 0.0, 0.0, 0.0, 1.0;
}

Eigen::Isometry3d SingleWorldFromBase(double mapping_yaw_deg)
{
    Eigen::Isometry3d world_from_base = Eigen::Isometry3d::Identity();
    // Targets return through world_from_base.inverse(): use the opposite
    // frame angle to rotate mapped increments by +mapping_yaw_deg in base.
    world_from_base.linear() = Eigen::AngleAxisd(
        -mapping_yaw_deg * kPi / 180.0, Eigen::Vector3d::UnitZ())
                                  .toRotationMatrix();
    return world_from_base;
}

struct ArmReference
{
    bool valid = false;
    std::string name;
    Eigen::Isometry3d world_from_reference = Eigen::Isometry3d::Identity();
};

bool ResolveArmReference(rcore::sdk::Robot &robot,
                         const rcore::RobotState &state,
                         const Eigen::Isometry3d &world_from_base,
                         ArmReference &result, std::string &error)
{
    result = {};
    auto set_result = robot.Frames().SetCurrentReference("base");
    if (!set_result.IsSuccess())
    {
        error = "切换 base 参考系失败";
        return false;
    }

    double previous_timestamp = state.timestamp;
    int fresh_samples = 0;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline)
    {
        const rcore::RobotState fresh_state = robot.GetState();
        if (robot.Frames().GetCurrentReference() != "base")
        {
            error = "base 参考系未生效";
            return false;
        }
        if (fresh_state.timestamp != previous_timestamp)
        {
            previous_timestamp = fresh_state.timestamp;
            if (fresh_state.reference_frame == "base")
            {
                ++fresh_samples;
            }
            else
            {
                fresh_samples = 0;
            }
        }
        if (fresh_samples >= 2)
        {
            result.valid = true;
            result.name = "base";
            result.world_from_reference = world_from_base;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    error = "base 参考系状态未确认";
    return false;
}

bool ResolveDualReferences(rcore::sdk::DualArmRobot &arms, ArmReference &left,
                           ArmReference &right,
                           const Eigen::Isometry3d &world_from_left_base,
                           const Eigen::Isometry3d &world_from_right_base,
                           std::string &error)
{
    const auto left_state = arms.left_robot().GetState();
    const auto right_state = arms.right_robot().GetState();
    std::string left_error;
    std::string right_error;
    const bool left_ok = ResolveArmReference(
        arms.left_robot(), left_state, world_from_left_base, left, left_error);
    const bool right_ok =
        ResolveArmReference(arms.right_robot(), right_state,
                            world_from_right_base, right, right_error);
    if (!left_ok || !right_ok)
    {
        error = "L: " + left_error + "; R: " + right_error;
        return false;
    }
    return true;
}

// Reject malformed inputs before mapping, IK or gripper edge processing.
template <typename Values>
bool AllFinite(const Values &values)
{
    for (const auto value : values)
    {
        if (!std::isfinite(value))
        {
            return false;
        }
    }
    return true;
}

template <typename Pose>
bool ValidPicoPoseNumbers(const Pose &pose)
{
    double norm_squared = 0.0;
    for (const double value : pose.orientation_xyzw)
    {
        norm_squared += value * value;
    }
    return AllFinite(pose.position) && AllFinite(pose.orientation_xyzw) &&
           std::isfinite(norm_squared) && norm_squared >= 1e-18;
}

bool ValidPicoControllerNumbers(const PicoControllerPose &pose)
{
    return ValidPicoPoseNumbers(pose) && std::isfinite(pose.trigger) &&
           std::isfinite(pose.grip) && AllFinite(pose.thumbstick);
}

void ValidatePicoPacket(const PicoSample &packet, bool single_arm,
                        bool head_yaw_clutched)
{
    if (!std::isfinite(packet.timestamp_sec))
    {
        throw std::runtime_error("PICO invalid receive timestamp");
    }
    if (!ValidPicoControllerNumbers(packet.right))
    {
        throw std::runtime_error("PICO right: nonfinite input or invalid quaternion");
    }
    if (!single_arm && !ValidPicoControllerNumbers(packet.left))
    {
        throw std::runtime_error("PICO left: nonfinite input or invalid quaternion");
    }
    if (head_yaw_clutched && !ValidPicoPoseNumbers(packet.headset))
    {
        throw std::runtime_error("PICO headset: nonfinite input or invalid quaternion");
    }
}

Eigen::Matrix3d WebXrFromGrip(const PicoControllerPose &pose)
{
    const Eigen::Quaterniond quaternion(
        pose.orientation_xyzw[3], pose.orientation_xyzw[0],
        pose.orientation_xyzw[1], pose.orientation_xyzw[2]);
    if (quaternion.norm() < 1e-9)
    {
        return Eigen::Matrix3d::Identity();
    }
    return quaternion.normalized().toRotationMatrix();
}

Eigen::Vector3d WebXrPosition(const PicoControllerPose &pose)
{
    return {pose.position[0], pose.position[1], pose.position[2]};
}

Eigen::Matrix3d RobotWorldFromWebXr()
{
    Eigen::Matrix3d transform;
    // WebXR: +X right, +Y up, -Z forward.
    // Robot world: +X forward, +Y left, +Z up.
    transform << 0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0;
    return transform;
}

Eigen::Matrix3d
ApplyOperatorFrame(const Eigen::Matrix3d &robot_world_from_webxr,
                   bool operator_facing_robot)
{
    if (!operator_facing_robot)
    {
        return robot_world_from_webxr;
    }
    // 面对面时操作者的前/左分别对应机器人的后/右，竖直轴保持一致。
    const Eigen::Matrix3d face_to_face_rotation =
        Eigen::AngleAxisd(kPi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    return face_to_face_rotation * robot_world_from_webxr;
}

bool RobotWorldFromHeadYaw(const PicoHeadsetPose &headset,
                           Eigen::Matrix3d &robot_world_from_webxr,
                           double &head_yaw)
{
    if (!headset.present || !headset.tracked)
    {
        return false;
    }
    const Eigen::Quaterniond quaternion(
        headset.orientation_xyzw[3], headset.orientation_xyzw[0],
        headset.orientation_xyzw[1], headset.orientation_xyzw[2]);
    if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1e-9)
    {
        return false;
    }

    Eigen::Vector3d forward_webxr =
        quaternion.normalized().toRotationMatrix() * -Eigen::Vector3d::UnitZ();
    forward_webxr.y() = 0.0;
    if (!forward_webxr.allFinite() || forward_webxr.norm() < 1e-3)
    {
        return false;
    }
    forward_webxr.normalize();
    const Eigen::Vector3d up_webxr = Eigen::Vector3d::UnitY();
    const Eigen::Vector3d left_webxr =
        up_webxr.cross(forward_webxr).normalized();

    Eigen::Matrix3d webxr_from_head_yaw;
    webxr_from_head_yaw.col(0) = forward_webxr;
    webxr_from_head_yaw.col(1) = left_webxr;
    webxr_from_head_yaw.col(2) = up_webxr;
    robot_world_from_webxr = webxr_from_head_yaw.transpose();
    head_yaw = std::atan2(forward_webxr.x(), -forward_webxr.z());
    return robot_world_from_webxr.allFinite() &&
           std::abs(robot_world_from_webxr.determinant() - 1.0) < 1e-6 &&
           std::isfinite(head_yaw);
}

double SecondsSince(const std::chrono::steady_clock::time_point &time)
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                         time)
        .count();
}

double SecondsSince(double timestamp_sec)
{
    const double now_sec =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count();
    return now_sec - timestamp_sec;
}

bool TakePicoSample(Pico &pico, PicoSample &sample)
{
    auto next = pico.GetSample();
    if (!next)
    {
        return false;
    }
    sample = std::move(*next);
    return true;
}

struct HandServoState
{
    uint64_t trace_sequence = 0;
    double trace_received = 0.0;
    double trace_taken = 0.0, trace_mapped = 0.0;
    bool active = false;
    bool require_release = false;
    bool target_valid = false;
    Eigen::Vector3d hand_start_position = Eigen::Vector3d::Zero();
    Eigen::Vector3d filtered_hand_position = Eigen::Vector3d::Zero();
    Eigen::Quaterniond filtered_hand_orientation =
        Eigen::Quaterniond::Identity();
    Eigen::Matrix3d robot_world_from_webxr_clutch = RobotWorldFromWebXr();
    double clutch_head_yaw = 0.0;
    Eigen::Isometry3d tcp_start_world = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d target_tcp_reference = Eigen::Isometry3d::Identity();
    Eigen::Vector3d target_rpy = Eigen::Vector3d::Zero();
    bool servop_stream_started = false;
    bool stop_requested = false;
    bool stop_from_joint6 = false;
    bool joint6_active = false;
    bool joint6_target_valid = false;
    rcore::sdk::JointPositions joint_target{};
    double joint6_velocity = 0.0;
    std::chrono::steady_clock::time_point last_joint6_update{};
    uint64_t command_frames = 0;
    double translation_step = 0.0;
    double rotation_step = 0.0;
    Eigen::Vector3d last_world_delta = Eigen::Vector3d::Zero();
    std::chrono::steady_clock::time_point last_xr_update{};
    double singularity_scale = 1.0;
    int64_t singularity_scale_updated_ns = 0;
    SingularityTravel singularity_travel = SingularityTravel::Nominal;
};

struct SingularitySlowdownStatus
{
    bool fresh = false;
    bool toward_valid = false;
    double condition_number = std::numeric_limits<double>::infinity();
    Eigen::Matrix<double, 6, 1> toward_singularity =
        Eigen::Matrix<double, 6, 1>::Zero();
};

int64_t SteadyNowNanoseconds()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

class SingularitySlowdownMonitor
{
public:
    explicit SingularitySlowdownMonitor(rcore::sdk::Robot &robot)
        : robot_(robot)
    {
    }

    ~SingularitySlowdownMonitor() { Stop(); }

    SingularitySlowdownMonitor(const SingularitySlowdownMonitor &) = delete;
    SingularitySlowdownMonitor &
    operator=(const SingularitySlowdownMonitor &) = delete;

    void Start()
    {
        if (running_.exchange(true, std::memory_order_acq_rel))
        {
            return;
        }
        thread_ = std::thread(&SingularitySlowdownMonitor::Run, this);
    }

    void Stop()
    {
        if (!running_.exchange(false, std::memory_order_acq_rel))
        {
            return;
        }
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

    bool WaitForInitialSample(std::chrono::milliseconds timeout) const
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (session_running->load(std::memory_order_acquire) &&
               std::chrono::steady_clock::now() < deadline)
        {
            if (Snapshot().fresh)
            {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        return Snapshot().fresh;
    }

    SingularitySlowdownStatus Snapshot() const
    {
        SingularitySlowdownStatus result;
        int64_t updated_at_ns = 0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            updated_at_ns = updated_at_ns_;
            result.condition_number = condition_number_;
            result.toward_singularity = toward_singularity_;
            result.toward_valid = toward_valid_;
        }
        const double age =
            static_cast<double>(SteadyNowNanoseconds() - updated_at_ns) / 1e9;
        result.fresh = updated_at_ns > 0 && std::isfinite(age) && age >= 0.0 &&
                       age <= kSingularityMonitorStaleSeconds;
        return result;
    }

private:
    static Eigen::Matrix<double, 6, 1>
    BaseTwistFromState(const rcore::RobotState &state)
    {
        Eigen::Matrix<double, 6, 1> twist;
        for (Eigen::Index axis = 0; axis < 6; ++axis)
        {
            twist[axis] = state.cartesian_velocity[static_cast<size_t>(axis)];
        }
        return twist;
    }

    void Run()
    {
        auto next_sample = std::chrono::steady_clock::now();
        bool have_previous = false;
        bool previous_toward_valid = false;
        double previous_condition = std::numeric_limits<double>::infinity();
        Eigen::Matrix<double, 6, 1> previous_toward =
            Eigen::Matrix<double, 6, 1>::Zero();
        while (running_.load(std::memory_order_acquire))
        {
            rcore::sdk::Jacobian sdk_jacobian;
            auto result = robot_.Motion().GetJacobian(sdk_jacobian);
            if (result.IsSuccess())
            {
                Eigen::Matrix<double, 6, 6> jacobian;
                for (Eigen::Index row = 0; row < 6; ++row)
                {
                    for (Eigen::Index column = 0; column < 6; ++column)
                    {
                        jacobian(row, column) =
                            sdk_jacobian(static_cast<size_t>(row),
                                         static_cast<size_t>(column));
                    }
                }

                const auto analysis = AnalyzeJacobian(jacobian);
                if (analysis.valid)
                {
                    Eigen::Matrix<double, 6, 1> toward =
                        analysis.least_singular_direction;
                    bool toward_valid = false;
                    const auto robot_state = robot_.GetState();
                    const Eigen::Matrix<double, 6, 1> base_twist =
                        BaseTwistFromState(robot_state);
                    const bool twist_usable =
                        base_twist.allFinite() &&
                        base_twist.norm() > kSingularityTwistAlignEpsilon;
                    if (have_previous && twist_usable &&
                        std::isfinite(previous_condition) &&
                        std::abs(analysis.condition_number -
                                 previous_condition) >
                            kSingularityConditionDeltaEpsilon)
                    {
                        toward = ResolveTowardFromObservedMotion(
                            toward, base_twist, analysis.condition_number,
                            previous_condition);
                        toward_valid = true;
                    }
                    else if (have_previous && previous_toward_valid)
                    {
                        toward =
                            AlignSingularDirection(toward, previous_toward);
                        toward_valid = true;
                    }

                    previous_condition = analysis.condition_number;
                    previous_toward = toward;
                    previous_toward_valid = toward_valid;
                    have_previous = true;

                    const int64_t now_ns = SteadyNowNanoseconds();
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        condition_number_ = analysis.condition_number;
                        toward_singularity_ = toward;
                        toward_valid_ = toward_valid;
                        updated_at_ns_ = now_ns;
                    }
                }
            }

            next_sample +=
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(
                        kSingularityMonitorPeriodSeconds));
            const auto now = std::chrono::steady_clock::now();
            if (next_sample < now)
            {
                next_sample = now;
            }
            std::this_thread::sleep_until(next_sample);
        }
    }

    rcore::sdk::Robot &robot_;
    std::atomic<bool> running_{false};
    mutable std::mutex mutex_;
    double condition_number_{std::numeric_limits<double>::infinity()};
    Eigen::Matrix<double, 6, 1> toward_singularity_{
        Eigen::Matrix<double, 6, 1>::Zero()};
    bool toward_valid_{false};
    int64_t updated_at_ns_{0};
    std::thread thread_;
};

// Estimate a forward stopping target, not a raw actuator trajectory.
// Existing controller Ruckig must own every output sample (ServoP and ServoJ
// share its continuous state). Measured feedback is not the last command.
class PicoStopTarget
{
public:
    static bool Build(const rcore::ControllerConfig &limits,
                      const rcore::RobotState &state,
                      rcore::sdk::JointPositions &target)
    {
        rcore::sdk::JointPositions candidate{};
        for (size_t joint = 0; joint < 6; ++joint)
        {
            const double q = state.positions[joint];
            const double v = state.velocities[joint];
            const double a = limits.max_acceleration[joint];
            const double j = limits.max_jerk[joint];
            if (!std::isfinite(q) || !std::isfinite(v) ||
                !std::isfinite(a) || a <= 0 || !std::isfinite(j) || j <= 0 ||
                !std::isfinite(limits.position_limit_min[joint]) ||
                !std::isfinite(limits.position_limit_max[joint]) ||
                limits.position_limit_min[joint] >= limits.position_limit_max[joint])
                return false;
            // Integral of a smooth velocity ramp to zero; only its endpoint
            // is used. This estimate cannot guarantee zero physical rebound.
            const double duration = std::max(
                0.02, std::max(1.5 * std::abs(v) / a,
                               std::sqrt(6.0 * std::abs(v) / j)));
            const double end = q + 0.5 * v * duration;
            if (!std::isfinite(duration) || duration > 2.0 ||
                !std::isfinite(end) ||
                q < limits.position_limit_min[joint] ||
                q > limits.position_limit_max[joint] ||
                end < limits.position_limit_min[joint] ||
                end > limits.position_limit_max[joint])
                return false;
            candidate[joint] = end;
        }
        target = candidate;
        return true;
    }
};

class PicoServoStopper
{
public:
    bool Configure(rcore::sdk::Robot &robot)
    {
        limits_ = robot.Config().GetControllerConfig();
        for (size_t joint = 0; joint < 6; ++joint)
        {
            if (!std::isfinite(limits_.max_velocity[joint]) ||
                limits_.max_velocity[joint] <= 0 ||
                !std::isfinite(limits_.velocity_percentage[joint]) ||
                limits_.velocity_percentage[joint] <= 0 ||
                limits_.velocity_percentage[joint] > 1 ||
                !std::isfinite(limits_.max_acceleration[joint]) ||
                limits_.max_acceleration[joint] <= 0 ||
                !std::isfinite(limits_.max_jerk[joint]) ||
                limits_.max_jerk[joint] <= 0 ||
                !std::isfinite(limits_.position_limit_min[joint]) ||
                !std::isfinite(limits_.position_limit_max[joint]) ||
                limits_.position_limit_min[joint] >= limits_.position_limit_max[joint])
                return false;
        }
        return true;
    }

    double LimitJ6Speed(double requested) const
    {
        return std::min(requested, 0.8 * limits_.max_velocity[5] *
                                      limits_.velocity_percentage[5]);
    }

    void Request(rcore::sdk::Robot &robot, bool = false)
    {
        if (pending_.valid())
            return;
        pending_ = std::async(
            std::launch::async,
            [&robot, limits = limits_]() -> std::string
            {
                try
                {
                    const auto initial = robot.GetState();
                    rcore::sdk::JointPositions hold{};
                    if (!PicoStopTarget::Build(limits, initial, hold))
                        return "invalid or out-of-bounds stop target";
                    if (!std::isfinite(initial.timestamp))
                        return "invalid feedback timestamp while stopping";
                    auto last_timestamp = initial.timestamp;
                    auto still_since = std::chrono::steady_clock::time_point{};
                    auto last_feedback_at = std::chrono::steady_clock::now();
                    const auto deadline = std::chrono::steady_clock::now() +
                                          std::chrono::seconds(3);
                    while (std::chrono::steady_clock::now() < deadline)
                    {
                        const auto state = robot.GetState();
                        const auto observed_at =
                            std::chrono::steady_clock::now();
                        if (observed_at - last_feedback_at >
                            std::chrono::milliseconds(100))
                            return "stale joint feedback while stopping";
                        if (state.errors.GetLatchedErrorCount() > 0)
                            return "controller fault while stopping";
                        if (std::isfinite(state.timestamp) &&
                            state.timestamp > last_timestamp)
                        {
                            last_timestamp = state.timestamp;
                            last_feedback_at = observed_at;
                            bool still = true;
                            for (size_t joint = 0; joint < 6; ++joint)
                            {
                                if (!std::isfinite(state.positions[joint]) ||
                                    !std::isfinite(state.velocities[joint]))
                                    return "invalid joint feedback while stopping";
                                still = still &&
                                    std::abs(state.velocities[joint]) <= 0.01 &&
                                    std::abs(state.positions[joint] -
                                             hold[joint]) <= 0.001;
                            }
                            const auto now = std::chrono::steady_clock::now();
                            if (!still)
                                still_since = {};
                            else if (still_since ==
                                     std::chrono::steady_clock::time_point{})
                                still_since = now;
                            else if (now - still_since >=
                                     std::chrono::milliseconds(50))
                                return {};
                        }
                        // Never replace the controller command reference with
                        // measured positions through raw passthrough.
                        robot.Motion().ServoJ(hold);
                        std::this_thread::sleep_for(
                            std::chrono::milliseconds(1));
                    }
                    return "stop did not reach fresh, stable standstill within "
                           "3s";
                }
                catch (const std::exception &error)
                {
                    return error.what();
                }
            });
    }
    bool Pending() const { return pending_.valid(); }
    bool Poll(std::string &error)
    {
        if (!pending_.valid() ||
            pending_.wait_for(std::chrono::seconds(0)) !=
                std::future_status::ready)
            return false;
        error = pending_.get();
        return true;
    }
    void Wait()
    {
        if (pending_.valid())
            pending_.wait();
    }

private:
    rcore::ControllerConfig limits_;
    std::future<std::string> pending_;
};

#ifdef SMRCORE_PICO_LEGACY_RELEASE_STOP
#include "legacy_release_stop.hpp"
using SelectedPicoStopper = PicoLegacyReleaseStopper;
#else
using SelectedPicoStopper = PicoServoStopper;
#endif

// Preflight is deliberately outside the streaming thread. Only a checked pose
// is published; a generation invalidates in-flight RPCs on clutch release.
class PicoTargetGuard
{
public:
    enum class SingularityPolicy { CandidateBoundary, MeasuredSlowdown };
    static SingularityPolicy PolicyForControlMode(
        smrcore::peripherals::PicoControlMode mode)
    {
        using Mode = smrcore::peripherals::PicoControlMode;
        if (mode == Mode::Fdcc || mode == Mode::CartesianImpedance)
        {
            return SingularityPolicy::MeasuredSlowdown;
        }
        return SingularityPolicy::CandidateBoundary;
    }
    struct Checked
    {
        bool ready = false;
        int64_t requested_ns = 0;
        int64_t completed_ns = 0;
        bool pending = false;
        double ik_ms = 0.0, fk_ms = 0.0;
        double trace_check_start = 0.0, trace_check_end = 0.0;
        uint64_t trace_sequence = 0;
        double trace_received = 0.0;
    double trace_taken = 0.0, trace_mapped = 0.0;
        Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
        Eigen::Isometry3d raw_target = Eigen::Isometry3d::Identity();
        Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
        std::array<double, 6> joints{};
        double condition = 0.0;
        std::string blocked_reason;
        std::string error;
    };
    explicit PicoTargetGuard(rcore::sdk::Robot &robot,
        SingularityPolicy policy = SingularityPolicy::CandidateBoundary)
        : robot_(robot), singularity_policy_(policy), worker_([this] { Run(); })
    {
    }
    ~PicoTargetGuard() { Stop(); }
    void Stop()
    {
        running_.store(false);
        if (worker_.joinable())
        {
            worker_.join();
        }
    }
    PicoTargetGuard(const PicoTargetGuard &) = delete;
    PicoTargetGuard &operator=(const PicoTargetGuard &) = delete;
    void Submit(const HandServoState &hand, const Eigen::Isometry3d *raw_target = nullptr)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // Identical stream frames must not refresh an old approval's age.
        if (active_ && frame_ == hand.command_frames)
        {
            return;
        }
        active_ = true;
        frame_ = hand.command_frames;
        pending_.trace_taken = hand.trace_taken;
        pending_.trace_mapped = hand.trace_mapped;
        pending_.trace_sequence = hand.trace_sequence;
        pending_.trace_received = hand.trace_received;
        pending_.target = hand.target_tcp_reference;
        pending_.raw_target = raw_target ? *raw_target : hand.target_tcp_reference;
        pending_.rpy = hand.target_rpy;
        pending_.requested_ns = SteadyNowNanoseconds();
        pending_valid_ = true;
        if (!started_ns_)
        {
            started_ns_ = pending_.requested_ns;
        }
    }
    void Cancel()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_)
        {
            ++generation_;
        }
        active_ = false;
        pending_valid_ = false;
        checked_ = {};
        started_ns_ = 0;
    }
    Checked Snapshot()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto result = checked_;
        const auto now = SteadyNowNanoseconds();
        const auto checked_ns = singularity_policy_ == SingularityPolicy::MeasuredSlowdown ?
            result.completed_ns : result.requested_ns;
        if (active_ && result.error.empty() && result.blocked_reason.empty() &&
            now - (result.ready ? checked_ns : started_ns_) >
                200000000)
        {
            result.pending = true;
            if (singularity_policy_ == SingularityPolicy::CandidateBoundary)
            {
                result.blocked_reason = "candidate check pending (>200ms)";
            }
        }
        return result;
    }

private:
    Eigen::Isometry3d Fk(const rcore::sdk::JointPositions &q)
    {
        rcore::sdk::Pose pose;
        auto result = robot_.Motion().ForwardKinematics(q, pose);
        if (!result.IsSuccess())
        {
            throw std::runtime_error("candidate FK failed");
        }
        Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
        t.translation() =
            Eigen::Vector3d(pose.tvec()[0], pose.tvec()[1], pose.tvec()[2]);
        t.linear() = EulerZyxToRotation(
            {pose.rvec()[0], pose.rvec()[1], pose.rvec()[2]});
        if (!t.matrix().allFinite())
        {
            throw std::runtime_error("nonfinite candidate FK");
        }
        return t;
    }
    void Validate(Checked &c)
    {
        if (!c.target.matrix().allFinite() || !c.rpy.allFinite())
        {
            throw std::runtime_error("candidate Cartesian target nonfinite");
        }
        rcore::sdk::Pose pose;
        for (int i = 0; i < 3; ++i)
        {
            pose.tvec()[i] = c.target.translation()[i];
            pose.rvec()[i] = c.rpy[i];
        }
        rcore::sdk::JointPositions q;
        const auto ik_started_ns = SteadyNowNanoseconds();
        auto result = robot_.Motion().InverseKinematics(pose, q);
        c.ik_ms = (SteadyNowNanoseconds() - ik_started_ns) / 1e6;
        if (!result.IsSuccess())
        {
            c.blocked_reason = "candidate IK rejected";
            c.ready = true;
            return;
        }
        const auto state = robot_.GetState();
        for (int i = 0; i < 6; ++i)
        {
            c.joints[i] = q[i];
            if (!std::isfinite(q[i]) || !std::isfinite(state.positions[i]))
            {
                throw std::runtime_error("candidate joint nonfinite");
            }
        }
        // IK continuity is owned by the backend ServoP chain; the client only
        // keeps the elbow/wrist clearance check here.
        if (!smrcore::peripherals::GuardJointCandidate(state.positions,
                                                       c.joints))
        {
            c.blocked_reason = "candidate elbow/wrist clearance";
            c.ready = true;
            return;
        }
        const auto fk_started_ns = SteadyNowNanoseconds();
        const auto t = Fk(q);
        c.fk_ms = (SteadyNowNanoseconds() - fk_started_ns) / 1e6;
        if ((t.translation() - c.target.translation()).norm() > 0.001 ||
            Eigen::AngleAxisd(t.linear() * c.target.linear().transpose())
                    .angle() > 0.01)
        {
            c.blocked_reason = "candidate IK/FK mismatch";
            c.ready = true;
            return;
        }
        // FDCC integrates its own joint state; impedance tracks an equilibrium
        // pose in torque mode. Neither executes the endpoint IK branch as a
        // joint trajectory. Use the measured Jacobian slowdown already applied
        // to the target, not an additional endpoint condition-number veto.
        // Finite values, IK reachability, clearance and FK consistency above
        // remain mandatory. This also avoids six sequential FK RPCs per input.
        if (singularity_policy_ == SingularityPolicy::MeasuredSlowdown)
        {
            c.ready = true;
            return;
        }
        // SDK only exposes J at measured q. Build J(candidate q) via FK;
        // do not confuse the measured-pose condition with a target check.
        Eigen::Matrix<double, 6, 6> j;
        constexpr double epsilon = 1e-5;
        for (int i = 0; i < 6; ++i)
        {
            auto shifted = q;
            shifted[i] += epsilon;
            const auto next = Fk(shifted);
            j.block<3, 1>(0, i) =
                (next.translation() - t.translation()) / epsilon;
            const Eigen::AngleAxisd delta(next.linear() *
                                          t.linear().transpose());
            j.block<3, 1>(3, i) = delta.axis() * delta.angle() / epsilon;
        }
        c.condition = AnalyzeJacobian(j).condition_number;
        if (!std::isfinite(c.condition) ||
            c.condition >= kSingularityHardConditionNumber)
        {
            c.blocked_reason =
                "candidate condition >=60: " + std::to_string(c.condition);
            c.ready = true;
            return;
        }
        c.ready = true;
    }
    void Run()
    {
        while (running_.load())
        {
            Checked candidate;
            uint64_t generation = 0;
            bool have = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                generation = generation_;
                have = pending_valid_ && checked_.error.empty();
                if (have)
                {
                    candidate = pending_;
                    pending_valid_ = false;
                }
            }
            try
            {
                if (have)
                {
                    candidate.trace_check_start = SteadyNowNanoseconds() / 1e9;
                    Validate(candidate);
                    candidate.completed_ns = SteadyNowNanoseconds();
                    candidate.trace_check_end = candidate.completed_ns / 1e9;
                }
            }
            catch (const std::exception &e)
            {
                candidate.error = e.what();
                candidate.ready = false;
            }
            if (have)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (generation == generation_ && active_)
                {
                    checked_ = candidate;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    rcore::sdk::Robot &robot_;
    const SingularityPolicy singularity_policy_;
    std::atomic<bool> running_{true};
    std::mutex mutex_;
    Checked pending_, checked_;
    uint64_t generation_ = 0, frame_ = 0;
    int64_t started_ns_ = 0;
    bool active_ = false, pending_valid_ = false;
    std::thread worker_;
};

double FilterAlpha(double dt, double time_constant)
{
    if (!std::isfinite(dt) || dt <= 0.0 || time_constant <= 0.0)
    {
        return 1.0;
    }
    return 1.0 - std::exp(-dt / time_constant);
}

bool GetTcpInReference(const rcore::RobotState &robot_state,
                       Eigen::Isometry3d &tcp_in_reference)
{
    if (!robot_state.cartesian_valid)
    {
        return false;
    }
    const Eigen::Vector3d position(robot_state.cartesian_position[0],
                                   robot_state.cartesian_position[1],
                                   robot_state.cartesian_position[2]);
    const Eigen::Vector3d rpy(robot_state.cartesian_orientation[0],
                              robot_state.cartesian_orientation[1],
                              robot_state.cartesian_orientation[2]);
    tcp_in_reference = MakeIsometry(position, rpy);
    return tcp_in_reference.matrix().allFinite();
}

bool SetTargetFromRobotState(const rcore::RobotState &robot_state,
                             HandServoState &state)
{
    Eigen::Isometry3d current_tcp = Eigen::Isometry3d::Identity();
    if (!GetTcpInReference(robot_state, current_tcp))
    {
        state.target_valid = false;
        return false;
    }
    state.target_tcp_reference = current_tcp;
    state.target_rpy = {
        robot_state.cartesian_orientation[0],
        robot_state.cartesian_orientation[1],
        robot_state.cartesian_orientation[2],
    };
    state.target_valid = state.target_rpy.allFinite();
    return state.target_valid;
}

bool SetJointTargetFromRobotState(const rcore::RobotState &robot_state,
                                  HandServoState &state)
{
    for (size_t joint = 0; joint < 6; ++joint)
    {
        if (!std::isfinite(robot_state.positions[joint]))
        {
            state.joint6_target_valid = false;
            return false;
        }
        state.joint_target[joint] = robot_state.positions[joint];
    }
    state.joint6_target_valid = true;
    return true;
}

void FreezeHand(const rcore::RobotState &robot_state, HandServoState &state,
                bool require_release)
{
    if (!state.stop_requested && (state.joint6_active ||
        (state.active && state.servop_stream_started)))
        state.stop_from_joint6 = state.joint6_active;
    state.stop_requested = state.stop_requested ||
                           (state.active && state.servop_stream_started) ||
                           state.joint6_active;
    (void)SetJointTargetFromRobotState(robot_state, state);
    state.active = false;
    state.joint6_active = false;
    state.require_release = state.require_release || require_release;
    state.servop_stream_started = false;
    state.joint6_velocity = 0.0;
    state.last_joint6_update = {};
    state.translation_step = 0.0;
    state.rotation_step = 0.0;
    state.singularity_scale = 1.0;
    state.singularity_scale_updated_ns = 0;
    state.singularity_travel = SingularityTravel::Nominal;
    (void)SetTargetFromRobotState(robot_state, state);
}

double NormalizeJ6Thumbstick(double axis)
{
    if (!std::isfinite(axis))
    {
        return 0.0;
    }
    axis = std::clamp(axis, -1.0, 1.0);
    const double magnitude = std::abs(axis);
    if (magnitude <= kJ6ThumbstickDeadzone)
    {
        return 0.0;
    }
    const double remapped =
        (magnitude - kJ6ThumbstickDeadzone) / (1.0 - kJ6ThumbstickDeadzone);
    return std::copysign(remapped, kJ6ThumbstickAxisSign * axis);
}

bool ReadJointPositions(const rcore::RobotState &robot_state,
                        rcore::sdk::JointPositions &positions)
{
    for (size_t joint = 0; joint < 6; ++joint)
    {
        if (!std::isfinite(robot_state.positions[joint]))
        {
            return false;
        }
        positions[joint] = robot_state.positions[joint];
    }
    return true;
}

bool JointsNearStill(const rcore::RobotState &robot_state)
{
    constexpr double kDisableVelocityThreshold = 0.01;
    for (size_t joint = 0; joint < 6; ++joint)
    {
        if (!std::isfinite(robot_state.velocities[joint]) ||
            std::abs(robot_state.velocities[joint]) > kDisableVelocityThreshold)
        {
            return false;
        }
    }
    return true;
}

void PrintDisableStatus(const char *arm, rcore::sdk::Result &result,
                        const rcore::MotorStatus &status)
{
    std::cout << arm << " Disable rpc=" << (result.IsSuccess() ? "ok" : "fail")
              << "(" << result.GetErrorCode() << ") " << result.GetErrorMsg()
              << " enabled=" << (status.enabled ? "true" : "false")
              << " operational=" << (status.operational ? "true" : "false")
              << std::endl;
}

// Cia402Motor::Disable() refuses when any axis exceeds 0.01 rad/s. The
// tracking arm is often still catching the last ServoP when Ctrl+C arrives,
// while the idle arm is already still — only the idle arm would drop enable.
void HoldThenDisable(rcore::sdk::Robot &robot, const char *arm)
{
    rcore::sdk::JointPositions hold;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline)
    {
        const auto state = robot.GetState();
        if (state.errors.GetLatchedErrorCount() > 0)
        {
            break;
        }
        if (!ReadJointPositions(state, hold))
        {
            break;
        }
        robot.Motion().ServoJ(hold);
        if (JointsNearStill(state))
        {
            break;
        }
        std::this_thread::sleep_for(kServoPeriod);
    }

    for (int attempt = 0; attempt < 5; ++attempt)
    {
        auto result = robot.Disable();
        const auto status = robot.Control().GetMotorStatus();
        PrintDisableStatus(arm, result, status);
        if (result.IsSuccess() && !status.enabled)
        {
            return;
        }
        const auto state = robot.GetState();
        if (ReadJointPositions(state, hold))
        {
            robot.Motion().ServoJ(hold);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cerr << arm << " 退出时仍使能，请在硬件侧确认 Disable。" << std::endl;
}

bool EnsureSdkCommandSource(rcore::sdk::Robot &robot, const char *arm,
                            std::string &error)
{
    if (robot.Control().GetCommandSource() == rcore::sdk::CommandSource::Sdk)
    {
        return true;
    }
    auto result =
        robot.Control().SetCommandSource(rcore::sdk::CommandSource::Sdk);
    if (!result.IsSuccess())
    {
        error = std::string(arm) + " 获取 SDK 命令源失败: [" +
                std::to_string(result.GetErrorCode()) + "] " +
                result.GetErrorMsg();
        return false;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (robot.Control().GetCommandSource() ==
            rcore::sdk::CommandSource::Sdk)
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    error = std::string(arm) + " SDK 命令源状态 1s 内未同步";
    return false;
}

class RobotStateFreshnessWatchdog
{
public:
    bool Observe(const rcore::RobotState &robot_state,
                 std::chrono::steady_clock::time_point observed_at)
    {
        if (!std::isfinite(robot_state.timestamp))
        {
            return false;
        }
        if (!seen_ || robot_state.timestamp != last_timestamp_)
        {
            seen_ = true;
            last_timestamp_ = robot_state.timestamp;
            last_change_at_ = observed_at;
        }
        return seen_ &&
               std::chrono::duration<double>(observed_at - last_change_at_)
                       .count() <= kRobotStateStaleSeconds;
    }

private:
    bool seen_ = false;
    double last_timestamp_ = 0.0;
    std::chrono::steady_clock::time_point last_change_at_{};
};

void PrintLatchedErrors(const char *arm, const rcore::RobotState &robot_state)
{
    const auto &errors = robot_state.errors.GetLatchedErrors();
    for (size_t index = 0; index < robot_state.errors.GetLatchedErrorCount();
         ++index)
    {
        std::cerr << " " << arm << "[code=" << errors[index].code
                  << ",level=" << static_cast<int>(errors[index].level) << "]";
    }
}

double OrientationErrorDegrees(const rcore::RobotState &robot_state,
                               const HandServoState &state)
{
    if (!robot_state.cartesian_valid)
    {
        return -1.0;
    }
    const Eigen::Matrix3d actual_rotation =
        EulerZyxToRotation({robot_state.cartesian_orientation[0],
                            robot_state.cartesian_orientation[1],
                            robot_state.cartesian_orientation[2]});
    const Eigen::AngleAxisd error(actual_rotation.transpose() *
                                  state.target_tcp_reference.linear());
    return std::isfinite(error.angle()) ? error.angle() * 180.0 / kPi : -1.0;
}

double PositionErrorMillimeters(const rcore::RobotState &robot_state,
                                const HandServoState &state)
{
    if (!robot_state.cartesian_valid)
    {
        return -1.0;
    }
    const Eigen::Vector3d actual(robot_state.cartesian_position[0],
                                 robot_state.cartesian_position[1],
                                 robot_state.cartesian_position[2]);
    const double error =
        (state.target_tcp_reference.translation() - actual).norm() * 1000.0;
    return std::isfinite(error) ? error : -1.0;
}

bool SendSdkServoPTick(rcore::sdk::Robot &robot, HandServoState &state)
{
    if (state.joint6_active)
    {
        if (!state.joint6_target_valid ||
            !std::isfinite(state.joint6_velocity))
        {
            return false;
        }

        for (size_t joint = 0; joint < 6; ++joint)
        {
            if (!std::isfinite(state.joint_target[joint]))
            {
                return false;
            }
        }

        if (state.joint6_active)
        {
            const auto now = std::chrono::steady_clock::now();
            double dt =
                std::chrono::duration<double>(now - state.last_joint6_update)
                    .count();
            if (!std::isfinite(dt) || dt <= 0.0 || dt > kInputStaleSeconds)
            {
                dt = 0.0;
            }
            state.joint_target[5] += state.joint6_velocity * dt;
            state.last_joint6_update = now;
            if (!std::isfinite(state.joint_target[5]))
            {
                state.joint6_active = false;
                state.joint6_target_valid = false;
                return false;
            }
        }

        // Keep the same planner state when entering J6 and when stopping.
        robot.Motion().ServoJ(state.joint_target);
        return true;
    }

    const bool tracking = state.active && state.servop_stream_started;
    if (!tracking || !state.target_valid ||
        !state.target_tcp_reference.matrix().allFinite() ||
        !state.target_rpy.allFinite())
    {
        return false;
    }

    rcore::sdk::Pose pose;
    for (Eigen::Index axis = 0; axis < 3; ++axis)
    {
        pose.tvec()[axis] = state.target_tcp_reference.translation()[axis];
        pose.rvec()[axis] = state.target_rpy[axis];
    }
    robot.Motion().ServoP(pose);
    return true;
}

void PrintArmDiagnostics(const char *name, const PicoControllerPose &pose,
                         const rcore::RobotState &robot_state,
                         const HandServoState &hand,
                         const ArmReference &reference,
                         const SingularitySlowdownStatus &singularity)
{
    const Eigen::Isometry3d actual_reference = [&]()
    {
        Eigen::Isometry3d tcp = Eigen::Isometry3d::Identity();
        if (!GetTcpInReference(robot_state, tcp))
        {
            tcp.translation() =
                Eigen::Vector3d(robot_state.cartesian_position[0],
                                robot_state.cartesian_position[1],
                                robot_state.cartesian_position[2]);
        }
        return tcp;
    }();
    const Eigen::Vector3d actual_world =
        (reference.world_from_reference * actual_reference).translation();
    const Eigen::Vector3d target_world =
        (reference.world_from_reference * hand.target_tcp_reference)
            .translation();
    const char *grip_hint = "";
    if (hand.require_release)
    {
        grip_hint = " wait_release";
    }
    else if (!hand.active && pose.grip > kGripReleaseThreshold &&
             pose.grip < kGripPressThreshold)
    {
        grip_hint = " grip_light";
    }
    std::cout << name << " grip=" << std::fixed << std::setprecision(2)
              << pose.grip << " clutch=" << (hand.active ? "ON" : "OFF")
              << " stick_y=" << pose.thumbstick[1]
              << " j6=" << (hand.joint6_active ? "JOINT" : "OFF") << grip_hint
              << " stream=";
    if (hand.joint6_active)
    {
        std::cout << "J6";
    }
    else if (hand.active && hand.servop_stream_started)
    {
        std::cout << "TRACK";
    }
    else if (hand.stop_requested)
    {
        std::cout << "BRAKE";
    }
    else
    {
        std::cout << "OFF";
    }
    std::cout << " frames=" << hand.command_frames
              << " dxyz=" << hand.translation_step * 1000.0 << "mm"
              << " drot=" << hand.rotation_step * 180.0 / kPi << "deg"
              << " cond=";
    if (singularity.fresh && std::isfinite(singularity.condition_number))
    {
        std::cout << singularity.condition_number;
    }
    else if (singularity.fresh)
    {
        std::cout << "inf";
    }
    else
    {
        std::cout << "stale";
    }
    std::cout << " sing=" << SingularityTravelToString(hand.singularity_travel)
              << " sing_scale=" << hand.singularity_scale
              << " map_yaw=" << hand.clutch_head_yaw * 180.0 / kPi << "deg"
              << " pos_err=" << PositionErrorMillimeters(robot_state, hand)
              << "mm ori_err=" << OrientationErrorDegrees(robot_state, hand)
              << "deg";
    if (hand.active && hand.servop_stream_started &&
        PositionErrorMillimeters(robot_state, hand) > 15.0)
    {
        std::cout << " LAG";
    }
    std::cout << "\n"
              << "  ref=\"" << reference.name << "\" arm xyz=["
              << robot_state.cartesian_position[0] << ","
              << robot_state.cartesian_position[1] << ","
              << robot_state.cartesian_position[2] << "] target_arm=["
              << hand.target_tcp_reference.translation()[0] << ","
              << hand.target_tcp_reference.translation()[1] << ","
              << hand.target_tcp_reference.translation()[2] << "]\n"
              << "  world xyz=[" << actual_world[0] << "," << actual_world[1]
              << "," << actual_world[2] << "] target_world=[" << target_world[0]
              << "," << target_world[1] << "," << target_world[2] << "] dW=["
              << hand.last_world_delta[0] << "," << hand.last_world_delta[1]
              << "," << hand.last_world_delta[2] << "]" << std::endl;
}

void UpdateHandTarget(const PicoControllerPose &pose,
                      const PicoHeadsetPose &headset,
                      const rcore::RobotState &robot_state,
                      const ArmReference &reference, bool enable_rotation,
                      bool head_yaw_clutched, double translation_scale,
                      double rotation_scale, double j6_thumbstick_speed,
                      bool operator_facing_robot,
                      const SingularitySlowdownStatus &singularity,
                      HandServoState &state)
{
    if (!pose.present || !pose.tracked || !ValidPicoControllerNumbers(pose))
    {
        FreezeHand(robot_state, state, true);
        return;
    }
    if (pose.grip <= kGripReleaseThreshold)
    {
        if (state.active || state.joint6_active || state.require_release)
        {
            FreezeHand(robot_state, state, false);
        }
        state.require_release = false;
        return;
    }
    if (state.require_release ||
        (!state.active && pose.grip < kGripPressThreshold))
    {
        return;
    }

    const double j6_axis = NormalizeJ6Thumbstick(pose.thumbstick[1]);
    const auto now = std::chrono::steady_clock::now();
    if (std::abs(j6_axis) > 0.0)
    {
        if (!state.joint6_active &&
            !SetJointTargetFromRobotState(robot_state, state))
        {
            FreezeHand(robot_state, state, true);
            return;
        }
        if (state.joint6_active && !state.joint6_target_valid)
        {
            FreezeHand(robot_state, state, true);
            return;
        }

        if (!state.joint6_active && state.active && state.servop_stream_started)
        {
            FreezeHand(robot_state, state, false);
            return;
        }
        if (!state.joint6_active)
        {
            // Switching from ServoP to J6 is a deliberate mode transition.
            // Start from measured joint positions so the first passthrough
            // frame cannot jump the other five joints.
            state.active = false;
            state.servop_stream_started = false;
            state.joint6_active = true;
            state.last_joint6_update = now;
            state.command_frames = 0;
        }
        state.joint6_velocity = j6_axis * j6_thumbstick_speed;
        ++state.command_frames;
        return;
    }

    if (state.joint6_active)
    {
        FreezeHand(robot_state, state, false);
        return;
    }

    if (!reference.valid)
    {
        FreezeHand(robot_state, state, true);
        return;
    }

    Eigen::Isometry3d current_tcp_reference = Eigen::Isometry3d::Identity();
    if (!GetTcpInReference(robot_state, current_tcp_reference))
    {
        FreezeHand(robot_state, state, true);
        return;
    }

    const Eigen::Vector3d hand_position = WebXrPosition(pose);
    const Eigen::Matrix3d hand_orientation = WebXrFromGrip(pose);
    if (!state.active)
    {
        Eigen::Matrix3d robot_world_from_webxr = RobotWorldFromWebXr();
        double head_yaw = 0.0;
        if (head_yaw_clutched &&
            !RobotWorldFromHeadYaw(headset, robot_world_from_webxr, head_yaw))
        {
            FreezeHand(robot_state, state, true);
            std::cerr << "\n头显水平朝向不可用，已拒绝 Grip 并要求松开后重试。"
                      << std::endl;
            return;
        }
        robot_world_from_webxr =
            ApplyOperatorFrame(robot_world_from_webxr, operator_facing_robot);
        if (operator_facing_robot)
        {
            head_yaw = std::remainder(head_yaw + kPi, 2.0 * kPi);
        }
        state.active = true;
        // Hold the current TCP immediately. Waiting for a 1e-6 m first step
        // leaves a ServoP gap after clutch-in; the controller then coasts
        // while the headset is already moving.
        state.servop_stream_started = true;
        state.robot_world_from_webxr_clutch = robot_world_from_webxr;
        state.clutch_head_yaw = head_yaw;
        state.hand_start_position = hand_position;
        state.filtered_hand_position = hand_position;
        state.filtered_hand_orientation =
            Eigen::Quaterniond(hand_orientation).normalized();
        state.tcp_start_world =
            reference.world_from_reference * current_tcp_reference;
        (void)SetTargetFromRobotState(robot_state, state);
        state.command_frames = 0;
        state.last_xr_update = now;
        state.singularity_scale = 1.0;
        state.singularity_scale_updated_ns = 0;
        state.singularity_travel = SingularityTravel::Nominal;
        return;
    }

    const double dt =
        std::chrono::duration<double>(now - state.last_xr_update).count();
    if (!std::isfinite(dt) || dt <= 0.0 || dt > kInputStaleSeconds)
    {
        // Keep Grip held: next fresh packet re-clutches from current TCP.
        FreezeHand(robot_state, state, false);
        return;
    }
    state.last_xr_update = now;

    Eigen::Quaterniond hand_quaternion(hand_orientation);
    hand_quaternion.normalize();
    if (state.filtered_hand_orientation.dot(hand_quaternion) < 0.0)
    {
        hand_quaternion.coeffs() *= -1.0;
    }
    const Eigen::Quaterniond previous_hand_orientation =
        state.filtered_hand_orientation;
    const double position_alpha =
        FilterAlpha(dt, kHandPositionFilterTimeConstant);
    const Eigen::Vector3d previous_hand_position = state.filtered_hand_position;
    state.filtered_hand_position +=
        position_alpha * (hand_position - state.filtered_hand_position);
    const double orientation_alpha =
        FilterAlpha(dt, kHandOrientationFilterTimeConstant);
    state.filtered_hand_orientation =
        state.filtered_hand_orientation
            .slerp(orientation_alpha, hand_quaternion)
            .normalized();

    const Eigen::Vector3d unscaled_translation_increment =
        translation_scale *
        (state.robot_world_from_webxr_clutch *
         (state.filtered_hand_position - previous_hand_position));

    const Eigen::Matrix3d raw_rotation_delta_webxr =
        state.filtered_hand_orientation.toRotationMatrix() *
        previous_hand_orientation.toRotationMatrix().transpose();
    const Eigen::AngleAxisd rotation_delta_axis(raw_rotation_delta_webxr);
    if (!std::isfinite(rotation_delta_axis.angle()) ||
        !rotation_delta_axis.axis().allFinite())
    {
        FreezeHand(robot_state, state, true);
        return;
    }
    const double unscaled_rotation_angle =
        enable_rotation
            ? std::min(rotation_scale * rotation_delta_axis.angle(),
                       kMaxOrientationTargetRateRadiansPerSecond * dt)
            : 0.0;
    const Eigen::Matrix3d &world_from_webxr =
        state.robot_world_from_webxr_clutch;
    const Eigen::Vector3d unscaled_rotation_increment_world =
        unscaled_rotation_angle *
        (world_from_webxr * rotation_delta_axis.axis());

    const Eigen::Matrix3d base_from_world =
        reference.world_from_reference.linear().transpose();
    Eigen::Matrix<double, 6, 1> commanded_twist_base;
    commanded_twist_base.head<3>() =
        base_from_world * unscaled_translation_increment;
    commanded_twist_base.tail<3>() =
        base_from_world * unscaled_rotation_increment_world;
    const bool approaching =
        !singularity.toward_valid ||
        IsApproachingSingularity(singularity.toward_singularity,
                                 commanded_twist_base);
    double singularity_speed_scale =
        ConditionNumberToSpeedScale(singularity.condition_number, approaching);
    if (state.singularity_scale_updated_ns > 0)
    {
        singularity_speed_scale = FilterSingularityRecoveryScale(
            singularity_speed_scale, state.singularity_scale, dt,
            kSingularityRecoveryFilterTimeConstantSeconds);
    }
    singularity_speed_scale = std::clamp(singularity_speed_scale, 0.0, 1.0);
    state.singularity_scale = singularity_speed_scale;
    state.singularity_scale_updated_ns = SteadyNowNanoseconds();
    state.singularity_travel = ClassifySingularityTravel(
        singularity.condition_number, singularity.toward_valid, approaching);

    const Eigen::Isometry3d current_tcp_world =
        reference.world_from_reference * current_tcp_reference;
    const Eigen::Vector3d previous_target_world =
        (reference.world_from_reference * state.target_tcp_reference)
            .translation();
    const Eigen::Vector3d unscaled_target_world =
        previous_target_world + unscaled_translation_increment;
    Eigen::Vector3d next_target_world =
        previous_target_world +
        singularity_speed_scale * unscaled_translation_increment;
    if (approaching && singularity_speed_scale < 1.0)
    {
        // Scaling only the newest hand increment leaves an old ServoP target
        // backlog in front of the measured TCP. The robot then keeps moving
        // toward the singularity even when scale reaches zero. Scale the
        // complete tracking error so zero means an actual measured-pose hold.
        next_target_world =
            current_tcp_world.translation() +
            singularity_speed_scale *
                (unscaled_target_world - current_tcp_world.translation());
    }
    Eigen::Vector3d translation_delta =
        next_target_world - state.tcp_start_world.translation();
    const double translation_norm = translation_delta.norm();
    if (translation_norm > kMaxTranslationDeltaMeters)
    {
        translation_delta *= kMaxTranslationDeltaMeters / translation_norm;
    }
    state.last_world_delta = translation_delta;

    const Eigen::Matrix3d unscaled_rotation_delta_webxr =
        Eigen::AngleAxisd(unscaled_rotation_angle, rotation_delta_axis.axis())
            .toRotationMatrix();
    const Eigen::Matrix3d unscaled_rotation_delta_world =
        world_from_webxr * unscaled_rotation_delta_webxr *
        world_from_webxr.transpose();
    const Eigen::Matrix3d previous_target_rotation_world =
        reference.world_from_reference.linear() *
        state.target_tcp_reference.linear();
    Eigen::Quaterniond next_target_rotation_world(
        unscaled_rotation_delta_world * previous_target_rotation_world);
    if (approaching && singularity_speed_scale < 1.0)
    {
        next_target_rotation_world =
            Eigen::Quaterniond(current_tcp_world.linear())
                .slerp(singularity_speed_scale,
                       next_target_rotation_world.normalized());
    }
    else
    {
        next_target_rotation_world =
            Eigen::Quaterniond(previous_target_rotation_world)
                .slerp(singularity_speed_scale,
                       next_target_rotation_world.normalized());
    }

    Eigen::Isometry3d target_tcp_world = state.tcp_start_world;
    target_tcp_world.translation() =
        state.tcp_start_world.translation() + translation_delta;
    target_tcp_world.linear() =
        next_target_rotation_world.normalized().toRotationMatrix();

    const Eigen::Isometry3d target_tcp_reference =
        reference.world_from_reference.inverse() * target_tcp_world;
    if (!target_tcp_reference.matrix().allFinite())
    {
        FreezeHand(robot_state, state, true);
        return;
    }

    const Eigen::Isometry3d previous_target = state.target_tcp_reference;
    state.target_tcp_reference = target_tcp_reference;
    state.target_rpy = RotationToContinuousEulerZyx(
        state.target_tcp_reference.linear(), state.target_rpy);
    state.translation_step = (state.target_tcp_reference.translation() -
                              previous_target.translation())
                                 .norm();
    const Eigen::AngleAxisd rotation_step(state.target_tcp_reference.linear() *
                                          previous_target.linear().transpose());
    state.rotation_step =
        std::isfinite(rotation_step.angle()) ? rotation_step.angle() : 0.0;
    state.servop_stream_started =
        state.servop_stream_started ||
        state.translation_step >= kServoPStartTranslationMeters ||
        state.rotation_step >= kServoPStartRotationRadians;
    state.target_valid = true;
    ++state.command_frames;
}

bool WaitForControllers(Pico &pico, PicoSample &packet, bool single_right = false)
{
    std::cout << (single_right ? "\n等待 PICO 右手柄进入 WebXR..." : "\n等待 PICO 左右手柄进入 WebXR...") << std::endl;
    while (session_running->load())
    {
        if (TakePicoSample(pico, packet) &&
            SecondsSince(packet.timestamp_sec) <= kInputStaleSeconds &&
            (single_right || packet.left.tracked) && packet.right.tracked)
        {
            std::cout << (single_right ? "右手柄 6DoF 已就绪，参考空间: " : "左右手柄 6DoF 已就绪，参考空间: ")
                      << packet.reference_space << std::endl;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}

bool WaitForReleasedGrips(Pico &pico, bool single_right = false)
{
    std::cout << (single_right ? "请松开右 Grip，保持 0.5 秒..." : "请松开左右 Grip，保持 0.5 秒...") << std::endl;
    auto released_since = std::chrono::steady_clock::time_point{};
    auto last_print = std::chrono::steady_clock::now();
    PicoSample packet;
    bool has_packet = false;
    while (session_running->load())
    {
        has_packet = TakePicoSample(pico, packet) || has_packet;
        const bool fresh = has_packet && SecondsSince(packet.timestamp_sec) <=
                                             kInputStaleSeconds;
        const bool released = fresh &&
                              (single_right || (packet.left.tracked &&
                               packet.left.grip <= kGripReleaseThreshold)) &&
                              packet.right.tracked &&
                              packet.right.grip <= kGripReleaseThreshold;
        if (!released)
        {
            released_since = {};
        }
        else if (released_since == std::chrono::steady_clock::time_point{})
        {
            released_since = std::chrono::steady_clock::now();
        }
        else if (SecondsSince(released_since) >= 0.5)
        {
            std::cout << "Grip 已松开，开始连接机器人。" << std::endl;
            return true;
        }
        if (SecondsSince(last_print) >= 0.5)
        {
            if (fresh)
            {
                std::cout << "等待松开 Grip: " << std::fixed << std::setprecision(2);
                if (!single_right) { std::cout << "L=" << packet.left.grip << " "; }
                std::cout << "R=" << packet.right.grip << std::endl;
            }
            else
            {
                std::cout << "等待新的 PICO 输入..." << std::endl;
            }
            last_print = std::chrono::steady_clock::now();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

bool ValidateHeadYawMapping()
{
    PicoHeadsetPose headset;
    headset.present = true;
    headset.tracked = true;
    Eigen::Matrix3d mapping;
    double yaw = 0.0;
    if (!RobotWorldFromHeadYaw(headset, mapping, yaw) ||
        (mapping * -Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitX())
                .norm() > 1e-9 ||
        std::abs(yaw) > 1e-9)
    {
        return false;
    }

    const Eigen::Quaterniond turned_right(
        Eigen::AngleAxisd(-kPi / 2.0, Eigen::Vector3d::UnitY()));
    headset.orientation_xyzw = {
        turned_right.x(),
        turned_right.y(),
        turned_right.z(),
        turned_right.w(),
    };
    return RobotWorldFromHeadYaw(headset, mapping, yaw) &&
           (mapping * Eigen::Vector3d::UnitX() - Eigen::Vector3d::UnitX())
                   .norm() <= 1e-9 &&
           std::abs(yaw - kPi / 2.0) <= 1e-9 &&
           (ApplyOperatorFrame(RobotWorldFromWebXr(), true) *
                -Eigen::Vector3d::UnitZ() +
            Eigen::Vector3d::UnitX())
                   .norm() <= 1e-9 &&
           (ApplyOperatorFrame(RobotWorldFromWebXr(), true) *
                Eigen::Vector3d::UnitX() -
            Eigen::Vector3d::UnitY())
                   .norm() <= 1e-9;
}

} // namespace

namespace
{
// Independent single-arm connection. Only physical right-hand input is used.
int RunSingleRightSession(const Options &options,
                          std::array<std::atomic<double>, 16> &diagnostics)
{
    const bool use_impedance = options.control_mode == PicoControlMode::CartesianImpedance;
    const bool use_compliance = options.control_mode != PicoControlMode::ServoP;
    const char *mode_name = use_impedance ? "Impedance" : use_compliance ? "FDCC" : "ServoP";
    Pico pico;
    PicoOptions pico_options;
    pico_options.port = options.port;
    if (!pico.Initialize(pico_options) || !pico.Start())
    {
        std::cerr << "PICO 服务启动失败: " << pico.GetLastError() << '\n';
        return 1;
    }
    std::cout << "独立单臂 / 物理右手柄 / SDK " << mode_name << " / base 参考系\n"
              << "单臂映射绕 base Z 偏转: " << options.single_mapping_yaw_deg
              << " deg；方向基准: "
              << (options.head_yaw_clutched ? "每次 Grip 锁定头显 yaw" : "WebXR room-fixed")
              << "；操作者: " << (options.operator_facing_robot ? "face-to-face" : "same-side") << '\n'
              << "PICO: http://127.0.0.1:" << options.port << "\n"
              << "adb reverse tcp:" << options.port << " tcp:" << options.port
              << "\n按住右 Grip 跟随；松开制动。左手柄不参与控制。\n";
    PicoSample packet;
    if (!WaitForControllers(pico, packet, true) ||
        !WaitForReleasedGrips(pico, true))
    {
        return 0;
    }
    ValidatePicoPacket(packet, true, options.head_yaw_clutched);
    rcore::sdk::Robot robot;
    rcore::sdk::RobotOptions robot_options;
    robot_options.arm_instance = rcore::sdk::RobotArmInstance::Single;
    if (!robot.Initialize(options.robot_ip, robot_options))
    {
        std::cerr << "独立单臂连接失败\n";
        return 1;
    }
    // Declared before workers: exceptions destroy/join workers before stopping
    // the robot. The guard remains armed until normal teardown completes.
    struct Cleanup
    {
        rcore::sdk::Robot &robot;
        bool armed = true;
        ~Cleanup()
        {
            if (armed)
            {
                try { robot.Motion().StopMotion(); } catch (...) {}
                try { robot.Disable(); } catch (...) {}
            }
        }
    } cleanup{robot};
    std::string error;
    ArmReference reference;
    if (!EnsureSdkCommandSource(robot, "single", error) ||
        !ResolveArmReference(robot, robot.GetState(),
                             SingleWorldFromBase(options.single_mapping_yaw_deg),
                             reference, error))
    {
        std::cerr << error << '\n';
        return 1;
    }
    SelectedPicoStopper stopper;
    if (!use_compliance && !stopper.Configure(robot))
    {
        std::cerr << "无法读取有效的关节制动限制\n";
        return 1;
    }
    if (use_compliance)
    {
        if (static_cast<int>(robot.GetState().control_mode) != 0)
        {
            std::cerr << mode_name << " 遥操需从运动学持位模式启动（mode=0）\n";
            return 1;
        }
        if (use_impedance)
        {
            std::cout << "笛卡尔阻抗 CST；平衡点经 DDS 更新，200Hz 上限\nK=[";
            for (size_t i = 0; i < 6; ++i)
            {
                std::cout << (i ? "," : "") << options.impedance_stiffness[i];
            }
            std::cout << "] D=[";
            for (size_t i = 0; i < 6; ++i)
            {
                std::cout << (i ? "," : "") << options.impedance_damping[i];
            }
            std::cout << "]；desired_wrench=0；SDK 默认死区/饱和与任务阻尼\n"
                      << "平移目标上限: " << options.impedance_max_speed << "m/s, "
                      << options.impedance_max_accel << "m/s2；入口跟随增益="
                      << options.impedance_follow_gain << "/s\n"
                      << "转动目标上限: 0.8rad/s, 3rad/s2；无虚拟目标放大\n"
                      << "松开 Grip 经阻尼退出后持位；重新握紧继续；摇杆 J6 不参与控制。\n";
        }
        else
        {
            // Read-only compatibility query: preserve the cabinet's calibrated
            // force source. The backend validates its readiness at activation.
            const auto source = robot.Control().GetFdCartesianAdmittanceWrenchSource();
            std::cout << "FDCC force_source=" << static_cast<int>(source)
                      << "（沿用柜子配置）；SDK 默认参数，tracking_error_limit=0.1rad\n"
                      << "FDCC 平移目标上限: " << options.fdcc_max_speed << "m/s, "
                      << options.fdcc_max_accel << "m/s2；入口跟随增益=" << options.fdcc_follow_gain
                      << "/s；虚拟目标倍数=" << options.fdcc_target_scale << '\n'
                      << "目标速度前馈=" << options.fdcc_target_feedforward
                      << "；实测姿态渐进减速，候选目标不按条件数 60 暂停\n"
                      << "转动目标上限: 0.8rad/s, 3rad/s2；50Hz RPC\n"
                      << "松开 Grip 退出 FDCC 持位，重新握紧继续；摇杆 J6 不参与控制。\n";
        }
    }
    if (!session_running->load()) { return 0; }
    auto enabled = robot.Enable();
    if (!enabled.IsSuccess())
    {
        std::cerr << "单臂使能失败: " << enabled.GetErrorMsg() << '\n';
        return 1;
    }
    HandServoState hand;
    const auto initial_state = robot.GetState();
    if (!SetTargetFromRobotState(initial_state, hand))
    {
        std::cerr << "单臂 TCP 状态无效\n";
        return 1;
    }
    std::unique_ptr<PicoCompliance> compliance;
    if (use_impedance)
    {
        compliance = std::make_unique<PicoImpedance>(robot, options.impedance_stiffness,
            options.impedance_damping, options.impedance_max_speed,
            options.impedance_max_accel, options.impedance_follow_gain);
    }
    else if (use_compliance)
    {
        compliance = std::make_unique<PicoFdcc>(robot, options.fdcc_max_speed,
            options.fdcc_max_accel, options.fdcc_follow_gain, options.fdcc_target_feedforward);
    }
    EndBoardDhGripper gripper(robot, "单臂夹爪");
    if (options.enable_gripper &&
        !gripper.Initialize(options.gripper_force, options.gripper_speed))
    {
        return 1;
    }
    SingularitySlowdownMonitor singularity_monitor(robot);
    singularity_monitor.Start();
    if (!singularity_monitor.WaitForInitialSample(std::chrono::seconds(1)))
    {
        std::cerr << "未取得有效 Jacobian\n";
        return 1;
    }
    PicoTargetGuard guard(robot,
        PicoTargetGuard::PolicyForControlMode(options.control_mode));
    RobotStateFreshnessWatchdog watchdog;
    (void)watchdog.Observe(initial_state, std::chrono::steady_clock::now());
    bool input_was_fresh = false;
    bool singularity_was_fresh = true;
    bool trigger_pressed = true; // Require release before first gripper toggle.
    bool gripper_closed = false;
    bool fault = false;
    uint64_t last_sequence = packet.sequence;
    auto next_tick = std::chrono::steady_clock::now();
    auto last_print = next_tick;
    auto last_compliance_phase = PicoCompliance::Phase::Holding;
    bool last_rearm_required = false;
    auto last_input_loop = std::chrono::steady_clock::now();
    while (session_running->load())
    {
        const auto input_loop_now = std::chrono::steady_clock::now();
        const double input_loop_gap_ms = std::chrono::duration<double, std::milli>(
            input_loop_now - last_input_loop).count();
        last_input_loop = input_loop_now;
        (void)TakePicoSample(pico, packet);
        ValidatePicoPacket(packet, options.single_arm, options.head_yaw_clutched);
        const auto state = robot.GetState();
        const auto singularity = singularity_monitor.Snapshot();
        const auto compliance_status = compliance ? compliance->Snapshot() : PicoCompliance::Status{};
        if (compliance && !compliance_status.error.empty())
        {
            std::cerr << mode_name << " 遥操停止: " << compliance_status.error << '\n';
            fault = true;
            break;
        }
        if (compliance && compliance_status.phase != last_compliance_phase)
        {
            if (compliance_status.phase == PicoCompliance::Phase::Active)
            {
                std::cout << mode_name << " active: mode=" << (use_impedance ? 5 : 6) << "，右手柄跟随\n";
            }
            else if (compliance_status.phase == PicoCompliance::Phase::Holding)
            {
                std::cout << mode_name << " paused: mode=0，持位等待重新握紧 Grip\n";
            }
            last_compliance_phase = compliance_status.phase;
        }
        if (state.errors.GetLatchedErrorCount() > 0 ||
            !watchdog.Observe(state, std::chrono::steady_clock::now()) ||
            state.reference_frame != reference.name)
        {
            std::cerr << "单臂状态过期、参考系改变或锁存错误，结束遥操\n";
            fault = true;
            break;
        }
        const bool input_connected = pico.IsConnected();
        const double input_age_ms = 1000.0 * SecondsSince(packet.timestamp_sec);
        const bool fresh = input_connected && packet.right.tracked &&
                           input_age_ms <= 1000.0 * kInputStaleSeconds;
        const auto print_input_health = [&] {
            std::cout << " connected=" << input_connected
                      << " tracked=" << packet.right.tracked
                      << " input_age_ms=" << input_age_ms
                      << " input_seq=" << packet.sequence
                      << " loop_gap_ms=" << input_loop_gap_ms;
        };
        if (compliance && compliance_status.rearm_required)
        {
            if (!last_rearm_required)
            {
                std::cout << mode_name << " pause reason: " << compliance_status.pause_reason
                          << "; release Grip before resuming";
                print_input_health();
                std::cout << '\n';
            }
            guard.Cancel();
            FreezeHand(state, hand, true);
            if (fresh && packet.right.grip <= kGripReleaseThreshold &&
                compliance->AcknowledgeGripRelease())
            {
                hand.require_release = false;
            }
        }
        last_rearm_required = compliance && compliance_status.rearm_required;
        if ((!fresh && input_was_fresh) ||
            singularity.fresh != singularity_was_fresh)
        {
            FreezeHand(state, hand, use_compliance);
            gripper.CancelPending();
        }
        if (!fresh) { trigger_pressed = true; }
        singularity_was_fresh = singularity.fresh;
        if (fresh && packet.sequence != last_sequence)
        {
            if (singularity.fresh && (!compliance || compliance_status.phase != PicoCompliance::Phase::Stopping))
            {
                auto right = packet.right;
                if (use_compliance)
                {
                    // No J6 ServoJ while a compliance mode owns the robot.
                    right.thumbstick = {0.0, 0.0};
                }
                UpdateHandTarget(right, packet.headset, state, reference,
                                 options.enable_rotation, options.head_yaw_clutched,
                                 options.translation_scale, options.rotation_scale,
                                 use_compliance ? 0.0 : stopper.LimitJ6Speed(options.j6_thumbstick_speed),
                                 options.operator_facing_robot, singularity, hand);
            }
            const bool pressed = TriggerIsPressed(packet.right.trigger, trigger_pressed);
            if (options.enable_gripper && pressed && !trigger_pressed)
            {
                if (gripper.Move(!gripper_closed)) { gripper_closed = !gripper_closed; }
            }
            trigger_pressed = pressed;
            last_sequence = packet.sequence;
        }
        input_was_fresh = fresh;
        bool ok = true;
        if (compliance && (hand.stop_requested || !fresh || !singularity.fresh ||
                     compliance_status.phase == PicoCompliance::Phase::Stopping))
        {
            if (compliance_status.phase == PicoCompliance::Phase::Active ||
                compliance_status.phase == PicoCompliance::Phase::Starting)
            {
                const char *reason = !input_connected ? "Pico WebSocket disconnected" :
                    !packet.right.tracked ? "Pico right-hand tracking lost" :
                    !fresh ? "Pico sample age exceeded 250ms" :
                    !singularity.fresh ? "Jacobian stale" : "Grip released / clutch stopped";
                std::cout << mode_name << " pause reason: " << reason;
                print_input_health();
                std::cout << '\n';
            }
            guard.Cancel();
            compliance->Pause();
            FreezeHand(state, hand, !fresh || !singularity.fresh);
            hand.stop_requested = false;
        }
        else if (!compliance && (hand.stop_requested || stopper.Pending()))
        {
            guard.Cancel();
            if (hand.stop_requested) { stopper.Request(robot, hand.stop_from_joint6); }
            FreezeHand(state, hand, false);
            hand.stop_requested = false;
            std::string stop_error;
            if (stopper.Poll(stop_error) && !stop_error.empty())
            {
                std::cerr << "单臂制动失败: " << stop_error << '\n';
                ok = false;
            }
        }
        else if (!compliance && hand.joint6_active)
        {
            guard.Cancel();
            ok = SendSdkServoPTick(robot, hand);
        }
        else if (!hand.active || !hand.servop_stream_started)
        {
            guard.Cancel();
            if (compliance)
            {
                compliance->Pause();
            }
            ok = guard.Snapshot().error.empty();
        }
        else
        {
            if (compliance)
            {
                // Receive timestamp belongs to the current packet. A slow
                // guard must not look like input loss, nor keep dead input alive.
                compliance->UpdateInputDeadline(PicoCompliance::Clock::time_point(
                    std::chrono::duration_cast<PicoCompliance::Clock::duration>(
                        std::chrono::duration<double>(packet.timestamp_sec + kInputStaleSeconds))));
                auto candidate = hand;
                Eigen::Isometry3d measured;
                if (!GetTcpInReference(state, measured))
                {
                    throw std::runtime_error("compliance target requires finite measured TCP");
                }
                // One candidate per input frame: repeated feedback cannot keep
                // stale hand input approved. Never overwrite the raw anchor.
                candidate.target_tcp_reference = smrcore::peripherals::detail::MakeFdccVirtualTarget(
                    hand.target_tcp_reference, measured, use_impedance ? 1.0 : options.fdcc_target_scale);
                guard.Submit(candidate, &hand.target_tcp_reference);
            }
            else
            {
                guard.Submit(hand);
            }
            const auto checked = guard.Snapshot();
            if (!checked.error.empty())
            {
                std::cerr << "单臂目标校验失败: " << checked.error << '\n';
                ok = false;
            }
            else if (checked.ready && checked.blocked_reason.empty() && !checked.pending)
            {
                HandServoState approved = hand;
                approved.target_tcp_reference = checked.target;
                approved.target_rpy = checked.rpy;
                if (compliance)
                {
                    const auto valid_until = PicoCompliance::Clock::time_point(
                        std::chrono::nanoseconds(checked.completed_ns)) + std::chrono::milliseconds(200);
                    compliance->Submit(approved.target_tcp_reference, valid_until, checked.raw_target);
                }
                else
                {
                    ok = SendSdkServoPTick(robot, approved);
                }
            }
            else if (compliance && !checked.blocked_reason.empty() &&
                             (compliance_status.phase == PicoCompliance::Phase::Active ||
                              compliance_status.phase == PicoCompliance::Phase::Starting))
            {
                // A rejected candidate requires release; a delayed check only
                // freezes the equilibrium in the worker and resumes on approval.
                std::cout << mode_name << " pause reason: " << (checked.blocked_reason.empty() ?
                    "candidate not ready" : checked.blocked_reason) << '\n';
                compliance->PauseUntilRelease(checked.blocked_reason.empty() ?
                    "candidate not ready" : checked.blocked_reason);
                guard.Cancel();
                FreezeHand(state, hand, true);
                hand.stop_requested = false;
            }
        }
        if (!ok) { fault = true; break; }
        for (size_t i = 0; i < 6; ++i)
        {
            diagnostics[6 + i].store(state.positions[i], std::memory_order_relaxed);
        }
        diagnostics[13].store(singularity.condition_number, std::memory_order_relaxed);
        diagnostics[15].store(hand.singularity_scale, std::memory_order_relaxed);
        if (SecondsSince(last_print) >= 1.0)
        {
            PrintArmDiagnostics("single/right-hand", packet.right, state, hand,
                                reference, singularity);
            if (compliance)
            {
                const auto checked = guard.Snapshot();
                std::cout << mode_name << " sent=" << compliance_status.sent
                          << (use_impedance ? " send_ms=" : " rpc_ms=")
                          << compliance_status.call_ms << " guard_age_ms=" << compliance_status.guard_age_ms
                          << " raw_err_mm=" << compliance_status.raw_error_mm
                          << " lead_mm=" << compliance_status.lead_mm
                          << " shape_gap_mm=" << compliance_status.shape_gap_mm
                          << " follow_err_mm=" << compliance_status.follow_error_mm
                          << " v_target=" << compliance_status.target_speed_mps << "m/s"
                          << " shape_deg=" << compliance_status.shape_gap_deg
                          << " follow_deg=" << compliance_status.follow_error_deg
                          << " w_target=" << compliance_status.target_angular_speed_rps << "rad/s"
                          << " check=" << (compliance_status.waiting_for_check ? "WAIT" : "OK")
                          << " ik_ms=" << checked.ik_ms << " fk_ms=" << checked.fk_ms
                          << " check_queue_ms=" << (checked.trace_check_start > 0.0 ?
                              1000.0 * (checked.trace_check_start - checked.requested_ns / 1e9) : 0.0)
                          << '\n';
            }
            last_print = std::chrono::steady_clock::now();
        }
        next_tick += kServoPeriod;
        const auto now = std::chrono::steady_clock::now();
        if (next_tick < now - kServoPeriod) { next_tick = now; }
        std::this_thread::sleep_until(next_tick);
    }
    if (compliance)
    {
        compliance->Stop();
        const auto final_status = compliance->Snapshot();
        if (!final_status.error.empty())
        {
            std::cerr << mode_name << " 退出: " << final_status.error << '\n';
            fault = true;
        }
    }
    if (fault) { robot.Motion().StopMotion(); }
    stopper.Wait();
    guard.Stop();
    singularity_monitor.Stop();
    gripper.Stop();
    if (compliance)
    {
        // Mode exit has reset the planner to measured joints. Wait for
        // standstill without sending ServoP/ServoJ into this control path.
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!fault && !JointsNearStill(robot.GetState()) &&
               std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        auto result = robot.Disable();
        const auto motors = robot.Control().GetMotorStatus();
        PrintDisableStatus(use_impedance ? "single Impedance" : "single FDCC", result, motors);
        if (!result.IsSuccess() || motors.enabled)
        {
            fault = true;
            std::cerr << mode_name << " 退出下使能未确认，请在控制柜确认停止状态\n";
        }
    }
    else if (fault)
    {
        robot.Disable();
    }
    else
    {
        HoldThenDisable(robot, "single");
    }
    cleanup.armed = false;
    robot.Shutdown();
    pico.Stop();
    return fault ? 1 : 0;
}

int RunSession(const Options &options,
               std::array<std::atomic<double>, 16> &diagnostics)
{
    const Options defaults;
    if (!smrcore::peripherals::detail::ValidTargetFeedforward(options.fdcc_target_feedforward) ||
        !smrcore::peripherals::detail::ValidFdccShaping(options.fdcc_max_speed,
            options.fdcc_max_accel, options.fdcc_follow_gain, options.fdcc_target_scale) ||
        (options.control_mode != PicoControlMode::Fdcc &&
         (options.fdcc_max_speed != defaults.fdcc_max_speed ||
          options.fdcc_max_accel != defaults.fdcc_max_accel ||
          options.fdcc_follow_gain != defaults.fdcc_follow_gain ||
          options.fdcc_target_scale != defaults.fdcc_target_scale ||
          options.fdcc_target_feedforward != defaults.fdcc_target_feedforward)))
    {
        std::cerr << "FDCC 参数仅用于 --control-mode fdcc：speed [0.01,0.5]m/s, "
                     "accel [0.05,2]m/s2, follow_gain [1,40]/s, target_scale [1,3], "
                     "target_feedforward [0,1]\n";
        return 2;
    }
    if (!smrcore::peripherals::detail::ValidImpedanceGains(options.impedance_stiffness,
            options.impedance_damping) ||
        !smrcore::peripherals::detail::ValidFdccShaping(options.impedance_max_speed,
            options.impedance_max_accel, options.impedance_follow_gain, 1.0) ||
        (options.control_mode != PicoControlMode::CartesianImpedance &&
         (options.impedance_stiffness != defaults.impedance_stiffness ||
          options.impedance_damping != defaults.impedance_damping ||
          options.impedance_max_speed != defaults.impedance_max_speed ||
          options.impedance_max_accel != defaults.impedance_max_accel ||
          options.impedance_follow_gain != defaults.impedance_follow_gain)))
    {
        std::cerr << "阻抗参数仅用于 --control-mode impedance；K/D 与速度参数须在帮助所示范围内\n";
        return 2;
    }
    if ((options.control_mode != PicoControlMode::ServoP &&
         options.control_mode != PicoControlMode::Fdcc &&
         options.control_mode != PicoControlMode::CartesianImpedance) ||
        (options.control_mode != PicoControlMode::ServoP && !options.single_arm))
    {
        std::cerr << "--control-mode fdcc/impedance 目前仅支持 --single-arm 独立单臂\n";
        return 2;
    }
    if (!std::isfinite(options.single_mapping_yaw_deg) ||
        std::abs(options.single_mapping_yaw_deg) > 180.0 ||
        (!options.single_arm && options.single_mapping_yaw_deg != 0.0))
    {
        std::cerr << "--single-mapping-yaw 仅用于单臂，必须是 [-180, 180] 内的有限角度\n";
        return 2;
    }
    if (!options.enable_motion || options.robot_ip.empty() ||
        options.port == 0 || !std::isfinite(options.translation_scale) ||
        options.translation_scale < 0.1 || options.translation_scale > 3.0 ||
        !std::isfinite(options.rotation_scale) ||
        options.rotation_scale < 0.1 || options.rotation_scale > 3.0 ||
        options.gripper_force < 20 || options.gripper_force > 100 ||
        options.gripper_speed == 0 || options.gripper_speed > 100)
    {
        return 2;
    }
    if (!session_running->load())
    {
        return 0;
    }
    if (!ValidateHeadYawMapping())
    {
        std::cerr << "PICO 头显 yaw 坐标映射自检失败" << std::endl;
        return 1;
    }

    if (options.single_arm)
    {
        return RunSingleRightSession(options, diagnostics);
    }

    Eigen::Isometry3d world_from_left_base = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d world_from_right_base = Eigen::Isometry3d::Identity();
    LoadEmbeddedMountTransforms(world_from_left_base, world_from_right_base);

    Pico pico;
    PicoOptions pico_options;
    pico_options.port = options.port;
    if (!pico.Initialize(pico_options) || !pico.Start())
    {
        std::cerr << "PICO WebXR 服务启动失败: " << pico.GetLastError()
                  << std::endl;
        return 1;
    }

    std::cout << "\n========================================\n"
              << "  Peripheral PICO SDK Bridge\n"
              << "========================================\n"
              << "PC 服务: http://127.0.0.1:" << options.port << "/\n"
              << "USB 映射: adb reverse tcp:" << options.port
              << " tcp:" << options.port << "\n"
              << "模式: SDK ServoP"
              << "\n映射: 平移 " << options.translation_scale << "x，旋转 "
              << (options.enable_rotation
                      ? std::to_string(options.rotation_scale) + "x"
                      : "关闭")
              << "\n方向基准: "
              << (options.head_yaw_clutched ? "Grip 锁定头显 yaw"
                                            : "WebXR room-fixed")
              << "\nJ6 推杆: 已启用；最大速度 " << options.j6_thumbstick_speed
              << " rad/s，死区 " << kJ6ThumbstickDeadzone << "\n操作者坐标系: "
              << (options.operator_facing_robot ? "face-to-face" : "same-side")
              << std::endl;
    if (options.operator_facing_robot)
    {
        std::cout << "face-to-face 手柄对调: 物理右手柄→左臂，物理左手柄→右臂。"
                  << std::endl;
    }
    std::cout << "必须按住 Grip≥" << std::fixed << std::setprecision(2)
              << kGripPressThreshold << " 才会发 ServoP；松开到 ≤"
              << kGripReleaseThreshold
              << " 冻结。Trigger 按下沿切换 DH 夹爪开/关。"
                 "终端 clutch=OFF 时手柄移动不会带动机械臂。"
              << std::endl;

    PicoSample first_packet;
    if (!WaitForControllers(pico, first_packet))
    {
        pico.Stop();
        return 0;
    }
    ValidatePicoPacket(first_packet, false, options.head_yaw_clutched);
    if (!WaitForReleasedGrips(pico))
    {
        pico.Stop();
        return 0;
    }

    rcore::sdk::DualArmRobot arms;
    if (!arms.Initialize(options.robot_ip))
    {
        std::cerr << "DualArmRobot 初始化失败" << std::endl;
        pico.Stop();
        return 1;
    }

    std::string command_source_error;
    if (!EnsureSdkCommandSource(arms.left_robot(), "L", command_source_error) ||
        !EnsureSdkCommandSource(arms.right_robot(), "R", command_source_error))
    {
        std::cerr << command_source_error << std::endl;
        arms.Shutdown();
        pico.Stop();
        return 1;
    }

    ArmReference left_reference;
    ArmReference right_reference;
    std::string reference_error;
    if (!ResolveDualReferences(arms, left_reference, right_reference,
                               world_from_left_base, world_from_right_base,
                               reference_error))
    {
        std::cerr << "世界系解析失败: " << reference_error << std::endl;
        arms.Shutdown();
        pico.Stop();
        return 1;
    }
    const auto describe_world_ref = [](const ArmReference &ref)
    {
        const Eigen::Vector3d t = ref.world_from_reference.translation();
        const double rot_from_i =
            (ref.world_from_reference.linear() - Eigen::Matrix3d::Identity())
                .norm();
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3) << "ref=\"" << ref.name
            << "\" W.t=[" << t[0] << "," << t[1] << "," << t[2]
            << "] |R-I|=" << rot_from_i;
        return oss.str();
    };
    std::cout << "遥操方向: smorebot world 映射，ServoP 使用各臂 base 参考系。"
              << std::endl;
    std::cout << "世界系 L: " << describe_world_ref(left_reference)
              << std::endl;
    std::cout << "世界系 R: " << describe_world_ref(right_reference)
              << std::endl;
    std::cout << "内置 B3 支架变换（仅适用于对应 B3 双臂安装构型）" << std::endl;

    struct MotionCleanup
    {
        rcore::sdk::DualArmRobot &arms;
        bool armed = true;
        ~MotionCleanup()
        {
            if (!armed)
                return;
            for (auto *robot : {&arms.left_robot(), &arms.right_robot()})
            {
                try
                {
                    robot->Motion().StopMotion();
                }
                catch (...)
                {
                }
                try
                {
                    robot->Disable();
                }
                catch (...)
                {
                }
            }
        }
    } cleanup{arms};
    if (!session_running->load())
    {
        cleanup.armed = false;
        return 0;
    }
    SelectedPicoStopper left_stopper, right_stopper;
    if (!left_stopper.Configure(arms.left_robot()) ||
        !right_stopper.Configure(arms.right_robot()))
    {
        std::cerr << "无法读取有效的关节制动限制，拒绝启动 PICO 遥操。\n";
        return 1;
    }
    auto enable_result = arms.Enable();
    if (!enable_result.Ok())
    {
        (void)arms.left_robot().Disable();
        (void)arms.right_robot().Disable();
        cleanup.armed = false;
        std::cerr << "双臂使能失败: L(" << enable_result.left.GetErrorCode()
                  << ") " << enable_result.left.GetErrorMsg() << " R("
                  << enable_result.right.GetErrorCode() << ") "
                  << enable_result.right.GetErrorMsg() << std::endl;
        arms.Shutdown();
        pico.Stop();
        return 1;
    }

    HandServoState left_hand;
    HandServoState right_hand;
    const auto initial_left_state = arms.left_robot().GetState();
    const auto initial_right_state = arms.right_robot().GetState();
    if (!SetTargetFromRobotState(initial_left_state, left_hand) ||
        !SetTargetFromRobotState(initial_right_state, right_hand))
    {
        std::cerr << "当前 TCP 状态无效，拒绝启动 PICO 遥操作。" << std::endl;
        (void)arms.Disable();
        cleanup.armed = false;
        arms.Shutdown();
        pico.Stop();
        return 1;
    }
    EndBoardDhGripper left_gripper(arms.left_robot(), "左臂夹爪");
    EndBoardDhGripper right_gripper(arms.right_robot(), "右臂夹爪");
    if (options.enable_gripper && (!left_gripper.Initialize(options.gripper_force,
                                 options.gripper_speed) ||
        !right_gripper.Initialize(options.gripper_force, options.gripper_speed)))
    {
        left_gripper.Stop();
        right_gripper.Stop();
        (void)arms.Disable();
        cleanup.armed = false;
        arms.Shutdown();
        pico.Stop();
        return 1;
    }

    SingularitySlowdownMonitor left_singularity_monitor(arms.left_robot());
    SingularitySlowdownMonitor right_singularity_monitor(arms.right_robot());
    left_singularity_monitor.Start();
    right_singularity_monitor.Start();
    if (!left_singularity_monitor.WaitForInitialSample(
            std::chrono::seconds(1)) ||
        !right_singularity_monitor.WaitForInitialSample(
            std::chrono::seconds(1)))
    {
        std::cerr << "1s 内未取得双臂 Jacobian，拒绝启动 PICO ServoP。"
                  << std::endl;
        left_singularity_monitor.Stop();
        right_singularity_monitor.Stop();
        left_gripper.Stop();
        right_gripper.Stop();
        (void)arms.Disable();
        cleanup.armed = false;
        arms.Shutdown();
        pico.Stop();
        return 1;
    }

    std::cout
        << "PICO guarded ServoP: condition 30..60 slowdown; "
           "candidate IK/FK preflight; J6 thumbstick enabled; faults stop "
           "both arms."
        << std::endl;

    RobotStateFreshnessWatchdog left_state_watchdog;
    RobotStateFreshnessWatchdog right_state_watchdog;
    const auto initial_observed_at = std::chrono::steady_clock::now();
    (void)left_state_watchdog.Observe(initial_left_state, initial_observed_at);
    (void)right_state_watchdog.Observe(initial_right_state,
                                       initial_observed_at);

    auto next_servo_tick = std::chrono::steady_clock::now();
    auto last_print = next_servo_tick;
    uint64_t last_sequence = first_packet.sequence;
    bool input_was_fresh = true;
    bool robot_state_was_fresh = true;
    bool left_singularity_was_fresh = true;
    bool right_singularity_was_fresh = true;
    bool safety_fault_detected = false;
    bool pico_guard_fault = false;
    std::cout << "PICO GUARD: condition soft=30 hard=60, target preflight, "
                 "elbow/wrist clearance>=10deg; IK continuity by backend "
                 "ServoP; J6 thumbstick enabled; fault requires restart\n";
    PicoTargetGuard left_guard(arms.left_robot());
    PicoTargetGuard right_guard(arms.right_robot());
    PicoLatencyTrace latency_trace;
    std::array<double, 2> trace_feedback_stamp{}, trace_feedback_seen{};
    PicoSample packet = first_packet;
    const PicoControllerPose &initial_left_controller =
        options.operator_facing_robot ? first_packet.right : first_packet.left;
    const PicoControllerPose &initial_right_controller =
        options.operator_facing_robot ? first_packet.left : first_packet.right;
    bool left_trigger_pressed =
        TriggerIsPressed(initial_left_controller.trigger, false);
    bool right_trigger_pressed =
        TriggerIsPressed(initial_right_controller.trigger, false);
    bool left_gripper_closed = false;
    bool right_gripper_closed = false;

    while (session_running->load(std::memory_order_acquire))
    {
        (void)TakePicoSample(pico, packet);
        ValidatePicoPacket(packet, options.single_arm, options.head_yaw_clutched);
        const double trace_taken = SteadyNowNanoseconds() / 1e9;
        const bool fresh =
            pico.IsConnected() &&
            SecondsSince(packet.timestamp_sec) <= kInputStaleSeconds;
        const double trace_left_read_start = SteadyNowNanoseconds() / 1e9;
        const auto left_state = arms.left_robot().GetState();
        const double trace_left_read_end = SteadyNowNanoseconds() / 1e9;
        const double trace_right_read_start = SteadyNowNanoseconds() / 1e9;
        const auto right_state = arms.right_robot().GetState();
        const double trace_right_read_end = SteadyNowNanoseconds() / 1e9;
        const double stamps[2] = {left_state.timestamp, right_state.timestamp};
        const double read_ends[2] = {trace_left_read_end, trace_right_read_end};
        for (int i = 0; i < 2; ++i)
            if (stamps[i] != trace_feedback_stamp[i])
            {
                trace_feedback_stamp[i] = stamps[i];
                trace_feedback_seen[i] = read_ends[i];
            }
        const SingularitySlowdownStatus left_singularity =
            left_singularity_monitor.Snapshot();
        const SingularitySlowdownStatus right_singularity =
            right_singularity_monitor.Snapshot();

        if (left_state.errors.GetLatchedErrorCount() > 0 ||
            right_state.errors.GetLatchedErrorCount() > 0)
        {
            safety_fault_detected = true;
            std::cerr << "\n检测到控制器锁存安全错误，停止双臂命令：";
            PrintLatchedErrors("L", left_state);
            PrintLatchedErrors("R", right_state);
            std::cerr << std::endl;
            break;
        }

        // WebXR and Jacobian streams may transiently hitch. Their freshness
        // transitions below freeze and brake the affected arm, then re-clutch
        // from measured state after recovery without terminating the session.
        const auto observed_at = std::chrono::steady_clock::now();
        const bool states_fresh =
            left_state_watchdog.Observe(left_state, observed_at) &&
            right_state_watchdog.Observe(right_state, observed_at);
        if (!states_fresh)
        {
            if (robot_state_was_fresh)
            {
                FreezeHand(left_state, left_hand, false);
                FreezeHand(right_state, right_hand, false);
                std::cerr << "\nRobotState 超过 "
                          << static_cast<int>(kRobotStateStaleSeconds * 1000.0)
                          << "ms 未刷新，停止双臂命令；状态恢复后按住 Grip "
                             "会从当前 TCP 重新离合。"
                          << std::endl;
            }
            pico_guard_fault = true;
            break;
        }
        if (!robot_state_was_fresh)
        {
            FreezeHand(left_state, left_hand, false);
            FreezeHand(right_state, right_hand, false);
            std::cout
                << "\nRobotState 已恢复，按住 Grip 将从当前 TCP 重新离合。"
                << std::endl;
        }
        robot_state_was_fresh = true;

        const auto update_singularity_freshness =
            [&](const char *arm, const SingularitySlowdownStatus &status,
                bool &was_fresh, const rcore::RobotState &robot_state,
                HandServoState &hand)
        {
            if (!status.fresh && was_fresh)
            {
                FreezeHand(robot_state, hand, false);
                std::cerr << "\n"
                          << arm << " Jacobian 条件数超过 "
                          << static_cast<int>(kSingularityMonitorStaleSeconds *
                                              1000.0)
                          << "ms 未刷新，已冻结该臂 ServoP。" << std::endl;
            }
            else if (status.fresh && !was_fresh)
            {
                FreezeHand(robot_state, hand, false);
                std::cout << "\n"
                          << arm
                          << " Jacobian 条件数监测已恢复，Grip 将从当前 TCP "
                             "重新离合。"
                          << std::endl;
            }
            was_fresh = status.fresh;
        };
        update_singularity_freshness("L", left_singularity,
                                     left_singularity_was_fresh, left_state,
                                     left_hand);
        update_singularity_freshness("R", right_singularity,
                                     right_singularity_was_fresh, right_state,
                                     right_hand);

        if (!fresh && input_was_fresh)
        {
            left_gripper.CancelPending();
            right_gripper.CancelPending();
            FreezeHand(left_state, left_hand, false);
            FreezeHand(right_state, right_hand, false);
            std::cerr << "\nPICO 输入超过 "
                      << static_cast<int>(kInputStaleSeconds * 1000.0)
                      << "ms 未刷新，双臂已冻结；恢复后按住 Grip 会重新离合。"
                      << std::endl;
        }
        if (fresh && packet.sequence != last_sequence)
        {
            const PicoControllerPose &left_controller =
                options.operator_facing_robot ? packet.right : packet.left;
            const PicoControllerPose &right_controller =
                options.operator_facing_robot ? packet.left : packet.right;
            if (left_singularity.fresh)
            {
                UpdateHandTarget(
                    left_controller, packet.headset, left_state, left_reference,
                    options.enable_rotation, options.head_yaw_clutched,
                    options.translation_scale, options.rotation_scale,
                    left_stopper.LimitJ6Speed(options.j6_thumbstick_speed),
                    options.operator_facing_robot,
                    left_singularity, left_hand);
                left_hand.trace_mapped = SteadyNowNanoseconds() / 1e9;
            }
            if (right_singularity.fresh)
            {
                UpdateHandTarget(
                    right_controller, packet.headset, right_state,
                    right_reference, options.enable_rotation,
                    options.head_yaw_clutched, options.translation_scale,
                    options.rotation_scale,
                    right_stopper.LimitJ6Speed(options.j6_thumbstick_speed),
                    options.operator_facing_robot, right_singularity,
                    right_hand);
                right_hand.trace_mapped = SteadyNowNanoseconds() / 1e9;
            }
            left_hand.trace_taken = right_hand.trace_taken = trace_taken;
            left_hand.trace_sequence = right_hand.trace_sequence = packet.sequence;
            left_hand.trace_received = right_hand.trace_received = packet.timestamp_sec;
            const bool next_left_trigger =
                TriggerIsPressed(left_controller.trigger, left_trigger_pressed);
            const bool next_right_trigger = TriggerIsPressed(
                right_controller.trigger, right_trigger_pressed);
            if (options.enable_gripper && next_left_trigger && !left_trigger_pressed)
            {
                const bool next_closed = !left_gripper_closed;
                if (left_gripper.Move(next_closed))
                {
                    left_gripper_closed = next_closed;
                }
            }
            if (options.enable_gripper && next_right_trigger && !right_trigger_pressed)
            {
                const bool next_closed = !right_gripper_closed;
                if (right_gripper.Move(next_closed))
                {
                    right_gripper_closed = next_closed;
                }
            }
            left_trigger_pressed = next_left_trigger;
            right_trigger_pressed = next_right_trigger;
            last_sequence = packet.sequence;
        }
        input_was_fresh = fresh;

        // Do not call GetBasePose/GetControllerConfig RPC on the 1kHz ServoP
        // loop. A hung or slow RPC stalls the stream and freezes both arms.
        // Changing frames invalidates every pending candidate; terminate the
        // session.
        if (left_state.reference_frame != left_reference.name ||
            right_state.reference_frame != right_reference.name)
        {
            pico_guard_fault = true;
            std::cerr
                << "PICO GUARD: reference frame changed; restart required\n";
            break;
        }

        const auto checked_tick =
            [&](const char *name, rcore::sdk::Robot &robot,
                HandServoState &hand, PicoTargetGuard &guard,
                SelectedPicoStopper &stopper, const rcore::RobotState &robot_state)
        {
            if (hand.stop_requested || stopper.Pending() || hand.joint6_active ||
                !hand.active || !hand.servop_stream_started)
                latency_trace.Record(name[0] == 'L' ? 0 : 1, hand.trace_sequence,
                    hand.trace_received, hand.target_tcp_reference, hand.target_rpy,
                    robot_state, false);
            if (hand.stop_requested || stopper.Pending())
            {
                guard.Cancel();
                if (hand.stop_requested)
                    stopper.Request(robot, hand.stop_from_joint6);
                // 关节减速和保持期间禁止跟随；停稳后从新反馈重新离合。
                FreezeHand(robot_state, hand, false);
                hand.stop_requested = false;
                std::string error;
                if (stopper.Poll(error) && !error.empty())
                {
                    std::cerr << name << " PICO stop failed: " << error << "\n";
                    return false;
                }
                return true;
            }
            if (hand.joint6_active)
            {
                guard.Cancel();
                return SendSdkServoPTick(robot, hand);
            }
            if (!hand.active || !hand.servop_stream_started)
            {
                guard.Cancel();
                return guard.Snapshot().error.empty();
            }
            guard.Submit(hand);
            const auto checked = guard.Snapshot();
            if (!checked.error.empty())
            {
                std::cerr << name << " PICO GUARD: " << checked.error << "\n";
                return false;
            }
            if (!checked.blocked_reason.empty())
            {
                return true; // Hold the last approved target; keep session.
            }
            if (!checked.ready)
                return true; // Never send an unchecked target.
            HandServoState approved = hand;
            approved.target_tcp_reference = checked.target;
            approved.target_rpy = checked.rpy;
            const double trace_send_start = SteadyNowNanoseconds() / 1e9;
            const bool sent = SendSdkServoPTick(robot, approved);
            const double trace_send_end = SteadyNowNanoseconds() / 1e9;
            latency_trace.Record(name[0] == 'L' ? 0 : 1, checked.trace_sequence,
                checked.trace_received, checked.target, checked.rpy, robot_state, sent,
                {checked.trace_taken, checked.trace_mapped, checked.requested_ns / 1e9,
                 checked.trace_check_start, checked.trace_check_end, 0, 0,
                 trace_send_start, trace_send_end,
                 name[0] == 'L' ? trace_left_read_start : trace_right_read_start,
                 name[0] == 'L' ? trace_left_read_end : trace_right_read_end,
                 trace_feedback_seen[name[0] == 'L' ? 0 : 1]});
            return sent;
        };
        if (!checked_tick("L", arms.left_robot(), left_hand, left_guard,
                          left_stopper, left_state) ||
            !checked_tick("R", arms.right_robot(), right_hand, right_guard,
                          right_stopper, right_state))
        {
            pico_guard_fault = true;
            break;
        }

        for (size_t i = 0; i < 6; ++i)
        {
            diagnostics[i].store(left_state.positions[i],
                                 std::memory_order_relaxed);
            diagnostics[6 + i].store(right_state.positions[i],
                                     std::memory_order_relaxed);
        }
        diagnostics[12].store(left_singularity.condition_number,
                              std::memory_order_relaxed);
        diagnostics[13].store(right_singularity.condition_number,
                              std::memory_order_relaxed);
        diagnostics[14].store(left_hand.singularity_scale,
                              std::memory_order_relaxed);
        diagnostics[15].store(right_hand.singularity_scale,
                              std::memory_order_relaxed);

        if (SecondsSince(last_print) >= 1.0)
        {
            const PicoControllerPose &left_controller =
                options.operator_facing_robot ? packet.right : packet.left;
            const PicoControllerPose &right_controller =
                options.operator_facing_robot ? packet.left : packet.right;
            std::cout << "XR age="
                      << (fresh ? SecondsSince(packet.timestamp_sec) * 1000.0
                                : -1.0)
                      << "ms ref=" << packet.reference_space << std::endl;
            PrintArmDiagnostics("L", left_controller, left_state, left_hand,
                                left_reference, left_singularity);
            PrintArmDiagnostics("R", right_controller, right_state, right_hand,
                                right_reference, right_singularity);
            last_print = std::chrono::steady_clock::now();
        }

        next_servo_tick += kServoPeriod;
        const auto now = std::chrono::steady_clock::now();
        if (next_servo_tick < now - kServoPeriod)
        {
            next_servo_tick = now;
        }
        std::this_thread::sleep_until(next_servo_tick);
    }

    if (pico_guard_fault)
    {
        // No Cartesian or joint hold targets on this exit path.
        safety_fault_detected = true;
        const auto stop_arm = [](rcore::sdk::Robot &robot)
        {
            try
            {
                return robot.Motion().StopMotion().IsSuccess();
            }
            catch (...)
            {
                return false;
            }
        };
        auto left_stop = std::async(std::launch::async, [&]
                                    { return stop_arm(arms.left_robot()); });
        auto right_stop = std::async(std::launch::async, [&]
                                     { return stop_arm(arms.right_robot()); });
        std::cerr << "PICO GUARD latched; restart required. StopMotion L="
                  << left_stop.get() << " R=" << right_stop.get() << std::endl;
    }
    left_stopper.Wait();
    right_stopper.Wait();
    left_guard.Stop();
    right_guard.Stop();
    left_singularity_monitor.Stop();
    right_singularity_monitor.Stop();
    left_gripper.Stop();
    right_gripper.Stop();
    std::cout << "\n正在结束遥操并分别下使能..." << std::endl;
    try
    {
        if (!safety_fault_detected)
        {
            HoldThenDisable(arms.left_robot(), "L");
            HoldThenDisable(arms.right_robot(), "R");
        }
        else
        {
            auto left_disable = arms.left_robot().Disable();
            auto right_disable = arms.right_robot().Disable();
            PrintDisableStatus("L", left_disable,
                               arms.left_robot().Control().GetMotorStatus());
            PrintDisableStatus("R", right_disable,
                               arms.right_robot().Control().GetMotorStatus());
        }
    }
    catch (const std::exception &exception)
    {
        std::cerr << "双臂后端已离线，Disable 未完成: " << exception.what()
                  << std::endl;
    }
    cleanup.armed = false;
    try
    {
        arms.Shutdown();
    }
    catch (const std::exception &exception)
    {
        std::cerr << "双臂 Shutdown 清理异常: " << exception.what()
                  << std::endl;
    }
    pico.Stop();
    std::cout << "PICO 双臂遥操作已结束。" << std::endl;
    return safety_fault_detected ? 1 : 0;
}
} // namespace

int smrcore::peripherals::PicoTeleop::Run(const PicoTeleopOptions &options)
{
    if (started_.exchange(true))
        return 2;
    std::unique_lock<std::mutex> lock(session_mutex, std::try_to_lock);
    if (!lock.owns_lock())
    {
        exit_code_.store(2);
        return 2;
    }
    active_.store(true);
    session_running = &running_;
    int result = 1;
    try
    {
        result = RunSession(options, diagnostics_);
    }
    catch (const std::exception &e)
    {
        std::cerr << "PICO session exception: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "PICO session exception" << std::endl;
    }
    session_running = nullptr;
    exit_code_.store(result);
    active_.store(false);
    return result;
}

smrcore::peripherals::PicoTeleopStatus
smrcore::peripherals::PicoTeleop::GetStatus() const noexcept
{
    PicoTeleopStatus s;
    s.running = IsRunning();
    s.exit_code = GetExitCode();
    for (size_t i = 0; i < 6; ++i)
    {
        s.left_positions[i] = diagnostics_[i].load();
        s.right_positions[i] = diagnostics_[6 + i].load();
    }
    s.left_condition = diagnostics_[12].load();
    s.right_condition = diagnostics_[13].load();
    s.left_scale = diagnostics_[14].load();
    s.right_scale = diagnostics_[15].load();
    return s;
}
