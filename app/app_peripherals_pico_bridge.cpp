#include "peripherals/pico/teleop.hpp"
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <string>
namespace
{
using Options = smrcore::peripherals::PicoTeleopOptions;
constexpr double kMinimumMappingScale = 0.1;
constexpr double kMaximumMappingScale = 3.0;
smrcore::peripherals::PicoTeleop session;
void SignalHandler(int) { session.RequestStop(); }
void PrintUsage(const char *program)
{
    std::cout << "用法: " << program
              << " [--enable-motion] [--single-arm] [--no-gripper] [--translation-only] [--room-fixed]"
                 " [--operator-frame same-side|face-to-face] [--port 8765]"
                 " [--single-mapping-yaw DEG]"
                 " [--control-mode servop|fdcc|impedance]"
                 " [--motion-scale 1.0] [--translation-scale 0.8]"
                 " [--rotation-scale 1.0] [--j6-speed 0.8]"
                 " [--gripper-force 80] [--gripper-speed 100]"
                 " [--robot-ip ADDRESS]\n"
              << "--enable-motion 是必需的运动确认；默认 SDK ServoP。\n"
                 "--control-mode fdcc：单臂 FDCC；松开 Grip 退出力控持位，重新握紧继续；禁用摇杆 J6。\n"
                 "--control-mode impedance：单臂笛卡尔阻抗，CST 力矩控制，200Hz 平衡点更新；松手退出阻抗持位。\n"
              << "--single-arm：独立单臂，只用物理右手柄，base 坐标系；\n"
                 "--single-mapping-yaw：单臂映射绕 base Z 轴偏转，单位度；从 +Z 看向原点，-90 为顺时针 90 度。\n"
                 "--no-gripper：跳过 DH 夹爪初始化与 Trigger 控制。\n"
                 "FDCC 使用内置跟随配置，日常启动无需填写控制器调试参数。\n"
                 "--help-advanced：查看工程调试参数及默认值。\n";
}

void PrintAdvancedUsage(const char *program)
{
    PrintUsage(program);
    const Options defaults;
    std::cout << "\n工程调试参数（旧命令继续兼容）：\n"
              << "--fdcc-max-speed：平移目标速度 [0.01,0.5] m/s，默认 " << defaults.fdcc_max_speed << "。\n"
              << "--fdcc-max-accel：平移目标加速度 [0.05,2] m/s2，默认 " << defaults.fdcc_max_accel << "。\n"
              << "--fdcc-follow-gain：入口平移跟随增益 [1,40] /s，默认 " << defaults.fdcc_follow_gain << "，不是控制器 kp/kd。\n"
              << "--fdcc-target-feedforward：目标速度前馈比例 [0,1]，默认 " << defaults.fdcc_target_feedforward << "；0 使用原平滑器。\n"
              << "--fdcc-target-scale：虚拟目标倍数 [1,3]，默认 " << defaults.fdcc_target_scale << " 关闭；额外前推最多 30mm。\n"
                 "--impedance-stiffness：六轴 K，默认 120,120,120,20,20,20；XYZ [0,2000] N/m、转动 [0,150] N·m/rad。\n"
                 "--impedance-damping：六轴 D，默认 20,20,20,1,1,1；XYZ [0,200] N·s/m、转动 [0,60] N·m·s/rad。\n"
                 "--impedance-max-speed / --impedance-max-accel / --impedance-follow-gain：默认 0.15 / 0.6 / 8，范围与 FDCC 同名参数相同。\n";
}

bool ParseMappingYaw(const std::string &text, double &yaw)
{
    try
    {
        size_t parsed = 0;
        const double value = std::stod(text, &parsed);
        if (parsed != text.size() || !std::isfinite(value) ||
            value < -180.0 || value > 180.0)
        {
            return false;
        }
        yaw = value;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool ParseOperatorFrame(const std::string &text, bool &operator_facing_robot)
{
    if (text == "same-side")
    {
        operator_facing_robot = false;
        return true;
    }
    if (text == "face-to-face")
    {
        operator_facing_robot = true;
        return true;
    }
    return false;
}

bool ParsePort(const std::string &text, uint16_t &port)
{
    try
    {
        const unsigned long value = std::stoul(text);
        if (value == 0 || value > 65535)
        {
            return false;
        }
        port = static_cast<uint16_t>(value);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool ParseMappingScale(const std::string &text, double &scale)
{
    try
    {
        size_t parsed = 0;
        const double value = std::stod(text, &parsed);
        if (parsed != text.size() || !std::isfinite(value) ||
            value < kMinimumMappingScale || value > kMaximumMappingScale)
        {
            return false;
        }
        scale = value;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool ParsePercent(const std::string &text, uint16_t minimum, uint16_t &value)
{
    try
    {
        size_t parsed = 0;
        const unsigned long parsed_value = std::stoul(text, &parsed);
        if (parsed != text.size() || parsed_value < minimum ||
            parsed_value > 100)
        {
            return false;
        }
        value = static_cast<uint16_t>(parsed_value);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool ParseFiniteNumber(const std::string &text, double &value)
{
    try
    {
        size_t parsed = 0;
        const double number = std::stod(text, &parsed);
        if (parsed != text.size() || !std::isfinite(number))
        {
            return false;
        }
        value = number;
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool ParseSixNumbers(const std::string &text, std::array<double, 6> &values)
{
    std::array<double, 6> parsed{};
    size_t start = 0;
    for (size_t i = 0; i < parsed.size(); ++i)
    {
        const size_t end = text.find(',', start);
        if ((i < 5 && end == std::string::npos) ||
            (i == 5 && end != std::string::npos) ||
            !ParseFiniteNumber(text.substr(start, end == std::string::npos ? end : end - start), parsed[i]))
        {
            return false;
        }
        start = end == std::string::npos ? text.size() : end + 1;
    }
    values = parsed;
    return true;
}

bool ParseOptions(int argc, char **argv, Options &options)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string argument = argv[i];
        if (argument == "--enable-motion")
        {
            options.enable_motion = true;
        }
        else if (argument == "--single-arm")
        {
            options.single_arm = true;
        }
        else if (argument == "--no-gripper")
        {
            options.enable_gripper = false;
        }
        else if (argument == "--translation-only")
        {
            options.enable_rotation = false;
        }
        else if (argument == "--room-fixed")
        {
            options.head_yaw_clutched = false;
        }
        else if (argument == "--control-mode" && i + 1 < argc)
        {
            const std::string mode = argv[++i];
            if (mode == "servop")
            {
                options.control_mode = smrcore::peripherals::PicoControlMode::ServoP;
            }
            else if (mode == "fdcc")
            {
                options.control_mode = smrcore::peripherals::PicoControlMode::Fdcc;
            }
            else if (mode == "impedance")
            {
                options.control_mode = smrcore::peripherals::PicoControlMode::CartesianImpedance;
            }
            else
            {
                return false;
            }
        }
        else if ((argument == "--fdcc-max-speed" || argument == "--fdcc-max-accel" ||
                  argument == "--fdcc-follow-gain" || argument == "--fdcc-target-scale" ||
                  argument == "--fdcc-target-feedforward") && i + 1 < argc)
        {
            double &value = argument == "--fdcc-max-speed" ? options.fdcc_max_speed :
                            argument == "--fdcc-max-accel" ? options.fdcc_max_accel :
                            argument == "--fdcc-follow-gain" ? options.fdcc_follow_gain :
                            argument == "--fdcc-target-scale" ? options.fdcc_target_scale :
                            options.fdcc_target_feedforward;
            if (!ParseFiniteNumber(argv[++i], value))
            {
                return false;
            }
        }
        else if ((argument == "--impedance-stiffness" || argument == "--impedance-damping") && i + 1 < argc)
        {
            auto &values = argument == "--impedance-stiffness" ? options.impedance_stiffness : options.impedance_damping;
            if (!ParseSixNumbers(argv[++i], values))
            {
                return false;
            }
        }
        else if ((argument == "--impedance-max-speed" || argument == "--impedance-max-accel" ||
                  argument == "--impedance-follow-gain") && i + 1 < argc)
        {
            double &value = argument == "--impedance-max-speed" ? options.impedance_max_speed :
                            argument == "--impedance-max-accel" ? options.impedance_max_accel : options.impedance_follow_gain;
            if (!ParseFiniteNumber(argv[++i], value))
            {
                return false;
            }
        }
        else if (argument == "--single-mapping-yaw" && i + 1 < argc)
        {
            if (!ParseMappingYaw(argv[++i], options.single_mapping_yaw_deg))
            {
                return false;
            }
        }
        else if (argument == "--operator-frame" && i + 1 < argc)
        {
            if (!ParseOperatorFrame(argv[++i], options.operator_facing_robot))
            {
                return false;
            }
        }
        else if (argument == "--port" && i + 1 < argc)
        {
            if (!ParsePort(argv[++i], options.port))
            {
                return false;
            }
        }
        else if (argument == "--motion-scale" && i + 1 < argc)
        {
            double scale = 0.0;
            if (!ParseMappingScale(argv[++i], scale))
            {
                return false;
            }
            options.translation_scale = scale;
            options.rotation_scale = scale;
        }
        else if (argument == "--translation-scale" && i + 1 < argc)
        {
            if (!ParseMappingScale(argv[++i], options.translation_scale))
            {
                return false;
            }
        }
        else if (argument == "--rotation-scale" && i + 1 < argc)
        {
            if (!ParseMappingScale(argv[++i], options.rotation_scale))
            {
                return false;
            }
        }
        else if (argument == "--j6-speed" && i + 1 < argc)
        {
            if (!ParseMappingScale(argv[++i], options.j6_thumbstick_speed))
            {
                return false;
            }
        }
        else if (argument == "--gripper-force" && i + 1 < argc)
        {
            if (!ParsePercent(argv[++i], 20, options.gripper_force))
            {
                return false;
            }
        }
        else if (argument == "--gripper-speed" && i + 1 < argc)
        {
            if (!ParsePercent(argv[++i], 1, options.gripper_speed))
            {
                return false;
            }
        }
        else if (argument == "--robot-ip" && i + 1 < argc)
        {
            options.robot_ip = argv[++i];
        }
        else if (argument == "--help-advanced")
        {
            PrintAdvancedUsage(argv[0]);
            std::exit(0);
        }
        else if (argument == "--help" || argument == "-h")
        {
            PrintUsage(argv[0]);
            std::exit(0);
        }
        else
        {
            return false;
        }
    }
    return true;
}

} // namespace
int main(int argc, char **argv)
{
    Options options;
    options.translation_scale = .8;
    options.rotation_scale = 1.;
    options.j6_thumbstick_speed = .8;
    if (!ParseOptions(argc, argv, options))
    {
        PrintUsage(argv[0]);
        return 2;
    }
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);
    return session.Run(options);
}
