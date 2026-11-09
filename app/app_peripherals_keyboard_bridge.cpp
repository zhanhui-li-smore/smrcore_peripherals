#include "keyboard_control.hpp"
#include "keyboard_input.hpp"
#include "keyboard_channel.hpp"
#include "keyboard_desktop.hpp"
#include "sdk/robot.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>

namespace
{
namespace keyboard = smrcore::peripherals::keyboard;
using Clock = std::chrono::steady_clock;
constexpr auto kPeriod = std::chrono::milliseconds(1);
static_assert(std::atomic_bool::is_always_lock_free, "signal flag must be lock-free");
std::atomic_bool running{true};
void SignalHandler(int) { running = 0; }
double Now() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }

struct Options
{
    std::string robot_ip;
    bool local = false;
    bool dual = false;
    bool motion = false;
    bool dry_run = false;
    bool help = false;
    bool tool = false;
    bool terminal = false;
    double speed = 0.03;
    double angular_speed = 0.15;
};

void Usage()
{
    std::cout
        << "Keyboard teleoperation (terminal; no device permissions)\n"
        << "  --dry-run [--dual-arm]\n"
        << "  (--robot-ip IP | --local) --enable-motion [--dual-arm]\n"
        << "  [--frame base|tool] [--speed 0.03] [--angular-speed 0.15] [--terminal]\n\n"
        << "SPACE toggles motion input. TAB switches cartesian/joint for both arms.\n"
        << "ESC or Ctrl+C exits. Starts paused; changing space returns to paused.\n"
        << "Single/left: X/J1 W/S, Y/J2 A/D, Z/J3 R/F; Rx/J4 Z/X, Ry/J5 C/V, Rz/J6 Q/E.\n"
        << "Right: X/J1 Up/Down, Y/J2 Left/Right, Z/J3 PageUp/PageDown; Rx/J4 I/K, Ry/J5 J/L, Rz/J6 U/O.\n"
        << "Local X11: hold direction keys continuously; release stops target increments.\n"
        << "Keep this terminal focused. Focus loss pauses; Space is required to resume.\n"
        << "Each arm uses its own base/tool frame. --dual-arm routes both key groups concurrently.\n"
        << "--terminal: text/SSH fallback, 1mm / 0.5deg bounded steps; terminal repeat may have gaps.\n";
}

double Number(const std::string &text)
{
    size_t end = 0;
    const double value = std::stod(text, &end);
    if (end != text.size() || !std::isfinite(value))
    {
        throw std::invalid_argument("Invalid number: " + text);
    }
    return value;
}

Options Parse(int argc, char **argv)
{
    Options options;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        const auto value = [&]() -> std::string
        {
            if (i + 1 >= argc)
            {
                throw std::invalid_argument("Missing value for " + arg);
            }
            return argv[++i];
        };
        if (arg == "--help" || arg == "-h") { options.help = true; }
        else if (arg == "--robot-ip") { options.robot_ip = value(); }
        else if (arg == "--local") { options.local = true; }
        else if (arg == "--dual-arm") { options.dual = true; }
        else if (arg == "--enable-motion") { options.motion = true; }
        else if (arg == "--dry-run") { options.dry_run = true; }
        else if (arg == "--terminal") { options.terminal = true; }
        else if (arg == "--speed") { options.speed = Number(value()); }
        else if (arg == "--angular-speed") { options.angular_speed = Number(value()); }
        else if (arg == "--frame")
        {
            const auto frame = value();
            if (frame != "base" && frame != "tool")
            {
                throw std::invalid_argument("--frame must be base or tool");
            }
            options.tool = frame == "tool";
        }
        else { throw std::invalid_argument("Unknown argument: " + arg); }
    }
    keyboard::Control validate(options.dual, options.speed, options.angular_speed);
    if (!options.help)
    {
        if (options.motion == options.dry_run)
        {
            throw std::invalid_argument("Provide exactly one of --dry-run / --enable-motion");
        }
        if (options.motion && (options.local == !options.robot_ip.empty()))
        {
            throw std::invalid_argument("Motion requires exactly one of --robot-ip IP / --local");
        }
        if (options.dry_run && (options.local || !options.robot_ip.empty()))
        {
            throw std::invalid_argument("--dry-run does not accept a robot connection");
        }
    }
    return options;
}

void Require(rcore::sdk::Result result, const std::string &operation)
{
    if (!result.IsSuccess())
    {
        throw std::runtime_error(operation + ": " + std::to_string(result.GetErrorCode()) +
                                 " " + result.GetErrorMsg());
    }
}

keyboard::Feedback Observe(rcore::sdk::Robot &robot)
{
    const auto state = robot.GetState();
    const auto motor = robot.GetMotorStatus();
    keyboard::Feedback result;
    result.connected = robot.IsConnected();
    result.timestamp = state.timestamp;
    result.positions = state.positions;
    result.velocities = state.velocities;
    result.valid = state.cartesian_valid && motor.enabled && motor.operational;
    for (const auto &values : {state.positions, state.velocities})
    {
        result.valid = result.valid && std::all_of(values.begin(), values.end(),
                            [](double value) { return std::isfinite(value); });
    }
    for (const auto &values : {state.cartesian_position, state.cartesian_orientation})
    {
        result.valid = result.valid && std::all_of(values.begin(), values.end(),
                            [](double value) { return std::isfinite(value); });
    }
    result.sdk_source = state.command_source == rcore::sdk::CommandSource::Sdk;
    result.kinematics = state.control_mode == rcore::ControlMode::Kinematics;
    result.fault = state.errors.GetLatchedErrorCount() != 0 || motor.error || motor.estop;
    result.reference = state.reference_frame;
    result.tool_offset = state.tcp_offset_from_nominal;
    return result;
}

class Session
{
public:
    explicit Session(const Options &options)
        : count_(options.dual ? 2 : 1),
          frame_(options.tool ? rcore::CartesianJogFrame::Tool : rcore::CartesianJogFrame::Reference)
    {
    }
    ~Session() { Close(); }
    Session(const Session &) = delete;
    Session &operator=(const Session &) = delete;

    void Prepare(const Options &options)
    {
        for (size_t arm = 0; arm < count_; ++arm)
        {
            robots_[arm] = std::make_unique<rcore::sdk::Robot>();
            rcore::sdk::RobotOptions sdk_options;
            sdk_options.arm_instance = !options.dual ? rcore::sdk::RobotArmInstance::Single :
                arm == 0 ? rcore::sdk::RobotArmInstance::Left : rcore::sdk::RobotArmInstance::Right;
            std::cout << "Connecting " << (options.dual ? (arm == 0 ? "left" : "right") : "single")
                      << "..." << std::endl;
            if (!robots_[arm]->Initialize(options.robot_ip, sdk_options))
            {
                throw std::runtime_error("Robot connection failed");
            }
            initialized_[arm] = true;
            const auto state = robots_[arm]->GetState();
            const auto motor = robots_[arm]->GetMotorStatus();
            if (!robots_[arm]->IsConnected() || !state.cartesian_valid ||
                state.control_mode != rcore::ControlMode::Kinematics ||
                state.errors.GetLatchedErrorCount() != 0 || motor.estop || motor.error || !motor.operational ||
                !std::all_of(state.velocities.begin(), state.velocities.end(),
                    [](double velocity) { return std::isfinite(velocity) && std::abs(velocity) < 0.01; }))
            {
                throw std::runtime_error("Start requires connected, stopped Kinematics mode without faults");
            }
            if (!options.tool && state.reference_frame != "base")
            {
                throw std::runtime_error("Select reference frame base in the HMI before --frame base");
            }
            const auto limits = robots_[arm]->Config().GetControllerConfig();
            for (size_t joint = 0; joint < 6; ++joint)
            {
                lower_[arm][joint] = limits.position_limit_min[joint];
                upper_[arm][joint] = limits.position_limit_max[joint];
                if (!std::isfinite(lower_[arm][joint]) || !std::isfinite(upper_[arm][joint]) ||
                    lower_[arm][joint] >= upper_[arm][joint])
                {
                    throw std::runtime_error("Invalid joint limits from controller");
                }
            }
            const auto source = robots_[arm]->Control().GetCommandSource();
            if (source != rcore::sdk::CommandSource::Sdk && source != rcore::sdk::CommandSource::Unclaimed)
            {
                throw std::runtime_error("Command source is occupied; select SDK control in the HMI first");
            }
        }
        // All arms pass the read-only checks before enabling either one.
        for (size_t arm = 0; arm < count_; ++arm)
        {
            if (!running) { throw std::runtime_error("Startup canceled"); }
            Require(robots_[arm]->Control().SetCommandSource(rcore::sdk::CommandSource::Sdk), "Select SDK source");
            enabled_[arm] = true; // Cleanup also covers an ambiguous/failed Enable.
            Require(robots_[arm]->Enable(), "Enable");
        }
        const double deadline = Now() + 2.0;
        const auto initial = States();
        while (running && Now() < deadline)
        {
            const auto states = States();
            bool ready = true;
            for (size_t arm = 0; arm < count_; ++arm)
            {
                ready = ready && states[arm].connected && states[arm].sdk_source &&
                        states[arm].valid && states[arm].kinematics && !states[arm].fault &&
                        states[arm].timestamp != initial[arm].timestamp;
            }
            if (ready) { return; }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        throw std::runtime_error("Fresh SDK feedback did not arrive after Enable");
    }

    std::array<keyboard::Feedback, 2> States()
    {
        std::array<keyboard::Feedback, 2> states{};
        for (size_t arm = 0; arm < count_; ++arm)
        {
            states[arm] = Observe(*robots_[arm]);
        }
        return states;
    }

    void SetSpace(keyboard::Space space, const std::array<keyboard::Feedback, 2> &states)
    {
        keyboard::Commands seeds{};
        for (size_t arm = 0; arm < count_; ++arm)
        {
            for (size_t joint = 0; joint < 6; ++joint)
            {
                const double q = states[arm].positions[joint];
                if (!std::isfinite(q) || q < lower_[arm][joint] || q > upper_[arm][joint])
                {
                    throw std::runtime_error("Cannot switch space: measured joint outside limits");
                }
                seeds[arm][joint] = q;
            }
        }
        joint_targets_ = seeds;
        space_ = space;
    }

    void Send(const keyboard::Commands &commands)
    {
        auto candidates = joint_targets_;
        // Validate both arms before sending either, preserving target type on failure.
        for (size_t arm = 0; arm < count_; ++arm)
        {
            for (size_t joint = 0; joint < 6; ++joint)
            {
                if (!std::isfinite(commands[arm][joint])) { throw std::runtime_error("Nonfinite increment"); }
                candidates[arm][joint] = std::clamp(candidates[arm][joint] + commands[arm][joint],
                                                  lower_[arm][joint], upper_[arm][joint]);
            }
        }
        for (size_t arm = 0; arm < count_; ++arm)
        {
            if (space_ == keyboard::Space::Joint)
            {
                rcore::sdk::JointPositions q;
                for (size_t joint = 0; joint < 6; ++joint) { q[joint] = candidates[arm][joint]; }
                robots_[arm]->Motion().ServoJ(q);
                joint_targets_[arm] = candidates[arm];
            }
            else
            {
                const auto &d = commands[arm];
                robots_[arm]->Motion().ServoRelativeP({d[0], d[1], d[2], d[3], d[4], d[5]}, frame_);
            }
        }
    }

    void Hold(size_t arm)
    {
        if (space_ == keyboard::Space::Joint)
        {
            rcore::sdk::JointPositions q;
            for (size_t joint = 0; joint < 6; ++joint) { q[joint] = joint_targets_[arm][joint]; }
            robots_[arm]->Motion().ServoJ(q);
        }
        else { robots_[arm]->Motion().ServoRelativeP({}, frame_); }
    }

    bool Close() noexcept
    {
        if (closed_) { return close_ok_; }
        closed_ = true;
        // Send zero increments to BOTH arms before any potentially blocking RPC.
        const double deadline = Now() + 0.5;
        while (Now() < deadline && (enabled_[0] || enabled_[1]))
        {
            for (size_t arm = 0; arm < count_; ++arm)
            {
                if (!enabled_[arm]) { continue; }
                try
                {
                    const auto state = Observe(*robots_[arm]);
                    if (state.connected && state.sdk_source && state.kinematics)
                    {
                        Hold(arm);
                    }
                }
                catch (...) { close_ok_ = false; }
            }
            std::this_thread::sleep_for(kPeriod);
        }
        for (size_t arm = 0; arm < count_; ++arm)
        {
            if (!initialized_[arm]) { continue; }
            if (enabled_[arm])
            {
                // Servo streams do not create Move tasks. StopMotion only stops
                // those tasks and can reject a still-draining Servo planner (3218).
                // Hold above stops target accumulation; Disable ends our Servo session.
                try
                {
                    Require(robots_[arm]->Disable(), "Disable");
                    const double disable_deadline = Now() + 1.0;
                    bool disabled = false;
                    while (Now() < disable_deadline)
                    {
                        if (robots_[arm]->IsConnected() && !robots_[arm]->GetMotorStatus().enabled)
                        {
                            disabled = true;
                            break;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    }
                    if (!disabled) { throw std::runtime_error("Disable acknowledged, but disabled feedback not confirmed"); }
                    std::cout << "Arm " << arm + 1 << " Disable: confirmed (enabled=false)\n";
                }
                catch (const std::exception &error)
                {
                    std::cerr << "Arm " << arm + 1 << " " << error.what() << '\n';
                    close_ok_ = false;
                }
                catch (...) { close_ok_ = false; }
            }
            try { robots_[arm]->Shutdown(); }
            catch (...) { close_ok_ = false; }
        }
        if (!close_ok_) { std::cerr << "Servo cleanup incomplete; check the per-arm error and cabinet status.\n"; }
        return close_ok_;
    }

private:
    size_t count_;
    rcore::CartesianJogFrame frame_;
    std::array<std::unique_ptr<rcore::sdk::Robot>, 2> robots_;
    std::array<bool, 2> initialized_{};
    std::array<bool, 2> enabled_{};
    keyboard::Space space_ = keyboard::Space::Cartesian;
    keyboard::Commands joint_targets_{}, lower_{}, upper_{};
    bool closed_ = false;
    bool close_ok_ = true;
};
int RunControl(const Options &options, keyboard::Input *input, keyboard::Channel *channel)
{
    keyboard::Control control(options.dual, options.speed, options.angular_speed);
    std::unique_ptr<Session> session;
    if (options.motion)
    {
        session = std::make_unique<Session>(options);
        session->Prepare(options);
    }
    // Do not intercept terminal Ctrl+C during potentially slow SDK startup.
    if (input) { input->Activate(); }
    if (channel) { channel->DiscardStartupInput(); }
    std::cout << (session ? "LIVE" : "DRY RUN (no robot connection)")
              << " / " << (options.dual ? "simultaneous left + right" : "single (left key block)")
              << " / frame=" << (options.tool ? "tool" : "base") << '\n'
              << "Press SPACE once to enable direction input; press again to pause.\n";
    auto next = Clock::now();
    double previous = Now();
    double printed = 0.0;
    while (running)
    {
        const double now = Now();
        const auto snapshot = input ? input->Read(now) : channel->Take(now);
        if (!running) { break; }
        if (std::find(snapshot.presses.begin(), snapshot.presses.end(), KEY_ESC) != snapshot.presses.end()) { break; }
        std::array<keyboard::Feedback, 2> states{};
        if (session)
        {
            states = session->States();
            const auto error = control.Check(states, now);
            if (!error.empty()) { throw std::runtime_error(error); }
        }
        else
        {
            // Synthetic stationary feedback solely for dry-run space switching.
            for (auto &state : states) { state.timestamp = now; }
        }
        const bool was_ready = control.Ready();
        const bool was_switching = control.Switching();
        const auto commands = snapshot.continuous ? control.UpdateHeld(snapshot, now - previous, now) :
            control.Update(snapshot.presses, now - previous, now, snapshot.synchronized);
        previous = now;
        if (control.TrySwitch(states, now))
        {
            if (session) { session->SetSpace(control.CurrentSpace(), states); }
            if (input) { std::cout << "Space changed: " << keyboard::Name(control.CurrentSpace()) << "; paused, SPACE to enable.\n"; }
        }
        if (session) { session->Send(commands); }
        if (channel)
        {
            channel->Publish({true, control.Ready(), control.Switching(), options.dual,
                              options.dry_run, options.tool, control.CurrentSpace(),
                              options.speed, options.angular_speed});
        }
        if (input && (!snapshot.presses.empty() || was_ready != control.Ready() ||
            was_switching != control.Switching() || now - printed >= 0.2))
        {
            std::cout << "space=" << keyboard::Name(control.CurrentSpace()) << " "
                      << (control.Switching() ? "switching (waiting for standstill)" : control.Ready() ? "ENABLED" : "PAUSED")
                      << std::fixed << std::setprecision(5);
            for (size_t arm = 0; arm < control.ArmCount(); ++arm)
            {
                std::cout << (arm == 0 ? " L/single delta=[" : " R delta=[");
                for (size_t axis = 0; axis < 6; ++axis) { std::cout << (axis ? "," : "") << commands[arm][axis]; }
                std::cout << "]";
            }
            std::cout << (control.CurrentSpace() == keyboard::Space::Joint ? " rad\n" : " m,rad\n") << std::flush;
            printed = now;
        }
        next += kPeriod;
        if (next < Clock::now()) { next = Clock::now() + kPeriod; }
        std::this_thread::sleep_until(next);
    }
    return !session || session->Close() ? 0 : 1;
}

int RunDesktop(const Options &options)
{
    keyboard::Input input;
    keyboard::DesktopKeys desktop;
    input.Activate();
    keyboard::Channel channel;
    channel.Publish({false, false, false, options.dual, options.dry_run, options.tool,
                     keyboard::Space::Cartesian, options.speed, options.angular_speed});
    std::cout << "Input: continuous held keys in this terminal (local X11)\n";
    std::exception_ptr worker_error, ui_error;
    int result = 1;
    std::thread worker([&]()
    {
        try { result = RunControl(options, nullptr, &channel); }
        catch (...) { worker_error = std::current_exception(); }
        running = false;
    });
    try
    {
        double printed = 0.0;
        keyboard::DisplayState previous;
        while (running)
        {
            auto snapshot = desktop.Read(input.Read(Now()));
            if (std::find(snapshot.presses.begin(), snapshot.presses.end(), KEY_ESC) != snapshot.presses.end())
            {
                running = false;
                break;
            }
            channel.Push(std::move(snapshot), Now());
            const auto state = channel.Display();
            if (state.connected && (Now() - printed >= 0.5 || state.enabled != previous.enabled ||
                                    state.switching != previous.switching || state.space != previous.space))
            {
                std::cout << "\rspace=" << keyboard::Name(state.space) << " "
                          << (state.switching ? "switching" : state.enabled ? "ENABLED" : "PAUSED ")
                          << " | Space: enable/pause | Tab: space | Ctrl+C/Esc: exit      " << std::flush;
                printed = Now();
                previous = state;
            }
            std::this_thread::sleep_for(kPeriod);
        }
    }
    catch (...) { ui_error = std::current_exception(); }
    running = false;
    worker.join(); // Join after Servo hold/disable; then restore terminal settings.
    std::cout << '\n';
    if (worker_error) { std::rethrow_exception(worker_error); }
    if (ui_error) { std::rethrow_exception(ui_error); }
    return result;
}
} // namespace

int main(int argc, char **argv)
{
    try
    {
        const auto options = Parse(argc, argv);
        if (options.help) { Usage(); return 0; }
        running = true;
        std::signal(SIGINT, SignalHandler);
        std::signal(SIGTERM, SignalHandler);
        std::signal(SIGHUP, SignalHandler);
        Usage();
        if (!options.terminal && keyboard::DesktopKeys::Available()) { return RunDesktop(options); }
        std::cout << "Input: bounded terminal steps (--terminal / no local X11)\n";
        keyboard::Input input;
        return RunControl(options, &input, nullptr);
    }
    catch (const std::exception &error)
    {
        std::cerr << "Keyboard teleoperation: " << error.what() << '\n';
        return 1;
    }
}
