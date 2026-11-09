#pragma once

#include <linux/input-event-codes.h>

#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace smrcore::peripherals::keyboard
{
using Delta = std::array<double, 6>;
using Commands = std::array<Delta, 2>;
using Keys = std::bitset<KEY_CNT>;

struct Snapshot
{
    std::vector<int> presses;
    Keys held;
    bool continuous = false;
    bool synchronized = true;
};

// Columns are +X/-X, +Y/-Y, +Z/-Z, +Rx/-Rx, +Ry/-Ry, +Rz/-Rz.
inline constexpr std::array<std::array<int, 12>, 2> KEY_MAP{{
    {{KEY_W, KEY_S, KEY_A, KEY_D, KEY_R, KEY_F,
      KEY_Z, KEY_X, KEY_C, KEY_V, KEY_Q, KEY_E}},
    {{KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_PAGEUP, KEY_PAGEDOWN,
      KEY_I, KEY_K, KEY_J, KEY_L, KEY_U, KEY_O}}
}};

enum class Space { Cartesian, Joint };
inline const char *Name(Space space) { return space == Space::Joint ? "joint" : "cartesian"; }
inline constexpr double TRANSLATION_STEP = 0.001;
inline constexpr double ANGLE_STEP = 0.008726646259971648; // 0.5 degree

struct Feedback
{
    double timestamp = 0.0;
    Delta positions{};
    Delta velocities{};
    bool connected = false;
    bool valid = false;
    bool sdk_source = false;
    bool kinematics = false;
    bool fault = false;
    std::string reference;
    std::array<double, 6> tool_offset{};
};

// A single tick produces both commands from the same keyboard snapshot.
// Backend ServoRelativeP/Ruckig owns pose integration, IK and motion limits.
class Control
{
public:
    Control(bool dual_arm, double speed, double angular_speed)
        : count_(dual_arm ? 2 : 1), speed_(speed), angular_speed_(angular_speed)
    {
        if (!std::isfinite(speed) || speed <= 0.0 || speed > 0.1 ||
            !std::isfinite(angular_speed) || angular_speed <= 0.0 || angular_speed > 0.5)
        {
            throw std::invalid_argument("speed must be (0,0.1] m/s; angular speed (0,0.5] rad/s");
        }
    }

    bool Ready() const { return enabled_; }
    size_t ArmCount() const { return count_; }
    Space CurrentSpace() const { return space_; }
    bool Switching() const { return switching_; }
    void Pause() { enabled_ = false; remaining_ = {}; }

    Commands UpdateHeld(const Snapshot &input, double elapsed, double now)
    {
        std::vector<int> controls;
        for (int key : input.presses)
        {
            if (key == KEY_SPACE || key == KEY_TAB || key == KEY_ESC) { controls.push_back(key); }
        }
        Update(controls, elapsed, now, input.synchronized);
        Commands commands{};
        if (!enabled_ || switching_) { return commands; }
        const double dt = std::min(elapsed, 0.001);
        for (size_t arm = 0; arm < count_; ++arm)
        {
            for (size_t axis = 0; axis < 6; ++axis)
            {
                commands[arm][axis] = static_cast<double>(input.held[KEY_MAP[arm][2 * axis]]) -
                                      static_cast<double>(input.held[KEY_MAP[arm][2 * axis + 1]]);
            }
            for (size_t offset : {size_t{0}, size_t{3}})
            {
                double squared = 0.0;
                for (size_t axis = offset; axis < offset + 3; ++axis)
                {
                    squared += commands[arm][axis] * commands[arm][axis];
                }
                const double speed = space_ == Space::Joint || offset == 3 ? angular_speed_ : speed_;
                const double divisor = space_ == Space::Joint ? 1.0 : std::max(1.0, std::sqrt(squared));
                for (size_t axis = offset; axis < offset + 3; ++axis)
                {
                    commands[arm][axis] *= speed * dt / divisor;
                }
            }
        }
        return commands;
    }

    Commands Update(const std::vector<int> &presses, double elapsed, double now,
                    bool synchronized = true)
    {
        Commands commands{};
        if (!synchronized || !std::isfinite(now) || !std::isfinite(elapsed) || elapsed <= 0.0 || elapsed > 0.05)
        {
            Pause();
            return commands;
        }
        for (int key : presses)
        {
            if (key == KEY_SPACE)
            {
                if (enabled_) { Pause(); }
                else if (!switching_) { enabled_ = true; }
                continue;
            }
            if (key == KEY_TAB)
            {
                const bool fresh = now - last_tab_ > 0.7;
                last_tab_ = now;
                Pause();
                if (!switching_ && fresh) { switching_ = true; still_since_ = -1.0; }
                continue;
            }
            if (key == KEY_ESC) { Pause(); return {}; }
            if (!enabled_ || switching_) { continue; }
            for (size_t arm = 0; arm < count_; ++arm)
            {
                for (size_t axis = 0; axis < 6; ++axis)
                {
                    const double step = space_ == Space::Joint || axis >= 3 ? ANGLE_STEP : TRANSLATION_STEP;
                    if (key == KEY_MAP[arm][axis * 2])
                    {
                        remaining_[arm][axis] = std::min(step, remaining_[arm][axis] + step);
                    }
                    else if (key == KEY_MAP[arm][axis * 2 + 1])
                    {
                        remaining_[arm][axis] = std::max(-step, remaining_[arm][axis] - step);
                    }
                }
            }
        }
        if (!enabled_ || switching_) { return commands; }
        const double dt = std::min(elapsed, 0.001);
        for (size_t arm = 0; arm < count_; ++arm)
        {
            for (size_t axis = 0; axis < 6; ++axis)
            {
                const double speed = space_ == Space::Joint || axis >= 3 ? angular_speed_ : speed_;
                commands[arm][axis] = std::clamp(remaining_[arm][axis], -speed * dt, speed * dt);
            }
            if (space_ == Space::Cartesian)
            {
                for (size_t offset : {size_t{0}, size_t{3}})
                {
                    double squared = 0.0;
                    for (size_t axis = offset; axis < offset + 3; ++axis)
                    {
                        squared += commands[arm][axis] * commands[arm][axis];
                    }
                    const double limit = (offset == 0 ? speed_ : angular_speed_) * dt;
                    const double scale = std::max(1.0, std::sqrt(squared) / limit);
                    for (size_t axis = offset; axis < offset + 3; ++axis) { commands[arm][axis] /= scale; }
                }
            }
            for (size_t axis = 0; axis < 6; ++axis) { remaining_[arm][axis] -= commands[arm][axis]; }
        }
        return commands;
    }

    // Called after feedback validation. Both arms must provide fresh stopped
    // observations for 100ms before a paused space transition can complete.
    bool TrySwitch(const std::array<Feedback, 2> &states, double now)
    {
        if (!switching_) { return false; }
        bool fresh = true;
        for (size_t arm = 0; arm < count_; ++arm)
        {
            const auto &state = states[arm];
            fresh = fresh && state.timestamp != switch_stamp_[arm];
            for (size_t joint = 0; joint < 6; ++joint)
            {
                if (!std::isfinite(state.positions[joint]) || !std::isfinite(state.velocities[joint]) ||
                    std::abs(state.velocities[joint]) >= 0.01 ||
                    (still_since_ >= 0 && std::abs(state.positions[joint] - switch_position_[arm][joint]) > 0.001))
                {
                    still_since_ = -1.0;
                    return false;
                }
            }
        }
        if (!fresh) { return false; }
        for (size_t arm = 0; arm < count_; ++arm) { switch_stamp_[arm] = states[arm].timestamp; }
        if (still_since_ < 0)
        {
            still_since_ = now;
            for (size_t arm = 0; arm < count_; ++arm) { switch_position_[arm] = states[arm].positions; }
            return false;
        }
        if (now - still_since_ < 0.1) { return false; }
        space_ = space_ == Space::Cartesian ? Space::Joint : Space::Cartesian;
        switching_ = false;
        return true;
    }

    // Called for every arm before either arm receives a nonzero command.
    std::string Check(const std::array<Feedback, 2> &states, double now)
    {
        if (!std::isfinite(now))
        {
            return "invalid local clock";
        }
        for (size_t arm = 0; arm < count_; ++arm)
        {
            const auto &state = states[arm];
            if (!state.connected || !state.valid || !state.sdk_source ||
                !state.kinematics || state.fault || !std::isfinite(state.timestamp) ||
                state.timestamp <= 0.0 || state.reference.empty())
            {
                return "arm " + std::to_string(arm + 1) + ": invalid state, source, mode or fault";
            }
            if (!std::all_of(state.tool_offset.begin(), state.tool_offset.end(),
                             [](double value) { return std::isfinite(value); }))
            {
                return "invalid tool offset";
            }
            if (!seen_[arm])
            {
                initial_[arm] = state;
                seen_[arm] = true;
                changed_at_[arm] = now;
            }
            if (state.reference != initial_[arm].reference ||
                state.tool_offset != initial_[arm].tool_offset)
            {
                return "arm " + std::to_string(arm + 1) + ": reference/tool changed";
            }
            if (state.timestamp != stamp_[arm])
            {
                stamp_[arm] = state.timestamp;
                changed_at_[arm] = now;
            }
            if (now - changed_at_[arm] > 0.25 || now < changed_at_[arm])
            {
                return "arm " + std::to_string(arm + 1) + ": feedback stale";
            }
        }
        return {};
    }

private:
    size_t count_;
    double speed_;
    double angular_speed_;
    bool enabled_ = false;
    Space space_ = Space::Cartesian;
    Commands remaining_{};
    double last_tab_ = -1.0;
    bool switching_ = false;
    double still_since_ = -1.0;
    std::array<double, 2> switch_stamp_{};
    Commands switch_position_{};
    std::array<bool, 2> seen_{};
    std::array<double, 2> stamp_{};
    std::array<double, 2> changed_at_{};
    std::array<Feedback, 2> initial_{};
};
} // namespace smrcore::peripherals::keyboard
