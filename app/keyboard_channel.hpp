#pragma once

#include "keyboard_control.hpp"
#include <mutex>

namespace smrcore::peripherals::keyboard
{
struct DisplayState
{
    bool connected = false;
    bool enabled = false;
    bool switching = false;
    bool dual = false;
    bool dry_run = false;
    bool tool = false;
    Space space = Space::Cartesian;
    double speed = 0.03;
    double angular_speed = 0.15;
};
// Terminal input and the 1kHz worker exchange short snapshots. SDK calls and
// terminal output never hold this mutex. Focus loss survives focus regain.
class Channel
{
public:
    void Push(Snapshot snapshot, double now)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        input_.synchronized = input_.synchronized && snapshot.synchronized;
        input_.continuous = snapshot.continuous;
        input_.held = snapshot.held;
        input_.presses.insert(input_.presses.end(), snapshot.presses.begin(), snapshot.presses.end());
        if (input_.presses.size() > 128)
        {
            input_.presses.clear();
            input_.synchronized = false;
        }
        received_at_ = now;
    }

    Snapshot Take(double now)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto result = input_;
        input_.presses.clear();
        input_.synchronized = true;
        if (received_at_ <= 0 || now < received_at_ || now - received_at_ > 0.15)
        {
            result.synchronized = false;
            result.held.reset();
            result.presses.clear();
        }
        return result;
    }

    void DiscardStartupInput()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        input_ = {};
        received_at_ = 0;
    }

    void Publish(const DisplayState &state)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        display_ = state;
    }

    DisplayState Display()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return display_;
    }

private:
    std::mutex mutex_;
    Snapshot input_;
    DisplayState display_;
    double received_at_ = 0;
};
} // namespace smrcore::peripherals::keyboard
