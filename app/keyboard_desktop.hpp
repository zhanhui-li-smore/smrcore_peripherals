#pragma once
#include "keyboard_control.hpp"
#include <cstdint>
#include <memory>

namespace smrcore::peripherals::keyboard
{
// Only a Space character received by this terminal can bind desktop focus.
// Physical key state supplies releases; terminal autorepeat cannot toggle twice.
class HeldInput
{
public:
    Snapshot Apply(const Snapshot &terminal, const Keys &keys, uint32_t focus, bool modifiers)
    {
        Snapshot result;
        result.continuous = true;
        latched_ &= keys;
        const bool safe_focus = focus > 1;
        if (!terminal.synchronized || modifiers || !safe_focus || (focus_ != 0 && focus_ != focus))
        {
            focus_ = 0;
            latched_ |= keys;
            result.synchronized = false;
        }
        else
        {
            for (int key : terminal.presses)
            {
                if (key != KEY_SPACE && key != KEY_TAB) { continue; }
                if (keys[key] && !latched_[key])
                {
                    latched_.set(key);
                    if (key == KEY_SPACE) { focus_ = focus; }
                    if (focus_ != 0) { result.presses.push_back(key); }
                }
            }
            result.synchronized = focus_ == focus;
            if (result.synchronized) { result.held = keys; }
        }
        for (int key : terminal.presses)
        {
            if (key == KEY_ESC) { result.presses.push_back(KEY_ESC); }
        }
        return result;
    }
private:
    uint32_t focus_ = 0;
    Keys latched_;
};

// Local X11 session only, queries predefined teleop keys, never grabs input
// or creates a window. SSH/Wayland use the bounded terminal fallback.
class DesktopKeys
{
public:
    DesktopKeys();
    ~DesktopKeys();
    DesktopKeys(const DesktopKeys &) = delete;
    DesktopKeys &operator=(const DesktopKeys &) = delete;
    static bool Available();
    Snapshot Read(const Snapshot &terminal);
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace smrcore::peripherals::keyboard
