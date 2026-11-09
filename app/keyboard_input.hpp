#pragma once

#include "keyboard_control.hpp"
#include <termios.h>

namespace smrcore::peripherals::keyboard
{
class Input
{
public:
    Input();
    ~Input();
    Input(const Input &) = delete;
    Input &operator=(const Input &) = delete;
    void Activate();
    Snapshot Read(double now);

private:
    termios saved_{};
    bool active_ = false;
    bool report_modes_ = false;
    bool paste_ = false;
    bool focused_ = true;
    std::string escape_;
    double escape_at_ = 0.0;
};
} // namespace smrcore::peripherals::keyboard
