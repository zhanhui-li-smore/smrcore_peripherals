#include "keyboard_input.hpp"

#include <cerrno>
#include <cctype>
#include <cstring>
#include <poll.h>
#include <unistd.h>

namespace smrcore::peripherals::keyboard
{
namespace
{
int Letter(unsigned char value)
{
    switch (std::tolower(value))
    {
    case 'w': return KEY_W; case 's': return KEY_S;
    case 'a': return KEY_A; case 'd': return KEY_D;
    case 'r': return KEY_R; case 'f': return KEY_F;
    case 'z': return KEY_Z; case 'x': return KEY_X;
    case 'c': return KEY_C; case 'v': return KEY_V;
    case 'q': return KEY_Q; case 'e': return KEY_E;
    case 'i': return KEY_I; case 'k': return KEY_K;
    case 'j': return KEY_J; case 'l': return KEY_L;
    case 'u': return KEY_U; case 'o': return KEY_O;
    case ' ': return KEY_SPACE; case '\t': return KEY_TAB;
    case 3: return KEY_ESC;
    default: return -1;
    }
}
void ReportModes(bool enable)
{
    const char *sequence = enable ? "\x1b[?2004h\x1b[?1004h" : "\x1b[?2004l\x1b[?1004l";
    const auto ignored = write(STDOUT_FILENO, sequence, std::strlen(sequence));
    (void)ignored;
}
} // namespace

Input::Input()
{
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) < 0)
    {
        throw std::runtime_error("Interactive terminal required (use ssh -t for a remote shell)");
    }
}

void Input::Activate()
{
    auto settings = saved_;
    settings.c_lflag &= ~(ICANON | ECHO | ECHONL);
    settings.c_lflag |= ISIG;
    settings.c_iflag &= ~(IXON | IXOFF);
    settings.c_cc[VINTR] = 3;
    settings.c_cc[VQUIT] = _POSIX_VDISABLE;
    settings.c_cc[VSUSP] = _POSIX_VDISABLE;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    // Discard keystrokes accumulated during SDK startup.
    if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &settings) < 0)
    {
        throw std::runtime_error("Cannot configure terminal input");
    }
    active_ = true;
    report_modes_ = isatty(STDOUT_FILENO);
    if (report_modes_) { ReportModes(true); }
}

Input::~Input()
{
    if (active_) { tcsetattr(STDIN_FILENO, TCSANOW, &saved_); }
    if (report_modes_) { ReportModes(false); }
}

Snapshot Input::Read(double now)
{
    Snapshot snapshot;
    pollfd descriptor{STDIN_FILENO, POLLIN, 0};
    const int result = poll(&descriptor, 1, 0);
    if ((result < 0 && errno != EINTR) || (descriptor.revents & (POLLHUP | POLLERR | POLLNVAL)))
    {
        throw std::runtime_error("Terminal disconnected");
    }
    if (descriptor.revents & POLLIN)
    {
        std::array<unsigned char, 256> bytes{};
        const auto size = read(STDIN_FILENO, bytes.data(), bytes.size());
        if (size < 0 && errno != EINTR) { throw std::runtime_error("Terminal read failed"); }
        if (size == static_cast<ssize_t>(bytes.size()))
        {
            tcflush(STDIN_FILENO, TCIFLUSH);
            escape_.clear();
            snapshot.synchronized = false;
            return snapshot;
        }
        for (ssize_t i = 0; i < size; ++i)
        {
            const auto value = bytes[i];
            if (value == 27) { escape_ = "\x1b"; escape_at_ = now; continue; }
            if (!escape_.empty())
            {
                escape_ += static_cast<char>(value);
                if (escape_.size() == 2 && (value == '[' || value == 'O')) { continue; }
                const bool complete = escape_.size() >= 3 && value >= 0x40 && value <= 0x7e;
                if (complete)
                {
                    if (escape_ == "\x1b[200~") { paste_ = true; snapshot.synchronized = false; }
                    else if (escape_ == "\x1b[201~") { paste_ = false; }
                    else if (escape_ == "\x1b[O") { focused_ = false; snapshot.synchronized = false; }
                    else if (escape_ == "\x1b[I") { focused_ = true; }
                    else if (!paste_)
                    {
                        int key = -1;
                        if (escape_ == "\x1b[A" || escape_ == "\x1bOA") { key = KEY_UP; }
                        else if (escape_ == "\x1b[B" || escape_ == "\x1bOB") { key = KEY_DOWN; }
                        else if (escape_ == "\x1b[C" || escape_ == "\x1bOC") { key = KEY_RIGHT; }
                        else if (escape_ == "\x1b[D" || escape_ == "\x1bOD") { key = KEY_LEFT; }
                        else if (escape_ == "\x1b[5~") { key = KEY_PAGEUP; }
                        else if (escape_ == "\x1b[6~") { key = KEY_PAGEDOWN; }
                        if (key >= 0) { snapshot.presses.push_back(key); }
                    }
                    escape_.clear();
                }
                else if (escape_.size() > 24 || (escape_.size() == 2 && value != '[' && value != 'O'))
                {
                    escape_.clear();
                    snapshot.synchronized = false;
                }
                continue;
            }
            if (!paste_)
            {
                const int key = Letter(value);
                if (key >= 0) { snapshot.presses.push_back(key); }
            }
        }
    }
    if (!escape_.empty() && now - escape_at_ > 0.05)
    {
        if (escape_ == "\x1b" && !paste_) { snapshot.presses.push_back(KEY_ESC); }
        else { snapshot.synchronized = false; }
        escape_.clear();
    }
    if (paste_ || !focused_) { snapshot.synchronized = false; }
    return snapshot;
}
} // namespace smrcore::peripherals::keyboard
