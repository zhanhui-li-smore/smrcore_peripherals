#include "keyboard_desktop.hpp"
#ifdef SMR_KEYBOARD_HAS_XCB
#include <xcb/xcb.h>
#include <xcb/xcbext.h>
#include <poll.h>
#include <chrono>
#include <cstdlib>
#include <cstring>

namespace smrcore::peripherals::keyboard
{
namespace
{
struct Binding { uint32_t symbol; int key; };
constexpr Binding bindings[] = {
    {'w',KEY_W}, {'s',KEY_S}, {'a',KEY_A}, {'d',KEY_D}, {'r',KEY_R}, {'f',KEY_F},
    {'z',KEY_Z}, {'x',KEY_X}, {'c',KEY_C}, {'v',KEY_V}, {'q',KEY_Q}, {'e',KEY_E},
    {0xff52,KEY_UP}, {0xff54,KEY_DOWN}, {0xff51,KEY_LEFT}, {0xff53,KEY_RIGHT},
    {0xff55,KEY_PAGEUP}, {0xff56,KEY_PAGEDOWN}, {'i',KEY_I}, {'k',KEY_K},
    {'j',KEY_J}, {'l',KEY_L}, {'u',KEY_U}, {'o',KEY_O}, {' ',KEY_SPACE}, {0xff09,KEY_TAB}
};
} // namespace
class DesktopKeys::Impl
{
public:
    Impl()
    {
        connection_ = xcb_connect(nullptr, nullptr);
        try
        {
            if (!connection_ || xcb_connection_has_error(connection_)) { throw std::runtime_error("Cannot query local X11 keyboard; use --terminal"); }
            const auto *setup = xcb_get_setup(connection_);
            const int count = setup->max_keycode - setup->min_keycode + 1;
            const auto cookie = xcb_get_keyboard_mapping(connection_, setup->min_keycode, count);
            auto reply = Reply<xcb_get_keyboard_mapping_reply_t>(cookie.sequence);
            const auto *symbols = xcb_get_keyboard_mapping_keysyms(reply.get());
            for (int i = 0; i < count; ++i)
            {
                const auto symbol = symbols[i * reply->keysyms_per_keycode];
                const int code = i + setup->min_keycode;
                for (const auto &binding : bindings)
                {
                    if (symbol == binding.symbol) { codes_[binding.key] = code; }
                }
                // Control/Meta/Alt/Super shortcuts pause instead of jogging.
                if (symbol >= 0xffe3 && symbol <= 0xffec) { modifiers_.push_back(code); }
            }
        }
        catch (...) { xcb_disconnect(connection_); connection_ = nullptr; throw; }
    }
    ~Impl() { xcb_disconnect(connection_); }

    Snapshot Read(const Snapshot &terminal)
    {
        const auto key_cookie = xcb_query_keymap(connection_);
        const auto focus_cookie = xcb_get_input_focus(connection_);
        auto keymap = Reply<xcb_query_keymap_reply_t>(key_cookie.sequence);
        auto focus = Reply<xcb_get_input_focus_reply_t>(focus_cookie.sequence);
        const auto down = [&](int code) { return code != 0 && (keymap->keys[code / 8] & (1 << (code % 8))); };
        Keys keys;
        for (const auto &binding : bindings) { keys[binding.key] = down(codes_[binding.key]); }
        bool modifiers = false;
        for (int code : modifiers_) { modifiers = modifiers || down(code); }
        return policy_.Apply(terminal, keys, focus->focus, modifiers);
    }
private:
    template <class T> std::unique_ptr<T, decltype(&std::free)> Reply(unsigned int sequence)
    {
        xcb_flush(connection_);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(40);
        do
        {
            void *reply = nullptr;
            xcb_generic_error_t *error = nullptr;
            if (xcb_poll_for_reply(connection_, sequence, &reply, &error))
            {
                const bool failed = error != nullptr || reply == nullptr;
                std::free(error);
                if (failed) { std::free(reply); throw std::runtime_error("Desktop key query failed"); }
                return {static_cast<T *>(reply), &std::free};
            }
            if (xcb_connection_has_error(connection_)) { break; }
            pollfd fd{xcb_get_file_descriptor(connection_), POLLIN, 0};
            poll(&fd, 1, 1);
        } while (std::chrono::steady_clock::now() < deadline);
        xcb_discard_reply(connection_, sequence);
        throw std::runtime_error("Desktop key query timed out; keyboard session stopped");
    }
    xcb_connection_t *connection_ = nullptr;
    std::array<int, KEY_CNT> codes_{};
    std::vector<int> modifiers_;
    HeldInput policy_;
};

bool DesktopKeys::Available()
{
    const char *session = std::getenv("XDG_SESSION_TYPE");
    return session && std::strcmp(session,"x11") == 0 && std::getenv("DISPLAY") &&
           !std::getenv("SSH_CONNECTION") && !std::getenv("WAYLAND_DISPLAY");
}
DesktopKeys::DesktopKeys() : impl_(std::make_unique<Impl>()) {}
DesktopKeys::~DesktopKeys() = default;
Snapshot DesktopKeys::Read(const Snapshot &terminal) { return impl_->Read(terminal); }
} // namespace smrcore::peripherals::keyboard
#else
namespace smrcore::peripherals::keyboard
{
class DesktopKeys::Impl {};
bool DesktopKeys::Available() { return false; }
DesktopKeys::DesktopKeys() { throw std::runtime_error("Local X11 key support was not built; use --terminal"); }
DesktopKeys::~DesktopKeys() = default;
Snapshot DesktopKeys::Read(const Snapshot &terminal) { return terminal; }
} // namespace smrcore::peripherals::keyboard
#endif
