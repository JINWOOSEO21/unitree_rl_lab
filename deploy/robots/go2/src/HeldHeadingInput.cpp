#include "HeldHeadingInput.h"
#include <X11/Xlib.h>
#include <X11/keysym.h>

struct Go2HeldHeadingInput::Impl
{
    Display* display = nullptr;
    Window terminal = None;
    KeyCode left = 0, right = 0;
    bool armed = false;
    ~Impl() { if (display) XCloseDisplay(display); }
    bool focused() const
    {
        Window focus; int revert;
        XGetInputFocus(display, &focus, &revert);
        return terminal != None && terminal != PointerRoot && focus == terminal;
    }
};

Go2HeldHeadingInput::Go2HeldHeadingInput() : impl_(std::make_unique<Impl>())
{
    impl_->display = XOpenDisplay(nullptr);
    if (!impl_->display) return;
    int revert;
    XGetInputFocus(impl_->display, &impl_->terminal, &revert);
    impl_->left = XKeysymToKeycode(impl_->display, XK_a);
    impl_->right = XKeysymToKeycode(impl_->display, XK_d);
}
Go2HeldHeadingInput::~Go2HeldHeadingInput() = default;
bool Go2HeldHeadingInput::available() const
{
    return impl_->display && impl_->left && impl_->right &&
           impl_->terminal != None && impl_->terminal != PointerRoot;
}
bool Go2HeldHeadingInput::focused() const
{
    return available() && impl_->focused();
}
void Go2HeldHeadingInput::arm()
{
    impl_->armed = available() && impl_->focused();
}
void Go2HeldHeadingInput::clear() { impl_->armed = false; }
Go2HeldHeadingKeys Go2HeldHeadingInput::poll()
{
    if (!impl_->armed) return {};
    if (!impl_->focused()) { clear(); return {}; }
    char keys[32]{};
    XQueryKeymap(impl_->display, keys);
    const auto pressed = [&keys](KeyCode code) {
        return (static_cast<unsigned char>(keys[code / 8]) & (1u << (code % 8))) != 0;
    };
    Go2HeldHeadingKeys result{pressed(impl_->left), pressed(impl_->right)};
    if (!result.left && !result.right) clear();
    return result;
}
