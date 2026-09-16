#include "HeldHeadingInput.h"
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <cassert>
#include <cstring>

// Isolated fake X server: never inject keys into the user's desktop or robot.
namespace {
bool connected = true;
Window focus = 42;
char keymap[32]{};
void key(unsigned code, bool down) {
    if (down) keymap[code / 8] |= 1u << (code % 8);
    else keymap[code / 8] &= ~(1u << (code % 8));
}
}
extern "C" {
Display* XOpenDisplay(const char*) { return connected ? reinterpret_cast<Display*>(1) : nullptr; }
int XCloseDisplay(Display*) { return 0; }
int XGetInputFocus(Display*, Window* w, int* revert) { *w = focus; *revert = 0; return 1; }
KeyCode XKeysymToKeycode(Display*, KeySym sym) { return sym == XK_a ? 38 : 40; }
int XQueryKeymap(Display*, char out[32]) { std::memcpy(out, keymap, 32); return 1; }
}
int main()
{
    Go2HeldHeadingInput input;
    assert(input.available());
    key(38, true);
    assert(!input.poll().left); // No terminal activation, no global capture.
    input.arm();
    for (int i = 0; i < 100; ++i) assert(input.poll().left); // No repeat required.
    key(38, false);
    assert(!input.poll().left);
    key(40, true); input.arm();
    assert(input.poll().right);
    key(38, true);
    auto both = input.poll(); assert(both.left && both.right);
    focus = 43;
    auto lost = input.poll(); assert(!lost.left && !lost.right);
    focus = 42;
    assert(!input.poll().left); // Focus return alone cannot reactivate held keys.
    input.arm(); input.clear(); assert(!input.poll().right);
    connected = false;
    Go2HeldHeadingInput unavailable;
    assert(!unavailable.available());
    unavailable.arm(); assert(!unavailable.poll().left);
}
