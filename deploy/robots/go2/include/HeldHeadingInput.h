#pragma once
#include <memory>

struct Go2HeldHeadingKeys { bool left = false; bool right = false; };

// X11 calls are confined to the terminal thread; no global key grab.
class Go2HeldHeadingInput
{
public:
    Go2HeldHeadingInput();
    ~Go2HeldHeadingInput();
    bool available() const;
    void arm();
    void clear();
    Go2HeldHeadingKeys poll();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
