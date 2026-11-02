#pragma once

#include "common.hh"
#include "device.hh"
#include <memory>

namespace bdmg {

using SPJoypad = std::shared_ptr<class Joypad>;

struct InputState {
    bool up = false;
    bool down = false;
    bool left = false;
    bool right = false;
    bool a = false;
    bool b = false;
    bool select = false;
    bool start = false;
};

inline InputState operator|(const InputState& lhs, const InputState& rhs) {
    return {
        .up = lhs.up || rhs.up,
        .down = lhs.down || rhs.down,
        .left = lhs.left || rhs.left,
        .right = lhs.right || rhs.right,
        .a = lhs.a || rhs.a,
        .b = lhs.b || rhs.b,
        .select = lhs.select || rhs.select,
        .start = lhs.start || rhs.start,
    };
}

enum class Button : u8 {
    right  = 0x01,
    left   = 0x02,
    up     = 0x04,
    down   = 0x08,
    a      = 0x01,
    b      = 0x02,
    select = 0x04,
    start  = 0x08
};

class Joypad : public IDevice {
    struct OnlySharedPtr { explicit OnlySharedPtr() = default; };

public:
    explicit Joypad(OnlySharedPtr);
    static SPJoypad create();

    u8 read_byte(u16 addr) const final;
    void write_byte(u16 addr, u8 data) final;

    std::size_t id() const final {
        return DeviceTypeId::joypad;
    }

    void poll(const InputState& state);

private:
    u8 _joypad_reg = 0xFF;
    u8 _button_keys = 0x0F;
    u8 _dpad_keys = 0x0F;
};

constexpr RangeU16 joypad_range {.start = 0xFF00, .length = 1};

} // namespace bdmg
