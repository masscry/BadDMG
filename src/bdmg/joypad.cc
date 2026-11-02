#include "joypad.hh"

namespace bdmg {

Joypad::Joypad(OnlySharedPtr) {}

SPJoypad Joypad::create() {
    return std::make_shared<Joypad>(OnlySharedPtr{});
}

u8 Joypad::read_byte(u16 addr) const {
    if (addr == 0xFF00) {
        u8 result = _joypad_reg;
        if (!(result & 0x20)) {
            result = (result & 0xF0) | _button_keys;
        }
        if (!(result & 0x10)) {
            result = (result & 0xF0) | _dpad_keys;
        }
        return result;
    }
    return 0xFF;
}

void Joypad::write_byte(u16 addr, u8 data) {
    if (addr == 0xFF00) {
        _joypad_reg = (data & 0x30) | 0xCF;
    }
}

void Joypad::poll(const InputState& state) {
    _dpad_keys = 0x0F;
    if (state.right && !state.left) {
        _dpad_keys &= ~static_cast<u8>(Button::right);
    }
    if (state.left && !state.right) {
        _dpad_keys &= ~static_cast<u8>(Button::left);
    }
    if (state.up && !state.down) {
        _dpad_keys &= ~static_cast<u8>(Button::up);
    }
    if (state.down && !state.up) {
        _dpad_keys &= ~static_cast<u8>(Button::down);
    }

    _button_keys = 0x0F;
    if (state.a) {
        _button_keys &= ~static_cast<u8>(Button::a);
    }
    if (state.b) {
        _button_keys &= ~static_cast<u8>(Button::b);
    }
    if (state.select) {
        _button_keys &= ~static_cast<u8>(Button::select);
    }
    if (state.start) {
        _button_keys &= ~static_cast<u8>(Button::start);
    }
}

} // namespace bdmg
