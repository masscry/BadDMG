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

void Joypad::poll(bool right, bool left, bool up, bool down,
                  bool a, bool b, bool select, bool start) {
    _dpad_keys = 0x0F;
    if (right && !left) {
        _dpad_keys &= ~static_cast<u8>(Button::right);
    }
    if (left && !right) {
        _dpad_keys &= ~static_cast<u8>(Button::left);
    }
    if (up && !down) {
        _dpad_keys &= ~static_cast<u8>(Button::up);
    }
    if (down && !up) {
        _dpad_keys &= ~static_cast<u8>(Button::down);
    }

    _button_keys = 0x0F;
    if (a) {
        _button_keys &= ~static_cast<u8>(Button::a);
    }
    if (b) {
        _button_keys &= ~static_cast<u8>(Button::b);
    }
    if (select) {
        _button_keys &= ~static_cast<u8>(Button::select);
    }
    if (start) {
        _button_keys &= ~static_cast<u8>(Button::start);
    }
}

} // namespace bdmg
