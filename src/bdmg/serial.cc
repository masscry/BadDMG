#include "serial.hh"

namespace bdmg {

SPSerial Serial::create() {
    return std::make_shared<Serial>();
}

Serial::Serial() {}

u8 Serial::read_byte(u16 addr) const {
    switch (addr) {
        case 0xFF01: return _sb;
        case 0xFF02: return _sc;
        default: return 0xFF;
    }
}

void Serial::write_byte(u16 addr, u8 data) {
    switch (addr) {
        case 0xFF01:
            _sb = data;
            break;
        case 0xFF02:
            _sc = data;
            if (data & 0x80) {
                _transfer_in_progress = true;
                _remaining_cycles = 8;
            }
            break;
    }
}

void Serial::advance(Bus&, u8) {
    if (_transfer_in_progress) {
        if (--_remaining_cycles <= 0) {
            _transfer_in_progress = false;
            _sc &= 0x7F;
            _sb = 0xFF;
        }
    }
}

} // namespace bdmg
