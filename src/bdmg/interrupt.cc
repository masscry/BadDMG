#include "interrupt.hh"

namespace bdmg {

Interrupt::Interrupt(OnlySharedPtr) {}

SPInterrupt Interrupt::create() {
    return std::make_shared<Interrupt>(OnlySharedPtr{});
}

u8 Interrupt::read_byte(u16 addr) const {
    switch (addr) {
    case int_enable_addr:
        return _int_enable;
    case int_flag_addr:
        return _int_flag;
    default:
        return 0xFF;
    }
}

void Interrupt::write_byte(u16 addr, u8 data) {
    switch (addr) {
    case int_enable_addr:
        _int_enable = data;
        break;
    case int_flag_addr:
        _int_flag = 0xE0 | data;
        break;
    }
}

std::size_t Interrupt::id() const {
    return DeviceTypeId::interrupt;
}

} // namespace bdmg
