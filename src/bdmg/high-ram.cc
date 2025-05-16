#include "high-ram.hh"

namespace bdmg {

HighRam::HighRam(OnlySharedPtr) {
    _memory.fill(0);
    _memory[0x0F] = 0xE1;
}

SPHighRam HighRam::create() {
    return std::make_shared<HighRam>(OnlySharedPtr{});
}

u8 HighRam::read_byte(u16 addr) const {
    if (high_ram_range.has(addr)) {
        return _memory[addr - high_ram_range.start];
    }
    return 0xFF;
}

void HighRam::write_byte(u16 addr, u8 data) {
    if (high_ram_range.has(addr)) {
        _memory[addr - high_ram_range.start] = data;
    }
}

} // namespace bdmg
