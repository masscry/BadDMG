#include "work-ram.hh"

namespace bdmg {

WorkRam::WorkRam(OnlySharedPtr) {
    _memory.fill(0);
}

SPWorkRam WorkRam::create() {
    return std::make_shared<WorkRam>(OnlySharedPtr{});
}

u8 WorkRam::read_byte(u16 addr) const {
    if (work_ram_range.has(addr)) {
        return _memory[addr - work_ram_range.start];
    }
    panic("address out of work ram");
}

void WorkRam::write_byte(u16 addr, u8 data) {
    if (work_ram_range.has(addr)) {
        _memory[addr - work_ram_range.start] = data;
        return;
    }
    panic("address out of work ram");
}

} // namespace bdmg
