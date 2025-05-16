#include "bus.hh"

#include <format>
#include <utility>

namespace bdmg {

constexpr RangeU32 full_mem_range { 0x0000, 0x10000 };

Bus::Bus(OnlySharedPtr)
    : _mapping(full_mem_range.length, nullptr)
{}

SPBus Bus::create() {
    return std::make_shared<Bus>(OnlySharedPtr{});
}

u8 Bus::read_byte(u16 addr) const {
    if (auto device = _mapping[addr]) {
        return device->read_byte(addr);
    }
    return 0xFF;
}

void Bus::write_byte(u16 addr, u8 data) {
    if (auto device = _mapping[addr]) {
        device->write_byte(addr, data);
    }
}

u8 Bus::advance() {
    constexpr auto t_cycles = 4;
    for (auto& device : _ticking_devices) {
        device->advance(*this, t_cycles);
    }
    return t_cycles;
}

void Bus::register_device(RangeU16 range, SPDevice device) {
    if (!_devices.contains(device)) {
        _devices.insert(device);
    }
    for (u32 addr = range.start; addr < range.start + range.length; ++addr) {
        if (auto ptr = std::exchange(_mapping[addr], device.get())) {
            if (ptr != device.get()) {
                throw BusError(std::format(
                    "there a device registered at 0x{:04X}", addr
                ));
            }
        }
    }
}

void Bus::register_device(RangeU16 range, SPTickingDevice device) {
    if (!_ticking_devices.contains(device)) {
        _ticking_devices.insert(device);
    }
    return register_device(range, SPDevice(device));
}

} // namespace bdmg
