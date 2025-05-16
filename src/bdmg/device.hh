#pragma once

#include "common.hh"

#include <memory>
#include <unordered_set>

namespace bdmg {

using SPDevice = std::shared_ptr<class IDevice>;

class Bus;

class IDevice : std::enable_shared_from_this<IDevice> {
public:
    virtual ~IDevice() = default;

    virtual u8 read_byte(u16 addr) const = 0;
    virtual void write_byte(u16 addr, u8 data) = 0;
    virtual std::size_t id() const = 0;

    friend
    bool operator==(const IDevice& lhs, const IDevice& rhs) {
        return lhs.id() == rhs.id();
    }

    SPDevice shared();
};

struct HashDevice {
    std::size_t operator()(const SPDevice& device) const {
        return device->id();
    }
};

using SPTickingDevice = std::shared_ptr<class ITickingDevice>;

template<typename DeviceType>
using DeviceSet = std::unordered_set<DeviceType, HashDevice>; 

class ITickingDevice: public IDevice {
public:
    virtual void advance(Bus&, u8 cycles) = 0;
};

inline
bool operator==(const SPDevice& lhs, const SPDevice& rhs) {
    return *lhs == *rhs;
}

inline
SPDevice IDevice::shared() {
    return shared_from_this();
}

enum DeviceTypeId : std::size_t {
    boot_rom = 0,
    cartridge,
    timer,
    serial,
    audio,
    joypad,
    ppu,
    wram,
    hram,
    interrupt
};

} // namespace bdmg
