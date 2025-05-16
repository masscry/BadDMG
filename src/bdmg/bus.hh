#pragma once

#include "common.hh"
#include "pixel-processor.hh"
#include "timer.hh"

#include <stdexcept>

namespace bdmg {

class BusError: public std::runtime_error {
    using std::runtime_error::runtime_error;
};

constexpr std::size_t memory_size = 0x10000;

using SPBus = std::shared_ptr<class Bus>;

class Bus : public std::enable_shared_from_this<Bus> {
    struct OnlySharedPtr { explicit OnlySharedPtr() = default; };

public:
    explicit Bus(OnlySharedPtr);

    static SPBus create();

    void print_cart_info(std::ostream&);

    u8 read_byte(u16 addr) const;
    void write_byte(u16 addr, u8 data);

    s8 signed_read_byte(u16 addr);

    u8 advance();

    template<typename DevicePtr>
    void register_device(RangeU16, DevicePtr);

private:
    void register_device(RangeU16, SPDevice device);
    void register_device(RangeU16, SPTickingDevice device);

    DeviceSet<SPDevice> _devices;
    DeviceSet<SPTickingDevice> _ticking_devices;
    std::vector<IDevice*> _mapping;
};

inline
s8 Bus::signed_read_byte(u16 addr) {
    return static_cast<s8>(read_byte(addr));
}

template<typename DevicePtr>
void Bus::register_device(RangeU16 range, DevicePtr device) {
    if constexpr (std::is_base_of_v<ITickingDevice, std::decay_t<decltype(*device)>>) {
        return register_device(range, SPTickingDevice(device));
    }
    return register_device(range, SPDevice(device));
}

} // namespace bdmg
