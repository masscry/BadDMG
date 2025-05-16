#pragma once

#include "common.hh"
#include "device.hh"

#include <array>

namespace bdmg {

using SPHighRam = std::shared_ptr<class HighRam>;

class HighRam : public IDevice {
    struct OnlySharedPtr { explicit OnlySharedPtr() = default; };
public:
    static constexpr std::size_t size = 0x7F;

    explicit HighRam(OnlySharedPtr);
    static SPHighRam create();

    u8 read_byte(u16 addr) const final;
    void write_byte(u16, u8) final;

    std::size_t id() const final {
        return DeviceTypeId::hram;
    }

private:
    std::array<u8, size> _memory;
};

constexpr auto high_ram_range = RangeU16 { 0xFF80, HighRam::size};

} // namespace bdmg
