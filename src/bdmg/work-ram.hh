#pragma once

#include "common.hh"
#include "device.hh"

#include <array>

namespace bdmg {

class WorkRamError: public std::runtime_error {
    using std::runtime_error::runtime_error;
};

using SPWorkRam = std::shared_ptr<class WorkRam>;

class WorkRam : public IDevice {
    struct OnlySharedPtr { explicit OnlySharedPtr() = default; };
public:
    static constexpr std::size_t size = (0x8 << 10);

    explicit WorkRam(OnlySharedPtr);
    static SPWorkRam create();

    u8 read_byte(u16 addr) const final;
    void write_byte(u16, u8) final;

    std::size_t id() const final {
        return DeviceTypeId::wram;
    }

private:
    std::array<u8, size> _memory;
};

constexpr auto work_ram_range = RangeU16 { 0xC000, WorkRam::size };

} // namespace bdmg
