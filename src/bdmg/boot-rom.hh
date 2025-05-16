#pragma once

#include "common.hh"
#include "device.hh"

#include <array>
#include <filesystem>
#include <span>

namespace bdmg {

namespace fs = std::filesystem;

class BootRomError: public std::runtime_error {
    using std::runtime_error::runtime_error;
};

using SPBootRom = std::shared_ptr<class BootRom>;

class BootRom : public IDevice {
    struct OnlySharedPtr { explicit OnlySharedPtr() = default; };
public:

    static constexpr std::size_t size = 0x100;

    BootRom(std::span<const u8, size>, OnlySharedPtr);
    static SPBootRom create(const fs::path& path);
    static SPBootRom create_from_memory(std::span<const u8, size>);

    u8 read_byte(u16 addr) const final {
        return _memory[addr];
    }

    void write_byte(u16, u8) final {}

    std::size_t id() const final {
        return DeviceTypeId::boot_rom;
    }

private:
    std::array<u8, size> _memory;
};

constexpr RangeU16 boot_rom_memory_range {.start = 0x0, .length = BootRom::size};

} // namespace bdmg
