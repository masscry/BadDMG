#pragma once

#include "common.hh"
#include "device.hh"

#include <span>
#include <stdexcept>
#include <filesystem>
#include <vector>

namespace bdmg {

namespace fs = std::filesystem;

class CartridgeError: public std::runtime_error {
    using std::runtime_error::runtime_error;
};

using SPCartridge = std::shared_ptr<class Cartridge>;

class Cartridge: public IDevice {
    struct OnlySharedPtr { explicit OnlySharedPtr() = default; };
public:
    enum class Type : u8 {
        rom_only = 0x00,
        mbc1 = 0x01,
        mbc1_ram = 0x02,
        mbc1_ram_bat = 0x03,
        mbc2 = 0x05,
        mbc2_bat = 0x06,
        rom_ram = 0x08,
        rom_ram_bat = 0x09,
        mmm01 = 0x0b,
        mmm01_ram = 0x0c,
        mmm01_ram_bat = 0x0d,
        mbc3_timer_bat = 0x0f,
        mbc3_timer_ram_bat = 0x10,
        mbc3 = 0x11,
        mbc3_ram = 0x12,
        mbc3_ram_bat = 0x13,
        mbc5 = 0x19,
        mbc5_ram = 0x1a,
        mbc5_ram_bat = 0x1b,
        mbc5_rumble = 0x1c,
        mbc5_rumble_ram = 0x1d,
        mbc5_rumble_ram_bat = 0x1e,
        mbc6 = 0x20,
        mbc7_sensor_rumble_ram_bat = 0x22,
        pocket_camera = 0xfc,
        bandai_tama5 = 0xfd,
        huc3 = 0xfe,
        huc1_ram_bat = 0xff
    };

    Cartridge(std::span<const u8>, OnlySharedPtr);
    static SPCartridge create(const fs::path& path);
    static SPCartridge create_from_memory(std::span<const u8>);

    u8 read_byte(u16 addr) const final;
    void write_byte(u16, u8) final;

    std::size_t id() const final {
        return DeviceTypeId::cartridge;
    }

    auto title() const -> std::string_view;
    auto type() const -> Type;

    void print_info(std::ostream&);

private:
    std::vector<u8> _memory;
    std::vector<u8> _ram;
    std::string _title;
    Type _type {};
    u32 _rom_size {};
    u32 _ram_size {};

    u32 _current_rom_bank;
    bool _ram_enabled {false};
    u8 _current_ram_bank {0};
};

inline
auto Cartridge::title() const -> std::string_view {
    return _title;
}

inline
auto Cartridge::type() const -> Type {
    return _type;
}

constexpr auto rom_bank_zero = RangeU16{ 0x0000, 0x4000 };
constexpr auto rom_bank_n = RangeU16{0x4000, 0x4000 };
constexpr auto ram_bank = RangeU16 { 0xA000, 0x2000 };
constexpr auto ram_enable_range = RangeU16{ 0x0000, 0x2000 };
constexpr auto rom_bank_select_range = RangeU16{ 0x2000, 0x2000 };
constexpr auto ram_bank_select_range = RangeU16{ 0x4000, 0x2000 };

} // namespace bdmg
