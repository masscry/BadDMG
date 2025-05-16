#include "cartridge.hh"
#include <fstream>
#include <iostream>
#include <format>

namespace bdmg {

constexpr auto title_range = RangeU16{0x134,0x144-0x134};
constexpr auto olic_range = RangeU16{0x14b,1};
constexpr auto nlic_range = RangeU16{0x144,2};
constexpr auto type_range = RangeU16{0x147, 1};
constexpr auto ram_size_range = RangeU16{0x149, 1};

template<typename Container>
auto span_from(const Container& data, RangeU16 range) {
    auto span = std::span(data.begin(), data.end());
    return span.subspan(range.start, range.length);
}

auto string_from(auto data) {
    return std::string(data.begin(), data.end());
}

std::string_view type_text(Cartridge::Type type) {
    switch(type) {
    case Cartridge::Type::rom_only: return "ROM ONLY";
    case Cartridge::Type::mbc1: return "MBC1";
    case Cartridge::Type::mbc1_ram: return "MBC1+RAM";
    case Cartridge::Type::mbc1_ram_bat: return "MBC1+RAM+BATTERY";
    case Cartridge::Type::mbc2: return "MBC2";
    case Cartridge::Type::mbc2_bat: return "MBC2+BATTERY";
    case Cartridge::Type::rom_ram: return "ROM+RAM";
    case Cartridge::Type::rom_ram_bat: return "ROM+RAM+BATTERY";
    case Cartridge::Type::mmm01: return "MMM01";
    case Cartridge::Type::mmm01_ram: return "MMM01+RAM";
    case Cartridge::Type::mmm01_ram_bat: return "MMM01+RAM+BATTERY";
    case Cartridge::Type::mbc3_timer_bat: return "MBC3+TIMER+RAM";
    case Cartridge::Type::mbc3_timer_ram_bat: return "MBC3+TIMER+RAM+BATTERY";
    case Cartridge::Type::mbc3: return "MBC3";
    case Cartridge::Type::mbc3_ram: return "MBC3+RAM";
    case Cartridge::Type::mbc3_ram_bat: return "MBC3+RAM+BATTERY";
    case Cartridge::Type::mbc5: return "MBC5";
    case Cartridge::Type::mbc5_ram: return "MBC5+RAM";
    case Cartridge::Type::mbc5_ram_bat: return "MBC5+RAM+BATTERY";
    case Cartridge::Type::mbc5_rumble: return "MBC5+RUMBLE";
    case Cartridge::Type::mbc5_rumble_ram: return "MBC5+RUMBLE+RAM";
    case Cartridge::Type::mbc5_rumble_ram_bat: return "MBC5+RUMBLE+RAM+BATTERY";
    case Cartridge::Type::mbc6: return "MBC6";
    case Cartridge::Type::mbc7_sensor_rumble_ram_bat: return "MBC7+SENSOR+RUBLE+RAM+BATTERY";
    case Cartridge::Type::pocket_camera: return "POCKET CAMERA";
    case Cartridge::Type::bandai_tama5: return "BANDAI_TAMA5";
    case Cartridge::Type::huc3: return "HUC3";
    case Cartridge::Type::huc1_ram_bat: return "HUC1+RAM+BATTERY";
    default: return "???";
    }
}

constexpr u32 operator""_KB(unsigned long long value) {
    return value * 1024;
}

u32 calculate_ram_size(u8 size_code) {
    constexpr u32 size_mapping[] = {
        0, 8_KB, 32_KB, 128_KB, 64_KB
    };
    if (size_code >= std::size(size_mapping)) {
        return 0;
    }
    return size_mapping[size_code];
}

Cartridge::Cartridge(std::span<const u8> data, OnlySharedPtr)
    : _memory(data.begin(), data.end()) {
    _type = static_cast<Type>(_memory[type_range.start]);
    _rom_size = (data.size() + 0x3FFF) & ~0x3FFF;
    while (_rom_size & (_rom_size - 1)) {
        _rom_size |= _rom_size >> 1;
        _rom_size++;
    }
    if (_rom_size < 0x8000) {
        _rom_size = 0x8000;
    }
    _ram_size = calculate_ram_size(_memory[ram_size_range.start]);
    if (_ram_size > 0) {
        _ram.resize(_ram_size);
    }

    _current_rom_bank = 1;
    _ram_enabled = false;
    _current_ram_bank = 0;

    switch(_type) {
    case Type::rom_only:
    case Type::mbc1:
    case Type::mbc1_ram:
    case Type::mbc1_ram_bat:
        break;
    default:
        throw CartridgeError(std::format("unsupported cartridge type: {}", type_text(_type)));
    }
}

SPCartridge Cartridge::create(const fs::path& path) {
    std::vector<u8> buffer;
    auto file = std::ifstream(path, std::ios::binary);

    if (!file || file.bad()) {
        throw CartridgeError(std::format("failed to open cartridge ROM file ({})", path.native()));
    }

    auto size = fs::file_size(path);
    buffer.resize(size);

    file.read(reinterpret_cast<char*>(buffer.data()), size);
    return create_from_memory(std::span<const u8>(buffer));
}

SPCartridge Cartridge::create_from_memory(std::span<const u8> data) {
    return std::make_shared<Cartridge>(data, OnlySharedPtr{});
}

u8 Cartridge::read_byte(u16 addr) const {
    if (rom_bank_zero.has(addr)) {
        return _memory[addr];
    }
    if (rom_bank_n.has(addr)) {
        u32 effective_address = (addr & 0x3FFF) + (_current_rom_bank * 0x4000);
        return _memory[effective_address & (_rom_size - 1)];
    }
    if (ram_bank.has(addr) && _ram_enabled && !_ram.empty()) {
        u32 effective_address = (addr & 0x1FFF) + (_current_ram_bank * 0x2000);
        return _ram[effective_address % _ram_size];
    }
    return 0xFF;
}

void Cartridge::write_byte(u16 addr, u8 data) {
    if (ram_enable_range.has(addr)) {
        _ram_enabled = (data & 0x0F) == 0x0A;
        return;
    }
    
    if (rom_bank_select_range.has(addr)) {
        if (_type == Type::mbc1 || _type == Type::mbc1_ram || _type == Type::mbc1_ram_bat) {
            u8 bank = (data & 0x1F);
            if (bank == 0) bank = 1;
            _current_rom_bank = bank;
        }
        return;
    }
    
    if (ram_bank_select_range.has(addr)) {
        if (_type == Type::mbc1_ram || _type == Type::mbc1_ram_bat) {
            _current_ram_bank = data & 0x03;
        }
        return;
    }
    
    if (ram_bank.has(addr) && _ram_enabled && !_ram.empty()) {
        u32 effective_address = (addr & 0x1FFF) + (_current_ram_bank * 0x2000);
        _ram[effective_address % _ram_size] = data;
    }
}

void Cartridge::print_info(std::ostream& out) {
    auto title_str = string_from(span_from(_memory, title_range));

    out << "TITLE: " << title_str << std::endl;

    auto old_licensee_span = span_from(_memory, olic_range);
    if (old_licensee_span[0] != 0x33) {
        out << "LICENSEE CODE (OLD): " << std::hex << static_cast<int>(old_licensee_span[0]) << std::endl;
    } else {
        auto new_licensee_str = string_from(span_from(_memory, nlic_range));
        out << "LICENSEE CODE (NEW): " << new_licensee_str << std::endl;
    }

    auto type_id = static_cast<Cartridge::Type>(span_from(_memory, type_range)[0]);
    out << "TYPE: " << std::hex << static_cast<int>(type_id) << ' ' << type_text(type_id) << std::endl;

    out << "ROM-SIZE: " << std::dec << (_rom_size / 1024) << 'k' << std::endl;
    out << "RAM-SIZE: " << std::dec << (_ram_size / 1024) << 'k' << std::endl;
    out.flush();
}


} // namespace bdmg
