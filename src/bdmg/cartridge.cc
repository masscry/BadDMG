#include "cartridge.hh"
#include <fstream>
#include <iostream>
#include <format>
#include <utility>

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
        0, 2_KB, 8_KB, 32_KB, 128_KB, 64_KB
    };
    if (size_code >= std::size(size_mapping)) {
        return 0;
    }
    return size_mapping[size_code];
}

namespace {

bool is_mbc5_rumble(Cartridge::Type type) {
    return type == Cartridge::Type::mbc5_rumble ||
           type == Cartridge::Type::mbc5_rumble_ram ||
           type == Cartridge::Type::mbc5_rumble_ram_bat;
}

} // namespace

Cartridge::Cartridge(std::span<const u8> data, MapperClock clock, OnlySharedPtr)
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
    // The mapper masks addresses with _rom_size; pad odd-sized files so those
    // reads stay in bounds.
    if (_memory.size() < _rom_size) {
        _memory.resize(_rom_size, 0xFF);
    }
    _ram_size = calculate_ram_size(_memory[ram_size_range.start]);
    switch (_type) {
    case Type::mbc1_ram:
    case Type::mbc1_ram_bat:
        // Some cartridge headers are broken, we can't trust them
        if (_ram_size == 0) {
            _ram_size = 8_KB;
        }
        break;
    case Type::mbc2:
    case Type::mbc2_bat:
        // The MBC2 die has 512x4 bits of RAM regardless of the header.
        _ram_size = 512;
        break;
    default:
        break;
    }
    if (_ram_size > 0) {
        _ram.resize(_ram_size);
    }

    switch(_type) {
    case Type::rom_only:
        _mapper = std::make_unique<RomOnly>(_memory, _ram, _rom_size);
        break;
    case Type::mbc1:
    case Type::mbc1_ram:
    case Type::mbc1_ram_bat:
        _mapper = std::make_unique<Mbc1>(_memory, _ram, _rom_size);
        break;
    case Type::mbc2:
    case Type::mbc2_bat:
        _mapper = std::make_unique<Mbc2>(_memory, _ram, _rom_size);
        break;
    case Type::mbc3_timer_bat:
    case Type::mbc3_timer_ram_bat:
    case Type::mbc3:
    case Type::mbc3_ram:
    case Type::mbc3_ram_bat:
        _mapper = std::make_unique<Mbc3>(_memory, _ram, _rom_size, std::move(clock));
        break;
    case Type::mbc5:
    case Type::mbc5_ram:
    case Type::mbc5_ram_bat:
    case Type::mbc5_rumble:
    case Type::mbc5_rumble_ram:
    case Type::mbc5_rumble_ram_bat:
        _mapper = std::make_unique<Mbc5>(_memory, _ram, _rom_size, is_mbc5_rumble(_type));
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

SPCartridge Cartridge::create_from_memory(std::span<const u8> data, MapperClock clock) {
    return std::make_shared<Cartridge>(data, std::move(clock), OnlySharedPtr{});
}

u8 Cartridge::read_byte(u16 addr) const {
    return _mapper->read(addr);
}

void Cartridge::write_byte(u16 addr, u8 data) {
    _mapper->write(addr, data);
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
