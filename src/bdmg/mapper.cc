#include "mapper.hh"

#include <utility>

namespace bdmg {

Mapper::Mapper(std::span<const u8> rom, std::span<u8> ram, u32 rom_size)
    : _rom(rom), _ram(ram), _rom_mask(rom_size - 1) {
}

u8 Mapper::read_rom(u16 bank, u16 offset) const {
    return _rom[((static_cast<u32>(bank) << 14) | offset) & _rom_mask];
}

u8 Mapper::read_ram(u16 offset) const {
    if (!_ram_enabled || _ram.empty()) {
        return 0xFF;
    }
    return _ram[((static_cast<u32>(_ram_bank) << 13) | offset) % _ram.size()];
}

void Mapper::write_ram(u16 offset, u8 data) {
    if (!_ram_enabled || _ram.empty()) {
        return;
    }
    _ram[((static_cast<u32>(_ram_bank) << 13) | offset) % _ram.size()] = data;
}

u8 RomOnly::read(u16 addr) const {
    if (addr < 0x4000) {
        return read_rom(0, addr);
    }
    if (addr < 0x8000) {
        return read_rom(1, addr & 0x3FFF);
    }
    return 0xFF;
}

void RomOnly::write(u16, u8) {
}

u8 Mbc1::read(u16 addr) const {
    if (addr < 0x4000) {
        const u16 bank = _mode ? static_cast<u16>(_bank2 << 5) : 0;
        return read_rom(bank, addr);
    }
    if (addr < 0x8000) {
        return read_rom(static_cast<u16>((_bank2 << 5) | _bank1), addr & 0x3FFF);
    }
    if (addr >= 0xA000 && addr < 0xC000) {
        return read_ram(addr & 0x1FFF);
    }
    return 0xFF;
}

void Mbc1::write(u16 addr, u8 data) {
    if (addr < 0x2000) {
        _ram_enabled = (data & 0x0F) == 0x0A;
    } else if (addr < 0x4000) {
        _bank1 = data & 0x1F;
        if (_bank1 == 0) {
            _bank1 = 1;
        }
    } else if (addr < 0x6000) {
        _bank2 = data & 0x03;
        _ram_bank = _mode ? _bank2 : 0;
    } else if (addr < 0x8000) {
        _mode = (data & 0x01) != 0;
        _ram_bank = _mode ? _bank2 : 0;
    } else if (addr >= 0xA000 && addr < 0xC000) {
        write_ram(addr & 0x1FFF, data);
    }
}

u8 Mbc2::read(u16 addr) const {
    if (addr < 0x4000) {
        return read_rom(0, addr);
    }
    if (addr < 0x8000) {
        return read_rom(_rom_bank, addr & 0x3FFF);
    }
    if (addr >= 0xA000 && addr < 0xC000) {
        return static_cast<u8>(0xF0 | (read_ram(addr & 0x1FFF) & 0x0F));
    }
    return 0xFF;
}

void Mbc2::write(u16 addr, u8 data) {
    if (addr < 0x4000) {
        if (addr & 0x0100) {
            _rom_bank = data & 0x0F;
            if (_rom_bank == 0) {
                _rom_bank = 1;
            }
        } else {
            _ram_enabled = (data & 0x0F) == 0x0A;
        }
    } else if (addr >= 0xA000 && addr < 0xC000) {
        write_ram(addr & 0x1FFF, data & 0x0F);
    }
}

Mbc3::Mbc3(std::span<const u8> rom, std::span<u8> ram, u32 rom_size, MapperClock clock)
    : Mapper(rom, ram, rom_size),
      _clock(std::move(clock)),
      _anchor(_clock ? _clock() : std::chrono::system_clock::now()) {
}

auto Mbc3::now() const -> std::chrono::system_clock::time_point {
    return _clock ? _clock() : std::chrono::system_clock::now();
}

void Mbc3::sync() {
    if (_live.halt) {
        _anchor = now();
        return;
    }

    const auto current = now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current - _anchor);
    if (elapsed.count() <= 0) {
        return;
    }
    _anchor += elapsed;

    u64 total = static_cast<u64>(_live.sec) + static_cast<u64>(elapsed.count());
    _live.sec = static_cast<u8>(total % 60);
    total /= 60;
    total += _live.min;
    _live.min = static_cast<u8>(total % 60);
    total /= 60;
    total += _live.hour;
    _live.hour = static_cast<u8>(total % 24);
    total /= 24;
    total += _live.days;
    if (total > 0x1FF) {
        _live.carry = true;
        total &= 0x1FF;
    }
    _live.days = static_cast<u16>(total);
}

void Mbc3::latch() {
    sync();
    _latched = _live;
}

u8 Mbc3::read_rtc() const {
    switch (_ram_bank) {
    case 0x08: return _latched.sec;
    case 0x09: return _latched.min;
    case 0x0A: return _latched.hour;
    case 0x0B: return static_cast<u8>(_latched.days & 0xFF);
    case 0x0C:
        return static_cast<u8>((( _latched.days >> 8) & 0x01) |
                               (_latched.halt ? 0x40 : 0x00) |
                               (_latched.carry ? 0x80 : 0x00));
    default: return 0xFF;
    }
}

void Mbc3::write_rtc(u8 data) {
    sync();
    switch (_ram_bank) {
    case 0x08: _live.sec = data; break;
    case 0x09: _live.min = data; break;
    case 0x0A: _live.hour = data; break;
    case 0x0B:
        _live.days = static_cast<u16>((_live.days & 0x100) | data);
        break;
    case 0x0C:
        _live.days = static_cast<u16>((_live.days & 0xFF) | ((data & 0x01) << 8));
        _live.halt = (data & 0x40) != 0;
        _live.carry = (data & 0x80) != 0;
        _anchor = now();
        break;
    default: break;
    }
    _latched = _live;
}

u8 Mbc3::read(u16 addr) const {
    if (addr < 0x4000) {
        return read_rom(0, addr);
    }
    if (addr < 0x8000) {
        return read_rom(_rom_bank, addr & 0x3FFF);
    }
    if (addr >= 0xA000 && addr < 0xC000) {
        if (_ram_bank <= 0x07) {
            return read_ram(addr & 0x1FFF);
        }
        if (_ram_enabled && _ram_bank >= 0x08 && _ram_bank <= 0x0C) {
            return read_rtc();
        }
    }
    return 0xFF;
}

void Mbc3::write(u16 addr, u8 data) {
    if (addr < 0x2000) {
        _ram_enabled = (data & 0x0F) == 0x0A;
    } else if (addr < 0x4000) {
        _rom_bank = data & 0x7F;
        if (_rom_bank == 0) {
            _rom_bank = 1;
        }
    } else if (addr < 0x6000) {
        _ram_bank = data;
    } else if (addr < 0x8000) {
        if (_latch_prev == 0x00 && data == 0x01) {
            latch();
        }
        _latch_prev = data;
    } else if (addr >= 0xA000 && addr < 0xC000) {
        if (_ram_bank <= 0x07) {
            write_ram(addr & 0x1FFF, data);
        } else if (_ram_enabled && _ram_bank >= 0x08 && _ram_bank <= 0x0C) {
            write_rtc(data);
        }
    }
}

Mbc5::Mbc5(std::span<const u8> rom, std::span<u8> ram, u32 rom_size, bool rumble)
    : Mapper(rom, ram, rom_size), _rumble(rumble) {
}

u8 Mbc5::read(u16 addr) const {
    if (addr < 0x4000) {
        return read_rom(0, addr);
    }
    if (addr < 0x8000) {
        return read_rom(_rom_bank, addr & 0x3FFF);
    }
    if (addr >= 0xA000 && addr < 0xC000) {
        return read_ram(addr & 0x1FFF);
    }
    return 0xFF;
}

void Mbc5::write(u16 addr, u8 data) {
    if (addr < 0x2000) {
        _ram_enabled = (data & 0x0F) == 0x0A;
    } else if (addr < 0x3000) {
        _rom_bank = static_cast<u16>((_rom_bank & 0x100) | data);
    } else if (addr < 0x4000) {
        _rom_bank = static_cast<u16>((_rom_bank & 0xFF) | ((data & 0x01) << 8));
    } else if (addr < 0x6000) {
        _ram_bank = data & (_rumble ? 0x07 : 0x0F);
    } else if (addr >= 0xA000 && addr < 0xC000) {
        write_ram(addr & 0x1FFF, data);
    }
}

} // namespace bdmg
