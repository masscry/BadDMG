#pragma once

#include "common.hh"

#include <chrono>
#include <functional>
#include <span>

namespace bdmg {

using MapperClock = std::function<std::chrono::system_clock::time_point()>;

class Mapper {
public:
    Mapper(std::span<const u8> rom, std::span<u8> ram, u32 rom_size);
    virtual ~Mapper() = default;

    virtual u8 read(u16 addr) const = 0;
    virtual void write(u16 addr, u8 data) = 0;

protected:
    u8 read_rom(u16 bank, u16 offset) const;
    u8 read_ram(u16 offset) const;
    void write_ram(u16 offset, u8 data);

    std::span<const u8> _rom;
    std::span<u8> _ram;
    u32 _rom_mask;
    bool _ram_enabled { false };
    u16 _ram_bank { 0 };
    u16 _rom_bank { 1 };
};

class RomOnly final : public Mapper {
public:
    using Mapper::Mapper;

    u8 read(u16 addr) const final;
    void write(u16, u8) final;
};

class Mbc1 final : public Mapper {
public:
    using Mapper::Mapper;

    u8 read(u16 addr) const final;
    void write(u16 addr, u8 data) final;

private:
    u8 _bank1 { 1 };
    u8 _bank2 { 0 };
    bool _mode { false };
};

class Mbc2 final : public Mapper {
public:
    using Mapper::Mapper;

    u8 read(u16 addr) const final;
    void write(u16 addr, u8 data) final;
};

constexpr std::size_t rtc_save_size = 24;

class Mbc3 final : public Mapper {
public:
    Mbc3(std::span<const u8> rom, std::span<u8> ram, u32 rom_size, MapperClock clock,
         std::span<u8> rtc_save = {});

    u8 read(u16 addr) const final;
    void write(u16 addr, u8 data) final;

private:
    struct Rtc {
        u8 sec { 0 };
        u8 min { 0 };
        u8 hour { 0 };
        u16 days { 0 };
        bool halt { false };
        bool carry { false };
    };

    auto now() const -> std::chrono::system_clock::time_point;
    void sync();
    void latch();
    void load_rtc();
    void persist();
    u8 read_rtc() const;
    void write_rtc(u8 data);

    MapperClock _clock;
    std::chrono::system_clock::time_point _anchor;
    Rtc _live;
    Rtc _latched;
    u8 _latch_prev { 0 };
    std::span<u8> _rtc_save;
};

class Mbc5 final : public Mapper {
public:
    Mbc5(std::span<const u8> rom, std::span<u8> ram, u32 rom_size, bool rumble);

    u8 read(u16 addr) const final;
    void write(u16 addr, u8 data) final;

private:
    bool _rumble;
};

} // namespace bdmg
