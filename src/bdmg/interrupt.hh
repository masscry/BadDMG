#pragma once

#include "common.hh"
#include "device.hh"

namespace bdmg {

using SPInterrupt = std::shared_ptr<class Interrupt>;

constexpr u16 int_enable_addr = 0xFFFF;
constexpr u16 int_flag_addr = 0xFF0F;

constexpr u8 int_vblank_bit = 0x01;
constexpr u8 int_stat_bit   = 0x02;
constexpr u8 int_timer_bit  = 0x04;
constexpr u8 int_serial_bit = 0x08;
constexpr u8 int_joypad_bit = 0x10;

class Interrupt: public IDevice {
    struct OnlySharedPtr { explicit OnlySharedPtr() = default; };
public:
    explicit Interrupt(OnlySharedPtr);
    static SPInterrupt create();

    u8 read_byte(u16 addr) const final;
    void write_byte(u16 addr, u8 data) final;
    std::size_t id() const final;

private:
    u8 _int_enable = 0x00;
    u8 _int_flag = 0xE1;
};

} // namespace bdmg
