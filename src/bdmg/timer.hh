#pragma once

#include "common.hh"
#include "device.hh"

namespace bdmg {

class Bus;

using SPTimer = std::shared_ptr<class Timer>;

constexpr RangeU16 timer_range { 0xFF04, 4 };

class Timer: public ITickingDevice {
protected:
    struct OnlySharedPtr { explicit OnlySharedPtr() = default; };
public:
    explicit Timer(OnlySharedPtr);
    static SPTimer create();

    u8 read_byte(u16 addr) const override;
    void write_byte(u16 addr, u8 value) override;
    void advance(Bus&, u8 cycles) final;

    std::size_t id() const final {
        return DeviceTypeId::timer;
    }

private:
    u16 _div = 0;
    u8 _tima = 0;
    u8 _tma = 0;
    u8 _tac = 0;
    
    u8 _reload_cycles = 0;
};

} // namespace bdmg
