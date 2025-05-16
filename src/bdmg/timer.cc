#include "timer.hh"
#include "bus.hh"
#include "interrupt.hh"

namespace bdmg {

Timer::Timer(OnlySharedPtr) 
    : _div(0), _tima(0), _tma(0), _tac(0), _reload_cycles(0) {}

SPTimer Timer::create() {
    return std::make_shared<Timer>(OnlySharedPtr{});
}

constexpr auto div_addr = RangeU16{ 0xFF04, 1 };
constexpr auto tima_addr = RangeU16{ 0xFF05, 1 };
constexpr auto tma_addr = RangeU16{ 0xFF06, 1 };
constexpr auto tac_addr = RangeU16{ 0xFF07, 1 };

u8 Timer::read_byte(u16 addr) const {
    switch(addr) {
    case div_addr.start: return _div >> 8;
    case tima_addr.start: return _tima;
    case tma_addr.start: return _tma;
    case tac_addr.start: return _tac;
    default:     return 0xFF;
    }
}

void Timer::write_byte(u16 addr, u8 value) {
    switch(addr) {
    case div_addr.start:
        _div = 0;
        break;
    case tima_addr.start:
        if (_reload_cycles == 0){
            _tima = value;
        }
        break;
    case tma_addr.start:
        _tma = value;
        break;
    case tac_addr.start:
        _tac = value & 0x07;
        break;
    }
}

void Timer::advance(Bus& bus, u8 cycles) {
    if (_reload_cycles > 0) {
        if (cycles >= _reload_cycles) {
            _tima = _tma;
            bus.write_byte(int_flag_addr, bus.read_byte(int_flag_addr) | int_timer_bit);
            _reload_cycles = 0;
        } else {
            _reload_cycles -= cycles;
        }
    }

    const u16 prev_div = _div;
    _div += cycles;

    if (!(_tac & 0x04)){
        return;
    }

    static constexpr u8 div_bits[] = {9, 3, 5, 7};
    const u8 bit = div_bits[_tac & 0x03];
    const u16 mask = 1 << bit;

    for (u16 i = 1; i <= cycles; ++i) {
        const u16 old_div = prev_div + i - 1;
        const u16 new_div = prev_div + i;
        
        if ((old_div & mask) && !(new_div & mask)) {
            if (++_tima == 0x00) {
                _reload_cycles = 4;
            }
        }
    }
}

} // namespace bdmg
