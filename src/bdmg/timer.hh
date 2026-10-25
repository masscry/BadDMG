#pragma once

#include "common.hh"
#include "device.hh"

#include <cstddef>
#include <memory>
#include <utility>

namespace bdmg {

class Bus;

using SPTimer = std::shared_ptr<class TimerImpl>;

constexpr RangeU16 timer_range { 0xFF04, 4 };

using SPDivReset = std::shared_ptr<class IDivReset>;

class IDivReset {
public:
    virtual ~IDivReset() = default;
    virtual void fire() = 0;
};

template<typename Func>
auto make_div_reset_handler(Func&& func) {
    class DivResetImpl final : public IDivReset {
    public:
        DivResetImpl(Func func) : _func(std::move(func)) {}
        void fire() final { _func(); }
    private:
        Func _func;
    };
    return std::make_shared<DivResetImpl>(std::forward<Func>(func));
}

class TimerImpl: public ITickingDevice {
protected:
    struct OnlySharedPtr { explicit OnlySharedPtr() = default; };
public:
    explicit TimerImpl(OnlySharedPtr);
    static SPTimer create();

    u8 read_byte(u16 addr) const override;
    void write_byte(u16 addr, u8 value) override;
    void advance(Bus&, u8 cycles) final;

    std::size_t id() const final {
        return DeviceTypeId::timer;
    }

    void set_div_reset(SPDivReset handler) {
        _handler = handler;
    }

private:
    u16 _div = 0;
    u8 _tima = 0;
    u8 _tma = 0;
    u8 _tac = 0;

    u8 _reload_cycles = 0;

    SPDivReset _handler;
};

} // namespace bdmg
