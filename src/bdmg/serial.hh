#pragma once

#include "common.hh"
#include "device.hh"

namespace bdmg {

using SPSerial = std::shared_ptr<class Serial>;

class Serial : public ITickingDevice {
public:
    static SPSerial create();
    explicit Serial();

    u8 read_byte(u16 addr) const override;
    void write_byte(u16 addr, u8 data) override;
    void advance(Bus&, u8 cycles) final;

    std::size_t id() const final {
        return DeviceTypeId::serial;
    }

private:
    u8 _sb{0xFF};
    u8 _sc{0x7E};
    bool _transfer_in_progress{false};
    int _remaining_cycles{0};
};

constexpr RangeU16 serial_range {.start = 0xFF01, .length = 2};

} // namespace bdmg
