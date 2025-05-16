#pragma once

#include <array>
#include <stdexcept>
#include <memory>

#include "common.hh"
#include "device.hh"

namespace bdmg {

class AudioError: public std::runtime_error {
    using std::runtime_error::runtime_error;
};

using SPAudio = std::shared_ptr<class Audio>;

constexpr RangeU16 audio_range { 0xFF10, 23};

class Audio : public IDevice {
    struct OnlySharedPtr { explicit OnlySharedPtr() = default; };
public:

    explicit Audio(OnlySharedPtr);
    static SPAudio create();

    u8 read_byte(u16 addr) const final;
    void write_byte(u16, u8) final;

    std::size_t id() const final {
        return DeviceTypeId::audio;
    }

private:
    std::array<u8, audio_range.length> _memory{};
};

} // namespace bdmg
