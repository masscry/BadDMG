#include "audio.hh"

namespace bdmg {

Audio::Audio(OnlySharedPtr) {
    _memory[0xFF26 - audio_range.start] = 0x70;
}

SPAudio Audio::create() {
    return std::make_shared<Audio>(OnlySharedPtr{});
}

u8 Audio::read_byte(u16 addr) const {
    if (audio_range.has(addr)) {
        return _memory[addr - audio_range.start];
    }
    panic("address out of audio registers");
}

void Audio::write_byte(u16 addr, u8 data) {
    if (audio_range.has(addr)) {
        _memory[addr - audio_range.start] = data;
        return;
    }
    panic("address out of audio registers");
}

} // namespace bdmg
