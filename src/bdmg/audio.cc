#include "audio.hh"
#include "common.hh"

#include <rsmp/rsmp.hh>

#include <format>
#include <memory>
#include <span>

namespace bdmg {

namespace {

constexpr u32 apu_sample_cycles = 4;
constexpr u32 frame_sequencer_hz = 512;

constexpr s8 pulse_duty_table[4][8] = {
    {-1, -1, -1, -1, -1, -1, -1,  1},
    { 1, -1, -1, -1, -1, -1, -1,  1},
    { 1, -1, -1, -1, -1,  1,  1,  1},
    {-1,  1,  1,  1,  1,  1,  1, -1}
};

void clock_length_unit(bool length_enabled, u16& length_timer, bool& chan_enabled) {
    if (length_enabled && length_timer > 0) {
        length_timer--;
        if (length_timer == 0) {
            chan_enabled = false;
        }
    }
}

void clock_envelope_unit(
    bool& enabled, u16& timer, u16& volume, u8 pace, bool dir) {
    if (!enabled) {
        return;
    }
    if (pace == 0) {
        return;
    }

    if (timer > 0) {
        timer--;
    }
    if (timer == 0) {
        timer = pace;

        if (dir) {
            // increase
            if (volume < 15) {
                volume++;
            }
            if (volume >= 15) {
                enabled = false;
            }
        } else {
            // decrease
            if (volume > 0) {
                volume--;
            }
            if (volume == 0) {
                enabled = false;
            }
        }
    }
}

} // namespace

Audio::Audio(OnlySharedPtr, f64 speed)
    : _resampler(rsmp::Config {
        .input_rate = static_cast<rsmp::Number>(dmg_cpu_speed) / apu_sample_cycles * speed,
        .output_rate = static_cast<rsmp::Number>(default_audio_sample_rate),
    }) {
    // Real hardware boots with the APU powered on (NR52 bit 7 = 1).
    _control.byte(2) = 0x80;
}

SPAudio Audio::create(f64 speed) {
    return std::make_shared<Audio>(OnlySharedPtr{}, speed);
}

u8 Audio::read_byte(u16 addr) const {
    if (audio_chan1.has(addr)) {
        return _chan1.read(static_cast<AudioRegister>(addr - audio_chan1.start));
    }
    if (audio_chan2.has(addr)) {
        if (addr == 0xFF15) {
            return 0xFF;
        }
        return _chan2.read(static_cast<AudioRegister>(addr - audio_chan2.start));
    }
    if (audio_chan3.has(addr)) {
        return _chan3.read(static_cast<AudioRegister>(addr - audio_chan3.start));
    }
    if (audio_chan4.has(addr)) {
        return _chan4.read(static_cast<AudioRegister>(addr - audio_chan4.start));
    }
    if (audio_control.has(addr)) {
        auto offset = addr - audio_control.start;
        if (offset == NRX2) {

            u8 val = _control.byte(2) & 0x80;
            val |= _chan1.chan_enabled * 0x01;
            val |= _chan2.chan_enabled * 0x02;
            val |= _chan3.chan_enabled * 0x04;
            val |= _chan4.chan_enabled * 0x08;
            val |= 0x70;
            return val;
        }
        return _control.byte(offset);
    }
    if (audio_waves.has(addr)) {
        if (_chan3.chan_enabled) {
            if (_chan3.sample_countdown != 0) {
                return 0xFF;
            }
            return _waves[((_chan3.wave_index + 1) & 31) >> 1];
        }
        return _waves[addr - audio_waves.start];
    }
    panic(std::format("read byte: {:04x}", addr));
}

void Audio::write_byte(u16 addr, u8 byte) {
    if (audio_control.has(addr)) {
        const auto regId = addr - audio_control.start;
        switch (regId) {
        case NRX0:
        case NRX1:
            if (!_control.audio_enabled) {
                return;
            }
            _control.byte(addr - audio_control.start) = byte;
            return;
        case NRX2: {
            const bool was_off = !_control.audio_enabled;
            _control.byte(addr - audio_control.start) = byte & 0x80;
            if (was_off && _control.audio_enabled) {
                _chan1.reset_sweep_state();
                _div_apu_step = 7;
                _div_apu_cycles = 0;
            } else if (!_control.audio_enabled) {
                cleanup();
            }
            return;
        }
        default:
            panic(std::format("write byte: {:04x} <- {:02x}", addr, byte));
        }
    }
    if (audio_waves.has(addr)) {
        if (_chan3.chan_enabled) {

            if (_chan3.sample_countdown != 0) {
                return;
            }
            _waves[((_chan3.wave_index + 1) & 31) >> 1] = byte;
            return;
        }
        _waves[addr - audio_waves.start] = byte;
        return;
    }
    if (!_control.audio_enabled) {
        if (addr == 0xFF11 || addr == 0xFF16 || addr == 0xFF20) {
            const u8 masked = byte & 0x3F;
            if (addr == 0xFF11) {
                _chan1.write(NRX1, masked);
            } else if (addr == 0xFF16) {
                _chan2.write(NRX1, masked);
            } else {
                _chan4.write(NRX0, masked);
            }
        } else if (addr == 0xFF1B) {
            _chan3.write(NRX1, byte);
        }
        return;
    }

    auto handle_nrx4 = [&](auto& chan, AudioRegister ctrl_reg, u8 byte) {
        bool old_len_enabled = chan.regs.length_enabled;
        u16 length_before_trigger = chan.length_timer;
        bool first_half = (_div_apu_step & 1) == 0;

        if (!old_len_enabled && (byte & 0x40) && first_half && length_before_trigger > 0) {
            chan.length_timer--;
            if (chan.length_timer == 0) {
                chan.chan_enabled = false;
            }
            length_before_trigger = chan.length_timer;
        }

        chan.write(ctrl_reg, byte);

        if ((byte & 0x80) && length_before_trigger == 0 && first_half) {
            chan.clock_length();
        }
    };

    if (audio_chan1.has(addr)) {
        auto reg = static_cast<AudioRegister>(addr - audio_chan1.start);
        if (reg == NRX4) {
            handle_nrx4(_chan1, NRX4, byte);
        } else if (reg == NRX0) {
            if (_chan1.sweep_shadow + _chan1.sweep_completed_addend + 1 > 0x7FF &&
                !(byte & 0x08)) {
                _chan1.chan_enabled = false;
            }
            _chan1.write(reg, byte);
            _chan1.trigger_sweep_calculation();
        } else {
            _chan1.write(reg, byte);
        }
        return;
    }
    if (audio_chan2.has(addr)) {
        if (addr == 0xFF15) {
            return;
        }
        auto reg = static_cast<AudioRegister>(addr - audio_chan2.start);
        if (reg == NRX4) {
            handle_nrx4(_chan2, NRX4, byte);
        } else {
            _chan2.write(reg, byte);
        }
        return;
    }
    if (audio_chan3.has(addr)) {
        auto reg = static_cast<AudioRegister>(addr - audio_chan3.start);
        if (reg == NRX4) {
            if ((byte & 0x80) && _chan3.chan_enabled && _chan3.sample_countdown == 1) {
                const auto offset = ((_chan3.wave_index + 1) >> 1) & 0xF;
                if (offset < 4) {
                    _waves[0] = _waves[offset];
                } else {
                    const auto base = offset & ~3;
                    for (int i = 0; i < 4; ++i) {
                        _waves[i] = _waves[base + i];
                    }
                }
            }
            handle_nrx4(_chan3, NRX4, byte);
        } else {
            _chan3.write(reg, byte);
        }
        return;
    }
    if (audio_chan4.has(addr)) {
        auto reg = static_cast<AudioRegister>(addr - audio_chan4.start);
        if (reg == NRX3) {
            handle_nrx4(_chan4, NRX3, byte);
        } else {
            _chan4.write(reg, byte);
        }
        return;
    }
    panic(std::format("write byte: {:04x} <- {:02x}", addr, byte));
}

void Audio::advance(Bus&, u8 cycles) {
    static constexpr u32 sequencer_step_period = dmg_cpu_speed / frame_sequencer_hz;
    _div_apu_cycles += cycles;
    while (_div_apu_cycles >= sequencer_step_period) {
        _div_apu_cycles -= sequencer_step_period;
        _div_apu_step = (_div_apu_step + 1) & 7;

        switch (_div_apu_step) {
        case 0:
        case 2:
        case 4:
        case 6:
            _chan1.clock_length();
            _chan2.clock_length();
            _chan3.clock_length();
            _chan4.clock_length();
            if (_div_apu_step == 2 || _div_apu_step == 6) {
                _chan1.clock_sweep();
            }
            break;
        case 7:
            _chan1.clock_envelope();
            _chan2.clock_envelope();
            _chan4.clock_envelope();
            break;
        }
    }

    _chan1.advance_sweep(cycles);

    _chan1.advance(cycles);
    _chan2.advance(cycles);
    _chan3.advance(cycles);
    _chan4.advance(cycles);

    auto cur_sample = mix_now();

    const auto resampled = _resampler.push(rsmp::Sample {
        .left = cur_sample.left,
        .right = cur_sample.right,
    });
    if (resampled) {
        const AudioSample out {
            .left = _hp_l.process(resampled->left),
            .right = _hp_r.process(resampled->right),
        };
        if (_sample_callback) {
            _sample_callback(out);
        }
    }
}

void Audio::reset_sequencer() {
    _div_apu_step = 0;
    _div_apu_cycles = 0;
}

AudioSample Audio::mix_now() const {
    if (!_control.audio_enabled) {
        return {};
    }

    auto ch1 = _chan1.sample();
    auto ch2 = _chan2.sample();
    auto ch3 = _chan3.sample(std::span<const u8, audio_waves.length>(_waves));
    auto ch4 = _chan4.sample();

    AudioSample result {};

    result.left += ch1 * _control.ch1_left;
    result.left += ch2 * _control.ch2_left;
    result.left += ch3 * _control.ch3_left;
    result.left += ch4 * _control.ch4_left;
    result.left *= (static_cast<f32>(_control.vol_left) + 1.0f) / 8.0f / 4.0f;

    result.right += ch1 * _control.ch1_right;
    result.right += ch2 * _control.ch2_right;
    result.right += ch3 * _control.ch3_right;
    result.right += ch4 * _control.ch4_right;
    result.right *= (static_cast<f32>(_control.vol_right) + 1.0f) / 8.0f / 4.0f;

    return result;
}

void Audio::cleanup() {
    _chan1.cleanup();
    _chan2.cleanup();
    _chan3.cleanup();
    _chan4.cleanup();
    _control = {};
}

void PulseChannel::trigger() {
    const bool was_active = this->chan_enabled;

    if (this->dac_enabled) {
        this->chan_enabled = true;
    }

    if (this->length_timer == 0) {
        this->length_timer = 64;
    }

    u16 period = (this->regs.period_high << 8) | this->regs.period_low;
    this->duty_timer = (2048 - period) * 4;

    this->envelope_volume = this->regs.init_volume;
    this->envelope_timer = this->regs.sweep_pace;
    this->envelope_enabled = true;

    this->sweep_freq = period;

    this->sweep_instant_done = false;
    this->sweep_shadow = 0;
    this->sweep_completed_addend = 0;
    if (this->regs.step > 0) {
        this->sweep_calculate_countdown = this->regs.step;
        this->sweep_calc_reload_timer = 3;
        if (!was_active) {
            this->sweep_calc_reload_timer++;
        }
        this->sweep_unshifted = false;
        this->sweep_addend = this->sweep_freq >> this->regs.step;
    } else {
        this->sweep_addend = 0;
    }
    this->restart_hold = 2;
    this->sweep_clock_countdown = this->regs.pace ^ 7;
}

u8 PulseChannel::read(AudioRegister reg) const {
    switch (reg) {
    case NRX0:
        return this->regs.byte(0) | 0x80;
    case NRX1:
        return this->regs.byte(1) | 0x3F;
    case NRX2:
        return this->regs.byte(2);
    case NRX3:
        return 0xFF;
    case NRX4:
        return this->regs.byte(4) | 0xBF;
    default:
        panic("address out of audio registers");
    }
}

void PulseChannel::write(AudioRegister reg, u8 data) {
    this->regs.byte(reg) = data;

    switch (reg) {
    case NRX0:
        return;
    case NRX1:
        this->length_timer = 64 - this->regs.init_length;
        return;
    case NRX2:
        this->dac_enabled = (data & 0xF8) != 0;
        if (!this->dac_enabled) {
            this->chan_enabled = false;
        }
        return;
    case NRX3:
        return;
    case NRX4:
        if (this->regs.trigger) {
            trigger();
        }
        return;
    default:
        panic("address out of audio registers");
    }
}

void PulseChannel::clock_length() {
    clock_length_unit(this->regs.length_enabled, this->length_timer, this->chan_enabled);
}

void PulseChannel::clock_sweep() {
    this->sweep_clock_countdown = (this->sweep_clock_countdown + 1) & 7;
    this->trigger_sweep_calculation();
}

void PulseChannel::trigger_sweep_calculation() {
    if ((this->regs.byte(0) & 0x70) && this->sweep_clock_countdown == 7) {
        if (this->regs.step > 0) {
            this->sweep_freq = this->sweep_addend + this->sweep_shadow + (this->regs.dir ? 1 : 0);
            this->sweep_freq &= 0x7FF;
            this->regs.period_low = this->sweep_freq & 0xFF;
            this->regs.period_high = (this->sweep_freq >> 8) & 0x07;
        }
        if (this->restart_hold == 0) {
            this->sweep_addend = this->sweep_freq;
            this->sweep_addend >>= this->regs.step;
        }
        this->sweep_calculate_countdown = this->regs.step;
        this->sweep_calc_reload_timer = 1;
        this->sweep_unshifted = (this->regs.step == 0);
        this->sweep_clock_countdown = this->regs.pace ^ 7;
        if (this->sweep_calculate_countdown == 0) {
            this->sweep_instant_done = true;
        }
    }
}

void PulseChannel::sweep_calculation_done() {
    if (this->restart_hold == 0) {
        this->sweep_shadow = this->sweep_freq;
    }
    if (this->regs.dir) {
        this->sweep_addend ^= 0x7FF;
    }
    if (this->sweep_shadow + this->sweep_addend > 0x7FF && !this->regs.dir) {
        this->chan_enabled = false;
    }
    this->sweep_completed_addend = this->sweep_addend;
}

void PulseChannel::reset_sweep_state() {
    this->sweep_shadow = 0;
    this->sweep_addend = 0;
    this->sweep_completed_addend = 0;
    this->sweep_calculate_countdown = 0;
    this->sweep_calc_reload_timer = 0;
    this->sweep_clock_countdown = 0;
    this->restart_hold = 0;
    this->sweep_instant_done = false;
    this->sweep_unshifted = false;
}

void PulseChannel::advance_sweep(u8 cycles) {
    u8 sweep_cycles = cycles / 2;

    if (this->sweep_calc_reload_timer > sweep_cycles) {
        this->sweep_calc_reload_timer -= sweep_cycles;
        sweep_cycles = 0;
    } else {
        if (this->sweep_calc_reload_timer != 0 &&
            this->sweep_calculate_countdown == 0 &&
            this->sweep_instant_done) {
            this->sweep_calculation_done();
        }
        this->sweep_instant_done = false;
        sweep_cycles -= this->sweep_calc_reload_timer;
        this->sweep_calc_reload_timer = 0;
    }

    if (this->sweep_calculate_countdown != 0 && (this->regs.step != 0 || this->sweep_unshifted)) {
        if (this->sweep_calculate_countdown > sweep_cycles) {
            this->sweep_calculate_countdown -= sweep_cycles;
        } else {
            this->sweep_calculate_countdown = 0;
            this->sweep_calculation_done();
        }
    }

    if (this->restart_hold != 0) {
        if (this->restart_hold > cycles) {
            this->restart_hold -= cycles;
        } else {
            this->restart_hold = 0;
        }
    }
}

void PulseChannel::clock_envelope() {
    clock_envelope_unit(
        this->envelope_enabled,
        this->envelope_timer,
        this->envelope_volume,
        this->regs.sweep_pace,
        this->regs.env_dir);
}

void PulseChannel::advance(u8 cycles) {
    if (!this->chan_enabled) {
        return;
    }

    if (this->duty_timer > cycles) {
        this->duty_timer -= cycles;
        return;
    }

    u8 left = cycles - duty_timer;

    auto period = (this->regs.period_high << 8) | (this->regs.period_low);
    this->duty_timer = ((2048 - period) * 4) - left;
    this->duty_step = (this->duty_step + 1) % 8;
}

f32 PulseChannel::sample() const {
    if (!this->dac_enabled) {
        return 0.0f;
    }

    const bool high = this->chan_enabled && this->length_timer != 0 &&
        pulse_duty_table[this->regs.wave_duty][this->duty_step] > 0;
    return dac_level(high ? static_cast<u8>(this->envelope_volume) : 0);
}

void PulseChannel::cleanup() {
    this->regs = {};
    this->duty_timer = 0;
    this->duty_step = 0;
    this->envelope_timer = 0;
    this->envelope_volume = 0;
    this->sweep_freq = 0;
    this->sweep_shadow = 0;
    this->sweep_addend = 0;
    this->sweep_completed_addend = 0;
    this->sweep_calculate_countdown = 0;
    this->sweep_calc_reload_timer = 0;
    this->sweep_clock_countdown = 0;
    this->restart_hold = 0;
    this->sweep_instant_done = false;
    this->sweep_unshifted = false;
    this->envelope_enabled = false;
    this->chan_enabled = false;
    this->dac_enabled = false;
}

u16 wave_period(const WaveChannelRegisters& regs) {
    return static_cast<u16>((regs.period_high << 8) | regs.period_low);
}

u16 wave_reload(u16 period) {
    return static_cast<u16>(2047 - period);
}

void WaveChannel::trigger() {
    this->chan_enabled = this->regs.dac_enabled;

    if (this->length_timer == 0) {
        this->length_timer = 256;
    }

    this->sample_countdown = wave_reload(wave_period(this->regs)) + 4;

    this->wave_index = 0;
}

u8 WaveChannel::read(AudioRegister addr) const {
    switch (addr) {
    case NRX0:
        return this->regs.byte(0) | 0x7F;
    case NRX1:
        return 0xFF;
    case NRX2:
        return this->regs.byte(2) | 0x9F;
    case NRX3:
        return 0xFF;
    case NRX4:
        return this->regs.byte(4) | 0xBF;
    default:
        panic("address out of audio registers");
    }
}

void WaveChannel::write(AudioRegister addr, u8 data) {
    this->regs.byte(addr) = data;
    switch (addr) {
    case NRX0:
        if (!this->regs.dac_enabled) {
            this->chan_enabled = false;
        }
        return;
    case NRX1:
        this->length_timer = 256 - this->regs.init_length;
        return;
    case NRX2:
        return;
    case NRX3:
        return;
    case NRX4:
        if (this->regs.trigger) {
            trigger();
        }
        return;
    default:
        panic("address out of audio registers");
    }
}

void WaveChannel::clock_length() {
    clock_length_unit(this->regs.length_enabled, this->length_timer, this->chan_enabled);
}

void WaveChannel::advance(u8 cycles) {
    if (!this->chan_enabled) {
        return;
    }

    for (u8 tick = 0; tick < cycles / 2; ++tick) {
        if (this->sample_countdown == 0) {
            this->sample_countdown = wave_reload(wave_period(this->regs));
            this->wave_index = (this->wave_index + 1) & 31;
        } else {
            this->sample_countdown--;
        }
    }
}

f32 WaveChannel::sample(std::span<const u8, audio_waves.length> wave_table) const {
    if (!this->regs.dac_enabled) {
        return 0.0f;
    }

    static constexpr u8 shifts[4] = {4, 0, 1, 2};
    const u8 shift = shifts[this->regs.output_level & 0x03];

    const bool playing = this->chan_enabled &&
        !(this->length_timer == 0 && this->regs.length_enabled);
    u8 value = 0;
    if (playing) {
        const u8 byte_index = this->wave_index / 2;
        const u8 nibble = (this->wave_index % 2 == 0)
            ? (wave_table[byte_index] >> 4) & 0xF
            : wave_table[byte_index] & 0xF;
        value = nibble >> shift;
    }
    return dac_level(value);
}

void WaveChannel::cleanup() {
    this->regs = {};
    this->sample_countdown = 0;
    this->wave_index = 0;
    this->chan_enabled = false;
}

void NoiseChannel::trigger() {
    if (this->dac_enabled) {
        this->chan_enabled = true;
    }

    if (this->length_timer == 0) {
        this->length_timer = 64;
    }

    this->envelope_volume = this->regs.init_volume;
    this->envelope_timer = this->regs.sweep_pace;
    this->envelope_enabled = true;

    this->lfsr = 0;
}

u8 NoiseChannel::read(AudioRegister addr) const {
    switch (addr) {
    case NRX0:
        return 0xFF;
    case NRX1:
        return this->regs.byte(1);
    case NRX2:
        return this->regs.byte(2);
    case NRX3:
        return this->regs.byte(3) | 0xBF;
    default:
        panic("address out of audio registers");
    }
}

void NoiseChannel::write(AudioRegister addr, u8 data) {
    this->regs.byte(addr) = data;
    switch (addr) {
    case NRX0:
        this->length_timer = 64 - this->regs.init_length;
        return;
    case NRX1:
        this->dac_enabled = (data & 0xF8) != 0;
        if (!this->dac_enabled) {
            this->chan_enabled = false;
        }
        return;
    case NRX2:
        return;
    case NRX3:
        if (this->regs.trigger) {
            trigger();
        }
        return;
    default:
        panic("address out of audio registers");
    }
}

void NoiseChannel::clock_length() {
    clock_length_unit(this->regs.length_enabled, this->length_timer, this->chan_enabled);
}

void NoiseChannel::clock_envelope() {
    clock_envelope_unit(
        this->envelope_enabled,
        this->envelope_timer,
        this->envelope_volume,
        this->regs.sweep_pace,
        this->regs.env_dir);
}

void NoiseChannel::advance(u8 cycles) {
    if (!this->chan_enabled) {
        return;
    }

    if (this->regs.clock_shift >= 14) {
        return;
    }

    u32 period = this->regs.clock_divider == 0
        ? 8u
        : static_cast<u32>(this->regs.clock_divider) * 16u;
    period <<= this->regs.clock_shift;

    this->lfsr_cycles += cycles;
    while (this->lfsr_cycles >= period) {
        this->lfsr_cycles -= period;
        shift_lfsr();
    }
}

void NoiseChannel::shift_lfsr() {
    bool bit0 = (this->lfsr & 1) != 0;
    bool bit1 = (this->lfsr & 2) != 0;
    bool feedback = !(bit0 ^ bit1);

    this->lfsr >>= 1;

    if (this->regs.lfsr_width == 0) {
        if (feedback) {
            this->lfsr |= 0x4000;
        }
    } else {
        this->lfsr = (feedback ? 0x0040 : 0) | (this->lfsr & 0x003F);
    }
}

f32 NoiseChannel::sample() const {
    if (!this->dac_enabled) {
        return 0.0f;
    }

    const bool high = this->chan_enabled && this->length_timer != 0 &&
        (this->lfsr & 1) != 0;
    return dac_level(high ? static_cast<u8>(this->envelope_volume) : 0);
}

void NoiseChannel::cleanup() {
    this->regs = {};
    this->envelope_timer = 0;
    this->envelope_volume = 0;
    this->lfsr = 0;
    this->lfsr_cycles = 0;
    this->envelope_enabled = false;
    this->chan_enabled = false;
    this->dac_enabled = false;
}

} // namespace bdmg
