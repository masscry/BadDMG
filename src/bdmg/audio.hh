#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <numbers>
#include <span>
#include <utility>

#include "common.hh"
#include "device.hh"

#include <rsmp/rsmp.hh>

namespace bdmg {

struct AudioSample {
    f64 left = 0.0;
    f64 right = 0.0;
};

constexpr u32 default_audio_sample_rate = 48000;

using SPAudio = std::shared_ptr<class Audio>;

constexpr RangeU16 audio_chan1 { .start=0xFF10, .length=5  };
constexpr RangeU16 audio_chan2 { .start=0xFF15, .length=5  };
constexpr RangeU16 audio_chan3 { .start=0xFF1A, .length=5  };
constexpr RangeU16 audio_chan4 { .start=0xFF20, .length=4  };
constexpr RangeU16 audio_control { .start=0xFF24, .length=3  };
constexpr RangeU16 audio_waves { .start=0xFF30, .length=16 };

enum AudioRegister {
    NRX0 = 0,
    NRX1 = 1,
    NRX2 = 2,
    NRX3 = 3,
    NRX4 = 4,
};

// Register banks, one byte per hardware register. Bitfield layout is
// LSB-first, which matches the hardware on GCC/Clang; byte() is the raw byte.
struct GlobalControlRegisters {
    // NR50 0xFF24
    u8 vol_right: 3;
    u8 vin_right: 1;
    u8 vol_left: 3;
    u8 vin_left: 1;

    // NR51 0xFF25
    u8 ch1_right: 1;
    u8 ch2_right: 1;
    u8 ch3_right: 1;
    u8 ch4_right: 1;
    u8 ch1_left: 1;
    u8 ch2_left: 1;
    u8 ch3_left: 1;
    u8 ch4_left: 1;

    // NR52 0xFF26
    u8 ch1_enabled: 1;
    u8 ch2_enabled: 1;
    u8 ch3_enabled: 1;
    u8 ch4_enabled: 1;
    u8 _unused1: 3;
    u8 audio_enabled: 1;

    u8 byte(int i) const { return reinterpret_cast<const u8*>(this)[i]; }
    u8& byte(int i) { return reinterpret_cast<u8*>(this)[i]; }
};

struct PulseChannelRegisters {
    // NR10 0xFF10
    u8 step: 3;
    u8 dir: 1;
    u8 pace: 3;
    u8 _unused1: 1;

    // NR11 0xFF11 NR21 0xFF16
    u8 init_length: 6;
    u8 wave_duty: 2;

    // NR12 0xFF12 NR22 0xFF17
    u8 sweep_pace: 3;
    u8 env_dir: 1;
    u8 init_volume: 4;

    // NR13 0xFF13 NR23 0xFF18
    u8 period_low;

    // NR14 0xFF14 NR24 0xFF19
    u8 period_high: 3;
    u8 _unused2: 3;
    u8 length_enabled: 1;
    u8 trigger: 1;

    u8 byte(int i) const { return reinterpret_cast<const u8*>(this)[i]; }
    u8& byte(int i) { return reinterpret_cast<u8*>(this)[i]; }
};

struct WaveChannelRegisters {
    // NR30 0xFF1A
    u8 _unused1: 7;
    u8 dac_enabled: 1;

    // NR31 0xFF1B
    u8 init_length;

    // NR32 0xFF1C
    u8 _unused2: 5;
    u8 output_level: 2;
    u8 _unused3: 1;

    // NR33 0xFF1D
    u8 period_low;

    // NR34 0xFF1E
    u8 period_high: 3;
    u8 _unused4: 3;
    u8 length_enabled: 1;
    u8 trigger: 1;

    u8 byte(int i) const { return reinterpret_cast<const u8*>(this)[i]; }
    u8& byte(int i) { return reinterpret_cast<u8*>(this)[i]; }
};

struct NoiseChannelRegisters {
    // NR41 0xFF20
    u8 init_length: 6;
    u8 _unused1: 2;

    // NR42 0xFF21
    u8 sweep_pace: 3;
    u8 env_dir: 1;
    u8 init_volume: 4;

    // NR43 0xFF22
    u8 clock_divider: 3;
    u8 lfsr_width: 1;
    u8 clock_shift: 4;

    // NR44 0xFF23
    u8 _unused2: 6;
    u8 length_enabled: 1;
    u8 trigger: 1;

    u8 byte(int i) const { return reinterpret_cast<const u8*>(this)[i]; }
    u8& byte(int i) { return reinterpret_cast<u8*>(this)[i]; }
};

struct DcBlocker {
    static constexpr f64 cutoff_hz = 30.0;
    static constexpr f64 kR =
        1.0 - 2.0 * std::numbers::pi * cutoff_hz / default_audio_sample_rate;

    f64 x_prev = 0.0;
    f64 y_prev = 0.0;

    f64 process(f64 x) {
        const f64 y = x - x_prev + kR * y_prev;
        x_prev = x;
        y_prev = y;
        return y;
    }
};

inline f64 dac_level(u8 value) {
    constexpr f64 dac_steps = 15.0;
    return 1.0 - 2.0 * static_cast<f64>(value) / dac_steps;
}

struct PulseChannel {
    PulseChannelRegisters regs {};

    u16 duty_timer = 0;
    u16 duty_step = 0;

    u16 length_timer = 0;

    u16 envelope_timer = 0;
    u16 envelope_volume = 0;

    bool envelope_enabled = false;
    bool chan_enabled = false;
    bool dac_enabled = false;

    u16 sweep_freq = 0;                
    u16 sweep_shadow = 0;
    u16 sweep_addend = 0;    
    u16 sweep_completed_addend = 0;  
    u8 sweep_calculate_countdown = 0;
    u8 sweep_calc_reload_timer = 0;
    u8 sweep_clock_countdown = 0;
    u8 restart_hold = 0;
    bool sweep_instant_done = false;
    bool sweep_unshifted = false;

    u8 read(AudioRegister) const;
    void write(AudioRegister, u8);
    void advance(u8 cycles);

    /// The sweep's 1 MHz-tick machinery; runs regardless of the channel state.
    void advance_sweep(u8 cycles);

    void clock_length();
    void clock_sweep();
    void clock_envelope();

    /// Runs on each 128 Hz sweep tick and after NR10 writes.
    void trigger_sweep_calculation();

    void reset_sweep_state();

    f32 sample() const;

    void cleanup();

private:
    void trigger();
    void sweep_calculation_done();
};

struct WaveChannel {
    WaveChannelRegisters regs {};

    u16 length_timer = 0;

    /// 2 MHz ticks until the channel next reads a wave RAM byte.
    s32 sample_countdown = 0;

    u8 wave_index = 0;
    bool chan_enabled = false;

    u8 read(AudioRegister addr) const;
    void write(AudioRegister, u8);
    void advance(u8 cycles);

    void clock_length();

    f32 sample(std::span<const u8, audio_waves.length> wave_table) const;

    void cleanup();

private:
    void trigger();
};

struct NoiseChannel {
    NoiseChannelRegisters regs {};

    u16 length_timer = 0;
    u16 envelope_timer = 0;
    u16 envelope_volume = 0;
    u16 lfsr = 0;
    u32 lfsr_cycles = 0;
    bool envelope_enabled = false;
    bool chan_enabled = false;
    bool dac_enabled = false;

    u8 read(AudioRegister addr) const;
    void write(AudioRegister, u8);
    void advance(u8 cycles);

    void clock_length();
    void clock_envelope();

    f32 sample() const;

    void cleanup();

private:
    void trigger();
    void shift_lfsr();
};

class Audio : public ITickingDevice {
    struct OnlySharedPtr { explicit OnlySharedPtr() = default; };
public:

    explicit Audio(OnlySharedPtr, f64 speed);
    static SPAudio create(f64 speed = 1.0);

    u8 read_byte(u16 addr) const final;
    void write_byte(u16, u8) final;

    void advance(Bus&, u8 cycles) final;

    std::size_t id() const final {
        return DeviceTypeId::audio;
    }

    /// Called for every resampled sample, from inside advance(): the frontend
    /// buffers and paces the output as it needs.
    using SampleCallback = std::function<void(const AudioSample&)>;
    void set_sample_callback(SampleCallback callback) {
        _sample_callback = std::move(callback);
    }

    /// Call when DIV (0xFF04) is written.
    void reset_sequencer();

private:

    void cleanup();

    AudioSample mix_now() const;

    rsmp::Resampler _resampler;
    DcBlocker _hp_l {};
    DcBlocker _hp_r {};

    SampleCallback _sample_callback;

    u8 _div_apu_step = 0;
    u32 _div_apu_cycles = 0;

    GlobalControlRegisters _control;
    PulseChannel _chan1;
    PulseChannel _chan2;
    WaveChannel _chan3;
    NoiseChannel _chan4;
    std::array<u8, audio_waves.length> _waves;
};

} // namespace bdmg
