#include "app.hh"
#include "sdl.hh"
#include "wav.hh"

#include <bdmg/common.hh>
#include <bdmg/boot-rom.hh>
#include <bdmg/audio.hh>
#include <bdmg/cartridge.hh>
#include <bdmg/joypad.hh>
#include <bdmg/pixel-processor.hh>
#include <bdmg/serial.hh>
#include <bdmg/timer.hh>
#include <bdmg/work-ram.hh>
#include <bdmg/high-ram.hh>
#include <bdmg/interrupt.hh>

#include <SDL3/SDL.h>
#include <SDL3/SDL_render.h>

#include <argparse/argparse.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <ctime>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

namespace bdmg {

constexpr s32 screen_scale = 3;

constexpr auto app_name = "BadDMG";
constexpr auto app_version = "0.1";
constexpr auto audio_freq = default_audio_sample_rate;

constexpr int device_audio_frames = 256;

constexpr u64 frame_lines = 154;
constexpr u64 line_cycles = 456;
constexpr u64 frame_cycles = frame_lines * line_cycles;
constexpr f64 console_rate = static_cast<f64>(dmg_cpu_speed) / static_cast<f64>(frame_cycles);
constexpr u64 ns_per_second = 1'000'000'000;
constexpr u64 target_fps = 60;
constexpr f64 resample_speed = static_cast<f64>(target_fps) / console_rate;
constexpr auto frame_time = std::chrono::nanoseconds(static_cast<s64>(ns_per_second / target_fps));
constexpr u64 samples_per_frame = audio_freq / target_fps;

constexpr u64 audio_prebuffer = device_audio_frames + samples_per_frame;
constexpr u64 audio_push_chunk = 256;
constexpr u64 max_queued_frames = 20;
constexpr u64 max_audio_queued = samples_per_frame * max_queued_frames;   // 0.33 s
constexpr u64 max_audio_pause_ns = 20'000'000;   // 20 ms
constexpr u64 stall_grace_frames = 2;

struct Options {
    std::string boot_rom;
    std::string cartridge;
    std::string record_wav;

    Options(int argc, char* argv[]) {
        argparse::ArgumentParser program(app_name, app_version);

        program
            .add_argument("-c", "--cartridge")
            .help("cartridge ROM file")
            .required();
        program
            .add_argument("-b", "--boot-rom")
            .help("boot code ROM file");
        program
            .add_argument("--record-wav")
            .help("record audio to a WAV file (48 kHz stereo F32)");

        program.parse_args(argc, argv);
        if (program.is_used("--boot-rom")) {
            boot_rom = program.get("--boot-rom");
        }
        if (program.is_used("--record-wav")) {
            record_wav = program.get("--record-wav");
        }
        cartridge = program.get("--cartridge");
    }
};

Application::Application(int argc, char* argv[]) {
    auto options = Options(argc, argv);

    const auto frames_hint = std::to_string(device_audio_frames);
    SDL_SetHint(SDL_HINT_AUDIO_DEVICE_SAMPLE_FRAMES, frames_hint.c_str());

    if (!options.record_wav.empty()) {
        _wav = std::make_unique<WavWriter>();
        _wav->open(options.record_wav, audio_freq);
    }

    _window.reset(SDL_CreateWindow(
        std::format("{} v{}", app_name, app_version).c_str(),
        screen_size.x*screen_scale,
        screen_size.y*screen_scale,
        SDL_WINDOW_HIGH_PIXEL_DENSITY
    ));
    if (!_window) {
        throw SDLError();
    }

    _renderer.reset(SDL_CreateRenderer(_window.get(), nullptr));
    if (!_renderer) {
        throw SDLError();
    }
    call_check(SDL_SetRenderVSync, _renderer.get(), 1);

    call_check(SDL_SetRenderLogicalPresentation,
        _renderer.get(),
        screen_size.x, screen_size.y,
        SDL_LOGICAL_PRESENTATION_INTEGER_SCALE
    );

    SDL_AudioSpec spec {
        .format = SDL_AUDIO_F32,
        .channels = 2,
        .freq = audio_freq
    };

    _audio_stream.reset(SDL_OpenAudioDeviceStream(
        SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK,
        &spec,
        nullptr,
        nullptr
    ));
    if (!_audio_stream) {
        throw SDLError();
    }
    call_check(SDL_ResumeAudioStreamDevice, _audio_stream.get());

    _tex.reset(SDL_CreateTexture(
        _renderer.get(), 
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        screen_size.x, screen_size.y
    ));
    if (!_tex) {
        throw SDLError();
    }
    call_check(SDL_SetTextureScaleMode, _tex.get(), SDL_SCALEMODE_NEAREST);

    auto cartridge = Cartridge::create(options.cartridge);
    cartridge->print_info(std::cerr);

    _bus = Bus::create();
    _bus->register_device(rom_bank_zero, cartridge);
    _bus->register_device(rom_bank_n, cartridge);
    _bus->register_device(ram_bank, cartridge);

    _bus->register_device(work_ram_range, bdmg::WorkRam::create());

    _pp = PixelProcessor::create();

    _bus->register_device(video_ram_range, _pp);
    _bus->register_device(lcd_range, _pp);
    _bus->register_device(oam_range, _pp);

    _audio = bdmg::Audio::create(resample_speed);
    _audio->set_sample_callback([this](const bdmg::AudioSample& sample) {
        _audio_samples.push_back(sample);
        if (_wav) {
            _wav->push(sample);
        }
        if (_audio_samples.size() >= audio_push_chunk) {
            push_audio();
        }
    });
    _bus->register_device(audio_chan1, _audio);
    _bus->register_device(audio_chan2, _audio);
    _bus->register_device(audio_chan3, _audio);
    _bus->register_device(audio_chan4, _audio);
    _bus->register_device(audio_control, _audio);
    _bus->register_device(audio_waves, _audio);

    _bus->register_device(high_ram_range, bdmg::HighRam::create());

    _joypad = bdmg::Joypad::create();
    _bus->register_device(joypad_range, _joypad);
    _bus->register_device(serial_range, bdmg::Serial::create());

    auto timer = bdmg::TimerImpl::create();
    timer->set_div_reset(bdmg::make_div_reset_handler(
        [audio = _audio]() { audio->reset_sequencer(); }
    ));
    _bus->register_device(timer_range, timer);

    auto interrupt = bdmg::Interrupt::create();
    _bus->register_device({ bdmg::int_flag_addr, 1 }, interrupt);
    _bus->register_device({ bdmg::int_enable_addr, 1 }, interrupt);

    _cpu = bdmg::Cpu(_bus, bdmg::BootRom::create(options.boot_rom));

    _frame.resize(screen_size.x*screen_size.y);
    _pp->set_frame_ready_callback([this](PixelProcessor::FrameSpan buf){
        return update_frame(buf);
    });
    timer->advance(*_bus, 0x8);
}

Application::~Application() = default;

void Application::update_frame(PixelProcessor::FrameSpan fb) {
    constexpr u32 color_mapping[4] = {
        0xFFFFFFFF,
        0xFFAAAAAA,
        0xFF555555,
        0xFF000000
    };
    _cycles_since_frame = 0;
    auto dst = _frame.data();
    for (auto src: fb) {
        *dst++ = color_mapping[src & 0x03];
    }
    _frame_ready = true;

    if (_make_screenshot) {
        _make_screenshot = false;
        if (auto out = std::ofstream(std::format("screenshot_{}.pgm", std::time(nullptr)), std::ios::binary)) {
            constexpr u8 sc_mappings[4] = { 0xFF, 0xAA, 0x55, 0x00 };
            out << std::format("P5\n{} {}\n255\n", screen_size.x, screen_size.y);
            for (auto src: fb) {
                out.write(reinterpret_cast<const char*>(&sc_mappings[src & 0x03]), 1);
            }
        }
    }
}

void Application::idle() {
    SDL_Event event{};
    while(SDL_PollEvent(&event)) {
        push_event(&event);
    }

    if (_frame_ready) {
        auto now = SDL_GetTicksNS();
        if (now - _last_frame_ready > static_cast<u64>(frame_time.count())) {
            _last_frame_ready = now;
            _frame_ready = false;
        }
        return;
    }
    while (!_frame_ready) {
        _cycles_since_frame += _cpu.step();
    }
    push_audio();

    _counter.update();
    auto fps_title = std::format("FPS: {:4.2}", _counter.get());
    call_check(SDL_SetWindowTitle, _window.get(), fps_title.c_str());

    call_check(SDL_SetRenderDrawColor, _renderer.get(), 0, 0, 0, SDL_ALPHA_OPAQUE);
    call_check(SDL_RenderClear, _renderer.get());
    call_check(SDL_UpdateTexture, _tex.get(), nullptr, _frame.data(), screen_size.x * sizeof(u32));
    call_check(SDL_RenderTexture, _renderer.get(), _tex.get(), nullptr, nullptr);
    call_check(SDL_RenderPresent, _renderer.get());
}

void Application::push_audio() {
    if (!_audio_stream || _audio_samples.empty()) {
        return;
    }
    constexpr auto device_frame_bytes = static_cast<int>(2 * sizeof(f32));
    static_assert(device_frame_bytes == 2 * 4, "the SDL stream above is declared as F32 stereo");

    const auto queued = static_cast<u64>(
        SDL_GetAudioStreamQueued(_audio_stream.get()) / device_frame_bytes);

    const bool frames_stalled = _cycles_since_frame > stall_grace_frames * frame_cycles;
    const auto backlog = queued + _audio_samples.size();
    if (frames_stalled && backlog > audio_prebuffer) {
        const auto excess_ns = (backlog - audio_prebuffer) * ns_per_second / audio_freq;
        SDL_DelayNS(std::min<u64>(excess_ns, max_audio_pause_ns));
    }

    if (!_audio_started) {
        if (_audio_samples.size() < audio_prebuffer) {
            return;
        }
        _audio_started = true;
    }

    if (queued >= max_audio_queued) {
        _audio_samples.clear();
        return;
    }

    const auto send = std::min<u64>(_audio_samples.size(), max_audio_queued - queued);

    _device_samples.resize(send * 2);
    for (std::size_t i = 0; i < send; ++i) {
        _device_samples[2 * i] = static_cast<f32>(_audio_samples[i].left);
        _device_samples[2 * i + 1] = static_cast<f32>(_audio_samples[i].right);
    }

    call_check(SDL_PutAudioStreamData, _audio_stream.get(),
    _device_samples.data(), static_cast<int>(send * device_frame_bytes));
    _audio_samples.erase(
    _audio_samples.begin(),
    _audio_samples.begin() + static_cast<std::ptrdiff_t>(send)
);

}

void Application::push_event(const SDL_Event* event) {
    if (event->type == SDL_EVENT_QUIT) {
        throw Application::Exit{};
    }
    if (event->type == SDL_EVENT_KEY_DOWN && event->key.key == SDLK_F12) {
        _make_screenshot = true;
    }
    if (event->type == SDL_EVENT_KEY_DOWN || event->type == SDL_EVENT_KEY_UP) {
        bool pressed = event->type == SDL_EVENT_KEY_DOWN;
        switch (event->key.key) {
            case SDLK_UP:     _up = pressed; break;
            case SDLK_DOWN:   _down = pressed; break;
            case SDLK_LEFT:   _left = pressed; break;
            case SDLK_RIGHT:  _right = pressed; break;
            case SDLK_Z:      _a = pressed; break;
            case SDLK_X:      _b = pressed; break;
            case SDLK_RETURN: _start = pressed; break;
            case SDLK_RSHIFT: _select = pressed; break;
        }
        _joypad->poll(_right, _left, _up, _down, _a, _b, _select, _start);
    }
}

} // namespace bdmg
