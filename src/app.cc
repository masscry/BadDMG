#include "app.hh"
#include "sdl.hh"

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
#include <SDL3/SDL_surface.h>

#include <argparse/argparse.hpp>

#include <format>
#include <iostream>
#include <fstream>

namespace bdmg {

constexpr s32 screen_scale = 3;

constexpr auto app_name = "BadDMG";
constexpr auto app_version = "0.1";

struct Options {
    std::string boot_rom;
    std::string cartridge;

    Options(int argc, char* argv[]) {
        argparse::ArgumentParser program(app_name, app_version);

        program
            .add_argument("-c", "--cartridge")
            .help("cartridge ROM file")
            .required();
        program
            .add_argument("-b", "--boot-rom")
            .help("boot code ROM file");

        program.parse_args(argc, argv);
        if (program.is_used("--boot-rom")) {
            boot_rom = program.get("--boot-rom");
        }
        cartridge = program.get("--cartridge");
    }
};

Application::Application(int argc, char* argv[]) {
    auto options = Options(argc, argv);

    _window = SDL_CreateWindow(
        std::format("{} v{}", app_name, app_version).c_str(),
        screen_size.x*screen_scale,
        screen_size.y*screen_scale,
        SDL_WINDOW_HIGH_PIXEL_DENSITY
    );
    if (_window == nullptr) {
        throw SDLError();
    }

    _renderer = SDL_CreateRenderer(_window, nullptr);
    if (_renderer == nullptr) {
        throw SDLError();
    }
    call_check(SDL_SetRenderVSync, _renderer, 1);

    call_check(SDL_SetRenderLogicalPresentation,
        _renderer,
        screen_size.x, screen_size.y,
        SDL_LOGICAL_PRESENTATION_INTEGER_SCALE
    );

    _tex = SDL_CreateTexture(
        _renderer, 
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        screen_size.x, screen_size.y
    );
    call_check(SDL_SetTextureScaleMode, _tex, SDL_SCALEMODE_NEAREST);


    if (!_tex) {
        throw SDLError();
    }

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

    _bus->register_device(audio_range, bdmg::Audio::create());
    _bus->register_device(high_ram_range, bdmg::HighRam::create());

    _joypad = bdmg::Joypad::create();
    _bus->register_device(joypad_range, _joypad);
    _bus->register_device(serial_range, bdmg::Serial::create());

    auto timer = bdmg::Timer::create();
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

Application::~Application() {
    SDL_DestroyRenderer(_renderer);
    SDL_DestroyWindow(_window);
}

void Application::update_frame(PixelProcessor::FrameSpan fb) {
    constexpr u32 color_mapping[4] = {
        0xFFFFFFFF,
        0xFFAAAAAA,
        0xFF555555,
        0xFF000000
    };
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
    _counter.update();
    auto fps_title = std::format("FPS: {:4.2}", _counter.get());
    call_check(SDL_SetWindowTitle, _window, fps_title.c_str());

    _frame_ready = false;
    while (!_frame_ready) {
        _cpu.step();
    }

    call_check(SDL_SetRenderDrawColor, _renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    call_check(SDL_RenderClear, _renderer);
    call_check(SDL_UpdateTexture, _tex, nullptr, _frame.data(), screen_size.x * sizeof(u32));
    call_check(SDL_RenderTexture, _renderer, _tex, nullptr, nullptr);
    call_check(SDL_RenderPresent, _renderer);
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
