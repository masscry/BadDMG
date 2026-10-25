#pragma once

#include <bdmg/bus.hh>
#include <bdmg/cpu.hh>
#include <bdmg/joypad.hh>
#include <bdmg/audio.hh>

#include "fps-counter.hh"
#include "sdl.hh"

#include <memory>
#include <span>
#include <vector>

namespace bdmg {

class Application {
public:
    Application(int argc, char* argv[]);
    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;
    Application(Application&&) = delete;
    Application& operator=(Application&&) = delete;
    ~Application();

    class Exit {};

    void idle();
    void push_event(const SDL_Event* event);

private:
    void update_frame(std::span<const u8, screen_size.x * screen_size.y>);
    void push_audio();

    bool _up = false;
    bool _down = false;
    bool _left = false;
    bool _right = false;
    bool _a = false;
    bool _b = false;
    bool _select = false;
    bool _start = false;

    Window _window;
    Renderer _renderer;
    Texture _tex;
    AudioStream _audio_stream;

    bdmg::Cpu _cpu;
    bdmg::SPBus _bus;
    bdmg::SPPixelProcessor _pp;
    bdmg::SPJoypad _joypad;
    bdmg::SPAudio _audio;

    FpsCounter _counter;
    std::vector<u32> _frame {};
    u64 _last_frame_ready = 0;
    u64 _cycles_since_frame = 0;

    std::vector<bdmg::AudioSample> _audio_samples;
    std::vector<f32> _device_samples;
    bool _audio_started = false;

    bool _frame_ready = false;
    bool _make_screenshot = false;

};

} // namespace bdmg
