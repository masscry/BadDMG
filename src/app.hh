#pragma once

#include <bdmg/bus.hh>
#include <bdmg/cpu.hh>
#include <bdmg/joypad.hh>

#include "fps-counter.hh"
#include "sdl.hh"

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

    bdmg::Cpu _cpu;
    bdmg::SPBus _bus;
    bdmg::SPPixelProcessor _pp;
    bdmg::SPJoypad _joypad;

    FpsCounter _counter;
    std::vector<u32> _frame {}; 
    u64 _last_frame_ready = 0;

    bool _frame_ready = false;
    bool _make_screenshot = false;

};

} // namespace bdmg
