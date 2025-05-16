#pragma once

#include <bdmg/bus.hh>
#include <bdmg/cpu.hh>
#include <bdmg/joypad.hh>

#include "fps-counter.hh"

union SDL_Event;
struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;

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

    SDL_Window* _window = nullptr;
    SDL_Renderer* _renderer = nullptr;
    SDL_Texture* _tex = nullptr;

    bdmg::Cpu _cpu;
    bdmg::SPBus _bus;
    bdmg::SPPixelProcessor _pp;
    bdmg::SPJoypad _joypad;

    FpsCounter _counter;
    std::vector<u32> _frame {}; 
    bool _frame_ready = false;

    bool _make_screenshot = false;
};

} // namespace bdmg
