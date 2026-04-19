#pragma once

#include <stdexcept>
#include <functional>
#include <chrono>

union SDL_Event;
struct SDL_Window;
struct SDL_Renderer;
struct SDL_Surface;
struct SDL_Texture;
using SDL_TimerID = uint32_t;

namespace bdmg {

class ITimer {
public:
    explicit ITimer(std::chrono::nanoseconds);
    ITimer(const ITimer&) = delete;
    ITimer& operator=(const ITimer&) = delete;
    ITimer(ITimer&&) = delete;
    ITimer& operator=(ITimer&&) = delete;
    virtual ~ITimer();

private:
    virtual void fire() = 0;
    SDL_TimerID _id;
};

class ResourceDestructor {
public:
    void operator()(SDL_Window*);
    void operator()(SDL_Renderer*);
    void operator()(SDL_Surface*);
    void operator()(SDL_Texture*);
};

using Window = std::unique_ptr<SDL_Window, ResourceDestructor>;
using Renderer = std::unique_ptr<SDL_Renderer, ResourceDestructor>;
using Surface = std::unique_ptr<SDL_Surface, ResourceDestructor>;
using Texture = std::unique_ptr<SDL_Texture, ResourceDestructor>;
using Timer = std::unique_ptr<ITimer>;

template<typename Func>
Timer make_timer(Func&& func, std::chrono::nanoseconds timeout) {
    class LambdaTimer final: public ITimer {
    public:
        LambdaTimer(Func&& func, std::chrono::nanoseconds timeout)
            : ITimer(timeout)
            , _func(std::forward<Func>(func)) {}

        void fire() override {
            _func();
        }
    private:
        Func _func;
    };
    return std::make_unique<LambdaTimer>(std::forward<Func>(func),timeout);
}

class SDLError: public std::runtime_error {
public:
    SDLError();
};

template<typename Func, typename... Args>
auto call_check(Func&& func, Args&&... args) {
    if (auto result = std::invoke(func, std::forward<Args>(args)...)) {
        return result;
    }
    throw SDLError();
}

} // namespace bdmg
