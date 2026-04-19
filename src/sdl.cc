#include "sdl.hh"

#include <SDL3/SDL.h>

namespace bdmg {

SDLError::SDLError() : std::runtime_error(SDL_GetError()) {}

void ResourceDestructor::operator()(SDL_Window* window) {
    SDL_DestroyWindow(window);
}

void ResourceDestructor::operator()(SDL_Renderer* renderer) {
    SDL_DestroyRenderer(renderer);
}

void ResourceDestructor::operator()(SDL_Surface* surface) {
    SDL_DestroySurface(surface);
}

void ResourceDestructor::operator()(SDL_Texture* texture) {
    SDL_DestroyTexture(texture);
}

ITimer::ITimer(std::chrono::nanoseconds timeout) {
    auto callback = [](void *userdata, SDL_TimerID, Uint64 interval){
        auto& self = *static_cast<ITimer*>(userdata);
        self.fire();
        return interval;
    };
    _id = SDL_AddTimerNS(timeout.count(), callback, this);
    if (_id == 0) {
        throw SDLError();
    }
}

ITimer::~ITimer() {
    if (_id != 0) {
        SDL_RemoveTimer(_id);
    }
}
    
} // namespace bdmg
