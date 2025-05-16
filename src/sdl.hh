#pragma once

#include <stdexcept>
#include <functional>

#include <SDL3/SDL.h>

namespace bdmg {

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

inline
SDLError::SDLError() : std::runtime_error(SDL_GetError()) {}

} // namespace bdmg
