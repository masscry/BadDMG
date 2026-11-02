#include "input.hh"

#include <SDL3/SDL.h>

namespace bdmg {

namespace {

constexpr Sint16 axis_deadzone = 16000;

} // namespace

bool KeyboardInput::handle_event(const SDL_Event& event) {
    if (event.type != SDL_EVENT_KEY_DOWN && event.type != SDL_EVENT_KEY_UP) {
        return false;
    }
    const bool pressed = event.type == SDL_EVENT_KEY_DOWN;
    switch (event.key.key) {
    case SDLK_UP: _state.up = pressed; return true;
    case SDLK_DOWN: _state.down = pressed; return true;
    case SDLK_LEFT: _state.left = pressed; return true;
    case SDLK_RIGHT: _state.right = pressed; return true;
    case SDLK_Z: _state.a = pressed; return true;
    case SDLK_X: _state.b = pressed; return true;
    case SDLK_RETURN: _state.start = pressed; return true;
    case SDLK_RSHIFT: _state.select = pressed; return true;
    default: return false;
    }
}

GamepadInput::GamepadInput() {
    // shortcut: one gamepad at a time, upgrade if local multiplayer is needed
    int count = 0;
    SDL_JoystickID* ids = SDL_GetGamepads(&count);
    if (!ids) {
        return;
    }
    if (count > 0) {
        open(ids[0]);
    }
    SDL_free(ids);
}

GamepadInput::~GamepadInput() {
    close();
}

void GamepadInput::open(SDL_JoystickID id) {
    _pad = SDL_OpenGamepad(id);
    if (!_pad) {
        SDL_LogWarn(SDL_LOG_CATEGORY_APPLICATION, "failed to open gamepad: %s", SDL_GetError());
        return;
    }
    _id = id;
    _buttons = 0;
    _axis_up = _axis_down = _axis_left = _axis_right = false;
}

void GamepadInput::close() {
    if (_pad) {
        SDL_CloseGamepad(_pad);
    }
    _pad = nullptr;
    _id = 0;
    _buttons = 0;
    _axis_up = _axis_down = _axis_left = _axis_right = false;
}

bool GamepadInput::handle_event(const SDL_Event& event) {
    switch (event.type) {
    case SDL_EVENT_GAMEPAD_ADDED:
        if (!_pad) {
            open(event.gdevice.which);
            return _pad != nullptr;
        }
        return false;
    case SDL_EVENT_GAMEPAD_REMOVED:
        if (_pad && event.gdevice.which == _id) {
            close();
            return true;
        }
        return false;
    case SDL_EVENT_GAMEPAD_BUTTON_DOWN:
    case SDL_EVENT_GAMEPAD_BUTTON_UP:
        if (!_pad || event.gbutton.which != _id) {
            return false;
        }
        {
            const auto index = static_cast<unsigned>(event.gbutton.button);
            if (index >= 32) {
                return false;
            }
            if (event.type == SDL_EVENT_GAMEPAD_BUTTON_DOWN) {
                _buttons |= 1u << index;
            } else {
                _buttons &= ~(1u << index);
            }
        }
        return true;
    case SDL_EVENT_GAMEPAD_AXIS_MOTION:
        if (!_pad || event.gaxis.which != _id) {
            return false;
        }
        switch (event.gaxis.axis) {
        case SDL_GAMEPAD_AXIS_LEFTX:
            _axis_left = event.gaxis.value < -axis_deadzone;
            _axis_right = event.gaxis.value > axis_deadzone;
            return true;
        case SDL_GAMEPAD_AXIS_LEFTY:
            _axis_up = event.gaxis.value < -axis_deadzone;
            _axis_down = event.gaxis.value > axis_deadzone;
            return true;
        default:
            return false;
        }
    default:
        return false;
    }
}

InputState GamepadInput::state() const {
    const auto pressed = [this](SDL_GamepadButton button) {
        const auto index = static_cast<unsigned>(button);
        return index < 32 && ((_buttons >> index) & 1u);
    };

    return {
        .up = pressed(SDL_GAMEPAD_BUTTON_DPAD_UP) || _axis_up,
        .down = pressed(SDL_GAMEPAD_BUTTON_DPAD_DOWN) || _axis_down,
        .left = pressed(SDL_GAMEPAD_BUTTON_DPAD_LEFT) || _axis_left,
        .right = pressed(SDL_GAMEPAD_BUTTON_DPAD_RIGHT) || _axis_right,
        .a = pressed(SDL_GAMEPAD_BUTTON_SOUTH),
        .b = pressed(SDL_GAMEPAD_BUTTON_EAST),
        .select = pressed(SDL_GAMEPAD_BUTTON_BACK),
        .start = pressed(SDL_GAMEPAD_BUTTON_START),
    };
}

} // namespace bdmg
