#pragma once

#include <bdmg/joypad.hh>

#include <SDL3/SDL_events.h>
#include <SDL3/SDL_gamepad.h>

namespace bdmg {

class IInput {
public:
    virtual ~IInput() = default;

    // Returns true if this source's state may have changed.
    virtual bool handle_event(const SDL_Event& event) = 0;
    virtual InputState state() const = 0;
};

class KeyboardInput final: public IInput {
public:
    bool handle_event(const SDL_Event& event) override;
    InputState state() const override { return _state; }

private:
    InputState _state;
};

class GamepadInput final: public IInput {
public:
    GamepadInput();
    ~GamepadInput() override;
    GamepadInput(const GamepadInput&) = delete;
    GamepadInput& operator=(const GamepadInput&) = delete;

    bool handle_event(const SDL_Event& event) override;
    InputState state() const override;

private:
    void open(SDL_JoystickID id);
    void close();

    SDL_Gamepad* _pad = nullptr;
    SDL_JoystickID _id = 0;
    Uint32 _buttons = 0;
    bool _axis_up = false;
    bool _axis_down = false;
    bool _axis_left = false;
    bool _axis_right = false;
};

} // namespace bdmg
