#pragma once

#include "input.hh"

#include <SDL3/SDL_rect.h>

#include <vector>

struct SDL_Renderer;

namespace bdmg {

// On-screen D-pad and buttons for touch-only devices.
class TouchInput final: public IInput {
public:
    enum class Control { up, down, left, right, a, b, start, select };

    bool handle_event(const SDL_Event& event) override;
    InputState state() const override;

private:
    struct Press {
        SDL_FingerID finger;
        Control control;
    };

    std::vector<Press> _pressed;
};

// Destination of the 160x144 screen inside a w x h output. Top-anchored
// layouts leave the space below for touch controls.
SDL_FRect game_destination(float w, float h, bool top_anchored);

void draw_touch_controls(SDL_Renderer* renderer, const InputState& state);

} // namespace bdmg
