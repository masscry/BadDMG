#include "touch-controls.hh"

#include <SDL3/SDL.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string_view>

namespace bdmg {

namespace {

constexpr std::size_t control_count = 8;
constexpr u8 idle_alpha = 80;
constexpr u8 pressed_alpha = 190;
constexpr u8 glyph_alpha = 255;

struct Rect {
    float x;
    float y;
    float w;
    float h;

    bool contains(float px, float py) const {
        return px >= x && py >= y && px < x + w && py < y + h;
    }
};

struct Layout {
    Rect control[control_count];
    Rect dpad_center;
    Rect dpad_bounds;
};

Layout make_layout(float w, float h) {
    const float unit = std::min(w, h);
    const SDL_FRect game = game_destination(w, h, true);
    const float area_top = game.y + game.h + unit * 0.03f;
    const float area_bottom = h - unit * 0.04f;
    const float area_h = std::max(area_bottom - area_top, unit * 0.22f);

    const float dpad_x = w * 0.23f;
    const float arm = std::min({
        unit * 0.14f, area_h * 0.22f, (dpad_x - unit * 0.02f) / 1.5f});
    const float radius = std::min(unit * 0.12f, area_h * 0.19f);
    const float pill_w = std::min(unit * 0.16f, w * 0.22f);
    const float pill_h = std::min(unit * 0.08f, area_h * 0.12f);
    const auto centered = [](float cx, float cy, float rw, float rh) {
        return Rect{cx - rw / 2, cy - rh / 2, rw, rh};
    };

    const float dpad_y = area_top + area_h * 0.30f;

    Layout layout;
    layout.control[static_cast<std::size_t>(TouchInput::Control::up)] =
        centered(dpad_x, dpad_y - arm, arm, arm);
    layout.control[static_cast<std::size_t>(TouchInput::Control::down)] =
        centered(dpad_x, dpad_y + arm, arm, arm);
    layout.control[static_cast<std::size_t>(TouchInput::Control::left)] =
        centered(dpad_x - arm, dpad_y, arm, arm);
    layout.control[static_cast<std::size_t>(TouchInput::Control::right)] =
        centered(dpad_x + arm, dpad_y, arm, arm);
    layout.dpad_center = centered(dpad_x, dpad_y, arm, arm);
    layout.dpad_bounds = Rect{
        dpad_x - arm * 1.5f, dpad_y - arm * 1.5f, arm * 3, arm * 3};

    layout.control[static_cast<std::size_t>(TouchInput::Control::a)] =
        centered(w * 0.82f, area_top + area_h * 0.24f, radius * 2, radius * 2);
    layout.control[static_cast<std::size_t>(TouchInput::Control::b)] =
        centered(w * 0.66f, area_top + area_h * 0.52f, radius * 2, radius * 2);
    layout.control[static_cast<std::size_t>(TouchInput::Control::start)] =
        centered(w * 0.60f, area_top + area_h * 0.85f, pill_w, pill_h);
    layout.control[static_cast<std::size_t>(TouchInput::Control::select)] =
        centered(w * 0.40f, area_top + area_h * 0.85f, pill_w, pill_h);
    return layout;
}

bool is_pressed(const InputState& state, TouchInput::Control control) {
    switch (control) {
    case TouchInput::Control::up: return state.up;
    case TouchInput::Control::down: return state.down;
    case TouchInput::Control::left: return state.left;
    case TouchInput::Control::right: return state.right;
    case TouchInput::Control::a: return state.a;
    case TouchInput::Control::b: return state.b;
    case TouchInput::Control::start: return state.start;
    case TouchInput::Control::select: return state.select;
    }
    return false;
}

void set_pressed(InputState& state, TouchInput::Control control) {
    switch (control) {
    case TouchInput::Control::up: state.up = true; break;
    case TouchInput::Control::down: state.down = true; break;
    case TouchInput::Control::left: state.left = true; break;
    case TouchInput::Control::right: state.right = true; break;
    case TouchInput::Control::a: state.a = true; break;
    case TouchInput::Control::b: state.b = true; break;
    case TouchInput::Control::start: state.start = true; break;
    case TouchInput::Control::select: state.select = true; break;
    }
}

bool is_empty(const InputState& state) {
    return !state.up && !state.down && !state.left && !state.right
        && !state.a && !state.b && !state.start && !state.select;
}

bool is_dpad(const InputState& state) {
    return state.up || state.down || state.left || state.right;
}

std::optional<InputState> hit_test(const Layout& layout, float x, float y) {
    for (auto i = static_cast<std::size_t>(TouchInput::Control::a);
         i < control_count; ++i) {
        if (layout.control[i].contains(x, y)) {
            InputState state;
            set_pressed(state, static_cast<TouchInput::Control>(i));
            return state;
        }
    }

    if (!layout.dpad_bounds.contains(x, y)) {
        return std::nullopt;
    }

    const float cx = layout.dpad_bounds.x + layout.dpad_bounds.w / 2;
    const float cy = layout.dpad_bounds.y + layout.dpad_bounds.h / 2;
    const float dead = layout.dpad_bounds.w / 6;
    const float dx = x - cx;
    const float dy = y - cy;

    InputState state;
    state.left = dx < -dead;
    state.right = dx > dead;
    state.up = dy < -dead;
    state.down = dy > dead;
    return state;
}

void fill(SDL_Renderer* renderer, const Rect& rect, u8 alpha) {
    const SDL_FRect fr{rect.x, rect.y, rect.w, rect.h};
    SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, alpha);
    SDL_RenderFillRect(renderer, &fr);
}

void fill_circle(SDL_Renderer* renderer, const Rect& rect, u8 alpha) {
    const float radius = std::min(rect.w, rect.h) / 2;
    const float cx = rect.x + rect.w / 2;
    const float cy = rect.y + rect.h / 2;
    SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, alpha);
    for (float dy = -radius; dy <= radius; dy += 1.0f) {
        const float half = std::sqrt(std::max(0.0f, radius * radius - dy * dy));
        const SDL_FRect row{cx - half, cy + dy, half * 2 + 1, 1};
        SDL_RenderFillRect(renderer, &row);
    }
}

enum class Arrow { up, down, left, right };

void draw_arrow(SDL_Renderer* renderer, const Rect& arm, Arrow arrow) {
    const float size = std::min(arm.w, arm.h) * 0.5f;
    constexpr int steps = 5;
    const float step = size / steps;
    const float cx = arm.x + arm.w / 2;
    const float cy = arm.y + arm.h / 2;
    SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, glyph_alpha);
    for (int i = 0; i < steps; ++i) {
        const float width = size * (i + 1) / steps;
        SDL_FRect px;
        switch (arrow) {
        case Arrow::up:
            px = {cx - width / 2, cy - size / 2 + i * step, width, step};
            break;
        case Arrow::down:
            px = {cx - width / 2, cy + size / 2 - (i + 1) * step, width, step};
            break;
        case Arrow::left:
            px = {cx - size / 2 + i * step, cy - width / 2, step, width};
            break;
        case Arrow::right:
            px = {cx + size / 2 - (i + 1) * step, cy - width / 2, step, width};
            break;
        }
        SDL_RenderFillRect(renderer, &px);
    }
}

// 3x5 pixel font, enough for the control labels.
struct Glyph {
    char c;
    const char* rows[5];
};

constexpr Glyph glyphs[] = {
    {'A', {".#.", "#.#", "###", "#.#", "#.#"}},
    {'B', {"##.", "#.#", "##.", "#.#", "##."}},
    {'C', {".##", "#..", "#..", "#..", ".##"}},
    {'E', {"###", "#..", "##.", "#..", "###"}},
    {'L', {"#..", "#..", "#..", "#..", "###"}},
    {'R', {"##.", "#.#", "##.", "#.#", "#.#"}},
    {'S', {".##", "#..", ".#.", "..#", "##."}},
    {'T', {"###", ".#.", ".#.", ".#.", ".#."}},
};

const char* const* glyph_rows(char c) {
    for (const auto& glyph: glyphs) {
        if (glyph.c == c) {
            return glyph.rows;
        }
    }
    return nullptr;
}

float text_width(std::string_view text, float scale) {
    return (4.0f * static_cast<float>(text.size()) - 1.0f) * scale;
}

void draw_text(SDL_Renderer* renderer, float x, float y, float scale,
               std::string_view text, u8 alpha) {
    SDL_SetRenderDrawColor(renderer, 0xFF, 0xFF, 0xFF, alpha);
    for (const char c: text) {
        const auto rows = glyph_rows(c);
        if (rows) {
            for (int row = 0; row < 5; ++row) {
                for (int col = 0; col < 3; ++col) {
                    if (rows[row][col] == '#') {
                        const SDL_FRect px{x + col * scale, y + row * scale, scale, scale};
                        SDL_RenderFillRect(renderer, &px);
                    }
                }
            }
        }
        x += 4 * scale;
    }
}

void draw_letter(SDL_Renderer* renderer, const Rect& rect, char letter, float scale) {
    draw_text(renderer,
              rect.x + (rect.w - text_width(std::string_view(&letter, 1), scale)) / 2,
              rect.y + (rect.h - 5 * scale) / 2,
              scale, std::string_view(&letter, 1), glyph_alpha);
}

void draw_label(SDL_Renderer* renderer, const Rect& rect, std::string_view label) {
    const auto scale = std::max(2.0f, std::floor(std::min(
        rect.w * 0.88f / (4.0f * static_cast<float>(label.size()) - 1.0f),
        rect.h * 0.7f / 5.0f)));
    draw_text(renderer,
              rect.x + (rect.w - text_width(label, scale)) / 2,
              rect.y + (rect.h - 5 * scale) / 2,
              scale, label, glyph_alpha);
}

void draw_control(SDL_Renderer* renderer, const Layout& layout,
                  TouchInput::Control control, bool pressed) {
    const auto index = static_cast<std::size_t>(control);
    const auto& rect = layout.control[index];
    const u8 alpha = pressed ? pressed_alpha : idle_alpha;
    switch (control) {
    case TouchInput::Control::up:
        fill(renderer, rect, alpha);
        draw_arrow(renderer, rect, Arrow::up);
        break;
    case TouchInput::Control::down:
        fill(renderer, rect, alpha);
        draw_arrow(renderer, rect, Arrow::down);
        break;
    case TouchInput::Control::left:
        fill(renderer, rect, alpha);
        draw_arrow(renderer, rect, Arrow::left);
        break;
    case TouchInput::Control::right:
        fill(renderer, rect, alpha);
        draw_arrow(renderer, rect, Arrow::right);
        break;
    case TouchInput::Control::a:
        fill_circle(renderer, rect, alpha);
        draw_letter(renderer, rect, 'A', std::max(2.0f, rect.w * 0.10f));
        break;
    case TouchInput::Control::b:
        fill_circle(renderer, rect, alpha);
        draw_letter(renderer, rect, 'B', std::max(2.0f, rect.w * 0.10f));
        break;
    case TouchInput::Control::start:
        fill(renderer, rect, alpha);
        draw_label(renderer, rect, "START");
        break;
    case TouchInput::Control::select:
        fill(renderer, rect, alpha);
        draw_label(renderer, rect, "SELECT");
        break;
    }
}

} // namespace

SDL_FRect game_destination(float w, float h, bool top_anchored) {
    const float max_w = top_anchored ? w * 0.94f : w;
    const float max_h = top_anchored ? h * 0.52f : h;
    const int scale = std::max(1, std::min(
        static_cast<int>(max_w) / 160, static_cast<int>(max_h) / 144));
    const float dw = 160.0f * static_cast<float>(scale);
    const float dh = 144.0f * static_cast<float>(scale);
    const float x = (w - dw) / 2;
    const float y = top_anchored ? h * 0.05f : (h - dh) / 2;
    return {x, y, dw, dh};
}

InputState touch_state_at(float w, float h, float x, float y) {
    const auto layout = make_layout(w, h);
    if (const auto state = hit_test(layout, x, y)) {
        return *state;
    }
    return {};
}

bool TouchInput::handle_event(const SDL_Event& event) {
    switch (event.type) {
    case SDL_EVENT_FINGER_DOWN:
    case SDL_EVENT_FINGER_MOTION:
    case SDL_EVENT_FINGER_UP:
    case SDL_EVENT_FINGER_CANCELED:
        break;
    default:
        return false;
    }

    const auto finger = event.tfinger.fingerID;
    const auto previous = std::find_if(_pressed.begin(), _pressed.end(),
        [finger](const Press& press) { return press.finger == finger; });

    std::optional<InputState> target;
    if (event.type == SDL_EVENT_FINGER_DOWN || event.type == SDL_EVENT_FINGER_MOTION) {
        SDL_Window* window = SDL_GetWindowFromID(event.tfinger.windowID);
        int w = 0;
        int h = 0;
        if (window && SDL_GetWindowSizeInPixels(window, &w, &h) && w > 0 && h > 0) {
            const auto layout = make_layout(static_cast<float>(w), static_cast<float>(h));
            const auto x = event.tfinger.x * static_cast<float>(w);
            const auto y = event.tfinger.y * static_cast<float>(h);
            target = hit_test(layout, x, y);
        }
    }

    if (!target) {
        if (previous == _pressed.end()) {
            return false;
        }
        _pressed.erase(previous);
        return true;
    }

    if (is_empty(*target)) {
        // The finger is in the middle of the D-pad. Keep the last direction
        // while it crosses the dead zone, but don't invent one from nothing.
        if (previous == _pressed.end()) {
            return false;
        }
        if (is_dpad(previous->state)) {
            return false;
        }
        _pressed.erase(previous);
        return true;
    }

    if (previous != _pressed.end()) {
        if (previous->state == *target) {
            return false;
        }
        previous->state = *target;
        return true;
    }
    _pressed.push_back({finger, *target});
    return true;
}

InputState TouchInput::state() const {
    InputState state;
    for (const auto& press: _pressed) {
        state = state | press.state;
    }
    return state;
}

void draw_touch_controls(SDL_Renderer* renderer, const InputState& state) {
    int w = 0;
    int h = 0;
    if (!SDL_GetRenderOutputSize(renderer, &w, &h) || w <= 0 || h <= 0) {
        return;
    }
    const auto layout = make_layout(static_cast<float>(w), static_cast<float>(h));
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);
    fill(renderer, layout.dpad_center, idle_alpha);
    for (std::size_t i = 0; i < control_count; ++i) {
        const auto control = static_cast<TouchInput::Control>(i);
        draw_control(renderer, layout, control, is_pressed(state, control));
    }
}

} // namespace bdmg
