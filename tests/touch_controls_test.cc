#include <gtest/gtest.h>

#include <SDL3/SDL.h>

#include "touch-controls.hh"

namespace bdmg {
namespace {

Uint32 pixel_at(SDL_Surface* surface, int x, int y) {
    auto* row = static_cast<Uint32*>(static_cast<void*>(
        static_cast<Uint8*>(surface->pixels) + y * surface->pitch));
    return row[x];
}

// Mirrors Application::idle(): a game frame rendered into game_destination(),
// then the touch controls on top in window coordinates.
TEST(TouchControls, game_is_top_anchored_with_controls_below) {
    constexpr int width = 1080;
    constexpr int height = 2424;
    SDL_Surface* surface = SDL_CreateSurface(width, height, SDL_PIXELFORMAT_ARGB8888);
    ASSERT_NE(surface, nullptr);
    SDL_Renderer* renderer = SDL_CreateSoftwareRenderer(surface);
    ASSERT_NE(renderer, nullptr);

    const SDL_FRect game = game_destination(width, height, true);
    EXPECT_EQ(game.w, 960.0f);
    EXPECT_EQ(game.h, 864.0f);
    EXPECT_FLOAT_EQ(game.y, height * 0.05f);

    SDL_SetRenderDrawColor(renderer, 0, 0, 0, SDL_ALPHA_OPAQUE);
    SDL_RenderClear(renderer);
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, SDL_ALPHA_OPAQUE);
    SDL_RenderFillRect(renderer, &game);
    draw_touch_controls(renderer, InputState{});
    SDL_RenderPresent(renderer);

    EXPECT_EQ(pixel_at(surface, 540, 553) & 0xFFFFFF, 0xFFFFFF) << "game screen";
    EXPECT_LT(pixel_at(surface, 540, 40) & 0xFF, 10) << "space above the game";
    EXPECT_GT(pixel_at(surface, 248, 1427) & 0xFF, 40) << "d-pad";
    EXPECT_GT(pixel_at(surface, 864, 1399) & 0xFF, 40) << "A button";
    EXPECT_GT(pixel_at(surface, 575, 2176) & 0xFF, 40) << "start button";
    EXPECT_GT(pixel_at(surface, 360, 2176) & 0xFF, 40) << "select button";

    SDL_DestroyRenderer(renderer);
    SDL_DestroySurface(surface);
}

TEST(TouchControls, dpad_has_diagonals_and_a_neutral_center) {
    constexpr float width = 1080.0f;
    constexpr float height = 2424.0f;

    // make_layout(): center (248.4, 1426.56), arm 151.2, so the corner
    // between up and left is around (97.2, 1275.36).
    const auto up = touch_state_at(width, height, 248.4f, 1275.36f);
    EXPECT_TRUE(up.up);
    EXPECT_FALSE(up.left || up.right || up.down);

    const auto up_left = touch_state_at(width, height, 97.2f, 1275.36f);
    EXPECT_TRUE(up_left.up);
    EXPECT_TRUE(up_left.left);
    EXPECT_FALSE(up_left.right || up_left.down);

    const auto center = touch_state_at(width, height, 248.4f, 1426.56f);
    EXPECT_FALSE(center.up || center.down || center.left || center.right);
}

} // namespace
} // namespace bdmg
