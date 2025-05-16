#define SDL_MAIN_USE_CALLBACKS
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <SDL3/SDL_log.h>

#include "app.hh"
#include "sdl.hh"

#include <bdmg/common.hh>

SDL_AppResult SDL_AppInit(void **pApp, int argc, char* argv[]) {
    try {
        SDL_Log("%s", "Init Application");
        bdmg::call_check(SDL_InitSubSystem, SDL_INIT_VIDEO);
        *pApp = new bdmg::Application(argc, argv);
        return SDL_APP_CONTINUE;
    } catch (const std::exception& exc) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "init failed: %s", exc.what());
    }
    return SDL_APP_FAILURE;
}

SDL_AppResult SDL_AppIterate(void *pApp) {
    try {
        auto& app = *reinterpret_cast<bdmg::Application*>(pApp);
        app.idle();
        return SDL_APP_CONTINUE;
    } catch (bdmg::Application::Exit) {
        return SDL_APP_SUCCESS;
    } catch (const std::exception& exc) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "idle failed: %s", exc.what());
    }
    return SDL_APP_FAILURE;
}

SDL_AppResult SDL_AppEvent(void *pApp, SDL_Event *event) {
    try {
        auto& app = *reinterpret_cast<bdmg::Application*>(pApp);
        app.push_event(event);
        return SDL_APP_CONTINUE;
    } catch (bdmg::Application::Exit) {
        return SDL_APP_SUCCESS;
    } catch (const std::exception& exc) {
        SDL_LogError(SDL_LOG_CATEGORY_APPLICATION, "event failed: %s", exc.what());
    }
    return SDL_APP_FAILURE;
}

void SDL_AppQuit(void *pApp, SDL_AppResult result) {
    SDL_Log("Quit Application (result: %d)", static_cast<int>(result));
    delete reinterpret_cast<bdmg::Application*>(pApp);
    SDL_QuitSubSystem(SDL_INIT_VIDEO);
    SDL_Quit();
}
