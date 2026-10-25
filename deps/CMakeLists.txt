include(FetchContent)

set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(argparse)

FetchContent_Declare(
    SDL3
    GIT_REPOSITORY "https://github.com/libsdl-org/SDL.git"
    GIT_TAG "release-3.2.16"
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(SDL3)

FetchContent_Declare(
    r8brain
    GIT_REPOSITORY "https://github.com/avaneev/r8brain-free-src.git"
    GIT_TAG "9e73d2dd59fd5b95108fdb4f590083e35758b45f"
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(r8brain)

add_library(bdmg_rsmp STATIC)
add_library(bdmg::rsmp ALIAS bdmg_rsmp)

target_sources(bdmg_rsmp
    PRIVATE
        rsmp/rsmp.cc
)

target_include_directories(bdmg_rsmp
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
    PRIVATE
        ${r8brain_SOURCE_DIR}
)
