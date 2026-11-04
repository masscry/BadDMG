include(FetchContent)

if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

set(FETCHCONTENT_QUIET FALSE)

FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
    GIT_PROGRESS TRUE
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(argparse)

# It is faster to checkout an archive, than a git repo.
FetchContent_Declare(
    SDL3
    URL "https://github.com/libsdl-org/SDL/archive/32e9d379f62ed0f2f092c269967f557c307431e8.tar.gz"
    URL_HASH "SHA256=ffa99dbe0dbd699cfd509983c1122dacc72e83dfaa1eadb28dd0dae518199292"
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
