# Dependencies are git submodules: clone recursively, or run
# `git submodule update --init --recursive` before configuring.
add_subdirectory(argparse EXCLUDE_FROM_ALL)
add_subdirectory(SDL EXCLUDE_FROM_ALL)

# sha2.hpp trips -Wunused-lambda-capture with our -Werror; treat it as a
# system header instead of patching the dependency.
add_library(bdmg_sha2 INTERFACE)
add_library(bdmg::sha2 ALIAS bdmg_sha2)
target_include_directories(bdmg_sha2
    SYSTEM INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/sha-2/src
)

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
        ${CMAKE_CURRENT_SOURCE_DIR}/r8brain
)
