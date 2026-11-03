add_subdirectory(bdmg)

add_executable(BadDMG)

target_sources(BadDMG
    PRIVATE
        app.cc
        sdl-main.cc
        sdl.cc
        wav.cc
        input.cc
        touch-controls.cc
)

target_compile_options(BadDMG
    PRIVATE
        ${BDMG_COMPILE_OPTIONS}
)

target_link_libraries(BadDMG 
    PRIVATE
        bdmg::bdmg
        SDL3::SDL3
        argparse
)
