add_subdirectory(bdmg)

if(ANDROID)
    # SDLActivity loads the application code as libmain.so
    add_library(BadDMG SHARED)
    set_target_properties(BadDMG PROPERTIES OUTPUT_NAME main)
else()
    add_executable(BadDMG)
endif()

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
