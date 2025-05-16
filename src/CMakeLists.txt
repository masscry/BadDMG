add_subdirectory(bdmg)

add_executable(BadDMG)

target_sources(BadDMG
    PRIVATE
        sdl-main.cc
        app.cc
        sdl.cc
)

target_link_libraries(BadDMG 
    PRIVATE
        bdmg::bdmg
        SDL3::SDL3
        argparse
)
