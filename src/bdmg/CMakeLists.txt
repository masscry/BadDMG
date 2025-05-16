add_library(bdmg)
add_library(bdmg::bdmg ALIAS bdmg)

target_sources(bdmg
    PRIVATE
        audio.cc
        bus.cc
        boot-rom.cc
        cartridge.cc
        common.cc
        cpu.cc
        work-ram.cc
        high-ram.cc
        pixel-processor.cc
        timer.cc
        joypad.cc
        serial.cc
        interrupt.cc
)

target_include_directories(bdmg
    INTERFACE
        ${CMAKE_SOURCE_DIR}/src
)
