if(NOT BDMG_BUILD_TESTS)
    return()
endif()

include(FetchContent)

FetchContent_Declare(
    GoogleTest
    GIT_SHALLOW    TRUE
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.17.0
)

FetchContent_Declare(
    gb-test-roms
    GIT_SHALLOW    TRUE
    GIT_REPOSITORY https://github.com/masscry/gb-test-roms.git
    GIT_TAG        c240dd7d700e5c0b00a7bbba52b53e4ee67b5f15
    SOURCE_SUBDIR  nomake # no CMake build of its own, only data files
)

FetchContent_MakeAvailable(GoogleTest gb-test-roms)
include(GoogleTest)

set(BDMG_TEST_ROMS
    cpu_instrs/cpu_instrs.gb
    dmg_sound/dmg_sound.gb
    halt_bug.gb
    instr_timing/instr_timing.gb
    interrupt_time/interrupt_time.gb
    mem_timing-2/mem_timing.gb
    oam_bug/oam_bug.gb
)

function(bdmg_embed_roms src_dir out_dir roms_var out_sources_var)
    # Mach-O symbols are prefixed with '_'; ELF and COFF are not.
    if(APPLE)
        set(_rom_sym "_")
        set(_rom_section "__TEXT,__const")
    elseif(WIN32)
        set(_rom_sym "")
        set(_rom_section ".rdata,\"dr\"")
    else()
        set(_rom_sym "")
        set(_rom_section ".rodata")
    endif()

    set(_rom_sources "")
    set(ROM_DECLS "")
    set(ROM_ENTRIES "")
    set(_rom_idx 0)
    foreach(_rom IN LISTS ${roms_var})
        set(ROM_ID "rom_${_rom_idx}")
        set(ROM_SYM "${_rom_sym}rom_${_rom_idx}")
        set(ROM_PATH "${src_dir}/${_rom}")
        set(ROM_SECTION "${_rom_section}")
        configure_file(roms_data.cc.in "${out_dir}/rom_${_rom_idx}.cc" @ONLY)
        list(APPEND _rom_sources "${out_dir}/rom_${_rom_idx}.cc")

        # Declarations and table entries come from the ROM's file name, so the
        # list above stays the only place that names a ROM.
        get_filename_component(_rom_stem "${_rom}" NAME_WE)
        string(REGEX REPLACE "[^A-Za-z0-9_]" "_" _rom_ident "${_rom_stem}")
        string(APPEND ROM_DECLS
            "extern \"C\" const unsigned char ${ROM_ID}[];\n"
            "extern \"C\" const unsigned char ${ROM_ID}_end[];\n")
        string(APPEND ROM_ENTRIES
            "inline const Rom ${_rom_ident} = {\"${_rom}\", ${ROM_ID}, ${ROM_ID}_end};\n")

        math(EXPR _rom_idx "${_rom_idx}+1")
    endforeach()

    configure_file(roms_embedded.hh.in "${out_dir}/roms_embedded.hh" @ONLY)

    set(${out_sources_var} "${_rom_sources}" PARENT_SCOPE)
endfunction()

add_executable(rom_test)
target_sources(rom_test
    PRIVATE
        rom_tests.cc
)

bdmg_embed_roms("${gb-test-roms_SOURCE_DIR}"
                "${CMAKE_CURRENT_BINARY_DIR}"
                BDMG_TEST_ROMS
                BDMG_EMBEDDED_ROM_SOURCES)

target_sources(rom_test PRIVATE ${BDMG_EMBEDDED_ROM_SOURCES})
target_include_directories(rom_test PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")

target_link_libraries(rom_test
    bdmg::bdmg
    GTest::gtest_main
)

gtest_discover_tests(rom_test)
