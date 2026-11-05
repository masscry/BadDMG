cmake_minimum_required(VERSION 3.22)

project(BadDMG
    LANGUAGES CXX C
    VERSION 0.2
)

include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)

set(BDMG_COMPILE_OPTIONS -Wall -Wextra -Werror)

if(ANDROID)
    # Keep the build directory out of __FILE__ and the build-id so the .so
    # files do not depend on where the sources were checked out.
    add_compile_options(-ffile-prefix-map=${CMAKE_SOURCE_DIR}=.)
    add_link_options(-Wl,--build-id=none)
    set(BDMG_BUILD_TESTS_DEFAULT OFF)
else()
    set(BDMG_BUILD_TESTS_DEFAULT ON)
endif()

add_subdirectory(deps)

option(
    BDMG_BUILD_TESTS
    "Fetch the test ROMs and GoogleTest, and build the ROM test suite"
    ${BDMG_BUILD_TESTS_DEFAULT}
)

add_subdirectory(src)
add_subdirectory(tests)
