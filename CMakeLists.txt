cmake_minimum_required(VERSION 3.22)

project(BadDMG
    LANGUAGES CXX C
    VERSION 0.2
)

include(CTest)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)

add_subdirectory(deps)

add_compile_options(-Wall -Wextra -Werror)

option(
    BDMG_BUILD_TESTS
    "Fetch the test ROMs and GoogleTest, and build the ROM test suite"
    ON
)

add_subdirectory(src)
add_subdirectory(tests)
