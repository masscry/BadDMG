cmake_minimum_required(VERSION 3.22)

project(BadDMG
    LANGUAGES CXX C
    VERSION 0.1
)

add_subdirectory(deps)

add_compile_options(-Wall -Wextra -Werror)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)

add_subdirectory(src)
