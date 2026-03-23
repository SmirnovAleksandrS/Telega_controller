cmake_minimum_required(VERSION 3.16)
project(telega_cpp_autopilot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(telega_cpp_autopilot
    src/main.cpp
    src/runtime_hooks.cpp
    src/runtime_protocol.cpp
    src/runtime_server.cpp
)

target_include_directories(telega_cpp_autopilot PRIVATE
    include
)

target_compile_options(telega_cpp_autopilot PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)
