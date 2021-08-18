cmake_minimum_required(VERSION 3.20)
project(to_ihc_2)

set(CMAKE_CXX_STANDARD 20)

set(EXECUTEBLE_NAME control)

add_subdirectory(hopper)

add_executable(control main.cpp dynamics.cpp cost.cpp)
target_link_libraries(control hopper)
