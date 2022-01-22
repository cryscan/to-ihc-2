cmake_minimum_required(VERSION 3.16)
project(to_ihc_2)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenMP REQUIRED)

add_subdirectory(biped)

add_executable(simulation simulation.cpp biped_model.cpp)
target_link_libraries(simulation biped dl)