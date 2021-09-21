cmake_minimum_required(VERSION 3.20)
project(to_ihc_2)

set(CMAKE_CXX_STANDARD 20)

find_package(OpenMP REQUIRED)

add_subdirectory(hopper)

add_executable(control main.cpp kinetics.cpp dynamics.cpp cost.cpp stabilizer.cpp lqr.cpp)
target_link_libraries(control hopper OpenMP::OpenMP_CXX dl)

add_executable(simulation simulation.cpp kinetics.cpp dynamics.cpp stabilizer.cpp)
target_link_libraries(simulation hopper OpenMP::OpenMP_CXX dl)
