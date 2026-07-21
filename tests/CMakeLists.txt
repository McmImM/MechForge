cmake_minimum_required(VERSION 3.15)
project(MechForgeTest VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(EIGEN_DIR ${CMAKE_SOURCE_DIR}/../dependencies/eigen)

# Test 0: Crank-slider example
add_executable(test0_crank_slider
    test0_crank_slider/crank_slider.cpp
    ../src/core/solver.cpp
)

target_include_directories(test0_crank_slider PRIVATE
    ${CMAKE_SOURCE_DIR}/../src
    ${EIGEN_DIR}
)