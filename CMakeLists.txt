cmake_minimum_required(VERSION 3.15)
project(MechForge VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

set(EIGEN_DIR ${CMAKE_SOURCE_DIR}/dependencies/eigen)
set(JSON_DIR ${CMAKE_SOURCE_DIR}/dependencies/json)
set(TINYEXPR_DIR ${CMAKE_SOURCE_DIR}/dependencies/tinyexpr)

add_executable(mechforge_solve
    src/cli/mechforge_solve.cpp
    src/core/solver.cpp
    ${TINYEXPR_DIR}/tinyexpr.c
)

target_include_directories(mechforge_solve PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN_DIR}
    ${JSON_DIR}
    ${TINYEXPR_DIR}
)