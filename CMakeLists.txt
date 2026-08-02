cmake_minimum_required(VERSION 3.15)
project(MechForge VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


# dependency directories
set(DEPENDENCIES_DIR ${CMAKE_SOURCE_DIR}/dependencies)

set(EIGEN_DIR ${DEPENDENCIES_DIR}/eigen)
set(JSON_DIR ${DEPENDENCIES_DIR}/json)
set(TINYEXPR_DIR ${DEPENDENCIES_DIR}/tinyexpr)


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

# Copy the Python CLI script to the output directory
configure_file(
    ${CMAKE_SOURCE_DIR}/src/cli/mechforgeCLI.py
    ${CMAKE_BINARY_DIR}/bin/mechforgeCLI.py
    COPYONLY
)

# Copy the schemas directory to the output directory
file(COPY ${CMAKE_SOURCE_DIR}/src/cli/schemas/ DESTINATION ${CMAKE_BINARY_DIR}/schemas/)