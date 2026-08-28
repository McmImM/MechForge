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


add_executable(mf_engine
    src/core/mf_engine.cpp
    src/core/solver.cpp
    ${TINYEXPR_DIR}/tinyexpr.c
)

target_include_directories(mf_engine PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN_DIR}
    ${JSON_DIR}
    ${TINYEXPR_DIR}
)

# Python modules: copy (flattened) into lib/
set(MF_PYTHON_MODULES
    ${CMAKE_SOURCE_DIR}/src/core/mf_core.py
    ${CMAKE_SOURCE_DIR}/src/cli/mf_commands.py
    ${CMAKE_SOURCE_DIR}/src/cli/mf_repl.py
    ${CMAKE_SOURCE_DIR}/src/gui/mf_gui.py
    ${CMAKE_SOURCE_DIR}/src/gui/mfgui_hints.py
    ${CMAKE_SOURCE_DIR}/src/gui/mfgui_commandLine.py
    ${CMAKE_SOURCE_DIR}/src/server/mf_server.py
    ${CMAKE_SOURCE_DIR}/src/server/mf_remote.py
    ${CMAKE_SOURCE_DIR}/src/mcp/mf_mcp.py
)

foreach(mfpy ${MF_PYTHON_MODULES})
    get_filename_component(mfname ${mfpy} NAME)
    configure_file(${mfpy} ${CMAKE_BINARY_DIR}/lib/${mfname} COPYONLY)
endforeach()

# Copy the schemas directory into lib/schemas/
file(COPY ${CMAKE_SOURCE_DIR}/src/core/schemas/ DESTINATION ${CMAKE_BINARY_DIR}/lib/schemas/)

# Compile *.ui files into Python modules in lib/
find_program(PYSIDE6_UIC NAMES pyside6-uic PATHS ${CMAKE_SOURCE_DIR}/.venv/bin)
if(PYSIDE6_UIC)
    file(GLOB UI_FILES 
        CONFIGURE_DEPENDS
        ${CMAKE_SOURCE_DIR}/src/gui/*.ui)
    foreach(ui_file ${UI_FILES})
        get_filename_component(ui_name ${ui_file} NAME_WE)
        set(output_file ${CMAKE_BINARY_DIR}/lib/ui_${ui_name}.py)
        add_custom_command(
            OUTPUT ${output_file}
            COMMAND ${PYSIDE6_UIC} -o ${output_file} ${ui_file}
            DEPENDS ${ui_file}
            COMMENT "Compiling ${ui_file} to ${output_file}"
        )
        list(APPEND GENERATED_UI_FILES ${output_file})
    endforeach()
    add_custom_target(GenerateUIModules ALL DEPENDS ${GENERATED_UI_FILES})
else()
    message(FATAL_ERROR "pyside6-uic not found. GUI modules will not be generated.")
endif()


# --- Thin executable entry scripts in build/bin/ ----------------------------
# Each entry adds lib/ to sys.path and calls main() in the implementation
# module. The mf_* modules gain main() during the server/REPL refactor steps.
file(WRITE ${CMAKE_BINARY_DIR}/bin/mf-repl
"#!/usr/bin/env python3
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'lib'))
from mf_repl import main
raise SystemExit(main())
")

file(WRITE ${CMAKE_BINARY_DIR}/bin/mf-server
"#!/usr/bin/env python3
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'lib'))
from mf_server import main
raise SystemExit(main())
")

file(WRITE ${CMAKE_BINARY_DIR}/bin/mf-mcp
"#!/usr/bin/env python3
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'lib'))
from mf_mcp import main
raise SystemExit(main())
")