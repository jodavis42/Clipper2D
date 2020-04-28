################################################################################
# Copyright (c) 2020 Josh Davis
################################################################################

################################################################################
# Set required version of cmake and any version specific policies
################################################################################
cmake_minimum_required(VERSION 3.9.1 FATAL_ERROR)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CONFIGURATION_TYPES Debug Release RelWithDebInfo)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Zi")
set(CMAKE_SHARED_LINKER_FLAGS_RELEASE "${CMAKE_SHARED_LINKER_FLAGS_RELEASE} /DEBUG /OPT:REF /OPT:ICF")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(Clipper2D CXX)

include(CMake/CommonCompileOptions.cmake)
include(CMake/Precompiled.cmake)
add_definitions(-DCOMPILER_MICROSOFT=1)
add_definitions(-DPLATFORM_WINDOWS=1 -DPLATFORM_HARDWARE=1 -DPLATFORM_NAME="Win" -D_UNICODE -DUNICODE)

add_subdirectory(Clipper)
add_subdirectory(Tests)

set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Tests)
