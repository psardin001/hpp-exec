#
# Copyright (c) 2026 CNRS Author: Paul Sardin
#
# hpp-exec is free software: you can redistribute it and/or modify it under the
# terms of the BSD 2-Clause License.
#

cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME hpp-exec)
set(PROJECT_DESCRIPTION "ROS2 execution utilities for HPP-generated trajectories")
set(PROJECT_URL "https://github.com/humanoid-path-planner/${PROJECT_NAME}")

set(CXX_DISABLE_WERROR TRUE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
  find_package(jrl-cmakemodules QUIET CONFIG)
  if(jrl-cmakemodules_FOUND)
    get_property(
      JRL_CMAKE_MODULES
      TARGET jrl-cmakemodules::jrl-cmakemodules
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

include("${JRL_CMAKE_MODULES}/hpp.cmake")
include("${JRL_CMAKE_MODULES}/python.cmake")

set(PROJECT_USE_CMAKE_EXPORT TRUE)

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

if(NOT FINDPYTHON_ALREADY_CALLED)
  findpython()
endif()

# Install Python package
python_install_on_site("hpp_exec" "__init__.py")
python_install_on_site("hpp_exec" "trajectory_utils.py")
python_install_on_site("hpp_exec" "ros2_sender.py")
python_install_on_site("hpp_exec" "graph_segments.py")
python_install_on_site("hpp_exec" "segments.py")

# Interface target so hpp-doc and other consumers can find_package(hpp-exec)
# and so doc/Doxyfile.extra.in is picked up by the JRL doxygen macros.
add_library(${PROJECT_NAME} INTERFACE)
install(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
