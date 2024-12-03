# ##############################################################################
# cmake/nuttx_extensions.cmake
#
# SPDX-License-Identifier: Apache-2.0
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

# This is NuttX's enhancement of various native CMake operations.

include(nuttx_parse_function_args)

# "nuttx_apps_interface" is a source-less target that encapsulates all the apps
# compiler options and include path needed by all apps libraries.
add_custom_target(nuttx_apps_interface)

# Macro: nuttx_library
#
# Creates a library target with the given name and mode. If MODE is "KERNEL", it
# calls nuttx_add_kernel_library instead.
#
# Usage: nuttx_library(mylib) nuttx_library(mylib MODE "KERNEL")
macro(nuttx_library name)
  cmake_parse_arguments(ARGS "" MODE "" ${ARGN})

  set(NX_CURRENT_LIBRARY ${name})

  if(NOT ARGS_MODE)
    nuttx_add_library(${name} STATIC)
  elseif("${ARGS_MODE}" STREQUAL "KERNEL")
    nuttx_add_kernel_library(${name})
  endif()
endmacro()

# Macro: nuttx_library_ifdef
#
# Conditionally creates a library target if the given condition is true.
#
# Usage: nuttx_library_ifdef(MY_CONDITION mylib)
macro(nuttx_library_ifdef cond name)
  if(${cond})
    nuttx_library(${name} ${ARGN})
  endif()
endmacro()

# Macro: nuttx_library_ifndef
#
# Conditionally creates a library target if the given condition is false.
#
# Usage: nuttx_library_ifndef(MY_CONDITION mylib)
macro(nuttx_library_ifndef cond name)
  if(NOT ${cond})
    nuttx_library(${name} ${ARGN})
  endif()
endmacro()

# Function: nuttx_sources
#
# Adds source files to the current library target.
#
# Usage: nuttx_sources(source1.cpp source2.cpp)
function(nuttx_sources)
  if(TARGET ${NX_CURRENT_LIBRARY})
    target_sources(${NX_CURRENT_LIBRARY} PRIVATE ${ARGN})
  endif()
endfunction()

# Function: nuttx_sources_ifdef
#
# Conditionally adds source files to the current library target if the given
# condition is true.
#
# Usage: nuttx_sources_ifdef(MY_CONDITION source1.cpp source2.cpp)
function(nuttx_sources_ifdef cond)
  if(${cond})
    nuttx_sources(${ARGN})
  endif()
endfunction()

# Function: nuttx_sources_ifndef
#
# Conditionally adds source files to the current library target if the given
# condition is false.
#
# Usage: nuttx_sources_ifndef(MY_CONDITION source1.cpp source2.cpp)
function(nuttx_sources_ifndef cond)
  if(NOT ${cond})
    nuttx_sources(${ARGN})
  endif()
endfunction()

# Function: nuttx_wildcard_sources
#
# Adds source files matching a wildcard pattern to the current library target,
# excluding those matching the exclude pattern.
#
# Usage: nuttx_wildcard_sources("*.c" EXCLUDE "exclude_me.c")
function(nuttx_wildcard_sources)
  cmake_parse_arguments(ARGS "" "" EXCLUDE ${ARGN})

  file(GLOB SRCS ${ARGN})
  if(ARGS_EXCLUDE)
    file(GLOB RM_SRCS ${ARGS_EXCLUDE})
    list(REMOVE_ITEM SRCS ${RM_SRCS})
  endif()
  nuttx_sources(${SRCS})
endfunction()

# Function: nuttx_include_directories
#
# Adds include directories to the current library target.
#
# Usage: nuttx_include_directories("include/path1" "include/path2")
function(nuttx_include_directories)
  if(TARGET ${NX_CURRENT_LIBRARY})
    target_include_directories(${NX_CURRENT_LIBRARY} PRIVATE ${ARGN})
  endif()
endfunction()

# Function: nuttx_include_directories_ifdef
#
# Conditionally adds include directories to the current library target if the
# given condition is true.
#
# Usage: nuttx_include_directories_ifdef(MY_CONDITION "include/path1"
# "include/path2")
function(nuttx_include_directories_ifdef cond)
  if(${cond})
    nuttx_include_directories(${ARGN})
  endif()
endfunction()

# Function: nuttx_include_directories_ifndef
#
# Conditionally adds include directories to the current library target if the
# given condition is false.
#
# Usage: nuttx_include_directories_ifndef(MY_CONDITION "include/path1"
# "include/path2")
function(nuttx_include_directories_ifndef cond)
  if(NOT ${cond})
    nuttx_include_directories(${ARGN})
  endif()
endfunction()

# Function: nuttx_compile_definitions
#
# Adds compile definitions to the current library target.
#
# Usage: nuttx_compile_definitions("DEF1" "DEF2")
function(nuttx_compile_definitions)
  if(TARGET ${NX_CURRENT_LIBRARY})
    target_compile_definitions(${NX_CURRENT_LIBRARY} PRIVATE ${ARGN})
  endif()
endfunction()

# Function: nuttx_compile_definitions_ifdef
#
# Conditionally adds compile definitions to the current library target if the
# given condition is true.
#
# Usage: nuttx_compile_definitions_ifdef(MY_CONDITION "DEF1" "DEF2")
function(nuttx_compile_definitions_ifdef cond)
  if(${cond})
    nuttx_compile_definitions(${ARGN})
  endif()
endfunction()

# Function: nuttx_compile_definitions_ifndef
#
# Conditionally adds compile definitions to the current library target if the
# given condition is false.
#
# Usage: nuttx_compile_definitions_ifndef(MY_CONDITION "DEF1" "DEF2")
function(nuttx_compile_definitions_ifndef cond)
  if(NOT ${cond})
    nuttx_compile_definitions(${ARGN})
  endif()
endfunction()

# Function: nuttx_compile_options
#
# Adds compile options to the current library target.
#
# Usage: nuttx_compile_options("-O2" "-Wall")
function(nuttx_compile_options)
  if(TARGET ${NX_CURRENT_LIBRARY})
    target_compile_options(${NX_CURRENT_LIBRARY} PRIVATE ${ARGN})
  endif()
endfunction()

# Function: nuttx_compile_options_ifdef
#
# Conditionally adds compile options to the current library target if the given
# condition is true.
#
# Usage: nuttx_compile_options_ifdef(MY_CONDITION "-O2" "-Wall")
function(nuttx_compile_options_ifdef cond)
  if(${cond})
    nuttx_compile_options(${ARGN})
  endif()
endfunction()

# Function: nuttx_compile_options_ifndef
#
# Conditionally adds compile options to the current library target if the given
# condition is false.
#
# Usage: nuttx_compile_options_ifndef(MY_CONDITION "-O2" "-Wall")
function(nuttx_compile_options_ifndef cond)
  if(NOT ${cond})
    nuttx_compile_options(${ARGN})
  endif()
endfunction()

# the visible scope is all the APPS include search path
function(nuttx_include_directories_for_all_apps)
  set_property(
    TARGET nuttx_apps_interface
    APPEND
    PROPERTY APPS_INCLUDE_DIRECTORIES ${ARGN})
endfunction()

# all apps compile_options
function(nuttx_compile_options_for_all_apps)
  set_property(
    TARGET nuttx_apps_interface
    APPEND
    PROPERTY APPS_COMPILE_OPTIONS ${ARGN})
endfunction()

# all apps compile_definitions
function(nuttx_compile_definitions_for_all_apps)
  set_property(
    TARGET nuttx_apps_interface
    APPEND
    PROPERTY APPS_COMPILE_DEFINITIONS ${ARGN})
endfunction()

# since we dont call `target_link_libraries` directly, we only inherit their
# compilation configuration
function(nuttx_link_libraries)
  set(TARGET ${ARGV0})
  if(TARGET ${TARGET})
    foreach(dep ${ARGN})
      target_compile_options(
        ${TARGET}
        PRIVATE
          $<GENEX_EVAL:$<TARGET_PROPERTY:${dep},INTERFACE_COMPILE_OPTIONS>>)
      target_compile_definitions(
        ${TARGET}
        PRIVATE
          $<GENEX_EVAL:$<TARGET_PROPERTY:${dep},INTERFACE_COMPILE_DEFINITIONS>>)
      target_include_directories(
        ${TARGET}
        PRIVATE
          $<GENEX_EVAL:$<TARGET_PROPERTY:${dep},INTERFACE_INCLUDE_DIRECTORIES>>)
    endforeach()
  endif()
endfunction()
