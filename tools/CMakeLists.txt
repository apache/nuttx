# ##############################################################################
# tools/CMakeLists.txt
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

# Configure project
cmake_minimum_required(VERSION 3.16)
project(nuttx_tools LANGUAGES C)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      "Release"
      CACHE STRING "Build type" FORCE)
endif()

option(NUTTX_INCLUDE_ALL_TOOLS "Build all NuttX Host Tools" ON)
option(NUTTX_INCLUDE_CONFIGURE "Build configure" OFF)
option(NUTTX_INCLUDE_GENCROMFS "Build gencromfs" OFF)
option(NUTTX_INCLUDE_INCDIR "Build incdir" OFF)
option(NUTTX_INCLUDE_MKCONFIG "Build mkconfig" OFF)
option(NUTTX_INCLUDE_MKDEPS "Build mkdeps" OFF)
option(NUTTX_INCLUDE_MKSYMTAB "Build mksymtab" OFF)
option(NUTTX_INCLUDE_MKSYSCALL "Build mksyscall" OFF)
option(NUTTX_INCLUDE_MKVERSION "Build mkversion" OFF)
option(NUTTX_INCLUDE_NXSTYLE "Build nxstyle" OFF)

message(STATUS "NuttX Host Tools")

message(STATUS "CMake C compiler: ${CMAKE_C_COMPILER_ID}")
message(STATUS "CMake system name: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMake host system processor: ${CMAKE_HOST_SYSTEM_PROCESSOR}")

if(TOOLS_DIR)
  message("   TOOLS_DIR path is \"${TOOLS_DIR}\"")
else()
  set(TOOLS_PATH ${CMAKE_CURRENT_SOURCE_DIR})
  cmake_path(GET TOOLS_PATH PARENT_PATH TOOLS_DIR)

  message("   TOOLS_DIR path is \"${TOOLS_DIR}\"")
endif()

# set basic warnings

add_compile_options(-Wall -Wstrict-prototypes -Wshadow -Wundef)

# configure according to platform

if(CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
  message("   HOST = WINDOWS NATIVE")
  add_compile_definitions(CONFIG_WINDOWS_NATIVE=y)
else()
  # GCC or clang is assumed in all other POSIX environments (Linux, Cygwin,
  # MSYS2, macOS). strtok_r and strndup is used in some tools, but does not seem
  # to be available in the MinGW environment.

  add_compile_definitions(HAVE_STRTOK_C=1)
  add_compile_definitions(HAVE_STRNDUP=1)

  if(CMAKE_HOST_SYSTEM_NAME MATCHES "Linux")
    message("   HOST = Linux")
  elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "Darwin")
    message("   HOST = Darwin")
  elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "FreeBSD")
    message("   HOST = FreeBSD")
  elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "CYGWIN")
    message("   HOST = WINDOWS CYGWIN")
    add_compile_definitions(HOST_CYGWIN=1)
  elseif(CMAKE_HOST_SYSTEM_NAME MATCHES "MSYS")
    message("   HOST = WINDOWS MSYS")
  endif()
endif()

# define targets

# ============================================================================
# configure binaries Configure a work-alike program as a replacement for
# configure.sh
# ============================================================================

if(NUTTX_INCLUDE_ALL_TOOLS OR NUTTX_INCLUDE_CONFIGURE)
  add_executable(configure configure.c cfgparser.c)
  install(TARGETS configure DESTINATION bin)
endif()

# ============================================================================
# gencromfs binaries Generate a CROMFS file system images
# ============================================================================

if(NUTTX_INCLUDE_ALL_TOOLS OR NUTTX_INCLUDE_GENCROMFS)
  add_executable(gencromfs gencromfs.c)
  if(CMAKE_HOST_SYSTEM_NAME MATCHES "Windows")
    target_compile_definitions(gencromfs PRIVATE _POSIX_)
  endif()
  install(TARGETS gencromfs DESTINATION bin)
endif()

# ============================================================================
# incdir binaries Generate compiler-specific include paths
# ============================================================================

if(NUTTX_INCLUDE_ALL_TOOLS OR NUTTX_INCLUDE_INCDIR)
  add_executable(incdir incdir.c)
  install(TARGETS incdir DESTINATION bin)
endif()

# ============================================================================
# mkconfig binaries Convert a .config file into a C config.h file
# ============================================================================

if(NUTTX_INCLUDE_ALL_TOOLS OR NUTTX_INCLUDE_MKCONFIG)
  add_executable(mkconfig mkconfig.c cfgdefine.c)
  install(TARGETS mkconfig DESTINATION bin)
endif()

# ============================================================================
# mkdeps binaries Create dependencies for a list of files
# ============================================================================

if(NUTTX_INCLUDE_ALL_TOOLS OR NUTTX_INCLUDE_MKDEPS)
  add_executable(mkdeps mkdeps.c cfgdefine.c)
  install(TARGETS mkdeps DESTINATION bin)
endif()

# ============================================================================
# mksyscall binaries Convert a CSV file into syscall stubs and proxies
# ============================================================================

if(NUTTX_INCLUDE_ALL_TOOLS OR NUTTX_INCLUDE_MKSYSCALL)
  add_executable(mksyscall mksyscall.c csvparser.c)
  install(TARGETS mksyscall DESTINATION bin)
endif()

# ============================================================================
# mksymtab binaries Convert a CSV file into a symbol table
# ============================================================================

if(NUTTX_INCLUDE_ALL_TOOLS OR NUTTX_INCLUDE_MKSYMTAB)
  add_executable(mksymtab mksymtab.c csvparser.c)
  install(TARGETS mksymtab DESTINATION bin)
endif()

# ============================================================================
# mkversion binaries Convert a .version file into a C version.h file
# ============================================================================

if(NUTTX_INCLUDE_ALL_TOOLS OR NUTTX_INCLUDE_MKVERSION)
  add_executable(mkversion mkversion.c cfgdefine.c)
  install(TARGETS mkversion DESTINATION bin)
endif()

# ============================================================================
# nxstyle binaries Check a file for compliance to NuttX coding style
# ============================================================================

if(NUTTX_INCLUDE_ALL_TOOLS OR NUTTX_INCLUDE_NXSTYLE)
  add_executable(nxstyle nxstyle.c)
  target_compile_definitions(nxstyle PRIVATE TOOLS_DIR=${TOOLS_DIR})
  install(TARGETS nxstyle DESTINATION bin)
endif()
