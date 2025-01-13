# ##############################################################################
# CMakeLists.txt
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

# ~~~
# Instructions:
# - Run CMake from the user project directory:
#   cmake -S <nuttx-dir> -B <build-directory> -DBOARD_CONFIG=<board>
#   - NuttX will look for the nuttx-apps repository from its parent folder
#   i.e., ../nuttx-apps.
#     - A custom directory can be specified with -DNUTTX_APPS_DIR=<apps-dir>.
# - Build the user project with:
#   cmake --build <build-dir>
# ~~~

# Request a version available on latest Ubuntu LTS (20.04)

cmake_minimum_required(VERSION 3.16)

# Handle newer CMake versions correctly by setting policies

if(POLICY CMP0115)
  # do not auto-guess extension in target_sources()
  cmake_policy(SET CMP0115 NEW)
endif()

# Avoid warning about DOWNLOAD_EXTRACT_TIMESTAMP in CMake 3.24:

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
  cmake_policy(SET CMP0135 NEW)
endif()

if(POLICY CMP0169)
  # allow to call FetchContent_Populate directly
  cmake_policy(SET CMP0169 OLD)
endif()

# Basic CMake configuration ##################################################

set(CMAKE_CXX_EXTENSIONS OFF)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Setup build type (Debug Release RelWithDebInfo MinSizeRel Coverage). Default
# to minimum size release

# Use nuttx optimization configuration options, workaround for cmake build type
# TODO Integration the build type with CMAKE

# if (NOT CMAKE_BUILD_TYPE) set(CMAKE_BUILD_TYPE "MinSizeRel" CACHE STRING
# "Build type" FORCE) endif() set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
# STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel")

# Process board config & directory locations #################################

set(NUTTX_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# unceaned previous make build can cause various types of cmake error
if(EXISTS "${NUTTX_DIR}/.config")
  message(
    FATAL_ERROR "Please distclean previous make build with `make distclean`")
endif()

if(NOT DEFINED BOARD_CONFIG)
  message(FATAL_ERROR "Please define configuration with BOARD_CONFIG")
endif()

find_program(KCONFIGLIB olddefconfig)
if(NOT KCONFIGLIB)
  # cmake-format: off
  message(
    FATAL_ERROR "Kconfig environment depends on kconfiglib, Please install:
  (APT source)
    $ sudo apt install python3-kconfiglib
  or (pip source)
    $ pip install kconfiglib
  or (After Ubuntu 24.04)
    $ pip install kconfiglib --break-system-packages")
  # cmake-format: on
endif()

# BOARD CONFIG can be set to directory path, or <board-name>[/:]<config-name>
# configuration pair

if((EXISTS ${BOARD_CONFIG} AND EXISTS ${BOARD_CONFIG}/defconfig)
   OR (EXISTS ${NUTTX_DIR}/${BOARD_CONFIG}
       AND EXISTS ${NUTTX_DIR}/${BOARD_CONFIG}/defconfig))
  get_filename_component(NUTTX_BOARD_ABS_DIR ${BOARD_CONFIG} ABSOLUTE BASE_DIR
                         ${NUTTX_DIR})

  string(REPLACE "/" ";" CONFIG_ARRAY ${NUTTX_BOARD_ABS_DIR})

  list(LENGTH CONFIG_ARRAY CONFIG_ARRAY_LENGTH)

  if(${CONFIG_ARRAY_LENGTH} LESS 4)
    message(FATAL_ERROR "Please define correct board config : ${BOARD_CONFIG}")
  endif()

  math(EXPR NUTTX_CONFIG_INDEX "${CONFIG_ARRAY_LENGTH} - 1")
  math(EXPR NUTTX_BOARD_INDEX "${CONFIG_ARRAY_LENGTH} - 3")
  list(GET CONFIG_ARRAY ${NUTTX_BOARD_INDEX} NUTTX_BOARD)
  list(GET CONFIG_ARRAY ${NUTTX_CONFIG_INDEX} NUTTX_CONFIG)

  string(REGEX REPLACE "(.*)/(.*)/${NUTTX_CONFIG}" "\\1" NUTTX_BOARD_DIR
                       ${NUTTX_BOARD_ABS_DIR})
  set(NUTTX_DEFCONFIG ${NUTTX_BOARD_ABS_DIR}/defconfig)
else()
  if(BOARD_CONFIG MATCHES "/")
    set(MATCH_REGEX "/")
  else()
    set(MATCH_REGEX ":")
  endif()

  string(REPLACE ${MATCH_REGEX} ";" CONFIG_ARRAY ${BOARD_CONFIG})

  list(LENGTH CONFIG_ARRAY CONFIG_ARRAY_LENGTH)

  if(${CONFIG_ARRAY_LENGTH} LESS 2)
    message(FATAL_ERROR "Please define correct board config : ${BOARD_CONFIG}")
  endif()

  list(GET CONFIG_ARRAY 0 NUTTX_BOARD)
  list(GET CONFIG_ARRAY 1 NUTTX_CONFIG)

  file(
    GLOB NUTTX_BOARD_DIR
    LIST_DIRECTORIES true
    "${NUTTX_DIR}/boards/*/*/${NUTTX_BOARD}")

  if(EXISTS ${NUTTX_BOARD_DIR}/configs/${NUTTX_CONFIG}/defconfig)
    set(NUTTX_DEFCONFIG ${NUTTX_BOARD_DIR}/configs/${NUTTX_CONFIG}/defconfig)
  endif()
endif()

if("${NUTTX_CONFIG}" STREQUAL "")
  message(FATAL_ERROR "Please define correct board config : ${NUTTX_CONFIG}")
endif()

if(NOT EXISTS "${NUTTX_DEFCONFIG}")
  message(FATAL_ERROR "No config file found at ${NUTTX_DEFCONFIG}")
endif()

# Generate inital .config ###################################################
# This is needed right before any other configure step so that we can source
# Kconfig variables into CMake variables

# The following commands need these variables to be passed via environment

include(nuttx_kconfig)
nuttx_export_kconfig_by_value(${NUTTX_DEFCONFIG} "CONFIG_APPS_DIR")

if((NOT NUTTX_APPS_DIR) AND (NOT CONFIG_APPS_DIR))
  if(EXISTS "${NUTTX_DIR}/../apps")
    set(NUTTX_APPS_DIR "${NUTTX_DIR}/../apps")
  elseif(EXISTS "${NUTTX_DIR}/../nuttx-apps")
    set(NUTTX_APPS_DIR "${NUTTX_DIR}/../nuttx-apps")
  else()
    message(
      WARNING
        "apps/nuttx-apps directory is not found, use dummy directory instead")
    set(NUTTX_APPS_DIR "${NUTTX_DIR}/dummy")
  endif()
else()
  set(NUTTX_APPS_DIR ${CONFIG_APPS_DIR})
  set(CONFIG_APPS_DIR)
endif()

if(NOT EXISTS "${NUTTX_APPS_DIR}")
  message(FATAL_ERROR "Application directory ${NUTTX_APPS_DIR} is not found")
endif()

get_filename_component(NUTTX_APPS_DIR ${NUTTX_APPS_DIR} ABSOLUTE)
get_filename_component(apps_dir ${NUTTX_APPS_DIR} NAME)
set(NUTTX_APPS_BINDIR "${CMAKE_BINARY_DIR}/${apps_dir}")

# Support not having application directory

if("${apps_dir}" STREQUAL "dummy")
  file(MAKE_DIRECTORY ${NUTTX_APPS_BINDIR})
  file(TOUCH ${NUTTX_APPS_BINDIR}/Kconfig)
endif()

set(ENV{PYTHONPYCACHEPREFIX} ${CMAKE_BINARY_DIR})
set(ENV{APPSDIR} ${NUTTX_APPS_DIR}) # TODO: support not having apps/
set(ENV{APPSBINDIR} ${NUTTX_APPS_BINDIR}) # TODO: support not having apps/
set(ENV{BINDIR} ${CMAKE_BINARY_DIR}) # TODO: support not having apps/
set(ENV{EXTERNALDIR} dummy) # TODO
set(ENV{DRIVERS_PLATFORM_DIR} dummy) # TODO

set(ENV{HOST_LINUX} n)
set(ENV{HOST_MACOS} n)
set(ENV{HOST_BSD} n)
set(ENV{HOST_WINDOWS} n)
set(ENV{HOST_OTHER} n)

include(nuttx_sethost)

include(nuttx_parse_function_args)
include(nuttx_add_subdirectory)
include(nuttx_create_symlink)

# Add apps/ to the build (if present)

if(NOT EXISTS ${NUTTX_APPS_BINDIR}/Kconfig)
  add_subdirectory(${NUTTX_APPS_DIR} preapps)
endif()

nuttx_export_kconfig(${NUTTX_DEFCONFIG})

if(CONFIG_ARCH_BOARD_CUSTOM)
  get_filename_component(NUTTX_BOARD_DIR ${CONFIG_ARCH_BOARD_CUSTOM_DIR}
                         ABSOLUTE BASE_DIR ${NUTTX_DIR})
endif()

if("${NUTTX_BOARD_DIR}" STREQUAL "")
  message(FATAL_ERROR "Please define correct board : ${NUTTX_BOARD_DIR}")
endif()

if(NOT EXISTS "${NUTTX_BOARD_DIR}/CMakeLists.txt"
   AND NOT EXISTS "${NUTTX_BOARD_DIR}/../common/CMakeLists.txt")
  message(FATAL_ERROR "No CMakeLists.txt found at ${NUTTX_BOARD_DIR}")
endif()

# Custom board ###################################################

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/boards/dummy)
if(CONFIG_ARCH_BOARD_CUSTOM)
  get_filename_component(NUTTX_BOARD_ABS_DIR ${CONFIG_ARCH_BOARD_CUSTOM_DIR}
                         ABSOLUTE BASE_DIR ${NUTTX_DIR})
else()
  set(NUTTX_BOARD_ABS_DIR ${NUTTX_BOARD_DIR})
  file(TOUCH ${CMAKE_BINARY_DIR}/boards/dummy/Kconfig)
endif()

if(NOT EXISTS ${CMAKE_BINARY_DIR}/boards/dummy/Kconfig)
  if(CONFIG_ARCH_BOARD_CUSTOM AND EXISTS ${NUTTX_BOARD_ABS_DIR}/Kconfig)
    nuttx_create_symlink(${NUTTX_BOARD_ABS_DIR}/Kconfig
                         ${CMAKE_BINARY_DIR}/boards/dummy/Kconfig)
  else()
    file(TOUCH ${CMAKE_BINARY_DIR}/boards/dummy/Kconfig)
  endif()
endif()

# board platfrom driver

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/drivers)

if(EXISTS ${NUTTX_BOARD_ABS_DIR}/../drivers
   AND EXISTS ${NUTTX_BOARD_ABS_DIR}/../drivers/Kconfig)
  nuttx_create_symlink(${NUTTX_BOARD_ABS_DIR}/../drivers
                       ${CMAKE_BINARY_DIR}/drivers/platform)
else()
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/drivers/platform)
  file(TOUCH ${CMAKE_BINARY_DIR}/drivers/platform/Kconfig)
endif()

# Custom chip ###################################################

if(CONFIG_ARCH_CHIP_CUSTOM)
  get_filename_component(NUTTX_CHIP_ABS_DIR ${CONFIG_ARCH_CHIP_CUSTOM_DIR}
                         ABSOLUTE BASE_DIR ${NUTTX_DIR})
  set(NUTTX_CHIP_ABS_DIR ${NUTTX_CHIP_ABS_DIR})
else()
  set(NUTTX_CHIP_ABS_DIR
      "${NUTTX_DIR}/arch/${CONFIG_ARCH}/src/${CONFIG_ARCH_CHIP}")
endif()

if(NOT EXISTS ${CMAKE_BINARY_DIR}/arch/dummy)
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/arch/dummy)
endif()

if(NOT EXISTS ${CMAKE_BINARY_DIR}/arch/dummy/Kconfig)
  if(CONFIG_ARCH_CHIP_CUSTOM AND EXISTS ${NUTTX_CHIP_ABS_DIR}/Kconfig)
    nuttx_create_symlink(${NUTTX_CHIP_ABS_DIR}/Kconfig
                         ${CMAKE_BINARY_DIR}/arch/dummy/Kconfig)
  else()
    file(TOUCH ${CMAKE_BINARY_DIR}/arch/dummy/Kconfig)
  endif()
endif()

if(NOT EXISTS ${CMAKE_BINARY_DIR}/arch/${CONFIG_ARCH})
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/arch/${CONFIG_ARCH})
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/arch/${CONFIG_ARCH}/src)
endif()

if(NOT EXISTS ${CMAKE_BINARY_DIR}/arch/${CONFIG_ARCH}/src/chip)
  nuttx_create_symlink(${NUTTX_CHIP_ABS_DIR}
                       ${CMAKE_BINARY_DIR}/arch/${CONFIG_ARCH}/src/chip)
endif()

# Unsupport custom board/chips yet, workaround

if(NOT EXISTS ${NUTTX_APPS_BINDIR}/platform/board/Kconfig)
  file(MAKE_DIRECTORY ${NUTTX_APPS_BINDIR}/platform/board)
  file(TOUCH ${NUTTX_APPS_BINDIR}/platform/board/Kconfig)
endif()

# Copy board defconfig into main directory and expand TODO: do also for changes
# in board/config (by comparing stored defconfig to specified one)

if(NOT EXISTS ${CMAKE_BINARY_DIR}/.config OR NOT "${NUTTX_DEFCONFIG}" STREQUAL
                                             "${NUTTX_DEFCONFIG_SAVED}")

  message(STATUS "Initializing NuttX")
  configure_file(${NUTTX_DEFCONFIG} defconfig COPYONLY)
  configure_file(${NUTTX_DEFCONFIG} .config.compressed COPYONLY)

  set(ENV{KCONFIG_CONFIG} ${CMAKE_BINARY_DIR}/.config.compressed)

  # Do olddefconfig step to expand the abbreviated defconfig into normal config
  nuttx_olddefconfig()

  file(RENAME ${CMAKE_BINARY_DIR}/.config.compressed
       ${CMAKE_BINARY_DIR}/.config)
  set(ENV{KCONFIG_CONFIG} ${CMAKE_BINARY_DIR}/.config)

  # store original expanded .config
  configure_file(${CMAKE_BINARY_DIR}/.config ${CMAKE_BINARY_DIR}/.config.orig
                 COPYONLY)

  # We define host
  nuttx_sethost()

  set(NUTTX_DEFCONFIG_SAVED
      ${NUTTX_DEFCONFIG}
      CACHE INTERNAL "Saved defconfig path" FORCE)

  # Print configuration choices
  message(STATUS "  CMake:  ${CMAKE_VERSION}")
  if(CMAKE_GENERATOR MATCHES "Ninja")
    execute_process(
      COMMAND ninja --version
      OUTPUT_VARIABLE ninja_version
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    message(STATUS "  Ninja:  ${ninja_version}")

    # Ninja generator does not yet know how to build archives in pieces, so
    # response files must be used to deal with very long linker command lines.
    set(CMAKE_NINJA_FORCE_RESPONSE_FILE
        1
        CACHE INTERNAL "")
  endif()
  message(STATUS "  Board:  ${NUTTX_BOARD}")
  message(STATUS "  Config: ${NUTTX_CONFIG}")
  message(STATUS "  Appdir: ${NUTTX_APPS_DIR}")
endif()

# Include .cmake files #######################################################

# this exposes all Kconfig vars to CMake

nuttx_export_kconfig(${CMAKE_BINARY_DIR}/.config)

include(nuttx_generate_headers)
include(nuttx_generate_outputs)
include(nuttx_add_library)

# add NuttX CMake extenstion after nuttx_add_library
include(nuttx_extensions)

include(nuttx_add_application)
include(nuttx_add_romfs)
include(nuttx_add_symtab)
include(nuttx_add_module)
include(nuttx_add_dependencies)
include(nuttx_export_header)
include(nuttx_remove_compile_options)
include(nuttx_source_file_properties)
include(menuconfig)

include(ExternalProject)
include(FetchContent)

set(FETCHCONTENT_QUIET OFF)

# Board common directory #####################################################

if(CONFIG_ARCH_BOARD_COMMON)
  file(
    GLOB NUTTX_COMMON_DIR
    LIST_DIRECTORIES true
    "${NUTTX_DIR}/boards/${CONFIG_ARCH}/${CONFIG_ARCH_CHIP}/common")
endif()

# Setup toolchain ############################################################

# This needs to happen before project() when binaries are searched for

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/arch/${CONFIG_ARCH}/src/cmake)
set(CMAKE_TOOLCHAIN_FILE
    "${CMAKE_SOURCE_DIR}/arch/${CONFIG_ARCH}/src/cmake/Toolchain.cmake")

# Define project #############################################################
# This triggers configuration

project(NuttX LANGUAGES C CXX ASM)

# include common toolchain setting
include(nuttx_toolchain)

if(WIN32)
  enable_language(ASM_MASM)
endif()

# Setup platform options (this needs to happen after project(), once the
# toolchain file has been processed)

# Support custom Toolchain options by custom Boards
if(CONFIG_ARCH_BOARD_CUSTOM)
  if(EXISTS ${NUTTX_BOARD_ABS_DIR}/cmake
     AND EXISTS ${NUTTX_BOARD_ABS_DIR}/cmake/Toolchain.cmake)
    # must be added AFTER ToolchainFile and BEFORE platform
    include(${NUTTX_BOARD_ABS_DIR}/cmake/Toolchain.cmake)
  endif()
endif()

include(platform)

# Setup main nuttx target ####################################################

add_executable(nuttx)
add_custom_target(nuttx_post)
if(CONFIG_BUILD_PROTECTED)
  add_executable(nuttx_user)
  add_dependencies(nuttx_post nuttx_user)
  nuttx_add_library_internal(nuttx_user)
endif()

if(CONFIG_ALLSYMS OR CONFIG_MM_KASAN_GLOBAL)
  include(nuttx_multiple_link)
endif()

add_dependencies(nuttx nuttx_context)
add_dependencies(nuttx_post nuttx)

if(WIN32)
  set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT
                                                              nuttx)
endif()

if(CONFIG_ARCH_SIM)
  # Create separate sim_head OBJECT library built as part of NuttX kernel It
  # must be separated to allow for linking against the rest of NuttX libraries

  add_library(sim_head OBJECT)
  nuttx_add_library_internal(sim_head)
  get_property(
    definitions
    TARGET nuttx
    PROPERTY NUTTX_KERNEL_DEFINITIONS)
  target_compile_definitions(sim_head PRIVATE ${definitions})

  get_property(
    options
    TARGET nuttx
    PROPERTY NUTTX_KERNEL_COMPILE_OPTIONS)
  target_compile_options(sim_head PRIVATE ${options})
  target_compile_options(sim_head PRIVATE -fvisibility=default)

  # We need the relocatable object to be first in the list of libraries to be
  # linked against final nuttx binary

  if(NOT WIN32)
    target_link_libraries(nuttx PRIVATE ${CMAKE_BINARY_DIR}/nuttx.rel)
  endif()
else()
  # These flags apply to source files not part of the library. In sim build this
  # corresponds to "host" files, so we only do this on non-sim build
  target_compile_definitions(
    nuttx
    PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_COMPILE_DEFINITIONS>>)
  target_compile_options(
    nuttx PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_COMPILE_OPTIONS>>)
endif()

if(MSVC)
  add_compile_options(
    -W2
    -wd4116 # unnamed type definition in parentheses
    -wd4146 # unary minus operator applied to unsigned type, result still
            # unsigned
    -wd4244 # 'argument' : conversion from 'type1' to 'type2', possible loss of
            # data
    -wd4305 # 'context' : truncation from 'type1' to 'type2'
  )
elseif(NOT CONFIG_ARCH_TOOLCHAIN_TASKING)
  add_compile_options(
    # system wide warnings
    -Wall $<$<COMPILE_LANGUAGE:C>:-Wstrict-prototypes> -Wshadow -Wundef
    # system wide options
    $<$<COMPILE_LANGUAGE:ASM>:-D__ASSEMBLY__>)
endif()

if(CONFIG_NDEBUG)
  add_compile_options(-DNDEBUG)
endif()

# Cmake build provide absolute paths to compile files. If __FILE__ macros are
# used in the source code(ASSERT), the binary will contain many invalid paths.
# This saves some memory, stops exposing build systems locations in binaries,
# make failure logs more deterministic and most importantly makes builds more
# failure logs more deterministic and most importantly makes builds more
# deterministic. Debuggers usually have a path mapping feature to ensure the
# files are still found.
if(NOT MSVC)
  if(CONFIG_OUTPUT_STRIP_PATHS)
    add_compile_options(-fmacro-prefix-map=${NUTTX_DIR}=)
    add_compile_options(-fmacro-prefix-map=${NUTTX_APPS_DIR}=)
    add_compile_options(-fmacro-prefix-map=${NUTTX_BOARD_ABS_DIR}=)
    add_compile_options(-fmacro-prefix-map=${NUTTX_CHIP_ABS_DIR}=)
  endif()
endif()

add_definitions(-D__NuttX__)

add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-D__ASSEMBLY__>)

set_property(
  TARGET nuttx
  APPEND
  PROPERTY NUTTX_KERNEL_DEFINITIONS __KERNEL__)

# Recurse subdirectories #####################################################

# Each subdirectory will generate a static library

if(CONFIG_OPENAMP)
  include_directories(${CMAKE_SOURCE_DIR}/openamp/open-amp/lib/include)
endif()

add_subdirectory(openamp)
add_subdirectory(arch)
add_subdirectory(audio)
add_subdirectory(binfmt)
add_subdirectory(crypto)
add_subdirectory(drivers)
add_subdirectory(fs)
add_subdirectory(graphics)
add_subdirectory(libs)
add_subdirectory(mm)
add_subdirectory(net)
add_subdirectory(sched)
add_subdirectory(syscall)
add_subdirectory(wireless)

# This picks up the chosen board (as well as common board code)

add_subdirectory(boards)

# POSTBUILD -- Perform post build operations Some architectures require the use
# of special tools and special handling AFTER building the NuttX binary.
# Make.defs files for those architectures should override the following define
# with the correct operations for that platform

if(TARGET nuttx_post_build)
  # ensure thae the custom post build is executed after all steps of the nuttx
  # build are completed
  add_dependencies(nuttx_post_build nuttx_post)
  add_custom_target(post_build ALL DEPENDS nuttx_post_build)
endif()

# Add apps/ to the build (if present)

if(NOT CONFIG_BUILD_KERNEL)

  if(EXISTS ${NUTTX_APPS_DIR}/CMakeLists.txt)
    add_subdirectory(${NUTTX_APPS_DIR} apps)
  else()
    message(
      STATUS "Application directory not found at ${NUTTX_APPS_DIR}, skipping")
  endif()

endif()

# after we traverse all build directories unify all target dependencies and all
# romfs target
process_all_target_dependencies()
process_all_directory_romfs()

# Link step ##################################################################

# Get linker script to use
get_property(ldscript GLOBAL PROPERTY LD_SCRIPT)

# Pre-compile linker script
if(NOT CONFIG_ARCH_SIM AND NOT CONFIG_ARCH_TOOLCHAIN_TASKING)
  get_filename_component(LD_SCRIPT_NAME ${ldscript} NAME)
  set(LD_SCRIPT_TMP "${CMAKE_BINARY_DIR}/${LD_SCRIPT_NAME}.tmp")

  nuttx_generate_preprocess_target(SOURCE_FILE ${ldscript} TARGET_FILE
                                   ${LD_SCRIPT_TMP})

  add_custom_target(ldscript_tmp DEPENDS ${LD_SCRIPT_TMP})
  add_dependencies(nuttx ldscript_tmp)

  set(ldscript ${LD_SCRIPT_TMP})
endif()

# Perform link

# Add empty source file to nuttx target since cmake requires at least one file
# and we will only be linking libraries

if(CONFIG_HAVE_CXX)
  file(TOUCH "${CMAKE_CURRENT_BINARY_DIR}/empty.cxx")
  target_sources(nuttx PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/empty.cxx")
else()
  file(TOUCH "${CMAKE_CURRENT_BINARY_DIR}/empty.c")
  target_sources(nuttx PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/empty.c")
endif()

# initialize manifest to hold all generated files
file(TOUCH ${CMAKE_BINARY_DIR}/nuttx.manifest)

get_property(nuttx_kernel_libs GLOBAL PROPERTY NUTTX_KERNEL_LIBRARIES)
get_property(nuttx_extra_libs GLOBAL PROPERTY NUTTX_EXTRA_LIBRARIES)

if(CONFIG_BUILD_FLAT)
  get_property(nuttx_system_libs GLOBAL PROPERTY NUTTX_SYSTEM_LIBRARIES)
  get_property(nuttx_apps_libs GLOBAL PROPERTY NUTTX_APPS_LIBRARIES)
endif()

set(nuttx_libs ${nuttx_kernel_libs} ${nuttx_system_libs} ${nuttx_apps_libs}
               ${nuttx_extra_libs})

if(NOT CONFIG_ARCH_SIM)
  # TODO: nostart/nodefault not applicable to nuttx toolchain
  if(CONFIG_ARCH_TOOLCHAIN_TASKING)
    target_link_libraries(nuttx PRIVATE --lsl-file=${ldscript} ${nuttx_libs})
    # Remove executable suffix
    set(CMAKE_EXECUTABLE_SUFFIX "")
  else()
    target_link_libraries(
      nuttx
      PRIVATE ${NUTTX_EXTRA_FLAGS}
              -T
              ${ldscript}
              $<$<NOT:$<BOOL:${DISABLE_LINK_GROUP}>>:-Wl,--start-group>
              ${nuttx_libs}
              $<$<NOT:$<BOOL:${DISABLE_LINK_GROUP}>>:-Wl,--end-group>)
  endif()

  # generate binary outputs in different formats (.bin, .hex, etc)
  nuttx_generate_outputs(nuttx)

  if(CONFIG_UBOOT_UIMAGE)
    add_custom_command(
      OUTPUT uImage
      COMMAND
        ${MKIMAGE} -A ${CONFIG_ARCH} -O linux -C none -T kernel -a
        ${CONFIG_UIMAGE_LOAD_ADDRESS} -e ${CONFIG_UIMAGE_ENTRY_POINT} -n nuttx
        -d nuttx.bin uImage
      DEPENDS nuttx)
    add_custom_target(nuttx-uImage ALL DEPENDS uImage)

    # TODO: install? $(Q) if [ -w /tftpboot ] ; then \ cp -f uImage
    # /tftpboot/uImage; \ fi
    file(APPEND ${CMAKE_BINARY_DIR}/nuttx.manifest uImage)
  endif()
elseif(WIN32)
  target_link_options(nuttx PUBLIC /SAFESEH:NO)
  math(EXPR LINK_STACKSIZE
       "${CONFIG_SIM_STACKSIZE_ADJUSTMENT} + ${CONFIG_IDLETHREAD_STACKSIZE}"
       OUTPUT_FORMAT DECIMAL)
  target_link_options(nuttx PUBLIC /STACK:${LINK_STACKSIZE},${LINK_STACKSIZE})
  if("${CMAKE_GENERATOR_PLATFORM}" STREQUAL "x64")
    target_link_options(nuttx PUBLIC /LARGEADDRESSAWARE:NO)
  endif()
  set(nuttx_libs_paths)
  foreach(lib ${nuttx_libs})
    list(APPEND nuttx_libs_paths $<TARGET_FILE:${lib}>)
  endforeach()

  add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/nuttx_all.lib
    COMMAND ${CMAKE_AR} /OUT:${CMAKE_BINARY_DIR}/nuttx_all.lib
            ${nuttx_libs_paths}
    DEPENDS ${nuttx_libs}
    VERBATIM)
  add_custom_target(nuttx_all-lib DEPENDS ${CMAKE_BINARY_DIR}/nuttx_all.lib)
  add_dependencies(nuttx nuttx_all-lib)
  target_link_libraries(nuttx PRIVATE $<TARGET_OBJECTS:sim_head>
                                      ${CMAKE_BINARY_DIR}/nuttx_all.lib)
else()

  if(NOT APPLE)
    # generate SIM ld.script for cheat C++ global construction
    include(nuttx_generate_sim_ld)
  endif()

  # On sim platform the link step is a little different. NuttX is first built
  # into a partially linked relocatable object nuttx.rel with no interface to
  # host OS. Then, the names of symbols that conflict with libc symbols are
  # renamed. The final nuttx binary is built by linking the host-specific
  # objects with the relocatable binary.
  include(nuttx_redefine_symbols)

  # TODO: do with single function call?
  set(nuttx_libs_paths)
  foreach(lib ${nuttx_libs})
    list(APPEND nuttx_libs_paths $<TARGET_FILE:${lib}>)
  endforeach()

  if(APPLE)
    add_custom_command(
      OUTPUT nuttx.rel
      COMMAND
        ${CMAKE_LINKER} ARGS -r $<$<BOOL:${CONFIG_SIM_M32}>:-m32>
        $<TARGET_OBJECTS:sim_head> $<$<NOT:$<BOOL:${APPLE}>>:-Wl,--start-group>
        ${nuttx_libs_paths} $<$<NOT:$<BOOL:${APPLE}>>:-Wl,--end-group> -o
        nuttx.rel
      DEPENDS ${nuttx_libs} sim_head
      COMMAND_EXPAND_LISTS)
  else()
    add_custom_command(
      OUTPUT nuttx.rel
      COMMAND
        ${CMAKE_C_COMPILER} ARGS -r $<$<BOOL:${CONFIG_SIM_M32}>:-m32>
        $<$<BOOL:${CONFIG_HOST_LINUX}>:-Wl,-z,noexecstack>
        $<TARGET_OBJECTS:sim_head> $<$<NOT:$<BOOL:${APPLE}>>:-Wl,--start-group>
        ${nuttx_libs_paths} $<$<NOT:$<BOOL:${APPLE}>>:-Wl,--end-group> -o
        nuttx.rel
      COMMAND ${CMAKE_OBJCOPY} --redefine-syms=nuttx-names.dat nuttx.rel
      DEPENDS ${nuttx_libs} sim_head sim_redefine_symbols
      COMMAND_EXPAND_LISTS)
  endif()
  add_custom_target(nuttx-rel DEPENDS nuttx.rel
                                      $<$<NOT:$<BOOL:${APPLE}>>:nuttx.ld>)

  # link the final nuttx binary
  add_dependencies(nuttx nuttx-rel)
  target_link_options(nuttx PUBLIC $<$<NOT:$<BOOL:${APPLE}>>:-T nuttx.ld>
                      $<$<BOOL:${CONFIG_SIM_M32}>:-m32>)
endif()

# TODO: if we use an install target a manifest may not be needed
if(CONFIG_ARCH_SIM)
  file(APPEND ${CMAKE_BINARY_DIR}/nuttx.manifest "nuttx\n")
endif()

# Generate system map using the compiler toolchain. Conventionally, the tool
# which dump symbols are called nm, though, some compiler toolchain may have a
# different name.
if(NOT CMAKE_HOST_WIN32)
  add_custom_command(
    OUTPUT System.map
    COMMAND
      ${CMAKE_NM} nuttx | grep -v
      '\(compiled\)\|\(\${CMAKE_C_OUTPUT_EXTENSION}$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)'
      | sort > System.map
    DEPENDS nuttx)
  add_custom_target(systemmap ALL DEPENDS System.map)
  add_dependencies(nuttx_post systemmap)
endif()

# Userspace portion ##########################################################

if(CONFIG_BUILD_PROTECTED)

  get_property(nuttx_system_libs GLOBAL PROPERTY NUTTX_SYSTEM_LIBRARIES)

  get_property(nuttx_apps_libs GLOBAL PROPERTY NUTTX_APPS_LIBRARIES)

  get_property(nuttx_user_extra_libs GLOBAL PROPERTY NUTTX_USER_EXTRA_LIBRARIES)

  get_property(user_ldscript GLOBAL PROPERTY LD_SCRIPT_USER)

  # Pre-compile linker script
  get_filename_component(LD_SCRIPT_USER_NAME ${user_ldscript} NAME)
  set(LD_SCRIPT_USER_TMP "${CMAKE_BINARY_DIR}/${LD_SCRIPT_USER_NAME}.tmp")
  nuttx_generate_preprocess_target(SOURCE_FILE ${user_ldscript} TARGET_FILE
                                   ${LD_SCRIPT_USER_TMP})
  add_custom_target(user_ldscript_tmp DEPENDS ${LD_SCRIPT_USER_TMP})
  add_dependencies(nuttx_user user_ldscript_tmp)
  set(user_ldscript ${LD_SCRIPT_USER_TMP})

  # reset link options that don't fit userspace
  get_target_property(nuttx_user_LINK_OPTIONS nuttx_user LINK_OPTIONS)
  list(REMOVE_ITEM nuttx_user_LINK_OPTIONS "-Wl,--cref")
  list(REMOVE_ITEM nuttx_user_LINK_OPTIONS "-Wl,-Map=nuttx.map")
  if(CONFIG_ARCH_TOOLCHAIN_GHS)
    list(REMOVE_ITEM nuttx_user_LINK_OPTIONS "-entry=__start")
    list(REMOVE_ITEM nuttx_user_LINK_OPTIONS "-map=nuttx.map")
    list(APPEND nuttx_user_LINK_OPTIONS "-map=nuttx_user.map")
  else()
    list(REMOVE_ITEM nuttx_user_LINK_OPTIONS "-Wl,--entry=__start")
    list(APPEND nuttx_user_LINK_OPTIONS "-Wl,-Map=nuttx_user.map")
  endif()

  set_target_properties(nuttx_user PROPERTIES LINK_OPTIONS
                                              "${nuttx_user_LINK_OPTIONS}")

  if(CONFIG_ARCH_TOOLCHAIN_GHS)
    target_link_options(nuttx_user PRIVATE -nostartfiles -minlib
                        -entry=${CONFIG_INIT_ENTRYPOINT})
  else()
    target_link_options(
      nuttx_user PRIVATE -nostartfiles -nodefaultlibs
      -Wl,--entry=${CONFIG_INIT_ENTRYPOINT}
      -Wl,--undefined=${CONFIG_INIT_ENTRYPOINT})
  endif()

  target_link_libraries(
    nuttx_user
    PRIVATE -T
            ${user_ldscript}
            $<$<NOT:$<BOOL:${DISABLE_LINK_GROUP}>>:-Wl,--start-group>
            ${nuttx_system_libs}
            ${nuttx_apps_libs}
            ${nuttx_user_extra_libs}
            $<$<BOOL:${CONFIG_HAVE_CXX}>:supc++>
            $<$<NOT:$<BOOL:${DISABLE_LINK_GROUP}>>:-Wl,--end-group>)

  target_include_directories(
    nuttx_user SYSTEM
    PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_BINARY_DIR}/include
            ${CMAKE_BINARY_DIR}/include_arch)

  add_custom_command(
    OUTPUT User.map
    COMMAND ${CMAKE_NM} nuttx_user > User.map
    DEPENDS nuttx_user)
  add_custom_target(usermap ALL DEPENDS User.map)
  add_dependencies(nuttx_post usermap)

  # generate binary outputs in different formats (.bin, .hex, etc)
  nuttx_generate_outputs(nuttx_user)

  # create merged .hex file ready to be flashed TODO: does not seem to be
  # generating a functional hex file
  if(CONFIG_INTELHEX_BINARY AND SREC_CAT)
    add_custom_command(
      OUTPUT nuttx_combined.hex
      COMMAND ${SREC_CAT} nuttx.hex -intel nuttx_user.hex -intel -o
              nuttx_combined.hex -intel
      DEPENDS nuttx_user nuttx)
    add_custom_target(nuttx-combined ALL DEPENDS nuttx_combined.hex)
  endif()

  # TODO: could also merge elf binaries
endif()

if(CONFIG_BUILD_KERNEL)
  # TODO: generate nuttx-export-xxx.tar.gz for userland development

endif()
