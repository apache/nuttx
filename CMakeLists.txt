# Request a version available on latest Ubuntu LTS (20.04)

cmake_minimum_required(VERSION 3.16)

# Basic CMake configuration

set(CMAKE_CXX_STANDARD 11)	# TODO: refine to 14?
set(CMAKE_C_STANDARD 99)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Setup build type (Debug Release RelWithDebInfo MinSizeRel Coverage). Default
# to minimum size release

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "MinSizeRel" CACHE STRING "Build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel")

# Define main directory locations

set(NUTTX_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(NUTTX_APPS_DIR "../apps" CACHE FILEPATH "Relative path to apps/ directory")

# Process board/config combination

set(NUTTX_CONFIG "nsh" CACHE STRING "Board confguration")
if (NOT DEFINED NUTTX_BOARD)
	message(FATAL_ERROR "Please define a board with NUTTX_BOARD and configuration with NUTTX_CONFIG")
endif()

set(BOARD_PATH "${NUTTX_DIR}/boards/${NUTTX_BOARD}")
if (NOT EXISTS "${BOARD_PATH}/CMakeLists.txt")
	message(FATAL_ERROR "No CMakeList.txt found for ${NUTTX_BOARD}")
endif()

set(NUTTX_DEFCONFIG ${BOARD_PATH}/configs/${NUTTX_CONFIG}/defconfig CACHE FILEPATH "path to defconfig" FORCE)

# Generate inital .config (needed so that we can source variables into build and configure)
# TODO: there's also a custom command for this, but it isn't useful since that means it happens
# after configure phase

set(ENV{APPSDIR} ${NUTTX_APPS_DIR}) # TODO: support not having apps/
set(ENV{EXTERNALDIR} dummy) # TODO
set(ENV{DRIVERS_PLATFORM_DIR} dummy) # TODO

execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DEFCONFIG} ${NUTTX_DIR}/.config
  WORKING_DIRECTORY ${NUTTX_DIR}
)

execute_process(
  COMMAND kconfig-conf --olddefconfig Kconfig
  WORKING_DIRECTORY ${NUTTX_DIR}
  OUTPUT_VARIABLE KCONFIG_OUTPUT
  RESULT_VARIABLE KCONFIG_STATUS
)

if (KCONFIG_STATUS AND NOT KCONFIG_STATUS EQUAL 0)
  message(FATAL_ERROR "Failed to initialize Kconfig configuration: ${KCONFIG_OUTPUT}")
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${NUTTX_DIR}/.config ${CMAKE_BINARY_DIR}/.config
  WORKING_DIRECTORY ${NUTTX_DIR}
)

# Include .cmake files

include(kconfig)

include(custom_commands)
include(set_parent_srcs)
include(nuttx_add_library)
include(menuconfig)

# Define toolchain, this needs to happen before project()
# TODO: move this under arch/

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/Toolchain-arm-none-eabi.cmake)

# Define project (this triggers configuration)

project(NuttX LANGUAGES ASM C CXX)

# Configure toolchain (this needs to happen after project(), once the toolchain
# file has been processed)
# TODO: move this under arch/

if("${CONFIG_ARCH_CORTEXM0}" STREQUAL "y")
  set(CMAKE_SYSTEM_PROCESSOR cortex-m0)
elseif("${CONFIG_ARCH_CORTEXM3}" STREQUAL "y")
  set(CMAKE_SYSTEM_PROCESSOR cortex-m3)
elseif("${CONFIG_ARCH_CORTEXM4}" STREQUAL "y")
  set(CMAKE_SYSTEM_PROCESSOR cortex-m4)
elseif("${CONFIG_ARCH_CORTEXM7}" STREQUAL "y")
  set(CMAKE_SYSTEM_PROCESSOR cortex-m7)
else()
  message(FATAL_ERROR "processor not set")
endif()

include(Platform/${CMAKE_SYSTEM_NAME}-${TOOLCHAIN_PREFIX}-gcc-${CMAKE_SYSTEM_PROCESSOR})

# Print configuration choices

message(STATUS "NuttX Configuration:")
message(STATUS "  Board: ${NUTTX_BOARD}")
message(STATUS "  Config: ${NUTTX_CONFIG}")
message(STATUS "  Appdir: ${NUTTX_APPS_DIR}")

# Setup top-level includes

include_directories(include)
include_directories(${CMAKE_BINARY_DIR}/include)

# Add child directories to build

add_subdirectory(arch)
add_subdirectory(audio)
add_subdirectory(binfmt)
add_subdirectory(drivers)
add_subdirectory(fs)
add_subdirectory(graphics)
add_subdirectory(libs)
add_subdirectory(mm)
add_subdirectory(net)
add_subdirectory(sched)
add_subdirectory(wireless)

# Add board directories

add_subdirectory(boards)

# Add apps/ and external/ to the build (if present)

if(EXISTS ${NUTTX_DIR}/${NUTTX_APPS_DIR}/CMakeLists.txt)
	add_subdirectory(${NUTTX_APPS_DIR} apps)
	
	execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink dummy ${NUTTX_APPS_DIR}/platform/board
                  WORKING_DIRECTORY ${NUTTX_APPS_DIR})
else()
  message(STATUS "Application directory not found at ${NUTTX_APPS_DIR}, skipping")
endif()

if (EXISTS ${CMAKE_SOURCE_DIR}/external/CMakeLists.txt)
	add_subdirectory(${CMAKE_SOURCE_DIR}/external)
endif()

# Compiler options
# TODO: move elsewhere

if ("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
  if(CMAKE_C_COMPILER_VERSION VERSION_GREATER 4.9)
    # force color for gcc > 4.9
    add_compile_options(-fdiagnostics-color=always)
  endif()
endif()


add_compile_options(
  -fno-builtin
  -fno-strength-reduce
  -fno-strict-aliasing

  -fomit-frame-pointer

  -Wno-cpp
)

# Link step

# TODO: add link step allowing the board to define the linker script 
get_property(nuttx_app_libs GLOBAL PROPERTY NUTTX_APP_LIBS)
get_property(nuttx_libs GLOBAL PROPERTY NUTTX_LIBRARIES)
get_property(ldscript GLOBAL PROPERTY LD_SCRIPT)

# create empty file to declare the nuttx target

file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/empty.c" "")
add_executable(nuttx "${CMAKE_CURRENT_BINARY_DIR}/empty.c")

target_link_libraries(nuttx PRIVATE
  --entry=__start

  -nostartfiles
  -nodefaultlibs

  -Wl,--script=${ldscript}
  #-Wl,-Map=${PX4_CONFIG}.map
  #-Wl,--warn-common
  #-Wl,--gc-sections

  -Wl,--start-group
    ${nuttx_libs}
    ${nuttx_app_libs}
  -Wl,--end-group

  gcc
)
