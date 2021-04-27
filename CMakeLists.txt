# Request a version available on latest Ubuntu LTS (20.04)

cmake_minimum_required(VERSION 3.16)

# Basic CMake configuration ##################################################

set(CMAKE_CXX_STANDARD 11)	# TODO: refine to 14?
#set(CMAKE_C_STANDARD 99)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Setup build type (Debug Release RelWithDebInfo MinSizeRel Coverage). Default
# to minimum size release

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "MinSizeRel" CACHE STRING "Build type" FORCE)
endif()
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug;Release;RelWithDebInfo;MinSizeRel")

# TODO: extra flags to defaults per config?

#ifneq ($(CONFIG_DEBUG_NOOPT),y)
#  ARCHOPTIMIZATION += -O2 -fno-strict-aliasing
#endif

# Process board config & directory locations #################################

set(NUTTX_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(NUTTX_APPS_DIR "../apps" CACHE FILEPATH "Relative path to apps/ directory")
get_filename_component(NUTTX_APPS_ABS_DIR ${NUTTX_APPS_DIR} ABSOLUTE BASE_DIR ${NUTTX_DIR})

# Board/config combination
set(NUTTX_CONFIG "nsh" CACHE STRING "Board confguration")
if (NOT DEFINED NUTTX_BOARD)
	message(FATAL_ERROR "Please define a board with NUTTX_BOARD and configuration with NUTTX_CONFIG")
endif()

set(BOARD_PATH "${NUTTX_DIR}/boards/${NUTTX_BOARD}")
if (NOT EXISTS "${BOARD_PATH}/CMakeLists.txt")
	message(FATAL_ERROR "No CMakeList.txt found at ${BOARD_PATH} for ${NUTTX_BOARD}")
endif()

set(NUTTX_DEFCONFIG ${BOARD_PATH}/configs/${NUTTX_CONFIG}/defconfig)

if (NOT EXISTS "${NUTTX_DEFCONFIG}")
  message(FATAL_ERROR "No config file found at ${NUTTX_DEFCONFIG}")
endif()

# Generate inital .config ###################################################
# This is needed right before any other configure step so that we can source
# Kconfig variables into CMake variables

# The following commands need these variables to be passed via environment

set(ENV{APPSDIR} ${NUTTX_APPS_DIR}) # TODO: support not having apps/
set(ENV{EXTERNALDIR} dummy) # TODO
set(ENV{DRIVERS_PLATFORM_DIR} dummy) # TODO

# Copy board defconfig into main directory and expand
# TODO: do also for changes in board/config (by comparing stored defconfig
# to specified one)

if (NOT EXISTS ${CMAKE_BINARY_DIR}/.config OR
    NOT "${NUTTX_DEFCONFIG}" STREQUAL "${NUTTX_DEFCONFIG_SAVED}")

  message(STATUS "Initializing NuttX")
  configure_file(${NUTTX_DEFCONFIG} defconfig COPYONLY)
  configure_file(${NUTTX_DEFCONFIG} .config.compressed COPYONLY)

  set(ENV{KCONFIG_CONFIG} ${CMAKE_BINARY_DIR}/.config.compressed)

  # Do olddefconfig step to expand the abbreviated defconfig into normal config
  execute_process(
    COMMAND kconfig-conf --olddefconfig Kconfig
    OUTPUT_VARIABLE KCONFIG_OUTPUT
    RESULT_VARIABLE KCONFIG_STATUS
    WORKING_DIRECTORY ${NUTTX_DIR}
  )

  file(RENAME ${CMAKE_BINARY_DIR}/.config.compressed ${CMAKE_BINARY_DIR}/.config)
  set(ENV{KCONFIG_CONFIG} ${CMAKE_BINARY_DIR}/.config)

  # store original expanded .config
  configure_file(${CMAKE_BINARY_DIR}/.config ${CMAKE_BINARY_DIR}/.config.orig COPYONLY)

  if (KCONFIG_STATUS AND NOT KCONFIG_STATUS EQUAL 0)
    message(FATAL_ERROR "Failed to initialize Kconfig configuration: ${KCONFIG_OUTPUT}")
  endif()

  set(NUTTX_DEFCONFIG_SAVED ${NUTTX_DEFCONFIG} CACHE INTERNAL "Saved defconfig path" FORCE)

  # Print configuration choices

  message(STATUS "  Board: ${NUTTX_BOARD}")
  message(STATUS "  Config: ${NUTTX_CONFIG}")
  message(STATUS "  Appdir: ${NUTTX_APPS_DIR}")
endif()

# Include .cmake files #######################################################

# this exposes all Kconfig vars to CMake

include(kconfig)

include(custom_commands)
include(nuttx_parse_function_args)
include(nuttx_add_library)
include(nuttx_add_application)
include(nuttx_add_romfs)
include(menuconfig)

include(ExternalProject)
include(FetchContent)

set(FETCHCONTENT_QUIET OFF)

# Setup toolchain ############################################################

# This needs to happen before project() when binaries are searched for

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/arch/${CONFIG_ARCH}/cmake/Toolchain.cmake")

# Define project #############################################################
# This triggers configuration

project(NuttX LANGUAGES ASM C CXX)

# Setup platform options (this needs to happen after project(),
# once the toolchain file has been processed)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/arch/${CONFIG_ARCH}/cmake)
include(Platform)

# Setup main nuttx target ####################################################

add_executable(nuttx)
add_dependencies(nuttx nuttx_context)

if (CONFIG_ARCH_SIM)
  # Create separate up_head OBJECT library built as part of NuttX kernel
  # It must be separated to allow for linking against the rest of NuttX
  # libraries

  add_library(up_head OBJECT)
  nuttx_add_library_internal(up_head)
  get_property(definitions GLOBAL PROPERTY NUTTX_KERNEL_DEFINITIONS)
  target_compile_definitions(up_head PRIVATE ${definitions})

  get_property(options GLOBAL PROPERTY NUTTX_KERNEL_COMPILE_OPTIONS)
  target_compile_options(up_head PRIVATE ${options})

  # We need the relocatable object to be first in the list of libraries
  # to be linked against final nuttx binary

  target_link_libraries(nuttx PRIVATE ${CMAKE_BINARY_DIR}/nuttx.rel)
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
  # system wide warnings
  -Wall
  $<$<COMPILE_LANGUAGE:C>:-Wstrict-prototypes>
  -Wshadow
  -Wundef
  # system wide options
  -fno-builtin
  $<$<COMPILE_LANGUAGE:CXX>:-nostdinc++>
  $<$<COMPILE_LANGUAGE:ASM>:-D__ASSEMBLY__>
)

if (CONFIG_CXX_EXCEPTION)
  add_compile_options(-fno-exceptions -fcheck-new)
endif()

if (CONFIG_STACK_CANARIES)
  add_compile_options(-fstack-protector-all)
endif()

add_definitions(
  -D__NuttX__
)

set_property(GLOBAL APPEND PROPERTY NUTTX_KERNEL_DEFINITIONS -D__KERNEL__)

# Recurse subdirectories #####################################################

# Add tools

ExternalProject_Add(nuttx_host_tools
	SOURCE_DIR ${CMAKE_SOURCE_DIR}/tools
	INSTALL_DIR ${CMAKE_BINARY_DIR}
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_BINARY_DIR}
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	USES_TERMINAL_INSTALL true
)

# Each subdirectory will generate a static library

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

# This picks up the chosen board (as well as common board code)

add_subdirectory(boards)

# Add apps/ to the build (if present)

if(EXISTS ${NUTTX_DIR}/${NUTTX_APPS_DIR}/CMakeLists.txt)
	add_subdirectory(${NUTTX_APPS_DIR} apps)
	
	execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink dummy ${NUTTX_APPS_DIR}/platform/board
                  WORKING_DIRECTORY ${NUTTX_APPS_DIR})
else()
  message(STATUS "Application directory not found at ${NUTTX_APPS_DIR}, skipping")
endif()

# Same for external/

if (EXISTS ${CMAKE_SOURCE_DIR}/external/CMakeLists.txt)
	add_subdirectory(${CMAKE_SOURCE_DIR}/external)
endif()

# Link step ##################################################################

# We will link nuttx libraries in a group so we need to the full path
# to each one

get_property(nuttx_libs GLOBAL PROPERTY NUTTX_LIBRARIES)
set(nuttx_libs_paths)
foreach(lib ${nuttx_libs})
  list(APPEND nuttx_libs_paths $<TARGET_FILE:${lib}>)
endforeach()

# Get linker script to use

get_property(ldscript GLOBAL PROPERTY LD_SCRIPT)

# Perform link

if (NOT CONFIG_ARCH_SIM)
  # Non-sim platform

  # Add empty source file to nuttx target since we will only link libraries
  # to create it and cmake requires at least one file

  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/empty.c" "")
  target_sources(nuttx PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/empty.c")

  # pass the apropriate linker flags

  target_link_libraries(nuttx PRIVATE
    --entry=__start -nostartfiles -nodefaultlibs
    -Wl,--script=${ldscript} -Wl,--start-group ${nuttx_libs} -Wl,--end-group
    gcc
  )
else()
  # On sim platform the link step is a little different. NuttX is first built
  # into a partially linked relocatable object nuttx.rel with no interface
  # to host OS. Then, the names of symbols that conflict with libc symbols
  # are renamed.
  # The final nuttx binary is built by linking the host-specific objects
  # with the relocatable binary.

  # C++ global objects are constructed before main get executed, but it isn't a good
  # point for simulator because NuttX doesn't finish the kernel initialization yet.
  # So we have to skip the standard facilities and do the construction by ourself.
  # But how to achieve the goal?
  # 1.Command linker generate the default script(-verbose)
  # 2.Replace __init_array_start/__init_array_end with _sinit/_einit
  # 3.Append __init_array_start = .; __init_array_end = .;
  # Step 2 let nxtask_startup find objects need to construct
  # Step 3 cheat the host there is no object to construct
  # Note: the destructor can be fixed in the same way.

  add_custom_command(
    OUTPUT nuttx.ld
    COMMAND ${CMAKE_C_COMPILER} ${CMAKE_EXE_LINKER_FLAGS} -Wl,-verbose 2> /dev/null > nuttx.ld || true
    COMMAND sed
    ARGS -i -e '/====/,/====/!d\;//d' -e 's/__executable_start/_stext/g' -e 's/__init_array_start/_sinit/g'
         -e 's/__init_array_end/_einit/g' -e 's/__fini_array_start/_sfini/g' -e 's/__fini_array_end/_efini/g' nuttx.ld
    COMMAND echo
    ARGS '__init_array_start = .\; __init_array_end = .\; __fini_array_start = .\; __fini_array_end = .\;' >> nuttx.ld
  )

  # conflicting symbols to rename

  set(NXSYMBOLS
    __cxa_atexit atexit bind calloc clock_gettime close closedir
    connect dlsym dup exit free fstat fsync ftruncate if_nametoindex
    ioctl longjmp lseek malloc mallinfo memalign memcpy mkdir mmap
    munmap open opendir perror poll pthread_cond_destroy pthread_cond_init
    pthread_cond_signal pthread_cond_wait pthread_create pthread_getspecific
    pthread_key_create pthread_kill pthread_mutex_destroy pthread_mutex_init
    pthread_mutex_lock pthread_mutex_unlock pthread_setspecific pthread_sigmask
    read readdir readv realloc rename rewinddir rmdir sched_yield select
    sendmsg setjmp shutdown sigaction sigaddset sigemptyset sigfillset
    sleep socket stat statvfs strerror syslog tcgetattr tcsetattr unlink
    usleep write writev
  )

  # write the remapping "old new" file for each entry

  set(NXSYMBOL_RENAMES)
  foreach(NXSYMBOL ${NXSYMBOLS})
    if (CONFIG_HOST_MACOS OR (CONFIG_HOST_WINDOWS AND CONFIG_SIM_CYGWIN_DECORATED))
      list(APPEND NXSYMBOL_RENAMES "_${NXSYMBOL} NX${NXSYMBOL}")
    else()
      list(APPEND NXSYMBOL_RENAMES "${NXSYMBOL} NX${NXSYMBOL}")
    endif()
  endforeach()
  string(REPLACE ";" "\n" NXSYMBOL_RENAMES "${NXSYMBOL_RENAMES}")
  file(WRITE ${CMAKE_BINARY_DIR}/nuttx-names.dat "${NXSYMBOL_RENAMES}\n")

  # create the relocatable object manually, from all internal libs

  add_custom_command(
    OUTPUT nuttx.rel
    COMMAND ${CMAKE_C_COMPILER} ARGS
      -r
      #-Wl,--script=${ldscript}
      $<TARGET_OBJECTS:up_head>
      -Wl,--start-group ${nuttx_libs_paths} -Wl,--end-group
      -o nuttx.rel
    COMMAND ${CMAKE_OBJCOPY} --redefine-syms=nuttx-names.dat nuttx.rel
    DEPENDS ${nuttx_libs} nuttx.ld up_head
  )
  add_custom_target(nuttx-rel DEPENDS nuttx.rel)

  # link the final nuttx binary
  add_dependencies(nuttx nuttx-rel)
  target_link_options(nuttx PUBLIC -T nuttx.ld)
endif()
