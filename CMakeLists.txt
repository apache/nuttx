# Request a version available on latest Ubuntu LTS (20.04)

cmake_minimum_required(VERSION 3.16)

# Handle newer CMake versions correctly by setting policies

if (POLICY CMP0115)
  # do not auto-guess extension in target_sources()
  cmake_policy(SET CMP0115 NEW)
endif()

# Basic CMake configuration ##################################################

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)
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

set(ENV{HOST_LINUX} n)
set(ENV{HOST_MACOS} n)
set(ENV{HOST_WINDOWS} n)
set(ENV{HOST_OTHER} n)

if(APPLE)
  set(ENV{HOST_MACOS} y)
elseif(WIN32)
  set(ENV{HOST_WINDOWS} y)
elseif(UNIX)
  set(ENV{HOST_LINUX} y)
  set(LINUX TRUE)
else()
  set(ENV{HOST_OTHER} y)
  set(OTHER_OS TRUE)
endif()

include(python)

message(STATUS "PYTHON_EXECUTABLE: ${PYTHON_EXECUTABLE}")

# this exposes all Kconfig vars to CMake
include(kconfig)

include(custom_commands)
include(nuttx_parse_function_args)
include(nuttx_add_library)
include(nuttx_add_application)
include(nuttx_add_romfs)
include(nuttx_add_symtab)
include(nuttx_add_module)
include(nuttx_generate_outputs)
#include(menuconfig)

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

# detect host platform
include(detect_platform)

# Setup main nuttx target ####################################################

add_executable(nuttx)
add_dependencies(nuttx nuttx_context)

if (CONFIG_ARCH_SIM)
  # Create separate up_head OBJECT library built as part of NuttX kernel
  # It must be separated to allow for linking against the rest of NuttX
  # libraries

  add_library(up_head OBJECT)
  nuttx_add_library_internal(up_head)
  get_property(definitions TARGET nuttx PROPERTY NUTTX_KERNEL_DEFINITIONS)
  target_compile_definitions(up_head PRIVATE ${definitions})

  get_property(options TARGET nuttx PROPERTY NUTTX_KERNEL_COMPILE_OPTIONS)
  target_compile_options(up_head PRIVATE ${options})

  # We need the relocatable object to be first in the list of libraries
  # to be linked against final nuttx binary

  target_link_libraries(nuttx PRIVATE ${CMAKE_BINARY_DIR}/nuttx.rel)
else()
  # These flags apply to source files not part of the library. In sim build this corresponds
  # to "host" files, so we only do this on non-sim build
  target_compile_definitions(nuttx PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_COMPILE_DEFINITIONS>>)
  target_compile_options(nuttx PRIVATE $<GENEX_EVAL:$<TARGET_PROPERTY:nuttx,NUTTX_COMPILE_OPTIONS>>)
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
#  $<$<COMPILE_LANGUAGE:C>:-Wstrict-prototypes>
#  -Wshadow
#  -Wundef
  # system wide options
  -fno-builtin
  $<$<COMPILE_LANGUAGE:CXX>:-nostdinc++>
  $<$<COMPILE_LANGUAGE:ASM>:-D__ASSEMBLY__>
)

if (NOT CONFIG_CXX_EXCEPTION)
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fcheck-new>)
endif()

if (CONFIG_STACK_CANARIES)
  add_compile_options(-fstack-protector-all)
endif()

add_definitions(
  -D__NuttX__
)

set_property(TARGET nuttx APPEND PROPERTY NUTTX_KERNEL_DEFINITIONS __KERNEL__)

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
add_subdirectory(openamp)
add_subdirectory(sched)
add_subdirectory(syscall)
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

# Get linker script to use
get_property(ldscript GLOBAL PROPERTY LD_SCRIPT)

# Perform link

# Add empty source file to nuttx target since
# cmake requires at least one file and we will only be linking libraries

if (CONFIG_HAVE_CXX)
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/empty.cxx" "")
  target_sources(nuttx PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/empty.cxx")
else()
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/empty.c" "")
  target_sources(nuttx PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/empty.c")
#target_sources(nuttx PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/../../apps/system/nsh/nsh_main.c")
endif()

# initialize manifest to hold all generated files
file(WRITE ${CMAKE_BINARY_DIR}/nuttx.manifest "")

if (NOT CONFIG_ARCH_SIM)
  # Non-sim platform
  # pass the apropriate linker flags

  # TODO: move elsewhere? toolchain?
  target_link_directories(nuttx PRIVATE ${LIBGCC_FILEPATH})

  if (NOT CONFIG_BUILD_PROTECTED)
    get_property(nuttx_system_libs GLOBAL PROPERTY NUTTX_SYSTEM_LIBRARIES)
  endif()

  get_property(nuttx_kernel_libs GLOBAL PROPERTY NUTTX_KERNEL_LIBRARIES)

  # TODO: nostart/nodefault not applicable to nuttx toolchain
  target_link_libraries(nuttx PRIVATE
    --entry=__start -nostartfiles -nodefaultlibs
    -Wl,--script=${ldscript}
    -Wl,--start-group ${nuttx_kernel_libs} ${nuttx_system_libs} gcc $<$<BOOL:${CONFIG_HAVE_CXX}>:supc++> -Wl,--end-group
  )

  # generate binary outputs in different formats (.bin, .hex, etc)
  nuttx_generate_outputs(nuttx)

  if (CONFIG_UBOOT_UIMAGE)
    add_custom_command(OUTPUT uImage
      COMMAND ${MKIMAGE} -A ${CONFIG_ARCH} -O linux -C none -T kernel -a ${CONFIG_UIMAGE_LOAD_ADDRESS} -e ${CONFIG_UIMAGE_ENTRY_POINT} -n nuttx -d nuttx.bin uImage
       DEPENDS nuttx)
    add_custom_target(nuttx-uImage ALL DEPENDS uImage)

    # TODO: install?
    #    $(Q) if [ -w /tftpboot ] ; then \
    #      cp -f uImage /tftpboot/uImage; \
    #    fi
    file(APPEND ${CMAKE_BINARY_DIR}/nuttx.manifest uImage)
  endif()
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

  get_property(nuttx_compile_options TARGET nuttx PROPERTY NUTTX_COMPILE_OPTIONS)

  if (NOT APPLE)
    add_custom_command(
      OUTPUT nuttx.ld
      COMMAND ${CMAKE_C_COMPILER} ${CMAKE_EXE_LINKER_FLAGS} $<$<BOOL:${CONFIG_SIM_M32}>:-m32> -Wl,-verbose 2> /dev/null > nuttx-orig.ld || true
      COMMAND cat nuttx-orig.ld |
                sed -e '/====/,/====/!d\;//d'
                    -e 's/__executable_start/_stext/g'
                    -e 's/__init_array_start/_sinit/g'
                    -e 's/__init_array_end/_einit/g'
                    -e 's/__fini_array_start/_sfini/g'
                    -e 's/__fini_array_end/_efini/g' > nuttx.ld
      COMMAND echo
      ARGS '__init_array_start = .\; __init_array_end = .\; __fini_array_start = .\; __fini_array_end = .\;' >> nuttx.ld
    )
  endif()

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
    if (APPLE OR (CYGWIN AND CONFIG_SIM_CYGWIN_DECORATED))
      list(APPEND NXSYMBOL_RENAMES "_${NXSYMBOL} NX${NXSYMBOL}")
    else()
      list(APPEND NXSYMBOL_RENAMES "${NXSYMBOL} NX${NXSYMBOL}")
    endif()
  endforeach()
  string(REPLACE ";" "\n" NXSYMBOL_RENAMES "${NXSYMBOL_RENAMES}")
  file(WRITE ${CMAKE_BINARY_DIR}/nuttx-names.dat "${NXSYMBOL_RENAMES}\n")

  # create the relocatable object manually, from all internal libs
  # TODO: link using target_link_libraries?

  get_property(nuttx_kernel_libs GLOBAL PROPERTY NUTTX_KERNEL_LIBRARIES)
  get_property(nuttx_system_libs GLOBAL PROPERTY NUTTX_SYSTEM_LIBRARIES)

  # TODO: do with single function call?
  set(nuttx_libs_paths)
  foreach(lib ${nuttx_kernel_libs} ${nuttx_system_libs})
    list(APPEND nuttx_libs_paths $<TARGET_FILE:${lib}>)
  endforeach()

  add_custom_command(
    OUTPUT nuttx.rel
    COMMAND ${CMAKE_C_COMPILER} ARGS
      -r
      $<$<BOOL:${CONFIG_SIM_M32}>:-m32>
      $<TARGET_OBJECTS:up_head>
      $<$<NOT:$<BOOL:${APPLE}>>:-Wl,--start-group> ${nuttx_libs_paths} $<$<NOT:$<BOOL:${APPLE}>>:-Wl,--end-group>
      -o nuttx.rel
    COMMAND ${CMAKE_OBJCOPY} --redefine-syms=nuttx-names.dat nuttx.rel
    DEPENDS ${nuttx_libs} up_head
  )
  add_custom_target(nuttx-rel DEPENDS nuttx.rel $<$<NOT:$<BOOL:${APPLE}>>:nuttx.ld>)

  # link the final nuttx binary
  add_dependencies(nuttx nuttx-rel)
  target_link_options(nuttx PUBLIC $<$<NOT:$<BOOL:${APPLE}>>:-T nuttx.ld> $<$<BOOL:${CONFIG_SIM_M32}>:-m32>)
endif()

# TODO: if we use an install target a manifest may not be needed
if (CONFIG_ARCH_SIM)
  file(APPEND ${CMAKE_BINARY_DIR}/nuttx.manifest "nuttx\n")
endif()

# Userspace portion ##########################################################

if (NOT CONFIG_BUILD_FLAT)  
  add_executable(nuttx_user)

  target_link_directories(nuttx_user PRIVATE ${LIBGCC_FILEPATH})
  get_property(nuttx_system_libs GLOBAL PROPERTY NUTTX_SYSTEM_LIBRARIES)

  get_property(user_ldscript GLOBAL PROPERTY LD_SCRIPT_USER)
  list(TRANSFORM user_ldscript PREPEND "-Wl,--script=")

  target_link_options(nuttx_user PRIVATE
    -nostartfiles -nodefaultlibs
    -Wl,--entry=${CONFIG_USER_ENTRYPOINT}
    -Wl,--undefined=${CONFIG_USER_ENTRYPOINT})

  target_link_libraries(nuttx_user PRIVATE
    ${user_ldscript} userspace
    $<$<NOT:$<BOOL:${APPLE}>>:-Wl,--start-group> ${nuttx_system_libs} gcc $<$<BOOL:${CONFIG_HAVE_CXX}>:supc++> $<$<NOT:$<BOOL:${APPLE}>>:-Wl,--end-group>
  )

  add_custom_command(OUTPUT User.map
    COMMAND ${CMAKE_NM} nuttx_user > User.map
    DEPENDS nuttx_user)
  add_custom_target(usermap ALL DEPENDS User.map)

  # generate binary outputs in different formats (.bin, .hex, etc)
  nuttx_generate_outputs(nuttx_user)

  # create merged .hex file ready to be flashed
  # TODO: does not seem to be generating a functional hex file
  if (CONFIG_INTELHEX_BINARY AND SREC_CAT)
    add_custom_command(OUTPUT nuttx_combined.hex
      COMMAND ${SREC_CAT} nuttx.hex -intel nuttx_user.hex -intel -o nuttx_combined.hex -intel
      DEPENDS nuttx_user nuttx)
    add_custom_target(nuttx-combined ALL DEPENDS nuttx_combined.hex)
  endif()

  # TODO: could also merge elf binaries
endif()
