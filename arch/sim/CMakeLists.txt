include_directories(include)
add_subdirectory(src)

# configure host binary ######################################################

target_include_directories(nuttx PRIVATE ${CONFIG_ARCH_CHIP})

target_compile_definitions(nuttx PRIVATE CONFIG_USEC_PER_TICK=${CONFIG_USEC_PER_TICK})

if (APPLE)
  target_compile_options(nuttx PRIVATE -Wno-deprecated-declarations)
endif()

# configure simulated nuttx ##################################################

# Add -fvisibility=hidden
# Because we don't want export nuttx's symbols to shared libraries

set_property(TARGET nuttx APPEND PROPERTY NUTTX_COMPILE_OPTIONS -fvisibility=hidden)

# Add -fno-common because macOS "ld -r" doesn't seem to pick objects
# for common symbols.

set_property(TARGET nuttx APPEND PROPERTY NUTTX_COMPILE_OPTIONS -fno-common)

if (CONFIG_SIM_SANITIZE)
  set_property(TARGET nuttx APPEND PROPERTY NUTTX_COMPILE_OPTIONS
                  -fsanitize=address
                  -fsanitize=undefined
                  -fno-omit-frame-pointer
  )
endif()

set_property(TARGET nuttx APPEND PROPERTY NUTTX_COMPILE_OPTIONS
  # NuttX is sometimes built as a native target.
  # In that case, the __NuttX__ macro is predefined by the compiler.
  # https://github.com/NuttX/buildroot
  #
  # In other cases, __NuttX__ is an ordinary user-definded macro.
  # It's especially the case for NuttX sim, which is a target to run
  # the entire NuttX as a program on the host OS, which can be Linux,
  # macOS, Windows, etc.
  # https://cwiki.apache.org/confluence/display/NUTTX/NuttX+Simulation
  # In that case, the host OS compiler is used to build NuttX.
  # Thus, eg. NuttX sim on macOS is built with __APPLE__.
  # We #undef predefined macros for those possible host OSes here
  # because the OS APIs this library should use are of NuttX,
  # not the host OS.

  -U_AIX -U_WIN32 -U__APPLE__ -U__FreeBSD__
  -U__NetBSD__ -U__linux__ -U__sun__ -U__unix__

  # Note: -fshort-wchar for the case where NuttX and the host OS have
  # differnt wchar_t. On Nuttx, it's uint16_t. On macOS, it's 32-bit.

  -fshort-wchar
)

# common options #############################################################

if (X86_64 AND CONFIG_SIM_M32)
  set_property(TARGET nuttx APPEND PROPERTY NUTTX_COMPILE_OPTIONS -m32)
  target_compile_options(nuttx PRIVATE -m32)
  target_link_options(nuttx PRIVATE -m32)
endif()

if(CONFIG_LIBCXX)
  # Why c++14?
  #  * libcxx seems to require c++11.
  #  * The compiler defaults varies:
  #      clang/macOS (from xcode): 199711L
  #      gcc/ubuntu:               201402L
  #  * There is a precedent to use c++14.
  #    (boards/arm/stm32l4/nucleo-l476rg/scripts/Make.defs)

  # TODO: place on top-level CMakeLists.txt?
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  if (APPLE)
    # macOS uses libc++abi

    set_property(TARGET nuttx APPEND PROPERTY NUTTX_DEFINITIONS LIBCXX_BUILDING_LIBCXXABI)
  else()
    # Linux C++ ABI seems vary.
    # Probably __GLIBCXX__ is the best bet.
    # XXX what to do for windows?
    set_property(TARGET nuttx APPEND PROPERTY NUTTX_DEFINITIONS __GLIBCXX__)
  endif()
endif()

## This is part of the top-level export target
# TODO: how to deal with in CMake?

##export_startup: board/libboard$(LIBEXT) up_head.o $(HOSTOBJS) nuttx-names.dat
##	cp up_head.o $(HOSTOBJS) ${EXPORT_DIR}/startup
##	cp nuttx-names.dat ${EXPORT_DIR}/libs
##	echo main NXmain >> ${EXPORT_DIR}/libs/nuttx-names.dat

# Loadable module definitions
# TODO: implement modules with CMake

## -fno-pic to avoid GOT relocations
set_property(TARGET nuttx APPEND PROPERTY NUTTX_ELF_MODULE_COMPILE_OPTIONS -fno-pic)
set_property(TARGET nuttx APPEND PROPERTY NUTTX_ELF_MODULE_LINK_OPTIONS -r -e module_initialize -T ${NUTTX_DIR}/libs/libc/modlib/gnu-elf.ld)

if (CONFIG_LIBC_ARCH_ELF_64BIT)
  # For amd64:
  # It seems macOS/x86_64 loads the program text around 00000001_xxxxxxxx.
  # The gcc default (-mcmodel=small) would produce out-of-range 32-bit
  # relocations.
  # Even on Linux, NuttX modules are loaded into the NuttX heap, which
  # can be out of range with -mcmodel=small.
  set_property(TARGET nuttx APPEND PROPERTY NUTTX_ELF_MODULE_COMPILE_OPTIONS -mcmodel=large)
endif()

## On Linux, we (ab)use the host compiler to compile binaries for NuttX.
## Explicitly disable features which might be default on the host while
## not available on NuttX.
set_property(TARGET nuttx APPEND PROPERTY NUTTX_ELF_MODULE_COMPILE_OPTIONS -fno-stack-protector)

# TODO: move to toolchain file
# NuttX modules are ELF binaries.
# Non-ELF platforms like macOS need to use a separate ELF toolchain.
#ifeq ($(CONFIG_HOST_MACOS),y)
#  # eg. brew install x86_64-elf-gcc
#  MODULECC = x86_64-elf-gcc
#  MODULELD = x86_64-elf-ld
#  MODULESTRIP = x86_64-elf-strip --strip-unneeded
#endif

## ELF module definitions

## -fno-pic to avoid GOT relocations
set_property(TARGET nuttx APPEND PROPERTY NUTTX_ELF_APP_COMPILE_OPTIONS -fno-pic)
set_property(TARGET nuttx APPEND PROPERTY NUTTX_ELF_APP_LINK_OPTIONS -r -e main -T${BOARD_PATH}/scripts/gnu-elf.ld)

# TODO: move to toolchain file
if (X86_64 AND CONFIG_SIM_M32)
  set_property(TARGET nuttx APPEND PROPERTY NUTTX_ELF_MODULE_LINK_OPTIONS -m32)
  set_property(TARGET nuttx APPEND PROPERTY NUTTX_ELF_APP_LINK_OPTIONS -m32)
endif()
