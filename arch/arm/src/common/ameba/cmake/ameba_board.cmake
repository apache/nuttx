# ##############################################################################
# arch/arm/src/common/ameba/cmake/ameba_board.cmake
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

# CMake counterpart of the make-side arch/.../<ic>/ameba_board.mk: the shared
# vendor-SDK build machinery for every Ameba IC.  The per-IC arch CMakeLists.txt
# sets the differing inputs (SoC subdir, source/include lists, AP project name)
# then include()s this file, exactly as each board's scripts/Make.defs
# include()s ameba_board.mk.  Every mechanism below mirrors a section of
# ameba_board.mk; see that file for the detailed rationale and for the inputs
# each arch chip CMakeLists.txt is expected to set.

set(AMEBA_COMMON_DIR ${CMAKE_CURRENT_LIST_DIR}/..)
set(AMEBA_TOOLS_DIR ${AMEBA_COMMON_DIR}/tools)

# AMEBA_SDK / AMEBA_TOOLCHAIN_DIR are resolved (and the compiler sanity-checked)
# by ameba_sdk.cmake, which the arch chip CMakeLists includes before its
# SDK-relative source lists; include it here too (no-op if already done) so this
# file is self-contained.
include(${CMAKE_CURRENT_LIST_DIR}/ameba_sdk.cmake)

# The IC arch chip dir (the includer's dir) holds IC-specific sources such as
# ameba_app_start.c, referenced by AMEBA_FWLIB_SRCS via ${AMEBA_CHIP_DIR}.
set(AMEBA_CHIP_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# SDK sub-trees + the board-local (gitignored) staging area, mirroring the make
# variables of the same name.  Generated artefacts (autoconf, the two .a, the
# combined ld script) go under the board prebuilt/ dir, shared with the make
# build's staging.
set(AMEBA_SOC ${AMEBA_SDK}/component/soc/${AMEBA_SOC_NAME})
set(AMEBA_PROJ ${AMEBA_SOC}/project)
set(AMEBA_KM4_PROJ ${AMEBA_PROJ}/${AMEBA_KM_PROJ})
set(AMEBA_SOC_LIB ${AMEBA_KM4_PROJ}/lib/soc)

set(AMEBA_PREBUILT ${NUTTX_BOARD_DIR}/prebuilt)
set(AMEBA_PREBUILT_LIBS ${AMEBA_PREBUILT}/libs)
set(AMEBA_AUTOCONF ${AMEBA_PREBUILT}/platform_autoconf.h)

file(MAKE_DIRECTORY ${AMEBA_PREBUILT_LIBS})

# ----------------------------------------------------------------------------
# #3 platform_autoconf.h -- regenerated from the SDK menuconfig (single source
# of truth for the flash layout + feature switches).  Done at CONFIGURE time so
# the VFS1 geometry can be read back and injected into ameba_flash_mtd.c, and so
# it exists before the ld-script preprocess / WiFi force-include below.
# ----------------------------------------------------------------------------

# Provision the SDK dev environment (prebuilts + python venv) the autoconf /
# packaging steps need -- idempotent, mirrors the make PREBUILD's setup call.
execute_process(COMMAND sh ${AMEBA_TOOLS_DIR}/ameba_setup_env.sh ${AMEBA_SDK}
                        ${AMEBA_TOOLCHAIN_DIR} RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "ameba_setup_env.sh failed (rc=${_rc})")
endif()

message(STATUS "ameba: generating platform_autoconf.h from SDK menuconfig")
execute_process(
  COMMAND sh ${AMEBA_TOOLS_DIR}/ameba_gen_autoconf.sh ${AMEBA_SDK}
          ${AMEBA_PY_SOC} ${AMEBA_AP_PROJECT} ${AMEBA_AUTOCONF}
  RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "ameba_gen_autoconf.sh failed (rc=${_rc})")
endif()

# VFS1 data-partition geometry -> neutral -D macros on ameba_flash_mtd.c only (a
# NuttX C file must not force-include the SDK autoconf).  Mirrors the make
# CFLAGS derivation.  ameba_flash_mtd.c falls back to the vendor default if
# absent.
if(AMEBA_CFG_FLASHFS AND EXISTS ${AMEBA_AUTOCONF})
  file(STRINGS ${AMEBA_AUTOCONF} _l REGEX "CONFIG_FLASH_VFS1_OFFSET")
  if(_l)
    list(GET _l 0 _l0)
    string(REGEX REPLACE ".*CONFIG_FLASH_VFS1_OFFSET[ \t]+([0-9xXa-fA-F]+).*"
                         "\\1" _vfs1_off "${_l0}")
  endif()
  file(STRINGS ${AMEBA_AUTOCONF} _l REGEX "CONFIG_FLASH_VFS1_SIZE")
  if(_l)
    list(GET _l 0 _l0)
    string(REGEX REPLACE ".*CONFIG_FLASH_VFS1_SIZE[ \t]+([0-9xXa-fA-F]+).*"
                         "\\1" _vfs1_size "${_l0}")
  endif()
  set(_flash_defs)
  if(_vfs1_off)
    list(APPEND _flash_defs AMEBA_FLASH_VFS1_OFFSET_XIP=${_vfs1_off})
  endif()
  if(_vfs1_size)
    list(APPEND _flash_defs AMEBA_FLASH_VFS1_SIZE_CFG=${_vfs1_size})
  endif()
  if(_flash_defs)
    set_property(
      SOURCE ${AMEBA_COMMON_DIR}/ameba_flash_mtd.c
      APPEND
      PROPERTY COMPILE_DEFINITIONS ${_flash_defs})
  endif()
endif()

# ----------------------------------------------------------------------------
# #4 libameba_fwlib.a / libameba_wifi.a
#
# Compiled from SDK source with an ISOLATED flag+include set (the SDK headers
# clash with NuttX's), so -- exactly like the make PREBUILD -- each source is
# compiled by a bare custom-command invocation using NUTTX_EXTRA_FLAGS (the
# toolchain cpu/opt flags, WITHOUT NuttX's include dirs) plus the SDK include
# set, then archived.  add_library() is deliberately NOT used: it would inherit
# the directory's NuttX include_directories() and pull in clashing headers.
# ----------------------------------------------------------------------------

# ~~~
# ameba_build_lib(<lib_path> <obj_subdir> <srcs-var> <inc-var>)
#   Compile each source in <srcs-var> to an object under <obj_subdir> with the
#   isolated flags, then archive them all into <lib_path>.  Returns via the
#   caller-supplied variable the target name to depend on.
# ~~~
function(ameba_build_lib lib obj_subdir srcs_var inc_var out_target)
  set(_objdir ${AMEBA_PREBUILT_LIBS}/${obj_subdir})
  file(MAKE_DIRECTORY ${_objdir})
  set(_objs)
  foreach(_src ${${srcs_var}})
    get_filename_component(_base ${_src} NAME_WE)
    set(_obj ${_objdir}/${_base}.o)
    add_custom_command(
      OUTPUT ${_obj}
      COMMAND ${CMAKE_C_COMPILER} ${NUTTX_EXTRA_FLAGS} -Os -ffunction-sections
              -fdata-sections ${${inc_var}} -c ${_src} -o ${_obj}
      DEPENDS ${_src} ${AMEBA_AUTOCONF}
      COMMAND_EXPAND_LISTS
      COMMENT "CC (ameba ${obj_subdir}) ${_base}.o")
    list(APPEND _objs ${_obj})
  endforeach()
  add_custom_command(
    OUTPUT ${lib}
    COMMAND ${CMAKE_AR} crs ${lib} ${_objs}
    DEPENDS ${_objs}
    COMMENT "AR ${lib}")
  get_filename_component(_libname ${lib} NAME_WE)
  add_custom_target(${_libname}_target DEPENDS ${lib})
  set(${out_target}
      ${_libname}_target
      PARENT_SCOPE)
endfunction()

set(AMEBA_FWLIB_A ${AMEBA_PREBUILT_LIBS}/libameba_fwlib.a)
ameba_build_lib(${AMEBA_FWLIB_A} fwlib_obj AMEBA_FWLIB_SRCS AMEBA_FWLIB_INC
                _fwlib_target)
add_dependencies(nuttx ${_fwlib_target})

set(AMEBA_EXTRA_LIBS ${AMEBA_FWLIB_A})

# chipinfo / pmc ship pre-compiled inside the SDK source tree.
list(APPEND AMEBA_EXTRA_LIBS ${AMEBA_SOC_LIB}/lib_chipinfo.a
     ${AMEBA_SOC_LIB}/lib_pmc.a)

if(AMEBA_CFG_WIFI)
  set(AMEBA_WIFI_A ${AMEBA_PREBUILT_LIBS}/libameba_wifi.a)
  ameba_build_lib(${AMEBA_WIFI_A} wifi_obj AMEBA_WIFI_SRCS AMEBA_WIFI_INC
                  _wifi_target)
  add_dependencies(nuttx ${_wifi_target})

  # KM4-side host WiFi control libs from the pinned SDK.  The exact set is
  # IC-specific (e.g. lib_coex is an AP-side lib on RTL8721Dx but NP-side on
  # RTL8720F), so the arch chip CMakeLists provides it via AMEBA_WIFI_APP_LIBS.
  list(APPEND AMEBA_EXTRA_LIBS ${AMEBA_WIFI_APP_LIBS} ${AMEBA_WIFI_A})
endif()

# crtbegin.o / crtend.o (asdk gcc runtime) -- positional objects appended inside
# the link group; -lm / -lstdc++ name-spec libs (C++ static-init + math).  The
# cpu flags MUST be passed so gcc returns the multilib-correct crt objects (the
# hard-float / cortex-m33 variant), else the link fails on a VFP/arch mismatch.
execute_process(
  COMMAND ${CMAKE_C_COMPILER} ${NUTTX_EXTRA_FLAGS} -print-file-name=crtbegin.o
  OUTPUT_VARIABLE _crtbegin
  OUTPUT_STRIP_TRAILING_WHITESPACE)
execute_process(
  COMMAND ${CMAKE_C_COMPILER} ${NUTTX_EXTRA_FLAGS} -print-file-name=crtend.o
  OUTPUT_VARIABLE _crtend
  OUTPUT_STRIP_TRAILING_WHITESPACE)
list(APPEND AMEBA_EXTRA_LIBS ${_crtbegin} ${_crtend} -lm -lstdc++)

# ----------------------------------------------------------------------------
# #5 ld.script.gen -- combined image2 linker script (see ameba_gen_ldscript.sh).
#
# Generated at CONFIGURE time (like the autoconf above), NOT via a build-time
# custom command: the top-level CMakeLists consumes LD_SCRIPT to build its cpp
# ".tmp" preprocessing target in a different directory scope, and the Makefiles
# generator cannot resolve a cross-directory add_custom_command OUTPUT as that
# target's dependency ("No rule to make target ld.script.gen").  Producing it as
# a real file up front sidesteps that -- its inputs (the SDK ld sources + the
# config-derived autoconf) are all already present at configure time.
# ----------------------------------------------------------------------------

set(AMEBA_KM4_LD ${AMEBA_KM4_PROJ}/ld)
set(AMEBA_IMG2_LD ${AMEBA_KM4_LD}/ameba_img2_all.ld)
set(AMEBA_ROM_LD ${AMEBA_KM4_LD}/ameba_rom_symbol_acut_s.ld)
set(GENLDSCRIPT ${AMEBA_PREBUILT}/ld.script.gen)

message(
  STATUS "ameba: generating ld.script.gen (AP ${AMEBA_AP_PROJECT} image2)")
execute_process(
  COMMAND
    sh ${AMEBA_TOOLS_DIR}/ameba_gen_ldscript.sh ${CMAKE_C_COMPILER}
    ${AMEBA_IMG2_LD} ${AMEBA_ROM_LD} ${AMEBA_AUTOCONF} ${AMEBA_PREBUILT}
    ${AMEBA_AP_PROJECT} ${GENLDSCRIPT}
  RESULT_VARIABLE _rc)
if(NOT _rc EQUAL 0)
  message(FATAL_ERROR "ameba_gen_ldscript.sh failed (rc=${_rc})")
endif()

set_property(GLOBAL PROPERTY LD_SCRIPT ${GENLDSCRIPT})

# ----------------------------------------------------------------------------
# #6 Link flags + EXTRA_LIBS (reproducing link_img2.sh).  The libs are appended
# to the `nuttx` target so they land inside the top-level --start-group/
# --end-group, resolving against each other and the NuttX libs; --gc-sections
# drops everything unreferenced.
# ----------------------------------------------------------------------------

target_link_options(
  nuttx
  PRIVATE
  -Wl,-u,app_start
  -Wl,-e,app_start
  -Wl,--gc-sections
  -Wl,--no-enum-size-warning
  -Wl,--warn-common
  -Wl,--build-id=none
  -Wl,--cref
  -Wl,--defsym=_sbss=__bss_start__
  -Wl,--defsym=_ebss=__bss_end__
  -Wl,--defsym=_sdata=__sram_image2_start__
  -Wl,--defsym=_edata=__sram_image2_start__
  -Wl,--defsym=_eronly=__sram_image2_start__
  -Wl,-Map=${CMAKE_BINARY_DIR}/nuttx.map
  ${AMEBA_EXTRA_LINK_OPTIONS})

# Append to NUTTX_EXTRA_LIBRARIES (not target_link_libraries): the top-level
# link places this property INSIDE the --start-group/--end-group, so the SDK
# archives resolve against each other and the NuttX libs.  A plain
# target_link_libraries here would land them before the group and leave the
# cross-references (FLASH_*, IPC_*, ameba_wifi_*) undefined.
set_property(GLOBAL APPEND PROPERTY NUTTX_EXTRA_LIBRARIES ${AMEBA_EXTRA_LIBS})

# ----------------------------------------------------------------------------
# #7 POSTBUILD -- package the linked image2 into a flashable nuttx.bin. Wired
# here (shared) so both boards on this IC get it; the whole recipe lives in
# ameba_package.sh, invoked after `nuttx` is linked.
# ----------------------------------------------------------------------------

if(EXISTS ${AMEBA_TOOLS_DIR}/ameba_package.sh)
  add_custom_target(
    nuttx_post_build
    DEPENDS nuttx
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "PACK nuttx.bin (Ameba AP + NP image2)")
  add_custom_command(
    TARGET nuttx_post_build
    POST_BUILD
    COMMAND
      ${CMAKE_COMMAND} -E env
      "AMEBA_FLASH_HINT=AMEBA_PORT=/dev/ttyUSB0 cmake --build ${CMAKE_BINARY_DIR} --target flash"
      sh ${AMEBA_TOOLS_DIR}/ameba_package.sh ${AMEBA_SDK} ${AMEBA_PY_SOC}
      ${AMEBA_SOC_NAME} ${AMEBA_PREBUILT} ${AMEBA_AP_PROJECT} ${AMEBA_KM_PROJ}
      ${AMEBA_NP_TARGET} ${AMEBA_TOOLCHAIN_DIR} ${CMAKE_BINARY_DIR}/nuttx
      ${CMAKE_BINARY_DIR}/nuttx.bin
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    # USES_TERMINAL -> Ninja "console" pool: stream the (long) SDK NP-build +
    # packaging output live instead of buffering it until the step finishes.
    USES_TERMINAL COMMAND_EXPAND_LISTS)
endif()

# ----------------------------------------------------------------------------
# `flash` target -- download boot.bin + nuttx.bin over serial (shared script).
# The serial port + baud come from the environment at build time, so no
# reconfigure is needed: AMEBA_PORT=/dev/ttyUSB0 [AMEBA_BAUD=1500000] cmake
# --build <dir> --target flash AMEBA_FLASH_PROFILE names the SDK .rdev
# flash-loader profile for this IC.
# ----------------------------------------------------------------------------

# The SDK flash-loader profile base name matches the public SoC id on both ICs
# (RTL8721Dx.rdev / RTL8720F.rdev); default to it unless the arch overrides.
if(NOT DEFINED AMEBA_FLASH_PROFILE)
  set(AMEBA_FLASH_PROFILE ${AMEBA_PY_SOC})
endif()

if(AMEBA_FLASH_PROFILE AND NOT TARGET flash)
  add_custom_target(
    flash
    DEPENDS nuttx_post_build
    COMMAND
      ${CMAKE_COMMAND} -E env sh ${AMEBA_TOOLS_DIR}/ameba_flash.sh ${AMEBA_SDK}
      ${AMEBA_FLASH_PROFILE} ${AMEBA_AUTOCONF} ${AMEBA_PREBUILT}
      ${CMAKE_BINARY_DIR}/nuttx.bin
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    USES_TERMINAL VERBATIM
    COMMENT "Flashing Ameba ${AMEBA_PY_SOC} over ${AMEBA_PORT} (AMEBA_PORT env)"
  )
endif()
