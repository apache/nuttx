# ##############################################################################
# arch/arm/src/rtl8730e/CMakeLists.txt
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

# IC-specific register drivers + IPC wiring live here; IC-agnostic glue is
# shared from arch/arm/src/common/ameba.
set(AMEBA_COMMON ${CMAKE_CURRENT_LIST_DIR}/../common/ameba)

set(SRCS rtl8730e_boot.c rtl8730e_serial.c rtl8730e_irq.c rtl8730e_timer.c)

if(CONFIG_ARCH_ARMV7A)
  list(APPEND SRCS rtl8730e_pgalloc.c rtl8730e_memorymap.c)
endif()

if(CONFIG_SMP)
  list(APPEND SRCS rtl8730e_cpuboot.c)
endif()

if(CONFIG_ARCH_IDLE_CUSTOM)
  list(APPEND SRCS rtl8730e_idle.c)
endif()

if(CONFIG_BUILD_KERNEL)
  list(APPEND SRCS rtl8730e_pgalloc.c)
endif()

if(CONFIG_BUILD_PROTECTED)
  list(APPEND SRCS rtl8730e_userspace.c rtl8730e_allocateheap.c)
endif()

if(CONFIG_RTL8730E_WIFI)
  list(APPEND SRCS rtl8730e_wifi_init.c rtl8730e_wifi_stubs.c
       ${AMEBA_COMMON}/ameba_os_wrap.c ${AMEBA_COMMON}/ameba_wlan.c)
endif()

if(CONFIG_RTL8730E_FLASH_FS)
  list(APPEND SRCS ${AMEBA_COMMON}/ameba_flash_mtd.c)
  # rtl8730e_flash_stubs.c supplies the RAM-side shims lib_rom.a objects expect
  # (DiagPrintfNano, irq_enable, Pinmux_Config, etc.).
  list(APPEND SRCS rtl8730e_flash_stubs.c)
endif()

if(CONFIG_AMEBA_GPIO)
  list(APPEND SRCS ${AMEBA_COMMON}/ameba_gpio.c)
  # rtl8730e_flash_stubs.c is also needed for GPIO (irq_enable / Pinmux_Config
  # called by lib_rom.a GPIO functions).  Avoid duplicate if already added
  # above.
  if(NOT CONFIG_RTL8730E_FLASH_FS)
    list(APPEND SRCS rtl8730e_flash_stubs.c)
  endif()
endif()

target_include_directories(arch PRIVATE ${AMEBA_COMMON})
target_sources(arch PRIVATE ${SRCS})

# ##############################################################################
# Vendor-SDK build machinery (fwlib + WiFi glue archives, prebuilt libs, and
# link flags).  RTL8730E (CA32) differs from the KM4-based ICs: - No SDK
# autoconf / image2 ldscript generation: the board uses its own dramboot.ld and
# a static prebuilt platform_autoconf.h. - No NP firmware build: KM0/KM4 are
# prebuilt blobs in boards/.../prebuilt/. - No -mcmse (CA32 is ARMv7-A, not
# Cortex-M33). - lib_rom.a provides ROM functions (GPIO_Init, Pinmux_Config,
# flash MTD).
# ##############################################################################

# Per-IC identifiers (parallel to the names in ameba_board.mk).
set(AMEBA_SOC_NAME amebasmart)
set(AMEBA_PY_SOC RTL8730E)
set(AMEBA_AP_PROJECT ap)
set(AMEBA_KM_PROJ project_ap)

# Resolve AMEBA_SDK / asdk toolchain (provisioned by `. tools/ameba/env.sh`).
include(${AMEBA_COMMON}/cmake/ameba_sdk.cmake)

set(AMEBA_SOC ${AMEBA_SDK}/component/soc/${AMEBA_SOC_NAME})
set(AMEBA_PREBUILT ${NUTTX_BOARD_DIR}/prebuilt)
set(AMEBA_PREBUILT_LIBS ${AMEBA_PREBUILT}/libs)
set(AMEBA_AUTOCONF ${AMEBA_PREBUILT}/platform_autoconf.h)
set(AMEBA_WIFI_DIR ${AMEBA_COMMON}/wifi)
file(MAKE_DIRECTORY ${AMEBA_PREBUILT_LIBS})

# ----------------------------------------------------------------------------
# ameba_build_lib: compile SDK sources with an isolated flag set (the vendor
# headers clash with NuttX's), archive into a prebuilt .a.  Mirrors the make
# PREBUILD loop in ameba_board.mk.
# ----------------------------------------------------------------------------
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
      DEPENDS ${_src}
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

# ----------------------------------------------------------------------------
# libameba_fwlib.a -- minimal fwlib subset needed by flash MTD + IPC (and the
# WiFi IPC path).  Built from SDK source with the amebasmart CA32 include set.
# Linked only when FLASH_FS is enabled (mirrors ameba_board.mk).
# ----------------------------------------------------------------------------

set(AMEBA_FWLIB_A ${AMEBA_PREBUILT_LIBS}/libameba_fwlib.a)

if(CONFIG_RTL8730E_FLASH_FS OR CONFIG_RTL8730E_WIFI)
  set(AMEBA_FWLIB_SRCS
      ${AMEBA_SOC}/fwlib/ram_common/ameba_arch.c
      ${AMEBA_SOC}/swlib/log.c
      ${AMEBA_SOC}/swlib/sscanf_minimal.c
      ${AMEBA_SOC}/fwlib/ram_common/ameba_ipc_api.c
      ${AMEBA_SOC}/fwlib/ram_common/ameba_ipc_ram.c)

  if(CONFIG_RTL8730E_FLASH_FS)
    list(APPEND AMEBA_FWLIB_SRCS
         ${AMEBA_SOC}/fwlib/ram_common/ameba_flash_ram.c)
  endif()

  # Silence SDK warnings, activate CA32 code paths.  No -mcmse (ARMv7-A).
  set(AMEBA_FWLIB_INC
      -Wno-int-conversion
      -Wno-shadow
      -DCONFIG_ARM_CORE_CA32
      -I${AMEBA_COMMON}/sdk_shim
      -I${AMEBA_SOC}/fwlib/include
      -I${AMEBA_SOC}/fwlib/include/rom
      -I${AMEBA_SOC}/swlib
      -I${AMEBA_SOC}/hal/include
      -I${AMEBA_SOC}/hal/src
      -I${AMEBA_SDK}/component/soc/common/include
      -I${AMEBA_SDK}/component/soc/common/include/cmsis
      -I${AMEBA_SOC}/app/monitor/include
      -I${AMEBA_SDK}/component/soc/usrcfg/${AMEBA_SOC_NAME}/include
      -I${AMEBA_SDK}/component/soc/usrcfg/common
      -I${AMEBA_SOC}/misc
      -I${AMEBA_SDK}/component/os/os_wrapper/include
      -I${AMEBA_PREBUILT}
      -I${AMEBA_SDK}/component/ssl/mbedtls-3.6.2/include
      -I${AMEBA_SDK}/component/os/freertos/heap_trace)

  ameba_build_lib(${AMEBA_FWLIB_A} fwlib_obj AMEBA_FWLIB_SRCS AMEBA_FWLIB_INC
                  _fwlib_target)
  add_dependencies(nuttx ${_fwlib_target})
endif()

# ----------------------------------------------------------------------------
# Extra libraries linked into NuttX.
# ----------------------------------------------------------------------------

set(AMEBA_EXTRA_LIBS)

# lib_rom.a: prebuilt AP ROM stubs (GPIO_Init, RCC_PeriphClockCmd, PAD_PullCtrl,
# Pinmux_Config, flash MTD primitives).
if(CONFIG_AMEBA_GPIO OR CONFIG_RTL8730E_FLASH_FS)
  list(APPEND AMEBA_EXTRA_LIBS
       ${AMEBA_SOC}/project/${AMEBA_KM_PROJ}/lib/soc/lib_rom.a)
endif()

# fwlib archive linked for flash MTD (provides ameba_flash_ram.c symbols).
if(CONFIG_RTL8730E_FLASH_FS)
  list(APPEND AMEBA_EXTRA_LIBS ${AMEBA_FWLIB_A})
endif()

# VFS1 geometry injected as -D macros via target_compile_definitions.
# set_property(SOURCE ...) has scope issues in NuttX's include()-based CMake
# structure; target-level definitions are reliable and harmless on other files.
if(CONFIG_RTL8730E_FLASH_FS AND EXISTS ${AMEBA_AUTOCONF})
  file(STRINGS ${AMEBA_AUTOCONF} _l REGEX "CONFIG_FLASH_VFS1_OFFSET")
  if(_l)
    list(GET _l 0 _l0)
    string(REGEX REPLACE ".*CONFIG_FLASH_VFS1_OFFSET[ \t]+([0-9xXa-fA-F]+).*"
                         "\\1" _vfs1_off "${_l0}")
    target_compile_definitions(arch
                               PRIVATE AMEBA_FLASH_VFS1_OFFSET_XIP=${_vfs1_off})
  endif()
  file(STRINGS ${AMEBA_AUTOCONF} _l REGEX "CONFIG_FLASH_VFS1_SIZE[ \t]")
  if(_l)
    list(GET _l 0 _l0)
    string(REGEX REPLACE ".*CONFIG_FLASH_VFS1_SIZE[ \t]+([0-9xXa-fA-F]+).*"
                         "\\1" _vfs1_size "${_l0}")
    target_compile_definitions(arch
                               PRIVATE AMEBA_FLASH_VFS1_SIZE_CFG=${_vfs1_size})
  endif()
endif()

# ----------------------------------------------------------------------------
# WiFi glue archive + prebuilt WHC host libs.
# ----------------------------------------------------------------------------

if(CONFIG_RTL8730E_WIFI)
  set(AMEBA_WIFI_SRCS
      ${AMEBA_SDK}/component/soc/usrcfg/${AMEBA_SOC_NAME}/ameba_wificfg.c
      ${AMEBA_SDK}/component/wifi/common/rtw_task_size.c
      ${AMEBA_SDK}/component/wifi/common/rtw_event.c
      ${AMEBA_SDK}/component/soc/common/diagnose/ameba_diagnose_none.c
      ${AMEBA_SDK}/component/wifi/wpa_supplicant/wpa_supplicant/wifi_p2p_disable.c
      ${AMEBA_WIFI_DIR}/ameba_wifi_depend.c
      ${AMEBA_WIFI_DIR}/ameba_wifi.c)

  set(AMEBA_WIFI_INC
      -include
      ${AMEBA_AUTOCONF}
      -include
      ${AMEBA_WIFI_DIR}/include/ameba_lwip_off.h
      -I${AMEBA_WIFI_DIR}/include
      -I${AMEBA_WIFI_DIR}/..
      -I${AMEBA_SDK}/component/wifi/api
      -I${AMEBA_SDK}/component/wifi/common
      -I${AMEBA_SDK}/component/wifi/driver/include
      -I${AMEBA_SDK}/component/wifi/driver/intf
      -I${AMEBA_SDK}/component/wifi/whc
      -I${AMEBA_SDK}/component/wifi/whc/whc_host_rtos
      -I${AMEBA_SDK}/component/wifi/whc/whc_host_rtos/ipc
      -I${AMEBA_SDK}/component/at_cmd
      -I${AMEBA_SDK}/component/wifi/wpa_supplicant/wpa_supplicant
      -I${AMEBA_SDK}/component/wifi/wpa_supplicant/wpa_lite
      -I${AMEBA_SDK}/component/wifi/wpa_supplicant/wpa_lite/rom
      -I${AMEBA_SDK}/component/wifi/wpa_supplicant/src
      -I${AMEBA_SDK}/component/wifi/wpa_supplicant/src/utils
      -I${AMEBA_SDK}/component/wifi/rtk_app/wifi_auto_reconnect
      -I${AMEBA_SDK}/component/network
      -I${AMEBA_SDK}/component/soc/common/diagnose
      -I${AMEBA_SOC}/app/monitor/include
      -I${AMEBA_SOC}/fwlib/include
      -I${AMEBA_SOC}/fwlib/include/rom
      -I${AMEBA_SOC}/swlib
      -I${AMEBA_SOC}/hal/include
      -I${AMEBA_SOC}/misc
      -I${AMEBA_SDK}/component/soc/common/include
      -I${AMEBA_SDK}/component/soc/common/include/cmsis
      -I${AMEBA_SDK}/component/os/os_wrapper/include
      -I${AMEBA_SDK}/component/soc/usrcfg/${AMEBA_SOC_NAME}/include
      -I${AMEBA_SDK}/component/soc/usrcfg/common
      -I${AMEBA_SDK}/component/ssl/mbedtls-3.6.2/include
      -I${AMEBA_SDK}/component/os/freertos/heap_trace
      -I${AMEBA_PREBUILT})

  set(AMEBA_WIFI_A ${AMEBA_PREBUILT_LIBS}/libameba_wifi.a)
  ameba_build_lib(${AMEBA_WIFI_A} wifi_obj AMEBA_WIFI_SRCS AMEBA_WIFI_INC
                  _wifi_target)
  add_dependencies(nuttx ${_wifi_target})

  set(_ameba_ap_lib ${AMEBA_SOC}/project/${AMEBA_KM_PROJ}/lib/application)
  list(
    APPEND
    AMEBA_EXTRA_LIBS
    ${_ameba_ap_lib}/lib_wifi_whc_ap.a
    ${_ameba_ap_lib}/lib_wifi_rtk_app.a
    ${_ameba_ap_lib}/lib_wifi_com_sec.a
    ${_ameba_ap_lib}/lib_wpa_lite.a
    ${AMEBA_WIFI_A})
endif()

# Suppress the VFP-ABI mismatch warning between lib_rom.a / WiFi libs (compiled
# without hard-float) and the rest of the NuttX build (compiled with
# hard-float).
if(AMEBA_EXTRA_LIBS)
  target_link_options(nuttx PRIVATE -Wl,--no-warn-mismatch)
endif()

# Append inside the top-level --start-group/--end-group so cross-references
# between the SDK archives and the NuttX libs resolve correctly.
set_property(GLOBAL APPEND PROPERTY NUTTX_EXTRA_LIBRARIES ${AMEBA_EXTRA_LIBS})

# ----------------------------------------------------------------------------
# `flash` target -- download boot.bin + nuttx.bin via ameba_smart_flash.sh.
# Usage: AMEBA_PORT=/dev/ttyUSB1 [AMEBA_BAUD=1500000] cmake --build <dir>
# --target flash
# ----------------------------------------------------------------------------

set(AMEBA_TOOLS_DIR ${AMEBA_COMMON}/tools)

if(EXISTS ${AMEBA_TOOLS_DIR}/ameba_smart_flash.sh AND NOT TARGET flash)
  add_custom_target(
    flash
    DEPENDS nuttx
    COMMAND ${CMAKE_COMMAND} -E env sh ${AMEBA_TOOLS_DIR}/ameba_smart_flash.sh
            ${AMEBA_SDK} ${AMEBA_PREBUILT} ${NUTTX_BINARY_DIR}/nuttx.bin
    WORKING_DIRECTORY ${NUTTX_BINARY_DIR}
    USES_TERMINAL VERBATIM
    COMMENT "Flashing RTL8730E over ${AMEBA_PORT}")
endif()
