# ##############################################################################
# arch/arm/src/rtl8720f/CMakeLists.txt
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
set(SRCS ameba_allocateheap.c ameba_irq.c ameba_loguart.c ameba_start.c
         ameba_timerisr.c)

set(AMEBA_COMMON ${CMAKE_CURRENT_LIST_DIR}/../common/ameba)

list(APPEND SRCS ${AMEBA_COMMON}/ameba_os_wrap.c)

# km4tz<->km4ns IPC bring-up: needed by the SDK flash erase/program path
# (inter-core XIP pause) AND by WiFi, so compile it whenever either is enabled.
if(CONFIG_RTL8720F_WIFI OR CONFIG_RTL8720F_FLASH_FS)
  list(APPEND SRCS ameba_ipc.c)
endif()

if(CONFIG_RTL8720F_WIFI)
  list(APPEND SRCS ameba_wifi_init.c ${AMEBA_COMMON}/ameba_kv.c)
  if(CONFIG_NET)
    list(APPEND SRCS ${AMEBA_COMMON}/ameba_wlan.c)
  endif()
endif()

if(CONFIG_RTL8720F_FLASH_FS)
  list(APPEND SRCS ${AMEBA_COMMON}/ameba_flash_mtd.c)
endif()

target_include_directories(arch PRIVATE ${AMEBA_COMMON})
target_sources(arch PRIVATE ${SRCS})

# ##############################################################################
# Vendor-SDK build machinery (shared mechanism in common/ameba/cmake).  This
# IC's differing inputs are set here (RTL8720F: km4tz AP core, km4ns NP core).
# ##############################################################################

set(AMEBA_SOC_NAME RTL8720F)
set(AMEBA_PY_SOC RTL8720F)
set(AMEBA_AP_PROJECT km4tz)
set(AMEBA_KM_PROJ project_km4tz)
set(AMEBA_NP_TARGET km4ns)
set(AMEBA_CFG_WIFI ${CONFIG_RTL8720F_WIFI})
set(AMEBA_CFG_FLASHFS ${CONFIG_RTL8720F_FLASH_FS})

# Resolve AMEBA_SDK / asdk toolchain (provisioned by `. tools/ameba/env.sh`)
# before the SDK-relative source lists below reference it.
include(${AMEBA_COMMON}/cmake/ameba_sdk.cmake)

set(AMEBA_SOC ${AMEBA_SDK}/component/soc/${AMEBA_SOC_NAME})
set(AMEBA_PREBUILT ${NUTTX_BOARD_DIR}/prebuilt)
set(AMEBA_WIFI_DIR ${AMEBA_COMMON}/wifi)

# fwlib register-layer sources compiled into libameba_fwlib.a, plus the
# NuttX-owned image2 entry (ameba_app_start.c, IC-specific).
set(AMEBA_FWLIB_SRCS
    ${AMEBA_SOC}/fwlib/ram_common/ameba_arch.c
    ${AMEBA_SOC}/fwlib/ram_common/ameba_loguart.c
    ${AMEBA_SOC}/fwlib/ram_common/ameba_ipc_api.c
    ${AMEBA_SOC}/fwlib/ram_common/ameba_ipc_ram.c
    ${AMEBA_SOC}/fwlib/ram_common/ameba_clk.c
    ${AMEBA_SOC}/fwlib/ram_common/ameba_pmctimer.c
    ${AMEBA_SOC}/fwlib/ram_common/ameba_pmu.c
    ${CMAKE_CURRENT_LIST_DIR}/ameba_app_start.c
    ${AMEBA_SOC}/fwlib/ram_common/ameba_mpu_ram.c
    ${AMEBA_SOC}/fwlib/ram_km4tz/ameba_data_flashclk.c
    ${AMEBA_SOC}/fwlib/ram_km4tz/ameba_flashclk.c
    ${AMEBA_SOC}/fwlib/ram_km4tz/ameba_pinmap.c
    ${AMEBA_SDK}/component/soc/usrcfg/${AMEBA_SOC_NAME}/ameba_flashcfg.c
    ${AMEBA_SDK}/component/soc/usrcfg/${AMEBA_SOC_NAME}/ameba_pinmapcfg.c)

if(CONFIG_RTL8720F_FLASH_FS)
  list(APPEND AMEBA_FWLIB_SRCS ${AMEBA_SOC}/fwlib/ram_common/ameba_flash_ram.c)
endif()

# Silence a couple of warnings the vendored SDK sources trip under NuttX's
# warning set, scoped to this fwlib compile only (never relaxing NuttX's own):
# -Wno-int-conversion: the SDK passes NULL to irq_register()'s u32 "Data"
# (interrupt context) argument in many places -- an intentional idiom.
# -Wno-shadow: SDK swlib/log.c shadows a file-scope global with a parameter.
set(AMEBA_FWLIB_INC
    -Wno-int-conversion
    -Wno-shadow
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
    -I${AMEBA_SDK}/component/ssl/mbedtls-3.6.5/include
    -I${AMEBA_SDK}/component/soc/common/crashdump/include
    -I${AMEBA_PREBUILT})

# Host WiFi glue -> libameba_wifi.a (identical source/include set to the other
# Ameba ICs; shim include first, SDK autoconf force-included).
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
    ${AMEBA_PREBUILT}/platform_autoconf.h
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
    -I${AMEBA_SDK}/component/ssl/mbedtls-3.6.5/include
    -I${AMEBA_PREBUILT})

# KM4TZ-side host WiFi control libs.  Unlike RTL8721Dx, lib_coex.a is NP-side
# (km4ns) on RTL8720F and is NOT linked into the AP image.
set(_ameba_app_lib ${AMEBA_SOC}/project/${AMEBA_KM_PROJ}/lib/application)
set(AMEBA_WIFI_APP_LIBS
    ${_ameba_app_lib}/lib_wifi_whc_ap.a ${_ameba_app_lib}/lib_wifi_rtk_app.a
    ${_ameba_app_lib}/lib_wifi_com_sec.a ${_ameba_app_lib}/lib_wpa_lite.a)

# RTL8720F pulls the ROM symbol implementations from lib_rom.a via a
# whole-archive link (TrustZone off in this config -> lib_rom.a, not
# lib_rom_tz.a), in addition to the ROM symbol addresses appended to the
# generated ld script.
set(_ameba_soc_lib ${AMEBA_SOC}/project/${AMEBA_KM_PROJ}/lib/soc)
set(AMEBA_EXTRA_LINK_OPTIONS -Wl,--whole-archive ${_ameba_soc_lib}/lib_rom.a
                             -Wl,--no-whole-archive)

include(${AMEBA_COMMON}/cmake/ameba_board.cmake)
