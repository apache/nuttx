# ##############################################################################
# drivers/wireless/ieee80211/morse/CMakeLists.txt
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

if(CONFIG_IEEE80211_MORSE)

  if(CONFIG_ARCH_CORTEXM7)
    set(MORSE_CORE arm-cortex-m7f)
  elseif(CONFIG_ARCH_CORTEXM33)
    set(MORSE_CORE arm-cortex-m33f)
  else()
    set(MORSE_CORE arm-cortex-m4f)
  endif()

  if(CONFIG_IEEE80211_MORSE_MM8108)
    set(MORSE_CHIP mm8108)
    set(MORSE_FW_NAME mm8108b2-rl.mbin)
  else()
    set(MORSE_CHIP mm6108)
    set(MORSE_FW_NAME mm6108.mbin)
  endif()

  # MM-IoT-SDK files: only the files used by the driver are downloaded from the
  # SDK repository (about 3 MB), see Make.defs.

  set(MORSE_SDK_HEADERS
      emmet.h
      mbin.h
      mm_mbedtls_config.h
      mmhal.h
      mmhal_app.h
      mmhal_core.h
      mmhal_flash.h
      mmhal_os.h
      mmhal_uart.h
      mmhal_wlan.h
      mmlog.h
      mmosal.h
      mmpkt.h
      mmpkt_list.h
      mmtrace.h
      mmversion.h
      mmwlan.h
      mmwlan_stats.h)

  list(TRANSFORM MORSE_SDK_HEADERS PREPEND framework/morselib/include/
                                           OUTPUT_VARIABLE MORSE_SDK_FILES)
  list(
    APPEND
    MORSE_SDK_FILES
    framework/morselib/lib/${MORSE_CORE}/libmorse.a
    framework/src/mmregdb/mmregdb.c
    framework/src/mmregdb/mmregdb.h
    framework/src/mmpktmem/mmpktmem_heap.c
    framework/src/mmutils/mmutils.h
    framework/morsefirmware/${MORSE_FW_NAME}
    LICENSES/Apache-2.0.txt
    LICENSES/LicenseRef-MorseMicroBDL.txt)

  if(CONFIG_IEEE80211_MORSE_BCF_QUECTEL_FGH100MH)
    list(APPEND MORSE_SDK_FILES
         framework/tools/buildsystem/convert-bin-to-mbin.py)
  elseif(CONFIG_IEEE80211_MORSE_BCF_SDK)
    list(
      APPEND
      MORSE_SDK_FILES
      framework/morsefirmware/${MORSE_CHIP}/bcfs/${CONFIG_IEEE80211_MORSE_BCF_NAME}
    )
  endif()

  # Files already present are not downloaded again: to build without network
  # access copy them in MORSE_SDK beforehand.

  set(MORSE_SDK
      ${CMAKE_CURRENT_LIST_DIR}/mm-iot-sdk-${CONFIG_IEEE80211_MORSE_SDK_VERSION}
  )
  set(MORSE_SDK_URL
      https://raw.githubusercontent.com/MorseMicro/mm-iot-sdk/${CONFIG_IEEE80211_MORSE_SDK_VERSION}
  )
  foreach(file ${MORSE_SDK_FILES})
    if(NOT EXISTS ${MORSE_SDK}/${file})
      message(STATUS "Downloading: MM-IoT-SDK ${file}")
      file(DOWNLOAD ${MORSE_SDK_URL}/${file} ${MORSE_SDK}/${file}.tmp
           STATUS MORSE_DL_STATUS)
      list(GET MORSE_DL_STATUS 0 MORSE_DL_ERROR)
      if(MORSE_DL_ERROR)
        file(REMOVE ${MORSE_SDK}/${file}.tmp)
        message(FATAL_ERROR "Failed to download ${MORSE_SDK_URL}/${file}")
      endif()
      file(RENAME ${MORSE_SDK}/${file}.tmp ${MORSE_SDK}/${file})
    endif()
  endforeach()

  set(MORSE_FRAMEWORK ${MORSE_SDK}/framework)
  set(MORSE_FW ${MORSE_FRAMEWORK}/morsefirmware/${MORSE_FW_NAME})

  # Board configuration file

  if(CONFIG_IEEE80211_MORSE_BCF_QUECTEL_FGH100MH)
    set(MORSE_BCF_COMMIT 919f27a6dc2d46a988682d6ff64dd3fdb9ffc465)
    set(MORSE_BCF_ELF ${CMAKE_CURRENT_LIST_DIR}/bcf_fgh100mhaamd.bin)
    set(MORSE_BCF ${CMAKE_CURRENT_LIST_DIR}/bcf_fgh100mhaamd.mbin)
    if(NOT EXISTS ${MORSE_BCF})
      file(
        DOWNLOAD
        https://raw.githubusercontent.com/MorseMicro/morse-firmware/${MORSE_BCF_COMMIT}/bcf/quectel/bcf_fgh100mhaamd.bin
        ${MORSE_BCF_ELF}
        STATUS MORSE_BCF_STATUS)
      list(GET MORSE_BCF_STATUS 0 MORSE_DL_ERROR)
      if(MORSE_DL_ERROR)
        message(FATAL_ERROR "Failed to download the FGH100M-H BCF")
      endif()

      # The BCF is distributed in ELF format: convert it to mbin (requires
      # python3 with pyelftools)

      execute_process(
        COMMAND
          python3 -I
          ${MORSE_FRAMEWORK}/tools/buildsystem/convert-bin-to-mbin.py -o
          ${MORSE_BCF} ${MORSE_BCF_ELF} COMMAND_ERROR_IS_FATAL ANY)
    endif()
  elseif(CONFIG_IEEE80211_MORSE_BCF_SDK)
    set(MORSE_BCF
        ${MORSE_FRAMEWORK}/morsefirmware/${MORSE_CHIP}/bcfs/${CONFIG_IEEE80211_MORSE_BCF_NAME}
    )
  else()
    set(MORSE_BCF ${CONFIG_IEEE80211_MORSE_BCF_PATH})
  endif()

  # Pre-built libmorse

  nuttx_add_extra_library(
    ${MORSE_FRAMEWORK}/morselib/lib/${MORSE_CORE}/libmorse.a)

  set(SRCS
      morse_mmosal.c morse_mmhal.c morse_netdev.c
      ${MORSE_FRAMEWORK}/src/mmregdb/mmregdb.c
      ${MORSE_FRAMEWORK}/src/mmpktmem/mmpktmem_heap.c)

  set_property(
    SOURCE morse_mmhal.c
    APPEND
    PROPERTY OBJECT_DEPENDS ${MORSE_FW} ${MORSE_BCF})

  target_compile_definitions(drivers PRIVATE MORSE_FW_FILE=${MORSE_FW}
                                             MORSE_BCF_FILE=${MORSE_BCF})

  target_include_directories(
    drivers
    PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${MORSE_FRAMEWORK}/morselib/include
            ${MORSE_FRAMEWORK}/src/mmregdb ${MORSE_FRAMEWORK}/src/mmutils)

  target_sources(drivers PRIVATE ${SRCS})

endif()
