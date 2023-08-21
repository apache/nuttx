# ##############################################################################
# drivers/segger/CMakeLists.txt
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

if(CONFIG_SEGGER_RTT)

  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/RTT)

    set(RTT_VERSION 7.54)

    FetchContent_Declare(
      rtt
      DOWNLOAD_NAME "RTT.zip"
      DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
      URL "https://github.com/SEGGERMicro/RTT/archive/refs/tags/V${RTT_VERSION}.zip"
          SOURCE_DIR
          ${CMAKE_CURRENT_LIST_DIR}/RTT
          BINARY_DIR
          ${CMAKE_BINARY_DIR}/RTT
      PATCH_COMMAND
        patch -p1 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0001-segger-RTT-include-the-SEGGER_RTT_Conf.h-from-custom.patch
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 30)

    FetchContent_GetProperties(rtt)

    if(NOT rtt_POPULATED)
      FetchContent_Populate(rtt)
    endif()

  endif()

  nuttx_add_kernel_library(RTT)

  set(RTT_SRCS RTT/RTT/SEGGER_RTT.c)

  if(CONFIG_SEGGER_RTT_SECTION)
    target_compile_options(
      RTT PRIVATE -DSEGGER_RTT_SECTION=${CONFIG_SEGGER_RTT_SECTION})
  endif()

  if(CONFIG_SEGGER_RTT_BUFFER_SECTION)
    target_compile_options(
      RTT
      PRIVATE -DSEGGER_RTT_BUFFER_SECTION=${CONFIG_SEGGER_RTT_BUFFER_SECTION})
  endif()

  if(CONFIG_ARCH_ARMV7M)
    list(APPEND RTT_SRCS RTT/RTT/SEGGER_RTT_ASM_ARMv7M.S)
  endif()

  if(CONFIG_STREAM_RTT)
    list(APPEND RTT_SRCS stream_rtt.c)
  endif()

  if(CONFIG_SYSLOG_RTT)
    list(APPEND RTT_SRCS syslog_rtt.c)
  endif()

  if(CONFIG_NOTE_RTT)
    list(APPEND RTT_SRCS note_rtt.c)
  endif()

  if(CONFIG_SERIAL_RTT)
    list(APPEND RTT_SRCS serial_rtt.c)
  endif()

  target_compile_options(RTT PRIVATE -Wno-shadow -Wno-array-bounds)
  target_sources(RTT PRIVATE ${RTT_SRCS})
  target_include_directories(RTT PRIVATE config RTT/RTT)

endif()

if(CONFIG_SEGGER_SYSVIEW)

  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/SystemView)

    set(SYSVIEW_VERSION 3.30)

    FetchContent_Declare(
      systemview
      DOWNLOAD_NAME "SystemView.zip"
      DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
      URL "https://github.com/SEGGERMicro/SystemView/archive/refs/tags/V${SYSVIEW_VERSION}.zip"
          SOURCE_DIR
          ${CMAKE_CURRENT_LIST_DIR}/SystemView
          BINARY_DIR
          ${CMAKE_BINARY_DIR}/SystemView
      DOWNLOAD_NO_PROGRESS true
      TIMEOUT 30)

    FetchContent_GetProperties(systemview)

    if(NOT systemview_POPULATED)
      FetchContent_Populate(systemview)
    endif()
  endif()

  nuttx_add_kernel_library(SystemView)

  target_compile_options(SystemView PRIVATE -Wno-shadow)

  target_sources(SystemView PRIVATE note_sysview.c
                                    SystemView/SYSVIEW/SEGGER_SYSVIEW.c)

  target_include_directories(
    SystemView PRIVATE ../../sched config RTT/RTT SystemView/SEGGER
                       SystemView/SYSVIEW)

endif()
