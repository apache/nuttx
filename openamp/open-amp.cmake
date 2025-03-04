# ##############################################################################
# openamp/open-amp.cmake
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
if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/open-amp)
  FetchContent_Declare(
    open-amp
    DOWNLOAD_NAME "libopen-amp-main.zip"
    DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
    URL "https://github.com/OpenAMP/open-amp/archive/${OPENAMP_COMMIT}.zip"
        SOURCE_DIR
        ${CMAKE_CURRENT_LIST_DIR}/open-amp
        BINARY_DIR
        ${CMAKE_BINARY_DIR}/openamp/open-amp
        CONFIGURE_COMMAND
        ""
        BUILD_COMMAND
        ""
        INSTALL_COMMAND
        ""
        TEST_COMMAND
        ""
    PATCH_COMMAND
      patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0001-ns-acknowledge-the-received-creation-message.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0002-Negotiate-individual-buffer-size-dynamically.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0003-rpmsg-notify-the-user-when-the-remote-address-is-rec.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0004-openamp-virtio.h-negotiate_features-also-can-be-call.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0005-remoteproc-rpmsg_virtio-change-sched_yeild-to-usleep.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0006-rpmsg-wait-ept-ready-in-rpmsg_send.patch &&
      patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0007-openamp-add-VIRTIO_RING_F_MUST_NOTIFY-event.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0008-rpmsg_virtio-don-t-need-check-status-when-get_tx_pay.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0009-openamp-swap-get_rx_buffer-return_rx_buffer-to-resol.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0010-rpmsg_virtio.c-virtqueue_kick-after-all-rx-buffer-re.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0011-virtio-change-feature-to-64-bit-in-all-virtio_dispat.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0012-rpmsg_virtio.c-fix-get_tx_payload_buffer-error.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0013-openamp-add-assert-when-get-tx-buffer-failed.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0014-virtio.h-add-memory-operation-for-virtio-device.patch
    DOWNLOAD_NO_PROGRESS true
    TIMEOUT 30)

  FetchContent_GetProperties(open-amp)

  if(NOT open-amp_POPULATED)
    FetchContent_Populate(open-amp)
  endif()
endif()

if(CONFIG_OPENAMP_CACHE)
  set(WITH_DCACHE_VRINGS ON)
endif()

if(CONFIG_OPENAMP_DEBUG)
  add_compile_definitions(RPMSG_DEBUG)
  add_compile_definitions(VQUEUE_DEBUG)
endif()

add_compile_definitions(elf_load=remoteproc_elf_load)

if(CONFIG_OPENAMP_VIRTIO_DEVICE_SUPPORT)
  add_compile_definitions(VIRTIO_DEVICE_SUPPORT=1)
else()
  add_compile_definitions(VIRTIO_DEVICE_SUPPORT=0)
endif()

if(CONFIG_OPENAMP_VIRTIO_DRIVER_SUPPORT)
  add_compile_definitions(VIRTIO_DRIVER_SUPPORT=1)
else()
  add_compile_definitions(VIRTIO_DRIVER_SUPPORT=0)
endif()

set(WITH_LIBMETAL_FIND OFF)
set(WITH_PROXY OFF)

if(NOT CMAKE_SYSTEM_PROCESSOR)
  set(CMAKE_SYSTEM_PROCESSOR ${CONFIG_ARCH})
endif()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/open-amp
                 ${CMAKE_CURRENT_BINARY_DIR}/open-amp EXCLUDE_FROM_ALL)

target_include_directories(
  open_amp-static PRIVATE $<TARGET_PROPERTY:metal-static,INCLUDE_DIRECTORIES>)

nuttx_add_external_library(open_amp-static MODE KERNEL)
