# ##############################################################################
# openamp/open-amp.cmake
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
if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/open-amp/.git)
  FetchContent_Declare(
    open-amp
    DOWNLOAD_NAME "libopen-amp-v${OPENAMP_VERSION}.zip"
    DOWNLOAD_DIR ${CMAKE_CURRENT_LIST_DIR}
    URL "https://github.com/OpenAMP/open-amp/archive/v${OPENAMP_VERSION}.zip"
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
    DOWNLOAD_NO_PROGRESS true
    TIMEOUT 30)

  FetchContent_GetProperties(open-amp)

  if(NOT open-amp_POPULATED)
    FetchContent_Populate(open-amp)
  endif()

  if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/.openamp_patch)
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_LIST_DIR}/.openamp_patch
      COMMAND touch ${CMAKE_CURRENT_LIST_DIR}/.openamp_patch
      COMMAND
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0001-openamp-add-ns_unbind_notify-support.patch
        > /dev/null || (exit 0)
      COMMAND
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0002-ns-acknowledge-the-received-creation-message.patch
        > /dev/null || (exit 0)
      COMMAND
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0003-Negotiate-individual-buffer-size-dynamically.patch
        > /dev/null || (exit 0)
      COMMAND
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0004-rpmsg-wait-endpoint-ready-in-rpmsg_send-and-rpmsg_se.patch
        > /dev/null || (exit 0)
      COMMAND
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0005-openamp-add-new-ops-notify_wait-support.patch
        > /dev/null || (exit 0)
      COMMAND
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0006-openamp-divide-shram-to-TX-shram-RX-shram-by-config-.patch
        > /dev/null || (exit 0)
      COMMAND
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0007-openamp-don-t-need-check-status-when-get_tx_payload.patch
        > /dev/null || (exit 0)
      COMMAND
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0008-openamp-add-available_idx-to-dump.patch >
        /dev/null || (exit 0)
      COMMAND
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0009-openamp-firstly-take-all-buffer-from-shram-pool.patch
        > /dev/null || (exit 0)
      COMMAND
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0010-rpmsg-notify-the-user-when-the-remote-address-is-rec.patch
        > /dev/null || (exit 0)
      COMMAND
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0011-openamp-avoid-double-calling-ns_bound-when-each-othe.patch
        > /dev/null || (exit 0)
      COMMAND
        patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
        ${CMAKE_CURRENT_LIST_DIR}/0012-remoteproc-make-all-elf_-functions-static-except-elf.patch
        > /dev/null || (exit 0)
      DEPENDS open-amp)
    add_custom_target(openamp_patch
                      DEPENDS ${CMAKE_CURRENT_LIST_DIR}/.openamp_patch)
  endif()
endif()

nuttx_add_kernel_library(openamp)
target_sources(
  openamp
  PRIVATE open-amp/lib/remoteproc/elf_loader.c
          open-amp/lib/remoteproc/remoteproc.c
          open-amp/lib/remoteproc/remoteproc_virtio.c
          open-amp/lib/remoteproc/rsc_table_parser.c
          open-amp/lib/rpmsg/rpmsg.c
          open-amp/lib/rpmsg/rpmsg_virtio.c
          open-amp/lib/virtio/virtio.c
          open-amp/lib/virtio/virtqueue.c)

if(TARGET openamp_patch)
  add_dependencies(openamp openamp_patch)
endif()
