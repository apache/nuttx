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
if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/open-amp)
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
    PATCH_COMMAND
      patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0001-ns-acknowledge-the-received-creation-message.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0002-Negotiate-individual-buffer-size-dynamically.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0003-rpmsg-wait-endpoint-ready-in-rpmsg_send-and-rpmsg_se.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0004-openamp-add-new-ops-notify_wait-support.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0005-rpmsg_virtio-don-t-need-check-status-when-get_tx_pay.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0006-rpmsg-notify-the-user-when-the-remote-address-is-rec.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0007-openamp-avoid-double-calling-ns_bound-when-each-othe.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0008-remoteproc-make-all-elf_-functions-static-except-elf.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0009-Fix-warn-declaration-of-vring_rsc-shadows-a-previous.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0010-rptun-fix-rptun-don-t-wait-issue-when-get-tx-patyloa.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0011-rpmsg-fix-rpmsg_virtio_get_tx_buffer-no-idx-return.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0012-rpmsg-add-new-API-rpdev_release_tx-rx_buffer.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0013-openamp-add-error-log-when-ept-cb-return-error.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0014-rpmsg-add-cache-flash-when-hold-rx-buffer.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0015-rpmsg-do-cache_invalidate-when-real-data-returned.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0016-openamp-add-new-API-rpmsg_virtio_get_rxbuffer_size.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0017-virtio-follow-virtio-1.2-spec-add-more-virtio-status.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0018-virtio-decoupling-the-transport-layer-and-virtio-dev.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0019-virtio.h-add-version-in-device-id-table.patch
      && patch -p0 -d ${CMAKE_CURRENT_LIST_DIR} <
      ${CMAKE_CURRENT_LIST_DIR}/0020-virtio-Add-the-virtio_negotiate_features-interface.patch
    DOWNLOAD_NO_PROGRESS true
    TIMEOUT 30)

  FetchContent_GetProperties(open-amp)

  if(NOT open-amp_POPULATED)
    FetchContent_Populate(open-amp)
  endif()
endif()

nuttx_add_kernel_library(openamp)

if(CONFIG_OPENAMP_CACHE)
  target_compile_options(openamp PRIVATE -DVIRTIO_CACHED_BUFFERS)
  target_compile_options(openamp PRIVATE -DVIRTIO_CACHED_VRINGS)
endif()

if(CONFIG_OPENAMP_RPMSG_DEBUG)
  target_compile_options(openamp PRIVATE -DRPMSG_DEBUG)
endif()

if(CONFIG_OPENAMP_VQUEUE_DEBUG)
  target_compile_options(openamp PRIVATE -DVQUEUE_DEBUG)
endif()

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
