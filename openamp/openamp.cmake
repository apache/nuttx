############################################################################
# openamp/open-amp.defs
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

if (CONFIG_OPENAMP)

  FetchContent_Declare(openamp
    URL https://github.com/OpenAMP/open-amp/archive/v${OPENAMP_VERSION}.zip
    PATCH_COMMAND
      patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0001-rpmsg-remove-the-address-check-in-rpmsg_send-rpmsg_t.patch &&
      patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0002-rpmsg-merge-rpmsg_register_endpoint-into-rpmsg_init_.patch &&
      patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0003-rpmsg-shouldn-t-allocate-0-1023-address-in-rpmsg_cre.patch &&
      patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0004-rpmsg-wait-ept-ready-in-rpmsg_send.patch &&
      patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0005-rpmsg-return-fail-if-either-source-or-destination-ad.patch &&
      patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0006-remoteproc_mmap-support-va-to-pa-da-conversion.patch &&
      patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0007-rpmsg-bring-back-zero-copy-transfer.patch &&
      patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0008-ns-acknowledge-the-received-creation-message.patch &&
      patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0009-implement-rproc_virtio_read_config-rproc_virtio_writ.patch &&
      patch -p1 < ${CMAKE_CURRENT_LIST_DIR}/0010-Negotiate-individual-buffer-size-dynamically.patch)

    FetchContent_Populate(openamp)
    FetchContent_GetProperties(openamp SOURCE_DIR OPENAMP_SOURCE_DIR)

  set(SRCS
    lib/remoteproc/elf_loader.c
    lib/remoteproc/remoteproc.c
    lib/remoteproc/remoteproc_virtio.c
    lib/remoteproc/rsc_table_parser.c
    lib/rpmsg/rpmsg.c
    lib/rpmsg/rpmsg_virtio.c
    lib/virtio/virtio.c
    lib/virtio/virtqueue.c)

  list(TRANSFORM SRCS PREPEND ${OPENAMP_SOURCE_DIR}/)
  target_sources(openamp PRIVATE ${SRCS})

  set_property(TARGET nuttx APPEND PROPERTY NUTTX_INCLUDE_DIRECTORIES ${OPENAMP_SOURCE_DIR}/lib/include)
endif()
