# ##############################################################################
# drivers/usbdev/CMakeLists.txt
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

if(CONFIG_USBDEV)
  set(SRCS)

  # Include USB device drivers

  if(CONFIG_PL2303)
    list(APPEND SRCS pl2303.c)
  endif()

  if(CONFIG_CDCACM)
    list(APPEND SRCS cdcacm.c cdcacm_desc.c)
  endif()

  if(CONFIG_USBMSC)
    list(APPEND SRCS usbmsc.c usbmsc_desc.c usbmsc_scsi.c)
  endif()

  if(CONFIG_USBDEV_COMPOSITE)
    list(APPEND SRCS composite_desc.c)
  endif()

  if(CONFIG_USBDEV_TRACE_STRINGS)
    list(APPEND SRCS usbdev_strings.c)
  endif()

  if(CONFIG_RNDIS)
    list(APPEND SRCS rndis.c)
  endif()

  if(CONFIG_DFU)
    list(APPEND SRCS dfu.c)
  endif()

  if(CONFIG_USBADB)
    list(APPEND SRCS adb.c)
  endif()

  if(CONFIG_NET_CDCECM)
    list(APPEND SRCS cdcecm.c)
  endif()

  if(CONFIG_NET_CDCNCM)
    list(APPEND SRCS cdcncm.c)
  endif()

  if(CONFIG_USBDEV_FS)
    list(APPEND SRCS usbdev_fs.c)
  endif()

  list(APPEND SRCS composite.c usbdev_desc.c usbdev_req.c)
  list(APPEND SRCS usbdev_trace.c usbdev_trprintf.c)

  target_sources(drivers PRIVATE ${SRCS})
endif()
