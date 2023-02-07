# ##############################################################################
# drivers/usbhost/CMakeLists.txt
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

if(CONFIG_USBHOST)

  # Include built-in USB host driver logic

  set(SRCS usbhost_registry.c usbhost_registerclass.c usbhost_findclass.c
           usbhost_enumerate.c usbhost_devaddr.c)

  if(CONFIG_USBHOST_HUB)
    list(APPEND SRCS usbhost_hub.c)
  endif()

  if(CONFIG_USBHOST_COMPOSITE)
    list(APPEND SRCS usbhost_composite.c)
  endif()

  if(CONFIG_USBHOST_MSC)
    list(APPEND SRCS usbhost_storage.c)
  endif()

  if(CONFIG_USBHOST_CDCACM)
    list(APPEND SRCS usbhost_cdcacm.c)
  endif()

  if(CONFIG_USBHOST_CDCMBIM)
    list(APPEND SRCS usbhost_cdcmbim.c)
  endif()

  if(CONFIG_USBHOST_HID)
    list(APPEND SRCS hid_parser.c)
  endif()

  if(CONFIG_USBHOST_HIDKBD)
    list(APPEND SRCS usbhost_hidkbd.c)
  endif()

  if(CONFIG_USBHOST_HIDMOUSE)
    list(APPEND SRCS usbhost_hidmouse.c)
  endif()

  if(CONFIG_USBHOST_XBOXCONTROLLER)
    list(APPEND SRCS usbhost_xboxcontroller.c)
  endif()

  if(CONFIG_USBHOST_MAX3421E)
    list(APPEND SRCS usbhost_max3421e.c)
  endif()

  if(CONFIG_USBHOST_FT232R)
    list(APPEND SRCS usbhost_ft232r.c)
  endif()

  # HCD debug/trace logic

  if(CONFIG_USBHOST_TRACE)
    list(APPEND SRCS usbhost_trace.c)
  elseif(CONFIG_DEBUG_USB)
    list(APPEND SRCS usbhost_trace.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
