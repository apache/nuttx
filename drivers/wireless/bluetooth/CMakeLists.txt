# ##############################################################################
# drivers/wireless/bluetooth/CMakeLists.txt
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

if(CONFIG_DRIVERS_BLUETOOTH)
  set(SRCS)

  list(APPEND SRCS bt_driver.c)

  if(CONFIG_BLUETOOTH_UART)
    list(APPEND SRCS bt_uart.c)

    if(CONFIG_BLUETOOTH_UART_GENERIC)
      list(APPEND SRCS bt_uart_generic.c)
    endif()
    if(CONFIG_BLUETOOTH_UART_SHIM)
      list(APPEND SRCS bt_uart_shim.c)
    endif()
    if(CONFIG_BLUETOOTH_UART_CC2564)
      list(APPEND SRCS bt_uart_cc2564.c)
    endif()
    if(CONFIG_BLUETOOTH_BCM4343X)
      list(APPEND SRCS bt_uart_bcm4343x.c)
    endif()
  endif()

  if(CONFIG_BLUETOOTH_NULL)
    list(APPEND SRCS bt_null.c)
  endif()

  if(CONFIG_BLUETOOTH_RPMSG_SERVER)
    list(APPEND SRCS bt_rpmsghci_server.c)
  endif()

  if(CONFIG_BLUETOOTH_RPMSG)
    list(APPEND SRCS bt_rpmsghci.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
