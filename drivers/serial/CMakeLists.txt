# ##############################################################################
# drivers/serial/CMakeLists.txt
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

# Include serial drivers

set(SRCS serial.c serial_io.c)

if(CONFIG_SERIAL_GDBSTUB)
  list(APPEND SRCS serial_gdbstub.c)
endif()

if(CONFIG_CMSDK_UART)
  list(APPEND SRCS serial_cmsdk.c)
endif()

if(CONFIG_SERIAL_RXDMA)
  list(APPEND SRCS serial_dma.c)
elseif(CONFIG_SERIAL_TXDMA)
  list(APPEND SRCS serial_dma.c)
endif()

if(CONFIG_16550_UART)
  list(APPEND SRCS uart_16550.c)
endif()

if(CONFIG_UART_PL011)
  list(APPEND SRCS uart_pl011.c)
endif()

if(CONFIG_RPMSG_UART)
  list(APPEND SRCS uart_rpmsg.c)
endif()

if(CONFIG_UART_HOSTFS)
  list(APPEND SRCS uart_hostfs.c)
endif()

# Pseudo-terminal support

if(CONFIG_PSEUDOTERM)
  list(APPEND SRCS pty.c)
  if(CONFIG_PSEUDOTERM_SUSV1)
    list(APPEND SRCS ptmx.c)
  endif()
endif()

# Bluetooth H:4 UART driver

if(CONFIG_UART_BTH4)
  list(APPEND SRCS uart_bth4.c)
endif()

# UART PCI drivers

if(CONFIG_16550_PCI_UART)
  list(APPEND SRCS uart_pci_16550.c)
endif()

# Bluetooth H:5 UART driver

if(CONFIG_UART_BTH5)
  list(APPEND SRCS uart_bth5.c)
endif()

# RAM uart support

if(CONFIG_RAM_UART)
  list(APPEND SRCS uart_ram.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
