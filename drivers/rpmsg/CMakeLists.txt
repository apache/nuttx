# ##############################################################################
# drivers/rpmsg/CMakeLists.txt
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

if(CONFIG_RPMSG)
  set(SRCS)

  list(APPEND SRCS rpmsg.c)

  if(CONFIG_RPMSG_PING)
    list(APPEND SRCS rpmsg_ping.c)
  endif()

  if(CONFIG_RPMSG_PORT)
    list(APPEND SRCS rpmsg_port.c)
    target_include_directories(drivers
                               PRIVATE ${NUTTX_DIR}/openamp/open-amp/lib)
  endif()

  if(CONFIG_RPMSG_PORT_SPI)
    list(APPEND SRCS rpmsg_port_spi.c)
  endif()

  if(CONFIG_RPMSG_PORT_SPI_SLAVE)
    list(APPEND SRCS rpmsg_port_spi_slave.c)
  endif()

  if(CONFIG_RPMSG_PORT_UART)
    list(APPEND SRCS rpmsg_port_uart.c)
  endif()

  if(CONFIG_RPMSG_VIRTIO_LITE)
    list(APPEND SRCS rpmsg_virtio_lite.c)
  endif()

  if(CONFIG_RPMSG_VIRTIO_IVSHMEM)
    list(APPEND SRCS rpmsg_virtio_ivshmem.c)
  endif()

  if(CONFIG_RPMSG_ROUTER)
    list(APPEND SRCS rpmsg_router_hub.c rpmsg_router_edge.c)
  endif()

  target_include_directories(drivers PRIVATE ${NUTTX_DIR}/openamp/open-amp/lib)
  target_sources(drivers PRIVATE ${SRCS})
endif()
