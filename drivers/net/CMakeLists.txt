# ##############################################################################
# drivers/net/CMakeLists.txt
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

if(CONFIG_NET)
  set(SRCS)

  # Include network interface drivers

  if(CONFIG_MM_IOB)
    list(APPEND SRCS netdev_upperhalf.c)
  endif()

  if(CONFIG_NET_LOOPBACK)
    list(APPEND SRCS loopback.c)
  endif()

  if(CONFIG_NET_RPMSG_DRV)
    list(APPEND SRCS rpmsgdrv.c)
  endif()

  if(CONFIG_NETDEV_TELNET)
    list(APPEND SRCS telnet.c)
  endif()

  if(CONFIG_NET_DM90x0)
    list(APPEND SRCS dm90x0.c)
  endif()

  if(CONFIG_ENC28J60)
    list(APPEND SRCS enc28j60.c)
  endif()

  if(CONFIG_ENCX24J600)
    list(APPEND SRCS encx24j600.c)
  endif()

  if(CONFIG_NET_SLIP)
    list(APPEND SRCS slip.c)
  endif()

  if(CONFIG_NET_TUN)
    list(APPEND SRCS tun.c)
  endif()

  if(CONFIG_NET_FTMAC100)
    list(APPEND SRCS ftmac100.c)
  endif()

  if(CONFIG_NET_LAN91C111)
    list(APPEND SRCS lan91c111.c)
  endif()

  if(CONFIG_NET_SKELETON)
    list(APPEND SRCS skeleton.c)
  endif()

  if(CONFIG_NET_W5500)
    list(APPEND SRCS w5500.c)
  endif()

  if(CONFIG_NET_LAN9250)
    list(APPEND SRCS lan9250.c)
  endif()

  if(CONFIG_NET_E1000)
    list(APPEND SRCS e1000.c)
  endif()

  if(CONFIG_NET_IGC)
    list(APPEND SRCS igc.c)
  endif()

  if(CONFIG_ARCH_PHY_INTERRUPT)
    list(APPEND SRCS phy_notify.c)
  endif()

  if(CONFIG_DRIVERS_WIFI_SIM)
    list(APPEND SRCS wifi_sim.c)
  endif()

  target_sources(drivers PRIVATE ${SRCS})
endif()
