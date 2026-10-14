# ##############################################################################
# arch/arm/src/rm57/CMakeLists.txt
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

set(SRCS
    rm57_boot.c
    rm57_clockconfig.c
    rm57_mpuinit.c
    rm57_gio.c
    rm57_gioirq.c
    rm57_irq.c
    rm57_lowputc.c
    rm57_serial.c
    rm57_esm.c
    rm57_iomm.c
    rm57_coreinit.S)

if(CONFIG_RM57_SELFTEST)
  list(APPEND SRCS rm57_selftest.c)
  if(CONFIG_RM57_SELFTEST_PBIST_ROM)
    list(APPEND SRCS rm57_errata.c)
  endif()
endif()

if(NOT CONFIG_SCHED_TICKLESS)
  list(APPEND SRCS rm57_timerisr.c)
endif()

if(CONFIG_RM57_DMA)
  list(APPEND SRCS rm57_dma.c)
endif()

if(CONFIG_RM57_LIN)
  list(APPEND SRCS rm57_lin.c)
endif()

if(CONFIG_RM57_MIBSPI)
  list(APPEND SRCS rm57_mibspi.c)
endif()

if(CONFIG_RM57_I2C)
  list(APPEND SRCS rm57_i2c.c)
endif()

if(CONFIG_RM57_ADC)
  list(APPEND SRCS rm57_adc.c)
  if(CONFIG_RM57_ADC_TEMPSENSOR)
    list(APPEND SRCS rm57_adc_temp.c)
  endif()
endif()

if(CONFIG_RM57_DCAN)
  list(APPEND SRCS rm57_dcan.c)
  if(CONFIG_CAN)
    list(APPEND SRCS rm57_dcan_chardev.c)
  endif()
  if(CONFIG_NET_CAN)
    list(APPEND SRCS rm57_dcan_sock.c)
  endif()
endif()

if(CONFIG_RM57_EMAC)
  list(APPEND SRCS rm57_ethernet.c rm57_mdio.c)
endif()

if(CONFIG_NET AND NOT CONFIG_NETDEV_LATEINIT)
  list(APPEND SRCS rm57_netinitialize.c)
endif()

target_sources(arch PRIVATE ${SRCS})
