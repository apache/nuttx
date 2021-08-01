############################################################################
# boards/arm/stm32f7/stm32f746-ws/src/Makefile
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

include $(TOPDIR)/Make.defs

CSRCS = stm32_boot.c stm32_spi.c stm32_dma_alloc.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += stm32_appinitialize.c
endif

ifeq ($(CONFIG_STM32F7_OTGFS),y)
CSRCS += stm32_usb.c
endif

ifeq ($(CONFIG_STM32F7_SDMMC1),y)
CSRCS += stm32_sdmmc.c
endif

include $(TOPDIR)/boards/Board.mk
