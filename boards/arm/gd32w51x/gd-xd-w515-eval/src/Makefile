############################################################################
# boards/arm/gd32w51x/gd-xd-w515-eval/src/Makefile
#
# SPDX-License-Identifier: Apache-2.0
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

CSRCS = gd32_boot.c gd32_bringup.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += gd32_appinit.c
endif

ifeq ($(CONFIG_USERLED),y)
CSRCS += gd32_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += gd32_buttons.c
endif

ifeq ($(CONFIG_GD32W51X_SPI),y)
CSRCS += gd32_spi.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += gd32_gpio.c
endif

ifeq ($(CONFIG_LCD),y)
CSRCS += gd32_lcd.c
endif

ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += gd32_reset.c
endif

ifeq ($(CONFIG_GD_XD_W515_EVAL_QSPI_XIP),y)
CSRCS += gd32_qspixip.c
endif

include $(TOPDIR)/boards/Board.mk
