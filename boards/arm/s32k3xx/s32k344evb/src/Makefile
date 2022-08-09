############################################################################
# boards/arm/s32k3xx/s32k344evb/src/Makefile
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

# Copyright 2022 NXP

include $(TOPDIR)/Make.defs

CSRCS  = s32k3xx_boot.c s32k3xx_bringup.c s32k3xx_clockconfig.c
CSRCS += s32k3xx_periphclocks.c

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += s32k3xx_buttons.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += s32k3xx_autoleds.c
else
CSRCS += s32k3xx_userleds.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += s32k3xx_appinit.c
endif

ifeq ($(CONFIG_S32K3XX_LPI2C),y)
CSRCS += s32k3xx_i2c.c
endif

ifeq ($(CONFIG_S32K3XX_LPSPI),y)
CSRCS += s32k3xx_spi.c
endif

include $(TOPDIR)/boards/Board.mk
