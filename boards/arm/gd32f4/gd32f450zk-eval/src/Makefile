############################################################################
# boards/arm/gd32f4/gd32f450zk-eval/src/Make.defs
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

CSRCS = gd32f4xx_boot.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += gd32f4xx_appinit.c
endif
# ifeq ($(CONFIG_BOARDCTL),y)
# CSRCS += gd32f4xx_appinit.c gd32f4xx_bringup.c
# else ifeq ($(CONFIG_BOARD_LATE_INITIALIZE),y)
# CSRCS += gd32f4xx_bringup.c
# endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += gd32f4xx_autoleds.c
else
CSRCS += gd32f4xx_userleds.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += gd32f4xx_gpio.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS +=  gd32f4xx_buttons.c
endif

ifeq ($(CONFIG_SPI),y)
CSRCS +=  gd32f4xx_spi.c
endif

ifeq ($(CONFIG_MTD_GD25),y)
CSRCS += gd32f4xx_gd25.c
endif

include $(TOPDIR)/boards/Board.mk
