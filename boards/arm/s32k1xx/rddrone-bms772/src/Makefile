############################################################################
# boards/arm/s32k1xx/rddrone-bms772/src/Makefile
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

CSRCS  = s32k1xx_boot.c s32k1xx_bringup.c s32k1xx_clockconfig.c
CSRCS += s32k1xx_periphclocks.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += s32k1xx_autoleds.c
else
CSRCS += s32k1xx_userleds.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += s32k1xx_appinit.c
endif

ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += s32k1xx_reset.c
endif

ifeq ($(CONFIG_BOARDCTL_UNIQUEID),y)
CSRCS += s32k1xx_uid.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += s32k1xx_gpio.c
endif

ifeq ($(CONFIG_S32K1XX_LPI2C),y)
CSRCS += s32k1xx_i2c.c
endif

ifeq ($(CONFIG_LCD_SSD1306_I2C),y)
CSRCS += s32k1xx_ssd1306.c
endif

ifeq ($(CONFIG_S32K1XX_LPSPI),y)
CSRCS += s32k1xx_spi.c
endif

ifeq ($(CONFIG_S32K1XX_NRSTCHECK_PROCFS),y)
CSRCS += s32k1xx_nrstcheck_procfs.c
endif

ifeq ($(CONFIG_SMBUS_SBD),y)
CSRCS += s32k1xx_smbus_sbd.c
endif

include $(TOPDIR)/boards/Board.mk
