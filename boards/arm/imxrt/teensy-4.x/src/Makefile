############################################################################
# boards/arm/imxrt/teensy-4.x/src/Makefile
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

CSRCS = imxrt_boot.c imxrt_flexspi_nor_boot.c imxrt_flexspi_nor_flash.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += imxrt_appinit.c imxrt_bringup.c
else ifeq ($(CONFIG_BOARD_LATE_INITIALIZE),y)
CSRCS += imxrt_bringup.c
endif

ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += imxrt_reset.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += imxrt_autoleds.c
else
CSRCS += imxrt_userleds.c
endif

ifeq ($(CONFIG_IMXRT_LPSPI),y)
CSRCS += imxrt_spi.c
endif

ifeq ($(CONFIG_IMXRT_LPI2C),y)
CSRCS += imxrt_i2c.c
endif

ifeq ($(CONFIG_IMXRT_FLEXCAN),y)
CSRCS += imxrt_flexcan.c
endif

ifeq ($(CONFIG_IMXRT_FLEXPWM),y)
CSRCS += imxrt_flexpwm.c
endif

ifeq ($(CONFIG_IMXRT_ENET),y)
CSRCS += imxrt_ethernet.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += imxrt_gpio.c
endif

ifeq ($(CONFIG_IMXRT_ADC),y)
CSRCS += imxrt_adc.c
endif

ifeq ($(CONFIG_LCD_ST7789),y)
CSRCS += imxrt_st7789.c
endif

ifeq ($(CONFIG_IMXRT_ENC),y)
CSRCS += imxrt_enc.c
endif



include $(TOPDIR)/boards/Board.mk
