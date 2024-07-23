############################################################################
# boards/arm/at32/at32f437-mini/src/Makefile
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

CSRCS = at32_boot.c at32_bringup.c

ifeq ($(CONFIG_BOARDCTL),y)
	CSRCS += at32_appinit.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
	CSRCS += at32_gpio.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
	CSRCS += at32_autoleds.c
else
	CSRCS += at32_userleds.c
endif

ifeq ($(CONFIG_SPI),y)
	CSRCS += at32_spi.c
endif

ifeq ($(CONFIG_MTD_W25),y)
	CSRCS += at32_w25.c
endif

ifeq ($(CONFIG_AT32_CAN_CHARDRIVER),y)
	CSRCS += at32_can.c
endif

ifeq ($(CONFIG_AT32_CAN_SOCKET),y)
	CSRCS += at32_cansock.c
endif

ifeq ($(CONFIG_AT32_ETHMAC),y)
	CSRCS += at32_ethernet.c
endif

ifeq ($(CONFIG_AT32_OTGFS),y)
	CSRCS += at32_usb.c
endif

ifeq ($(CONFIG_AT32_SDIO),y)
	CSRCS += at32_mmcsd.c
endif

ifeq ($(CONFIG_AT32_PWM),y)
	CSRCS += at32_pwm.c
endif

ifeq ($(CONFIG_TIMER),y)
	CSRCS += at32_timer.c
endif

ifeq ($(CONFIG_AT32_ADC),y)
	CSRCS += at32_adc.c
endif

ifeq ($(CONFIG_MTD_AT24XX),y)
	CSRCS += at32_at24.c
endif

ifeq ($(CONFIG_ETC_ROMFS),y)
	CSRCS += etc_romfs.c
endif

include $(TOPDIR)/boards/Board.mk
