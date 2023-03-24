############################################################################
# boards/arm/stm32f7/nucleo-144/src/Makefile
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

CSRCS = stm32_boot.c stm32_bringup.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += stm32_autoleds.c
else
CSRCS += stm32_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS +=  stm32_buttons.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += stm32_appinitialize.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += stm32_gpio.c
endif

ifeq ($(CONFIG_SPI),y)
CSRCS += stm32_spi.c
endif

ifeq ($(CONFIG_ADC),y)
CSRCS += stm32_adc.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += stm32_pwm.c
endif

ifeq ($(CONFIG_MMCSD),y)
CSRCS += stm32_sdio.c
endif

ifeq ($(CONFIG_STM32F7_OTGFS),y)
CSRCS += stm32_usb.c
endif

ifeq ($(CONFIG_STM32F7_BBSRAM),y)
CSRCS += stm32_bbsram.c
endif

ifeq ($(CONFIG_BOARDCTL_RESET),y)
CSRCS += stm32_reset.c
endif

ifeq ($(CONFIG_STM32_ROMFS),y)
CSRCS += stm32_romfs_initialize.c
endif

ifeq ($(CONFIG_SENSORS_QENCODER),y)
CSRCS += stm32_qencoder.c
endif

ifeq ($(CONFIG_STM32F7_CAN),y)
ifeq ($(CONFIG_STM32F7_CAN_CHARDRIVER),y)
CSRCS += stm32_can.c
endif
ifeq ($(CONFIG_STM32F7_CAN_SOCKET),y)
CSRCS += stm32_cansock.c
endif
endif

ifeq ($(CONFIG_USBDEV_COMPOSITE),y)
CSRCS += stm32_composite.c
endif

include $(TOPDIR)/boards/Board.mk
