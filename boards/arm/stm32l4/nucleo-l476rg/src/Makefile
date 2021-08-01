############################################################################
# boards/arm/stm32l4/nucleo-l476rg/src/Makefile
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

CSRCS = stm32_boot.c stm32_spi.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += stm32_autoleds.c
else
CSRCS += stm32_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS +=  stm32_buttons.c
endif

ifeq ($(CONFIG_WL_CC1101),y)
CSRCS += stm32_cc1101.c
endif

ifeq ($(CONFIG_ADC),y)
CSRCS += stm32_adc.c
ifeq ($(CONFIG_INPUT_AJOYSTICK),y)
CSRCS += stm32_ajoystick.c
endif
endif

# Ben vd Veen (DisruptiveNL) -- 31-03-2018
ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += stm32_gpio.c
endif

# Ben vd Veen (DisruptiveNL) -- 31-03-2018
ifeq ($(CONFIG_CAN),y)
CSRCS += stm32_can.c
endif

ifeq ($(CONFIG_MMCSD_SPI),y)
CSRCS += stm32_spimmcsd.c
endif

ifeq ($(CONFIG_LCD_PCD8544),y)
CSRCS += stm32_pcd8544.c
endif

ifeq ($(CONFIG_SENSORS_QENCODER),y)
CSRCS += stm32_qencoder.c
endif

ifeq ($(CONFIG_SENSORS_HTS221),y)
CSRCS += stm32_hts221.c
endif

ifeq ($(CONFIG_SENSORS_LSM6DSL),y)
CSRCS += stm32_lsm6dsl.c
endif

ifeq ($(CONFIG_SENSORS_LSM303AGR),y)
CSRCS += stm32_lsm303agr.c
endif

ifeq ($(CONFIG_SENSORS_AS726X),y)
CSRCS += stm32_as726x.c
endif

ifeq ($(CONFIG_SENSORS_BMP180),y)
CSRCS += stm32_bmp180.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += stm32_pwm.c
endif

ifeq ($(CONFIG_TIMER),y)
CSRCS += stm32_timer.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += stm32_appinit.c
endif

include $(TOPDIR)/boards/Board.mk
