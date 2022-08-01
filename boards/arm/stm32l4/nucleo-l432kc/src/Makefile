############################################################################
# boards/arm/stm32l4/nucleo-l432kc/src/Makefile
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

ifeq ($(CONFIG_DEV_GPIO),y)
  CSRCS += stm32_gpio.c
endif

ifeq ($(CONFIG_STM32L4_ADC),y)
CSRCS += stm32_adc.c
endif

ifeq ($(CONFIG_DAC7571),y)
CSRCS += stm32_dac7571.c
endif

ifeq ($(CONFIG_MTD_AT45DB),y)
CSRCS += stm32_at45db.c
endif

ifeq ($(CONFIG_SENSORS_INA226),y)
CSRCS += stm32_ina226.c
endif

ifeq ($(CONFIG_SENSORS_INA219),y)
CSRCS += stm32_ina219.c
endif

ifeq ($(CONFIG_SENSORS_QENCODER),y)
CSRCS += stm32_qencoder.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += stm32_pwm.c
endif

ifeq ($(CONFIG_SENSORS_ZEROCROSS),y)
  CSRCS += stm32_zerocross.c
endif

ifeq ($(CONFIG_TIMER),y)
CSRCS += stm32_timer.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += stm32_appinit.c
endif

ifeq ($(CONFIG_NUCLEOL432KC_SPWM),y)
CSRCS += stm32_spwm.c
endif

ifeq ($(CONFIG_NUCLEOL432KC_DAC_WGEN),y)
CSRCS += stm32_dac_wgen.c
endif

include $(TOPDIR)/boards/Board.mk
