############################################################################
# boards/arm/rtl8721f/rtl8721f_evb/src/Makefile
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

CSRCS = rtl8721f_boot.c rtl8721f_bringup.c

ifeq ($(CONFIG_AMEBA_GPIO),y)
CSRCS += rtl8721f_gpio.c
endif

ifeq ($(CONFIG_AMEBA_UART),y)
CSRCS += rtl8721f_uart.c
endif

ifeq ($(CONFIG_AMEBA_I2C),y)
CSRCS += rtl8721f_i2c.c
endif

ifeq ($(CONFIG_AMEBA_SPI),y)
CSRCS += rtl8721f_spi.c
endif

ifeq ($(CONFIG_AMEBA_PWM),y)
CSRCS += rtl8721f_pwm.c
endif

ifeq ($(CONFIG_AMEBA_ADC),y)
CSRCS += rtl8721f_adc.c
endif

ifeq ($(CONFIG_AMEBA_RTC),y)
CSRCS += rtl8721f_rtc.c
endif

ifeq ($(CONFIG_AMEBA_WDG),y)
CSRCS += rtl8721f_wdg.c
endif

ifeq ($(CONFIG_AMEBA_TIMER),y)
CSRCS += rtl8721f_timer.c
endif

# The board pin tables pull in the shared drivers' public headers from
# arch/arm/src/common/ameba/, which is not on the default board include path.

ifneq ($(CONFIG_AMEBA_GPIO)$(CONFIG_AMEBA_UART)$(CONFIG_AMEBA_I2C)$(CONFIG_AMEBA_SPI)$(CONFIG_AMEBA_PWM)$(CONFIG_AMEBA_ADC)$(CONFIG_AMEBA_RTC)$(CONFIG_AMEBA_WDG)$(CONFIG_AMEBA_TIMER),)
CFLAGS += ${INCDIR_PREFIX}$(TOPDIR)$(DELIM)arch$(DELIM)arm$(DELIM)src$(DELIM)common$(DELIM)ameba
endif

include $(TOPDIR)/boards/Board.mk
