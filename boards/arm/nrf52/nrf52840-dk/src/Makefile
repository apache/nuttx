############################################################################
# boards/arm/nrf52/nrf52840-dk/src/Makefile
#
# Licensed to the Apache Software Foundation (ASF) under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.  The
# ASF licenses this file to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the
# License.  You may obtain a copy of the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations
# under the License.
#
############################################################################

include $(TOPDIR)/Make.defs

CSRCS = nrf52_boot.c nrf52_bringup.c

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += nrf52_appinit.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += nrf52_autoleds.c
else
CSRCS += nrf52_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += nrf52_buttons.c
endif

ifeq ($(CONFIG_NRF52_SPI_MASTER),y)
CSRCS += nrf52_spi.c
endif

ifeq ($(CONFIG_SENSORS_LSM6DSL),y)
CSRCS += nrf52_lsm6dsl.c
endif

ifeq ($(CONFIG_SENSORS_LSM303AGR),y)
CSRCS += nrf52_lsm303agr.c
endif

ifeq ($(CONFIG_SENSORS_HTS221),y)
CSRCS += nrf52_hts221.c
endif

ifeq ($(CONFIG_LPWAN_SX127X),y)
CSRCS += nrf52_sx127x.c
endif

ifeq ($(CONFIG_NRF52840DK_HIGHPRI),y)
CSRCS += nrf52_highpri.c
endif

ifeq ($(CONFIG_TIMER),y)
ifeq ($(CONFIG_NRF52_TIMER),y)
CSRCS += nrf52_timer.c
endif
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += nrf52_pwm.c
endif

ifeq ($(CONFIG_ADC),y)
CSRCS += nrf52_adc.c
endif

ifeq ($(CONFIG_USBDEV),y)
CSRCS += nrf52_usb.c
endif

ifeq ($(CONFIG_USBDEV_COMPOSITE),y)
CSRCS += nrf52_composite.c
endif

include $(TOPDIR)/boards/Board.mk
