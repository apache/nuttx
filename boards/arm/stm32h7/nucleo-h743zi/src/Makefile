############################################################################
# boards/arm/stm32h7/nucleo-h743zi/src/Makefile
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

ifeq ($(CONFIG_ADC),y)
CSRCS += stm32_adc.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += stm32_autoleds.c
else
CSRCS += stm32_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS += stm32_buttons.c
endif

ifeq ($(CONFIG_STM32_ROMFS),y)
CSRCS += stm32_romfs_initialize.c
endif

ifeq ($(CONFIG_STM32H7_SPI),y)
CSRCS += stm32_spi.c
endif

ifeq ($(CONFIG_STM32H7_OTGFS),y)
CSRCS += stm32_usb.c
endif

ifeq ($(CONFIG_BOARDCTL_UNIQUEID),y)
CSRCS += stm32_uid.c
endif

ifeq ($(CONFIG_SENSORS_LSM6DSL),y)
CSRCS += stm32_lsm6dsl.c
endif

ifeq ($(CONFIG_SENSORS_LSM9DS1),y)
CSRCS += stm32_lsm9ds1.c
endif

ifeq ($(CONFIG_SENSORS_LSM303AGR),y)
CSRCS += stm32_lsm303agr.c
endif

ifeq ($(CONFIG_PCA9635PW),y)
CSRCS += stm32_pca9635.c
endif

ifeq ($(CONFIG_LCD_SSD1306),y)
CSRCS += stm32_ssd1306.c
endif

ifeq ($(CONFIG_BOARDCTL),y)
CSRCS += stm32_appinitialize.c
endif

ifeq ($(CONFIG_WL_NRF24L01),y)
CSRCS += stm32_nrf24l01.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
CSRCS += stm32_gpio.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += stm32_pwm.c
endif

ifeq ($(CONFIG_BOARDCTL_RESET), y)
CSRCS += stm32_reset.c
endif

include $(TOPDIR)/boards/Board.mk
