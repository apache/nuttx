############################################################################
# boards/arm/stm32l4/nucleo-l476rg/src/Makefile
#
#   Copyright (C) 2014 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
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
ifeq ($(CONFIG_AJOYSTICK),y)
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

ifeq ($(CONFIG_LIB_BOARDCTL),y)
CSRCS += stm32_appinit.c
endif

include $(TOPDIR)/boards/Board.mk
