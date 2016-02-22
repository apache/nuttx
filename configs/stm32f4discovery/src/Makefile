############################################################################
# configs/stm32f4discovery/src/Makefile
#
#   Copyright (C) 2011-2013, 2015-2016 Gregory Nutt. All rights reserved.
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

-include $(TOPDIR)/Make.defs

ASRCS =
CSRCS = stm32_boot.c stm32_bringup.c stm32_spi.c

ifeq ($(CONFIG_ARCH_LEDS),y)
CSRCS += stm32_autoleds.c
else
CSRCS += stm32_userleds.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
CSRCS +=  stm32_buttons.c
endif

ifeq ($(CONFIG_STM32_OTGFS),y)
CSRCS += stm32_usb.c
endif

ifeq ($(CONFIG_BMP180),y)
CSRCS += stm32_bmp180.c
endif

ifeq ($(CONFIG_MAX31855),y)
CSRCS += stm32_max31855.c
endif

ifeq ($(CONFIG_MAX6675),y)
CSRCS += stm32_max6675.c
endif

ifeq ($(CONFIG_PCA9635PW),y)
CSRCS += stm32_pca9635.c
endif

ifeq ($(CONFIG_STM32_SDIO),y)
CSRCS += stm32_sdio.c
endif

ifeq ($(CONFIG_STM32_ETHMAC),y)
CSRCS += stm32_ethernet.c
endif

ifeq ($(CONFIG_RGBLED),y)
CSRCS += stm32_rgbled.c
endif

ifeq ($(CONFIG_PWM),y)
CSRCS += stm32_pwm.c
endif

ifeq ($(CONFIG_QENCODER),y)
CSRCS += stm32_qencoder.c
endif

ifeq ($(CONFIG_WATCHDOG),y)
CSRCS += stm32_watchdog.c
endif

ifeq ($(CONFIG_NSH_LIBRARY),y)
CSRCS += stm32_appinit.c
endif

ifeq ($(CONFIG_ARCH_CUSTOM_PMINIT),y)
CSRCS += stm32_pm.c
endif

ifeq ($(CONFIG_PM_BUTTONS),y)
CSRCS += stm32_pmbuttons.c
endif

ifeq ($(CONFIG_ZEROCROSS),y)
CSRCS += stm32_zerocross.c
endif

ifeq ($(CONFIG_ARCH_IDLE_CUSTOM),y)
CSRCS += stm32_idle.c
endif

ifeq ($(CONFIG_STM32_FSMC),y)
CSRCS += stm32_extmem.c

ifeq ($(CONFIG_LCD_SSD1289),y)
CSRCS += stm32_ssd1289.c
endif
endif

ifeq ($(CONFIG_LCD_SSD1351),y)
CSRCS += stm32_ssd1351.c
endif

ifeq ($(CONFIG_LCD_UG2864AMBAG01),y)
CSRCS += stm32_ug2864ambag01.c
endif

ifeq ($(CONFIG_LCD_UG2864HSWEG01),y)
CSRCS += stm32_ug2864hsweg01.c
endif

ifeq ($(CONFIG_TIMER),y)
CSRCS += stm32_timer.c
endif

ifeq ($(CONFIG_BOARDCTL_UNIQUEID),y)
CSRCS += stm32_uid.c
endif

include $(TOPDIR)/configs/Board.mk
