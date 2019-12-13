############################################################################
# boards/arm/stm32/stm32f103-minimum/src/Makefile
#
#   Copyright (C) 2016-2017 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#           Laurent Latil <laurent@latil.nom.fr>
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

ifeq ($(CONFIG_LIB_BOARDCTL),y)
  CSRCS += stm32_appinit.c
endif

ifeq ($(CONFIG_ARCH_BUTTONS),y)
  CSRCS += stm32_buttons.c
endif

ifeq ($(CONFIG_ARCH_LEDS),y)
  CSRCS += stm32_autoleds.c
else
  CSRCS += stm32_userleds.c
endif

ifeq ($(CONFIG_ADC),y)
CSRCS +=  stm32_adc.c
endif

ifeq ($(CONFIG_DEV_GPIO),y)
  CSRCS += stm32_gpio.c
endif

ifeq ($(CONFIG_PWM),y)
  CSRCS += stm32_pwm.c
endif

ifeq ($(CONFIG_SENSORS_ZEROCROSS),y)
  CSRCS += stm32_zerocross.c
endif

ifeq ($(CONFIG_LCD_BACKPACK),y)
  CSRCS += stm32_lcd_backpack.c
endif

ifeq ($(CONFIG_LEDS_APA102),y)
  CSRCS += stm32_apa102.c
endif

ifeq ($(CONFIG_SENSORS_BMP180),y)
  CSRCS += stm32_bmp180.c
endif

ifeq ($(CONFIG_LM75_I2C),y)
  CSRCS += stm32_lm75.c
endif

ifeq ($(CONFIG_RGBLED),y)
  CSRCS += stm32_rgbled.c
endif

ifeq ($(CONFIG_MMCSD),y)
  CSRCS += stm32_mmcsd.c
endif

ifeq ($(CONFIG_MTD_W25),y)
  CSRCS += stm32_w25.c
endif

ifeq ($(CONFIG_MTD_AT24XX),y)
ifeq ($(CONFIG_STM32_I2C1),y)
CSRCS += stm32_at24.c
endif
endif

ifeq ($(CONFIG_AUDIO_TONE),y)
  CSRCS += stm32_tone.c
endif

ifeq ($(CONFIG_CAN_MCP2515),y)
  CSRCS += stm32_mcp2515.c
endif

ifeq ($(CONFIG_CL_MFRC522),y)
  CSRCS += stm32_mfrc522.c
endif

ifeq ($(CONFIG_SENSORS_HCSR04),y)
  CSRCS += stm32_hcsr04.c
endif

ifeq ($(CONFIG_SENSORS_MAX6675),y)
  CSRCS += stm32_max6675.c
endif

ifeq ($(CONFIG_LCD_MAX7219),y)
  CSRCS += stm32_max7219.c
endif

ifeq ($(CONFIG_INPUT_NUNCHUCK),y)
  CSRCS += stm32_nunchuck.c
endif

ifeq ($(CONFIG_LCD_SSD1306_I2C),y)
CSRCS += stm32_ssd1306.c
endif

ifeq ($(CONFIG_LCD_ST7567),y)
  CSRCS += stm32_lcd.c
endif

ifeq ($(CONFIG_LCD_PCD8544),y)
  CSRCS += stm32_pcd8544.c
endif

ifeq ($(CONFIG_SENSORS_APDS9960),y)
  CSRCS += stm32_apds9960.c
endif

ifeq ($(CONFIG_SENSORS_QENCODER),y)
  CSRCS += stm32_qencoder.c
endif

ifeq ($(CONFIG_SENSORS_VEML6070),y)
  CSRCS += stm32_veml6070.c
endif

ifeq ($(CONFIG_WL_NRF24L01),y)
  CSRCS += stm32_nrf24l01.c
endif

ifeq ($(CONFIG_USBDEV),y)
  CSRCS += stm32_usbdev.c
endif

ifeq ($(CONFIG_USBMSC),y)
CSRCS += stm32_usbmsc.c
endif

include $(TOPDIR)/boards/Board.mk
