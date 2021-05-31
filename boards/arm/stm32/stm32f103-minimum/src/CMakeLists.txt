############################################################################
# configs/stm32f103-minimum/src/Makefile
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

set(CSRCS)

list(APPEND CSRCS stm32_boot.c stm32_bringup.c stm32_spi.c)

if("${CONFIG_LIB_BOARDCTL}" STREQUAL "y")
	list(APPEND CSRCS stm32_appinit.c)
endif()

if("${CONFIG_ARCH_BUTTONS}" STREQUAL "y")
	list(APPEND CSRCS stm32_buttons.c)
endif()

if("${CONFIG_ARCH_LEDS}" STREQUAL "y")
	list(APPEND CSRCS stm32_autoleds.c)
else()
	list(APPEND CSRCS stm32_userleds.c)
endif()

if("${CONFIG_ADC}" STREQUAL "y")
	list(APPEND CSRCS  stm32_adc.c)
endif()

if("${CONFIG_DEV_GPIO}" STREQUAL "y")
	list(APPEND CSRCS stm32_gpio.c)
endif()

if("${CONFIG_PWM}" STREQUAL "y")
	list(APPEND CSRCS stm32_pwm.c)
endif()

if("${CONFIG_SENSORS_ZEROCROSS}" STREQUAL "y")
	list(APPEND CSRCS stm32_zerocross.c)
endif()

if("${CONFIG_LEDS_APA102}" STREQUAL "y")
	list(APPEND CSRCS stm32_apa102.c)
endif()

if("${CONFIG_SENSORS_BMP180}" STREQUAL "y")
	list(APPEND CSRCS stm32_bmp180.c)
endif()

if("${CONFIG_LM75_I2C}" STREQUAL "y")
	list(APPEND CSRCS stm32_lm75.c)
endif()

if("${CONFIG_RGBLED}" STREQUAL "y")
	list(APPEND CSRCS stm32_rgbled.c)
endif()

if("${CONFIG_MMCSD}" STREQUAL "y")
	list(APPEND CSRCS stm32_mmcsd.c)
endif()

if("${CONFIG_MTD_W25}" STREQUAL "y")
	list(APPEND CSRCS stm32_w25.c)
endif()

if("${CONFIG_MTD_AT24XX}" STREQUAL "y")
if("${CONFIG_STM32_I2C1}" STREQUAL "y")
	list(APPEND CSRCS stm32_at24.c)
endif()
endif()

if("${CONFIG_AUDIO_TONE}" STREQUAL "y")
	list(APPEND CSRCS stm32_tone.c)
endif()

if("${CONFIG_CAN_MCP2515}" STREQUAL "y")
	list(APPEND CSRCS stm32_mcp2515.c)
endif()

if("${CONFIG_CL_MFRC522}" STREQUAL "y")
	list(APPEND CSRCS stm32_mfrc522.c)
endif()

if("${CONFIG_SENSORS_HCSR04}" STREQUAL "y")
	list(APPEND CSRCS stm32_hcsr04.c)
endif()

if("${CONFIG_SENSORS_MAX6675}" STREQUAL "y")
	list(APPEND CSRCS stm32_max6675.c)
endif()

if("${CONFIG_LCD_MAX7219}" STREQUAL "y")
	list(APPEND CSRCS stm32_max7219.c)
endif()

if("${CONFIG_INPUT_NUNCHUCK}" STREQUAL "y")
	list(APPEND CSRCS stm32_nunchuck.c)
endif()

if("${CONFIG_LCD_SSD1306_I2C}" STREQUAL "y")
	list(APPEND CSRCS stm32_ssd1306.c)
endif()

if("${CONFIG_LCD_ST7567}" STREQUAL "y")
	list(APPEND CSRCS stm32_lcd.c)
endif()

if("${CONFIG_LCD_PCD8544}" STREQUAL "y")
	list(APPEND CSRCS stm32_pcd8544.c)
endif()

if("${CONFIG_SENSORS_APDS9960}" STREQUAL "y")
	list(APPEND CSRCS stm32_apds9960.c)
endif()

if("${CONFIG_SENSORS_QENCODER}" STREQUAL "y")
	list(APPEND CSRCS stm32_qencoder.c)
endif()

if("${CONFIG_SENSORS_VEML6070}" STREQUAL "y")
	list(APPEND CSRCS stm32_veml6070.c)
endif()

if("${CONFIG_WL_NRF24L01}" STREQUAL "y")
	list(APPEND CSRCS stm32_nrf24l01.c)
endif()

if("${CONFIG_USBDEV}" STREQUAL "y")
	list(APPEND CSRCS stm32_usbdev.c)
endif()

if("${CONFIG_USBMSC}" STREQUAL "y")
	list(APPEND CSRCS stm32_usbmsc.c)
endif()

target_sources(board PRIVATE ${CSRCS})
 	
# Configure linker script
# TODO: better to place this in boards/CMakeLists.txt?

set_property(GLOBAL PROPERTY LD_SCRIPT "${BOARD_PATH}/scripts/ld.script")
