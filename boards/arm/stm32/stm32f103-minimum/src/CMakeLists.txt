# ##############################################################################
# boards/arm/stm32/stm32f103-minimum/src/CMakeLists.txt
#
# Licensed to the Apache Software Foundation (ASF) under one or more contributor
# license agreements.  See the NOTICE file distributed with this work for
# additional information regarding copyright ownership.  The ASF licenses this
# file to you under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License.  You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
# License for the specific language governing permissions and limitations under
# the License.
#
# ##############################################################################

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
  list(APPEND CSRCS stm32_adc.c)
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

# Configure linker script TODO: better to place this in boards/CMakeLists.txt?

set_property(GLOBAL PROPERTY LD_SCRIPT "${NUTTX_BOARD_DIR}/scripts/ld.script")
