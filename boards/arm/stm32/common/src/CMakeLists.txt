#############################################################################
# boards/arm/stm32/common/src/Make.defs
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
#############################################################################

set(SRCS)

if (CONFIG_SENSORS_BMP180)
  list(APPEND SRCS stm32_bmp180.c)
endif()

if (CONFIG_LEDS_APA102)
  list(APPEND SRCS stm32_apa102.c)
endif()

if (CONFIG_WS2812)
  list(APPEND SRCS stm32_ws2812.c)
endif()

if (CONFIG_SENSORS_MAX6675)
  list(APPEND SRCS stm32_max6675.c)
endif()

if (CONFIG_SENSORS_VEML6070)
  list(APPEND SRCS stm32_veml6070.c)
endif()

if (CONFIG_INPUT_NUNCHUCK)
list(APPEND SRCS stm32_nunchuck.c)
endif()

if (CONFIG_AUDIO_TONE)
  list(APPEND SRCS stm32_tone.c)
endif()

if (CONFIG_LCD_BACKPACK)
  list(APPEND SRCS stm32_lcd_backpack.c)
endif()

if (CONFIG_LCD_SSD1306)
  list(APPEND SRCS stm32_ssd1306.c)
endif()

if (CONFIG_SENSORS_LM75)
  list(APPEND SRCS stm32_lm75.c)
endif()

if (CONFIG_WL_NRF24L01)
  list(APPEND SRCS stm32_nrf24l01.c)
endif()

if (CONFIG_SENSORS_HCSR04)
  list(APPEND SRCS stm32_hcsr04.c)
endif()

if (CONFIG_SENSORS_APDS9960)
  list(APPEND SRCS stm32_apds9960.c)
endif()

if (CONFIG_SENSORS_ZEROCROSS)
  list(APPEND SRCS stm32_zerocross.c)
endif()

if (CONFIG_SENSORS_QENCODER)
  list(APPEND SRCS stm32_qencoder.c)
endif()

if (CONFIG_SENSORS_INA219)
  list(APPEND SRCS stm32_ina219.c)
endif()

if (CONFIG_SENSORS_L3GD20)
  list(APPEND SRCS stm32_l3gd20.c)
endif()

if (CONFIG_SENSORS_MPL115A)
  list(APPEND SRCS stm32_mpl115a.c)
endif()

if (CONFIG_SENSORS_DHTXX)
  list(APPEND SRCS stm32_dhtxx.c)
endif()

if (CONFIG_SENSORS_XEN1210)
  list(APPEND SRCS stm32_xen1210.c)
endif()

if (CONFIG_SENSORS_BH1750FVI)
  list(APPEND SRCS stm32_bh1750.c)
endif()

if (CONFIG_SENSORS_MLX90614)
  list(APPEND SRCS stm32_mlx90614.c)
endif()

if (CONFIG_SENSORS_MAX31855)
  list(APPEND SRCS stm32_max31855.c)
endif()

if (CONFIG_LIS3DSH)
  list(APPEND SRCS stm32_lis3dsh.c)
endif()

if (CONFIG_BOARD_STM32_IHM07M1)
  list(APPEND SRCS stm32_ihm07m1.c)
endif()

if (CONFIG_BOARD_STM32_IHM08M1)
  list(APPEND SRCS stm32_ihm08m1.c)
endif()

target_sources(board PRIVATE ${SRCS})
