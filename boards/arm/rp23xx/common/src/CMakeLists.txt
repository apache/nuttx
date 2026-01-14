# ##############################################################################
# boards/arm/rp23xx/common/src/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
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

if(CONFIG_ARCH_BOARD_COMMON)
  set(RP23XXFLAGS -DPICO_RP2040=0 -DPICO_RP2350=1 -DLIB_PICO_BINARY_INFO=0
                  -DPICO_SECURE=1 -D__ARM_ARCH_6M__=0)

  list(APPEND SRCS rp23xx_common_bringup.c rp23xx_common_initialize.c)

  if(CONFIG_BOARDCTL_RESET)
    list(APPEND SRCS rp23xx_reset.c)
  endif()

  if(CONFIG_SPI)
    list(APPEND SRCS rp23xx_spi.c)
  endif()

  if(CONFIG_RP23XX_I2C_DRIVER)
    list(APPEND SRCS rp23xx_i2cdev.c)
  endif()

  if(CONFIG_RP23XX_PWM)
    list(APPEND SRCS rp23xx_pwmdev.c)
  endif()

  if(CONFIG_RP23XX_SPI_DRIVER)
    list(APPEND SRCS rp23xx_spidev.c)
  endif()

  if(CONFIG_RP23XX_I2S)
    list(APPEND SRCS rp23xx_i2sdev.c)
  endif()

  if(CONFIG_LCD_SSD1306)
    list(APPEND SRCS rp23xx_ssd1306.c)
  endif()

  if(CONFIG_LCD_ST7789)
    list(APPEND SRCS rp23xx_st7789.c)
  endif()

  if(CONFIG_LCD_ST7735)
    list(APPEND SRCS rp23xx_st7735.c)
  endif()

  if(CONFIG_LCD_GC9A01)
    list(APPEND SRCS rp23xx_gc9a01.c)
  endif()

  if(CONFIG_USBMSC)
    list(APPEND SRCS rp23xx_usbmsc.c)
  endif()

  if(CONFIG_USBDEV_COMPOSITE)
    list(APPEND SRCS rp23xx_composite.c)
  endif()

  if(CONFIG_RP23XX_SPISD)
    list(APPEND SRCS rp23xx_spisd.c)
  endif()

  if(CONFIG_SENSORS_BMP180)
    list(APPEND SRCS rp23xx_bmp180.c)
  endif()

  if(CONFIG_SENSORS_BMP280)
    list(APPEND SRCS rp23xx_bmp280.c)
  endif()

  if(CONFIG_SENSORS_INA219)
    list(APPEND SRCS rp23xx_ina219.c)
  endif()

  if(CONFIG_ENC28J60)
    list(APPEND SRCS rp23xx_enc28j60.c)
  endif()

  if(CONFIG_LCD_BACKPACK)
    list(APPEND SRCS rp23xx_lcd_backpack.c)
  endif()

  if(CONFIG_BOARDCTL_UNIQUEID)
    list(APPEND SRCS rp23xx_uniqueid.c)
  endif()

  if(CONFIG_NET_W5500)
    list(APPEND SRCS rp23xx_w5500.c)
  endif()

  if(CONFIG_SENSORS_MAX6675)
    list(APPEND SRCS rp23xx_max6675.c)
  endif()

endif()
target_sources(board PRIVATE ${SRCS})
target_compile_options(board PRIVATE ${RP23XXFLAGS})
