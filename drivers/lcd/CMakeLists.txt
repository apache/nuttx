# ##############################################################################
# drivers/lcd/CMakeLists.txt
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

set(SRCS)

if(CONFIG_LCD)

  # Support for the generic LCD framebuffer front-end

  if(CONFIG_LCD_FRAMEBUFFER)
    list(APPEND SRCS lcd_framebuffer.c)
  endif()

  if(CONFIG_LCD_DEV)
    list(APPEND SRCS lcd_dev.c)
  endif()

  # Include support for Graphics LCD drivers

  if(CONFIG_LCD_FT80X)
    list(APPEND SRCS ft80x.c)
    if(CONFIG_LCD_FT80X_SPI)
      list(APPEND SRCS ft80x_spi.c)
    elseif(CONFIG_LCD_FT80X_I2C)
      list(APPEND SRCS ft80x_i2c.c)
    endif()
  endif()

  if(CONFIG_LCD_LPM013M091A)
    list(APPEND SRCS lpm013m091a.c)
  endif()

  if(CONFIG_LCD_APA102)
    list(APPEND SRCS apa102.c)
  endif()

  if(CONFIG_LCD_P14201)
    list(APPEND SRCS p14201.c)
  endif()

  if(CONFIG_LCD_UG2864AMBAG01)
    list(APPEND SRCS ug-2864ambag01.c)
  endif()

  if(CONFIG_LCD_UG9664HSWAG01)
    list(APPEND SRCS ug-9664hswag01.c)
  endif()

  if(CONFIG_LCD_SSD1306)
    list(APPEND SRCS ssd1306_base.c)
  endif()

  if(CONFIG_LCD_SSD1306_SPI)
    list(APPEND SRCS ssd1306_spi.c)
  endif()

  if(CONFIG_LCD_SSD1306_I2C)
    list(APPEND SRCS ssd1306_i2c.c)
  endif()

  if(CONFIG_LCD_SSD1289)
    list(APPEND SRCS ssd1289.c)
  endif()

  if(CONFIG_LCD_SSD1680)
    list(APPEND SRCS ssd1680.c)
  endif()

  if(CONFIG_LCD_SSD1351)
    list(APPEND SRCS ssd1351.c)
  endif()

  if(CONFIG_LCD_MIO283QT2)
    list(APPEND SRCS mio283qt2.c)
  endif()

  if(CONFIG_LCD_MAX7219)
    list(APPEND SRCS max7219.c)
  endif()

  if(CONFIG_LCD_MIO283QT9A)
    list(APPEND SRCS mio283qt9a.c)
  endif()

  if(CONFIG_LCD_PCD8544)
    list(APPEND SRCS pcd8544.c)
  endif()

  if(CONFIG_LCD_ST7565)
    list(APPEND SRCS st7565.c)
  endif()

  if(CONFIG_LCD_ST7567)
    list(APPEND SRCS st7567.c)
  endif()

  if(CONFIG_LCD_SHARP_MEMLCD)
    list(APPEND SRCS memlcd.c)
  endif()

  if(CONFIG_LCD_ILI9225)
    list(APPEND SRCS ili9225.c)
  endif()

  if(CONFIG_LCD_ILI9340)
    list(APPEND SRCS ili9340.c)
  endif()

  if(CONFIG_LCD_ILI9341)
    list(APPEND SRCS ili9341.c)
  endif()

  if(CONFIG_LCD_LCDDRV_SPIIF)
    list(APPEND SRCS lcddrv_spiif.c)
  endif()

  if(CONFIG_LCD_RA8875)
    list(APPEND SRCS ra8875.c)
  endif()

  if(CONFIG_LCD_ST7735)
    list(APPEND SRCS st7735.c)
  endif()

  if(CONFIG_LCD_ST7789)
    list(APPEND SRCS st7789.c)
  endif()

  if(CONFIG_LCD_GC9A01)
    list(APPEND SRCS gc9a01.c)
  endif()

  if(CONFIG_LCD_JD9851)
    list(APPEND SRCS jd9851.c)
  endif()
endif() # CONFIG_LCD

if(CONFIG_SLCD)

  # Include support for Alphanumeric/Segment LCD drivers

  if(CONFIG_LCD_BACKPACK)
    list(APPEND SRCS pcf8574_lcd_backpack.c)
  endif()

  if(CONFIG_LCD_ST7032)
    list(APPEND SRCS st7032.c)
  endif()

  if(CONFIG_LCD_HT16K33)
    list(APPEND SRCS ht16k33_14seg.c)
  endif()
endif() # CONFIG_SLCD

# Other LCD-related devices

if(CONFIG_LCD_TDA19988)
  list(APPEND SRCS tda19988.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
