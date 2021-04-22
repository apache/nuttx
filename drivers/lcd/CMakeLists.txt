############################################################################
# drivers/lcd/CMakeLists.txt
#
#   Copyright (C) 2010-2012, 2017-2018 Gregory Nutt. All rights reserved.
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

set(SRCS)

if (CONFIG_LCD)
  # Support for the generic LCD framebuffer front-end

  if (CONFIG_LCD_FRAMEBUFFER)
    list(APPEND SRCS lcd_framebuffer.c)
  endif()

  if (CONFIG_LCD_DEV)
    list(APPEND SRCS lcd_dev.c)
  endif()

  # Include support for Graphics LCD drivers

  if (CONFIG_LCD_FT80X)
    list(APPEND SRCS ft80x.c)

    if (CONFIG_LCD_FT80X_SPI)
      list(APPEND SRCS ft80x_spi.c)
    elseif (CONFIG_LCD_FT80X_I2C)
      list(APPEND SRCS ft80x_i2c.c)
    endif()
  endif()

  if (CONFIG_LCD_LPM013M091A)
    list(APPEND SRCS lpm013m091a.c)
  endif()

  if (CONFIG_LCD_P14201)
    list(APPEND SRCS p14201.c)
  endif()

  if (CONFIG_LCD_UG2864AMBAG01)
    list(APPEND SRCS ug-2864ambag01.c)
  endif()

  if (CONFIG_LCD_UG9664HSWAG01)
    list(APPEND SRCS ug-9664hswag01.c)
  endif()

  if (CONFIG_LCD_SSD1306)
    list(APPEND SRCS ssd1306_base.c)
  endif()

  if (CONFIG_LCD_SSD1306_SPI)
    list(APPEND SRCS ssd1306_spi.c)
  endif()

  if (CONFIG_LCD_SSD1306_I2C)
    list(APPEND SRCS ssd1306_i2c.c)
  endif()

  if (CONFIG_LCD_SSD1289)
    list(APPEND SRCS ssd1289.c)
  endif()

  if (CONFIG_LCD_SSD1351)
    list(APPEND SRCS ssd1351.c)
  endif()

  if (CONFIG_LCD_MIO283QT2)
    list(APPEND SRCS mio283qt2.c)
  endif()

  if (CONFIG_LCD_MAX7219)
    list(APPEND SRCS max7219.c)
  endif()

  if (CONFIG_LCD_MIO283QT9A)
    list(APPEND SRCS mio283qt9a.c)
  endif()

  if (CONFIG_LCD_PCD8544)
    list(APPEND SRCS pcd8544.c)
  endif()

  if (CONFIG_LCD_ST7565)
    list(APPEND SRCS st7565.c)
  endif()

  if (CONFIG_LCD_ST7567)
    list(APPEND SRCS st7567.c)
  endif()

  if (CONFIG_LCD_SHARP_MEMLCD)
    list(APPEND SRCS memlcd.c)
  endif()

  if (CONFIG_LCD_ILI9225)
    list(APPEND SRCS ili9225.c)
  endif()

  if (CONFIG_LCD_ILI9340)
    list(APPEND SRCS ili9340.c)
  endif()

  if (CONFIG_LCD_ILI9341)
    list(APPEND SRCS ili9341.c)
  endif()

  if (CONFIG_LCD_LCDDRV_SPIIF)
    list(APPEND SRCS lcddrv_spiif.c)
  endif()

  if (CONFIG_LCD_RA8875)
    list(APPEND SRCS ra8875.c)
  endif()

  if (CONFIG_LCD_ST7735)
    list(APPEND SRCS st7735.c)
  endif()

  if (CONFIG_LCD_ST7789)
    list(APPEND SRCS st7789.c)
  endif()
endif()

if (CONFIG_SLCD)
  # Include support for Alphanumeric/Segment LCD drivers

  if (CONFIG_LCD_BACKPACK)
    list(APPEND SRCS pcf8574_lcd_backpack.c)
  endif()

  if (CONFIG_LCD_ST7032)
    list(APPEND SRCS st7032.c)
  endif()
endif()

# Other LCD-related devices

if (CONFIG_LCD_TDA19988)
  list(APPEND SRCS tda19988.c)
endif()

target_sources(drivers PRIVATE ${SRCS})
