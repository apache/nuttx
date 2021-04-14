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

set(CSRCS)

if("${CONFIG_LCD}" STREQUAL "y")

	# Support for the generic LCD framebufer front-end
	
	if("${CONFIG_LCD_FRAMEBUFFER}" STREQUAL "y")
		list(APPEND CSRCS lcd_framebuffer.c)
	endif()

	# Include support for Graphics LCD drivers
	
	if("${CONFIG_LCD_FT80X}" STREQUAL "y")
		list(APPEND CSRCS ft80x.c)
		if("${CONFIG_LCD_FT80X_SPI}" STREQUAL "y")
			list(APPEND CSRCS ft80x_spi.c)
		elseif("${CONFIG_LCD_FT80X_I2C}" STREQUAL "y")
			list(APPEND CSRCS ft80x_i2c.c)
		endif()
	endif()

	if("${CONFIG_LCD_P14201}" STREQUAL "y")
		list(APPEND CSRCS p14201.c)
	endif()
	
	if("${CONFIG_LCD_UG2864AMBAG01}" STREQUAL "y")
		list(APPEND CSRCS ug-2864ambag01.c)
	endif()
	
	if("${CONFIG_LCD_UG9664HSWAG01}" STREQUAL "y")
		list(APPEND CSRCS ug-9664hswag01.c)
	endif()
	
	if("${CONFIG_LCD_SSD1306}" STREQUAL "y")
		list(APPEND CSRCS ssd1306_base.c)
	endif()
	
	if("${CONFIG_LCD_SSD1306_SPI}" STREQUAL "y")
		list(APPEND CSRCS ssd1306_spi.c)
	endif()
	
	if("${CONFIG_LCD_SSD1306_I2C}" STREQUAL "y")
		list(APPEND CSRCS ssd1306_i2c.c)
	endif()
	
	if("${CONFIG_LCD_SSD1289}" STREQUAL "y")
		list(APPEND CSRCS ssd1289.c)
	endif()
	
	if("${CONFIG_LCD_SSD1351}" STREQUAL "y")
		list(APPEND CSRCS ssd1351.c)
	endif()
	
	if("${CONFIG_LCD_MIO283QT2}" STREQUAL "y")
		list(APPEND CSRCS mio283qt2.c)
	endif()
	
	if("${CONFIG_LCD_MAX7219}" STREQUAL "y")
		list(APPEND CSRCS max7219.c)
	endif()
	
	if("${CONFIG_LCD_MIO283QT9A}" STREQUAL "y")
		list(APPEND CSRCS mio283qt9a.c)
	endif()
	
	if("${CONFIG_LCD_PCD8544}" STREQUAL "y")
		list(APPEND CSRCS pcd8544.c)
	endif()
	
	if("${CONFIG_LCD_ST7565}" STREQUAL "y")
		list(APPEND CSRCS st7565.c)
	endif()
	
	if("${CONFIG_LCD_ST7567}" STREQUAL "y")
		list(APPEND CSRCS st7567.c)
	endif()
	
	if("${CONFIG_LCD_SHARP_MEMLCD}" STREQUAL "y")
		list(APPEND CSRCS memlcd.c)
	endif()
	
	if("${CONFIG_LCD_ILI9341}" STREQUAL "y")
		list(APPEND CSRCS ili9341.c)
	endif()
	
	if("${CONFIG_LCD_RA8875}" STREQUAL "y")
		list(APPEND CSRCS ra8875.c)
	endif()
endif() # CONFIG_LCD

if("${CONFIG_SLCD}" STREQUAL "y")

	# Include support for Alphanumeric/Segment LCD drivers
	
	if("${CONFIG_LCD_BACKPACK}" STREQUAL "y")
		list(APPEND CSRCS pcf8574_lcd_backpack.c)
	endif()

	if("${CONFIG_LCD_ST7032}" STREQUAL "y")
		list(APPEND CSRCS st7032.c)
	endif()
endif() # CONFIG_SLCD

# Include LCD driver build support (the nested if-then-else() implements an OR)

if("${CONFIG_LCD}" STREQUAL "y")
	set_parent_srcs(${CSRCS})
elseif("${CONFIG_SLCD}" STREQUAL "y")
	set_parent_srcs(${CSRCS})
endif()
