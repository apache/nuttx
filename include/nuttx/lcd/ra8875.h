/**************************************************************************************
 * include/nuttx/lcd/ra8875.h
 * Definitions for the RAiO Technologies RA8875 LCD controller
 *
 *   Copyright (C) 2015 Intuitive Aerial AB. All rights reserved.
 *   Author: Marten Svanfeldt <marten@intuitiveaerial.com>
 *
 * References: RA8875, Rev 1.6, Apr 2013, RAiO Technologies Inc
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************************/

#ifndef __INCLUDE_NUTTX_LCD_RA8875_H
#define __INCLUDE_NUTTX_LCD_RA8875_H

/**************************************************************************************
 * Included Files
 **************************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_LCD_RA8875

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/
/* Configuration **********************************************************************/
/* CONFIG_LCD_RA8875 - Enables support for the RA8875-based LCD.
 * CONFIG_LCD_NOGETRUN
 *   NX components need to know if it can read from the LCD or not. If reading from the
 *   LCD is supported then some graphic operations can be improved. Default: Supported
 * CONFIG_LCD_LANDSCAPE - Define for 320x240 display "landscape" support. Default is
 *   this 320x240 "landscape" orientation.
 * CONFIG_LCD_RLANDSCAPE - Define for 320x240 display "reverse landscape" support.
 *   Default is this 320x240 "landscape" orientation
 * CONFIG_LCD_PORTRAIT - Define for 240x320 display "portrait" orientation support.
 *   Default is this 320x240 "landscape" orientation
 * CONFIG_LCD_RPORTRAIT - Define for 240x320 display "reverse portrait" orientation
 *   support.  Default is this 320x240 "landscape" orientation
 */

/**************************************************************************************
 * Public Types
 **************************************************************************************/

/* This structure defines the interface to the LCD provided by the platform.  The
 * nature of this interface is hidden from the RA8875 driver.
 */

struct ra8875_lcd_s
{
  void (*write_reg)(FAR struct ra8875_lcd_s *dev, uint8_t regnum, uint8_t data);
  void (*write_reg16)(FAR struct ra8875_lcd_s *dev, uint8_t regnum, uint16_t data);

  uint8_t (*read_reg)(FAR struct ra8875_lcd_s *dev, uint8_t regnum);
  uint8_t (*read_status)(FAR struct ra8875_lcd_s *dev);

  /* Interface for fast pixel access */

  void (*pwrite_prepare)(FAR struct ra8875_lcd_s *dev, uint8_t regnum);
  void (*pwrite_data8)(FAR struct ra8875_lcd_s *dev, uint8_t data);
  void (*pwrite_data16)(FAR struct ra8875_lcd_s *dev, uint16_t data);
  void (*pwrite_finish)(FAR struct ra8875_lcd_s *dev);

  void (*pread_prepare)(FAR struct ra8875_lcd_s *dev, uint8_t regnum);
  uint16_t (*pread_data16)(FAR struct ra8875_lcd_s *dev);
  void (*pread_finish)(FAR struct ra8875_lcd_s *dev);

  /* platform-specific data may follow */
};

/**************************************************************************************
 * Public Data
 **************************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/**************************************************************************************
 * Public Function Prototypes
 **************************************************************************************/

/**************************************************************************************
 * Name:  ra8875_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 **************************************************************************************/

FAR struct lcd_dev_s *ra8875_lcdinitialize(FAR struct ra8875_lcd_s *lcd);

/**************************************************************************************
 * Name:  ra8875_clear
 *
 * Description:
 *   This is a non-standard LCD interface just for the RA8875.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   display is cleared by simply setting all video memory to the specified color.
 *
 *   NOTE: This function is not available to applications in the protected or kernel
 *   build modes.
 *
 **************************************************************************************/

void ra8875_clear(FAR struct lcd_dev_s *dev, uint16_t color);

/**************************************************************************************
 * Name:  ra8875_drawrectangle
 *
 * Description:
 *   This is a non-standard function to draw a rectangle on the LCD.  This function is
 *   also used internally as part of the ra8875_clear implementation.
 *
 *   NOTE: This non-standard function is not available to applications in the
 *   protected or kernel build modes.
 *
 **************************************************************************************/

void ra8875_drawrectangle(FAR struct lcd_dev_s *dev, uint16_t x, uint16_t y,
                          uint16_t width, uint16_t height, uint16_t color, bool fill);

/**************************************************************************************
 * Name:  ra8875_drawline
 *
 * Description:
 *   This is a non-standard function to draw a line on the LCD.  This function is
 *   also used internally as part of the ra8875_rectandle implementation.
 *
 *   NOTE: This non-standard function is not available to applications in the
 *   protected or kernel build modes.
 *
 **************************************************************************************/

void ra8875_drawline(FAR struct lcd_dev_s *dev, uint16_t x1, uint16_t y1, uint16_t x2,
                     uint16_t y2, uint16_t color);

/**************************************************************************************
 * Name:  ra8875_drawtriangle
 *
 * Description:
 *   This is a non-standard function to draw a triangle on the LCD.  This function is
 *   also used internally as part of the ra8875_rectandle implementation.
 *
 *   NOTE: This non-standard function is not available to applications in the
 *   protected or kernel build modes.
 *
 **************************************************************************************/

#ifdef CONFIG_LCD_RA8875_EXTENDED
void ra8875_drawtriangle(FAR struct lcd_dev_s *dev, uint16_t x0, uint16_t y0,
                         uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
                         uint16_t color, bool fill);
#endif

/**************************************************************************************
 * Name:  ra8875_drawcircle
 *
 * Description:
 *   This is a non-standard function to draw a circle on the LCD.  This function is
 *   also used internally as part of the ra8875_rectandle implementation.
 *
 *   NOTE: This non-standard function is not available to applications in the
 *   protected or kernel build modes.
 *
 **************************************************************************************/

#ifdef CONFIG_LCD_RA8875_EXTENDED
void ra8875_drawcircle(FAR struct lcd_dev_s *dev, uint16_t x, uint16_t y,
                       uint8_t radius, uint16_t color, bool fill);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LCD_SRA8875 */
#endif /* __INCLUDE_NUTTX_LCD_SRA8875_H */
