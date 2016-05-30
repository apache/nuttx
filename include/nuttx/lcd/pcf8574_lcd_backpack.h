/****************************************************************************
 * include/nuttx/lcd/pcf8574_lcd_backpack.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: dev@ziggurat29.com
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_LCD_PCF8574_LCD_BACKPACK_H
#define __INCLUDE_NUTTX_LCD_PCF8574_LCD_BACKPACK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <nuttx/i2c/i2c_master.h>
#include <sys/ioctl.h>
#include <nuttx/lcd/slcd_ioctl.h>
#include <nuttx/lcd/slcd_codec.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configurations of some well-known boards.  You may still have to modify the
 * address if was changed from the default.  You will also need to specify the
 * geometry of your attached LCD display.  You can support:
 * 1x8, 1x12, 1x16, 2x8, 2x12, 2x16, 2x20, 2x24, 2x40, 4x16, 4x20
 * Pretty much anything on the market except 4x40, which really consists of two
 * separate 2x40 controllers, and the I2C backpack doesn't support those due
 * to the second 'E' line being needed.
 * Additionally, you still need to set the (row,col) geometry explicitly, since
 * there is not a means of determining this dynamically.
 * Consider these 'informative'.
 * XXX Note, actual testing has been done on LCD_I2C_BACKPACK_CFG_MJKDZ
 * and LCD_I2C_BACKPACK_CFG_SAINSMART only, the others come from online
 * research.
 */

/* board marked 'mjkdz' and Arduino-IIC-LCD GY-LCD-V1 */

#define LCD_I2C_BACKPACK_CFG_MJKDZ {0x20,4,5,6,0,1,2,3,7,false,0,0}

/* YwRobot/DFRobot/SainSmart */

#define LCD_I2C_BACKPACK_CFG_SAINSMART {0x27,2,1,0,4,5,6,7,3,true,0,0}

/* Robot Arduino LCM1602/2004 */

#define LCD_I2C_BACKPACK_CFG_ROBOT {0x27,2,1,0,4,5,6,7,3,false,0,0}

/* I2CLCDextraIO board modded for backlight (pnp transistor) */

#define LCD_I2C_BACKPACK_CFG_I2CIO_PNP {0x38,6,5,4,0,1,2,3,7,false,0,0}

/* I2CLCDextraIO board modded for backlight (npn transistor) */

#define LCD_I2C_BACKPACK_CFG_I2CIO_NPN {0x38,6,5,4,0,1,2,3,7,true,0,0}

/* SLCDIOC_CREATECHAR: Create a custom character pattern
 *
 * argument:  pointer to slcd_createchar_s structure (defined below)
 */

#define SLCDIOC_CREATECHAR _SLCDIOC(0x80)

 /****************************************************************************
 * Public Types
 ****************************************************************************/

/* Used to specify the pin wiring for this particular module */

struct pcf8574_lcd_backpack_config_s
{
  uint8_t   addr; /* I2C address; 'unshifted' (i.e. disregarding the LSB R/W bit)
                   * these can vary widely depending on board pullups, whether it
                   * uses a PCF8574-T or -AT, etc.  Many default to either 0x20
                   * or 0x27, and some default to 0x38 or 0x3f.  Check with seller.
                   */
  uint8_t   en;   /* gpio bit for LCD EN */
  uint8_t   rw;   /* gpio bit for LCD RW */
  uint8_t   rs;   /* gpio bit for LCD RS */
  uint8_t   d4;   /* gpio bit for LCD D4 */
  uint8_t   d5;   /* gpio bit for LCD D5 */
  uint8_t   d6;   /* gpio bit for LCD D6 */
  uint8_t   d7;   /* gpio bit for LCD D7 */
  uint8_t   bl;   /* gpio bit for backlight control */
  bool      bl_active_high;  /* is the backlight control active high? */
  uint8_t   rows; /* screen geometry, rows, 1, 2 or 4 */
  uint8_t   cols; /* screen geometry, cols, 8, 12, 16, 20, 24, 40 */
};

/* Used with the SLCDIOC_CREATECHAR ioctl call */

struct slcd_createchar_s
{
  uint8_t idx;          /* Custom character index; 0-7.  Note; you'll probably
                         * want to avoid code point 0 unless you really need it,
                         * because embedded nul in a C string can cause surprises.
                         */
  uint8_t bmp[8];       /* Custom character bitmap.  The bitmap is structured as
                         * a 5x8 bitmap.  '1' = pixel on, msb-lsb left-to-right,
                         * and right-justified (i.e. only bits 4-0 are used).
                         * Each byte represents 1 row.  By convention, you are
                         * expected to leave the last row all 0's, because it is
                         * used by the cursor, but this is not strictly required.
                         */
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pcf8574_lcd_backpack_register
 *
 * Description:
 *  Register a character driver that is a I2C LCD 'backpack' based on the
 *  PCF8574 I2C IO expander.  It allows operation of the ever-popular HD44780
 *  based LCDs via I2C instead of parallel (saving a bunch of gpio lines).
 *
 *  There are a multitude of these available from various sources (e.g. ebay).
 *  They typically vary by gpio-to-lcd pin mapping, and I2C addresss, but
 *  otherwise are functionally identical.
 *
 *  The characters presented for codes depend on the masked rom of the
 *  particular LCD device attached, but the most common are:
 *    'A00', which present 0x20-0x7f, and 0xa0-0xff', similar to JIS X 0201
 *    'A02', which present 0x10-0xff, and include various european symbols
 *  In both cases, codes 0x00-0x07 map to the CGRAM characters, which can be
 *  loaded via ioctl SLCDIOC_CREATECHAR (q.v.).
 *
 *  This driver supports the SLCD codec for various escape sequences; q.v.
 *  nuttx/lcd/slcd_codec.h for details.  This driver supports the SLCD ioctl
 *  interface for various extended commands; q.v. nuttx/lcd/slcd_ioctl.h for
 *  details.  This driver supports an additional ioctl for defining custom
 *  characters; see above for details.
 *
 * Parameters:
 *  devpath - path to device node; arbitrary, but typically '/dev/lcd0' or such
 *  i2c - the low-level i2c bus onto which to bind
 *  cfg - the board-specific configuration
 *
 ****************************************************************************/

int pcf8574_lcd_backpack_register(FAR const char *devpath,
                                  FAR struct i2c_master_s *i2c,
                                  FAR struct pcf8574_lcd_backpack_config_s *cfg);

#endif /* __INCLUDE_NUTTX_LCD_PCF8574_LCD_BACKPACK_H */
