/****************************************************************************
 * include/nuttx/lcd/st7567.h
 *
 * Driver for the TM12864J1CCWGWA Display with the ST7567 LCD
 * controller.
 *
 *   Copyright (C) 2013 Zilogic Systems. All rights reserved.
 *   Author: Manikandan <code@zilogic.com>
 *
 * Based on include/nuttx/lcd/ug-9664hswag01.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __INCLUDE_NUTTX_ST7567_H
#define __INCLUDE_NUTTX_ST7567_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ST7567 Configuration Settings:
 *
 * CONFIG_ST7567_SPIMODE - Controls the SPI mode
 * CONFIG_ST7567_FREQUENCY - Define to use a different bus frequency
 * CONFIG_ST7567_NINTERFACES - Specifies the number of physical
 *   ST7567 devices that will be supported.  NOTE:  At present, this
 *   must be undefined or defined to be 1.
 * CONFIG_ST7567_POWER
 *   If the hardware supports a controllable LCD a power supply, this
 *   configuration should be defined.  (See st7567_power() below).
 *
 * Required LCD driver settings:
 * CONFIG_LCD_ST7567 - Enable ST7567 support
 * CONFIG_LCD_MAXCONTRAST should be 255, but any value >0 and <=255 will be accepted.
 * CONFIG_LCD_MAXPOWER should be 2:  0=off, 1=dim, 2=normal
 *
 * Required SPI driver settings:
 * CONFIG_SPI_CMDDATA - Include support for cmd/data selection.
 */

/* Some important "colors" */

#define ST7567_Y1_BLACK  0
#define ST7567_Y1_WHITE  1

/* Only two power settings are supported: */

#define ST7567_POWER_OFF 0
#define ST7567_POWER_ON  1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name:  st7567_initialize
 *
 * Description:
 *   Initialize the ST7567 video hardware.  The initial state of the
 *   LCD is fully initialized, display memory cleared, and the LCD ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 throst7567h CONFIG_ST7567_NINTERFACES-1.
 *     This allows support for multiple LCD devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified LCD.  NULL is returned on any failure.
 *
 ****************************************************************************/

struct lcd_dev_s; /* see nuttx/lcd.h */
struct spi_dev_s; /* see nuttx/spi/spi.h */
FAR struct lcd_dev_s *st7567_initialize(FAR struct spi_dev_s *spi, unsigned int devno);

/****************************************************************************
 * Name:  st7567_power
 *
 * Description:
 *   If the hardware supports a controllable LCD a power supply, this
 *   interface should be provided.  It may be called by the driver to turn
 *   the LCD power on and off as needed.
 *
 * Input Parameters:
 *
 *   devno - A value in the range of 0 through CONFIG_ST7567_NINTERFACES-1.
 *     This allows support for multiple LCD devices.
 *   on - true:turn power on, false: turn power off.
 *
 * Returned Value:
 *   None
 *
 **************************************************************************************/

#ifdef CONFIG_ST7567_POWER
void st7567_power(unsigned int devno, bool on);
#else
#  define st7567_power(a,b)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_ST7567_H */
