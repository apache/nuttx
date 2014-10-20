/************************************************************************************
 * configs/stm32f429i-disco/src/stm32_lcd.c
 *
 *   Copyright (C) 2014 Marco Krahl. All rights reserved.
 *   Author: Marco Krahl <ocram.lhark@gmail.com>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32f429i-disco.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/
#ifdef CONFIG_STM32F429I_DISCO_ILI9341_LCDDEVICE
# define ILI9341_LCD_DEVICE CONFIG_STM32F429I_DISCO_ILI9341_LCDDEVICE
#else
# define ILI9341_LCD_DEVICE  0
#endif
/******************************************************************************
 * Private Data
 ******************************************************************************/

FAR struct lcd_dev_s *g_lcd = NULL;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifdef CONFIG_STM32F429I_DISCO_ILI9341_LCDIFACE
/****************************************************************************
 * Name: up_lcduninitialize
 *
 * Description:
 *   Unitialize the LCD Device.
 *
 * Parameter:
 *
 * Return:
 *
 ***************************************************************************/

void up_lcduninitialize(void)
{
  /* Set display off */

  g_lcd->setpower(g_lcd, 0);

  g_lcd = NULL;
}


/****************************************************************************
 * Name: up_lcdgetdev
 *
 * Description:
 *   Return a reference to the LCD object for the specified LCD Device.
 *   This allows support for multiple LCD devices.
 *
 * Parameter:
 *   lcddev - Number of the LDC Device.
 *
 * Return:
 *   Reference to the LCD object if exist otherwise NULL
 *
 ***************************************************************************/

FAR struct lcd_dev_s *up_lcdgetdev(int lcddev)
{
  if (lcddev == ILI9341_LCD_DEVICE)
    {
      return g_lcd;
    }

  return NULL;
}


/****************************************************************************
 * Name: up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware. The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use, but
 *   with the power setting at 0 (full off).
 *
 * Parameter:
 *
 * Return:
 *   On success - Ok
 *   On error   - Error Code
 *
 ****************************************************************************/

int up_lcdinitialize(void)
{
  /* check if always initialized */

  if (!g_lcd)
    {
      /* Initialize the sub driver structure */

      FAR struct ili9341_lcd_s *dev = stm32_ili93414ws_initialize();

      /* Initialize public lcd driver structure */

      if (dev)
        {
          /*
           * Get a reference to valid lcd driver structure to avoid repeated
           * initialization of the LCD Device. Also enables uninitializing of
           * the LCD Device.
           */

          g_lcd = ili9341_initialize(dev, ILI9341_LCD_DEVICE);

          if (g_lcd)
            {
              return OK;
            }
        }
    }

  return -errno;
}

#endif
