/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_ssd1306.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>

#include "stm32.h"
#include "stm32f103_minimum.h"

#include "stm32_ssd1306.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OLED_I2C_PORT         1 /* OLED display connected to I2C1 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  int ret;

  ret = board_ssd1306_initialize(OLED_I2C_PORT);
  if (ret < 0)
    {
      lcderr("ERROR: Failed to initialize SSD1306\n");
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int devno)
{
  return board_ssd1306_getdev();
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  /* TO-FIX */
}
