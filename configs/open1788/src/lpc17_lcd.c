/************************************************************************************
 * configs/open1788/src/lpc17_lcd.c
 * arch/arm/src/board/lpc17_lcd.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <debug.h>

#include "lpc17_lcd.h"
#include "lpc17_gpio.h"

#include "open1788.h"

#ifdef CONFIG_LPC17_LCD

/************************************************************************************
 * Definitions
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: open1788_lcd_initialize
 *
 * Description:
 *   Initialize the LCD.  Setup backlight (initially off)
 *
 ************************************************************************************/

void open1788_lcd_initialize(void)
{
  /* Configure the LCD backlight (and turn the backlight off) */

  lpc17_configgpio(GPIO_LCD_BL);
}

/************************************************************************************
 * Name: lpc17_backlight
 *
 * Description:
 *   If CONFIG_LPC17_LCD_BACKLIGHT is defined, then the board-specific logic must
 *   provide this interface to turn the backlight on and off.
 *
 ************************************************************************************/

#ifdef CONFIG_LPC17_LCD_BACKLIGHT
void lpc17_backlight(bool blon)
{
  lpc17_gpiowrite(GPIO_LCD_BL, blon);
}
#endif

#endif /* CONFIG_LPC17_LCD */
