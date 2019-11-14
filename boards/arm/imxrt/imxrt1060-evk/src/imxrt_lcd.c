/****************************************************************************
 * boards/arm/imxrt/imxrt1060-evk/src/imxrt_lcd.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <debug.h>

#include "imxrt_lcd.h"
#include "imxrt_gpio.h"

#include "imxrt1060-evk.h"

#include <arch/board/board.h>

#ifdef CONFIG_IMXRT_LCD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_lcd_bkl_pin_setup
 *
 * Description:
 *   Setup backlight pin (initially off)
 *
 ****************************************************************************/

void imxrt_lcd_pkl_pin_setup(void)
{
  imxrt_config_gpio(GPIO_LCD_BL);
}

/****************************************************************************
 * Name: imxrt_backlight
 *
 * Description:
 *   If CONFIG_IMXRT_LCD_BACKLIGHT is defined, then the board-specific
 *   logic must provide this interface to turn the backlight on and off.
 *
 ****************************************************************************/

#ifdef CONFIG_IMXRT_LCD_BACKLIGHT
void imxrt_backlight(bool blon)
{
  imxrt_gpio_write(GPIO_LCD_BL, blon); /* High illuminates */
}
#endif

/****************************************************************************
 * Name: imxrt_lcd_initialize
 *
 * Description:
 *   Initialization of the LCD blackligt and pin for the imxrt_bringup().
 *
 ****************************************************************************/

void imxrt_lcd_initialize(void)
{
  /* Setup the backlight pin */

  imxrt_lcd_pkl_pin_setup();

#ifdef CONFIG_IMXRT_LCD_BACKLIGHT
  /* Turn ON the backlight */

  imxrt_backlight(true);
#endif
}

#endif /* CONFIG_IMXRT_LCD */
