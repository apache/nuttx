/****************************************************************************
 * boards/arm/imxrt/imxrt1064-evk/src/imxrt_lcd.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
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

#include "imxrt1064-evk.h"

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
