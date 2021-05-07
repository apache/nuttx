/****************************************************************************
 * boards/arm/lpc54xx/lpcxpresso-lpc54628/src/lpc54_lcd.c
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

#include "lpc54_lcd.h"
#include "lpc54_gpio.h"

#include "lpcxpresso-lpc54628.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_lcd_initialize
 *
 * Description:
 *   Initialize the LCD.  Setup backlight (initially off)
 *
 ****************************************************************************/

void lpc54_lcd_initialize(void)
{
  /* Configure the LCD backlight (and turn the backlight off) */

  lpc54_gpio_config(GPIO_LCD_BL);

  /* Initiale touchscreen controller nRST GPIOs here
   * (putting it into reset)
   */

  lpc54_gpio_config(GPIO_FT5X06_CTRSTN);
}

/****************************************************************************
 * Name: lpc54_backlight
 *
 * Description:
 *   If CONFIG_LPC54_LCD_BACKLIGHT is defined, then the board-specific
 *   logic must provide this interface to turn the backlight on and off.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC54_LCD_BACKLIGHT
void lpc54_backlight(bool blon)
{
  lpc54_gpio_write(GPIO_LCD_BL, blon); /* High illuminates */
}
#endif
