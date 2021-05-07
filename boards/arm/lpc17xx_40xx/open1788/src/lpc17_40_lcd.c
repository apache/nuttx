/****************************************************************************
 * boards/arm/lpc17xx_40xx/open1788/src/lpc17_40_lcd.c
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

#include "lpc17_40_lcd.h"
#include "lpc17_40_gpio.h"

#include "open1788.h"

#ifdef CONFIG_LPC17_40_LCD

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: open1788_lcd_initialize
 *
 * Description:
 *   Initialize the LCD.  Setup backlight (initially off)
 *
 ****************************************************************************/

void open1788_lcd_initialize(void)
{
  /* Configure the LCD backlight (and turn the backlight off) */

  lpc17_40_configgpio(GPIO_LCD_BL);
}

/****************************************************************************
 * Name: lpc17_40_backlight
 *
 * Description:
 *   If CONFIG_LPC17_40_LCD_BACKLIGHT is defined, then the board-specific
 *   logic must provide this interface to turn the backlight on and off.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC17_40_LCD_BACKLIGHT
void lpc17_40_backlight(bool blon)
{
  lpc17_40_gpiowrite(GPIO_LCD_BL, blon);
}
#endif
#endif /* CONFIG_LPC17_40_LCD */
