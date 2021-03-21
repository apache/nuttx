/****************************************************************************
 * boards/arm/imxrt/imxrt1050-evk/src/imxrt_lcd.c
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

#include "imxrt1050-evk.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_lcd_initialize
 *
 * Description:
 *   Initialize the LCD.  Setup backlight (initially off)
 *
 ****************************************************************************/

void imxrt_lcd_initialize(void)
{
  /* Configure the LCD backlight (and turn the backlight off) */

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
