/****************************************************************************
 * boards/arm/stm32/stm32f401rc-rs485/src/stm32_lcd_ssd1306.c
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

#include <stdio.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>

#include "stm32.h"
#include "stm32f401rc-rs485.h"

#include "stm32_ssd1306.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OLED_SPI_PORT         1 /* OLED display connected to SPI1 */

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

  /* Initialize the RESET and DC pins */

  stm32_configgpio(GPIO_OLED_RESET);
  stm32_configgpio(GPIO_OLED_DC);

  /* Reset the OLED display */

  stm32_gpiowrite(GPIO_OLED_RESET, 0);
  up_mdelay(1);
  stm32_gpiowrite(GPIO_OLED_RESET, 1);
  up_mdelay(120);

  ret = board_ssd1306_initialize(OLED_SPI_PORT);
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

struct lcd_dev_s *board_lcd_getdev(int devno)
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
