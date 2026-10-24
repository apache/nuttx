/****************************************************************************
 * boards/arm/gd32w51x/gd-xd-w515-eval/src/gd32_lcd.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7735.h>

#include <arch/board/board.h>

#include "gd32w51x_gpio.h"
#include "gd32w51x_spi.h"
#include "gd-xd-w515-eval.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The backlight (PC14, active high) is switched fully on with the
 * display
 */

#define GPIO_LCD_BL             (GPIO_CFG_MODE_OUTPUT | GPIO_CFG_PUPD_NONE | \
                                 GPIO_CFG_PP | GPIO_CFG_SPEED_2MHZ | \
                                 GPIO_CFG_OUTPUT_SET | GPIO_CFG_PORT_C | \
                                 GPIO_CFG_PIN_14)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lcd_dev_s *g_lcd;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 *
 * Description:
 *   Reset the ST7735S on SPI0 and bind the LCD driver to it.
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  struct spi_dev_s *spi;

  if (g_lcd != NULL)
    {
      return OK;
    }

  gd32_spidev_initialize();

  /* Hardware reset, timed as in the vendor firmware */

  gd32_gpio_config(GPIO_LCD_RST);
  up_mdelay(100);
  gd32_gpio_write(GPIO_LCD_RST, true);
  up_mdelay(50);

  spi = gd32_spibus_initialize(0);
  if (spi == NULL)
    {
      lcderr("ERROR: failed to initialize SPI0\n");
      return -ENODEV;
    }

  g_lcd = st7735_lcdinitialize(spi);
  if (g_lcd == NULL)
    {
      lcderr("ERROR: st7735_lcdinitialize() failed\n");
      return -ENODEV;
    }

  gd32_gpio_config(GPIO_LCD_BL);

  return OK;
}

/****************************************************************************
 * Name: board_lcd_getdev
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  return devno == 0 ? g_lcd : NULL;
}

/****************************************************************************
 * Name: board_lcd_uninitialize
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  g_lcd = NULL;
}
