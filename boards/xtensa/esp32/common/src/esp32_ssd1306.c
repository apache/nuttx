/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_ssd1306.c
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

/* SSD1306 OLED over I2C */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ssd1306.h>
#include <nuttx/i2c/i2c_master.h>

#if defined(CONFIG_VIDEO_FB) && defined(CONFIG_LCD_FRAMEBUFFER)
#  include <nuttx/video/fb.h>
#endif

#include "esp32_gpio.h"
#include "esp32_i2c.h"
#include "hardware/esp32_gpio_sigmap.h"

#define HAVE_SSD1306 1

#if !defined(CONFIG_ESP32_I2C) || !defined(CONFIG_ESP32_I2C0) || \
    !defined(CONFIG_LCD_SSD1306_I2C)
#  undef HAVE_SSD1306
#endif

#define GPIO_SSD1306_RST 16

#ifdef HAVE_SSD1306

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define OLED_I2C_PORT         0 /* OLED display connected to I2C0 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lcd_dev_s    *g_lcddev;

/* Configuration ************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_lcd_initialize
 ****************************************************************************/

int board_lcd_initialize(void)
{
  struct i2c_master_s *i2c;
  const int busno = OLED_I2C_PORT;
  const int devno = 0;
  int ret = OK;

  /* Configure the OLED GPIOs. This initial configuration is RESET low,
   * putting the OLED into reset state.
   */

  esp32_gpio_matrix_out(GPIO_SSD1306_RST, SIG_GPIO_OUT_IDX, 0, 0);
  esp32_configgpio(GPIO_SSD1306_RST, OUTPUT_FUNCTION_3 | INPUT_FUNCTION_3);
  esp32_gpiowrite(GPIO_SSD1306_RST, 0);

  /* Wait a bit then release the OLED from the reset state */

  up_mdelay(20);
  esp32_gpiowrite(GPIO_SSD1306_RST, 1);

  /* Initialize I2C */

  i2c = esp32_i2cbus_initialize(busno);
  if (!i2c)
    {
      lcderr("ERROR: Failed to initialize I2C%d\n", busno);
      return -ENODEV;
    }

  /* Bind the I2C port to the OLED */

  g_lcddev = ssd1306_initialize(i2c, NULL, devno);
  if (!g_lcddev)
    {
      lcderr("ERROR: Failed to bind I2C%d to OLED %d\n", busno, devno);
      return -ENODEV;
    }

  lcdinfo("Bound I2C0 to OLED %d\n", devno);

  /* And turn the OLED on */

  g_lcddev->setpower(g_lcddev, CONFIG_LCD_MAXPOWER);

#if defined(CONFIG_VIDEO_FB) && defined(CONFIG_LCD_FRAMEBUFFER)

  /* Initialize and register the simulated framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
      return -ENODEV;
    }
#endif

  return ret;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  if (lcddev == 0)
    {
      return g_lcddev;
    }

  return NULL;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
}

#endif /* HAVE_SSD1306 */
