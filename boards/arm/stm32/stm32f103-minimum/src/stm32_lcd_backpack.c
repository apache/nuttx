/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_lcd_backpack.c
 *
 *   Copyright (C) 2019 Alan Carvalho de Assis. All rights reserved.
 *   Author: Alan Carvalho de Assis <acassis@gmail.com>
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

#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/lcd/pcf8574_lcd_backpack.h>

#include "stm32.h"
#include "stm32f103_minimum.h"

#if defined(CONFIG_I2C) && defined(CONFIG_LCD_BACKPACK)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LCDBP_I2C_PORTNO 1     /* PCF8574 connected to I2C1 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_lcd_backpack_init
 *
 * Description:
 *   Initialize the LCD1602 display controlled by Backpack with PCF8574
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/slcd0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_lcd_backpack_init(FAR const char *devpath)
{
  FAR struct pcf8574_lcd_backpack_config_s cfg =
    LCD_I2C_BACKPACK_CFG_SAINSMART;
  FAR struct i2c_master_s *i2c;
  int ret;

  /* Setup the LCD row and cols size.
   * Note: We are using the LCD_I2C_BACKPACK_CFG_SAINSMART config that
   *       defined the I2C Address to 0x27 to PCF8574. Double check if all
   *       the bits (pins) from PCF8574 connected to the LCD controller
   *       are correct with this LCD CFG definition.
   */

  cfg.rows = 2;
  cfg.cols = 16;

  /* Initialize the I2C1 */

  i2c = stm32_i2cbus_initialize(LCDBP_I2C_PORTNO);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

#ifdef CONFIG_I2C_DRIVER
  /* Register the I2C to get the "nsh> i2c bus" command working */

  ret = i2c_register(i2c, LCDBP_I2C_PORTNO);
  if (ret < 0)
    {
      rtcerr("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
      return -ENODEV;
    }
#endif

  /* Register the LCD BACKPACK PCF8574 driver at the specified location. */

  ret = pcf8574_lcd_backpack_register(devpath, i2c, &cfg);
  if (ret < 0)
    {
      lcderr("ERROR: pcf8574_lcd_backpack_register(%s) failed: %d\n",
             devpath, ret);
      return ret;
    }

  return OK;
}

#endif /* CONFIG_I2C && CONFIG_LCD_BACKPACK */
