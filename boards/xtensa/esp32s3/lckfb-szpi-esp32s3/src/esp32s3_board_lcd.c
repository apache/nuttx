/****************************************************************************
 * boards/xtensa/esp32s3/lckfb-szpi-esp32s3/src/esp32s3_board_lcd.c
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

#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <debug.h>
#include <errno.h>
#include <sys/ioctl.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/signal.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/st7789.h>
#include <nuttx/fs/fs.h>
#include <nuttx/timers/pwm.h>

#include <arch/board/board.h>

#include "esp32s3_gpio.h"
#include "esp32s3_spi.h"
#include "hardware/esp32s3_gpio_sigmap.h"

#include "esp32s3-szpi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spi_dev_s *g_spidev;
static struct lcd_dev_s *g_lcd;
static struct file g_pwm_file;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use, but
 *   with the power setting at 0 (full off).
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  struct pwm_info_s pwm;
  struct file f;
  ssize_t n;
  int ret;

  /* Pull down C/S */

  ret = file_open(&f, SZPI_LCD_CS_PATH, O_RDWR);
  if (ret < 0)
    {
      spierr("open C/S pin failed\n");
      return -ENODEV;
    }

  n = file_write(&f, "1", 1);
  file_close(&f);
  if (n != 1)
    {
      spierr("write C/S pin failed\n");
      return -EIO;
    }

  /* Turn on LCD backlight (10% brightness) */

  pwm.frequency = SZPI_LCD_PWM_FREQ;
  pwm.duty = SZPI_LCD_PWM_DUTY;

  ret = file_open(&g_pwm_file, SZPI_LCD_PWM_PATH, O_RDONLY);
  if (ret < 0)
    {
      pwmerr("Open PWM failed\n");
      return -ENODEV;
    }

  ret = file_ioctl(&g_pwm_file, PWMIOC_SETCHARACTERISTICS,
                   (unsigned long)((uintptr_t)&pwm));
  if (ret < 0)
    {
      pwmerr("Set PWM failed\n");
      file_close(&g_pwm_file);
      return -ENOTTY;
    }

  ret = file_ioctl(&g_pwm_file, PWMIOC_START, 0);
  if (ret < 0)
    {
      pwmerr("Start PWM failed\n");
      file_close(&g_pwm_file);
      return -ENOTTY;
    }

  g_spidev = esp32s3_spibus_initialize(ESP32S3_SPI2);
  if (!g_spidev)
    {
      lcderr("ERROR: Failed to initialize SPI port %d\n", ESP32S3_SPI2);
      return -ENODEV;
    }

  g_lcd = st7789_lcdinitialize(g_spidev);
  if (!g_lcd)
    {
      lcderr("ERROR: st7789_lcdinitialize() failed\n");
      return -ENODEV;
    }

  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 * Input Parameters:
 *   devno - LCD device number
 *
 * Returned Value:
 *   LCD device pointer if success or NULL if failed.
 *
 ****************************************************************************/

struct lcd_dev_s *board_lcd_getdev(int devno)
{
  if (!g_lcd)
    {
      lcderr("ERROR: Failed to bind SPI port %d to LCD %d\n",
             ESP32S3_SPI2, devno);
    }
  else
    {
      lcdinfo("SPI port %d bound to LCD %d\n", ESP32S3_SPI2, devno);
      return g_lcd;
    }

  return NULL;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
  int ret;

  /* Turn the display off */

  g_lcd->setpower(g_lcd, 0);

  /* Close backlight PWM */

  ret = file_ioctl(&g_pwm_file, PWMIOC_STOP, 0);
  if (ret < 0)
    {
      pwmerr("Stop PWM failed\n");
    }

  file_close(&g_pwm_file);
}
