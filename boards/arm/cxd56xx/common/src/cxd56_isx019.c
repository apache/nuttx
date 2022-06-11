/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_isx019.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/signal.h>

#include "cxd56_gpio.h"
#include "cxd56_pinconfig.h"
#include "cxd56_i2c.h"
#include "cxd56_clock.h"

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

#ifndef IMAGER_RST
#  error "IMAGER_RST must be defined in board.h !!"
#endif

#define POWER_CHECK_TIME            (1 * USEC_PER_MSEC)   /* ms */
#define POWER_OFF_TIME              (50 * USEC_PER_MSEC)  /* ms */

#define POWER_CHECK_RETRY           (10)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_isx019_power_on(void)
{
  int ret;
  int i;

  ret = board_power_control(POWER_IMAGE_SENSOR, true);
  if (ret)
    {
      _err("ERROR: Failed to power on ImageSensor. %d\n", ret);
      return -ENODEV;
    }

  ret = -ETIMEDOUT;
  for (i = 0; i < POWER_CHECK_RETRY; i++)
    {
      /* Need to wait for a while after power-on */

      nxsig_usleep(POWER_CHECK_TIME);

      if (board_power_monitor(POWER_IMAGE_SENSOR))
        {
          ret = OK;
          break;
        }
    }

  return ret;
}

int board_isx019_power_off(void)
{
  int ret;
  int i;

  ret = board_power_control(POWER_IMAGE_SENSOR, false);
  if (ret)
    {
      _err("ERROR: Failed to power off ImageSensor. %d\n", ret);
      return -ENODEV;
    }

  /* Need to wait for power-off to be reflected */

  nxsig_usleep(POWER_OFF_TIME);

  ret = -ETIMEDOUT;
  for (i = 0; i < POWER_CHECK_RETRY; i++)
    {
      if (!board_power_monitor(POWER_IMAGE_SENSOR))
        {
          ret = OK;
          break;
        }

      nxsig_usleep(POWER_CHECK_TIME);
    }

  return ret;
}

void board_isx019_set_reset(void)
{
  cxd56_gpio_write(IMAGER_RST, false);
}

void board_isx019_release_reset(void)
{
  cxd56_gpio_write(IMAGER_RST, true);
}

struct i2c_master_s *board_isx019_initialize(void)
{
  int retry = 50;

  _info("Initializing ISX019...\n");

  while (!g_rtc_enabled && 0 < retry--)
    {
      /* ISX019 requires stable RTC */

      nxsig_usleep(100 * USEC_PER_MSEC);
    }

  cxd56_gpio_config(IMAGER_RST, false);
  board_isx019_set_reset();

  /* To avoid IS_DATA0 and IS_DATA7 being Hi-Z state during FPGA config,
   * output these pins to LOW.
   */

  cxd56_gpio_config(PIN_IS_DATA0, false);
  cxd56_gpio_config(PIN_IS_DATA7, false);
  cxd56_gpio_write(PIN_IS_DATA0, false);
  cxd56_gpio_write(PIN_IS_DATA7, false);

  /* Initialize i2c device */

  return cxd56_i2cbus_initialize(IMAGER_I2C);
}

int board_isx019_uninitialize(struct i2c_master_s *i2c)
{
  int ret;

  _info("Uninitializing ISX019...\n");

  /* Restore IS_DATA0 and IS_DATA7 to Hi-Z state */

  cxd56_gpio_config(PIN_IS_DATA0, false);
  cxd56_gpio_config(PIN_IS_DATA7, false);
  cxd56_gpio_write_hiz(PIN_IS_DATA0);
  cxd56_gpio_write_hiz(PIN_IS_DATA7);

  /* Uninitialize i2c device */

  ret = isx019_uninitialize();
  if (ret < 0)
    {
      _err("Failed to uninitialize ISX019.\n");
    }

  if (i2c == NULL)
    {
      _err("Error uninitialize ISX019.\n");
      return -ENODEV;
    }
  else
    {
      ret = cxd56_i2cbus_uninitialize(i2c);
      if (ret < 0)
        {
          _err("Error uninitialize I2C BUS.\n");
          return -EPERM;
        }
    }

  return ret;
}

uint32_t board_isx019_get_master_clock(void)
{
  return cxd56_get_xosc_clock();
}
