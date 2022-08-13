/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_isx012.c
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

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if the following are defined in the board.h */

#ifndef IMAGER_RST
#  error "IMAGER_RST must be defined in board.h !!"
#endif
#ifndef IMAGER_SLEEP
#  error "IMAGER_SLEEP must be defined in board.h !!"
#endif

#define STANDBY_TIME                (600*1000) /* TODO: (max100ms/30fps)*/
#define DEVICE_STARTUP_TIME           (6*1000) /* ms */
#define SLEEP_CANCEL_TIME            (13*1000) /* ms */
#define POWER_CHECK_TIME             (1*1000)  /* ms */

#define POWER_CHECK_RETRY           (10)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_isx012_power_on(void)
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

      if (true == board_power_monitor(POWER_IMAGE_SENSOR))
        {
          ret = OK;
          break;
        }
    }

  return ret;
}

int board_isx012_power_off(void)
{
  int ret;
  int i;

  ret = board_power_control(POWER_IMAGE_SENSOR, false);
  if (ret)
    {
      _err("ERROR: Failed to power off ImageSensor. %d\n", ret);
      return -ENODEV;
    }

  ret = -ETIMEDOUT;
  for (i = 0; i < POWER_CHECK_RETRY; i++)
    {
      if (false == board_power_monitor(POWER_IMAGE_SENSOR))
        {
          ret = OK;
          break;
        }

      nxsig_usleep(POWER_CHECK_TIME);
    }

  return ret;
}

void board_isx012_set_reset(void)
{
  cxd56_gpio_write(IMAGER_RST, false);
}

void board_isx012_release_reset(void)
{
  cxd56_gpio_write(IMAGER_RST, true);
}

void board_isx012_set_sleep(int kind)
{
  cxd56_gpio_write(IMAGER_SLEEP, false);
  if (kind == 0)
    {
      /* PowerON -> sleep */

      nxsig_usleep(DEVICE_STARTUP_TIME);
    }
  else
    {
      /* active -> sleep */

      nxsig_usleep(STANDBY_TIME);
    }
}

void board_isx012_release_sleep(void)
{
  cxd56_gpio_write(IMAGER_SLEEP, true);
  nxsig_usleep(SLEEP_CANCEL_TIME);
}

int isx012_register(struct i2c_master_s *i2c);
int isx012_unregister(void);

struct i2c_master_s *board_isx012_initialize(void)
{
  _info("Initializing ISX012...\n");

#ifdef IMAGER_ALERT
  cxd56_gpio_config(IMAGER_ALERT, true);
#endif
  cxd56_gpio_config(IMAGER_SLEEP, false);
  cxd56_gpio_config(IMAGER_RST, false);
  board_isx012_set_reset();
  cxd56_gpio_write(IMAGER_SLEEP, false);

  CXD56_PIN_CONFIGS(PINCONFS_IS);

  /* Initialize i2c device */

  return cxd56_i2cbus_initialize(IMAGER_I2C);
}

int board_isx012_uninitialize(struct i2c_master_s *i2c)
{
  int ret;

  _info("Uninitializing ISX012...\n");

  /* Initialize i2c device */

  ret = isx012_uninitialize();
  if (ret < 0)
    {
      _err("Failed to uninitialize ISX012.\n");
    }

  if (!i2c)
    {
      _err("Error uninitialize ISX012.\n");
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
