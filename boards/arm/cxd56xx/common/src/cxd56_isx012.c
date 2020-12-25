/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_isx012.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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
 * Private Data
 ****************************************************************************/

FAR struct i2c_master_s *i2c;

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

int isx012_register(FAR struct i2c_master_s *i2c);
int isx012_unregister(void);

int board_isx012_initialize(int i2c_bus_num)
{
  int ret;

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

  i2c = cxd56_i2cbus_initialize(i2c_bus_num);
  if (!i2c)
    {
      return -ENODEV;
    }

  ret = isx012_register(i2c);
  if (ret < 0)
    {
      _err("Error registering ISX012.\n");
    }

  return ret;
}

int board_isx012_uninitialize(void)
{
  int ret;

  _info("Uninitializing ISX012...\n");

  /* Initialize i2c device */

  ret = isx012_unregister();
  if (ret < 0)
    {
      _err("Error unregistering ISX012.\n");
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
