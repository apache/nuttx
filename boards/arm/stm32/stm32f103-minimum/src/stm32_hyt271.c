/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_hyt271.c
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

#include <sys/types.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#ifdef CONFIG_SENSORS_HYT271
#  include <nuttx/sensors/hyt271.h>
#endif

#include "chip.h"
#include "arm_internal.h"
#include "stm32_i2c.h"
#include "stm32f103_minimum.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef BOARD_HYT271_NBUS
# error "Configuration BOARD_HYT271_NBUS missing"
#endif
#ifndef BOARD_HYT271_POWOUT
# error "Configuration BOARD_HYT271_POWOUT for gpio power on missing"
#endif
#ifndef BOARD_HYT271_NBUS
# error "Configuration BOARD_HYT271_POWIN for gpio power detection is missing"
#endif

#define HYT271_NSENSORS     1

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_i2c_bus_s
{
    struct hyt271_bus_s bus;
    const int busnr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_i2c_power_reset(struct hyt271_bus_s *bus);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint8_t g_hyt271_sensors[HYT271_NSENSORS] =
{
    0x28
};

static struct stm32_i2c_bus_s g_bus =
{
  .bus =
    {
      .pwonreset = stm32_i2c_power_reset
    },
  .busnr = BOARD_HYT271_NBUS
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_power_reset
 ****************************************************************************/

static int stm32_i2c_power_reset(struct hyt271_bus_s *bus)
{
  volatile int n;
  struct stm32_i2c_bus_s *priv = (struct stm32_i2c_bus_s *)bus;

  if (priv->busnr != BOARD_HYT271_NBUS)
    {
      return -EINVAL;
    }

  stm32_gpiowrite(BOARD_HYT271_POWOUT, false);
  usleep(250000);
  stm32_gpiowrite(BOARD_HYT271_POWOUT, true);

  while (1)
    {
      int value;
      value = stm32_gpioread(BOARD_HYT271_POWIN);
      if (value != 0)
        {
          break;
        }
    }

  /* A small busy loop is necessary when using this hardware configuration
   * for powering the bus.
   */

  for (n = 0; n < 25000; n++);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hyt271initialize
 *
 * Description:
 *   Function used to initialize HYT271 snesors on a i2c bus
 *
 * Parameter:
 *   devno   - First character device number
 *
 * Return
 *   Error or number of device that have been successfully registered.
 *
 ****************************************************************************/

int stm32_hyt271initialize(int devno)
{
  int n;
  int ret;
  int ndevices = 0;
  struct i2c_master_s *i2c;

  iinfo("Setup gpios for bus power handling on I2C%d\n", BOARD_HYT271_NBUS);
  stm32_configgpio(BOARD_HYT271_POWIN);
  stm32_configgpio(BOARD_HYT271_POWOUT);
  stm32_gpiowrite(BOARD_HYT271_POWOUT, true);

  iinfo("Initialize I2C%d\n", BOARD_HYT271_NBUS);
  i2c = stm32_i2cbus_initialize(BOARD_HYT271_NBUS);
  if (!i2c)
    {
      ierr("ERROR: Failed to initialize I2C%d\n", BOARD_HYT271_NBUS);
      return -ENODEV;
    }

  ret = i2c_register(i2c, BOARD_HYT271_NBUS);
  if (ret < 0)
    {
      ierr("ERROR: Failed to register I2C%d driver: %d\n", BOARD_HYT271_NBUS,
           ret);
      return -ENODEV;
    }

  /* Register humidity/temperature sensors */

  for (n = 0; n < HYT271_NSENSORS; n++)
    {
      ret = hyt271_register(devno + n, i2c, g_hyt271_sensors[n],
                            &g_bus.bus);
      if (!ret)
        {
          ndevices++;
        }
    }

  return ndevices;
}
