/****************************************************************************
 * boards/arm/stm32h7/stm32h735g-dk/src/stm32_touchscreen.c
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
#include <stdio.h>
#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/gt9xx.h>
#include <nuttx/signal.h>

#include "stm32_gpio.h"
#include "stm32_i2c.h"
#include "stm32h735g-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_I2C4
#  error "The touchscreen requires CONFIG_STM32_I2C4"
#endif

#define TOUCH_I2C_PORT      4
#define TOUCH_I2C_FREQUENCY 100000
#define GT911_I2C_ADDRESS   0x5d

/* The GT911 needs ~200 ms after power-on to report its ID */

#define GT911_PROBE_RETRIES 50
#define GT911_PROBE_DELAY   10000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static xcpt_t g_touch_handler;
static void *g_touch_arg;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_gt911_attach(const struct gt9xx_board_s *state,
                              xcpt_t handler, void *arg)
{
  g_touch_handler = handler;
  g_touch_arg = arg;
  return OK;
}

static void stm32_gt911_enable(const struct gt9xx_board_s *state,
                               bool enable)
{
  stm32_gpiosetevent(GPIO_TOUCH_INT, false, true, true,
                     enable ? g_touch_handler : NULL,
                     enable ? g_touch_arg : NULL);
}

static int stm32_gt911_power(const struct gt9xx_board_s *state, bool on)
{
  /* The touch panel supply and reset are not controlled by GPIOs. */

  return OK;
}

static const struct gt9xx_board_s g_gt911_config =
{
  .irq_attach = stm32_gt911_attach,
  .irq_enable = stm32_gt911_enable,
  .set_power  = stm32_gt911_power
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_tsc_setup
 *
 * Description:
 *   Register the GT911 touchscreen on I2C4.
 *
 ****************************************************************************/

int stm32_tsc_setup(int minor)
{
  struct i2c_master_s *i2c;
  struct i2c_config_s config =
  {
    .frequency = TOUCH_I2C_FREQUENCY,
    .address   = GT911_I2C_ADDRESS,
    .addrlen   = 7
  };

  uint8_t reg[2];
  uint8_t id[4];
  char devpath[16];
  int ret;
  int i;

  stm32_configgpio(GPIO_TOUCH_INT);
  i2c = stm32_i2cbus_initialize(TOUCH_I2C_PORT);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  for (i = 0; i < GT911_PROBE_RETRIES; i++)
    {
      reg[0] = 0x81;
      reg[1] = 0x40;
      ret = i2c_writeread(i2c, &config, reg, 2, id, sizeof(id));
      if (ret >= 0 && id[0] == '9' && id[1] == '1' && id[2] == '1')
        {
          break;
        }

      nxsig_usleep(GT911_PROBE_DELAY);
    }

  if (i == GT911_PROBE_RETRIES)
    {
      stm32_i2cbus_uninitialize(i2c);
      return -ENODEV;
    }

  snprintf(devpath, sizeof(devpath), "/dev/input%d", minor);
  ret = gt9xx_register(devpath, i2c, GT911_I2C_ADDRESS, &g_gt911_config);
  if (ret < 0)
    {
      stm32_i2cbus_uninitialize(i2c);
    }

  return ret;
}
