/****************************************************************************
 * boards/arm/common/stm32/src/stm32_gt9xx.c
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
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/input/gt9xx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>

#include "stm32.h"
#include "stm32_exti.h"
#include "stm32_i2c.h"
#include "stm32_gt9xx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* INT low at the reset release selects this address */

#define GT9XX_I2C_ADDR      0x5d

/* Reset timing as in the Zephyr gt911 driver: reset low for at least
 * 10 ms after power-on, then INT held low for 55 ms after the release.
 */

#define GT9XX_RESET_DELAY   20000
#define GT9XX_ADDR_DELAY    55000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_gt9xx_s
{
  struct gt9xx_board_s board;   /* Must be first */
  uint32_t intpin;              /* Interrupt GPIO */
  xcpt_t handler;               /* Driver interrupt handler */
  void *arg;                    /* Driver interrupt argument */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int stm32_gt9xx_attach(const struct gt9xx_board_s *state,
                              xcpt_t handler, void *arg);
static void stm32_gt9xx_enable(const struct gt9xx_board_s *state,
                               bool enable);
static int stm32_gt9xx_power(const struct gt9xx_board_s *state, bool on);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_gt9xx_attach(const struct gt9xx_board_s *state,
                              xcpt_t handler, void *arg)
{
  struct stm32_gt9xx_s *priv = (struct stm32_gt9xx_s *)state;

  priv->handler = handler;
  priv->arg     = arg;
  return OK;
}

static void stm32_gt9xx_enable(const struct gt9xx_board_s *state,
                               bool enable)
{
  struct stm32_gt9xx_s *priv = (struct stm32_gt9xx_s *)state;

  stm32_gpiosetevent(priv->intpin, true, false, true,
                     enable ? priv->handler : NULL,
                     enable ? priv->arg : NULL);
}

static int stm32_gt9xx_power(const struct gt9xx_board_s *state, bool on)
{
  /* The supply is not switched and the reset may be shared with other
   * devices, so it is only pulsed once, at registration.
   */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_gt9xx_initialize
 ****************************************************************************/

int board_gt9xx_initialize(int devno, int busno, uint32_t intpin,
                           uint32_t rstpin)
{
  struct stm32_gt9xx_s *priv;
  struct i2c_master_s *i2c;
  char devpath[16];
  int ret;

  /* Reset the controller with INT driven low to select its address */

  stm32_configgpio(rstpin);
  stm32_configgpio((intpin & (GPIO_PORT_MASK | GPIO_PIN_MASK)) |
                   GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_CLEAR);
  nxsig_usleep(GT9XX_RESET_DELAY);
  stm32_gpiowrite(rstpin, true);
  nxsig_usleep(GT9XX_ADDR_DELAY);
  stm32_configgpio(intpin);

  priv = kmm_zalloc(sizeof(struct stm32_gt9xx_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->board.irq_attach = stm32_gt9xx_attach;
  priv->board.irq_enable = stm32_gt9xx_enable;
  priv->board.set_power  = stm32_gt9xx_power;
  priv->intpin           = intpin;

  i2c = stm32_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      kmm_free(priv);
      return -ENODEV;
    }

  snprintf(devpath, sizeof(devpath), "/dev/input%d", devno);
  ret = gt9xx_register(devpath, i2c, GT9XX_I2C_ADDR, &priv->board);
  if (ret < 0)
    {
      stm32_i2cbus_uninitialize(i2c);
      kmm_free(priv);
    }

  return ret;
}
