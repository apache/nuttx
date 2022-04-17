/****************************************************************************
 * arch/arm/src/nrf52/nrf52_i2c_bitbang.c
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
#include <assert.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/i2c/i2c_bitbang.h>
#include <nuttx/kmalloc.h>
#include "nrf52_gpio.h"
#include "arm_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nrf52_i2c_bitbang_dev_s
{
  struct i2c_bitbang_lower_dev_s lower;
  nrf52_pinset_t sda_pin;
  nrf52_pinset_t scl_pin;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void i2c_bb_initialize(struct i2c_bitbang_lower_dev_s *lower);

static void i2c_bb_set_scl(struct i2c_bitbang_lower_dev_s *lower,
                           bool high);
static void i2c_bb_set_sda(struct i2c_bitbang_lower_dev_s *lower,
                           bool high);

static bool i2c_bb_get_scl(struct i2c_bitbang_lower_dev_s *lower);
static bool i2c_bb_get_sda(struct i2c_bitbang_lower_dev_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

const static struct i2c_bitbang_lower_ops_s g_ops =
{
  .initialize = i2c_bb_initialize,
  .set_scl    = i2c_bb_set_scl,
  .set_sda    = i2c_bb_set_sda,
  .get_scl    = i2c_bb_get_scl,
  .get_sda    = i2c_bb_get_sda
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void i2c_bb_initialize(struct i2c_bitbang_lower_dev_s *lower)
{
#ifdef CONFIG_NRF52_HAVE_PORT1
  int port;
#endif
  int pin;
  uint32_t base = NRF52_GPIO_P0_BASE;

  struct nrf52_i2c_bitbang_dev_s *dev = lower->priv;

  /* Set to output open-drain (and leave disconnected by writing 1) */

  nrf52_gpio_config(GPIO_OUTPUT | GPIO_VALUE_ONE |
                    GPIO_DRIVE_H0D1 | dev->scl_pin);

  nrf52_gpio_config(GPIO_OUTPUT | GPIO_VALUE_ONE |
                    GPIO_DRIVE_H0D1 | dev->sda_pin);

  /* Enable input buffer to read pin without having to switch direction */

#ifdef CONFIG_NRF52_HAVE_PORT1
  port = GPIO_PORT_DECODE(dev->scl_pin);
  if (port == 1)
    {
      base = NRF52_GPIO_P1_BASE;
    }
#endif

  pin = GPIO_PIN_DECODE(dev->scl_pin);
  modifyreg32(base + NRF52_GPIO_PIN_CNF_OFFSET(pin), GPIO_CNF_INPUT, 0);

#ifdef CONFIG_NRF52_HAVE_PORT1
  port = GPIO_PORT_DECODE(dev->sda_pin);
  if (port == 1)
    {
      base = NRF52_GPIO_P1_BASE;
    }
#endif

  pin = GPIO_PIN_DECODE(dev->sda_pin);
  modifyreg32(base + NRF52_GPIO_PIN_CNF_OFFSET(pin), GPIO_CNF_INPUT, 0);
}

static void i2c_bb_set_scl(struct i2c_bitbang_lower_dev_s *lower,
                           bool high)
{
  struct nrf52_i2c_bitbang_dev_s *dev = lower->priv;

  nrf52_gpio_write(dev->scl_pin, high);
}

static void i2c_bb_set_sda(struct i2c_bitbang_lower_dev_s *lower,
                           bool high)
{
  struct nrf52_i2c_bitbang_dev_s *dev = lower->priv;

  nrf52_gpio_write(dev->sda_pin, high);
}

static bool i2c_bb_get_scl(struct i2c_bitbang_lower_dev_s *lower)
{
  struct nrf52_i2c_bitbang_dev_s *dev = lower->priv;

  return nrf52_gpio_read(dev->scl_pin);
}

static bool i2c_bb_get_sda(struct i2c_bitbang_lower_dev_s *lower)
{
  struct nrf52_i2c_bitbang_dev_s *dev = lower->priv;

  return nrf52_gpio_read(dev->sda_pin);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct i2c_master_s *nrf52_i2c_bitbang_initialize(nrf52_pinset_t sda_pin,
                                                  nrf52_pinset_t scl_pin)
{
  struct nrf52_i2c_bitbang_dev_s *dev =
      (struct nrf52_i2c_bitbang_dev_s *)
          kmm_zalloc(sizeof(struct nrf52_i2c_bitbang_dev_s));

  DEBUGASSERT(dev);

  dev->lower.ops = &g_ops;
  dev->lower.priv = dev;
  dev->scl_pin = scl_pin;
  dev->sda_pin = sda_pin;

  return i2c_bitbang_initialize(&dev->lower);
}

