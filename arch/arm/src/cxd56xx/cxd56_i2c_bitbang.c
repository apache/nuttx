/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_i2c_bitbang.c
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
#include <arch/board/board.h>
#include <arch/chip/pin.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cxd56_i2c_bitbang_dev_s
{
  struct i2c_bitbang_lower_dev_s lower;
  uint32_t sda_pin;
  uint32_t scl_pin;
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
 * Private Functions
 ****************************************************************************/

static void i2c_bb_initialize(struct i2c_bitbang_lower_dev_s *lower)
{
  struct cxd56_i2c_bitbang_dev_s *dev = lower->priv;

  /* Set to input enable and pull-up */

  board_gpio_config(dev->scl_pin, 0, true, true, PIN_PULLUP);
  board_gpio_config(dev->sda_pin, 0, true, true, PIN_PULLUP);
}

static void i2c_bb_set_scl(struct i2c_bitbang_lower_dev_s *lower,
                           bool high)
{
  struct cxd56_i2c_bitbang_dev_s *dev = lower->priv;
  int value;

  /* If set high, pin is pulled up by output disable */

  value = (high) ? -1 : 0;
  board_gpio_write(dev->scl_pin, value);
}

static void i2c_bb_set_sda(struct i2c_bitbang_lower_dev_s *lower,
                           bool high)
{
  struct cxd56_i2c_bitbang_dev_s *dev = lower->priv;
  int value;

  /* If set high, pin is pulled up by output disable */

  value = (high) ? -1 : 0;
  board_gpio_write(dev->sda_pin, value);
}

static bool i2c_bb_get_scl(struct i2c_bitbang_lower_dev_s *lower)
{
  struct cxd56_i2c_bitbang_dev_s *dev = lower->priv;

  return board_gpio_read(dev->scl_pin);
}

static bool i2c_bb_get_sda(struct i2c_bitbang_lower_dev_s *lower)
{
  struct cxd56_i2c_bitbang_dev_s *dev = lower->priv;

  return board_gpio_read(dev->sda_pin);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct i2c_master_s *cxd56_i2c_bitbang_initialize(uint32_t sda_pin,
                                                  uint32_t scl_pin)
{
  struct cxd56_i2c_bitbang_dev_s *dev =
      (struct cxd56_i2c_bitbang_dev_s *)
          kmm_zalloc(sizeof(struct cxd56_i2c_bitbang_dev_s));

  DEBUGASSERT(dev);

  dev->lower.ops = &g_ops;
  dev->lower.priv = dev;
  dev->scl_pin = scl_pin;
  dev->sda_pin = sda_pin;

  return i2c_bitbang_initialize(&dev->lower);
}
