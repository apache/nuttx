/****************************************************************************
 * drivers/i2c/i2c_bitbang_ioexpander.c
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

#include <assert.h>
#include <debug.h>
#include <stdbool.h>

#include <nuttx/kmalloc.h>
#include <nuttx/i2c/i2c_bitbang.h>
#include <nuttx/i2c/i2c_bitbang_ioexpander.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct i2c_bb_ioe_s
{
  struct i2c_bitbang_lower_dev_s lower;
  FAR struct ioexpander_dev_s *ioe;
  int sda_pin;
  int scl_pin;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void i2c_bb_ioe_init(FAR struct i2c_bitbang_lower_dev_s *lower);
static void i2c_bb_ioe_set_scl(FAR struct i2c_bitbang_lower_dev_s *lower,
                               bool value);
static void i2c_bb_ioe_set_sda(FAR struct i2c_bitbang_lower_dev_s *lower,
                               bool value);
static bool i2c_bb_ioe_get_scl(FAR struct i2c_bitbang_lower_dev_s *lower);
static bool i2c_bb_ioe_get_sda(FAR struct i2c_bitbang_lower_dev_s *lower);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower-half I2C bitbang data */

const static struct i2c_bitbang_lower_ops_s g_i2c_bitbang_ops =
{
  .initialize = i2c_bb_ioe_init,
  .set_scl    = i2c_bb_ioe_set_scl,
  .set_sda    = i2c_bb_ioe_set_sda,
  .get_scl    = i2c_bb_ioe_get_scl,
  .get_sda    = i2c_bb_ioe_get_sda
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_bb_ioe_init
 *
 * Description:
 *   Initialize the I2C bit-bang driver
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2c_bb_ioe_init(FAR struct i2c_bitbang_lower_dev_s *lower)
{
  FAR struct i2c_bb_ioe_s *dev = lower->priv;

  IOEXP_WRITEPIN(dev->ioe, dev->scl_pin, 1);
  IOEXP_WRITEPIN(dev->ioe, dev->sda_pin, 1);

  IOEXP_SETDIRECTION(dev->ioe, dev->scl_pin,
                     IOEXPANDER_DIRECTION_OUT_OPENDRAIN);
  IOEXP_SETDIRECTION(dev->ioe, dev->sda_pin,
                     IOEXPANDER_DIRECTION_OUT_OPENDRAIN);
}

/****************************************************************************
 * Name: i2c_bb_ioe_set_scl
 *
 * Description:
 *   Set SCL line value
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *   value - The value to be written (0 or 1).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2c_bb_ioe_set_scl(FAR struct i2c_bitbang_lower_dev_s *lower,
                               bool value)
{
  FAR struct i2c_bb_ioe_s *dev = lower->priv;

  IOEXP_WRITEPIN(dev->ioe, dev->scl_pin, value);
}

/****************************************************************************
 * Name: i2c_bb_ioe_set_sda
 *
 * Description:
 *   Set SDA line value
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *   value - The value to be written (0 or 1).
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void i2c_bb_ioe_set_sda(FAR struct i2c_bitbang_lower_dev_s *lower,
                               bool value)
{
  FAR struct i2c_bb_ioe_s *dev = lower->priv;

  IOEXP_WRITEPIN(dev->ioe, dev->sda_pin, value);
}

/****************************************************************************
 * Name: i2c_bb_ioe_get_scl
 *
 * Description:
 *   Get value from SCL line
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   The boolean representation of the SCL line value (true/false).
 *
 ****************************************************************************/

static bool i2c_bb_ioe_get_scl(FAR struct i2c_bitbang_lower_dev_s *lower)
{
  FAR struct i2c_bb_ioe_s *dev = lower->priv;
  bool value = true;
  int ret;

  ret = IOEXP_READPIN(dev->ioe, dev->scl_pin, &value);
  if (ret < 0)
    {
      i2cerr("read scl pin:%d, error res:%d\n", dev->scl_pin, ret);
    }

  return value;
}

/****************************************************************************
 * Name: i2c_bb_ioe_get_sda
 *
 * Description:
 *   Get value from SDA line
 *
 * Input Parameters:
 *   lower - A pointer the publicly visible representation of
 *           the "lower-half" driver state structure.
 *
 * Returned Value:
 *   The boolean representation of the SDA line value (true/false).
 *
 ****************************************************************************/

static bool i2c_bb_ioe_get_sda(FAR struct i2c_bitbang_lower_dev_s *lower)
{
  FAR struct i2c_bb_ioe_s *dev = lower->priv;
  bool value = true;
  int ret;

  ret = IOEXP_READPIN(dev->ioe, dev->sda_pin, &value);
  if (ret < 0)
    {
      i2cerr("read sda pin:%d, error res:%d\n", dev->sda_pin, ret);
    }

  return value;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_bitbang_ioexpander_initialize
 *
 * Description:
 *   Initialize i2c bitbang ioexapnder lower half driver.
 *
 * Input Parameters:
 *  ioe     - An instance of the ioexpander device to use for bitbanging
 *  scl_pin - The pin number to use for SCL
 *  sda_pin - The pin number to use for SDA
 *  busnum  - The I2C bus number to register
 *
 *  Returned Value:
 *  On success, a pointer to the initialized I2C driver for the specified.
 *
 ****************************************************************************/

FAR struct i2c_master_s *
i2c_bitbang_ioexpander_initialize(FAR struct ioexpander_dev_s *ioe,
                                  int scl_pin, int sda_pin, int busnum)
{
  FAR struct i2c_master_s *master = NULL;
  FAR struct i2c_bb_ioe_s *priv;

  priv = kmm_zalloc(sizeof(struct i2c_bb_ioe_s));
  if (priv == NULL)
    {
      i2cerr("i2c-bitbang%d kmm_zalloc failed\n", busnum);
      return master;
    }

  priv->lower.priv = priv;
  priv->lower.ops = &g_i2c_bitbang_ops;
  priv->scl_pin = scl_pin;
  priv->sda_pin = sda_pin;
  priv->ioe = ioe;

  master = i2c_bitbang_initialize(&priv->lower);
  if (master == NULL)
    {
      kmm_free(priv);
      i2cerr("I2c bitbang:%d initialize failed\n", busnum);
      return master;
    }

  if (busnum >= 0)
    {
      int ret;

      ret = i2c_register(master, busnum);
      if (ret < 0)
        {
          kmm_free(priv);
          kmm_free(master);
          master = NULL;
          i2cerr("I2c bitbang:%d register failed:%d\n", busnum, ret);
        }
    }

  return master;
}
