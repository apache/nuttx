/****************************************************************************
 * drivers/regmap/regmap_i2c.c
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

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/regmap/regmap.h>
#include <nuttx/kmalloc.h>

#include <debug.h>

#include "internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Regmap i2c bus configuration. */

struct regmap_bus_i2c_s
{
  struct regmap_bus_s base;
  struct i2c_config_s config;
  FAR struct i2c_master_s *i2c;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* regmap handle functions */

static int regmap_i2c_reg_read(FAR struct regmap_bus_s *bus,
                               unsigned int regaddr, FAR void *value);

static int regmap_i2c_reg_write(FAR struct regmap_bus_s *bus,
                                unsigned int regaddr, unsigned int value);

static int regmap_i2c_write(FAR struct regmap_bus_s *bus,
                            FAR const void *data, unsigned int count);

static int regmap_i2c_read(FAR struct regmap_bus_s *bus,
                           FAR const void *reg, unsigned int reg_size,
                           FAR void *val, unsigned int val_size);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int regmap_i2c_write(FAR struct regmap_bus_s *bus,
                            FAR const void *data, unsigned int count)
{
  FAR struct regmap_bus_i2c_s *dev = (FAR struct regmap_bus_i2c_s *)bus;

  return i2c_write(dev->i2c, &dev->config, data, count);
}

static int regmap_i2c_read(FAR struct regmap_bus_s *bus,
                           FAR const void *reg, unsigned int reg_size,
                           FAR void *val, unsigned int val_size)
{
  FAR struct regmap_bus_i2c_s *dev = (FAR struct regmap_bus_i2c_s *)bus;

  return i2c_writeread(dev->i2c, &dev->config, reg, reg_size, val,
                       val_size);
}

static int regmap_i2c_reg_write(FAR struct regmap_bus_s *bus,
                                unsigned int regaddr, unsigned int value)
{
  uint8_t txbuffer[2];

  txbuffer[0] = regaddr;
  txbuffer[1] = value;

  return regmap_i2c_write(bus, txbuffer, 2);
}

static int regmap_i2c_reg_read(FAR struct regmap_bus_s *bus,
                               unsigned int regaddr, FAR void *value)
{
  uint8_t tmp = regaddr;

  return regmap_i2c_read(bus, &tmp, 1, value, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: regmap_init_i2c
 *
 * Description:
 *   Regmap init i2c bus.
 *
 * Input Parameters:
 *   i2c        - An instance of the I2C interface to use to communicate.
 *   i2c_config - i2c device configuration.
 *   config     - regmap configuration.
 *
 * Returned Value:
 *   Description of the value returned by this function (if any),
 *   including an enumeration of all possible error values.
 *
 * Assumptions/Limitations:
 *   None.
 *
 ****************************************************************************/

FAR struct regmap_s *
regmap_init_i2c(FAR struct i2c_master_s *i2c,
                FAR struct i2c_config_s *i2c_config,
                FAR const struct regmap_config_s *config)
{
  FAR struct regmap_bus_i2c_s *dev;
  FAR struct regmap_s *regmap;

  dev = kmm_zalloc(sizeof(struct regmap_bus_i2c_s));
  if (dev == NULL)
    {
      return NULL;
    }

  dev->base.reg_write = regmap_i2c_reg_write;
  dev->base.reg_read  = regmap_i2c_reg_read;
  dev->base.write     = regmap_i2c_write;
  dev->base.read      = regmap_i2c_read;

  dev->config.frequency = i2c_config->frequency;
  dev->config.address   = i2c_config->address;
  dev->config.addrlen   = i2c_config->addrlen;

  dev->i2c = i2c;

  regmap = regmap_init(&dev->base, config);
  if (regmap == NULL)
    {
      kmm_free(dev);
    }

  return regmap;
}
