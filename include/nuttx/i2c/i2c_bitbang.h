/****************************************************************************
 * include/nuttx/i2c/i2c_bitbang.h
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

#ifndef __INCLUDE_NUTTX_I2C_I2C_BITBANG_H
#define __INCLUDE_NUTTX_I2C_I2C_BITBANG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <stdbool.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct i2c_bitbang_lower_dev_s;

struct i2c_bitbang_lower_ops_s
{
  /* Initialize pins to appropriate state (usually open-drain) */

  CODE void (*initialize)(FAR struct i2c_bitbang_lower_dev_s *lower);

  /* Set high/low level for SCL/SDA pins */

  CODE void (*set_scl)(FAR struct i2c_bitbang_lower_dev_s *lower, bool high);
  CODE void (*set_sda)(FAR struct i2c_bitbang_lower_dev_s *lower, bool high);

  /* Read level of SCL/SDA pins */

  CODE bool (*get_scl)(FAR struct i2c_bitbang_lower_dev_s *lower);
  CODE bool (*get_sda)(FAR struct i2c_bitbang_lower_dev_s *lower);
};

struct i2c_bitbang_lower_dev_s
{
  FAR const struct i2c_bitbang_lower_ops_s *ops;
  FAR void *priv;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: i2c_bitbang_initialize
 *
 * Description:
 *   Initialize a bitbang I2C device instance
 *
 * Input Parameters:
 *   lower  - Lower half of driver
 *
 * Returned Value:
 *   Pointer to a the I2C instance
 *
 ****************************************************************************/

FAR struct i2c_master_s *i2c_bitbang_initialize(
    FAR struct i2c_bitbang_lower_dev_s *lower);

#endif /* __INCLUDE_NUTTX_I2C_I2C_BITBANG_H */
