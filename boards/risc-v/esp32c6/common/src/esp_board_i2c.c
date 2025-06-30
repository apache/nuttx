/****************************************************************************
 * boards/risc-v/esp32c6/common/src/esp_board_i2c.c
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

#include <syslog.h>
#include <errno.h>
#include <sys/types.h>

#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_ESPRESSIF_I2C_BITBANG
#include "espressif/esp_i2c_bitbang.h"
#endif
#ifdef CONFIG_ESPRESSIF_I2C_PERIPH_MASTER_MODE
#include "espressif/esp_i2c.h"
#endif
#ifdef CONFIG_ESPRESSIF_I2C_PERIPH_SLAVE_MODE
#include "espressif/esp_i2c_slave.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_I2C_PERIPH_SLAVE_MODE
#define I2C0_SLAVE_ADDR   0x28
#define I2C0_SLAVE_NBITS  7
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_ESPRESSIF_I2C_BITBANG
static int i2c_bitbang_driver_init(int bus)
{
  struct i2c_master_s *i2c;
  int ret = OK;

  i2c = esp_i2cbus_bitbang_initialize();
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "Failed to get I2C%d interface\n", bus);
      return -ENODEV;
    }

#ifdef CONFIG_I2C_DRIVER
  ret = i2c_register(i2c, bus);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register I2C%d driver: %d\n", bus, ret);
    }
#endif

  return ret;
}
#endif

#ifdef CONFIG_ESPRESSIF_I2C_PERIPH_MASTER_MODE
static int i2c_driver_init(int bus)
{
  struct i2c_master_s *i2c;
  int ret = OK;

  i2c = esp_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "Failed to get I2C%d interface\n", bus);
      return -ENODEV;
    }

#ifdef CONFIG_I2C_DRIVER
  ret = i2c_register(i2c, bus);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register I2C%d driver: %d\n", bus, ret);
      esp_i2cbus_uninitialize(i2c);
    }
#endif

  return ret;
}
#endif

#ifdef CONFIG_ESPRESSIF_I2C_PERIPH_SLAVE_MODE
static int i2c_slave_driver_init(int bus, int addr)
{
  struct i2c_slave_s *i2c;
  int ret = OK;

  i2c = esp_i2cbus_slave_initialize(bus, addr);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "Failed to get I2C%d interface\n", bus);
      return -ENODEV;
    }

#ifdef CONFIG_I2C_DRIVER
  ret = i2c_slave_register(i2c, bus, addr, I2C0_SLAVE_NBITS);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to register I2C%d driver: %d\n", bus, ret);
      esp_i2cbus_slave_uninitialize(i2c);
    }
#endif

  return ret;
}
#endif

/****************************************************************************
 * Name: board_i2c_init
 *
 * Description:
 *   Configure the I2C driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_i2c_init(void)
{
  int ret = OK;

#ifdef CONFIG_ESPRESSIF_I2C_PERIPH_MASTER_MODE
  ret = i2c_driver_init(ESPRESSIF_I2C0);
#endif

#ifdef CONFIG_ESPRESSIF_I2C_BITBANG
  ret = i2c_bitbang_driver_init(ESPRESSIF_I2C_BITBANG);
#endif

#ifdef CONFIG_ESPRESSIF_I2C_PERIPH_SLAVE_MODE
  ret = i2c_slave_driver_init(ESPRESSIF_I2C0_SLAVE, I2C0_SLAVE_ADDR);
#endif

#ifdef CONFIG_ESPRESSIF_LP_I2C0
  ret = i2c_driver_init(ESPRESSIF_LP_I2C0);
#endif

  return ret;
}
