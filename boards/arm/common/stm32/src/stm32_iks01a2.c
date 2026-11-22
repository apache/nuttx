/****************************************************************************
 * boards/arm/common/stm32/src/stm32_iks01a2.c
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
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/hts221.h>
#include <nuttx/sensors/lps22hb.h>
#include <nuttx/sensors/lsm303agr.h>
#include <nuttx/sensors/lsm6ds3trc.h>

#if defined(CONFIG_ARCH_CHIP_STM32L4)
#  include "stm32l4_i2c.h"
#elif defined(CONFIG_ARCH_CHIP_STM32WB)
#  include "stm32wb_i2c.h"
#else
#  include "stm32_i2c.h"
#endif

#include "stm32_iks01a2.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* I2C addresses of the sensors on the shield */

#define IKS01A2_LSM6DSL_ADDR  0x6b
#define IKS01A2_HTS221_ADDR   0x5f
#define IKS01A2_LPS22HB_ADDR  0x5d

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_iks01a2_initialize
 ****************************************************************************/

int board_iks01a2_initialize(int devno, int busno)
{
  struct lsm6ds3trc_config_s imu =
  {
    0
  };

  FAR struct i2c_master_s *i2c;
  int result = OK;
  int ret;

  i2c = stm32_i2cbus_initialize(busno);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  /* The LSM6DSL has the LSM6DS3TR-C registers.  It is polled. */

  ret = lsm6ds3trc_register(i2c, IKS01A2_LSM6DSL_ADDR, devno, &imu);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: LSM6DSL registration failed: %d\n", ret);
      result = ret;
    }

  ret = lsm303agr_register_uorb(i2c, devno + 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: LSM303AGR registration failed: %d\n", ret);
      result = ret;
    }

  ret = hts221_register_uorb(devno, i2c, IKS01A2_HTS221_ADDR, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: HTS221 registration failed: %d\n", ret);
      result = ret;
    }

  ret = lps22hb_register_uorb(devno, i2c, IKS01A2_LPS22HB_ADDR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: LPS22HB registration failed: %d\n", ret);
      result = ret;
    }

  return result;
}
