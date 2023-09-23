/****************************************************************************
 * boards/arm/nrf52/thingy52/src/nrf52_sensors.c
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
#include <sys/types.h>
#include <syslog.h>

#include <nuttx/i2c/i2c_master.h>

#ifdef CONFIG_SENSORS_HTS221
#  include <nuttx/sensors/hts221.h>
#endif

#ifdef CONFIG_SENSORS_LPS22HB
#  include <nuttx/sensors/lps22hb.h>
#endif

#ifdef CONFIG_SENSORS_CCS811
#  include <nuttx/sensors/ccs811.h>
#endif

#ifdef CONFIG_SENSORS_BH1745NUC
#  include <nuttx/sensors/bh1745nuc.h>
#endif

#include "nrf52_i2c.h"
#include "thingy52.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SENSORS_CCS811
static const struct ccs811_config_s g_ccs811_config =
{
  .wake = nrf52_ccs811_wake,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_sensors_init
 *
 * Description:
 *   Initialzie on-board sensors
 *
 ****************************************************************************/

int nrf52_sensors_init(void)
{
  FAR struct i2c_master_s *i2c;
  int ret = OK;

  UNUSED(ret);

  /* All on-board sensors are connected to the I2C0 bus */

  i2c = nrf52_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C0 interface\n");
      return -ENODEV;
    }

#ifdef CONFIG_SENSORS_HTS221
  ret = hts221_register_uorb(0, i2c, HTS221_I2C_ADDR, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: hts221_register_uorb() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_LPS22HB
  ret = lps22hb_register_uorb(0, i2c, LPS22HB_I2C_ADDR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lps22hb_register_uorb() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_CCS811
  ret = ccs811_register_uorb(0, i2c, CCS811_I2C_ADDR, &g_ccs811_config);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: ccs811_register_uorb() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_BH1745NUC
  ret = bh1745nuc_register_uorb(0, i2c, BH1745NUC_I2C_ADDR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: bh1745nuc_register_uorb() failed: %d\n", ret);
    }
#endif

  return ret;
}
