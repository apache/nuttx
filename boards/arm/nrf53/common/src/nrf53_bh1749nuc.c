/****************************************************************************
 * boards/arm/nrf53/common/src/nrf53_bh1749nuc.c
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

#include <debug.h>
#include <errno.h>
#include <stdio.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bh1749nuc.h>

#include "nrf53_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SENSORS_BH1749NUC_UORB
#  error BH1749NUC UORB is only supported
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_bh1749nuc_init
 *
 * Description:
 *   Initialize and register the BH1749NUC as uorb sensor
 *
 * Input Parameters:
 *   devno - The user specifies device number, from 0.
 *   busno - I2C bus number
 *   addr  - The I2C address
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nrf53_bh1749nuc_init(int devno, int busno, uint8_t addr)
{
  struct bh1749nuc_config_s  config;
  struct i2c_master_s       *i2c;
  int                        ret;

  sninfo("Initializing BH1749NUC!\n");

  /* Initialize I2C */

  i2c = nrf53_i2cbus_initialize(busno);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the barometer sensor */

  config.i2c  = i2c;
  config.addr = addr;

  ret = bh1749nuc_register_uorb(devno, &config);
  if (ret < 0)
    {
      snerr("ERROR: Error registering BH1749NUC\n");
    }

  return ret;
}
