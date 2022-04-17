/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/src/stm32_pca9635.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/leds/pca9635pw.h>

#include <arch/irq.h>

#include "stm32.h"
#include "nucleo-h743zi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pca9635_initialize
 *
 * Description:
 *   This function is called by board initialization logic to configure the
 *   LED PWM chip.  This function will register the driver as /dev/leddrv0.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int stm32_pca9635_initialize(void)
{
  struct i2c_master_s *i2c;
  int ret;

  /* Get the I2C driver that interfaces with the pca9635 */

  i2c = stm32_i2cbus_initialize(PCA9635_I2CBUS);
  if (!i2c)
    {
      i2cerr("ERROR: Failed to initialize I2C%d\n", PCA9635_I2CBUS);
      return -1;
    }

  ret = pca9635pw_register("/dev/leddrv0", i2c, PCA9635_I2CADDR);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register PCA9635 driver: %d\n", ret);
      return ret;
    }

  return OK;
}
