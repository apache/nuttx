/****************************************************************************
 * boards/arm/stm32/common/src/stm32_amg88xx.c
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
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/amg88xx.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* What is the point of passing devno if only
 * a single config struct is defined
 * irm = infrared map / infrared matrix
 */

#define AMG88XX_DEVNO_PATH_0                  "/dev/irm0"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* One sensor connected to the board */

/* During device registration OS allocates an pointer to an amg88xx_config_s
 * not the actual struct, that implies that for each sensor there is the
 * need for a config struct at a valid memory location which will be passed
 * to the pointer mentioned above.  So there can be only one sensor,
 * as there is one global config struct instantiated here.
 * Is this the intended behaviour to minimize the memory allocated through
 * malloc?
 */

static struct amg88xx_config_s g_amg88xx_config_0 =
{
  .addr = CONFIG_SENSOR_AMG88XX_ADDR,
  .speed = I2C_SPEED_STANDARD
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_amg88xx_initialize()
 *
 * Description:
 *   Initialize and register the AMG88xx infrared matrix sensor driver.
 *
 * Input Parameters:
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_amg88xx_initialize(int busno)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing AMG88xx!\n");

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);

  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the sensor */

  ret = amg88xx_register(AMG88XX_DEVNO_PATH_0, i2c, &g_amg88xx_config_0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Error registering AMG88xx\n");
    }

  return ret;
}
