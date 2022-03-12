/****************************************************************************
 * boards/arm/kinetis/teensy-3.x/src/k20_i2c.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <arch/board/board.h>

#include "arm_arch.h"
#include "chip.h"
#include "kinetis.h"
#include "kinetis_i2c.h"
#include "teensy-3x.h"

#if defined(CONFIG_KINETIS_I2C0) || defined(CONFIG_KINETIS_I2C1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_i2cdev_initialize
 *
 * Description:
 *   Called to configure I2C
 *
 ****************************************************************************/

void kinetis_i2cdev_initialize(void)
{
  FAR struct i2c_master_s *i2c_dev = NULL;

#if defined(CONFIG_KINETIS_I2C0)
  i2c_dev = kinetis_i2cbus_initialize(0);
#if defined(CONFIG_I2C_DRIVER)
  i2c_register(i2c_dev, 0);
#endif
#endif

#if defined(CONFIG_KINETIS_I2C1)
  i2c_dev  = kinetis_i2cbus_initialize(1);
#if defined(CONFIG_I2C_DRIVER)
  i2c_register(i2c_dev, 1);
#endif
#endif
}

#endif /* CONFIG_KINETIS_I2C0 || CONFIG_KINETIS_I2C1 */
