/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_ina219.c
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
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ina219.h>
#include <arch/board/board.h>

#include "chip.h"
#include <stm32l4.h>

#if defined(CONFIG_I2C) && defined(CONFIG_STM32L4_I2C1) && \
    defined(CONFIG_SENSORS_INA219)

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#if !defined(CONFIG_INA219_ADDR)
#  define CONFIG_INA219_ADDR 0x40  /* A0 and A1 tied to ground */
#endif

#if !defined(CONFIG_INA219_SHUNTVAL)
#  define CONFIG_INA219_SHUNTVAL 2000
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_ina219initialize
 *
 * Description:
 *   Initialize and register the INA219 driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/ina219"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_ina219initialize(const char *devpath)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing INA219\n");

  /* Configure D4(PA5) and D5(PA6) as input floating */

  stm32l4_configgpio(GPIO_I2C1_D4);
  stm32l4_configgpio(GPIO_I2C1_D5);

  /* Get an instance of the I2C1 interface */

  i2c =  stm32l4_i2cbus_initialize(1);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then initialize and register INA219 */

  int config = INA219_CONFIG_PGA_X1      |
               INA219_CONFIG_RANGE_16V   |
               INA219_CONFIG_SADC_AVG16  |
               INA219_CONFIG_BADC_AVG16  ;

  ret = ina219_register(devpath, i2c, CONFIG_INA219_ADDR,
                  CONFIG_INA219_SHUNTVAL, config);

  if (ret < 0)
    {
      snerr("ERROR: ina219_register failed: %d\n", ret);
      goto error;
    }

  return OK;

error:
  stm32l4_i2cbus_uninitialize(i2c);
  return ret;
}

#endif /* defined(CONFIG_I2C) && defined(CONFIG_STM32_I2C1) &&
        * defined(CONFIG_SENSORS_INA219)
        */
