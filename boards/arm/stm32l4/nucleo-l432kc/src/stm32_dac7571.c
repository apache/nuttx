/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_dac7571.c
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
#include <nuttx/analog/dac.h>
#include <arch/board/board.h>

#include "chip.h"
#include <stm32l4.h>

#if defined(CONFIG_I2C) && defined(CONFIG_STM32L4_I2C1) && \
    defined(CONFIG_DAC7571)

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/

#if !defined(CONFIG_DAC7571_ADDR)
#  define CONFIG_DAC7571_ADDR 0x4C  /* A0 tied to ground */
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct dac_dev_s *g_dac;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dac7571initialize
 *
 * Description:
 *   Initialize and register the DAC7571 DAC driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/dac0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_dac7571initialize(const char *devpath)
{
  struct i2c_master_s *i2c;
  int ret;

  /* Configure D4(PA5) and D5(PA6) as input floating */

  stm32l4_configgpio(GPIO_I2C1_D4);
  stm32l4_configgpio(GPIO_I2C1_D5);

  /* Get an instance of the I2C1 interface */

  i2c =  stm32l4_i2cbus_initialize(1);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then initialize and register DAC7571 */

  g_dac = dac7571_initialize(i2c, CONFIG_DAC7571_ADDR);
  if (!g_dac)
    {
      ret = -ENODEV;
      goto error;
    }

  ret = dac_register(devpath, g_dac);
  if (ret < 0)
    {
      aerr("ERROR: dac_register failed: %d\n", ret);
      goto error;
    }

  return OK;

error:
  stm32l4_i2cbus_uninitialize(i2c);
  return ret;
}

#endif /* defined(CONFIG_I2C) && defined(CONFIG_STM32_I2C1) &&
        * defined(CONFIG_DAC7571)
        */
