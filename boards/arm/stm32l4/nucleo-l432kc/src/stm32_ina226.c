/*******************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_ina226.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/*******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ina226.h>
#include <arch/board/board.h>

#include "chip.h"
#include <stm32l4.h>

#if defined(CONFIG_I2C) && defined(CONFIG_STM32L4_I2C1) && \
    defined(CONFIG_SENSORS_INA226)

/*******************************************************************************
 * Preprocessor definitions
 ******************************************************************************/

#if !defined(CONFIG_INA226_ADDR)
#  define CONFIG_INA226_ADDR 0x44  /* A0 tied to ground and A1 tied to VS */
#endif

#if !defined(CONFIG_INA226_SHUNTVAL)
#  define CONFIG_INA226_SHUNTVAL 5000
#endif

/*******************************************************************************
 * Private Data
 ******************************************************************************/

/*******************************************************************************
 * Public Functions
 ******************************************************************************/

/*******************************************************************************
 * Name: stm32_ina226initialize
 *
 * Description:
 *   Initialize and register the INA226 driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/ina226"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ******************************************************************************/

int stm32_ina226initialize(FAR const char *devpath)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing INA226\n");

  /* Configure D4(PA5) and D5(PA6) as input floating */

  stm32l4_configgpio(GPIO_I2C1_D4);
  stm32l4_configgpio(GPIO_I2C1_D5);

  /* Get an instance of the I2C1 interface */

  i2c =  stm32l4_i2cbus_initialize(1);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then initialize and register INA226 */

  int config = INA226_CONFIG_AVG_4 |
               INA226_CONFIG_VBUSCT_140US |
               INA226_CONFIG_VSHCT_140US |
               INA226_CONFIG_MODE_SBCONT;

  ret = ina226_register(devpath, i2c, CONFIG_INA226_ADDR,
                  CONFIG_INA226_SHUNTVAL, config);

  if (ret < 0)
    {
      snerr("ERROR: ina226_register failed: %d\n", ret);
      goto error;
    }

  return OK;

error:
  stm32l4_i2cbus_uninitialize(i2c);
  return ret;
}

#endif /* defined(CONFIG_I2C) && defined(CONFIG_STM32_I2C1) &&
          defined(CONFIG_SENSORS_INA226) */
