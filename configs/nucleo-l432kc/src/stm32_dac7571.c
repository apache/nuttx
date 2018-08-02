/************************************************************************************
 * configs/nucleo-l432kc/src/stm32_dac7571.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/analog/dac.h>
#include <arch/board/board.h>

#include <chip.h>
#include <stm32l4.h>

#if defined(CONFIG_I2C) && defined(CONFIG_STM32L4_I2C1) && defined(CONFIG_DAC7571) 

/************************************************************************************
 * Preprocessor definitions
 ************************************************************************************/

#if !defined(CONFIG_DAC7571_ADDR)
#  define CONFIG_DAC7571_ADDR 0x4C  /* A0 tied to ground */
#endif

/************************************************************************************
 * Private Data
 ************************************************************************************/

static struct dac_dev_s *g_dac;

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
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
 ************************************************************************************/

int stm32_dac7571initialize(FAR const char *devpath)
{
  FAR struct i2c_master_s *i2c;
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
  (void)stm32l4_i2cbus_uninitialize(i2c);
  return ret;
}

#endif /* defined(CONFIG_I2C) && defined(CONFIG_STM32_I2C1) && defined(CONFIG_DAC7571) */
