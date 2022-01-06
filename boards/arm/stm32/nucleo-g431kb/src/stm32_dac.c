/****************************************************************************
 * boards/arm/stm32/nucleo-g431kb/src/stm32_dac.c
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

#include <nuttx/analog/dac.h>
#include <arch/board/board.h>

#include "stm32_dac.h"
#include "nucleo-g431kb.h"

#ifdef CONFIG_DAC

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_STM32_DAC1CH1
static struct dac_dev_s *g_dac1;
#endif

#ifdef CONFIG_STM32_DAC3CH2
static struct dac_dev_s *g_dac5;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dac_setup
 *
 * Description:
 *   Initialize and register the DAC driver.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/dac0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_dac_setup(void)
{
  int ret;
#ifdef CONFIG_STM32_DAC1CH1
  g_dac1 = stm32_dacinitialize(1);
  if (g_dac1 == NULL)
    {
      aerr("ERROR: Failed to get DAC interface\n");
      return -ENODEV;
    }

  /* Register the DAC driver at "/dev/dac0" */

  ret = dac_register("/dev/dac0", g_dac1);
  if (ret < 0)
    {
      aerr("ERROR: dac_register() failed: %d\n", ret);
      return ret;
    }

#endif

#ifdef CONFIG_STM32_DAC3CH2
  g_dac5 = stm32_dacinitialize(5);
  if (g_dac5 == NULL)
    {
      aerr("ERROR: Failed to get DAC interface\n");
      return -ENODEV;
    }

  /* Register the DAC driver at "/dev/dac5" */

  ret = dac_register("/dev/dac5", g_dac5);
  if (ret < 0)
    {
      aerr("ERROR: dac_register() failed: %d\n", ret);
      return ret;
    }

#endif

  UNUSED(ret);
  return OK;
}

#endif  /* CONFIG_DAC */
