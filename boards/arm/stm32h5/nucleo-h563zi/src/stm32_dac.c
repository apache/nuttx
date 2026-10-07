/****************************************************************************
 * boards/arm/stm32h5/nucleo-h563zi/src/stm32_dac.c
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

#include <stdbool.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/analog/dac.h>

#include "stm32.h"
#include "nucleo-h563zi.h"

#if defined(CONFIG_DAC) && defined(CONFIG_STM32_DAC1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dac_setup
 *
 * Description:
 *   Initialize the DAC and register the DAC driver.  DAC1_OUT2 (PA5) is
 *   registered as /dev/dac0.
 *
 ****************************************************************************/

int stm32_dac_setup(void)
{
  static bool initialized = false;
  struct dac_dev_s *dac;
  int ret;

  if (!initialized)
    {
#ifdef CONFIG_STM32_DAC1CH2
      dac = stm32_dacinitialize(1);
      if (dac == NULL)
        {
          aerr("ERROR: Failed to get DAC1 channel 2\n");
          return -ENODEV;
        }

      ret = dac_register("/dev/dac0", dac);
      if (ret < 0)
        {
          aerr("ERROR: dac_register /dev/dac0 failed: %d\n", ret);
          return ret;
        }
#endif

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_DAC && CONFIG_STM32_DAC1 */
