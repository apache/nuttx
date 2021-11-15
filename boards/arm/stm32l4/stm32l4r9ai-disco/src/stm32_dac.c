/****************************************************************************
 * boards/arm/stm32l4/stm32l4r9ai-disco/src/stm32_dac.c
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

#include <nuttx/board.h>
#include <nuttx/analog/dac.h>

#include "stm32l4_gpio.h"
#include "stm32l4_dac.h"
#include "stm32l4r9ai-disco.h"

#include <arch/board/board.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct dac_dev_s *g_dac;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_dac_setup
 ****************************************************************************/

int stm32l4_dac_setup(void)
{
  static bool initialized = false;

  if (!initialized)
    {
#ifdef CONFIG_STM32L4_DAC1
      int ret;

      g_dac = stm32l4_dacinitialize(0);
      if (g_dac == NULL)
        {
          aerr("ERROR: Failed to get DAC interface\n");
          return -ENODEV;
        }

      /* Register the DAC driver at "/dev/dac0" */

      ret = dac_register("/dev/dac0", g_dac);
      if (ret < 0)
        {
          aerr("ERROR: dac_register failed: %d\n", ret);
          return ret;
        }
#endif

      initialized = true;
    }

  return OK;
}
