/****************************************************************************
 * boards/arm/stm32/nucleo-f334r8/src/stm32_opamp.c
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
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/analog/opamp.h>

#include "stm32.h"

#if defined(CONFIG_OPAMP) && defined(CONFIG_STM32_OPAMP2)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_opamp_setup
 *
 * Description:
 *   Initialize OPAMP
 *
 ****************************************************************************/

int stm32_opamp_setup(void)
{
  static bool initialized = false;
  struct opamp_dev_s *opamp = NULL;
  int ret;

  if (!initialized)
    {
      /* Get the OPAMP interface */

#ifdef CONFIG_STM32_OPAMP2
      opamp = stm32_opampinitialize(2);
      if (opamp == NULL)
        {
          aerr("ERROR: Failed to get OPAMP%d interface\n", 2);
          return -ENODEV;
        }
#endif

      /* Register the OPAMP character driver at /dev/opamp0 */

      ret = opamp_register("/dev/opamp0", opamp);
      if (ret < 0)
        {
          aerr("ERROR: opamp_register failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_OPAMP && CONFIG_STM32_OPAMP2 */
