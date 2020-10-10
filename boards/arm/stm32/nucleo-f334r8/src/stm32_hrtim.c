/****************************************************************************
 * boards/arm/stm32/nucleo-f334r8/src/stm32_hrtim.c
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

#include "stm32_hrtim.h"

#ifndef CONFIG_STM32_HRTIM_DISABLE_CHARDRV

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_hrtim_setup
 *
 * Description:
 *  Initialize HRTIM driver
 *
 * Returned Value:
 *  0 on success, a negated errno value on failure
 *
 ****************************************************************************/

int stm32_hrtim_setup(void)
{
  static bool initialized = false;
  struct hrtim_dev_s *hrtim = NULL;
  int ret;

  if (!initialized)
    {
      /* Get the HRTIM interface */

      hrtim = stm32_hrtiminitialize();
      if (hrtim == NULL)
        {
          tmrerr("ERROR: Failed to get HRTIM1 interface\n");
          return -ENODEV;
        }

      /* Register the HRTIM character driver at /dev/hrtim0 */

      ret = hrtim_register("/dev/hrtim0", hrtim);
      if (ret < 0)
        {
          tmrerr("ERROR: hrtim_register failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_STM32_HRTIM && CONFIG_STM32_HRTIM1 */
