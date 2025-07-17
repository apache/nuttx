/****************************************************************************
 * boards/arm/stm32h5/nucleo-h563zi/src/stm32_dts.c
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
#include <debug.h>

#include <arch/board/board.h>

#include "stm32.h"

#if defined(CONFIG_STM32H5_DTS)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dts_setup
 *
 * Description:
 *   Initialize DTS and register the DTS driver.
 *
 ****************************************************************************/

int stm32_dts_setup(int devno)
{
  static bool initialized = false;
  int ret;

  /* Check if we have already initialized */

  if (!initialized)
    {
      /* Register the DTS driver at "/dev/sensor_temp0" */

      ret = stm32h5_dts_register(0);
      if (ret < 0)
        {
          aerr("ERROR: dts_register /dev/dts0 failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

  return OK;
}

#endif
