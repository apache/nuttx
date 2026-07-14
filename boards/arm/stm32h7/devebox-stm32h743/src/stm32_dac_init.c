/****************************************************************************
 * boards/arm/stm32h7/devebox-stm32h743/src/stm32_dac_init.c
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
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/analog/dac.h>
#include "devebox-stm32h743.h"
#include "stm32_dac.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_dac_initialize
 *
 * Description:
 *   Initialize the DAC and register the DAC devices.
 *   This function should be called from board_app_initialize().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_dac_initialize(void)
{
  int ret = OK;
  struct dac_dev_s *dac;

#ifdef CONFIG_STM32_DAC1CH1
  /* Initialize DAC1 channel 1 (interface 0) and register as /dev/dac0 */

  dac = stm32_dacinitialize(0);
  if (dac == NULL)
    {
      _err("ERROR: Failed to initialize DAC1 channel 1\n");
      return -ENODEV;
    }

  ret = dac_register("/dev/dac0", dac);
  if (ret < 0)
    {
      _err("ERROR: Failed to register DAC1 channel 1: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32_DAC1CH2
  /* Initialize DAC1 channel 2 (interface 1) and register as /dev/dac1 */

  dac = stm32_dacinitialize(1);
  if (dac == NULL)
    {
      _err("ERROR: Failed to initialize DAC1 channel 2\n");
      return -ENODEV;
    }

  ret = dac_register("/dev/dac1", dac);
  if (ret < 0)
    {
      _err("ERROR: Failed to register DAC1 channel 2: %d\n", ret);
      return ret;
    }
#endif

  return ret;
}
