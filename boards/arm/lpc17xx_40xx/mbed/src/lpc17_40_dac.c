/****************************************************************************
 * boards/arm/lpc17xx_40xx/mbed/src/lpc17_40_dac.c
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

#include "arm_internal.h"
#include "lpc17_40_dac.h"

#ifdef CONFIG_DAC

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: dac_devinit
 *
 * Description:
 *   All LPC17xx/LPC40xx architectures must provide the following interface
 *   to work with examples/diag.
 *
 ****************************************************************************/

int dac_devinit(void)
{
  static bool initialized = false;
  struct dac_dev_s *dac;
  int ret;

  if (!initialized)
    {
      /* Call lpc17_40_dacinitialize() to get an instance of the dac
       * interface
       */

      dac = lpc17_40_dacinitialize();
      if (dac == NULL)
        {
          aerr("ERROR: Failed to get dac interface\n");
          return -ENODEV;
        }

      ret = dac_register("/dev/dac0", dac);
      if (ret < 0)
        {
          aerr("ERROR: dac_register failed: %d\n", ret);
          return ret;
        }

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_DAC */
