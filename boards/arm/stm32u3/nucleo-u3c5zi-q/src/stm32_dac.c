/****************************************************************************
 * boards/arm/stm32u3/nucleo-u3c5zi-q/src/stm32_dac.c
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

#include <errno.h>
#include <syslog.h>

#include <nuttx/analog/dac.h>

#include "stm32_dac.h"
#include "nucleo-u3c5zi-q.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dac_register
 *
 * Description:
 *   Initialize one DAC1 channel and register it at the given path.
 *
 ****************************************************************************/

static int stm32_dac_register(int intf, const char *path)
{
  struct dac_dev_s *dac;
  int ret;

  dac = stm32_dacinitialize(intf);
  if (dac == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get DAC interface %d\n", intf);
      return -ENODEV;
    }

  ret = dac_register(path, dac);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: dac_register %s failed: %d\n", path, ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_dac_setup
 *
 * Description:
 *   Register DAC1 channel 1 (PA4) as /dev/dac0 and channel 2 (PA5) as
 *   /dev/dac1.
 *
 ****************************************************************************/

int stm32_dac_setup(void)
{
  int ret = OK;

#ifdef CONFIG_STM32_DAC1CH1
  ret = stm32_dac_register(0, "/dev/dac0");
  if (ret < 0)
    {
      return ret;
    }
#endif

#ifdef CONFIG_STM32_DAC1CH2
  ret = stm32_dac_register(1, "/dev/dac1");
#endif

  return ret;
}
