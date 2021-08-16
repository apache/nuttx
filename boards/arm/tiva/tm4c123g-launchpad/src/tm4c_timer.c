/****************************************************************************
 * boards/arm/tiva/tm4c123g-launchpad/src/tm4c_timer.c
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

#include <debug.h>

#include "tiva_timer.h"
#include "tm4c123g-launchpad.h"

#ifdef CONFIG_TIVA_TIMER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_TIVA_TIMER32_PERIODIC
#  error CONFIG_TIVA_TIMER32_PERIODIC is not defined
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_timer_configure
 *
 * Description:
 *   Configure the timer driver for the timer example application.
 *
 ****************************************************************************/

int tiva_timer_configure(void)
{
  static bool initialized = false;
  int ret = OK;

  /* Check if we have already initialized */

  if (!initialized)
    {
      struct tiva_gptm32config_s timer_config;
      timer_config.cmn.gptm      = 0;
      timer_config.cmn.mode      = TIMER32_MODE_PERIODIC;
      timer_config.cmn.alternate = false;

      timer_config.config.flags   = TIMER_FLAG_COUNTUP;
      timer_config.config.handler = 0;
      timer_config.config.arg     = 0;

      ret = tiva_timer_initialize(CONFIG_EXAMPLES_TIMER_DEVNAME,
                                  &timer_config);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register timer driver: %d\n",
                 ret);
        }

      /* now we are initialized */

      initialized = true;
    }

  return ret;
}
#endif /* CONFIG_TIVA_TIMER */
