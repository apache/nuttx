/****************************************************************************
 * boards/arm/am67/t3-gem-o1/src/am67_pwm.c
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

#include <debug.h>

#include <nuttx/timers/pwm.h>

#include "am67_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void am67_pwmdev_initialize(void)
{
  int ret = am67_epwm_init();

  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize EPWM: %d\n", ret);
    }
  else
    {
      struct pwm_lowerhalf_s *lower;

      syslog(LOG_INFO, "EPWM: CTRL_MMR unlocked\n");

#ifdef CONFIG_AM67_EPWM0
      lower = am67_epwminitialize(0);
      if (lower == NULL)
        {
          syslog(LOG_ERR, "ERROR: Failed to get EPWM0 lower half\n");
        }
      else
        {
          ret = pwm_register("/dev/pwm0", lower);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: pwm_register failed: %d\n", ret);
            }
          else
            {
              syslog(LOG_INFO, "EPWM0: registered /dev/pwm0\n");
            }
        }
#endif

#ifdef CONFIG_AM67_EPWM1
      lower = am67_epwminitialize(1);
      if (lower == NULL)
        {
          syslog(LOG_ERR, "ERROR: Failed to get EPWM1 lower half\n");
        }
      else
        {
          ret = pwm_register("/dev/pwm1", lower);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: pwm_register failed: %d\n", ret);
            }
          else
            {
              syslog(LOG_INFO, "EPWM1: registered /dev/pwm1\n");
            }
        }
#endif
    }
}

