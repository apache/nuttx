/****************************************************************************
 * boards/arm/ra4/arduino-r4-minima/src/ra4m1_pwm.c
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

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/timers/pwm.h>

#include <arch/board/board.h>

#include "ra_pwm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PWM.
 * There are no dedicated PWM output pins available to the user for PWM
 * testing.
 */

#ifndef CONFIG_ARDUINO_R4_MINIMA_PWM_CHANNEL
#  if defined(CONFIG_RA4M1_PWM_CHAN0)
#    warning Assuming PWM channel 0
#    define CONFIG_ARDUINO_R4_MINIMA_PWM_CHANNEL 0
#  elif defined(CONFIG_RA4M1_PWM_CHAN1)
#    warning Assuming PWM channel 1
#    define CONFIG_ARDUINO_R4_MINIMA_PWM_CHANNEL 1
#  elif defined(CONFIG_RA4M1_PWM_CHAN2)
#    warning Assuming PWM channel 2
#    define CONFIG_ARDUINO_R4_MINIMA_PWM_CHANNEL 2
#  elif defined(CONFIG_RA4M1_PWM_CHAN3)
#    warning Assuming PWM channel 3
#    define CONFIG_ARDUINO_R4_MINIMA_PWM_CHANNEL 3
#  endif
#endif

#if defined(CONFIG_PWM) && defined(CONFIG_RA_PWM)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ra4m1_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

int ra4m1_pwm_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call ra_pwminitialize() to get an instance of the PWM interface */

      pwm = ra_pwminitialize(CONFIG_ARDUINO_R4_MINIMA_PWM_CHANNEL);
      if (!pwm)
        {
          _err("ERROR: Failed to get the RA4M1 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm0" */

      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_PWM */
