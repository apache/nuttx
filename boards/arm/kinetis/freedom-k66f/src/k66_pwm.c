/****************************************************************************
 * boards/arm/kinetis/freedom-k66f/src/k66_pwm.c
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

#include "chip.h"
#include "arm_internal.h"
#include "kinetis_pwm.h"

#ifdef CONFIG_PWM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k66_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

int k66_pwm_setup(void)
{
  struct pwm_lowerhalf_s *pwm;
  static bool initialized = false;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call k66_pwminitialize() to get an instance of the PWM interface */

      pwm = kinetis_pwminitialize(0);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the K66 PWM lower half\n");
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
