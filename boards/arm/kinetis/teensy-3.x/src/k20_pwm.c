/****************************************************************************
 * boards/arm/kinetis/teensy-3.x/src/k20_pwm.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PWM
 *
 * The Kinetis Freedom board provides a LED on GPIO.
 */

#ifdef CONFIG_PWM

extern struct pwm_lowerhalf_s *kinetis_pwminitialize(int timer);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

int kinetis_pwm_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
#ifdef CONFIG_KINETIS_FTM0_PWM
      /* Call kinetis_pwminitialize() to get an instance of
       * the PWM interface
       */

      pwm = kinetis_pwminitialize(0);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the KL20 PWM lower half\n");
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

#endif

#ifdef CONFIG_KINETIS_FTM1_PWM
      pwm = kinetis_pwminitialize(1);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the KL20 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm1" */

      ret = pwm_register("/dev/pwm1", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

#endif

#ifdef CONFIG_KINETIS_FTM2_PWM
      pwm = kinetis_pwminitialize(2);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the KL20 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm2" */

      ret = pwm_register("/dev/pwm2", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

#endif
      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_PWM */
