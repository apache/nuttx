/****************************************************************************
 * boards/arm/lpc17xx_40xx/mcb1700/src/lpc17_40_pwm.c
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
#include "lpc17_40_pwm.h"
#include "lpc17_40_timer.h"
#include "mcb1700.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_PWM

struct pwm_lowerhalf_s *lpc17_40_pwminitialize(int timer);
struct pwm_lowerhalf_s *lpc17_40_mcpwminitialize(int timer);
struct pwm_lowerhalf_s *lpc17_40_timerinitialize(int timer);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mcb1700_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

int mcb1700_pwm_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  struct pwm_lowerhalf_s *mcpwm;
  struct pwm_lowerhalf_s *timer;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call lpc17_40_pwminitialize() to get an instance
       * of the PWM interface
       */

      pwm = lpc17_40_pwminitialize(0);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the LPC17XX_40XX PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm0" */

      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }

      mcpwm = lpc17_40_mcpwminitialize(0);
      if (!mcpwm)
        {
          aerr(
             "ERROR: Failed to get the LPC17XX_40XX MOTOR PWM lower half\n");
          return -ENODEV;
        }

      /* Register the MOTOR CONTROL PWM driver at "/dev/mcpwm0" */

      ret = pwm_register("/dev/mcpwm0", mcpwm);
      if (ret < 0)
        {
          aerr("ERROR: mcpwm_register failed: %d\n", ret);
          return ret;
        }

      timer = lpc17_40_timerinitialize(0);
      if (!timer)
        {
          aerr("ERROR: Failed to get the LPC17XX_40XX TIMER lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/timer0" */

      ret = pwm_register("/dev/timer0", timer);
      if (ret < 0)
        {
          aerr("ERROR: timer_register failed: %d\n", ret);
          return ret;
        }

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_PWM */
