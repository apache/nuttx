/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_pwm.c
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
#include "stm32l4_pwm.h"
#include "nucleo-l432kc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PWM
 *
 * The nucleo-l432kc has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using variously unused pins on the
 * board for PWM output (see board.h for details of pins).
 */

#ifdef CONFIG_PWM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

int stm32l4_pwm_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call stm32l4_pwminitialize() to get an instance of the PWM interface
       */

      /* PWM
       *
       * The Nucleo-l432kc has no real on-board PWM devices, but the board
       * can be configured to output a pulse train using TIM1 or 8, or others
       * (see board.h). Let's figure out which the user has configured.
       */

#if defined(CONFIG_STM32L4_TIM1_PWM)
      pwm = stm32l4_pwminitialize(1);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the STM32L4 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm0" */

      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

#if defined(CONFIG_STM32L4_TIM2_PWM)
      pwm = stm32l4_pwminitialize(2);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the STM32L4 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm1" */

      ret = pwm_register("/dev/pwm1", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

#if defined(CONFIG_STM32L4_TIM15_PWM)
      pwm = stm32l4_pwminitialize(15);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the STM32L4 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm2" */

      ret = pwm_register("/dev/pwm2", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

#if defined(CONFIG_STM32L4_TIM16_PWM)
      pwm = stm32l4_pwminitialize(16);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the STM32L4 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm3" */

      ret = pwm_register("/dev/pwm3", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

#if defined(CONFIG_STM32L4_LPTIM1_PWM)
      pwm = stm32l4_lp_pwminitialize(1);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the STM32L4 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm4" */

      ret = pwm_register("/dev/pwm4", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

#if defined(CONFIG_STM32L4_LPTIM2_PWM)
      pwm = stm32l4_lp_pwminitialize(2);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the STM32L4 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm5" */

      ret = pwm_register("/dev/pwm5", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_PWM */
