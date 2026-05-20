/****************************************************************************
 * boards/arm/stm32f0l0g0/common/src/board_pwm.c
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
#include <stdbool.h>
#include <sys/types.h>

#include <nuttx/debug.h>
#include <nuttx/timers/pwm.h>

#include "stm32_pwm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM driver.
 *
 ****************************************************************************/

int stm32_pwm_setup(void)
{
  static bool initialized;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  if (!initialized)
    {
#ifdef CONFIG_STM32F0L0G0_TIM1_PWM
      pwm = stm32_pwminitialize(1);
      if (pwm == NULL)
        {
          aerr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm0", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_STM32F0L0G0_TIM2_PWM
      pwm = stm32_pwminitialize(2);
      if (pwm == NULL)
        {
          aerr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm1", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_STM32F0L0G0_TIM3_PWM
      pwm = stm32_pwminitialize(3);
      if (pwm == NULL)
        {
          aerr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm2", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_STM32F0L0G0_TIM14_PWM
      pwm = stm32_pwminitialize(14);
      if (pwm == NULL)
        {
          aerr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm13", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_STM32F0L0G0_TIM15_PWM
      pwm = stm32_pwminitialize(15);
      if (pwm == NULL)
        {
          aerr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm14", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_STM32F0L0G0_TIM16_PWM
      pwm = stm32_pwminitialize(16);
      if (pwm == NULL)
        {
          aerr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm15", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

#ifdef CONFIG_STM32F0L0G0_TIM17_PWM
      pwm = stm32_pwminitialize(17);
      if (pwm == NULL)
        {
          aerr("ERROR: Failed to get the STM32 PWM lower half\n");
          return -ENODEV;
        }

      ret = pwm_register("/dev/pwm16", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

      initialized = true;
    }

  return OK;
}
