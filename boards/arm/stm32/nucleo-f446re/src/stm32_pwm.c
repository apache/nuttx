/****************************************************************************
 * boards/arm/stm32/nucleo-f446re/src/stm32_pwm.c
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
#include <sys/types.h>

#include <nuttx/board.h>
#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_pwm.h"
#include "nucleo-f446re.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 * Return Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_pwm_setup(void)
{
#ifdef CONFIG_PWM
  struct pwm_lowerhalf_s *pwm;
  int ret;

  /* Call stm32_pwminitialize() to get an instance of the PWM interface */

#if defined(CONFIG_STM32_TIM1_PWM)
  pwm = stm32_pwminitialize(1);
  if (!pwm)
    {
      pwmerr("ERROR: Failed to get the STM32 PWM lower half\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm0", pwm);
  if (ret < 0)
    {
      pwmerr("ERROR: pwm_register failed: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_STM32_TIM2_PWM)
  pwm = stm32_pwminitialize(2);
  if (!pwm)
    {
      pwmerr("ERROR: Failed to get the STM32 PWM lower half\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm1", pwm);
  if (ret < 0)
    {
      pwmerr("ERROR: pwm_register failed: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_STM32_TIM3_PWM)
  pwm = stm32_pwminitialize(3);
  if (!pwm)
    {
      pwmerr("ERROR: Failed to get the STM32 PWM lower half\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm2", pwm);
  if (ret < 0)
    {
      pwmerr("ERROR: pwm_register failed: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_STM32_TIM4_PWM)
  pwm = stm32_pwminitialize(4);
  if (!pwm)
    {
      pwmerr("ERROR: Failed to get the STM32 PWM lower half\n");
      return -ENODEV;
    }

  ret = pwm_register("/dev/pwm3", pwm);
  if (ret < 0)
    {
      pwmerr("ERROR: pwm_register failed: %d\n", ret);
      return ret;
    }

#endif

  return OK;
#else
  return -ENODEV;
#endif
}
