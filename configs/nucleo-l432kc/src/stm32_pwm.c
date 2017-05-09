/************************************************************************************
 * configs/nucleo-l432kc/src/stm32l4_pwm.c
 *
 *   Copyright (C) 2011, 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (C) 2016 Sebastien Lorquet. All rights reserved.
 *   Author: Sebastien Lorquet <sebastien@lorquet.fr>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/drivers/pwm.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "stm32l4_pwm.h"
#include "nucleo-l432kc.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration *******************************************************************/
/* PWM
 *
 * The STM3240G-Eval has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using variously unused pins on the board for
 * PWM output (see board.h for details of pins).
 */

#ifdef CONFIG_PWM

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32l4_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ************************************************************************************/

int stm32l4_pwm_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call stm32l4_pwminitialize() to get an instance of the PWM interface */

      /* PWM
       *
       * The Nucleo-l432kc has no real on-board PWM devices, but the board can be
       * configured to output a pulse train using TIM1 or 8, or others (see board.h).
       * Let's figure out which the user has configured.
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

#if defined(CONFIG_STM32L4_TIM3_PWM)
      pwm = stm32l4_pwminitialize(3);
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

#if defined(CONFIG_STM32L4_TIM4_PWM)
      pwm = stm32l4_pwminitialize(4);
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

#if defined(CONFIG_STM32L4_TIM5_PWM)
      pwm = stm32l4_pwminitialize(5);
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

#if defined(CONFIG_STM32L4_TIM8_PWM)
      pwm = stm32l4_pwminitialize(8);
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

#if defined(CONFIG_STM32L4_TIM15_PWM)
      pwm = stm32l4_pwminitialize(15);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the STM32L4 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm6" */

      ret = pwm_register("/dev/pwm6", pwm);
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

      /* Register the PWM driver at "/dev/pwm7" */

      ret = pwm_register("/dev/pwm7", pwm);
      if (ret < 0)
        {
          aerr("ERROR: pwm_register failed: %d\n", ret);
          return ret;
        }
#endif

#if defined(CONFIG_STM32L4_TIM17_PWM)
      pwm = stm32l4_pwminitialize(17);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the STM32L4 PWM lower half\n");
          return -ENODEV;
        }

      /* Register the PWM driver at "/dev/pwm8" */

      ret = pwm_register("/dev/pwm8", pwm);
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

