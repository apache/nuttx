/************************************************************************************
 * configs/stm32f3discovery/src/stm32_pwm.c
 *
 *   Copyright (C) 2013, 2015-2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include "stm32_pwm.h"
#include "stm32f3discovery.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration *******************************************************************/
/* PWM
 *
 * The stm32f3discovery has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using TIM4 CH2.  This pin is used by FSMC is
 * connected to CN5 just for this purpose:
 *
 * PD13 FSMC_A18 / MC_TIM4_CH2OUT pin 33 (EnB)
 *
 * FSMC must be disabled in this case!
 */

#define HAVE_PWM 1

#ifndef CONFIG_PWM
#  undef HAVE_PWM
#endif

#ifndef CONFIG_STM32_TIM4
#  undef HAVE_PWM
#endif

#ifndef CONFIG_STM32_TIM4_PWM
#  undef HAVE_PWM
#endif

#if CONFIG_STM32_TIM4_CHANNEL != STM32F3DISCOVERY_PWMCHANNEL
#  undef HAVE_PWM
#endif

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ************************************************************************************/

int stm32_pwm_setup(void)
{
#ifdef HAVE_PWM
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Call stm32_pwminitialize() to get an instance of the PWM interface */

      pwm = stm32_pwminitialize(STM32F3DISCOVERY_PWMTIMER);
      if (!pwm)
        {
          aerr("ERROR: Failed to get the STM32 PWM lower half\n");
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
#else
  return -ENODEV;
#endif
}

