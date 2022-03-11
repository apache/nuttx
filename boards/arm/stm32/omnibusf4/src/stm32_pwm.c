/****************************************************************************
 * boards/arm/stm32/omnibusf4/src/stm32_pwm.c
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
#include <string.h>
#include <limits.h>

#include <nuttx/timers/pwm.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"
#include "stm32_pwm.h"
#include "omnibusf4.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_pwm_setup
 *
 * Description:
 *
 *   Initialize PWM and register Omnibus F4's TIM2 and TIM3 PWM devices:
 *
 *    TIM3 CH3 PB0 S1_OUT
 *    TIM3 CH4 PB1 S2_OUT
 *    TIM2 CH4 PA3 S3_OUT
 *    TIM2 CH3 PA2 S4_OUT
 *
 ****************************************************************************/

int stm32_pwm_setup(void)
{
  int npwm = 0;                       /* hardware device enumerator */
  const char *ppwm = NULL;            /* pointer to PWM device name */
  struct pwm_lowerhalf_s *pwm = NULL; /* lower-half driver handle */

  /* Initialize TIM2 and TIM3.
   *
   * Ihe underlying STM32 driver "knows" there are up to four channels
   * available for each timer device, so we don't have to do anything
   * special here to export the two channels each that we're
   * interested in. The user will want to avoid the channels that
   * aren't connected to anything, however, or risk death by boredom
   * from resulting non-response.
   */

  for (npwm = 2; npwm <= 3; npwm++)
    {
      pwm = stm32_pwminitialize(npwm);

      /* If we can't get the lower-half handle, skip and keep going. */

      if (!pwm)
        {
        continue;
        }

      /* Translate the peripheral number to a device name. */

      switch (npwm)
      {
        case 2:
          ppwm = "/dev/pwm2";
          break;

        case 3:
          ppwm = "/dev/pwm3";
          break;

        /* Skip missing names. */

        default:
          continue;
      }

      pwm_register(ppwm, pwm);
    }

  return 0;
}
