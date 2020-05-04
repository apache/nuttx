/****************************************************************************
 * boards/arm/kinetis/freedom-k28f/src/k28_pwm.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include "chip.h"
#include "arm_arch.h"
#include "kinetis_pwm.h"

#include <arch/board/board.h>

#ifdef CONFIG_PWM

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k28_pwm_setup
 *
 * Description:
 *   Initialize PWM and register the PWM device.
 *
 ****************************************************************************/

int k28_pwm_setup(void)
{
  static bool initialized = false;
  struct pwm_lowerhalf_s *pwm;
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
#ifdef CONFIG_KINETIS_FTM0_PWM
      /* Call kinetis_pwminitialize()
       * to get an instance of the PWM interface
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
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_PWM */
