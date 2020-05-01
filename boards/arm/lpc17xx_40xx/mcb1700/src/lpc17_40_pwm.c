/****************************************************************************
 * boards/arm/lpc17xx_40xx/mcb1700/lpc17_40_pwm.c
 *
 * Based on boards/lpcexpresso-lpc1768/lpc17_40_pwm.c
 *
 *   Copyright (C) 2014-2017 Gregory Nutt. All rights reserved.
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

#include <arch/board/board.h>

#include "chip.h"
#include "arm_arch.h"
#include "lpc17_40_pwm.h"
#include "lpc17_40_timer.h"
#include "mcb1700.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_PWM

FAR struct pwm_lowerhalf_s *lpc17_40_pwminitialize(int timer);
FAR struct pwm_lowerhalf_s *lpc17_40_mcpwminitialize(int timer);
FAR struct pwm_lowerhalf_s *lpc17_40_timerinitialize(int timer);

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
          aerr("ERROR: Failed to get the LPC17XX_40XX MOTOR PWM lower half\n");
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
