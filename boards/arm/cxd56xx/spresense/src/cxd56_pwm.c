/****************************************************************************
 * boards/arm/cxd56xx/spresense/src/cxd56_pwm.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/timers/pwm.h>

#include "cxd56_pwm.h"

#ifdef CONFIG_PWM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_CXD56_PWM0) || defined(CONFIG_CXD56_PWM1) || \
    defined(CONFIG_CXD56_PWM2) || defined(CONFIG_CXD56_PWM3)
static int pwm_initialize(uint32_t channel)
{
  char devname[16];
  struct pwm_lowerhalf_s *pwm = NULL;
  int ret;

  /* Call cxd56_pwminitialize() to get an instance of the PWM interface */

  pwm = cxd56_pwminitialize(channel);
  if (!pwm)
    {
      pwmerr("Failed to get the CXD56 PWM%d lower half\n", channel);
      return -ENODEV;
    }

  /* Register the PWM driver at "/dev/pwmX" */

  snprintf(devname, sizeof(devname), "/dev/pwm%d", channel);
  ret = pwm_register(devname, pwm);
  if (ret < 0)
    {
      pwmerr("pwm_register(%s) failed: %d\n", devname, ret);
      return ret;
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_pwm_setup
 *
 * Description:
 *   All CXD56 architectures must provide the following interface to work
 *   with examples/pwm.
 *
 ****************************************************************************/

int board_pwm_setup(void)
{
  static bool initialized = false;

  /* Have we already initialized? */

  if (!initialized)
    {
#ifdef CONFIG_CXD56_PWM0
      pwm_initialize(CXD56_PWM_CH0);
#endif

#ifdef CONFIG_CXD56_PWM1
      pwm_initialize(CXD56_PWM_CH1);
#endif

#ifdef CONFIG_CXD56_PWM2
      pwm_initialize(CXD56_PWM_CH2);
#endif

#ifdef CONFIG_CXD56_PWM3
      pwm_initialize(CXD56_PWM_CH3);
#endif

      /* Now we are initialized */

      initialized = true;
    }

  return OK;
}

#endif /* CONFIG_PWM */
